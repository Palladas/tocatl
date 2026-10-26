"""Offline dashboard checks. CAN, subprocesses and ROS are mocked; nothing touches hardware."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, patch
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import operator_dashboard as d

READY_LINKS = [dict(name=b, operstate='UP', state='ERROR-ACTIVE', bitrate=1000000) for b in d.BUSES]


def fresh_state(legs=()):
    return dict(safety_state='DISARMED', status_reason='ok', legs=[dict(leg=l) for l in legs]), 0.05


class FakeProc:
    pid = 4242
    stdout = iter(())

    def __init__(self):
        self.code = None
        self.signals = []
        self.exited = threading.Event()

    def poll(self):
        return self.code

    def wait(self, timeout=None):
        if not self.exited.wait(timeout):
            raise d.subprocess.TimeoutExpired('motor_controller', timeout)
        return self.code

    def send_signal(self, sig):
        self.signals.append(sig)
        self.code = 0
        self.exited.set()


class CommandTests(unittest.TestCase):
    def test_telemetry_profile_never_enables_homing(self):
        argv = d.controller_command('telemetry')
        self.assertIn(str(d.CONFIG / 'hardware.yaml'), argv)
        self.assertFalse(any('allow_homing' in a or 'allow_hardware_motion' in a for a in argv))

    def test_homing_profile_flags_legs_and_output(self):
        argv = d.controller_command('homing', [3, 1, 3], True, '/tmp/x/out.json')
        self.assertIn(str(d.CONFIG / 'homing-hip-knee.yaml'), argv)
        for expected in ('allow_homing:=true', 'homing.legs:=[1,3]', 'homing.verify_repeatability:=true',
                         'homing.output_path:=/tmp/x/out.json'):
            self.assertIn(expected, argv)
        self.assertFalse(any('allow_hardware_motion' in a for a in argv))

    def test_bad_profile_and_legs_rejected(self):
        for profile, legs in (('motion', [1]), ('homing', []), ('homing', [0]), ('homing', [7]), ('homing', [True])):
            with self.assertRaises(ValueError):
                d.controller_command(profile, legs, False, 'out.json')
        with self.assertRaises(ValueError):
            d.controller_command('homing', [1], 'yes', 'out.json')

    def test_profiles_exist_and_keep_flags_off(self):
        for profile in d.PROFILES.values():
            text = profile['params'].read_text()
            self.assertIn('allow_hardware_motion: false', text)
            if 'allow_homing' in text:
                self.assertIn('allow_homing: false', text)
            self.assertNotIn('51', text.split('telemetry_motor_ids:')[1].split('\n')[0])

    def test_clean_replaces_nonfinite(self):
        self.assertEqual(d.clean({'a': [float('nan'), 1.0, float('inf')]}), {'a': [None, 1.0, None]})


class CanTests(unittest.TestCase):
    sample = {'ifname': 'can1', 'operstate': 'UP', 'txqlen': 10,
              'linkinfo': {'info_data': {'state': 'ERROR-ACTIVE', 'restart_ms': 0, 'bittiming': {'bitrate': 1000000}},
                           'info_xstats': {'bus_error': 0, 'restarts': 0}},
              'stats64': {'tx': {'packets': 5, 'errors': 0, 'dropped': 1}, 'rx': {'packets': 9, 'errors': 0, 'dropped': 0}}}

    def test_summary_and_problems(self):
        link = d.summarize_link(self.sample)
        self.assertEqual((link['bitrate'], link['txqlen'], link['tx_dropped']), (1000000, 10, 1))
        self.assertEqual(d.can_problems([link], ['can1']), [])
        self.assertEqual(d.can_problems([link]), ['can0 missing'])
        self.assertEqual(d.can_problems([dict(link, state='BUS-OFF')], ['can1']), ['can1 UP BUS-OFF'])
        self.assertEqual(d.can_problems([dict(link, bitrate=500000)], ['can1']), ['can1 bitrate 500000'])

    def test_robot_process_filter(self):
        listing = ('  10 /usr/bin/python3 /x/motor_controller --ros-args\n'
                   f'  {d.os.getpid()} python operator_dashboard.py\n'
                   '  11 bash\n  12 python -m tocatl_diagnostics.hold_capture\n')
        with patch.object(d.subprocess, 'run', return_value=MagicMock(stdout=listing)):
            found = d.robot_processes()
        self.assertEqual(len(found), 2)
        self.assertTrue(found[0].startswith('10 '))

    def test_reset_refused_while_running_and_for_bad_buses(self):
        dash = d.Dashboard(MagicMock())
        for buses in (None, [], ['can2'], ['can0', 'can0']):
            with self.assertRaises(ValueError):
                dash.reset_can({'buses': buses})
        dash.proc = FakeProc()
        with patch.object(d.subprocess, 'run') as run, self.assertRaises(ValueError):
            dash.reset_can({'buses': ['can1']})
        run.assert_not_called()

    def test_reset_sequence(self):
        dash = d.Dashboard(MagicMock())
        ok = MagicMock(returncode=0)
        with patch.object(d, 'robot_processes', return_value=[]), \
                patch.object(d.subprocess, 'run', return_value=ok) as run, \
                patch.object(d, 'can_links', return_value=[]), \
                patch.object(d, 'summarize_link'), \
                patch.object(d, 'can_problems', return_value=[]):
            dash.reset_can({'buses': ['can1']})
        self.assertEqual([c.args[0] for c in run.call_args_list], [
            ['sudo', '-n', 'ip', 'link', 'set', 'can1', 'down'],
            ['sudo', '-n', 'ip', 'link', 'set', 'can1', 'type', 'can', 'bitrate', '1000000'],
            ['sudo', '-n', 'ip', 'link', 'set', 'can1', 'up']])
        self.assertEqual(dash.busy, '')


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.bridge = MagicMock()
        self.bridge.snapshot.return_value = fresh_state()
        self.bridge.call.return_value = (True, 'Homing requested')
        self.dash = d.Dashboard(self.bridge, runs_root=self.tmp.name)
        self.exe = patch.object(d, 'EXECUTABLE', MagicMock(is_file=lambda: True, __str__=lambda _: '/x/motor_controller'))
        self.exe.start()
        self.addCleanup(self.exe.stop)

    def start(self, **data):
        request = dict(profile='homing', legs=[1, 2], checklist_confirmed=True)
        request.update(data)
        with patch.object(d, 'robot_processes', return_value=[]), \
                patch.object(d, 'can_links', return_value=[]), \
                patch.object(d, 'summarize_link'), \
                patch.object(d, 'can_problems', return_value=[]), \
                patch.object(d.subprocess, 'Popen', return_value=FakeProc()) as popen:
            self.dash.start_controller(request)
        return popen

    def test_preview_cannot_start_or_reset(self):
        preview = d.Dashboard(None)
        self.assertTrue(preview.preview_only)
        with self.assertRaises(ValueError):
            preview.start_controller(dict(profile='telemetry', checklist_confirmed=True))
        with self.assertRaises(ValueError):
            preview.reset_can({'buses': ['can0']})
        self.assertIsNone(preview.state()['robot'])

    def test_start_requires_checklist(self):
        with patch.object(d.subprocess, 'Popen') as popen, self.assertRaises(ValueError):
            self.dash.start_controller(dict(profile='telemetry'))
        popen.assert_not_called()

    def test_start_refused_when_bus_owned_or_links_down(self):
        for others, problems in ((['1 motor_controller'], []), ([], ['can1 DOWN'])):
            with patch.object(d, 'robot_processes', return_value=others), \
                    patch.object(d, 'can_links', return_value=[]), patch.object(d, 'summarize_link'), \
                    patch.object(d, 'can_problems', return_value=problems), \
                    patch.object(d.subprocess, 'Popen') as popen, self.assertRaises(ValueError):
                self.dash.start_controller(dict(profile='telemetry', checklist_confirmed=True))
            popen.assert_not_called()
            self.assertEqual(self.dash.busy, '')

    def test_start_homing_writes_session_folder_not_config(self):
        argv = self.start().call_args.args[0]
        output = Path(self.dash.output_path)
        self.assertEqual(output.parent.parent, Path(self.tmp.name))
        self.assertIn(f'homing.output_path:={output}', argv)
        self.assertNotIn(str(ROOT / 'config'), ' '.join(argv))
        self.assertEqual(self.dash.legs, [1, 2])
        self.assertTrue((output.parent / 'controller.log').is_file())
        with self.assertRaises(ValueError):
            self.start()

    def test_home_gates(self):
        with self.assertRaises(ValueError):
            self.dash.home(dict(leg=1, homing_confirmed=True))  # no controller
        self.start(legs=[1, 2])
        with self.assertRaises(ValueError):
            self.dash.home(dict(leg=1))  # checklist
        with self.assertRaises(ValueError):
            self.dash.home(dict(leg=3, homing_confirmed=True))  # not configured
        with self.assertRaises(ValueError):
            self.dash.home(dict(leg=2, homing_confirmed=True))  # unvalidated reference not acknowledged
        with self.assertRaises(ValueError):
            self.dash.home(dict(leg=None, homing_confirmed=True))  # batch includes leg 2
        self.bridge.snapshot.return_value = (fresh_state()[0], 5.0)
        with self.assertRaises(ValueError):
            self.dash.home(dict(leg=1, homing_confirmed=True))  # stale robot_state
        self.bridge.call.assert_not_called()
        self.bridge.snapshot.return_value = fresh_state()
        self.dash.home(dict(leg=1, homing_confirmed=True))
        self.dash.home(dict(leg=None, homing_confirmed=True, unvalidated_confirmed=True))
        self.assertEqual([c.args[0] for c in self.bridge.call.call_args_list], ['motors/leg_1/home', 'motors/home'])

    def test_home_refused_on_telemetry_profile(self):
        self.start(profile='telemetry')
        with self.assertRaises(ValueError):
            self.dash.home(dict(leg=1, homing_confirmed=True))

    def test_controller_rejection_is_reported(self):
        self.start(legs=[1])
        self.bridge.call.return_value = (False, 'Homing already running')
        with self.assertRaisesRegex(ValueError, 'already running'):
            self.dash.home(dict(leg=1, homing_confirmed=True))

    def test_shutdown_disarms_then_interrupts(self):
        self.start(legs=[1])
        proc = self.dash.proc
        self.dash.shutdown(proc)
        self.assertEqual(self.bridge.call.call_args.args[:2], ('motors/enable', 'setbool'))
        self.assertEqual(self.bridge.call.call_args.kwargs, {'data': False})
        self.assertEqual(proc.signals, [d.signal.SIGINT])

    def test_state_is_strict_json(self):
        self.start(legs=[1])
        self.bridge.snapshot.return_value = (dict(position=None), 0.1)
        json.dumps(self.dash.state(), allow_nan=False)


class ResultTests(unittest.TestCase):
    def test_committed_result_surfaces_reference_mismatch(self):
        rows = d.homing_results(ROOT / 'config' / 'hip-knee-homing.json')
        hip = next(r for r in rows if r['motor_id'] == 12)
        self.assertFalse(hip['within_window'])
        self.assertAlmostEqual(hip['home_rad'], -0.8429, places=3)
        self.assertEqual(hip['contacts'], 1)

    def test_missing_or_partial_file(self):
        self.assertEqual(d.homing_results('/nonexistent/x.json'), [])
        with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
            f.write('{')
            f.flush()
            self.assertEqual(d.homing_results(f.name), [])


class HttpTests(unittest.TestCase):
    def test_token_and_origin_required(self):
        import socket
        with socket.socket() as s:
            s.bind(('127.0.0.1', 0))
            port = s.getsockname()[1]
        dash = d.Dashboard(None)
        threading.Thread(target=d.serve, args=(dash, port), daemon=True).start()
        base = f'http://127.0.0.1:{port}'
        for _ in range(50):
            try:
                page = urllib.request.urlopen(base + '/').read().decode()
                break
            except OSError:
                time.sleep(0.05)
        self.assertNotIn('__TOKEN__', page)
        token = page.split("const token='")[1].split("'")[0]
        state = json.loads(urllib.request.urlopen(base + '/state').read())
        self.assertTrue(state['preview_only'])

        def post(headers):
            request = urllib.request.Request(base + '/plan', data=b'{"profile":"telemetry"}', method='POST',
                                             headers={'Content-Type': 'application/json', **headers})
            try:
                return urllib.request.urlopen(request).status
            except urllib.error.HTTPError as error:
                return error.code
        self.assertEqual(post({'Origin': base}), 403)
        self.assertEqual(post({'X-Control-Token': token}), 403)
        self.assertEqual(post({'Origin': 'http://evil', 'X-Control-Token': token}), 403)
        self.assertEqual(post({'Origin': base, 'X-Control-Token': token}), 200)


if __name__ == '__main__':
    unittest.main()
