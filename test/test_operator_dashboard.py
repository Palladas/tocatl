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
        for profile, legs in (('walking', [1]), ('homing', []), ('homing', [0]), ('homing', [7]), ('homing', [True])):
            with self.assertRaises(ValueError):
                d.controller_command(profile, legs, False, 'out.json')
        with self.assertRaises(ValueError):
            d.controller_command('homing', [1], 'yes', 'out.json')

    def test_profiles_exist_and_keep_flags_off(self):
        for profile in d.PROFILES.values():
            for params in profile['params']:
                text = '\n'.join(line for line in params.read_text().splitlines()
                                  if not line.lstrip().startswith('#'))
                self.assertIn('allow_hardware_motion: false', text)
                if 'allow_homing' in text:
                    self.assertIn('allow_homing: false', text)
                if 'telemetry_motor_ids:' in text:
                    self.assertNotIn('51', text.split('telemetry_motor_ids:')[1].split('\n')[0])

    def test_motion_profile_command_scopes_motors_to_selected_legs(self):
        argv = d.controller_command('motion', [2, 1], False, '/tmp/x/out.json')
        files = [argv[i + 1] for i, a in enumerate(argv) if a == '--params-file']
        self.assertEqual(files, [str(d.CONFIG / 'homing-hip-knee.yaml'), str(d.CONFIG / 'motion-feet-air.yaml')])
        for expected in ('allow_homing:=true', 'allow_hardware_motion:=true', 'homing.legs:=[1,2]',
                         'actuation_motor_ids:=[12,13,22,23]', 'telemetry_motor_ids:=[12,13,22,23]'):
            self.assertIn(expected, argv)
        self.assertFalse(any('allow_hardware_motion' in a for a in d.controller_command('homing', [1], False, 'o')))

    def test_player_command_validates_inputs(self):
        argv = d.player_command([2, 1], 'pose', {'r_m': 0.35, 'z_m': -0.05, 'loops': 1}, dry_run=True,
                                actuated_legs=[1, 2, 3])
        self.assertEqual(argv[1:9], ['-m', 'tocatl_kinematics.player', '--legs', '1,2', '--actuated-legs', '1,2,3',
                                     '--kind', 'pose'])
        self.assertIn('--r-m', argv)
        self.assertEqual(argv[-1], '--dry-run')
        for legs, kind, values in (([1], 'walk', {}), ([], 'pose', {}), ([7], 'pose', {}), ([1, 1], 'pose', {}),
                                   ([1], 'pose', {'kp': 100.0}), ([1], 'pose', {'r_m': float('nan')}),
                                   ([1], 'circle', {'loops': 1.5}), ([1], 'pose', {'r_m': True})):
            with self.assertRaises(ValueError):
                d.player_command(legs, kind, values, dry_run=False, actuated_legs=[1, 2, 3, 4, 5, 6])
        with self.assertRaises(ValueError):
            d.player_command([4], 'nudge', {}, dry_run=True, actuated_legs=[1, 2])

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


class ControllerBase(unittest.TestCase):
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


class ControllerTests(ControllerBase):
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

    def test_level3_needs_motion_confirmation_and_allows_homing(self):
        with patch.object(d.subprocess, 'Popen') as popen, self.assertRaises(ValueError):
            self.dash.start_controller(dict(profile='motion', legs=[1], checklist_confirmed=True))
        popen.assert_not_called()
        self.start(profile='motion', legs=[1], motion_confirmed=True)
        self.dash.home(dict(leg=1, homing_confirmed=True))
        self.assertEqual(self.bridge.call.call_args.args[0], 'motors/leg_1/home')

    def test_motion_and_arm_gating(self):
        self.start(profile='homing', legs=[1])
        with self.assertRaises(ValueError):  # not a Level 3 session
            self.dash.motion(dict(legs=[1], kind='nudge'), dry_run=True)
        self.dash.proc.send_signal(d.signal.SIGINT)
        self.start(profile='motion', legs=[1], motion_confirmed=True)
        with self.assertRaises(ValueError):  # leg outside the session
            self.dash.motion(dict(legs=[2], kind='nudge'), dry_run=True)
        with self.assertRaises(ValueError):  # run needs the motion checklist
            self.dash.motion(dict(legs=[1], kind='nudge'), dry_run=False)
        with self.assertRaises(ValueError):  # arming needs a streaming player
            self.dash.arm(dict(arm_confirmed=True))
        player = FakeProc()
        with patch.object(d.subprocess, 'Popen', return_value=player) as popen:
            self.dash.motion(dict(legs=[1], kind='nudge', values={'max_speed_rad_s': 0.05},
                                  motion_confirmed=True), dry_run=False)
        self.assertNotIn('--dry-run', popen.call_args.args[0])
        self.assertIn('--actuated-legs', popen.call_args.args[0])
        with self.assertRaises(ValueError):  # one motion at a time
            self.dash.motion(dict(legs=[1], kind='nudge'), dry_run=True)
        with self.assertRaises(ValueError):
            self.dash.arm(dict())
        self.bridge.call.reset_mock()
        self.dash.arm(dict(arm_confirmed=True))
        self.assertEqual(self.bridge.call.call_args.args[:2], ('motors/enable', 'setbool'))
        self.assertEqual(self.bridge.call.call_args.kwargs, {'data': True})
        self.dash.stop_player()
        self.assertEqual(self.bridge.call.call_args.kwargs, {'data': False})
        self.assertEqual(player.signals, [d.signal.SIGINT])

    def test_state_is_strict_json(self):
        self.start(legs=[1])
        self.bridge.snapshot.return_value = (dict(position=None), 0.1)
        json.dumps(self.dash.state(), allow_nan=False)


class RecordTests(ControllerBase):
    motors = [dict(motor_id=12, valid=True, fresh=True, position=0.5, velocity=0.0, torque=0.1, enabled=False),
              dict(motor_id=13, valid=True, fresh=False, position=0.7, velocity=0.0, torque=0.0, enabled=False),
              dict(motor_id=51, valid=False, fresh=False, position=None, velocity=None, torque=None, enabled=False)]

    def wait_done(self):
        for _ in range(100):
            if not self.dash.recording['active']:
                return
            time.sleep(0.02)
        self.fail('recording did not finish')

    def test_record_requires_controller_and_valid_input(self):
        with self.assertRaises(ValueError):
            self.dash.record(dict(label='leg1-hip-only', duration_s=1))  # no controller
        self.start(profile='telemetry')
        for label, duration in (('', 1), ('../x', 1), ('a b', 1), ('x' * 61, 1), ('ok', 0), ('ok', 121), ('ok', 1.5)):
            with self.assertRaises(ValueError):
                self.dash.record(dict(label=label, duration_s=duration))

    def test_record_writes_only_fresh_motors_and_skips_stale_state(self):
        self.start(profile='telemetry')
        states = iter([(dict(motors=self.motors), 0.05), (None, None), (dict(motors=self.motors), 5.0)])
        self.bridge.snapshot.side_effect = lambda: next(states, (dict(motors=self.motors), 0.05))
        with patch.object(d, 'RECORD_PERIOD_S', 0.01):
            self.dash.record(dict(label='leg1-hip-only', duration_s=1))
            with self.assertRaises(ValueError):
                self.dash.record(dict(label='again', duration_s=1))
            time.sleep(0.15)
            self.dash.stop_recording()
            self.wait_done()
        lines = [json.loads(l) for l in Path(self.dash.recording['path']).read_text().splitlines()]
        self.assertGreater(len(lines), 2)
        self.assertEqual(self.dash.recording['skipped'], 2)
        self.assertEqual(lines[0]['motors'], {'12': dict(position=0.5, velocity=0.0, torque=0.1, enabled=False)})
        self.assertEqual(lines[0]['label'], 'leg1-hip-only')
        self.assertTrue(Path(self.dash.recording['path']).parent.parent == Path(self.dash.run_dir))
        self.assertEqual(self.dash.state()['recordings'], [Path(self.dash.recording['path']).name])
        self.bridge.call.assert_not_called()


class ResultTests(unittest.TestCase):
    def test_committed_result_reports_rotor_turns_and_residual(self):
        rows = {r['motor_id']: r for r in d.homing_results(ROOT / 'config' / 'hip-knee-homing.json')}
        self.assertEqual(len(rows), 12)  # 2026-10-26 15:45 all-leg run
        hip = rows[12]
        self.assertAlmostEqual(hip['home_rad'], -0.0269, places=3)
        self.assertEqual(hip['turns'], -1)  # derived: the file predates turn-aware homing
        self.assertAlmostEqual(hip['residual_rad'], -0.0190, places=3)
        self.assertEqual(hip['contacts'], 1)
        self.assertAlmostEqual(rows[32]['residual_rad'], -0.2274, places=3)  # leg 3 hip outlier

    def test_turn_fields_from_new_results_are_used(self):
        with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
            json.dump({'legs': {'1': {'mode': 'commissioning', 'motors': [dict(
                motor_id=12, home_position_rad=-0.0223, previous_reference_rad=0.8028, reference_delta_rad=-0.8251,
                reference_turns=-1, reference_residual_rad=-0.0144, reference_within_window=True,
                contact_positions_rad=[-0.0223] * 3)]}}}, f)
            f.flush()
            row = d.homing_results(f.name)[0]
        self.assertEqual((row['turns'], row['residual_rad'], row['within_window']), (-1, -0.0144, True))

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
