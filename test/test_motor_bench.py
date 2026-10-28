"""Motor bench: CyberGear framing and the bench's safety logic against a simulated motor."""
import importlib.util
import math
from pathlib import Path
import struct
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest

spec = importlib.util.spec_from_file_location('motor_bench', Path(__file__).resolve().parents[1] / 'scripts' / 'motor_bench.py')
mb = importlib.util.module_from_spec(spec)
sys.modules['motor_bench'] = mb
spec.loader.exec_module(mb)


class FakeMotor:
    """Answers like a CyberGear: state to type 1/3/4, params to 17/18, UID to type 0."""

    def __init__(self, motor_id=127, uid='0011223344556677', compliance=0.0):
        self.motor_id, self.uid, self.compliance = motor_id, bytes.fromhex(uid), compliance
        self.enabled, self.position, self.velocity, self.torque = False, 0.3, 0.0, 0.0
        self.params = {mb.LIMIT_TORQUE: 12.0, mb.IQF: 0.0, mb.VBUS: 24.1, mb.RUN_MODE: 0}
        self.limit_writes = []

    def state(self):
        mid = (2 << 24) | ((2 if self.enabled else 0) << 22) | (self.motor_id << 8) | mb.HOST_ID
        data = struct.pack('>HHHH', mb.to_u16(self.position, *mb.P_RANGE), mb.to_u16(self.velocity, *mb.V_RANGE),
                           mb.to_u16(self.torque, *mb.T_RANGE), 300)
        return mid, data

    def handle(self, arbitration_id, data):
        kind, target = arbitration_id >> 24, arbitration_id & 0xff
        if kind == 0:
            return [((0 << 24) | (self.motor_id << 8) | 0xfe, self.uid)] if target == self.motor_id else []
        if target != self.motor_id:
            return []
        if kind == 1:
            command = mb.from_u16((arbitration_id >> 8) & 0xffff, *mb.T_RANGE)
            if self.enabled:
                limit = self.params[mb.LIMIT_TORQUE]
                self.torque = max(-limit, min(limit, command))
                self.params[mb.IQF] = self.torque / 0.87
                self.position += self.compliance * self.torque
        elif kind == 3:
            self.enabled = True
        elif kind == 4:
            self.enabled, self.torque = False, 0.0
        elif kind == 17:
            index = struct.unpack_from('<H', data)[0]
            value = self.params[index]
            payload = bytes([value, 0, 0, 0]) if index == mb.RUN_MODE else struct.pack('<f', value)
            return [((17 << 24) | (self.motor_id << 8) | mb.HOST_ID, data[:4] + payload)]
        elif kind == 18:
            index = struct.unpack_from('<H', data)[0]
            value = data[4] if index == mb.RUN_MODE else struct.unpack_from('<f', data, 4)[0]
            self.params[index] = value
            if index == mb.LIMIT_TORQUE:
                self.limit_writes.append(round(value, 3))
            return []
        else:
            return []
        return [self.state()]


def fake_can(motors):
    sent, listeners = [], []

    class Bus:
        def __init__(self, **kw):
            self.lock = threading.Lock()

        def send(self, msg, timeout=None):
            sent.append((msg.arbitration_id, bytes(msg.data)))
            with self.lock:
                replies = [r for m in motors for r in m.handle(msg.arbitration_id, bytes(msg.data))]
            for arbitration_id, data in replies:
                for listener in listeners:
                    listener(NS(arbitration_id=arbitration_id, data=data, is_extended_id=True,
                                is_error_frame=False, is_remote_frame=False))

        def shutdown(self):
            pass

    class Notifier:
        def __init__(self, bus, handlers):
            listeners.extend(handlers)

        def stop(self):
            pass

    module = NS(interface=NS(Bus=Bus), Notifier=Notifier, Message=lambda **kw: NS(**kw))
    return module, sent


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class BenchTest(unittest.TestCase):
    def setUp(self):
        self.patches = (mb.robot_processes, mb.link_state, mb.RUNS)
        mb.robot_processes = lambda: []
        mb.link_state = lambda name, max_age=1.0: dict(name=name, operstate='UP', state='ERROR-ACTIVE', bitrate=1_000_000)
        import tempfile
        self.tmp = tempfile.TemporaryDirectory()
        mb.RUNS = Path(self.tmp.name)
        self.benches = []

    def tearDown(self):
        for bench in self.benches:
            bench.close()
        mb.robot_processes, mb.link_state, mb.RUNS = self.patches
        self.tmp.cleanup()

    def bench(self, *motors):
        module, sent = fake_can(list(motors))
        bench = mb.Bench(can_module=module)
        self.benches.append(bench)
        bench.open(dict(bus='can1'))
        return bench, sent

    def keepalive(self, bench, until):
        while not until():
            bench.state()  # the page polls /state; without it the heartbeat stops the motor
            time.sleep(0.05)

    def test_frames_round_trip(self):
        arbitration_id, data = mb.control_frame(127, 1.0, -2.0, 5.5, 30.0, 1.5)
        self.assertEqual(arbitration_id >> 24, 1)
        self.assertEqual(arbitration_id & 0xff, 127)
        self.assertAlmostEqual(mb.from_u16((arbitration_id >> 8) & 0xffff, *mb.T_RANGE), 5.5, places=3)
        p, v, kp, kd = struct.unpack('>HHHH', data)
        self.assertAlmostEqual(mb.from_u16(p, *mb.P_RANGE), 1.0, places=3)
        self.assertAlmostEqual(mb.from_u16(kd, *mb.KD_RANGE), 1.5, places=3)
        motor = FakeMotor()
        motor.torque, motor.enabled = -3.0, True
        state = mb.parse(*motor.state())
        self.assertEqual((state['kind'], state['motor_id'], state['mode']), ('state', 127, 2))
        self.assertAlmostEqual(state['torque'], -3.0, places=3)
        self.assertEqual(state['temperature'], 30.0)
        self.assertEqual(mb.parse((21 << 24) | (127 << 8), struct.pack('<II', 0b100, 0))['faults'], ['Under voltage'])

    def test_no_zero_or_id_writes_exist(self):
        source = (Path(mb.__file__)).read_text()
        self.assertNotIn('frame(6', source)
        self.assertNotIn('frame(7', source)

    def test_scan_lists_ids_and_flags_robot_ids_and_conflicts(self):
        bench, _ = self.bench(FakeMotor(127), FakeMotor(12, '0101010101010101'),
                              FakeMotor(40, 'aa'*8), FakeMotor(40, 'bb'*8))
        found = {r['motor_id']: r for r in bench.scan()}
        self.assertEqual(set(found), {12, 40, 127})
        self.assertFalse(found[127]['robot_id'])
        self.assertTrue(found[12]['robot_id'])
        self.assertTrue(found[40]['conflict'])
        with self.assertRaisesRegex(ValueError, 'robot motor ID'):
            bench.select(dict(motor_id=12))
        bench.select(dict(motor_id=12, robot_ack=True))
        with self.assertRaisesRegex(ValueError, 'ID conflict'):
            bench.select(dict(motor_id=40, robot_ack=True))

    def test_open_refuses_while_robot_process_runs(self):
        mb.robot_processes = lambda: ['123 motor_controller']
        module, _ = fake_can([])
        with self.assertRaisesRegex(ValueError, 'Robot CAN process'):
            mb.Bench(can_module=module).open(dict(bus='can0'))

    def test_torque_sweep_records_steps_and_restores_limit(self):
        motor = FakeMotor()
        bench, sent = self.bench(motor)
        bench.select(dict(motor_id=127))
        self.assertTrue(wait_for(lambda: bench.feedback is not None))
        bench.torque_test(dict(steps=[1, 2.5], direction=-1, lever_m=0.1, hold_s=0.5, rest_s=0,
                               ramp_nm_s=20, kd=0.5))
        self.assertEqual(motor.limit_writes[0], 2.75)
        self.keepalive(bench, lambda: not bench.enabled)
        self.assertEqual(bench.test['stopped'], 'Torque test complete')
        self.assertEqual([r['commanded_nm'] for r in bench.results], [1.0, 2.5])
        self.assertAlmostEqual(bench.results[1]['reported_nm'], 2.5, places=2)
        self.assertAlmostEqual(bench.results[1]['iq_a'], 2.5 / 0.87, places=1)
        commanded = [mb.from_u16((a >> 8) & 0xffff, *mb.T_RANGE) for a, _ in sent if a >> 24 == 1]
        self.assertLess(min(commanded), -2.49)  # direction -1 pressed with negative torque
        self.assertTrue(all(c <= 0.001 for c in commanded))
        self.assertTrue(wait_for(lambda: motor.limit_writes[-1] == 12.0))  # original limit restored
        self.assertFalse(motor.enabled)
        bench.scale(dict(step=1, scale_kg=2.6, zero_kg=0.05))
        self.assertAlmostEqual(bench.results[1]['measured_nm'], 2.55 * mb.G * 0.1)
        self.assertTrue((bench.run_dir / 'results.json').exists())

    def test_slip_off_the_scale_stops_the_motor(self):
        motor = FakeMotor(compliance=0.02)  # output turns when torque is applied
        bench, _ = self.bench(motor)
        bench.select(dict(motor_id=127))
        self.assertTrue(wait_for(lambda: bench.feedback is not None))
        bench.torque_test(dict(steps=[5], lever_m=0.1, hold_s=5, travel_guard_deg=5, ramp_nm_s=20))
        self.keepalive(bench, lambda: not bench.enabled)
        self.assertIn('end effector off the scale', bench.test['stopped'])
        self.assertFalse(motor.enabled)
        self.assertEqual(bench.results, [])

    def test_page_heartbeat_loss_stops_the_motor(self):
        motor = FakeMotor()
        bench, _ = self.bench(motor)
        bench.select(dict(motor_id=127))
        self.assertTrue(wait_for(lambda: bench.feedback is not None))
        bench.rotate(dict(delta_deg=10, speed=1, kp=10, kd=1, torque_limit=2))
        self.assertTrue(motor.enabled)
        self.assertEqual(motor.limit_writes[0], 2.0)
        self.assertTrue(wait_for(lambda: not bench.enabled, timeout=mb.HEARTBEAT_S + 2))
        self.assertIn('stopped polling', bench.error)
        self.assertFalse(motor.enabled)

    def test_rotation_ramps_to_target(self):
        motor = FakeMotor()
        bench, sent = self.bench(motor)
        bench.select(dict(motor_id=127))
        self.assertTrue(wait_for(lambda: bench.feedback is not None))
        bench.set_reference()
        bench.rotate(dict(target_deg=20, speed=2, kp=10, kd=1, torque_limit=2))
        self.keepalive(bench, lambda: bench.mode == 'hold')
        positions = [mb.from_u16(struct.unpack('>H', d[:2])[0], *mb.P_RANGE) for a, d in sent if a >> 24 == 1]
        self.assertAlmostEqual(positions[-1], 0.3 + math.radians(20), places=3)
        steps = [b - a for a, b in zip(positions[1:], positions[2:])]
        self.assertLessEqual(max(steps), 2 / mb.CONTROL_HZ + 1e-3)  # speed-limited ramp
        bench.stop()
        self.assertFalse(motor.enabled)


if __name__ == '__main__':
    unittest.main()
