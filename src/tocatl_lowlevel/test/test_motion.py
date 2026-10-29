"""Level 3 motion safety: home-relative envelopes and the verified drive torque cap."""
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

from tocatl_lowlevel.hardware import Sample
from tocatl_lowlevel.motor_commands import HardwareMotion, MotorCommands

HOMES = {12: -0.0269, 13: 1.5795}
OFFSETS = {'hip_pitch': (-0.02, 1.6), 'knee': (-1.7, 2.0)}


def batch(targets, stamp):
    return NS(header=NS(stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9), frame_id=''),
              motors=[NS(motor_id=mid, position_rad=pos, velocity_rad_s=0., torque_nm=0., kp=30., kd=2.)
                      for mid, pos in targets.items()])


def commands(homes=HOMES, **kw):
    return MotorCommands(motor_ids=(12, 13), max_kp=30., max_kd=2., home_offsets=OFFSETS,
                         homes=lambda: dict(homes), **kw)


def test_home_relative_envelope_accepts_inside_and_rejects_outside():
    c = commands()
    c.accept(batch({12: -0.0269 + 1.0, 13: 1.5795 - 1.0}, 10**9), 10**9 + 1)
    with pytest.raises(ValueError, match=r'12 target .* envelope \[-0\.0469, 1\.5731\]'):
        c.accept(batch({12: -0.0269 + 1.7, 13: 1.5795}, 2 * 10**9), 2 * 10**9 + 1)
    with pytest.raises(ValueError, match='envelope'):
        c.accept(batch({12: -0.0269 - 0.05, 13: 1.5795}, 3 * 10**9), 3 * 10**9 + 1)


def test_home_relative_envelope_requires_a_session_home():
    c = commands(homes={12: -0.0269})
    with pytest.raises(ValueError, match='Motor 13 is not homed this session'):
        c.accept(batch({12: 0.0, 13: 1.6}, 10**9), 10**9 + 1)


@pytest.mark.parametrize('kw', [
    dict(motor_ids=(11, 12, 13)),                       # yaw has no home-relative envelope
    dict(minimum=[0., 0.], maximum=[1., 1.]),            # absolute and relative together
    dict(home_offsets={'hip_pitch': (1., 0.), 'knee': (0., 1.)}),
    dict(home_offsets=OFFSETS, homes=None),
])
def test_invalid_home_relative_configuration(kw):
    args = dict(motor_ids=(12, 13), home_offsets=OFFSETS, homes=lambda: HOMES)
    args.update(kw)
    with pytest.raises(ValueError):
        MotorCommands(**args)


class FakeBus:
    """Records drive writes in order; feedback follows commands instantly."""

    def __init__(self, fail_cap=None, fail_stop=None):
        self.events, self.tx_lock, self.stale_after = [], threading.Lock(), 1.0
        self.limits = {12: 12.0, 13: 12.0}
        self.samples = {mid: Sample(received=time.monotonic(), fault_received=time.monotonic(),
                                    position=HOMES[mid], velocity=0., torque=0., temperature=30., mode=0)
                        for mid in (12, 13)}
        self.fail_cap, self.fail_stop = fail_cap, fail_stop
        self.drivers = [self.driver(mid) for mid in (12, 13)]

    def driver(self, motor_id):
        bus = self
        class Driver:
            def __init__(self):
                self.motor_id = motor_id
            def mode(self, mode):
                bus.events.append((motor_id, 'mode'))
            def control(self, position, velocity, torque, kp, kd):
                bus.events.append((motor_id, 'control'))
            def enable(self):
                bus.events.append((motor_id, 'enable'))
                bus.samples[motor_id].mode = 2
            def stop(self):
                if bus.fail_stop == motor_id:
                    raise OSError('ENOBUFS')
                bus.events.append((motor_id, 'stop'))
        return Driver()

    def refresh(self, disabled_ok=True):
        now = time.monotonic()
        for sample in self.samples.values():
            sample.received = sample.fault_received = now

    def snapshot(self):
        self.refresh()
        return {mid: Sample(**vars(s)) for mid, s in self.samples.items()}, ''

    def read_runtime_torque_limit(self, motor_id):
        self.events.append((motor_id, 'read_limit'))
        return self.limits[motor_id]

    def set_runtime_torque_limit(self, motor_id, limit):
        if self.fail_cap == motor_id and limit < 12:
            raise ValueError(f'Motor {motor_id} runtime torque-limit readback unavailable')
        self.events.append((motor_id, 'limit', limit))
        self.limits[motor_id] = limit
        return limit


def armed_motion(bus, torque_limit=1.5):
    c = MotorCommands(motor_ids=(12, 13), max_kp=30., max_kd=2., home_offsets=OFFSETS,
                      homes=lambda: dict(HOMES))
    motion = HardwareMotion(bus, c, allowed=True, torque_limit=torque_limit)
    sys.modules.setdefault('CyberGearDriver', NS(RunMode=NS(OPERATION_CONTROL=1)))
    stamp = time.monotonic_ns()
    c.accept(batch(dict(HOMES), stamp), stamp + 1)
    c.received = time.monotonic()
    return motion, c


def wait_for(predicate, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_cap_is_written_and_verified_before_any_enable_then_restored_after_disable():
    bus = FakeBus()
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[0]), motion.status()
        first_enable = next(i for i, e in enumerate(bus.events) if e[1] == 'enable')
        caps = [e for e in bus.events[:first_enable] if e[1] == 'limit']
        assert caps == [(12, 'limit', 1.5), (13, 'limit', 1.5)]
        assert {e[0] for e in bus.events[:first_enable] if e[1] == 'read_limit'} == {12, 13}
        motion.enable(False)
        assert wait_for(lambda: (12, 'stop') in bus.events and (13, 'stop') in bus.events)
        assert bus.limits == {12: 1.5, 13: 1.5}  # still capped: drives report enabled
        for sample in bus.samples.values():
            sample.mode = 0
        assert wait_for(lambda: bus.limits == {12: 12.0, 13: 12.0}), bus.events
        stops = max(i for i, e in enumerate(bus.events) if e[1] == 'stop')
        assert all(i > stops for i, e in enumerate(bus.events) if e[1] == 'limit' and e[2] == 12.0)
    finally:
        motion.close()


def test_failed_cap_write_enables_nothing_and_restores_the_capped_motor():
    bus = FakeBus(fail_cap=13)
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[1]), motion.status()
        assert 'readback unavailable' in motion.status()[2]
        assert not any(e[1] in ('enable', 'mode', 'control') for e in bus.events)
        assert wait_for(lambda: bus.limits[12] == 12.0), bus.events
    finally:
        motion.close()


def test_motor_whose_stop_fails_keeps_the_reduced_cap():
    bus = FakeBus(fail_stop=13)
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[0])
        motion.enable(False)
        assert wait_for(lambda: (12, 'stop') in bus.events)
        bus.samples[12].mode = 0
        assert wait_for(lambda: bus.limits[12] == 12.0)
        time.sleep(0.1)
        assert bus.limits[13] == 1.5
        assert 'Motor stop failed' in motion.status()[2] and motion.status()[1]
    finally:
        bus.fail_stop = None
        motion.close()


def test_rearm_before_restore_keeps_the_true_original():
    bus = FakeBus()
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[0])
        motion.enable(False)
        assert wait_for(lambda: (13, 'stop') in bus.events)
        stamp = time.monotonic_ns()
        c.accept(batch(dict(HOMES), stamp), stamp + 1)
        motion.enable(True)  # drives still report enabled, so no restore happened yet
        assert wait_for(lambda: motion.status()[0])
        # Only the first arm reads the drives; re-arming reuses the true originals, not 1.5.
        assert [e for e in bus.events if e[1] == 'read_limit'] == [(12, 'read_limit'), (13, 'read_limit')]
        motion.enable(False)
        assert wait_for(lambda: bus.events.count((13, 'stop')) == 2)
        for sample in bus.samples.values():
            sample.mode = 0
        assert wait_for(lambda: bus.limits == {12: 12.0, 13: 12.0})
    finally:
        motion.close()


def test_without_a_cap_no_limit_writes_happen():
    bus = FakeBus()
    motion, c = armed_motion(bus, torque_limit=None)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[0])
        assert not any(e[1] in ('limit', 'read_limit') for e in bus.events)
    finally:
        motion.close()


@pytest.mark.parametrize('limit', [0.0, -1.0, 13.0, float('nan')])
def test_invalid_cap_rejected(limit):
    with pytest.raises(ValueError, match='torque limit'):
        HardwareMotion(FakeBus(), commands(), torque_limit=limit)


def test_disarm_during_a_command_pass_is_a_normal_stop_not_a_fault():
    """2026-10-27 15:40: the player's disarm landed mid-pass and latched a FAULT."""
    bus = FakeBus()
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[0])
        faults_before = motion.fault_count
        original = bus.drivers[0].control

        def disarm_mid_pass(*command):  # during motor 12's write, before motor 13's
            motion.enable(False)
            return original(*command)
        bus.drivers[0].control = disarm_mid_pass
        assert wait_for(lambda: (12, 'stop') in bus.events and (13, 'stop') in bus.events)
        armed, faulted, reason = motion.status()
        assert (armed, faulted, reason) == (False, False, 'Operator disarmed')
        assert motion.fault_count == faults_before
        bus.drivers[0].control = original
        for sample in bus.samples.values():
            sample.mode = 0
        assert wait_for(lambda: bus.limits == {12: 12.0, 13: 12.0})  # caps restored as on any stop
    finally:
        motion.close()


def test_disarm_between_passes_reports_operator_disarmed():
    bus = FakeBus()
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[0])
        motion.enable(False)
        assert wait_for(lambda: (13, 'stop') in bus.events)
        assert wait_for(lambda: motion.status() == (False, False, 'Operator disarmed'))
    finally:
        motion.close()


def test_real_faults_count_once_each():
    bus = FakeBus(fail_cap=13)
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[1])
        assert motion.fault_count == 1
        time.sleep(0.1)
        assert motion.fault_count == 1  # latched, not re-counted every loop
    finally:
        motion.close()


def test_latched_motion_fault_clears_readiness_once_so_rehoming_sticks():
    """2026-10-27 15:41: legs rehomed after a motion fault were cleared again every update."""
    pytest.importorskip('rclpy')
    from tocatl_interfaces.msg import RobotState
    from tocatl_lowlevel.homing import LegReadiness
    from tocatl_lowlevel.motor_controller import MotorController

    def results(leg, homes):
        return [dict(motor_id=leg * 10 + j, home_position_rad=h, contact_positions_rad=[h],
                     contact_verified=True, config=dict(contacts_required=1)) for j, h in zip((2, 3), homes)]
    now = time.monotonic()
    samples = {mid: Sample(received=now, fault_received=now, position=0.1, velocity=0., torque=0.,
                           temperature=25., mode=0) for mid in range(11, 64)}
    hardware = NS(snapshot=lambda: ({mid: Sample(**{**vars(s), 'received': time.monotonic(),
                                                    'fault_received': time.monotonic()})
                                     for mid, s in samples.items()}, ''), stale_after=1.5)
    motion = NS(fault_count=1, status=lambda: (False, True, 'Operator disarmed'))
    node = NS(homing=NS(status=lambda: (False, False, 'idle')), motion=motion,
              readiness=LegReadiness(), hardware=hardware)
    node.readiness.complete(results(1, (0.1, 0.1)))

    def update():
        state = RobotState()
        state.safety_state, state.status_reason = RobotState.FAULT, 'Operator disarmed'
        MotorController.add_leg_status(node, state)
        return {leg.leg_id: leg.ready for leg in state.legs}
    assert update()[1] is False                      # the new fault clears readiness once
    node.readiness.complete(results(1, (0.1, 0.1)))   # operator rehomes leg 1
    assert update()[1] is True and update()[1] is True  # the same latched fault does not clear it again
    motion.fault_count = 2                            # a genuinely new fault does
    assert update()[1] is False


def yaw_setup():
    from tocatl_lowlevel.homing import LegReadiness
    now = time.monotonic()
    samples = {mid: Sample(received=now, fault_received=now, position=0.1 * (mid % 10), velocity=0., torque=0.,
                           temperature=25., mode=0) for mid in range(11, 64)}
    hardware = NS(snapshot=lambda: ({mid: Sample(**{**vars(s), 'received': time.monotonic(),
                                                    'fault_received': time.monotonic()})
                                     for mid, s in samples.items()}, ''), stale_after=1.5)
    readiness = LegReadiness()
    for leg in (1, 2):
        readiness.complete([dict(motor_id=leg * 10 + j, home_position_rad=0.1 * j, contact_positions_rad=[0.1 * j],
                                 contact_verified=True, config=dict(contacts_required=1)) for j in (2, 3)])
    return readiness, hardware, samples


def test_declare_yaw_records_positions_and_makes_yaw_commandable():
    readiness, hardware, samples = yaw_setup()
    assert readiness.declare_yaw(hardware, [11, 21]) == {11: pytest.approx(0.1), 21: pytest.approx(0.1)}
    homes, _ = readiness.snapshot()
    assert homes[11] == pytest.approx(0.1) and homes[12] == pytest.approx(0.2)
    readiness.require(hardware, [NS(motor_id=11), NS(motor_id=12), NS(motor_id=13)])
    c = MotorCommands(motor_ids=(11, 12, 13), max_kp=100., max_kd=5.,
                      home_offsets={'hip_yaw': (-0.3, 0.3), **OFFSETS}, homes=lambda: readiness.snapshot()[0])
    assert c.envelope(11) == (pytest.approx(-0.2), pytest.approx(0.4))


@pytest.mark.parametrize('change,message', [
    (dict(mode=2), 'motor 11 must be disabled'),
    (dict(faults=('Over current',)), 'motor 11 reports faults'),
    (dict(position=float('nan')), 'motor 11 position unavailable'),
])
def test_declare_yaw_refuses_unsafe_feedback_and_records_nothing(change, message):
    readiness, hardware, samples = yaw_setup()
    for key, value in change.items():
        setattr(samples[11], key, value)
    with pytest.raises(ValueError, match=message):
        readiness.declare_yaw(hardware, [11, 21])
    assert 11 not in readiness.snapshot()[0] and 21 not in readiness.snapshot()[0]


def test_declare_yaw_needs_a_homed_leg_and_yaw_ids():
    readiness, hardware, samples = yaw_setup()
    with pytest.raises(ValueError, match='leg 3 is not homed'):
        readiness.declare_yaw(hardware, [31])
    with pytest.raises(ValueError, match='yaw motor IDs'):
        readiness.declare_yaw(hardware, [12])
    with pytest.raises(ValueError, match='not homed'):
        readiness.require(hardware, [NS(motor_id=11)])  # yaw not declared yet


def test_yaw_reference_is_cleared_with_its_leg_and_on_lost_feedback():
    readiness, hardware, samples = yaw_setup()
    readiness.declare_yaw(hardware, [11, 21])
    readiness.invalidate('Homing started; pair not ready', 1)  # rehoming leg 1
    homes, _ = readiness.snapshot()
    assert 11 not in homes and 21 in homes
    samples[21].faults = ('Over temperature',)
    readiness.check(hardware)
    assert 21 not in readiness.snapshot()[0] and 22 not in readiness.snapshot()[0]


def test_background_polling_scales_with_the_number_of_waiting_motors():
    from tocatl_lowlevel.hardware import background_per_sweep, select_drivers
    assert background_per_sweep(8, 0.10, 1.0) == 1    # Level 2/3: unchanged, one per sweep
    assert background_per_sweep(14, 0.10, 1.0) == 2   # Level 4: 14 waiting motors need two
    assert background_per_sweep(16, 0.12, 1.0) == 2
    assert background_per_sweep(16, 0.30, 1.0) == 5
    assert background_per_sweep(0, 0.10, 1.0) == 0
    assert background_per_sweep(5, float('nan'), 1.0) == 1  # before the first measured sweep
    assert background_per_sweep(3, 5.0, 1.0) == 3          # never more than exist
    drivers = [NS(motor_id=m) for m in (11, 12, 13, 21, 22, 23)]
    chosen, index = select_drivers(drivers, (12, 13), 0, 2)
    assert [d.motor_id for d in chosen] == [12, 13, 11, 21] and index == 2
    chosen, index = select_drivers(drivers, (12, 13), index, 2)
    assert [d.motor_id for d in chosen] == [12, 13, 22, 23] and index == 4
    chosen, _ = select_drivers(drivers, (12, 13), index, 2)
    assert [d.motor_id for d in chosen] == [12, 13, 11, 21]  # rotation wraps
    assert [d.motor_id for d in select_drivers(drivers, None, 7, 2)[0]] == [11, 12, 13, 21, 22, 23]



def test_background_rotation_can_be_limited_to_ready_motors():
    from tocatl_lowlevel.hardware import select_drivers
    drivers = [NS(motor_id=m) for m in (11, 12, 13, 21, 22, 23, 31, 32, 33)]
    chosen, index = select_drivers(drivers, (32, 33), 0, 1, background_ids={12, 13})
    assert [d.motor_id for d in chosen] == [32, 33, 12]
    chosen, index = select_drivers(drivers, (32, 33), index, 1, background_ids={12, 13})
    assert [d.motor_id for d in chosen] == [32, 33, 13]  # 11, 21-23 (not ready) wait for homing to end
    assert [d.motor_id for d in select_drivers(drivers, (32, 33), 0, 1, background_ids=set())[0]] == [32, 33]



def test_leg_scope_reports_a_declared_yaw():
    pytest.importorskip('rclpy')
    from tocatl_interfaces.msg import RobotState
    from tocatl_lowlevel.motor_controller import MotorController
    readiness, hardware, samples = yaw_setup()
    node = NS(homing=NS(status=lambda: (False, False, 'idle')), motion=None, readiness=readiness, hardware=hardware)

    def scopes():
        state = RobotState()
        state.safety_state = RobotState.DISARMED
        MotorController.add_leg_status(node, state)
        return {leg.leg_id: leg.scope for leg in state.legs}
    assert scopes()[1] == 'pitch_knee'
    readiness.declare_yaw(hardware, [11])
    assert scopes()[1] == 'pitch_knee_yaw' and scopes()[2] == 'pitch_knee'



@pytest.mark.parametrize('change,expected', [
    (dict(fault_received=-2.0), 'Motor 22 fault-status feedback 2.00 s old (limit 1.5 s)'),
    (dict(temperature=46.0), 'Motor 22 temperature 46.0 C (limit 45 C)'),
    (dict(state_faults=('Under voltage',)), "Motor 22 faults ['Under voltage']"),
    (dict(mode=1), 'Motor 22 mode 1'),
])
def test_readiness_drop_names_the_motor_and_condition_and_is_logged(change, expected):
    """2026-10-27 17:06: a stand faulted and the logs could not say which motor or why."""
    readiness, hardware, samples = yaw_setup()
    logged = []
    readiness.log = logged.append
    snapshot = hardware.snapshot

    def changed():
        all_samples, error = snapshot()
        values = {k: (time.monotonic() + v if k.endswith('received') else v) for k, v in change.items()}
        all_samples[22] = Sample(**{**vars(all_samples[22]), **values})
        return all_samples, error
    hardware.snapshot = changed
    readiness.check(hardware)
    assert readiness.leg_reason(2) == expected + '; rehome required'
    assert logged == [f'Readiness cleared for legs [2]: {expected}; rehome required']
    assert 12 in readiness.snapshot()[0]  # leg 1 unaffected


def test_motion_faults_are_logged_once_each():
    pytest.importorskip('rclpy')
    from tocatl_lowlevel.motor_controller import MotorController
    logged = []
    motion = NS(fault_count=0, status=lambda: (False, True, 'Motor 42 telemetry/fault status unavailable or stale'))
    node = NS(motion=motion, get_logger=lambda: NS(error=logged.append))
    MotorController.log_motion_fault(node)
    assert logged == []
    motion.fault_count = 1
    MotorController.log_motion_fault(node)
    MotorController.log_motion_fault(node)
    assert logged == ['Motion fault: Motor 42 telemetry/fault status unavailable or stale']


def test_fair_lock_serves_waiters_in_arrival_order():
    from tocatl_lowlevel.hardware import FairLock
    lock, order = FairLock(), []
    lock.acquire()
    threads = []
    for name in ('poll', 'motion', 'homing'):
        t = threading.Thread(target=lambda n=name: (lock.acquire(), order.append(n), lock.release()))
        t.start()
        threads.append(t)
        time.sleep(0.05)  # fix the arrival order
    lock.release()
    for t in threads:
        t.join(1)
    assert order == ['poll', 'motion', 'homing']
    with lock:  # usable as a context manager
        pass


def test_motion_pass_takes_the_can_lock_per_motor_and_extrapolates():
    """A long pass no longer holds the lock throughout, so telemetry can interleave."""
    bus = FakeBus()
    acquisitions = []

    class CountingLock:
        def __init__(self):
            self.inner = threading.Lock()
        def __enter__(self):
            self.inner.acquire()
            acquisitions.append(threading.current_thread().name)
        def __exit__(self, *exc):
            self.inner.release()
    bus.tx_lock = CountingLock()
    motion, c = armed_motion(bus)
    try:
        motion.enable(True)
        assert wait_for(lambda: motion.status()[0])
        first_pass = [e for e in bus.events if e[1] in ('mode', 'control', 'enable')][:6]
        assert [e[0] for e in first_pass] == [12, 12, 12, 13, 13, 13]
        assert len(acquisitions) >= 2  # at least one acquisition per motor
    finally:
        motion.close()


def test_extrapolation_follows_velocity_capped_and_clamped():
    from tocatl_lowlevel.motor_commands import EXTRAPOLATE_MAX_S, Target
    motion = HardwareMotion(FakeBus(), commands())
    try:
        target = Target(12, HOMES[12] + 0.5, 1.0, 0.0, 30.0, 2.0)
        assert motion.extrapolate(target, 0.05) == pytest.approx(HOMES[12] + 0.55)
        assert motion.extrapolate(target, 1.0) == pytest.approx(HOMES[12] + 0.5 + EXTRAPOLATE_MAX_S)
        assert motion.extrapolate(Target(12, HOMES[12] + 0.5, 0.0, 0.0, 30.0, 2.0), 0.1) == HOMES[12] + 0.5
        edge = Target(12, HOMES[12] + 1.59, 1.0, 0.0, 30.0, 2.0)
        assert motion.extrapolate(edge, 0.1) == pytest.approx(HOMES[12] + 1.6)  # stays inside the envelope
    finally:
        motion.close()


def telemetry_with_samples(received_ages, priority=None):
    """HardwareTelemetry over fake CAN; returns (hardware, calls) after one sweep."""
    from tocatl_lowlevel.hardware import HardwareTelemetry
    calls, swept = [], threading.Event()

    class Bus:
        def shutdown(self): pass

    class Notifier:
        exception = None
        def __init__(self, *args): pass
        def stop(self): pass

    class Driver:
        def __init__(self, motor_id, send_message):
            self.motor_id = motor_id
        def request_motor_state(self):
            calls.append(('state', self.motor_id))
        def request_parameter(self, name):
            calls.append((name, self.motor_id))
            if self.motor_id == 13:
                swept.set()
    import tocatl_lowlevel.hardware as h
    original = h.HardwareTelemetry.poll
    started = threading.Event()

    def delayed_poll(self):  # set sample ages before the first sweep
        started.wait(1)
        original(self)
    h.HardwareTelemetry.poll = delayed_poll
    try:
        hw = HardwareTelemetry(lambda: None, {'can0': 'unused'}, motor_ids=[12, 13],
                               can_module=NS(interface=NS(Bus=lambda **kw: Bus()), Notifier=Notifier),
                               driver_module=NS(CyberGearMotor=Driver))
    finally:
        h.HardwareTelemetry.poll = original
    now = time.monotonic()
    for mid, age in received_ages.items():
        hw.samples[mid].received = None if age is None else now - age
    hw.priority_motor = priority
    started.set()
    assert swept.wait(2)
    hw.close()
    return calls


def test_poll_skips_state_requests_that_command_replies_already_cover():
    calls = telemetry_with_samples({12: 0.05, 13: None})
    assert ('state', 12) not in calls and ('faultSta', 12) in calls  # replied to a command just now
    assert ('state', 13) in calls and ('faultSta', 13) in calls      # no recent state
    homing = telemetry_with_samples({12: 0.05, 13: 0.05}, priority=(12, 13))
    assert ('state', 12) in homing and ('state', 13) in homing      # homing keeps full polling
