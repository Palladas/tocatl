"""Feedback simulations and readiness checks; never open a CAN interface."""
from dataclasses import replace
from types import SimpleNamespace as NS
import time

import pytest

from tocatl_lowlevel.hardware import Sample
from tocatl_lowlevel.homing import KneeSeekConfig, KneeSeekSequence, LegReadiness
from tocatl_lowlevel.motor_commands import MotorCommands


def config(**changes):
    return KneeSeekConfig(motor_id=12, expected_stop=0.8, minimum=0.5, maximum=1.2, **changes)


def simulate_stop(stop=0.8):
    seq = KneeSeekSequence(config(), stop, 0.)
    position, reference, previous, torque = stop, stop, stop, 0.
    phases = []
    for tick in range(1, 3001):
        now = tick * 0.02
        position = max(stop, position + max(-0.004, min(0.004, reference-position)))
        velocity = (position-previous)/0.02
        previous = position
        command = seq.update(position, velocity, torque, now, now)
        phases.append(seq.phase)
        reference = command.position
        torque = seq.c.kp*(reference-position)
        if seq.done:
            return seq, phases
    pytest.fail('Simulated homing did not complete')


def test_three_independent_contacts_with_backoffs():
    seq, phases = simulate_stop()
    result = seq.result()
    assert result['independent_reapproaches']
    assert result['contact_positions_rad'] == pytest.approx([0.8]*3)
    assert not result['device_zero_written']
    assert sum(a != b and b == 'BACKOFF' for a, b in zip(phases, phases[1:])) == 2


def test_obstruction_is_not_a_home():
    with pytest.raises(ValueError, match='outside expected stop window'):
        simulate_stop(stop=0.85)


def test_cached_feedback_cannot_complete_contact():
    seq = KneeSeekSequence(config(), 0.9, 0.)
    seq.update(0.9, 0., 0., 0.02, 0.02)
    for _ in range(50):
        assert seq.update(0.9, 0., 0., 0.02, 0.03) is None
    assert not seq.contacts and not seq.done


@pytest.mark.parametrize('position,velocity,torque,stamp,now,error', [
    (0.8, 0., 0., 0., 1., 'stale'),
    (0.8, 0., 3.6, 0.02, 0.02, 'effort'),
    (1.3, 0., 0., 0.02, 0.02, 'configured range'),
    (float('nan'), 0., 0., 0.02, 0.02, 'Nonfinite'),
])
def test_homing_rejects_invalid_feedback(position, velocity, torque, stamp, now, error):
    with pytest.raises(ValueError, match=error):
        KneeSeekSequence(config(), 0.8, 0.).update(position, velocity, torque, stamp, now)


def results():
    return [dict(motor_id=mid, home_position_rad=0.8, independent_reapproaches=True,
                 contact_positions_rad=[0.8]*3) for mid in (12, 13)]


def hardware():
    now = time.monotonic()
    samples = {mid: Sample(received=now, fault_received=now, position=0.8, velocity=0.,
                           torque=0., temperature=25., mode=0) for mid in (12, 13)}
    return NS(snapshot=lambda: (samples, ''), stale_after=1.5,
              read_runtime_torque_limit=lambda mid: 12.,
              set_runtime_torque_limit=lambda mid, limit: limit), samples


def test_ready_requires_both_motors_and_latches_feedback_loss():
    ready = LegReadiness()
    hw, samples = hardware()
    assert not ready.check(hw)
    with pytest.raises(ValueError):
        ready.complete(results()[:1])
    ready.complete(results())
    assert ready.check(hw)
    ready.require(hw, [NS(motor_id=12), NS(motor_id=13)])
    with pytest.raises(ValueError, match='not homed'):
        ready.require(hw, [NS(motor_id=11)])
    samples[13] = replace(samples[13], faults=('Over temperature',))
    assert not ready.check(hw)
    samples[13] = replace(samples[13], faults=())
    assert not ready.check(hw)  # restored feedback alone cannot restore readiness
    assert not LegReadiness().check(hw)  # no automatic restoration on restart


def test_selected_pair_does_not_accept_yaw_or_other_legs():
    from test_control import batch
    commands = MotorCommands(motor_ids=[12, 13])
    msg = batch()
    with pytest.raises(ValueError, match='configured motor IDs'):
        commands.accept(msg, 10**9)
    msg.motors = [m for m in msg.motors if m.motor_id in (12, 13)]
    assert [t.motor_id for t in commands.accept(msg, 10**9)] == [12, 13]


def test_pair_success_requires_both_disabled_saved_stages(monkeypatch, tmp_path):
    import tocatl_lowlevel.homing as homing
    hw, samples = hardware()
    calls = []
    class Child:
        def __init__(self, hardware, config, output, reference):
            self.config = config
            calls.append(config.motor_id)
            self.completed_result = next(r for r in results() if r['motor_id'] == config.motor_id)
            self.thread = NS(join=lambda: None)
        def start(self): pass
        def status(self): return False, False, 'disabled'
        def cancel(self): pass
    monkeypatch.setattr(homing, 'BoundedHipHoming', Child)
    monkeypatch.setattr(homing, 'BoundedKneeHoming', Child)
    pair = homing.LegPairHoming(hw, tmp_path/'pair.json')
    pair.run()
    assert calls == [12, 13]
    assert pair.readiness.check(hw)
    import json
    assert json.loads((tmp_path/'pair.json').read_text())['legs']['1']['motor_ids'] == [12, 13]
    pair.cancel()
    assert not pair.readiness.check(hw)


def test_pair_failure_never_sets_ready(monkeypatch, tmp_path):
    import tocatl_lowlevel.homing as homing
    hw, _ = hardware()
    class Child:
        def __init__(self, *args):
            self.completed_result = None
            self.thread = NS(join=lambda: None)
        def start(self): pass
        def cancel(self): pass
        def status(self): return False, True, 'No fresh disabled acknowledgement'
    monkeypatch.setattr(homing, 'BoundedHipHoming', Child)
    pair = homing.LegPairHoming(hw, tmp_path/'pair.json')
    pair.run()
    assert pair.faulted and not pair.readiness.check(hw)
    assert not (tmp_path/'pair.json').exists()


def test_speed_guard_uses_fresh_encoders_not_noisy_drive_estimator():
    seq = KneeSeekSequence(config(), 0.8, 0.)
    seq.update(0.8, 0.7, 0., 0.02, 0.02)
    with pytest.raises(ValueError, match='encoder overspeed'):
        seq.update(0.82, 0., 0., 0.04, 0.04)



def test_runtime_torque_readback_is_not_fault_feedback():
    import struct, threading
    from tocatl_lowlevel.hardware import HardwareTelemetry
    hw = HardwareTelemetry.__new__(HardwareTelemetry)
    hw.lock = threading.Lock()
    hw.samples = {12: Sample()}
    driver = NS(host_id=0)
    msg = NS(is_extended_id=True, is_error_frame=False, is_remote_frame=False,
             arbitration_id=(17 << 24) | (12 << 8),
             data=struct.pack('<H', 0x700b) + bytes(2) + struct.pack('<f', 2.625))
    hw.receive(msg, {12: driver})
    assert hw.samples[12].runtime_torque_limit == 2.625
    assert hw.samples[12].torque_limit_received is not None
    assert hw.samples[12].fault_received is None
    assert hw.samples[12].received is None


def test_native_impedance_keeps_fast_drive_damping_and_bounds_spring():
    from tocatl_lowlevel.homing import NativeImpedanceHoming, PDCommand
    calls = []
    driver = NS(motor_id=12, control=lambda *args: calls.append(args))
    sample = Sample(position=0.8, runtime_torque_limit=2.625,
                    torque_limit_received=time.monotonic())
    worker = NS(config=config(), hardware=NS(snapshot=lambda: ({12: sample}, ''), drivers=[driver]),
                checked_sample=lambda: None, owned_motor_ids=lambda: (12,))
    NativeImpedanceHoming.preload_owned(worker, driver, {12: sample})
    NativeImpedanceHoming.send_command(worker, driver, sample, PDCommand(-10., 1.75, 2.))
    position, velocity, torque, kp, kd = calls[-1]
    assert velocity == torque == 0.
    assert kp == 1.75 and kd == 2.
    assert abs(kp*(position-sample.position)) <= 2.625
    sample.runtime_torque_limit = float('nan')
    with pytest.raises(ValueError):
        NativeImpedanceHoming.preload_owned(worker, driver, {12: sample})


def test_pair_position_envelopes_use_selected_motor_ids():
    commands = MotorCommands(motor_ids=[12, 13], minimum=[0.65, 0.27], maximum=[1.1, 1.47])
    assert commands.envelope_valid
    assert commands.minimum == {12: 0.65, 13: 0.27}
    assert commands.maximum == {12: 1.1, 13: 1.47}
    assert not MotorCommands(motor_ids=[12, 13], minimum=[0.], maximum=[1.]).envelope_valid


def test_hip_homing_knee_has_damping_and_zero_stiffness():
    from tocatl_lowlevel.homing import NativeImpedanceHoming, PDCommand
    calls = {12: [], 13: []}
    drivers = [NS(motor_id=mid, control=lambda *args, mid=mid: calls[mid].append(args))
               for mid in (12, 13)]
    samples = {mid: Sample(position=0.8, runtime_torque_limit=2.625,
                          torque_limit_received=time.monotonic()) for mid in (12, 13)}
    worker = NS(config=config(), hardware=NS(snapshot=lambda: (samples, ''), drivers=drivers),
                checked_sample=lambda: None, owned_motor_ids=lambda: (12, 13))
    for driver in drivers:
        NativeImpedanceHoming.preload_owned(worker, driver, samples)
    NativeImpedanceHoming.send_command(worker, drivers[0], samples[12], PDCommand(0.9, 1.75, 2.))
    assert calls[13][-1][1:] == (0., 0., 0., 2.)
    assert calls[12][-1][3:] == (1.75, 2.)


def test_home_request_reports_pending_stop_instead_of_only_already_active():
    from tocatl_lowlevel.homing import LegPairHoming
    pair = LegPairHoming(None, 'unused.json')
    pair.thread = NS(is_alive=lambda: True)
    pair.child = NS(stop_pending=True,
                    status=lambda: (True, True, 'CAN failed; stop pending'))
    with pytest.raises(ValueError, match='Stop must be confirmed before retry') as error:
        pair.start()
    assert 'CAN failed; stop pending' in str(error.value)
    pair.child = NS(stop_pending=False, status=lambda: (True, False, 'Seeking stop'))
    with pytest.raises(ValueError, match='Leg homing already active'):
        pair.start()


def test_leg_fault_reason_preserves_underlying_controller_error():
    pytest.importorskip('rclpy')
    from tocatl_interfaces.msg import RobotState
    from tocatl_lowlevel.motor_controller import MotorController
    state = RobotState()
    state.safety_state = RobotState.FAULT
    state.status_reason = 'CAN transmit failed; stop pending'
    worker = NS(readiness=LegReadiness(), hardware=None)
    MotorController.add_leg_status(worker, state)
    assert not state.legs[0].ready
    assert 'CAN transmit failed; stop pending' in state.legs[0].status_reason


def test_transport_polls_only_selected_motors():
    import threading
    from tocatl_lowlevel.hardware import HardwareTelemetry
    calls = []
    updated = threading.Event()
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
            if self.motor_id == 13: updated.set()
    hw = HardwareTelemetry(lambda: None, {'can1': 'unused'}, motor_ids=[12, 13],
                           can_module=NS(interface=NS(Bus=lambda **kwargs: Bus()), Notifier=Notifier),
                           driver_module=NS(CyberGearMotor=Driver))
    try:
        assert updated.wait(1)
    finally:
        hw.close()
    assert set(calls) == {('state', 12), ('faultSta', 12), ('state', 13), ('faultSta', 13)}
    assert hw.samples[51].received is None


@pytest.mark.parametrize('ids', [[], [12, 12], [12, 27], [22], [12.0]])
def test_invalid_telemetry_selection_rejected_before_opening_can(ids):
    from tocatl_lowlevel.hardware import HardwareTelemetry
    with pytest.raises(ValueError, match='telemetry motor IDs'):
        HardwareTelemetry(lambda: None, {'can1': 'unused'}, motor_ids=ids)


def simulate_arbitrary_start(motor_id, start, stop):
    c = KneeSeekConfig.from_start(motor_id, stop, start, verify_repeatability=True)
    seq = KneeSeekSequence(c, start, 0.)
    position = previous = reference = start
    torque = 0.
    phases = []
    first_contact_time = None
    for tick in range(1, int(c.timeout/.05)):
        now = tick * .05
        position = max(stop, position + max(-.0025, min(.0025, reference-position)))
        velocity = (position-previous)/.05
        previous = position
        command = seq.update(position, velocity, torque, now, now)
        phases.append(seq.phase)
        reference = command.position
        torque = max(-c.command_torque_limit, min(c.command_torque_limit,
                     command.kp*(reference-position)-command.kd*velocity))
        if seq.contacts and first_contact_time is None:
            first_contact_time = now
        if seq.done:
            return seq, phases, first_contact_time
    pytest.fail('Arbitrary-start simulated homing did not complete')


@pytest.mark.parametrize('motor_id,stop,start', [
    (12, .8028152895399412, 2.988670176241703),
    (12, .8028152895399412, 6.0),
    (12, .8028152895399412, 12.5),
    (13, .7394903486686513, 3.0),
])
def test_homing_approaches_from_far_start_then_repeats_contacts(motor_id, stop, start):
    seq, phases, first_contact_time = simulate_arbitrary_start(motor_id, start, stop)
    assert seq.initial_phase == 'SEEK_STOP'
    assert seq.result()['contact_positions_rad'] == pytest.approx([stop]*3)
    assert seq.c.minimum <= start <= seq.c.maximum
    assert first_contact_time > abs(start-stop)/seq.c.reference_speed
    assert seq.c.initial_seek_timeout > first_contact_time
    assert seq.c.initial_seek_timeout > 60
    # First contact comes from the initial approach; later contacts require actual backoff.
    assert sum(a != b and b == 'BACKOFF' for a, b in zip(phases, phases[1:])) == 2


def test_far_start_first_command_moves_toward_home():
    c = KneeSeekConfig.from_start(12, .8028, 2.989)
    seq = KneeSeekSequence(c, 2.989, 0.)
    seq.update(2.989, 0., 0., .05, .05)
    command = seq.update(2.989, 0., 0., .10, .10)
    assert command.position < 2.989
    assert command.position >= 2.989-c.reference_speed*.05-1e-12


def test_loading_while_moving_is_not_mistaken_for_obstruction():
    c = KneeSeekConfig.from_start(12, .8028, 2.989, verify_repeatability=True)
    seq = KneeSeekSequence(c, 2.989, 0.)
    seq.reference = 2.989-1.4
    for tick in range(1, 41):
        now = tick*.05
        position = 2.989-.0025*tick
        seq.update(position, -.05, -2.4, now, now)
    assert not seq.contacts
    # A sustained stationary loaded contact away from home remains an obstruction.
    with pytest.raises(ValueError, match='outside expected stop window'):
        for tick in range(41, 61):
            now = tick*.05
            seq.update(position, 0., -2.4, now, now)


def test_arbitrary_start_search_is_bounded_and_has_finite_time_budget():
    c = KneeSeekConfig.from_start(12, .8028, 2.989)
    seq = KneeSeekSequence(c, 2.989, 0.)
    with pytest.raises(ValueError, match='configured range'):
        seq.update(c.maximum+.001, 0., 0., .05, .05)
    with pytest.raises(ValueError, match='timeout'):
        KneeSeekSequence(c, 2.989, 0.).update(
            2.989, 0., 0., c.initial_seek_timeout+.01, c.initial_seek_timeout+.01)


@pytest.mark.parametrize('start', [float('nan'), float('inf'), -12.6, 12.6])
def test_arbitrary_start_rejects_invalid_coordinates(start):
    with pytest.raises(ValueError):
        KneeSeekConfig.from_start(12, .8028, start)


def test_following_knee_accepts_pose_outside_old_commissioning_window():
    from tocatl_lowlevel.homing import BoundedHipHoming
    hw, samples = hardware()
    samples[12].position = 2.989
    samples[13].position = -1.2
    worker = BoundedHipHoming(hw, KneeSeekConfig.from_start(12, .8028, 2.989),
                              'unused.json', float('nan'))
    sample, _ = worker.checked_sample(initial=True)
    assert sample.position == 2.989


def simulate_normal_home(motor_id, start, stop, reference_stop=None):
    c = KneeSeekConfig.from_start(motor_id, stop if reference_stop is None else reference_stop, start)
    seq = KneeSeekSequence(c, start, 0.)
    position = reference = previous = start
    torque = 0.
    for tick in range(1, 3001):
        now = tick*.02
        position = max(stop, position + max(-.004, min(.004, reference-position)))
        velocity = (position-previous)/.02
        previous = position
        command = seq.update(position, velocity, torque, now, now)
        reference = command.position
        torque = max(-c.command_torque_limit, min(c.command_torque_limit,
                     command.kp*(reference-position)-command.kd*velocity))
        assert seq.phase != 'BACKOFF'
        if seq.done:
            return seq.result(), now
    pytest.fail('Normal homing exceeded 60 seconds')


def test_normal_pair_has_one_verified_contact_and_finishes_simulation_under_minute():
    hip, hip_time = simulate_normal_home(12, 2.988670176241703, .8028152895399412)
    knee, knee_time = simulate_normal_home(13, .9767681391622798, .7394903486686513)
    assert hip_time+knee_time < 60
    for result in (hip, knee):
        assert result['config']['reference_speed'] == .2
        assert result['config']['contacts_required'] == 1
        assert len(result['contact_positions_rad']) == 1
        assert result['contact_verified'] and not result['independent_reapproaches']
        assert result['config']['command_torque_limit'] == 1.5
    ready = LegReadiness()
    ready.complete([hip, knee])
    assert set(ready.snapshot()[0]) == {12, 13}


def test_normal_home_at_stop_skips_unnecessary_initial_backoff():
    result, elapsed = simulate_normal_home(12, .8028, .8028)
    assert elapsed < 10
    assert result['initial_phase'] == 'SEEK_STOP'


def test_single_contact_cannot_set_ready_without_contact_verification():
    r, _ = simulate_normal_home(12, .8028, .8028)
    r['contact_verified'] = False
    knee, _ = simulate_normal_home(13, .7395, .7395)
    with pytest.raises(ValueError, match='verified contact'):
        LegReadiness().complete([r, knee])


def test_pair_deadline_aborts_worker_even_when_phase_timeout_has_time_remaining():
    from tocatl_lowlevel.homing import BoundedHipHoming
    hw, _ = hardware()
    worker = BoundedHipHoming(hw, KneeSeekConfig.from_start(12, .8028, .8),
                              'unused.json', float('nan'))
    worker.deadline = time.monotonic()-1
    with pytest.raises(ValueError, match='overall timeout'):
        worker.checked_sample(initial=True)


def test_commissioning_is_explicit_and_uses_separate_default_budget():
    from tocatl_lowlevel.homing import create_homing
    for verify, timeout in ((False, 60.), (True, 600.)):
        values = {'homing.method': 'leg1_pair', 'homing.verify_repeatability': verify}
        node = NS(declare_parameter=lambda name, default: NS(value=values.get(name, default)))
        pair = create_homing(node, None)
        assert pair.verify_repeatability == verify
        assert pair.timeout_s == timeout


def test_ros_completion_is_logged_once_and_published_as_ready():
    rclpy = pytest.importorskip('rclpy')
    from tocatl_lowlevel.motor_controller import MotorController
    from tocatl_lowlevel.motor_map import MOTORS
    rclpy.init()
    node = None
    original_logger = None
    try:
        node = MotorController()
        now = time.monotonic()
        samples = {m.motor_id: Sample(received=now, fault_received=now, mode=0,
                    position=.8, velocity=0., torque=0., temperature=25.) for m in MOTORS}
        node.hardware = NS(snapshot=lambda: (samples, ''), stale_after=1.5, close=lambda: None)
        node.motion = NS(status=lambda: (False, False, 'Hardware motion disabled'), close=lambda: None)
        hip, _ = simulate_normal_home(12, .8028, .8028)
        knee, _ = simulate_normal_home(13, .7395, .7395)
        node.readiness.complete([hip, knee])
        message = 'Leg 1 pitch/knee READY; yaw excluded; motors disabled'
        node.homing = NS(active=False, readiness=node.readiness,
                         status=lambda: (False, False, message), close=lambda: None)
        node.homing_was_active = True
        states, logs = [], []
        node.state_publisher = NS(publish=states.append)
        original_logger = node.get_logger
        node.get_logger = lambda: NS(info=logs.append, error=logs.append)
        node.update()
        node.update()
        assert logs == [message]
        assert states[-1].status_reason == message
        assert states[-1].legs[0].ready
        assert not states[-1].motors_enabled
    finally:
        if node is not None:
            if original_logger is not None:
                node.get_logger = original_logger
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('start', [.7, 1.1])
def test_normal_homing_learns_contact_outside_old_reference_and_corridor(start):
    result, _ = simulate_normal_home(12, start, .6, reference_stop=.8028)
    assert result['home_position_rad'] == pytest.approx(.6)
    assert result['reference_delta_rad'] == pytest.approx(-.2028)
    assert not result['reference_within_window']
    assert result['contact_verified']


def test_normal_homing_does_not_limit_elastic_motion_to_start_corridor():
    c = KneeSeekConfig.from_start(12, .8028, .9)
    assert (c.minimum, c.maximum) == (-12.5, 12.5)
    seq = KneeSeekSequence(c, .9, 0.)
    seq.update(.9, 0., 0., .02, .02)
    command = seq.update(1.05, 0., 0., .82, .82)
    assert command.position < 1.05
    assert not seq.done


@pytest.mark.parametrize('direction', (-1, 1))
def test_pair_unloads_spring_then_fades_damping_before_completion(direction):
    c = KneeSeekConfig.from_start(12, .8, .8, direction=direction)
    seq = KneeSeekSequence(c, .8, 0.)
    for tick in range(1, 500):
        now = tick*.02
        command = seq.update(.8, 0., direction*1.2, now, now)
        if seq.phase == 'UNLOADING':
            break
    assert seq.phase == 'UNLOADING' and not seq.done
    contact = seq.contacts[:]
    commands = []
    for tick in range(1, 152):
        now = seq.release_started+tick*.02
        commands.append(seq.update(.8, 0., 0., now, now))
        if seq.done:
            break
    errors = [abs(command.position-.8) for command in commands]
    assert all(b <= a+1e-12 for a,b in zip(errors, errors[1:]))
    assert commands[0].kd == c.kd
    assert commands[-1].kp == commands[-1].kd == commands[-1].release_scale == 0
    assert seq.done and seq.contacts == contact
    assert seq.result()['release_completed']


def test_unload_preserves_feedback_and_effort_guards():
    c = KneeSeekConfig.from_start(12, .8, .8)
    for fault in ('stale', 'effort', 'speed'):
        seq = KneeSeekSequence(c, .8, 0.)
        seq.phase = 'UNLOADING'
        seq.release_started = 0.
        seq.release_error = -.6
        seq.last_time, seq.last_position = .02, .8
        with pytest.raises(ValueError):
            seq.update(.9 if fault == 'speed' else .8, 0.,
                       4. if fault == 'effort' else 0., .04,
                       .5 if fault == 'stale' else .04)
