"""All-leg lifecycle simulations, using fake drives and no CAN devices."""
from dataclasses import asdict, replace
from types import SimpleNamespace as NS
import json
import threading
import time
import sys

import pytest

from tocatl_lowlevel.hardware import Sample
from tocatl_lowlevel.homing import (HipKneeHoming, LegHomingConfig, LegReadiness,
                                   KneeSeekConfig, KneeSeekSequence, ActiveKneeHoming,
                                   PDCommand)
from tocatl_lowlevel.motor_map import MOTORS, HIP_KNEE_IDS, hip_knee_ids


def profiles():
    return {leg: LegHomingConfig(leg, .8+leg*.01, .7+leg*.01,
                                -1 if leg % 2 else 1, -1 if leg % 2 else 1)
            for leg in range(1, 7)}


class FakeHardware:
    def __init__(self):
        self.stale_after = 1.5
        self.tx_lock = threading.RLock()
        self.priority_motor = None
        self.events = []
        self.fail_read = None
        self.cancel_at = None
        self.owner = None
        self.samples = {m.motor_id: Sample(mode=0, position=.8, velocity=0., torque=0.,
                        temperature=25., runtime_torque_limit=12.) for m in MOTORS}
        for leg, p in profiles().items():
            self.samples[leg*10+2].position = p.hip_stop
            self.samples[leg*10+3].position = p.knee_stop
        self.drivers = [FakeDriver(mid, self) for mid in HIP_KNEE_IDS]

    def snapshot(self):
        now = time.monotonic()
        return {mid: replace(s, received=now, fault_received=now, torque_limit_received=now)
                for mid, s in self.samples.items()}, ''

    def read_runtime_torque_limit(self, mid):
        if mid == self.fail_read:
            raise ValueError('Simulated torque-limit read failure')
        return self.samples[mid].runtime_torque_limit

    def set_runtime_torque_limit(self, mid, value):
        self.events.append((mid, 'limit', value))
        self.samples[mid].runtime_torque_limit = value
        return value

    def close(self): pass


class FakeDriver:
    def __init__(self, mid, hw):
        self.motor_id, self.hw = mid, hw
    def mode(self, mode): self.hw.events.append((self.motor_id, 'mode', mode))
    def control(self, *command): self.hw.events.append((self.motor_id, 'control', command))
    def enable(self):
        self.hw.events.append((self.motor_id, 'enable', None))
        self.hw.samples[self.motor_id].mode = 2
    def stop(self):
        self.hw.events.append((self.motor_id, 'stop', None))
        self.hw.samples[self.motor_id].mode = 0
    def request_motor_state(self): pass
    def request_parameter(self, name): pass


class VerifiedContact:
    """Shorten mechanical contact acquisition; retain real worker enable/stop/save paths."""
    def __init__(self, config, hw):
        self.config, self.hw = config, hw
        self.calls = 0
        self.done = False
        self.phase = 'SEEK_STOP'
        self.positions = []
        self.contacts = []
    def update(self, *args):
        self.calls += 1
        if self.hw.cancel_at == self.config.motor_id:
            self.hw.owner.cancel()
        if self.calls >= 2:
            self.done = True
            self.phase = 'DONE'
            self.positions = [self.config.expected_stop]*3
            self.contacts = [self.config.expected_stop]
        return PDCommand(self.config.expected_stop, self.config.kp, self.config.kd)
    def result(self):
        assert self.done
        mid = self.config.motor_id
        return dict(motor_id=mid, bus=next(m.bus for m in MOTORS if m.motor_id == mid),
                    home_position_rad=self.config.expected_stop, contact_positions_rad=self.contacts,
                    contact_verified=True, independent_reapproaches=False,
                    config=asdict(self.config), device_zero_written=False)


@pytest.fixture
def quick_contact(monkeypatch):
    monkeypatch.setitem(sys.modules, 'CyberGearDriver',
                        NS(RunMode=NS(OPERATION_CONTROL=0)))
    monkeypatch.setattr(ActiveKneeHoming, 'make_sequence',
                        lambda self, sample, now: VerifiedContact(self.config, self.hardware))


@pytest.mark.parametrize('leg', range(1, 7))
def test_each_leg_routes_only_its_pair_and_saves_correct_bus(leg, tmp_path, quick_contact):
    hw = FakeHardware()
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=range(1,7))
    hw.owner = worker
    worker.start([leg])
    worker.thread.join(5)
    assert not worker.thread.is_alive() and not worker.faulted, worker.reason
    pair = hip_knee_ids(leg)
    assert {e[0] for e in hw.events} == set(pair)
    assert set(worker.readiness.snapshot()[0]) == set(pair)
    assert all(hw.samples[mid].mode == 0 and hw.samples[mid].runtime_torque_limit == 12 for mid in pair)
    record = json.loads((tmp_path/'homes.json').read_text())['legs'][str(leg)]
    assert record['motor_ids'] == list(pair)
    assert {m['bus'] for m in record['motors']} == {'can1' if leg in (1,5,6) else 'can0'}
    assert record['motors'][0]['config']['direction'] == profiles()[leg].hip_direction
    assert record['motors'][1]['config']['direction'] == profiles()[leg].knee_direction


def test_all_six_legs_home_in_opposite_groups_and_accumulate_readiness(tmp_path, quick_contact):
    hw = FakeHardware()
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=range(1,7))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    assert not worker.thread.is_alive() and not worker.faulted, worker.reason
    assert set(worker.readiness.snapshot()[0]) == set(HIP_KNEE_IDS)
    enables = [mid // 10 for mid, action, value in hw.events if action == 'enable']
    assert set(enables) == set(range(1,7))
    group_index = {leg: index for index, group in enumerate(((1,4),(2,5),(3,6))) for leg in group}
    assert [group_index[leg] for leg in enables] == sorted(group_index[leg] for leg in enables)
    assert 51 not in {mid for mid, _, _ in hw.events}
    assert set(json.loads((tmp_path/'homes.json').read_text())['legs']) == set('123456')


def test_failed_leg_does_not_erase_completed_legs_or_start_later_legs(tmp_path, quick_contact):
    hw = FakeHardware()
    hw.fail_read = 32
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,2,3,4))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    assert not worker.thread.is_alive() and worker.faulted
    assert set(worker.readiness.snapshot()[0]) == {12,13,22,23,42,43}
    assert not any(mid in (52,53,62,63) for mid, _, _ in hw.events)
    assert 'Leg 3' in worker.reason


def test_cancel_keeps_completed_leg_ready_and_stops_before_next_enable(tmp_path, quick_contact):
    hw = FakeHardware()
    hw.cancel_at = 22
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,2,3))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    assert not worker.thread.is_alive() and worker.faulted
    assert set(worker.readiness.snapshot()[0]) == {12,13}
    assert not any(mid >= 22 and action == 'enable' for mid, action, _ in hw.events)
    assert all(s.mode == 0 for s in hw.samples.values())


def test_unconfigured_leg_fails_before_any_motor_writes(tmp_path):
    hw = FakeHardware()
    conf = profiles()
    conf[2] = LegHomingConfig(2, float('nan'), float('nan'), 0, 0)
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=conf, legs=(1,2))
    with pytest.raises(ValueError, match='Leg 2: configure'):
        worker.start()
    assert not hw.events and worker.thread is None


def test_readiness_fault_and_rehome_are_per_leg(tmp_path, quick_contact):
    hw = FakeHardware()
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,2))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    hw.samples[22].faults = ('Over temperature',)
    assert worker.readiness.check(hw)  # leg 1 remains usable
    assert set(worker.readiness.snapshot()[0]) == {12,13}
    assert '22' in worker.readiness.leg_reason(2)
    hw.samples[22].faults = ()
    assert worker.readiness.check(hw)
    assert set(worker.readiness.snapshot()[0]) == {12,13}
    worker.start([2])
    worker.thread.join(5)
    assert set(worker.readiness.snapshot()[0]) == {12,13,22,23}


@pytest.mark.parametrize('leg', range(1,7))
@pytest.mark.parametrize('direction', (-1,1))
def test_feedback_search_handles_each_joint_direction(leg, direction):
    stop = .8
    start = stop-direction*1.
    c = KneeSeekConfig.from_start(leg*10+2, stop, start, direction=direction)
    seq = KneeSeekSequence(c, start, 0.)
    position = previous = reference = start
    torque = 0.
    for tick in range(1,3001):
        now = tick*.02
        position += max(-.004, min(.004, reference-position))
        position = min(position, stop) if direction == 1 else max(position, stop)
        velocity = (position-previous)/.02
        previous = position
        command = seq.update(position, velocity, torque, now, now)
        reference = command.position
        torque = max(-2.625, min(2.625, command.kp*(reference-position)-command.kd*velocity))
        if seq.done:break
    assert seq.done
    assert seq.result()['home_position_rad'] == pytest.approx(stop)
    assert seq.result()['bus'] == ('can1' if leg in (1,5,6) else 'can0')


def test_all_leg_ros_profile_and_service_dispatch_without_can(monkeypatch, tmp_path, quick_contact):
    rclpy = pytest.importorskip('rclpy')
    from pathlib import Path
    import tocatl_lowlevel.hardware as transport
    from tocatl_lowlevel.motor_controller import MotorController
    from std_srvs.srv import Trigger
    hw = FakeHardware()
    requested = []
    def fake_transport(*args, **kwargs):
        requested.extend(kwargs['motor_ids'])
        return hw
    monkeypatch.setattr(transport, 'HardwareTelemetry', fake_transport)
    profile = Path(__file__).resolve().parents[2]/'tocatl_bringup/config/homing-hip-knee.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(profile),
                      '-p', 'allow_homing:=true', '-p', 'homing.leg_2.knee_direction:=0',
                      '-p', f'homing.output_path:={tmp_path}/homes.json'])
    node = None
    try:
        node = MotorController()
        hw.owner = node.homing
        assert requested == list(HIP_KNEE_IDS)
        assert {s.srv_name.strip('/') for s in node.leg_homing_services} == {
            f'motors/leg_{leg}/home' for leg in range(1,7)}
        rejected = node.leg_homing_services[1].callback(Trigger.Request(), Trigger.Response())
        assert not rejected.success and 'Leg 2: configure' in rejected.message
        assert not hw.events
        # Match the configured, previously measured leg-1 reference before simulating it.
        hw.samples[12].position = node.homing.profiles[1].hip_stop
        hw.samples[13].position = node.homing.profiles[1].knee_stop
        accepted = node.leg_homing_services[0].callback(Trigger.Request(), Trigger.Response())
        assert accepted.success
        node.homing.thread.join(5)
        assert not node.homing.faulted, node.homing.reason
        assert set(node.readiness.snapshot()[0]) == {12,13}
        assert {e[0] for e in hw.events} == {12,13}
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_local_homing_fault_publishes_other_completed_legs_as_ready(tmp_path, quick_contact):
    pytest.importorskip('rclpy')
    from tocatl_interfaces.msg import RobotState
    from tocatl_lowlevel.motor_controller import MotorController
    hw = FakeHardware()
    hw.fail_read = 32
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,2,3))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    state = RobotState()
    state.safety_state, state.status_reason = RobotState.FAULT, worker.reason
    node = NS(homing=worker, motion=NS(status=lambda: (False,False,'')),
              hardware=hw, readiness=worker.readiness)
    MotorController.add_leg_status(node, state)
    assert [l.leg_id for l in state.legs if l.ready] == [1,2]
    assert 'Leg 3' in state.legs[2].status_reason
    assert state.legs[0].status_reason != state.legs[2].status_reason


def test_stale_pair_feedback_does_not_remove_other_legs(tmp_path, quick_contact):
    hw = FakeHardware()
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,2))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    original = hw.snapshot
    def stale_leg2():
        samples, error = original()
        samples[22] = replace(samples[22], received=time.monotonic()-2)
        return samples, error
    hw.snapshot = stale_leg2
    assert worker.readiness.check(hw)
    assert set(worker.readiness.snapshot()[0]) == {12,13}
    worker.readiness.require(hw, [NS(motor_id=12), NS(motor_id=13)])
    with pytest.raises(ValueError, match='not homed'):
        worker.readiness.require(hw, [NS(motor_id=22), NS(motor_id=23)])


def test_global_transport_failure_clears_all_readiness(tmp_path, quick_contact):
    hw = FakeHardware()
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,2))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    original = hw.snapshot
    hw.snapshot = lambda: (original()[0], 'CAN transport unavailable')
    assert not worker.readiness.check(hw)
    assert not worker.readiness.snapshot()[0]


def test_missing_pair_driver_is_rejected_before_writes(tmp_path):
    hw = FakeHardware()
    hw.drivers = [d for d in hw.drivers if d.motor_id != 23]
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,2))
    with pytest.raises(ValueError, match='Leg 2: both motors'):
        worker.start([2])
    assert not hw.events


def test_profile_identity_must_match_selected_leg(tmp_path):
    conf = profiles()
    conf[2] = conf[3]
    with pytest.raises(ValueError, match='mapping key'):
        HipKneeHoming(None, tmp_path/'homes.json', profiles=conf, legs=(1,2))


def test_knee_stage_uses_relaxed_hip_pose_without_old_loaded_position_gate(tmp_path, quick_contact, monkeypatch):
    hw = FakeHardware()
    original = FakeDriver.stop
    def stop_with_relaxation(driver):
        original(driver)
        if driver.motor_id == 12 and hw.owner.child.config.motor_id == 12:
            hw.samples[12].position += .08
    monkeypatch.setattr(FakeDriver, 'stop', stop_with_relaxation)
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=(1,))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    assert not worker.thread.is_alive() and not worker.faulted, worker.reason
    assert set(worker.readiness.snapshot()[0]) == {12,13}
    record = json.loads((tmp_path/'homes.json').read_text())['legs']['1']
    assert record['motors'][1]['hip_reference_rad'] == pytest.approx(profiles()[1].hip_stop+.08)


@pytest.mark.parametrize('group', ((1,4), (2,5), (3,6)))
def test_opposite_legs_actually_overlap_and_share_polling(group, tmp_path, quick_contact, monkeypatch):
    barrier = threading.Barrier(2)
    hw = FakeHardware()
    class OverlapContact(VerifiedContact):
        def update(self, *args):
            if self.config.motor_id % 10 == 2 and self.calls == 1:
                barrier.wait(timeout=2)
                active = {mid//10 for mid,s in hw.samples.items() if s.mode == 2}
                assert active == set(group)
                assert set(hw.priority_motor) == {mid for leg in group for mid in hip_knee_ids(leg)}
                barrier.wait(timeout=2)
            return super().update(*args)
    monkeypatch.setattr(ActiveKneeHoming, 'make_sequence',
                        lambda self, sample, now: OverlapContact(self.config, hw))
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=group)
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    assert not worker.thread.is_alive() and not worker.faulted, worker.reason
    assert set(worker.readiness.snapshot()[0]) == {mid for leg in group for mid in hip_knee_ids(leg)}
    assert hw.priority_motor is None
    assert all(s.mode == 0 for s in hw.samples.values())
    assert all(value == 1.5 for mid, action, value in hw.events if action == 'limit' and value != 12)


def test_parallel_failure_cancels_partner_and_never_starts_next_group(tmp_path, quick_contact, monkeypatch):
    barrier = threading.Barrier(2)
    hw = FakeHardware()
    class FailingContact(VerifiedContact):
        def update(self, *args):
            if self.calls == 1 and self.config.motor_id % 10 == 2:
                barrier.wait(timeout=2)
                if self.config.motor_id == 12:
                    raise ValueError('Injected parallel contact failure')
                time.sleep(.05)
            return super().update(*args)
    monkeypatch.setattr(ActiveKneeHoming, 'make_sequence',
                        lambda self, sample, now: FailingContact(self.config, hw))
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=range(1,7))
    hw.owner = worker
    worker.start()
    worker.thread.join(5)
    assert not worker.thread.is_alive() and worker.faulted
    assert 'Injected parallel contact failure' in worker.reason
    assert not worker.readiness.snapshot()[0]
    assert {mid//10 for mid,action,_ in hw.events if action == 'enable'} == {1,4}
    assert all(s.mode == 0 and s.runtime_torque_limit == 12 for s in hw.samples.values())
    assert hw.priority_motor is None


def test_partial_selections_never_pair_adjacent_legs(tmp_path):
    worker = HipKneeHoming(FakeHardware(), tmp_path/'homes.json', profiles=profiles(), legs=(5,1,2,3))
    assert worker.groups() == [(5,2), (1,), (3,)]


def test_stop_recovery_cancels_opposite_leg_before_ack_recovers(tmp_path, quick_contact, monkeypatch):
    hw = FakeHardware()
    barrier = threading.Barrier(2)
    retrying = threading.Event()
    release_stop = threading.Event()
    partner_stopped = threading.Event()
    class FaultContact(VerifiedContact):
        def update(self, *args):
            if self.config.motor_id % 10 == 2 and self.calls == 1:
                barrier.wait(timeout=2)
                if self.config.motor_id == 12:
                    raise ValueError('Contact sensor failure')
                retrying.wait(2)
                # Keep the partner worker active until supervisor cancellation.
                while not hw.owner.cancelled.is_set():
                    time.sleep(.005)
            return super().update(*args)
    monkeypatch.setattr(ActiveKneeHoming, 'make_sequence',
                        lambda self, sample, now: FaultContact(self.config, hw))
    driver12 = next(d for d in hw.drivers if d.motor_id == 12)
    driver42 = next(d for d in hw.drivers if d.motor_id == 42)
    stop12, stop42 = driver12.stop, driver42.stop
    def unavailable_stop():
        if not release_stop.is_set():
            retrying.set()
            raise ValueError('Injected stop transmission failure')
        stop12()
    def observed_stop():
        stop42()
        partner_stopped.set()
    driver12.stop, driver42.stop = unavailable_stop, observed_stop
    worker = HipKneeHoming(hw, tmp_path/'homes.json', profiles=profiles(), legs=range(1,7))
    hw.owner = worker
    worker.start()
    try:
        assert retrying.wait(2)
        assert partner_stopped.wait(2)
        assert worker.thread.is_alive()
        assert worker.status()[1]
        assert set(hw.priority_motor) == {12,13,42,43}
        with pytest.raises(ValueError, match='fault recovery'):
            worker.start()
    finally:
        release_stop.set()
        worker.thread.join(5)
    assert not worker.thread.is_alive() and worker.faulted
    assert 'Contact sensor failure' in worker.reason
    assert all(s.mode == 0 and s.runtime_torque_limit == 12 for s in hw.samples.values())
    assert not worker.readiness.snapshot()[0]

