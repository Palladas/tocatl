"""Batch homing against a bus with real timing: 10 ms per message under one shared lock.

Opt-in (TOCATL_TIMING_TESTS=1): these measure wall-clock margins and are too timing
sensitive to gate every run. On 2026-10-27 the model reproduced the robot's three-leg
(tripod) startup failure (feedback 0.30 s vs 0.35 s on the robot) and showed three legs
at once exceed the 0.25 s fault-status freshness in steady state, so batch homing stays
with opposite pairs. Pairs pass, but close to the limit (worst fault age ~0.24 s).

The other homing tests use feedback that is always fresh, which hides lock contention.
Here a poll thread refreshes samples only as fast as the real driver can send, so a
worker that waits behind other legs for the CAN lock sees its feedback age.
"""
from dataclasses import asdict, replace
from types import SimpleNamespace as NS
import sys
import threading
import time

import pytest

from tocatl_lowlevel.hardware import Sample
from tocatl_lowlevel.homing import ActiveKneeHoming, HipKneeHoming, LegHomingConfig, PDCommand
from tocatl_lowlevel.motor_map import HIP_KNEE_IDS, MOTORS

import os

pytestmark = pytest.mark.skipif(not os.environ.get('TOCATL_TIMING_TESTS'),
                                reason='timing model; set TOCATL_TIMING_TESTS=1 to run')

SEND_S = 0.01  # CyberGearDriver PAUSE_AFTER_SEND


def profiles():
    return {leg: LegHomingConfig(leg, .8, .7, -1, -1) for leg in range(1, 7)}


class TimedBus:
    def __init__(self, poll_period=0.5):
        self.stale_after, self.poll_period = 1.5, poll_period
        self.tx_lock = threading.Lock()
        self.priority_motor = None
        self.stop_event = threading.Event()
        self.samples = {m.motor_id: Sample(mode=0, position=.8 if m.motor_id % 10 == 2 else .7, velocity=0.,
                                           torque=0., temperature=25., runtime_torque_limit=12.)
                        for m in MOTORS}
        self.drivers = [TimedDriver(mid, self) for mid in HIP_KNEE_IDS]
        self.max_age = {}
        self.thread = threading.Thread(target=self.poll, daemon=True)
        self.thread.start()

    def send(self):
        time.sleep(SEND_S)

    def refresh(self, motor_id, fault=False):
        now = time.monotonic()
        sample = self.samples[motor_id]
        if fault:
            sample.fault_received = now
        else:
            sample.received = now

    def poll(self):
        """Same selection as HardwareTelemetry.poll: priority motors plus one background."""
        background_index = 0
        while not self.stop_event.is_set():
            started = time.monotonic()
            priority, drivers = self.priority_motor, self.drivers
            if priority is not None:
                ids = (priority,) if isinstance(priority, int) else priority
                selected = [d for d in drivers if d.motor_id in ids]
                background = [d for d in drivers if d.motor_id not in ids]
                drivers = selected + ([background[background_index % len(background)]] if background else [])
                background_index += 1
            for driver in drivers:
                with self.tx_lock:
                    driver.request_motor_state()
                    driver.request_parameter('faultSta')
            period = min(self.poll_period, 0.02) if priority is not None else self.poll_period
            self.stop_event.wait(max(0.005 if priority is not None else 0.0,
                                     period - (time.monotonic() - started)))

    def snapshot(self):
        return {mid: replace(s) for mid, s in self.samples.items()}, ''

    def read_runtime_torque_limit(self, motor_id):
        with self.tx_lock:
            self.send()
        return self.samples[motor_id].runtime_torque_limit

    def set_runtime_torque_limit(self, motor_id, value):
        with self.tx_lock:
            self.send()
        self.samples[motor_id].runtime_torque_limit = value
        self.samples[motor_id].torque_limit_received = time.monotonic()
        return value

    def close(self):
        self.stop_event.set()
        self.thread.join()


class TimedDriver:
    def __init__(self, motor_id, bus):
        self.motor_id, self.bus = motor_id, bus

    def mode(self, mode):
        self.bus.send()

    # A CyberGear answers control, enable and stop with a state frame, so on the
    # robot every command also refreshes that motor's feedback (not its fault status).
    def control(self, *command):
        self.bus.send()
        self.bus.refresh(self.motor_id)

    def enable(self):
        self.bus.send()
        self.bus.samples[self.motor_id].mode = 2
        self.bus.refresh(self.motor_id)

    def stop(self):
        self.bus.send()
        self.bus.samples[self.motor_id].mode = 0
        self.bus.refresh(self.motor_id)

    def request_motor_state(self):
        self.bus.send()
        self.bus.refresh(self.motor_id)

    def request_parameter(self, name):
        self.bus.send()
        self.bus.refresh(self.motor_id, fault=True)


class NewSampleContact:
    """Like the real search: acts only on a new sample, done after a few of them."""
    def __init__(self, config):
        self.config, self.done, self.phase = config, False, 'SEEK_STOP'
        self.last, self.count, self.contacts, self.positions = None, 0, [], []

    def update(self, position, velocity, torque, sample_time, now):
        if self.last is not None and sample_time <= self.last:
            return None
        self.last, self.count = sample_time, self.count + 1
        if self.count >= 8:
            self.done, self.phase = True, 'DONE'
            self.contacts = [self.config.expected_stop] * self.config.contacts_required
            self.positions = [self.config.expected_stop] * 3
        return PDCommand(self.config.expected_stop, self.config.kp, self.config.kd)

    def result(self):
        return dict(motor_id=self.config.motor_id, bus=next(m.bus for m in MOTORS if m.motor_id == self.config.motor_id),
                    home_position_rad=self.config.expected_stop, contact_positions_rad=self.contacts,
                    contact_verified=True, independent_reapproaches=False, config=asdict(self.config),
                    device_zero_written=False)


@pytest.fixture
def timed(monkeypatch):
    monkeypatch.setitem(sys.modules, 'CyberGearDriver', NS(RunMode=NS(OPERATION_CONTROL=0)))
    monkeypatch.setattr(ActiveKneeHoming, 'make_sequence', lambda self, sample, now: NewSampleContact(self.config))
    bus = TimedBus()
    yield bus
    bus.close()


@pytest.mark.parametrize('legs', [(1, 4), (2, 5), range(1, 7)])
def test_pair_batches_survive_real_lock_contention(timed, tmp_path, legs):
    worker = HipKneeHoming(timed, tmp_path / 'homes.json', profiles=profiles(), legs=tuple(legs))
    # Start like the dashboard does: from the slow 0.5 s full sweep, so feedback is up to 0.5 s old.
    time.sleep(0.6)
    worker.start()
    worker.thread.join(60)
    assert not worker.thread.is_alive()
    assert not worker.faulted, worker.reason
    assert set(worker.readiness.snapshot()[0]) == {mid for leg in legs for mid in (leg * 10 + 2, leg * 10 + 3)}
    assert all(timed.samples[mid].mode == 0 for mid in HIP_KNEE_IDS)
