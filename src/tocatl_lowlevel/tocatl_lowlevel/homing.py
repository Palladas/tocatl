"""Bounded, single-motor hard-stop homing. No device zero command is used.

A contact is only a candidate mechanical stop: current/torque cannot distinguish
an obstruction. A measured stop window is therefore mandatory.
"""
from dataclasses import dataclass, asdict
import json
import logging
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import time
from tocatl_lowlevel.motor_map import MOTORS, HIP_KNEE_IDS, hip_knee_ids


@dataclass(frozen=True)
class HomingConfig:
    motor_id: int
    direction: int
    torque_limit: float
    min_position: float
    max_position: float
    expected_stop: float
    stop_window: float = 0.04
    speed: float = 0.03
    overspeed: float = 0.12
    velocity_gain: float = 0.3
    torque_ramp: float = 0.02
    backoff: float = 0.05
    max_travel: float = 0.5
    phase_timeout: float = 20.0
    total_timeout: float = 150.0
    stall_speed: float = 0.015
    stall_motion: float = 0.003
    contact_dwell: float = 1.0
    repeatability: float = 0.01
    freshness: float = 0.25
    max_temperature: float = 45.0
    breakaway_only: bool = False
    baseline_only: bool = False
    breakaway_hard_torque: float = 0.1
    breakaway_sustained_torque: float = 0.0  # zero uses torque_limit + 0.02
    breakaway_staged: bool = False
    breakaway_motion: float = 0.003
    breakaway_return_slow: bool = False
    return_speed: float = 0.01
    trajectory_overspeed: float = 0.06

    def __post_init__(self):
        if self.motor_id not in HIP_KNEE_IDS or self.direction not in (-1, 1):
            raise ValueError('Homing supports hip pitch or knee motors and direction +/-1')
        values = asdict(self)
        if any(not math.isfinite(v) for v in values.values()):
            raise ValueError('Homing values must be finite')
        positive = [v for k, v in values.items() if k not in
                    ('motor_id', 'direction', 'min_position', 'max_position', 'expected_stop', 'breakaway_only', 'baseline_only', 'breakaway_sustained_torque', 'breakaway_staged', 'breakaway_return_slow')]
        if any(v <= 0 for v in positive):
            raise ValueError('Homing limits must be positive')
        if not (-12.5 <= self.min_position < self.expected_stop < self.max_position <= 12.5):
            raise ValueError('Expected stop must be inside an explicit raw position envelope')
        if not (self.stall_speed < self.speed < self.overspeed <= 0.21):
            raise ValueError('Require stall_speed < speed < overspeed <= 0.21 rad/s')
        command_ceiling = 5.0 if self.breakaway_staged else 0.5
        if self.torque_limit > command_ceiling:
            raise ValueError(f'Homing torque cap must be <= {command_ceiling} Nm (not a mechanical safety rating)')
        if self.breakaway_staged and (not self.breakaway_only or self.baseline_only):
            raise ValueError('Staged torque requires away-only breakaway mode')
        ceiling = 5.5 if self.breakaway_staged else 0.3
        if not 0 < self.breakaway_hard_torque <= ceiling:
            raise ValueError(f'Breakaway immediate torque cutoff must be <= {ceiling} Nm')
        if not 0 <= self.breakaway_sustained_torque <= self.breakaway_hard_torque:
            raise ValueError('Breakaway sustained cutoff must be between zero and immediate cutoff')
        if self.breakaway_only:
            travel_bound = 0.12 if self.breakaway_return_slow else 0.02
            if not 0 < self.breakaway_motion < min(self.max_travel, travel_bound):
                raise ValueError('Breakaway movement target must be below the travel bound')
            sustained = self.breakaway_sustained_torque or self.torque_limit+0.02
            if not self.torque_limit < sustained <= self.breakaway_hard_torque:
                raise ValueError('Require command cap < sustained cutoff <= immediate cutoff')
        if self.breakaway_return_slow:
            if not self.breakaway_only or not self.breakaway_staged or self.baseline_only:
                raise ValueError('Slow return requires staged breakaway mode')
            if self.breakaway_motion > math.radians(5.1) or self.return_speed > 0.02:
                raise ValueError('Slow return supports <=5.1 degrees and <=0.02 rad/s')
            if not self.return_speed < self.trajectory_overspeed <= self.overspeed:
                raise ValueError('Require return speed < trajectory cutoff <= outer speed cutoff')
        if not (self.repeatability < self.stop_window < self.backoff < self.max_travel):
            raise ValueError('Require repeatability < stop_window < backoff < max_travel')

    @property
    def bus(self):
        return next(m.bus for m in MOTORS if m.motor_id == self.motor_id)


class SlowReturn:
    """Gravity-assisted return using bounded supporting torque only.

    A host position/velocity servo follows a descending position reference;
    it never commands torque toward the physical stop. At the starting pose,
    unload gradually before requesting disable. Faults still require a stop.
    """
    def __init__(self, config, origin, position, support, sample_time, lifting=False):
        self.c = config
        self.origin = origin
        self.away = -config.direction
        self.reference = self.away*(position-origin)
        self.support = abs(support)
        self.command = abs(support)
        self.previous_position = position
        self.previous_time = self.started = sample_time
        self.integral = 0.0
        self.filtered_velocity = 0.0
        self.settled_since = None
        self.lifting = lifting
        self.phase = 'LIFT_SLOWLY' if lifting else 'RETURN_SLOWLY'
        self.done = False
        self.final_position = None

    def update(self, position, sample_time):
        dt = sample_time-self.previous_time
        if dt <= 0:
            return None
        if sample_time-self.started > 60:
            raise ValueError('Slow lift timeout' if self.lifting else 'Slow return timeout')
        height = self.away*(position-self.origin)
        velocity = self.away*(position-self.previous_position)/dt
        self.previous_position, self.previous_time = position, sample_time
        if abs(velocity) > self.c.trajectory_overspeed:
            raise ValueError('Slow lift overspeed' if self.lifting else 'Slow return overspeed')
        if height < -0.005:
            raise ValueError('Slow return passed starting pose')
        self.filtered_velocity += min(1.0, dt/0.05)*(velocity-self.filtered_velocity)
        if self.phase == 'RETURN_UNLOAD':
            if abs(height) > 0.005:
                raise ValueError('Hip drifted while unloading at starting pose')
            self.command = max(0.0, self.command-0.2*dt)
            if self.command == 0:
                self.done = True
                self.final_position = position
                self.phase = 'DONE'
            return self.away*self.command
        if self.lifting:
            self.reference = min(self.c.breakaway_motion, self.reference+self.c.return_speed*dt)
        else:
            self.reference = max(0.0, self.reference-self.c.return_speed*dt)
        error = self.reference-height
        if abs(error) > 0.04:
            raise ValueError('Slow return tracking error')
        desired_velocity = (self.c.return_speed if self.reference < self.c.breakaway_motion else 0.0
                            ) if self.lifting else (-self.c.return_speed if self.reference > 0 else 0.0)
        self.integral = max(-self.c.torque_limit, min(self.c.torque_limit,
                            self.integral+5.0*error*dt))
        desired = self.support+5.0*error+1.5*(desired_velocity-self.filtered_velocity)+self.integral
        desired = max(0.0, min(self.c.torque_limit, desired))
        self.command += max(-0.2*dt, min(0.2*dt, desired-self.command))
        if self.lifting:
            if (self.reference == self.c.breakaway_motion and
                    height >= self.c.breakaway_motion-0.001 and
                    abs(self.filtered_velocity) <= 0.01):
                self.done = True
                self.final_position = position
                self.phase = 'LIFT_COMPLETE'
            return self.away*self.command
        settled = (self.reference == 0 and abs(height) <= 0.002 and
                   abs(self.filtered_velocity) <= 0.01)
        if settled:
            if self.settled_since is None:
                self.settled_since = sample_time
            elif sample_time-self.settled_since >= 0.5:
                self.phase = 'RETURN_UNLOAD'
        else:
            self.settled_since = None
        return self.away*self.command


class HomingSequence:
    """Pure feedback-driven state machine, advancing only on new samples.

    First move away to verify breakaway, then make three independent contacts.
    The host velocity servo sends torque only, clamped to torque_limit. The
    velocity limit is an abort threshold, not an independent drive speed limit.
    """
    def __init__(self, config, position, now):
        self.c = config
        self.phase = 'BREAKAWAY'
        self.started = self.phase_started = now
        self.origin = self.phase_origin = position
        self.last_time = None
        self.contacts = []
        self.contact_since = None
        self.contact_positions = []
        self.breakaway_torque = None
        self.done = False
        self.integral = 0.0
        self.last_command = 0.0
        self.breakaway_position = None
        self.breakaway_measured_torque = None
        self.baseline_samples = []
        self.torque_excess_since = None
        self.last_position = None
        self.return_control = None
        self.lift_control = None

    def transition(self, phase, position, now):
        self.phase = phase
        self.phase_started = now
        self.phase_origin = position
        self.contact_since = None
        self.contact_positions = []
        self.integral = 0.0

    def update(self, position, velocity, measured_torque, sample_time, now):
        c = self.c
        if not all(math.isfinite(v) for v in (position, velocity, measured_torque, sample_time, now)):
            raise ValueError('Nonfinite homing feedback')
        if not 0 <= now - sample_time < c.freshness:
            raise ValueError('Homing feedback stale or future-dated')
        if (now-self.started > c.total_timeout or
                (self.return_control is None and now-self.phase_started > c.phase_timeout)):
            raise ValueError('Homing timeout; no automatic torque increase')
        if not c.min_position <= position <= c.max_position:
            raise ValueError('Homing raw position envelope exceeded')
        breakaway_bound = 0.12 if c.breakaway_return_slow else 0.02
        travel_limit = min(c.max_travel, breakaway_bound) if c.breakaway_only else c.max_travel
        if abs(position - self.origin) > travel_limit or abs(position-self.phase_origin) > travel_limit:
            raise ValueError('Homing travel exceeded')
        if not c.breakaway_only and abs(velocity) > c.overspeed:
            raise ValueError('Homing overspeed')
        hard_limit = (0.1 if c.baseline_only else c.breakaway_hard_torque
                      if c.breakaway_only else c.torque_limit+0.02)
        if abs(measured_torque) > hard_limit:
            raise ValueError('Measured torque exceeded homing limit')
        if self.last_time is not None and sample_time <= self.last_time:
            return None  # cached feedback never counts as another contact sample
        dt = 0.0 if self.last_time is None else sample_time-self.last_time
        if c.breakaway_only and dt > 0 and self.last_position is not None:
            if abs(position-self.last_position)/dt > c.overspeed:
                raise ValueError('Breakaway encoder-derived overspeed')
        self.last_time = sample_time
        self.last_position = position
        if c.breakaway_only and not c.baseline_only:
            if abs(measured_torque) > (c.breakaway_sustained_torque or c.torque_limit+0.02):
                if self.torque_excess_since is None:
                    self.torque_excess_since = sample_time
                elif sample_time-self.torque_excess_since >= 0.15:
                    raise ValueError('Sustained measured torque exceeded breakaway limit')
            else:
                self.torque_excess_since = None
        if c.baseline_only:
            self.phase = 'ZERO_TORQUE_BASELINE'
            if abs(position-self.origin) >= c.stall_motion:
                raise ValueError('Movement during zero-torque baseline')
            self.baseline_samples.append(dict(time=sample_time, position=position,
                                              velocity=velocity, torque=measured_torque))
            if now-self.started >= 2.0 and len(self.baseline_samples) >= 10:
                self.done, self.phase = True, 'DONE'
            return 0.0
        if self.return_control is not None:
            torque = self.return_control.update(position, sample_time)
            self.phase = self.return_control.phase
            self.done = self.return_control.done
            if torque is not None:
                self.last_command = torque
            return torque
        if self.lift_control is not None:
            torque = self.lift_control.update(position, sample_time)
            self.phase = self.lift_control.phase
            if torque is not None:
                self.last_command = torque
            if self.lift_control.done:
                self.breakaway_torque = abs(self.last_command)
                self.breakaway_measured_torque = measured_torque
                self.breakaway_position = position
                self.return_control = SlowReturn(c, self.origin, position, self.last_command, sample_time)
                self.phase = self.return_control.phase
            return torque
        away = -c.direction * (position - self.phase_origin)
        if (c.breakaway_return_slow and c.breakaway_motion > 0.02 and c.stall_motion <= away < c.breakaway_motion):
            self.lift_control = SlowReturn(c, self.origin, position, self.last_command, sample_time,
                                           lifting=True)
            self.phase = self.lift_control.phase
            return self.last_command
        if c.breakaway_only and away >= c.breakaway_motion:
            self.breakaway_torque = abs(self.last_command)
            self.breakaway_measured_torque = measured_torque
            self.breakaway_position = position
            if c.breakaway_return_slow:
                self.return_control = SlowReturn(c, self.origin, position,
                                                 self.last_command, sample_time)
                self.phase = self.return_control.phase
                return self.last_command  # keep supporting the leg at turnaround
            self.done, self.phase = True, 'DONE'
            return 0.0
        if c.breakaway_only:
            if away < -c.stall_motion:
                raise ValueError('Breakaway moved toward the stop')
            elapsed = now-self.phase_started
            magnitude = min(c.torque_limit, c.torque_ramp*elapsed)
            if c.breakaway_staged:
                previous = 0.0
                stages = [v for v in (0.05, 0.1, 0.2, 0.35, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5) if v < c.torque_limit]+[c.torque_limit]
                for stage in stages:
                    ramp_time = (stage-previous)/c.torque_ramp
                    self.phase = f'BREAKAWAY_TO_{stage:.2f}_NM'
                    if elapsed <= ramp_time:
                        magnitude = previous+c.torque_ramp*elapsed
                        break
                    if elapsed <= ramp_time+1.0:
                        magnitude = stage
                        break
                    elapsed -= ramp_time+1.0
                    previous = stage
                else:
                    magnitude = c.torque_limit
            self.last_command = -c.direction*magnitude
            return self.last_command
        if self.phase in ('BREAKAWAY', 'BACKOFF', 'FINAL_BACKOFF') and away >= c.backoff:
            if self.phase == 'FINAL_BACKOFF':
                self.done = True
                self.phase = 'DONE'
                return 0.0
            self.transition('APPROACH', position, now)
        if self.phase == 'DONE':
            return 0.0
        direction = c.direction if self.phase == 'APPROACH' else -c.direction
        # A fixed, explicitly configured cap; the ramp resets for each motion.
        cap = min(c.torque_limit, c.torque_ramp * (now - self.phase_started))
        error = direction*c.speed - velocity
        self.integral = max(-cap, min(cap, self.integral + c.velocity_gain*error*dt))
        torque = max(-cap, min(cap, c.velocity_gain * error + self.integral))
        # Never reverse toward a stop during the away-only commissioning trial.
        if c.breakaway_only:
            torque = direction*max(0.0, direction*torque)
        if self.phase == 'BREAKAWAY' and self.breakaway_torque is None and away > c.stall_motion:
            self.breakaway_torque = abs(torque)
        candidate = (self.phase == 'APPROACH' and
                     abs(position-c.expected_stop) <= c.stop_window and
                     abs(velocity) <= c.stall_speed and
                     direction*measured_torque >= 0.8*c.torque_limit and
                     direction*torque >= 0.8*c.torque_limit)
        # Stalling outside the known stop window is an obstruction, not home.
        stalled = (self.phase == 'APPROACH' and abs(velocity) <= c.stall_speed and
                   direction*torque >= 0.8*c.torque_limit)
        if stalled and abs(position-c.expected_stop) > c.stop_window:
            raise ValueError('Stall outside expected stop window')
        if candidate:
            if self.contact_since is None:
                self.contact_since = sample_time
            self.contact_positions.append(position)
            if max(self.contact_positions)-min(self.contact_positions) > c.stall_motion:
                self.contact_since = sample_time
                self.contact_positions = [position]
            if sample_time-self.contact_since >= c.contact_dwell and len(self.contact_positions) >= 3:
                self.contacts.append(statistics.median(self.contact_positions))
                if max(self.contacts)-min(self.contacts) > c.repeatability:
                    raise ValueError('Contact positions are not repeatable')
                self.transition('FINAL_BACKOFF' if len(self.contacts) == 3 else 'BACKOFF', position, now)
                return 0.0  # release contact before reversing on the next sample
        else:
            self.contact_since = None
            self.contact_positions = []
        self.last_command = torque
        return torque

    def result(self):
        if self.c.baseline_only and self.done:
            return dict(motor_id=self.c.motor_id, bus=self.c.bus, kind='zero_torque_baseline',
                        samples=self.baseline_samples, config=asdict(self.c),
                        completed_unix_s=time.time(), device_zero_written=False)
        if self.c.breakaway_only and self.done:
            return dict(motor_id=self.c.motor_id, bus=self.c.bus, kind='breakaway_test',
                        commanded_torque_at_motion_nm=self.breakaway_torque,
                        measured_torque_at_motion_nm=self.breakaway_measured_torque,
                        initial_position_rad=self.origin, final_position_rad=self.breakaway_position,
                        slow_return_completed=bool(self.return_control and self.return_control.done),
                        returned_position_rad=(self.return_control.final_position if self.return_control else None),
                        config=asdict(self.c), completed_unix_s=time.time(),
                        device_zero_written=False)
        if not self.done or len(self.contacts) != 3:
            raise ValueError('Homing incomplete')
        return dict(motor_id=self.c.motor_id, bus=self.c.bus,
                    home_position_rad=statistics.median(self.contacts),
                    contact_positions_rad=self.contacts,
                    spread_rad=max(self.contacts)-min(self.contacts),
                    breakaway_torque_estimate_nm=self.breakaway_torque,
                    config=asdict(self.c), device_zero_written=False,
                    completed_unix_s=time.time())


def save_result(path, result):
    """Atomically preserve independent joint records; never overwrite a failed run."""
    path = Path(path)
    data = json.loads(path.read_text()) if path.exists() else {'version': 1, 'motors': {}}
    key = {'breakaway_test': 'breakaway_tests', 'zero_torque_baseline': 'baseline_tests'}.get(result.get('kind'), 'motors')
    if 'leg' in result:
        key, identity = 'legs', result['leg']
    else:
        identity = result['motor_id']
    data.setdefault(key, {})[str(identity)] = result
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix='.homing-', suffix='.json')
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class SingleMotorHoming:
    """Worker sharing the controller CAN lock; normal motion must be disabled.

    Only the selected driver receives mode/control/enable/stop commands. Other
    motors must be observably disabled. During knee homing hip drift aborts.
    """
    def __init__(self, hardware, config, output, knee_hip_reference):
        self.hardware, self.config, self.output = hardware, config, output
        self.knee_hip_reference = knee_hip_reference
        self.cancelled = threading.Event()
        self.thread = None
        self.lock = threading.Lock()
        self.reason = 'Homing idle'
        self.active = False
        self.enabled = False
        self.faulted = False
        self.stop_pending = False
        self.feedback_timeout = 2.0
        self.completed_result = None

    def status(self):
        with self.lock:
            return self.enabled, self.faulted, self.reason

    def start(self):
        with self.lock:
            if (self.thread is not None and self.thread.is_alive()) or self.stop_pending:
                raise ValueError('Homing already active or motor stop pending')
            self.completed_result = None
            self.cancelled.clear()
            self.active, self.faulted, self.reason = True, False, 'Homing preflight'
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def cancel(self):
        self.cancelled.set()

    @property
    def pair_ids(self):
        return hip_knee_ids(self.config.motor_id // 10)

    @property
    def hip_id(self):
        return self.pair_ids[0]

    @property
    def knee_id(self):
        return self.pair_ids[1]

    def owned_motor_ids(self):
        return (self.config.motor_id,)

    def priority_motor_ids(self):
        return self.owned_motor_ids()

    def mode_samples(self, samples):
        return samples

    def preload_owned(self, driver, samples):
        self.preload(driver, samples[driver.motor_id])

    def make_sequence(self, sample, now):
        return HomingSequence(self.config, sample.position, now)

    def preload(self, driver, sample):
        driver.control(0.0, 0.0, 0.0, 0.0, 0.0)

    def send_command(self, driver, sample, command):
        driver.control(0.0, 0.0, command, 0.0, 0.0)

    def describe_sequence(self, sequence):
        return f'Homing {self.config.motor_id}: {sequence.phase}; contacts={len(sequence.contacts)}/3'

    def checked_sample(self, initial=False):
        if time.monotonic() >= getattr(self, 'deadline', math.inf):
            raise ValueError('Homing overall timeout')
        samples, error = self.hardware.snapshot()
        if error:
            raise ValueError(error)
        now = time.monotonic()
        c = self.config
        # Fresh mode feedback from all motors prevents enabling alongside another
        # controller. Deployment still requires one exclusive CAN command owner.
        for mid, sample in self.mode_samples(samples).items():
            if sample.received is None or not 0 <= now-sample.received < self.hardware.stale_after:
                raise ValueError(f'Motor {mid} mode feedback unavailable/stale')
            if mid not in self.owned_motor_ids() or initial:
                if sample.mode != 0:
                    raise ValueError(f'Motor {mid} must be disabled')
        s = samples[c.motor_id]
        if (s.fault_received is None or not 0 <= now-s.fault_received < c.freshness or
                s.received is None or not 0 <= now-s.received < c.freshness):
            raise ValueError('Selected motor telemetry/fault status unavailable or stale '
                             f'(state age={None if s.received is None else now-s.received}, '
                             f'fault age={None if s.fault_received is None else now-s.fault_received})')
        if s.faults or s.state_faults:
            raise ValueError('Selected motor reports a fault')
        if not math.isfinite(s.temperature) or s.temperature >= c.max_temperature:
            raise ValueError('Homing temperature unavailable or excessive')
        if not initial and s.mode != 2:
            raise ValueError('Selected motor no longer enabled')
        if c.motor_id % 10 == 3:
            hip = samples[c.motor_id-1]
            if (not math.isfinite(self.knee_hip_reference) or not math.isfinite(hip.position)
                    or abs(hip.position-self.knee_hip_reference) > 0.01):
                raise ValueError('Knee homing requires hip at configured reference within 0.01 rad')
        return s, now

    def run(self):
        driver = next(d for d in self.hardware.drivers if d.motor_id == self.config.motor_id)
        owned = [next(d for d in self.hardware.drivers if d.motor_id == mid)
                 for mid in self.owned_motor_ids()]
        touched = False
        result = None
        failure_reason = None
        try:
            priority_ids = self.priority_motor_ids()
            if not getattr(self, 'external_priority', False):
                self.hardware.priority_motor = priority_ids[0] if len(priority_ids) == 1 else priority_ids
            priority_started = time.monotonic()
            # Allow the in-flight full telemetry sweep to complete and obtain
            # priority feedback before doing any motor writes.
            deadline = time.monotonic()+self.feedback_timeout
            while True:
                if self.cancelled.is_set():
                    raise ValueError('Homing cancelled')
                samples, _ = self.hardware.snapshot()
                now = time.monotonic()
                if all(samples[d.motor_id].received is not None and
                       samples[d.motor_id].fault_received is not None and
                       samples[d.motor_id].received > priority_started and
                       samples[d.motor_id].fault_received > priority_started and
                       0 <= now-samples[d.motor_id].received < self.config.freshness and
                       0 <= now-samples[d.motor_id].fault_received < self.config.freshness
                       for d in owned) and all(
                       samples[mid].received is not None and samples[mid].fault_received is not None and
                       samples[mid].received > priority_started and samples[mid].fault_received > priority_started and
                       0 <= now-samples[mid].received < self.config.freshness and
                       0 <= now-samples[mid].fault_received < self.config.freshness
                       for mid in priority_ids):
                    break
                if now >= deadline:
                    raise ValueError('Fresh telemetry/fault feedback unavailable; no enable sent')
                self.cancelled.wait(0.01)
            s, now = self.checked_sample(initial=True)
            sequence = self.make_sequence(s, now)
            # Validate numerical feedback/envelope before any enable operation.
            sequence.update(s.position, s.velocity, s.torque, s.received, now)
            from CyberGearDriver import RunMode
            with self.hardware.tx_lock:
                if self.cancelled.is_set():
                    raise ValueError('Homing cancelled')
                self.checked_sample(initial=True)
                touched = True
                for motor in owned:
                    motor.mode(RunMode.OPERATION_CONTROL)
                    self.preload_owned(motor, self.hardware.snapshot()[0])
                    motor.enable()
                enabled_at = time.monotonic()
                for motor in self.hardware.drivers:
                    if motor.motor_id not in priority_ids:
                        continue
                    motor.request_motor_state()
                    if hasattr(motor, 'request_parameter'):
                        motor.request_parameter('faultSta')
            # Wait for a new enabled sample, keeping the preloaded torque zero.
            deadline = time.monotonic()+self.config.freshness
            while True:
                if self.cancelled.wait(0.01):
                    raise ValueError('Homing cancelled')
                samples, _ = self.hardware.snapshot()
                selected = samples[self.config.motor_id]
                if all(samples[d.motor_id].received is not None and
                       samples[d.motor_id].received > enabled_at and samples[d.motor_id].mode == 2 and
                       samples[d.motor_id].fault_received is not None and
                       samples[d.motor_id].fault_received > enabled_at for d in owned) and all(
                       samples[mid].received is not None and samples[mid].received > enabled_at and
                       samples[mid].fault_received is not None and samples[mid].fault_received > enabled_at
                       for mid in priority_ids):
                    break
                if time.monotonic() >= deadline:
                    raise ValueError('No fresh enable acknowledgement')
            with self.lock:
                self.enabled = True
            if self.config.baseline_only:
                sequence = HomingSequence(self.config, selected.position, time.monotonic())
            while not self.cancelled.is_set():
                s, now = self.checked_sample()
                torque = sequence.update(s.position, s.velocity, s.torque, s.received, now)
                if torque is not None:
                    with self.hardware.tx_lock:
                        if self.cancelled.is_set():
                            raise ValueError('Homing cancelled')
                        if time.monotonic()-s.received >= self.config.freshness:
                            raise ValueError('Feedback expired waiting for CAN lock')
                        # Zero gains: no hidden impedance torque beyond the cap.
                        self.send_command(driver, s, torque)
                    with self.lock:
                        self.reason = self.describe_sequence(sequence)
                if sequence.done:
                    result = sequence.result()
                    if self.config.motor_id % 10 == 3:
                        result['hip_reference_rad'] = self.knee_hip_reference
                    break
                self.cancelled.wait(0.01)
            if self.cancelled.is_set():
                raise ValueError('Homing cancelled')
        except Exception as error:
            failure_reason = str(error)
            logging.getLogger(__name__).exception('Motor %s sequence failed', self.config.motor_id)
            with self.lock:
                self.faulted, self.reason = True, failure_reason
        finally:
            if touched:
                # Retry stop until successful; no successful record while stop pending.
                while True:
                    try:
                        with self.hardware.tx_lock:
                            stop_errors = []
                            for motor in reversed(owned):
                                try:
                                    motor.stop()
                                except Exception as error:
                                    stop_errors.append(str(error))
                            if stop_errors:
                                raise ValueError("; ".join(stop_errors))
                        sent = time.monotonic()
                        deadline = sent+self.config.freshness
                        while True:
                            samples, _ = self.hardware.snapshot()
                            if all(samples[d.motor_id].received is not None and
                                   samples[d.motor_id].received > sent and
                                   samples[d.motor_id].mode == 0 for d in owned):
                                break
                            if time.monotonic() >= deadline:
                                raise ValueError('No fresh disabled acknowledgement')
                            time.sleep(0.01)
                        with self.lock:
                            self.stop_pending = False
                            self.enabled = False
                            if self.faulted and 'stop pending' in self.reason:
                                self.reason = f'{failure_reason}; disabled now confirmed'
                        break
                    except Exception as error:
                        result = None
                        if failure_reason is None:
                            failure_reason = f'Homing stop failed: {error}'
                        with self.lock:
                            self.stop_pending, self.faulted = True, True
                            self.reason = f'{failure_reason}; stop pending: {error}'
                        time.sleep(0.05)
            if not getattr(self, 'external_priority', False):
                self.hardware.priority_motor = None
            if result is not None:
                try:
                    save_result(self.output, result)
                    with self.lock:
                        self.completed_result = result
                        self.reason = f'Homing complete; saved {self.output}'
                except Exception as error:
                    with self.lock:
                        self.faulted, self.reason = True, f'Homing result not saved: {error}'
            with self.lock:
                self.active = False

    def close(self):
        self.cancel()
        if self.thread is not None:
            self.thread.join()


@dataclass(frozen=True)
class ContactConfig:
    motor_id: int
    expected_stop: float
    minimum: float
    maximum: float
    kp: float = 1.75
    kd: float = 2.0
    direction: int = -1
    lead: float = 1.5
    reference_speed: float = 0.2
    command_torque_limit: float = 2.625
    overspeed: float = 0.5
    effort_limit: float = 3.5
    minimum_contact_effort: float = 0.5
    stop_window: float = 0.03
    settle_motion: float = 0.003
    dwell: float = 0.6
    timeout: float = 20.0
    freshness: float = 0.25
    max_temperature: float = 45.0
    baseline_only = False

    def __post_init__(self):
        if self.motor_id not in HIP_KNEE_IDS or self.direction not in (-1,1):
            raise ValueError('Impedance contact supports hip pitch/knee motors only')
        vals=asdict(self)
        if not all(math.isfinite(v) for v in vals.values()):
            raise ValueError('Contact settings must be finite')
        if any(v<=0 for k,v in vals.items() if k not in
               ('direction','expected_stop','minimum','maximum','release_duration')):
            raise ValueError('Contact gains/limits must be positive')
        if not -12.5<=self.minimum<self.expected_stop<self.maximum<=12.5:
            raise ValueError('Contact requires ordered encoder search bounds around the expected stop')
        target=self.expected_stop+self.direction*self.lead
        if not -12.5<=target<=12.5 or not 0<self.lead<=1.5:
            raise ValueError('Contact virtual target out of range')
        if not 0 < self.command_torque_limit < self.effort_limit:
            raise ValueError('Command torque must be below measured cutoff')
        if self.kp>1.75 or self.kd>2 or self.reference_speed>.2 or self.overspeed>.5:
            raise ValueError('Contact gains/speeds exceed this legacy profile')
        if not self.minimum_contact_effort<self.effort_limit<=3.5:
            raise ValueError('Invalid contact effort limits')
        # Reserve room for the damping term inside the measured effort cutoff.
        if self.kp*self.lead >= self.effort_limit:
            raise ValueError('Position-error effort exceeds contact budget')


@dataclass(frozen=True)
class PDCommand:
    position: float
    kp: float
    kd: float
    release_scale: float = 1.0


class ContactSequence:
    def __init__(self, config, position, now):
        self.c=config
        self.initial=position
        self.started=now
        self.last_time=None
        self.last_position=None
        self.reference=position
        self.phase='RAMP_TO_CONTACT'
        self.positions=[]
        self.since=None
        self.done=False

    def update(self, position, velocity, torque, sample_time, now):
        c=self.c
        if not all(math.isfinite(v) for v in (position,velocity,torque,sample_time,now)):
            raise ValueError('Nonfinite contact feedback')
        if not 0<=now-sample_time<c.freshness:
            raise ValueError('Contact feedback stale/future')
        if now-self.started>c.timeout:
            raise ValueError('Contact timeout')
        if not c.minimum<=position<=c.maximum:
            raise ValueError('Contact actual position outside measured envelope')
        if abs(torque)>c.effort_limit:
            raise ValueError('Contact measured effort exceeded limit')
        if self.last_time is not None and sample_time<=self.last_time:
            return None
        dt=0 if self.last_time is None else sample_time-self.last_time
        if dt>0 and abs(position-self.last_position)/dt>c.overspeed:
            raise ValueError('Contact encoder-derived overspeed')
        self.last_time,self.last_position=sample_time,position
        target=c.expected_stop+c.direction*c.lead
        self.reference+=max(-c.reference_speed*dt,min(c.reference_speed*dt,target-self.reference))
        self.reference=max(position-c.lead,min(position+c.lead,self.reference))
        at_target=abs(self.reference-target)<1e-8
        at_lead=abs(self.reference-position)>=c.lead-1e-8
        if ((at_target or at_lead) and c.direction*torque>=c.minimum_contact_effort and
                abs(position-c.expected_stop)>c.stop_window):
            raise ValueError('Loaded contact outside expected stop window')
        candidate=(at_target and abs(position-c.expected_stop)<=c.stop_window and
                   c.direction*torque>=c.minimum_contact_effort)
        if candidate:
            self.phase='SAMPLING_CONTACT'
            if self.since is None:self.since=sample_time
            self.positions.append(position)
            if max(self.positions)-min(self.positions)>c.settle_motion:
                self.positions=[position];self.since=sample_time
            if sample_time-self.since>=c.dwell and len(self.positions)>=3:
                self.done=True;self.phase='DONE'
        else:
            self.positions=[];self.since=None
            self.phase='RAMP_TO_CONTACT' if not at_target else 'WAITING_FOR_STABLE_CONTACT'
        return PDCommand(self.reference,c.kp,c.kd)

    def result(self):
        if not self.done:raise ValueError('Contact measurement incomplete')
        samples=self.positions[-3:]
        return dict(motor_id=self.c.motor_id,bus=next(m.bus for m in MOTORS if m.motor_id == self.c.motor_id),method='legacy_impedance_contact',
                    home_position_rad=statistics.median(samples),contact_positions_rad=samples,
                    spread_rad=max(samples)-min(samples),independent_reapproaches=False,
                    stable_samples=len(self.positions),device_zero_written=False,
                    config=asdict(self.c),completed_unix_s=time.time())


class ImpedanceContact(SingleMotorHoming):
    def make_sequence(self, sample, now):
        return ContactSequence(self.config,sample.position,now)

    def mode_samples(self, samples):
        pair = self.pair_ids
        for mid, sample in samples.items():
            if mid not in pair and mid not in getattr(self, 'concurrent_motor_ids', ()) and sample.mode == 2:
                raise ValueError(f'Other motor {mid} reports enabled')
        if not set(pair).issubset(samples):
            raise ValueError(f'Leg {self.config.motor_id // 10} pair feedback unavailable')
        return {mid: samples[mid] for mid in pair}

    def preload(self, driver, sample):
        driver.control(sample.position,0.0,0.0,self.config.kp,self.config.kd)

    def send_command(self, driver, sample, command):
        driver.control(command.position,0.0,0.0,command.kp,command.kd)

    def describe_sequence(self, sequence):
        return (f'Homing {self.config.motor_id}: {sequence.phase}; '
                f'stable contact samples={len(sequence.positions)}')


@dataclass(frozen=True)
class KneeSeekConfig(ContactConfig):
    timeout: float = 90.0
    backoff: float = 0.08726646259971647
    repeatability: float = 0.01
    phase_timeout: float = 25.0
    initial_seek_timeout: float = 25.0
    contacts_required: int = 3
    release_duration: float = 0.0

    def __post_init__(self):
        super().__post_init__()
        if not 0 <= self.release_duration <= 5:
            raise ValueError('Release duration must be between zero and five seconds')
        if self.motor_id not in HIP_KNEE_IDS or not 0<self.backoff<=0.1:
            raise ValueError('Endstop search requires a hip pitch or knee motor and backoff <=0.1 rad')
        if type(self.contacts_required) is not int or self.contacts_required not in (1, 3):
            raise ValueError('contacts_required must be 1 for normal homing or 3 for commissioning')
        if not 0<self.repeatability<self.backoff:
            raise ValueError('Invalid active search repeatability')

    @classmethod
    def from_start(cls, motor_id, expected_stop, position, *, verify_repeatability=False, direction=-1):
        """Capture a starting pose without inventing mechanical position limits.

        Normal homing learns the loaded stop; a previous/manual reference is
        comparison metadata. Commissioning can still verify the prior reference.
        """
        if not math.isfinite(position) or not -12.5 <= position <= 12.5:
            raise ValueError('Starting encoder position unavailable or outside protocol range')
        speed = 0.05 if verify_repeatability else 0.2
        backoff, window, lead = math.radians(5), 0.03, 1.5
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('Home direction must be +/-1')
        if verify_repeatability and -direction*(position-expected_stop) < -window:
            raise ValueError('Starting position is beyond the known home stop; verify encoder reference')
        # These are wire-format limits, not a start-derived motion corridor.
        minimum, maximum = -12.5, 12.5
        repeat_timeout = 60.0
        initial_timeout = max(repeat_timeout, abs(position-expected_stop)/speed+lead/speed+10.0)
        return cls(motor_id=motor_id, expected_stop=expected_stop, direction=direction,
                   minimum=minimum, maximum=maximum,
                   reference_speed=speed, minimum_contact_effort=1.0,
                   command_torque_limit=1.5, lead=1.5/1.75, release_duration=3.0,
                   contacts_required=3 if verify_repeatability else 1,
                   phase_timeout=repeat_timeout, initial_seek_timeout=initial_timeout,
                   timeout=(max(6*repeat_timeout, initial_timeout+4*repeat_timeout)
                            if verify_repeatability else 60.0))


class KneeSeekSequence:
    """Seek a stable stop contact, optionally repeating it for commissioning."""
    def __init__(self,config,position,now):
        self.c=config
        self.initial=position
        self.started=self.phase_started=now
        self.phase_origin=position
        self.reference=position
        away = -config.direction*(position-config.expected_stop)
        self.phase = ('SEEK_STOP' if config.contacts_required == 1 or away > config.stop_window
                      else 'BACKOFF')
        self.initial_phase = self.phase
        self.phase_timeout = (config.initial_seek_timeout if self.phase == 'SEEK_STOP'
                              else config.phase_timeout)
        self.last_time=None
        self.last_position=None
        self.positions=[]
        self.contacts=[]
        self.since=None
        self.done=False

    def update(self,position,velocity,torque,sample_time,now):
        c=self.c
        if not all(math.isfinite(v) for v in (position,velocity,torque,sample_time,now)):
            raise ValueError('Nonfinite active search feedback')
        if not 0<=now-sample_time<c.freshness:
            raise ValueError('Active search feedback stale/future')
        if now-self.started>c.timeout or now-self.phase_started>self.phase_timeout:
            raise ValueError('Active search timeout')
        if not c.minimum<=position<=c.maximum:
            raise ValueError(f'Encoder position {position:.6f} outside configured range '
                             f'[{c.minimum:.6f}, {c.maximum:.6f}]')
        if abs(torque)>c.effort_limit:
            raise ValueError('Active search effort limit exceeded')
        if self.last_time is not None and sample_time<=self.last_time:return None
        dt=0 if self.last_time is None else sample_time-self.last_time
        if dt>0 and abs(position-self.last_position)/dt>c.overspeed:
            raise ValueError('Active search encoder overspeed')
        self.last_time,self.last_position=sample_time,position
        if self.phase == 'UNLOADING':
            fraction = min(1.0, (sample_time-self.release_started)/c.release_duration)
            # Remove the loaded spring error first, keeping native damping. Then
            # fade impedance on both drives before requesting confirmed disable.
            spring_scale = max(0.0, 1.0-2.0*fraction)
            gain_scale = min(1.0, 2.0*(1.0-fraction))
            self.reference = position+self.release_error*spring_scale
            if fraction >= 1.0:
                self.done=True;self.phase='DONE'
            return PDCommand(self.reference, c.kp*gain_scale, c.kd*gain_scale, gain_scale)
        if self.phase=='BACKOFF':
            if -c.direction*(position-self.phase_origin)>=c.backoff:
                self.phase='SEEK_STOP'
                self.phase_timeout=c.phase_timeout
                self.phase_started=now
                self.reference=position
                self.positions=[];self.since=None
            else:
                self.reference-=c.direction*c.reference_speed*dt
                self.reference=max(position-c.lead,min(position+c.lead,self.reference))
                return PDCommand(self.reference,c.kp,c.kd)
        self.reference+=c.direction*c.reference_speed*dt
        self.reference=max(position-c.lead,min(position+c.lead,self.reference))
        commanded_inward=c.direction*c.kp*(self.reference-position)
        candidate=(c.direction*torque>=c.minimum_contact_effort and
                   commanded_inward>=c.minimum_contact_effort)
        if candidate:
            if self.since is None:self.since=sample_time
            self.positions.append(position)
            if max(self.positions)-min(self.positions)>c.settle_motion:
                self.positions=[position];self.since=sample_time
            if sample_time-self.since>=c.dwell and len(self.positions)>=3:
                if c.contacts_required > 1 and abs(position-c.expected_stop)>c.stop_window:
                    raise ValueError('Loaded contact outside expected stop window')
                self.contacts.append(statistics.median(self.positions[-3:]))
                if max(self.contacts)-min(self.contacts)>c.repeatability:
                    raise ValueError('Active search contacts are not repeatable')
                if len(self.contacts)==c.contacts_required:
                    if c.release_duration:
                        self.phase='UNLOADING'
                        self.phase_started=now
                        self.release_started=sample_time
                        self.release_error=self.reference-position
                    else:
                        self.done=True;self.phase='DONE'
                else:
                    self.phase='BACKOFF'
                    self.phase_timeout=c.phase_timeout
                    self.phase_origin=position
                    self.phase_started=now
                    self.reference=position
                    self.positions=[];self.since=None
        else:self.positions=[];self.since=None
        return PDCommand(self.reference,c.kp,c.kd)

    def result(self):
        if not self.done:raise ValueError('Active search incomplete')
        return dict(motor_id=self.c.motor_id,bus=next(m.bus for m in MOTORS if m.motor_id == self.c.motor_id),method='active_pd_endstop_search',
                    initial_position_rad=self.initial, initial_phase=self.initial_phase,
                    home_position_rad=statistics.median(self.contacts),
                    previous_reference_rad=self.c.expected_stop,
                    reference_delta_rad=statistics.median(self.contacts)-self.c.expected_stop,
                    reference_within_window=abs(statistics.median(self.contacts)-self.c.expected_stop)<=self.c.stop_window,
                    contact_positions_rad=self.contacts,
                    spread_rad=max(self.contacts)-min(self.contacts),
                    independent_reapproaches=len(self.contacts)>1,
                    contact_verified=True, release_completed=self.c.release_duration > 0,
                    device_zero_written=False,
                    config=asdict(self.c),completed_unix_s=time.time())


class ActiveKneeHoming(ImpedanceContact):
    def make_sequence(self,sample,now):
        return KneeSeekSequence(self.config,sample.position,now)

    def describe_sequence(self,sequence):
        return (f'Homing {self.config.motor_id}: {sequence.phase}; '
                f'contacts={len(sequence.contacts)}/{self.config.contacts_required}; stable samples={len(sequence.positions)}')


class HeldHipKneeHoming(ActiveKneeHoming):
    """Own both selected hip/knee motors and hold the hip at its initial pose."""
    hold_kp = 150.0
    hold_kd = 3.0

    def owned_motor_ids(self):
        return self.pair_ids

    def start(self):
        self._hip_last = None
        super().start()

    def checked_sample(self, initial=False):
        sample, now = super().checked_sample(initial)
        samples, error = self.hardware.snapshot()
        hip = samples[self.hip_id]
        now = time.monotonic()
        c = self.config
        if error:
            raise ValueError(error)
        if (hip.received is None or hip.fault_received is None or
                not 0 <= now-hip.received < c.freshness or
                not 0 <= now-hip.fault_received < c.freshness):
            raise ValueError('Holding hip telemetry/fault feedback stale')
        if hip.faults or hip.state_faults:
            raise ValueError('Holding hip reports a fault')
        if not all(math.isfinite(v) for v in (hip.position, hip.velocity, hip.torque, hip.temperature)):
            raise ValueError('Holding hip feedback nonfinite')
        if hip.temperature >= c.max_temperature or abs(hip.torque) > c.effort_limit:
            raise ValueError('Holding hip temperature/effort limit exceeded')
        if hip.mode != (0 if initial else 2):
            raise ValueError('Holding hip enable state unexpected')
        previous = getattr(self, '_hip_last', None)
        if not initial and previous is not None and hip.received > previous[0]:
            if abs(hip.position-previous[1])/(hip.received-previous[0]) > c.overspeed:
                raise ValueError('Holding hip encoder overspeed')
        self._hip_last = (hip.received, hip.position)
        return sample, now

    def preload_owned(self, driver, samples):
        if driver.motor_id == self.hip_id:
            driver.control(self.knee_hip_reference, 0.0, 0.0, self.hold_kp, self.hold_kd)
        else:
            super().preload_owned(driver, samples)

    def send_command(self, driver, sample, command):
        # Called with the shared CAN lock; recheck both motors after lock contention.
        self.checked_sample()
        hip_driver = next(d for d in self.hardware.drivers if d.motor_id == self.hip_id)
        hip_driver.control(self.knee_hip_reference, 0.0, 0.0, self.hold_kp, self.hold_kd)
        super().send_command(driver, sample, command)

    def describe_sequence(self, sequence):
        return super().describe_sequence(sequence) + '; hip position hold active'

    def make_sequence(self, sample, now):
        sequence = super().make_sequence(sample, now)
        original = sequence.result
        def result():
            saved = original()
            saved['hip_hold'] = dict(motor_id=self.hip_id, position_rad=self.knee_hip_reference,
                                    kp=self.hold_kp, kd=self.hold_kd,
                                    effort_limit_nm=self.config.effort_limit,
                                    deviation_limit_rad=0.01)
            return saved
        sequence.result = result
        return sequence


def create_homing(node, hardware):
    """Build a bounded homing worker from ROS parameters; no bench modes."""
    method = node.declare_parameter('homing.method', 'torque').value
    if method in ('leg1_pair', 'hip_knee'):
        verify = node.declare_parameter('homing.verify_repeatability', False).value
        legs = node.declare_parameter('homing.legs', [1] if method == 'leg1_pair' else [1,2,3,4,5,6]).value
        profiles = {}
        for leg in legs:
            hip_knee_ids(leg)
            prefix = f'homing.leg_{leg}.'
            profiles[leg] = LegHomingConfig(
                leg=leg,
                hip_stop=node.declare_parameter(prefix+'hip_stop_rad', .8028152895399412 if leg == 1 else float('nan')).value,
                knee_stop=node.declare_parameter(prefix+'knee_stop_rad', .7394903486686513 if leg == 1 else float('nan')).value,
                hip_direction=node.declare_parameter(prefix+'hip_direction', -1 if leg == 1 else 0).value,
                knee_direction=node.declare_parameter(prefix+'knee_direction', -1 if leg == 1 else 0).value)
        return HipKneeHoming(hardware,
            node.declare_parameter('homing.output_path', 'hip-knee-homing.json').value,
            verify_repeatability=verify, profiles=profiles, legs=legs,
            timeout_s=node.declare_parameter('homing.timeout_s', 600.0 if verify else 60.0).value)
    if method in ('legacy_impedance', 'active_endstop', 'active_endstop_hip_hold'):
        config_type = ContactConfig if method == 'legacy_impedance' else KneeSeekConfig
        worker_type = (HeldHipKneeHoming if method == 'active_endstop_hip_hold' else
                       ActiveKneeHoming if method == 'active_endstop' else ImpedanceContact)
        contact = config_type(
            motor_id=node.declare_parameter('homing.motor_id', 12).value,
            expected_stop=node.declare_parameter('homing.expected_stop_rad', 0.0).value,
            minimum=node.declare_parameter('homing.min_position_rad', 0.0).value,
            maximum=node.declare_parameter('homing.max_position_rad', 0.0).value,
            kp=node.declare_parameter('homing.kp', 1.75).value,
            kd=node.declare_parameter('homing.kd', 2.0).value)
        worker = worker_type(
            hardware, contact,
            node.declare_parameter('homing.output_path', 'leg1-homing.json').value,
            node.declare_parameter('homing.knee_hip_reference_rad', float('nan')).value)
    elif method == 'torque':
        config = HomingConfig(
            motor_id=node.declare_parameter('homing.motor_id', 32).value,
            direction=node.declare_parameter('homing.direction', -1).value,
            torque_limit=node.declare_parameter('homing.torque_limit_nm', 0.0).value,
            min_position=node.declare_parameter('homing.min_position_rad', 0.0).value,
            max_position=node.declare_parameter('homing.max_position_rad', 0.0).value,
            expected_stop=node.declare_parameter('homing.expected_stop_rad', 0.0).value,
            speed=node.declare_parameter('homing.speed_rad_s', 0.03).value,
            torque_ramp=node.declare_parameter('homing.torque_ramp_nm_s', 0.02).value,
            breakaway_only=node.declare_parameter('homing.breakaway_only', False).value,
            breakaway_staged=node.declare_parameter('homing.breakaway_staged', False).value,
            breakaway_motion=node.declare_parameter('homing.breakaway_motion_rad', 0.003).value,
            breakaway_return_slow=node.declare_parameter('homing.breakaway_return_slow', False).value,
            return_speed=node.declare_parameter('homing.return_speed_rad_s', 0.01).value,
            trajectory_overspeed=node.declare_parameter('homing.trajectory_overspeed_rad_s', 0.06).value,
            baseline_only=node.declare_parameter('homing.baseline_only', False).value,
            breakaway_hard_torque=node.declare_parameter('homing.breakaway_hard_torque_nm', 0.1).value,
            breakaway_sustained_torque=node.declare_parameter('homing.breakaway_sustained_torque_nm', 0.0).value,
            phase_timeout=node.declare_parameter('homing.phase_timeout_s', 20.0).value,
            total_timeout=node.declare_parameter('homing.total_timeout_s', 150.0).value)
        worker = SingleMotorHoming(
            hardware, config,
            node.declare_parameter('homing.output_path', 'leg3-homing.json').value,
            node.declare_parameter('homing.knee_hip_reference_rad', float('nan')).value)
    else:
        raise ValueError(f'Unsupported homing.method: {method}; use torque, legacy_impedance, '
                         'active_endstop, or active_endstop_hip_hold')
    return worker


class LegReadiness:
    """Session-only home references; faults or lost feedback latch unready.

    A saved file is an audit record, never permission to actuate after restart.
    Readiness covers pitch/knee only; yaw has not been commissioned.
    """
    def __init__(self):
        self.lock = threading.RLock()
        self.homes = {}
        self.reason = 'Pitch/knee pair has not completed homing in this session'
        self.reasons = {leg: 'Not homed in this session' for leg in range(1, 7)}

    def invalidate(self, reason, leg_id=None):
        with self.lock:
            legs = range(1, 7) if leg_id is None else (leg_id,)
            for leg in legs:
                for mid in hip_knee_ids(leg):
                    self.homes.pop(mid, None)
                self.reasons[leg] = reason
            self.reason = reason

    def leg_reason(self, leg):
        with self.lock:
            return self.reasons[leg]

    def complete(self, results):
        if len(results) != 2 or len({r['motor_id'] for r in results}) != 2:
            raise ValueError('Readiness requires two distinct motors of one hip/knee pair')
        homes = {}
        for result in results:
            contacts = result.get('contact_positions_rad', [])
            required = result.get('config', {}).get('contacts_required', 3)
            if (required not in (1, 3) or len(contacts) != required or
                    not all(math.isfinite(v) for v in contacts) or
                    max(contacts)-min(contacts) > 0.01 or
                    (required == 3 and not result.get('independent_reapproaches')) or
                    (required == 1 and not result.get('contact_verified'))):
                raise ValueError('Readiness requires a completed verified contact for each motor')
            homes[result['motor_id']] = result['home_position_rad']
        legs = {mid // 10 for mid in homes if mid in HIP_KNEE_IDS}
        if (len(legs) != 1 or set(homes) != set(hip_knee_ids(next(iter(legs)))) or
                not all(math.isfinite(v) for v in homes.values())):
            raise ValueError('Readiness requires both pitch/knee motors of one leg')
        leg = next(iter(legs))
        with self.lock:
            self.homes.update(homes)
            self.reason = 'Pitch/knee homed; yaw excluded; explicit arming required'
            self.reasons[leg] = self.reason

    def check(self, hardware):
        with self.lock:
            if not self.homes:
                return False
            samples, error = hardware.snapshot()
            if error:
                self.invalidate(f'CAN transport fault: {error}; rehome required')
                return False
            now = time.monotonic()
            for leg in {mid // 10 for mid in self.homes}:
                for mid in hip_knee_ids(leg):
                    s = samples[mid]
                    if (s.received is None or s.fault_received is None or
                            not 0 <= now-s.received < hardware.stale_after or
                            not 0 <= now-s.fault_received < hardware.stale_after or
                            s.mode not in (0, 2) or s.faults or s.state_faults or
                            not all(math.isfinite(v) for v in
                                    (s.position, s.velocity, s.torque, s.temperature)) or
                            s.temperature >= 45):
                        self.invalidate(f'Motor {mid} feedback lost or faulted; rehome required', leg)
                        break
            return bool(self.homes)

    def require(self, hardware, targets):
        with self.lock:
            if not self.check(hardware) or any(t.motor_id not in self.homes for t in targets):
                raise ValueError('Commanded motors are not homed and ready in this session')

    def snapshot(self):
        with self.lock:
            return dict(self.homes), self.reason


class PassiveKneeHipHoming(ActiveKneeHoming):
    """Monitor the following knee without imposing a fixed position."""
    def priority_motor_ids(self):
        return self.pair_ids

    def checked_sample(self, initial=False):
        sample, now = super().checked_sample(initial)
        knee = self.hardware.snapshot()[0][self.knee_id]
        now = time.monotonic()
        if (knee.received is None or knee.fault_received is None or
                not 0 <= now-knee.received < self.config.freshness or
                not 0 <= now-knee.fault_received < self.config.freshness or
                knee.mode != (0 if initial or self.knee_id not in self.owned_motor_ids() else 2) or
                knee.faults or knee.state_faults or
                not all(math.isfinite(v) for v in
                        (knee.position, knee.velocity, knee.torque, knee.temperature)) or
                not -12.5 <= knee.position <= 12.5 or
                abs(knee.torque) > self.config.effort_limit or knee.temperature >= 45):
            raise ValueError('Passive knee feedback, mode, or travel limit invalid')
        previous = getattr(self, '_passive_knee_last', None)
        if previous is not None and knee.received > previous[0]:
            speed = abs(knee.position-previous[1])/(knee.received-previous[0])
            if speed > self.config.overspeed:
                raise ValueError(f'Passive knee encoder overspeed: {speed:.3f} rad/s')
        self._passive_knee_last = (knee.received, knee.position)
        return sample, now


@dataclass(frozen=True)
class LegHomingConfig:
    leg: int
    hip_stop: float
    knee_stop: float
    hip_direction: int
    knee_direction: int

    def validate(self):
        hip_knee_ids(self.leg)
        if (not all(math.isfinite(v) for v in (self.hip_stop, self.knee_stop)) or
                any(type(d) is not int or d not in (-1, 1)
                    for d in (self.hip_direction, self.knee_direction))):
            raise ValueError(f'Leg {self.leg}: configure finite hip/knee stop references and directions +/-1')
        for stop, direction in ((self.hip_stop, self.hip_direction), (self.knee_stop, self.knee_direction)):
            if not -12.5 < stop < 12.5 or not -12.5 <= stop+direction*1.5 <= 12.5:
                raise ValueError(f'Leg {self.leg}: stop reference outside encoder range')


class HipKneeHoming:
    """Home hip then knee; repeated contacts are an optional commissioning mode."""
    def __init__(self, hardware, output, *, verify_repeatability=False, timeout_s=None,
                 profiles=None, legs=(1,)):
        if timeout_s is None:
            timeout_s = 600.0 if verify_repeatability else 60.0
        if type(verify_repeatability) is not bool or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Invalid homing repeatability mode or timeout')
        self.leg_ids = tuple(legs)
        if not self.leg_ids or len(set(self.leg_ids)) != len(self.leg_ids):
            raise ValueError('Select unique legs for homing')
        for leg in self.leg_ids:
            hip_knee_ids(leg)
        self.profiles = (dict(profiles) if profiles is not None else
                         {1: LegHomingConfig(1, .8028152895399412, .7394903486686513, -1, -1)})
        if any(profile.leg != leg for leg, profile in self.profiles.items()):
            raise ValueError('Homing profile leg does not match its mapping key')
        self.selected_legs = self.leg_ids
        self.current_leg = None
        self.verify_repeatability, self.timeout_s = verify_repeatability, timeout_s
        self.hardware, self.output = hardware, output
        self.readiness = LegReadiness()
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.thread = None
        self.child = None
        self.children = {}
        self.current_legs = ()
        self.active = False
        self.faulted = False
        self.reason = 'Hip/knee homing idle; yaw excluded'

    def status(self):
        with self.lock:
            if self.active and self.children:
                states = [child.status() for child in self.children.values()]
                return (any(s[0] for s in states), any(s[1] for s in states),
                        '; '.join(s[2] for s in states))
            return False, self.faulted, self.reason

    def start(self, legs=None):
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                for child in list(self.children.values()) or ([self.child] if self.child else []):
                    _, faulted, reason = child.status()
                    if faulted or child.stop_pending:
                        raise ValueError(f'Homing fault recovery still active: {reason}. '
                                         'Stop must be confirmed before retry.')
                raise ValueError('Leg homing already active')
            selected = self.leg_ids if legs is None else tuple(legs)
            if not selected or len(set(selected)) != len(selected) or any(leg not in self.leg_ids for leg in selected):
                raise ValueError('Requested leg is not selected in homing.legs')
            available = {d.motor_id for d in self.hardware.drivers}
            for leg in selected:
                if leg not in self.profiles:
                    raise ValueError(f'Leg {leg}: homing references are not configured')
                self.profiles[leg].validate()
                if not set(hip_knee_ids(leg)).issubset(available):
                    raise ValueError(f'Leg {leg}: both motors must be selected in telemetry_motor_ids')
            self.selected_legs = selected
            self.current_leg = selected[0]
            self.current_legs = selected
            for leg in selected:
                self.readiness.invalidate('Homing started; pair not ready', leg)
            self.cancelled.clear()
            self.child = None
            self.children = {}
            self.reason = 'Homing starting'
            self.active, self.faulted = True, False
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def cancel(self):
        with self.lock:
            self.cancelled.set()
            for leg in self.current_legs or (self.current_leg,):
                self.readiness.invalidate('Homing cancelled/disarmed; rehome required', leg)
            for child in self.children.values():
                child.cancel()

    def groups(self):
        remaining = set(self.selected_legs)
        groups = []
        for leg in self.selected_legs:
            if leg not in remaining:
                continue
            opposite = (leg+2) % 6+1
            group = (leg, opposite) if opposite in remaining else (leg,)
            remaining.difference_update(group)
            groups.append(group)
        return groups

    def run(self):
        try:
            for group in self.groups():
                self.current_legs = group
                self.current_leg = group[0]
                with self.lock:
                    self.children = {}
                    self.child = None
                if self.cancelled.is_set():
                    raise ValueError('Homing cancelled')
                self.concurrent_motor_ids = tuple(mid for leg in group for mid in hip_knee_ids(leg))
                self.hardware.priority_motor = self.concurrent_motor_ids
                errors = []
                def home(leg):
                    try:
                        self._home_leg(leg)
                    except Exception as error:
                        with self.lock:
                            errors.append(str(error))
                        self.cancel()
                threads = [threading.Thread(target=home, args=(leg,), daemon=True) for leg in group]
                for thread in threads:
                    thread.start()
                while any(thread.is_alive() for thread in threads):
                    # A drive can be faulted while its worker is still retrying stop.
                    # Cancel the other leg immediately, without waiting for recovery.
                    with self.lock:
                        failed = any(child.status()[1] for child in self.children.values())
                    if failed:
                        self.cancel()
                    for thread in threads:
                        thread.join(.01)
                self.hardware.priority_motor = None
                if errors:
                    raise ValueError('; '.join(errors))
                if self.cancelled.is_set():
                    raise ValueError('Homing cancelled')
            selected = ', '.join(str(leg) for leg in self.selected_legs)
            with self.lock:
                self.reason = (f'Leg {selected} pitch/knee READY; yaw excluded; motors disabled'
                               if len(self.selected_legs) == 1 else
                               f'Legs {selected} pitch/knee READY; yaw excluded; motors disabled')
        except Exception as error:
            with self.lock:
                self.faulted, self.reason = True, str(error)
                for leg in self.current_legs:
                    self.readiness.invalidate(self.reason, leg)
            logging.getLogger(__name__).exception('Hip/knee homing failed for leg %s', self.current_leg)
        finally:
            with self.lock:
                self.hardware.priority_motor = None
                self.active = False

    def _home_leg(self, leg):
        profile = self.profiles[leg]
        profile.validate()
        hip_id, knee_id = hip_knee_ids(leg)
        pair = (hip_id, knee_id)
        results = []
        original_limits = {}
        deadline = time.monotonic()+self.timeout_s
        try:
            samples, error = self.hardware.snapshot()
            now = time.monotonic()
            for mid in pair:
                sample = samples[mid]
                if (error or sample.mode != 0 or sample.received is None or
                        sample.fault_received is None or sample.faults or sample.state_faults or
                        not 0 <= now-sample.received < self.hardware.stale_after or
                        not 0 <= now-sample.fault_received < self.hardware.stale_after):
                    raise ValueError('Fresh disabled, fault-free pair required before torque-limit writes')
            for mid in pair:
                original_limits[mid] = self.hardware.read_runtime_torque_limit(mid)
                if original_limits[mid] < 1.5:
                    raise ValueError(f'Motor {mid} existing torque limit below homing profile; no increase permitted')
                self.hardware.set_runtime_torque_limit(mid, 1.5)
            stages = ((hip_id, profile.hip_stop, profile.hip_direction),
                      (knee_id, profile.knee_stop, profile.knee_direction))
            for mid, stop, direction in stages:
                initial = self.hardware.snapshot()[0][mid].position
                if time.monotonic() >= deadline:
                    raise ValueError('Homing overall timeout')
                config = KneeSeekConfig.from_start(mid, stop, initial,
                    verify_repeatability=self.verify_repeatability, direction=direction)
                hip_reference = (self.hardware.snapshot()[0][hip_id].position if mid == knee_id
                                 else float('nan'))
                if mid == knee_id and not math.isfinite(hip_reference):
                    raise ValueError('Hip holding reference unavailable')
                worker_type = BoundedHipHoming if mid == hip_id else BoundedKneeHoming
                with self.lock:
                    if self.cancelled.is_set():
                        raise ValueError('Leg homing cancelled')
                    child = worker_type(self.hardware, config,
                        str(Path(self.output).with_suffix(f'.motor-{mid}.json')), hip_reference)
                    self.child = child
                    self.children[leg] = child
                    child.external_priority = True
                    child.concurrent_motor_ids = self.concurrent_motor_ids
                    child.deadline = deadline
                    child.start()
                child.thread.join()
                if child.completed_result is None or child.status()[1]:
                    raise ValueError(child.status()[2])
                if self.cancelled.is_set():
                    raise ValueError('Leg homing cancelled')
                results.append(child.completed_result)
            self.restore_limits(original_limits)
            original_limits.clear()
            # Both child results require fresh disabled acknowledgements and saved results.
            with self.lock:
                if self.cancelled.is_set():
                    raise ValueError('Leg homing cancelled')
                record = dict(leg=leg, scope='pitch_knee', motor_ids=list(pair),
                              mode='commissioning' if self.verify_repeatability else 'normal',
                              motors=results, device_zero_written=False,
                              completed_unix_s=time.time())
                save_result(self.output, record)
                self.readiness.complete(results)
                self.readiness.check(self.hardware)
                if not set(pair).issubset(self.readiness.snapshot()[0]):
                    raise ValueError('Feedback invalid after homing; pair remains unready')
                self.reason = f'Leg {leg} pitch/knee READY; yaw excluded; motors disabled'
        except Exception as error:
            reason = str(error)
            try:
                self.restore_limits(original_limits)
            except Exception as restore_error:
                reason += f'; torque limit restore failed: {restore_error}'
            raise ValueError(f'Leg {leg}: {reason}') from error

    def restore_limits(self, originals):
        for mid, limit in originals.items():
            sample = self.hardware.snapshot()[0][mid]
            if (sample.mode != 0 or sample.received is None or
                    not 0 <= time.monotonic()-sample.received < self.hardware.stale_after):
                raise ValueError(f'Motor {mid} not confirmed disabled; torque limit left reduced')
            self.hardware.set_runtime_torque_limit(mid, limit)

    def close(self):
        self.cancel()
        if self.thread is not None:
            self.thread.join()


class NativeImpedanceHoming:
    """Native MIT stiffness/damping, under a verified volatile drive torque limit."""
    def preload_owned(self, driver, samples):
        sample = samples[driver.motor_id]
        if (sample.torque_limit_received is None or
                not math.isfinite(sample.runtime_torque_limit) or
                abs(sample.runtime_torque_limit-self.config.command_torque_limit) > 0.001):
            raise ValueError('Verified drive torque limit required before enable')
        if driver.motor_id == self.config.motor_id:
            kp, kd = self.config.kp, self.config.kd
        elif self.config.motor_id % 10 == 2:
            kp, kd = 0.0, self.config.kd  # following knee: damping only, no fixed angle
        else:
            kp, kd = self.hold_kp, self.hold_kd
        driver.control(sample.position, 0.0, 0.0, kp, kd)

    def send_command(self, driver, sample, command):
        self.checked_sample()
        samples, _ = self.hardware.snapshot()
        for mid in self.owned_motor_ids():
            feedback = samples[mid]
            if (not math.isfinite(feedback.runtime_torque_limit) or
                    abs(feedback.runtime_torque_limit-self.config.command_torque_limit) > 0.001):
                raise ValueError('Drive torque-limit verification lost')
            if mid == self.config.motor_id:
                pd = command
            elif self.config.motor_id % 10 == 2:
                pd = PDCommand(feedback.position, 0.0, self.config.kd*command.release_scale)
            else:
                pd = PDCommand(self.knee_hip_reference, self.hold_kp*command.release_scale,
                               self.hold_kd*command.release_scale)
            # Bound spring demand too; the drive caps stiffness+damping together.
            error = self.config.command_torque_limit/pd.kp if pd.kp else 0.0
            target = max(feedback.position-error, min(feedback.position+error, pd.position))
            motor = next(d for d in self.hardware.drivers if d.motor_id == mid)
            target = max(-12.5, min(12.5, target))
            motor.control(target, 0.0, 0.0, pd.kp, pd.kd)

    def make_sequence(self, sample, now):
        sequence = super().make_sequence(sample, now)
        original = sequence.result
        def result():
            record = original()
            record['control'] = 'native_mit_impedance_runtime_torque_limit'
            record['command_torque_limit_nm'] = self.config.command_torque_limit
            record['torque_limit_register'] = '0x700b'
            return record
        sequence.result = result
        return sequence


class BoundedHipHoming(NativeImpedanceHoming, PassiveKneeHipHoming):
    """Hip seek with a freely following, viscously damped knee."""
    def owned_motor_ids(self):
        return self.pair_ids


class BoundedKneeHoming(NativeImpedanceHoming, HeldHipKneeHoming):
    pass


# Existing leg-1 callers use the same generic implementation.
LegPairHoming = HipKneeHoming
