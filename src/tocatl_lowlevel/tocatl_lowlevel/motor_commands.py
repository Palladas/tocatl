"""Motor-space command validation and explicit hardware actuation lifecycle."""
from dataclasses import dataclass
from math import isfinite
import threading
import time

from tocatl_lowlevel.motor_map import MOTORS

JOINT_NAMES = tuple(f'leg_{m.leg}_{m.joint}' for m in MOTORS)

MOTOR_IDS = tuple(m.motor_id for m in MOTORS)


@dataclass(frozen=True)
class Target:
    motor_id: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    kp: float
    kd: float


class MotorCommands:
    """Validate a whole desired state before replacing it. Thread-safe snapshots."""

    def __init__(self, timeout=1.0, max_age=0.5, clock=time.monotonic,
                 minimum=None, maximum=None, max_velocity=0.21, max_torque=0.5,
                 max_kp=5.0, max_kd=1.0, max_step=0.1, motor_ids=MOTOR_IDS):
        self.motor_ids = tuple(motor_ids)
        if (not self.motor_ids or len(set(self.motor_ids)) != len(self.motor_ids) or
                any(type(mid) is not int or mid not in MOTOR_IDS for mid in self.motor_ids)):
            raise ValueError('motor_ids must be a nonempty unique subset of the motor inventory')
        self.clock = clock
        self.timeout = timeout
        self.max_age = max_age
        position_ids = (MOTOR_IDS if minimum is not None and len(minimum) == len(MOTOR_IDS)
                        else self.motor_ids)
        self.minimum = dict(zip(position_ids, minimum)) if minimum is not None else {}
        self.maximum = dict(zip(position_ids, maximum)) if maximum is not None else {}
        self.envelope_valid = (minimum is not None and maximum is not None and
                               len(minimum) == len(maximum) == len(position_ids) and
                               all(mid in self.minimum and mid in self.maximum and
                                   isfinite(self.minimum[mid]) and isfinite(self.maximum[mid]) and
                                   -12.5 <= self.minimum[mid] < self.maximum[mid] <= 12.5
                                   for mid in self.motor_ids))
        self.bounds = (max_velocity, max_torque, max_kp, max_kd)
        if (not all(isfinite(v) and v > 0 for v in (timeout, max_age, max_step)) or
                not all(isfinite(v) and v >= 0 for v in self.bounds) or
                any(a > b for a, b in zip(self.bounds, (30.0, 12.0, 500.0, 5.0)))):
            raise ValueError('Invalid command timing or CyberGear command limits')
        self.max_step = max_step
        self.lock = threading.Lock()
        self.targets = ()
        self.received = None
        self.last_stamp = 0

    def accept(self, message, ros_now_ns, check=None):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (stamp <= 0 or stamp > ros_now_ns or
                ros_now_ns - stamp > self.max_age * 1e9 or message.header.frame_id):
            raise ValueError('motor_command requires a recent nonfuture stamp and empty frame_id')
        ids = [m.motor_id for m in message.motors]
        if len(ids) != len(self.motor_ids) or set(ids) != set(self.motor_ids):
            raise ValueError(f'motor_command must contain configured motor IDs {self.motor_ids} exactly once')
        targets = []
        for m in message.motors:
            values = (m.position_rad, m.velocity_rad_s, m.torque_nm, m.kp, m.kd)
            if not all(isfinite(v) for v in values):
                raise ValueError('All motor targets must be finite')
            if (abs(m.position_rad) > 12.5 or abs(m.velocity_rad_s) > self.bounds[0] or
                    abs(m.torque_nm) > self.bounds[1] or not 0 <= m.kp <= self.bounds[2] or
                    not 0 <= m.kd <= self.bounds[3]):
                raise ValueError(f'Motor {m.motor_id} target exceeds command limits')
            if self.envelope_valid and not self.minimum[m.motor_id] <= m.position_rad <= self.maximum[m.motor_id]:
                raise ValueError(f'Motor {m.motor_id} target outside motor-space position envelope')
            targets.append(Target(m.motor_id, *values))
        ordered = tuple(sorted(targets, key=lambda t: t.motor_id))
        if check is not None:
            check(ordered)
        with self.lock:
            if stamp <= self.last_stamp:
                raise ValueError('motor_command stamp must increase; replay/out-of-order batch rejected')
            self.targets = ordered
            self.received = self.clock()
            self.last_stamp = stamp
        return ordered

    def snapshot(self):
        with self.lock:
            age = None if self.received is None else max(0.0, self.clock() - self.received)
            return self.targets, age

    def clear(self):
        with self.lock:
            self.targets = ()

    def check_feedback(self, hardware, targets):
        if not self.envelope_valid:
            raise ValueError('Configure measured motor_min_position_rad/motor_max_position_rad limits for selected motors')
        samples, error = hardware.snapshot()
        if error:
            raise ValueError(error)
        now = time.monotonic()
        for target in targets:
            sample = samples[target.motor_id]
            if (sample.received is None or now - sample.received >= hardware.stale_after or
                    sample.fault_received is None or now - sample.fault_received >= hardware.stale_after):
                raise ValueError(f'Motor {target.motor_id} telemetry/fault status unavailable or stale')
            if sample.faults or sample.state_faults:
                raise ValueError(f'Motor {target.motor_id} reports a fault')
            if not isfinite(sample.position) or abs(target.position_rad - sample.position) > self.max_step:
                raise ValueError(f'Motor {target.motor_id} position step exceeds max_position_step_rad')


class HardwareMotion:
    """Single worker owns driver mode/enable/control/stop operations.

    Startup is disarmed. Only explicit enable requests can arm. Faults and command
    timeout latch a disarm and require a new command and enable request.
    """

    def __init__(self, hardware, commands, allowed=False, readiness_check=None):
        self.hardware, self.commands, self.allowed = hardware, commands, allowed
        self.readiness_check = readiness_check
        self.lock = threading.Lock()
        self.requested = False
        self.armed = False
        self.reason = 'Hardware motion disabled' if not allowed else 'Disarmed'
        self.faulted = False
        self.stop_pending = False
        self.quit = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def enable(self, enabled):
        with self.lock:
            if not enabled:
                self.requested = False
                self.reason = 'Operator disarmed'
                self.commands.clear()
                return
            if self.stop_pending:
                raise ValueError('Motor stop is still pending')
            if not self.allowed:
                raise ValueError('Set allow_hardware_motion:=true to permit motor execution')
            targets, age = self.commands.snapshot()
            if not targets or age >= self.commands.timeout:
                raise ValueError('Publish a fresh complete motor_command before enabling')
            if self.readiness_check is not None:
                self.readiness_check(targets)
            self.commands.check_feedback(self.hardware, targets)
            self.requested = True
            self.faulted = False
            self.reason = 'Arm requested'

    def status(self):
        with self.lock:
            return self.armed, self.faulted, self.reason

    def run(self):
        touched = []
        try:
            while not self.quit.is_set():
                with self.lock:
                    requested = self.requested
                try:
                    if requested:
                        targets, age = self.commands.snapshot()
                        if not targets or age >= self.commands.timeout:
                            raise ValueError('Motor command watchdog expired')
                        if self.readiness_check is not None:
                            self.readiness_check(targets)
                        self.commands.check_feedback(self.hardware, targets)
                        by_id = {d.motor_id: d for d in self.hardware.drivers}
                        with self.hardware.tx_lock:
                            if not touched:
                                from CyberGearDriver import RunMode
                                for target in targets:
                                    with self.lock:
                                        if not self.requested or self.quit.is_set():
                                            raise ValueError('Operator disarmed')
                                    if self.commands.snapshot()[1] >= self.commands.timeout:
                                        raise ValueError('Motor command watchdog expired')
                                    driver = by_id[target.motor_id]
                                    touched.append(driver)
                                    driver.mode(RunMode.OPERATION_CONTROL)
                                    driver.control(target.position_rad, target.velocity_rad_s,
                                                   target.torque_nm, target.kp, target.kd)
                                    driver.enable()
                            else:
                                for target in targets:
                                    # Check timeout/disable between driver writes, too.
                                    with self.lock:
                                        if not self.requested:
                                            raise ValueError('Operator disarmed')
                                    if self.commands.snapshot()[1] >= self.commands.timeout:
                                        raise ValueError('Motor command watchdog expired')
                                    by_id[target.motor_id].control(
                                        target.position_rad, target.velocity_rad_s,
                                        target.torque_nm, target.kp, target.kd)
                        with self.lock:
                            self.armed = True
                            self.reason = 'Executing motor-space targets'
                    elif touched:
                        touched = self.stop_motors(touched)
                        with self.lock:
                            self.armed = False
                except Exception as error:
                    touched = self.stop_motors(touched)
                    with self.lock:
                        self.armed = False
                        self.requested = False
                        self.faulted = True
                        self.reason = str(error) + ('; motor stop pending' if touched else '')
                    self.commands.clear()
                self.quit.wait(0.02)
        finally:
            self.stop_motors(touched)
            with self.lock:
                self.armed = False

    def stop_motors(self, drivers):
        errors = []
        remaining = []
        with self.hardware.tx_lock:
            for driver in drivers:
                try:
                    driver.stop()
                except Exception as error:
                    errors.append(f'{driver.motor_id}: {error}')
                    remaining.append(driver)
        with self.lock:
            self.stop_pending = bool(remaining)
        if errors:
            with self.lock:
                self.reason = 'Motor stop failed: ' + '; '.join(errors)
                self.faulted = True
        return remaining

    def close(self):
        self.quit.set()
        self.thread.join()


class MockMotors:
    """Instant position tracking for ROS plumbing tests, not a physics simulator.

    Bounds are mock-only; they must never become hardware calibration limits.
    """

    def __init__(self, timeout_s=1.0, clock=time.monotonic):
        if not isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('command_timeout_s must be positive and finite')
        self.timeout_s = timeout_s
        self.clock = clock
        self.enabled = False
        self.command_received = False
        self.command_timed_out = False
        self.positions = dict.fromkeys(JOINT_NAMES, 0.0)
        self.last_command = clock()

    def enable(self, enabled):
        self.enabled = enabled
        self.command_received = False
        self.command_timed_out = False
        self.last_command = self.clock()

    def command(self, names, positions):
        self.expire()
        if not self.enabled:
            raise ValueError('motors are disabled')
        if not names or len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('provide unique joint names with one position each')
        if any(name not in self.positions for name in names):
            raise ValueError('unknown joint name')
        if any(not isfinite(p) or abs(p) > 3.141593 for p in positions):
            raise ValueError('mock positions must be finite and within +/- pi radians')
        self.positions.update(zip(names, positions))
        self.command_received = True
        self.command_timed_out = False
        self.last_command = self.clock()

    def expire(self):
        if self.enabled and self.clock() - self.last_command >= self.timeout_s:
            self.command_timed_out = True
            self.enabled = False
