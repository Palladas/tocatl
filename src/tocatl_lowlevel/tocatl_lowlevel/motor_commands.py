"""Motor-space command validation and explicit hardware actuation lifecycle."""
from dataclasses import dataclass
from math import isfinite
import threading
import time

from tocatl_lowlevel.motor_map import MOTORS

JOINT_OF = {m.motor_id: m.joint for m in MOTORS}
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
                 max_kp=5.0, max_kd=1.0, max_step=0.1, motor_ids=MOTOR_IDS,
                 home_offsets=None, homes=None):
        """``home_offsets`` ({joint: (low, high)}) bounds each target relative to that motor's
        homed raw position this session, from ``homes()``. Raw encoder positions shift by
        whole rotor turns across power cycles, so fixed raw envelopes cannot be reused."""
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
        self.home_offsets, self.homes = None, homes
        if home_offsets:
            if self.envelope_valid:
                raise ValueError('Configure either absolute or home-relative motor envelopes, not both')
            offsets = {joint: tuple(bounds) for joint, bounds in home_offsets.items()}
            if (homes is None or any(JOINT_OF[mid] not in offsets for mid in self.motor_ids) or
                    any(len(b) != 2 or not all(isfinite(v) for v in b) or not -12.5 <= b[0] < b[1] <= 12.5
                        for b in offsets.values())):
                raise ValueError('Home-relative envelopes need finite [low, high] offsets for every '
                                 'actuated joint type and a session home source')
            self.home_offsets = offsets
            self.envelope_valid = True
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
            if self.envelope_valid:
                low, high = self.envelope(m.motor_id)
                if not low <= m.position_rad <= high:
                    raise ValueError(f'Motor {m.motor_id} target {m.position_rad:.4f} outside motor-space '
                                     f'position envelope [{low:.4f}, {high:.4f}]')
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

    def envelope(self, motor_id):
        """Absolute [low, high] raw bounds for ``motor_id`` right now."""
        if self.home_offsets is None:
            return self.minimum[motor_id], self.maximum[motor_id]
        home = self.homes().get(motor_id)
        if home is None or not isfinite(home):
            raise ValueError(f'Motor {motor_id} is not homed this session; home-relative envelope unavailable')
        low, high = self.home_offsets[JOINT_OF[motor_id]]
        return home + low, home + high

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
            low, high = self.envelope(target.motor_id)
            if not low <= target.position_rad <= high:
                raise ValueError(f'Motor {target.motor_id} target outside motor-space position envelope')
            if not isfinite(sample.position) or abs(target.position_rad - sample.position) > self.max_step:
                raise ValueError(f'Motor {target.motor_id} position step exceeds max_position_step_rad')


class Disarmed(Exception):
    """An operator disarm noticed partway through a command pass: a normal stop, not a fault."""


class HardwareMotion:
    """Single worker owns driver mode/enable/control/stop operations.

    Startup is disarmed. Only explicit enable requests can arm. Faults and command
    timeout latch a disarm and require a new command and enable request.
    """

    def __init__(self, hardware, commands, allowed=False, readiness_check=None, torque_limit=None):
        """``torque_limit`` (Nm): verified volatile drive cap (0x700b) written and read back for
        each motor before its first enable; the original is restored only after disabled
        feedback newer than the stop is seen, otherwise the reduced cap stays in force."""
        if torque_limit is not None and not (isfinite(torque_limit) and 0 < torque_limit <= 12):
            raise ValueError('Motion drive torque limit must be in (0, 12] Nm')
        self.hardware, self.commands, self.allowed = hardware, commands, allowed
        self.readiness_check = readiness_check
        self.torque_limit = torque_limit
        self.original_limits = {}
        self.pending_restore = {}
        self.fault_count = 0  # increments on every new fault, so callers can act once per fault
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
                        if not touched and self.torque_limit is not None:
                            self.apply_torque_cap(targets)
                        with self.hardware.tx_lock:
                            if not touched:
                                from CyberGearDriver import RunMode
                                for target in targets:
                                    with self.lock:
                                        if not self.requested or self.quit.is_set():
                                            raise Disarmed('Operator disarmed')
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
                                            raise Disarmed('Operator disarmed')
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
                            if not self.faulted:
                                self.reason = 'Operator disarmed'
                        self.restore_torque_limits()  # motors that did stop, while others retry
                    else:
                        self.restore_torque_limits()
                except Disarmed:
                    # A disarm arriving while targets are being written (a 12-motor pass
                    # takes ~120 ms) used to be raised as an error and latch a FAULT,
                    # clearing every leg's homing (2026-10-27 15:40). Stop normally;
                    # stop_motors() still faults if a stop fails.
                    touched = self.stop_motors(touched)
                    held = {d.motor_id for d in touched}
                    for motor_id in [m for m in self.original_limits if m not in held]:
                        self.pending_restore[motor_id] = (self.original_limits.pop(motor_id), time.monotonic())
                    with self.lock:
                        self.armed = False
                        self.requested = False
                    self.commands.clear()
                except Exception as error:
                    touched = self.stop_motors(touched)
                    # Capped but never enabled (e.g. a later cap write failed): restore once
                    # disabled feedback arrives. Motors whose stop failed keep the cap.
                    held = {d.motor_id for d in touched}
                    for motor_id in [m for m in self.original_limits if m not in held]:
                        self.pending_restore[motor_id] = (self.original_limits.pop(motor_id), time.monotonic())
                    with self.lock:
                        self.armed = False
                        self.requested = False
                        self.faulted = True
                        self.fault_count += 1
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
        stopped = time.monotonic()
        for driver in drivers:
            if driver not in remaining and driver.motor_id in self.original_limits:
                self.pending_restore[driver.motor_id] = (self.original_limits.pop(driver.motor_id), stopped)
        with self.lock:
            self.stop_pending = bool(remaining)
        if errors:
            with self.lock:
                self.reason = 'Motor stop failed: ' + '; '.join(errors)
                self.faulted = True
                self.fault_count += 1
        return remaining

    def apply_torque_cap(self, targets):
        for target in targets:
            motor_id = target.motor_id
            if motor_id in self.original_limits:
                continue
            # Re-arming before a pending restore: the drive still holds the cap.
            original = (self.pending_restore.pop(motor_id)[0] if motor_id in self.pending_restore
                        else self.hardware.read_runtime_torque_limit(motor_id))
            self.original_limits[motor_id] = original
            self.hardware.set_runtime_torque_limit(motor_id, self.torque_limit)

    def restore_torque_limits(self):
        if not self.pending_restore:
            return
        samples, _ = self.hardware.snapshot()
        for motor_id, (original, stopped) in list(self.pending_restore.items()):
            sample = samples[motor_id]
            if sample.mode != 0 or sample.received is None or sample.received <= stopped:
                continue  # wait for disabled feedback newer than the stop
            try:
                self.hardware.set_runtime_torque_limit(motor_id, original)
                del self.pending_restore[motor_id]
            except Exception as error:
                with self.lock:
                    self.reason = f'Motor {motor_id} torque limit restore pending: {error}'

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
