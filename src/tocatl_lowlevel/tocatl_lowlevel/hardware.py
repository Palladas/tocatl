"""CyberGear telemetry transport with serialized access for optional motion."""
from contextlib import contextmanager
from dataclasses import dataclass
import math
import threading
import time
from types import SimpleNamespace

from tocatl_lowlevel.motor_map import MOTORS, CAN_BITRATE


@dataclass
class Sample:
    stamp: object = None
    received: float | None = None
    fault_received: float | None = None
    position: float = math.nan
    velocity: float = math.nan
    torque: float = math.nan
    temperature: float = math.nan
    mode: int | None = None
    runtime_torque_limit: float = math.nan
    torque_limit_received: float | None = None
    state_faults: tuple = ()
    faults: tuple = ()


# While a homing batch polls its motors at full rate, other motors are polled in the
# background, in rotation. Ready legs must be refreshed well inside stale_after, or
# readiness clears them as "feedback lost" (2026-10-27, Level 4: 14-16 background motors
# at one per sweep aged to ~1.5 s). Polling more motors per sweep starves the homing pair
# (0.25 s guard), so the homing coordinator narrows the rotation to the motors readiness
# checks (``background_motors``), and more than one per sweep is used only if needed.
BACKGROUND_REFRESH_FRACTION = 2 / 3
# Outside homing, a motor whose state arrived within this long (every control command is
# answered with a state frame) only needs its fault status polled. Sweeps are paced to at
# least SWEEP_S so motion commands keep most of the bus; fault status still refreshes
# well inside stale_after (2026-10-27: 18-motor stands starved fault-status polling).
STATE_REPLY_FRESH_S = 0.3
SWEEP_S = 0.6
HOST_ID = 0
# Startup get-ID wiring check: frame spacing keeps a burst inside the 10-frame TX queue,
# and replies normally arrive within a few milliseconds.
WIRING_SPACING_S = 0.002
WIRING_WAIT_S = 0.3
# Transport v2 (opt-in, can_transport:=v2): CyberGearDriver sleeps 10 ms after every frame
# (PAUSE_AFTER_SEND), which caps 18 motors at ~5 command passes per second. v2 sends the
# driver's own frames without that sleep. A frame is ~130 us at 1 Mbit/s, so a burst can
# briefly fill a bus's 10-frame TX queue (ENOBUFS): retry for up to TX_RETRY_S, then fail.
# A queue that never drains (no device acknowledging, the 2026-10-25 freeze) still faults.
TRANSPORTS = ('v1', 'v2')
TX_RETRY_S = 0.02
TX_RETRY_SLEEP_S = 0.0005


def fast_motor_class(driver_module):
    """CyberGearMotor whose _send is the driver's own, minus PAUSE_AFTER_SEND. Every frame
    (control, enable, stop, parameter read/write, state request) is built by the driver as
    before; set_zero_position and change_motor_id are never called by this package."""
    base = driver_module.CyberGearMotor

    class FastCyberGearMotor(base):
        def _send(self, command, data=bytearray(8), extended_data=0, log=True):
            self.send_message(SimpleNamespace(
                arbitration_id=(command.value << 24) | (extended_data << 8) | self.motor_id, data=data))

    return FastCyberGearMotor


def is_tx_queue_full(error):
    import errno
    return getattr(error, 'error_code', None) == errno.ENOBUFS or 'No buffer space' in str(error)


class FairLock:
    """First-come, first-served lock. A plain Lock lets a thread that releases and
    immediately re-acquires (a motion pass taken one motor at a time) starve the
    telemetry poll waiting for it. Not reentrant, like threading.Lock.

    ``prioritized()`` jumps the queue: it waits only for the current holder. The homing
    poll uses it, because in FIFO order each of its requests queued behind every homing
    worker's command, and fault status aged past the 0.25 s guard (2026-10-27, pair 1/4;
    reproduced by test_homing_timing)."""

    def __init__(self):
        self._cond = threading.Condition(threading.Lock())
        self._next = 0
        self._serving = 0
        self._held = False
        self._ticket_held = False
        self._priority_waiting = 0

    def acquire(self):
        with self._cond:
            ticket = self._next
            self._next += 1
            while ticket != self._serving or self._held or self._priority_waiting:
                self._cond.wait()
            self._held = self._ticket_held = True
        return True

    def acquire_priority(self):
        with self._cond:
            self._priority_waiting += 1
            while self._held:
                self._cond.wait()
            self._priority_waiting -= 1
            self._held, self._ticket_held = True, False
        return True

    def release(self):
        with self._cond:
            if self._ticket_held:
                self._serving += 1
            self._held = self._ticket_held = False
            self._cond.notify_all()

    def __enter__(self):
        return self.acquire()

    def __exit__(self, *exc):
        self.release()

    @contextmanager
    def prioritized(self):
        self.acquire_priority()
        try:
            yield
        finally:
            self.release()


def background_per_sweep(background_count, sweep_s, refresh_s):
    """How many background motors each priority sweep must poll so that every one is
    refreshed within ``refresh_s``, given the measured sweep duration."""
    if background_count <= 0:
        return 0
    if not math.isfinite(sweep_s) or sweep_s <= 0:
        return 1
    return max(1, min(background_count, math.ceil(background_count * sweep_s / refresh_s)))


def select_drivers(drivers, priority, background_index, per_sweep, background_ids=None):
    """(drivers to poll this sweep, next background index): priority motors first, then
    ``per_sweep`` background motors continuing the rotation. ``background_ids`` (if given)
    limits the rotation to those motors while priority polling is active."""
    if priority is None:
        return list(drivers), background_index
    ids = (priority,) if isinstance(priority, int) else priority
    selected = [d for d in drivers if d.motor_id in ids]
    background = [d for d in drivers if d.motor_id not in ids and
                  (background_ids is None or d.motor_id in background_ids)]
    if not background:
        return selected, background_index
    chosen = [background[(background_index + i) % len(background)] for i in range(per_sweep)]
    return selected + chosen, background_index + per_sweep


class AddressedMotor:
    """Logical inventory identity backed by a different physical CAN address."""
    def __init__(self, logical_motor_id, driver):
        self.motor_id = self.logical_motor_id = logical_motor_id
        self.driver = driver

    def __getattr__(self, name):
        return getattr(self.driver, name)


def validate_address_overrides(overrides):
    overrides = dict(overrides or {})
    if any(mid not in {m.motor_id for m in MOTORS} or type(address) is not int or
           not 1 <= address <= 127 for mid, address in overrides.items()):
        raise ValueError('CAN address overrides require known motors and IDs 1..127')
    for bus in {m.bus for m in MOTORS}:
        addresses = [overrides.get(m.motor_id, m.motor_id) for m in MOTORS if m.bus == bus]
        if len(set(addresses)) != len(addresses):
            raise ValueError(f'CAN address override collides on {bus}')
    return overrides


class HardwareTelemetry:
    def __init__(self, stamp_clock, channels, poll_period=0.1, stale_after=0.5,
                 interface='socketcan', can_module=None, driver_module=None,
                 motor_id_overrides=None, motor_ids=None, verify_wiring=False, transport='v1'):
        if not all(math.isfinite(v) and v > 0 for v in (poll_period, stale_after)):
            raise ValueError('poll_period_s and telemetry_timeout_s must be positive and finite')
        available = {m.motor_id for m in MOTORS if m.bus in channels}
        selected = tuple(sorted(available)) if motor_ids is None else tuple(motor_ids)
        if (not selected or len(set(selected)) != len(selected) or
                any(type(mid) is not int or mid not in available for mid in selected)):
            raise ValueError('telemetry motor IDs must be unique known motors on configured buses')
        self.motor_ids = selected
        if transport not in TRANSPORTS:
            raise ValueError(f'can_transport must be one of {TRANSPORTS}')
        self.transport = transport
        overrides = validate_address_overrides(motor_id_overrides)
        if can_module is None:
            import can as can_module
        if driver_module is None:
            import CyberGearDriver as driver_module
        self.stamp_clock = stamp_clock
        self.stale_after = stale_after
        self.poll_period = poll_period
        self.samples = {m.motor_id: Sample() for m in MOTORS}
        self.lock = threading.Lock()
        self.tx_lock = FairLock()
        self.stop_event = threading.Event()
        self.buses = []
        self.notifiers = []
        self.drivers = []
        self.error = ''
        self.priority_motor = None
        self.background_motors = None  # callable -> motor IDs to rotate during priority polling
        self.id_replies = {}  # logical bus -> motor IDs that answered a get-ID probe on it
        self.wiring = None
        sends = {}
        motor_class = fast_motor_class(driver_module) if transport == 'v2' else driver_module.CyberGearMotor
        try:
            for logical, channel in channels.items():
                bus = can_module.interface.Bus(interface=interface, channel=channel,
                                               bitrate=CAN_BITRATE)
                self.buses.append(bus)
                def send(message, bus=bus, logical=logical):
                    frame = can_module.Message(arbitration_id=message.arbitration_id,
                                               data=message.data, is_extended_id=True)
                    deadline = time.monotonic() + (TX_RETRY_S if self.transport == 'v2' else 0.0)
                    while True:
                        try:
                            bus.send(frame, timeout=0.1)
                            return
                        except Exception as error:
                            if not is_tx_queue_full(error) or time.monotonic() >= deadline:
                                raise
                        time.sleep(TX_RETRY_SLEEP_S)
                sends[logical] = send
                drivers = {}
                for motor in MOTORS:
                    if motor.bus != logical or motor.motor_id not in self.motor_ids:
                        continue
                    address = overrides.get(motor.motor_id, motor.motor_id)
                    driver = motor_class(motor_id=address, send_message=send)
                    drivers[address] = (AddressedMotor(motor.motor_id, driver)
                                        if address != motor.motor_id else driver)
                self.drivers.extend(drivers.values())
                self.notifiers.append(can_module.Notifier(
                    bus, [lambda msg, drivers=drivers, logical=logical: self.receive(msg, drivers, logical)]))
            if verify_wiring:
                self.wiring = self.verify_wiring(sends, overrides)
            self.thread = threading.Thread(target=self.poll, daemon=True)
            self.thread.start()
        except Exception:
            self.close()
            raise

    def receive(self, msg, drivers, logical=None):
        if (not msg.is_extended_id or msg.is_error_frame or msg.is_remote_frame
                or len(msg.data) != 8):
            return
        motor_id = (msg.arbitration_id >> 8) & 0xff
        command = (msg.arbitration_id >> 24) & 0x1f
        if command == 0 and (msg.arbitration_id & 0xff) == 0xFE:  # get-ID reply: 0xFE + unique ID
            with self.lock:
                self.id_replies.setdefault(logical, set()).add(motor_id)
            return
        driver = drivers.get(motor_id)
        if driver is None or (msg.arbitration_id & 0xff) != driver.host_id or command not in (2, 9, 17, 21):
            return
        if command == 9 and int.from_bytes(msg.data[:2], 'little') != 0x3022:
            return
        if command == 17:
            if int.from_bytes(msg.data[:2], 'little') != 0x700b:
                return
            import struct
            with self.lock:
                sample = self.samples[getattr(driver, 'logical_motor_id', motor_id)]
                sample.runtime_torque_limit = struct.unpack_from('<f', msg.data, 4)[0]
                sample.torque_limit_received = time.monotonic()
            return
        with self.lock:
            driver.message_received(msg)
            sample = self.samples[getattr(driver, 'logical_motor_id', motor_id)]
            now = time.monotonic()
            if command == 2:
                sample.stamp = self.stamp_clock()
                sample.received = now
                sample.position = float(driver.state['position'])
                sample.velocity = float(driver.state['velocity'])
                sample.torque = float(driver.state['torque'])
                sample.temperature = float(driver.state['temperature'])
                sample.mode = (msg.arbitration_id >> 22) & 3
                flags = ('Under voltage', 'Over current', 'Over temperature',
                         'Magnetic encoder failure', 'Hall encoder failure',
                         'Encoder not calibrated')
                sample.state_faults = tuple(name for i, name in enumerate(flags)
                                           if msg.arbitration_id & (1 << (16 + i)))
            else:
                sample.fault_received = now
                value = int.from_bytes(msg.data[4:] if command == 9 else msg.data[:4], 'little')
                bits = {0: 'Over temperature', 1: 'Driver chip', 2: 'Under voltage',
                        3: 'Over voltage', 4: 'Phase B over current',
                        5: 'Phase C over current', 7: 'Encoder not calibrated',
                        16: 'Phase A over current'}
                sample.faults = tuple(name for bit, name in bits.items() if value & (1 << bit))
                if value & 0xff00:
                    sample.faults += ('Overload',)

    def poll(self):
        background_index = 0
        sweep_s = math.nan  # measured duration of the last priority sweep
        while not self.stop_event.is_set():
            started = time.monotonic()
            errors = []
            priority = self.priority_motor
            ids = () if priority is None else ((priority,) if isinstance(priority, int) else priority)
            source = self.background_motors
            background_ids = None if source is None or priority is None else set(source())
            waiting = sum(d.motor_id not in ids and (background_ids is None or d.motor_id in background_ids)
                          for d in self.drivers)
            per_sweep = background_per_sweep(waiting, sweep_s, self.stale_after * BACKGROUND_REFRESH_FRACTION)
            drivers, background_index = select_drivers(self.drivers, priority, background_index, per_sweep,
                                                       background_ids)
            for driver in drivers:
                if self.stop_event.is_set():
                    break
                with self.lock:
                    received = self.samples[getattr(driver, 'logical_motor_id', driver.motor_id)].received
                state_fresh = (priority is None and received is not None and
                               time.monotonic() - received < STATE_REPLY_FRESH_S)
                try:
                    with self.tx_lock.prioritized() if priority is not None else self.tx_lock:
                        if not state_fresh:
                            driver.request_motor_state()
                        if hasattr(driver, 'request_parameter'):
                            driver.request_parameter('faultSta')
                        else:
                            driver.request_motor_fault_status()
                except Exception as error:
                    errors.append(f'{driver.motor_id}: {error}')
            with self.lock:
                errors.extend(str(n.exception) for n in self.notifiers if n.exception)
                self.error = '; '.join(errors)
            if priority is not None:
                sweep_s = time.monotonic() - started
            period = min(self.poll_period, 0.02) if priority is not None else max(self.poll_period, SWEEP_S)
            self.stop_event.wait(max(0.005 if priority is not None else 0.0,
                                     period - (time.monotonic() - started)))

    def verify_wiring(self, sends, overrides, wait_s=WIRING_WAIT_S):
        """Before polling, ask every selected motor for its ID (comm type 0, read-only: no
        enable, mode, parameter or zero write) on every configured bus, and check each answers
        on the bus motor_map expects. Linux names CAN adapters in the order it finds them, so a
        reboot or a swapped USB port can move legs to another name (2026-11-02: the only
        adapter came up as can1). Raises if a motor answers only on another bus; motors that
        answer nowhere are reported (unpowered or unwired) and left to the freshness checks."""
        addresses = {mid: overrides.get(mid, mid) for mid in self.motor_ids}
        by_address = {address: mid for mid, address in addresses.items()}
        expected = {m.motor_id: m.bus for m in MOTORS if m.motor_id in self.motor_ids}
        for logical, send in sends.items():
            for motor_id, address in addresses.items():
                try:
                    with self.tx_lock:
                        send(SimpleNamespace(arbitration_id=(0 << 24) | (HOST_ID << 8) | address, data=bytes(8)))
                except Exception as error:
                    raise ValueError(f'{logical}: get-ID probe could not be sent ({error}); if no device '
                                     'acknowledges frames, check motor power, CAN-H/L wiring and termination')
                time.sleep(WIRING_SPACING_S)  # stay inside the 10-frame TX queue
        time.sleep(wait_s)
        with self.lock:
            seen = {bus: {by_address[a] for a in ids if a in by_address} for bus, ids in self.id_replies.items()}
        answered = {mid: sorted(bus for bus, ids in seen.items() if mid in ids) for mid in expected}
        moved = {mid: buses for mid, buses in answered.items() if buses and expected[mid] not in buses}
        if moved:
            detail = ', '.join(f'{mid} on {"/".join(buses)} (expected {expected[mid]})' for mid, buses in sorted(moved.items()))
            raise ValueError(f'CAN wiring/naming mismatch: {detail}. Adapters may have swapped names after a '
                             'reboot: remap with canN_channel parameters (e.g. -p can0_channel:=can1) or fix the udev rule')
        return dict(answered={mid: buses for mid, buses in answered.items() if buses},
                    silent=sorted(mid for mid, buses in answered.items() if not buses))

    def snapshot(self):
        from dataclasses import replace
        with self.lock:
            return {key: replace(value) for key, value in self.samples.items()}, self.error

    def read_runtime_torque_limit(self, motor_id, timeout=0.75):
        """Read only volatile 0x700b; unrelated replies cannot acknowledge it."""
        import struct
        from types import SimpleNamespace
        driver = next(d for d in self.drivers if d.motor_id == motor_id)
        physical = getattr(driver, 'driver', driver).motor_id
        with self.tx_lock:
            sent = time.monotonic()
            driver.send_message(SimpleNamespace(
                arbitration_id=(17 << 24) | (driver.host_id << 8) | physical,
                data=struct.pack('<H', 0x700b) + bytes(6)))
        deadline = sent + timeout
        while time.monotonic() < deadline:
            sample = self.snapshot()[0][motor_id]
            if sample.torque_limit_received is not None and sample.torque_limit_received > sent:
                value = sample.runtime_torque_limit
                if math.isfinite(value) and 0 < value <= 12:
                    return value
                raise ValueError(f'Motor {motor_id} invalid runtime torque limit: {value}')
            if self.stop_event.wait(0.01):
                break
        raise ValueError(f'Motor {motor_id} runtime torque-limit readback unavailable')

    def set_runtime_torque_limit(self, motor_id, limit):
        """Write volatile 0x700b and require a matching readback before enabling."""
        import struct
        from types import SimpleNamespace
        if not math.isfinite(limit) or not 0 < limit <= 12:
            raise ValueError('Runtime torque limit must be finite and in (0, 12] Nm')
        driver = next(d for d in self.drivers if d.motor_id == motor_id)
        physical = getattr(driver, 'driver', driver).motor_id
        with self.tx_lock:
            driver.send_message(SimpleNamespace(
                arbitration_id=(18 << 24) | (driver.host_id << 8) | physical,
                data=struct.pack('<H', 0x700b) + bytes(2) + struct.pack('<f', limit)))
        actual = self.read_runtime_torque_limit(motor_id)
        if abs(actual-limit) > 0.001:
            raise ValueError(f'Motor {motor_id} torque-limit readback {actual} != requested {limit}')
        return actual

    def close(self):
        self.stop_event.set()
        if hasattr(self, 'thread'):
            self.thread.join()
        for notifier in self.notifiers:
            notifier.stop()
        for bus in self.buses:
            bus.shutdown()
