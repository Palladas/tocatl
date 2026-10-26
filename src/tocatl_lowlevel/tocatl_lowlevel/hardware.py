"""CyberGear telemetry transport with serialized access for optional motion."""
from dataclasses import dataclass
import math
import threading
import time

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
                 motor_id_overrides=None, motor_ids=None):
        if not all(math.isfinite(v) and v > 0 for v in (poll_period, stale_after)):
            raise ValueError('poll_period_s and telemetry_timeout_s must be positive and finite')
        available = {m.motor_id for m in MOTORS if m.bus in channels}
        selected = tuple(sorted(available)) if motor_ids is None else tuple(motor_ids)
        if (not selected or len(set(selected)) != len(selected) or
                any(type(mid) is not int or mid not in available for mid in selected)):
            raise ValueError('telemetry motor IDs must be unique known motors on configured buses')
        self.motor_ids = selected
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
        self.tx_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.buses = []
        self.notifiers = []
        self.drivers = []
        self.error = ''
        self.priority_motor = None
        try:
            for logical, channel in channels.items():
                bus = can_module.interface.Bus(interface=interface, channel=channel,
                                               bitrate=CAN_BITRATE)
                self.buses.append(bus)
                def send(message, bus=bus):
                    bus.send(can_module.Message(arbitration_id=message.arbitration_id,
                             data=message.data, is_extended_id=True), timeout=0.1)
                drivers = {}
                for motor in MOTORS:
                    if motor.bus != logical or motor.motor_id not in self.motor_ids:
                        continue
                    address = overrides.get(motor.motor_id, motor.motor_id)
                    driver = driver_module.CyberGearMotor(motor_id=address, send_message=send)
                    drivers[address] = (AddressedMotor(motor.motor_id, driver)
                                        if address != motor.motor_id else driver)
                self.drivers.extend(drivers.values())
                self.notifiers.append(can_module.Notifier(
                    bus, [lambda msg, drivers=drivers: self.receive(msg, drivers)]))
            self.thread = threading.Thread(target=self.poll, daemon=True)
            self.thread.start()
        except Exception:
            self.close()
            raise

    def receive(self, msg, drivers):
        if (not msg.is_extended_id or msg.is_error_frame or msg.is_remote_frame
                or len(msg.data) != 8):
            return
        motor_id = (msg.arbitration_id >> 8) & 0xff
        command = (msg.arbitration_id >> 24) & 0x1f
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
        while not self.stop_event.is_set():
            started = time.monotonic()
            errors = []
            priority = self.priority_motor
            drivers = self.drivers
            if priority is not None:
                ids = (priority,) if isinstance(priority, int) else priority
                selected = [d for d in drivers if d.motor_id in ids]
                background = [d for d in drivers if d.motor_id not in ids]
                drivers = selected + ([background[background_index % len(background)]] if background else [])
                background_index += 1
            for driver in drivers:
                if self.stop_event.is_set():
                    break
                try:
                    with self.tx_lock:
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
            period = min(self.poll_period, 0.02) if priority is not None else self.poll_period
            self.stop_event.wait(max(0.005 if priority is not None else 0.0,
                                     period - (time.monotonic() - started)))

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
