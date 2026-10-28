#!/usr/bin/env python3
"""Single-motor CyberGear test bench: CAN ID scan, rotation control and a blocked-output
torque test (end effector pressing on a scale).

Independent of the robot stack: no ROS, no motor_controller, its own port (8766). It talks
CyberGear frames directly over one SocketCAN bus chosen by the operator, and refuses to open
a bus while any robot CAN process runs. Connect through SSH forwarding, never a public port.
Starting the server sends no CAN traffic; only explicit operator actions open a bus.

Never writes device zeros or CAN IDs (no comm type 6 or 7). Host-side references only.
"""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import signal
import struct
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
BUSES = ('can0', 'can1')
BITRATE = 1_000_000
HOST_ID = 0
# Robot motor IDs (leg x 10 + joint). Commanding one needs an explicit acknowledgement.
ROBOT_IDS = tuple(leg * 10 + joint for leg in range(1, 7) for joint in (1, 2, 3))
ROBOT_PROCESSES = ('motor_controller', 'cybergear_telemetry', 'motor_telemetry', 'hold_capture',
                   'tocatl_diagnostics', 'run_spider_art_cue', 'joint_endpoint_probe', 'leg_auto_zero')
RUNS = Path.home() / 'tocatl-runs'
G = 9.80665

# CyberGear operation-control ranges (comm type 1/2 uint16 scaling).
P_RANGE, V_RANGE, T_RANGE, KP_RANGE, KD_RANGE = (-4 * math.pi, 4 * math.pi), (-30.0, 30.0), (-12.0, 12.0), (0.0, 500.0), (0.0, 5.0)
# Parameters (comm type 17 read / 18 write).
RUN_MODE, LIMIT_TORQUE, LIMIT_CUR, IQF, VBUS = 0x7005, 0x700B, 0x7018, 0x701A, 0x701C
STATE_FLAGS = ('Under voltage', 'Over current', 'Over temperature', 'Magnetic encoder failure',
               'Hall encoder failure', 'Encoder not calibrated')
FAULT_BITS = {0: 'Over temperature', 1: 'Driver chip', 2: 'Under voltage', 3: 'Over voltage',
              4: 'Phase B over current', 5: 'Phase C over current', 7: 'Encoder not calibrated',
              16: 'Phase A over current'}

# Bench bounds (the CyberGear's own maxima where it has one).
MAX_TORQUE_NM = 12.0
MAX_JOG_SPEED = 10.0
MAX_KP, MAX_KD = 100.0, 5.0
MAX_HOLD_S = 10.0
MAX_TEMPERATURE_C = 60.0
CONTROL_HZ = 50
FEEDBACK_STALE_S = 0.25
HEARTBEAT_S = 1.5  # page stops polling (tab closed, tunnel down) -> stop the motor


def to_u16(value, lo, hi):
    value = min(hi, max(lo, value))
    return int(round((value - lo) * 65535 / (hi - lo)))


def from_u16(raw, lo, hi):
    return lo + raw * (hi - lo) / 65535


def frame(kind, data=bytes(8), motor_id=0, middle=HOST_ID):
    return (kind << 24) | ((middle & 0xffff) << 8) | (motor_id & 0xff), bytes(data)


def control_frame(motor_id, position, velocity, torque, kp, kd):
    data = struct.pack('>HHHH', to_u16(position, *P_RANGE), to_u16(velocity, *V_RANGE),
                       to_u16(kp, *KP_RANGE), to_u16(kd, *KD_RANGE))
    return frame(1, data, motor_id, to_u16(torque, *T_RANGE))


def enable_frame(motor_id):
    return frame(3, bytes(8), motor_id)


def stop_frame(motor_id, clear_faults=False):
    return frame(4, bytes([1 if clear_faults else 0]) + bytes(7), motor_id)


def get_id_frame(motor_id):
    return frame(0, bytes(8), motor_id)


def read_param_frame(motor_id, index):
    return frame(17, struct.pack('<H', index) + bytes(6), motor_id)


def write_param_frame(motor_id, index, value):
    payload = struct.pack('<B', int(value)) + bytes(3) if index == RUN_MODE else struct.pack('<f', value)
    return frame(18, struct.pack('<H', index) + bytes(2) + payload, motor_id)


def parse(arbitration_id, data):
    """Decode a motor-to-host frame into a dict, or None."""
    if len(data) != 8:
        return None
    kind = (arbitration_id >> 24) & 0x1f
    if kind == 0:
        return dict(kind='id', motor_id=(arbitration_id >> 8) & 0xff, uid=bytes(data).hex())
    motor_id = (arbitration_id >> 8) & 0xff
    if kind == 2:
        position, velocity, torque, temperature = struct.unpack('>HHHH', bytes(data))
        return dict(kind='state', motor_id=motor_id, position=from_u16(position, *P_RANGE),
                    velocity=from_u16(velocity, *V_RANGE), torque=from_u16(torque, *T_RANGE),
                    temperature=temperature / 10, mode=(arbitration_id >> 22) & 3,
                    flags=[n for i, n in enumerate(STATE_FLAGS) if arbitration_id & (1 << (16 + i))])
    if kind == 17:
        index = struct.unpack_from('<H', data, 0)[0]
        value = data[4] if index == RUN_MODE else struct.unpack_from('<f', data, 4)[0]
        return dict(kind='param', motor_id=motor_id, index=index, value=value)
    if kind == 21:
        bits = struct.unpack_from('<I', data, 0)[0]
        faults = [n for b, n in FAULT_BITS.items() if bits & (1 << b)] + (['Overload'] if bits & 0xff00 else [])
        return dict(kind='fault', motor_id=motor_id, faults=faults)
    return None


def robot_processes():
    """Command lines of other processes that may own a CAN bus."""
    lines = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True, check=True).stdout
    found = []
    for line in lines.splitlines():
        pid, _, args = line.strip().partition(' ')
        if int(pid) != os.getpid() and 'motor_bench' not in args and any(n in args for n in ROBOT_PROCESSES):
            found.append(line.strip())
    return found


LINK_CACHE = {}


def link_state(name, max_age=1.0):
    cached = LINK_CACHE.get(name)
    if cached and time.monotonic() - cached[0] < max_age:
        return cached[1]
    LINK_CACHE[name] = (time.monotonic(), read_link(name))
    return LINK_CACHE[name][1]


def read_link(name):
    try:
        out = subprocess.run(['ip', '-details', '-json', 'link', 'show', name],
                             capture_output=True, text=True, timeout=2)
        info = json.loads(out.stdout)[0]
        can = info.get('linkinfo', {}).get('info_data', {})
        return dict(name=name, operstate=info.get('operstate'), state=can.get('state'),
                    bitrate=can.get('bittiming', {}).get('bitrate'))
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return dict(name=name, operstate='unknown', state=None, bitrate=None)


def number(data, key, lo, hi, default=None):
    value = data.get(key, default)
    if value is None or isinstance(value, bool):
        raise ValueError(f'{key} is required')
    value = float(value)
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{key} must be in [{lo:g}, {hi:g}]')
    return value


def summarize(samples, hold_s):
    """Mean reported torque/current over the settled part of a hold (last 70%)."""
    if not samples:
        return {}
    end = samples[-1]['t']
    settled = [s for s in samples if s['t'] >= end - 0.7 * hold_s] or samples
    mean = lambda key: (sum(s[key] for s in settled if s.get(key) is not None) /
                        max(1, sum(1 for s in settled if s.get(key) is not None)))
    return dict(reported_nm=mean('torque'), iq_a=mean('iq') if any(s.get('iq') is not None for s in settled) else None,
                peak_reported_nm=max(abs(s['torque']) for s in samples),
                max_temperature_c=max(s['temperature'] for s in samples),
                travel_rad=max(s['position'] for s in samples) - min(s['position'] for s in samples))


class Bench:
    def __init__(self, can_module=None, clock=time.monotonic):
        self.can = can_module
        self.clock = clock
        self.lock = threading.RLock()
        self.bus = self.notifier = self.channel = None
        self.motor_id = None
        self.feedback = None          # last state frame of the selected motor
        self.params = {}              # index -> (value, time)
        self.faults = []
        self.heard = {}               # motor_id -> uid (get-ID replies) or None (other frames)
        self.uids = {}                # motor_id -> set of UIDs (ID conflicts show as >1)
        self.enabled = False
        self.mode = 'idle'            # idle | hold | jog | spin | torque
        self.message = 'Bus closed'
        self.error = ''
        self.original_limit = None
        self.reference = 0.0          # host-side position reference (never written to the motor)
        self.jog = dict(target=None, position=None, speed=1.0, kp=10.0, kd=1.0, velocity=0.0)
        self.test = None
        self.results = []
        self.run_dir = None
        self.last_seen = clock()
        self.stop_event = threading.Event()
        self.thread = None

    # ---- bus -------------------------------------------------------------------------
    def open(self, data):
        channel = data.get('bus')
        if channel not in BUSES:
            raise ValueError(f'bus must be one of {BUSES}')
        with self.lock:
            if self.bus is not None:
                raise ValueError(f'{self.channel} already open; close it first')
            owners = robot_processes()
            if owners:
                raise ValueError('Robot CAN process running; stop it first: ' + '; '.join(owners))
            link = link_state(channel, max_age=0)
            if link['operstate'] != 'UP' or link['bitrate'] not in (None, BITRATE):
                raise ValueError(f"{channel} is {link['operstate']} at {link['bitrate']} bit/s; bring it up at 1 Mbit/s")
            can = self.can or __import__('can')
            self.bus = can.interface.Bus(interface='socketcan', channel=channel, bitrate=BITRATE)
            self.notifier = can.Notifier(self.bus, [self.receive])
            self.channel, self.heard, self.uids = channel, {}, {}
            self.stop_event.clear()
            self.thread = threading.Thread(target=self.loop, daemon=True)
            self.thread.start()
            self.message = f'{channel} open; scan or enter the motor ID'
        return self.message

    def close(self, _=None):
        self.disable('Bus closed')
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(2)
        with self.lock:
            if self.notifier is not None:
                self.notifier.stop()
            if self.bus is not None:
                self.bus.shutdown()
            self.bus = self.notifier = self.thread = self.channel = None
            self.motor_id = self.feedback = None
            self.message = 'Bus closed'
        return self.message

    def send(self, arbitration_id_data):
        arbitration_id, data = arbitration_id_data
        can = self.can or __import__('can')
        self.bus.send(can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True), timeout=0.05)

    def receive(self, msg):
        if not msg.is_extended_id or msg.is_error_frame or msg.is_remote_frame:
            return
        decoded = parse(msg.arbitration_id, msg.data)
        if decoded is None:
            return
        now = self.clock()
        with self.lock:
            mid = decoded['motor_id']
            if decoded['kind'] == 'id':
                self.heard[mid] = decoded['uid']
                self.uids.setdefault(mid, set()).add(decoded['uid'])
                return
            self.heard.setdefault(mid, None)
            if mid != self.motor_id:
                return
            if decoded['kind'] == 'state':
                self.feedback = dict(decoded, received=now)
            elif decoded['kind'] == 'param':
                self.params[decoded['index']] = (decoded['value'], now)
            elif decoded['kind'] == 'fault':
                self.faults = decoded['faults']

    def scan(self, _=None):
        """Ask every ID for its unique MCU ID (comm type 0, read-only) and list responders."""
        with self.lock:
            if self.bus is None:
                raise ValueError('Open a bus first')
            if self.enabled:
                raise ValueError('Stop the motor before scanning')
            self.heard, self.uids = {}, {}
        for mid in range(1, 128):
            self.send(get_id_frame(mid))
            time.sleep(0.002)
        time.sleep(0.4)
        with self.lock:
            found = [dict(motor_id=m, uid=self.heard[m], robot_id=m in ROBOT_IDS,
                          conflict=len(self.uids.get(m, ())) > 1) for m in sorted(self.heard)]
            self.message = (f'Scan on {self.channel}: ' + (', '.join(str(f['motor_id']) for f in found) or
                                                          'no replies (motor powered? CAN wired to this bus?)'))
        return found

    def select(self, data):
        mid = int(number(data, 'motor_id', 1, 127))
        with self.lock:
            if self.bus is None:
                raise ValueError('Open a bus first')
            if self.enabled:
                raise ValueError('Stop the motor before changing ID')
            if mid in ROBOT_IDS and data.get('robot_ack') is not True:
                raise ValueError(f'ID {mid} is a robot motor ID. Tick the acknowledgement only if the robot '
                                 'motors on this bus are unpowered or disconnected')
            if len(self.uids.get(mid, ())) > 1:
                raise ValueError(f'ID {mid}: two motors answered (ID conflict); disconnect one')
            self.motor_id, self.feedback, self.params, self.faults = mid, None, {}, []
            self.results, self.test, self.run_dir = [], None, None
            self.message = f'Motor {mid} selected on {self.channel}'
        return self.message

    # ---- enable / disable --------------------------------------------------------------
    def wait_param(self, index, since, timeout=0.5):
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            with self.lock:
                value = self.params.get(index)
            if value is not None and value[1] >= since:
                return value[0]
            time.sleep(0.01)
        raise ValueError(f'Motor {self.motor_id} did not answer parameter 0x{index:04x}')

    def set_limit(self, limit):
        since = self.clock()
        self.send(write_param_frame(self.motor_id, LIMIT_TORQUE, limit))
        time.sleep(0.01)
        self.send(read_param_frame(self.motor_id, LIMIT_TORQUE))
        actual = self.wait_param(LIMIT_TORQUE, since)
        if abs(actual - limit) > 0.001:
            raise ValueError(f'Torque limit readback {actual:.3f} != {limit:.3f} Nm')
        return actual

    def enable(self, limit):
        """Operation-control mode, verified drive torque limit, then enable holding still."""
        with self.lock:
            if self.motor_id is None:
                raise ValueError('Select a motor first')
            if self.enabled:
                raise ValueError('Already enabled; stop first')
            fb = self.feedback
            if fb is None or self.clock() - fb['received'] > FEEDBACK_STALE_S:
                raise ValueError('No fresh feedback from the motor')
            if fb['flags'] or self.faults:
                raise ValueError(f"Motor reports faults {fb['flags'] + self.faults}; clear them first")
        since = self.clock()
        self.send(read_param_frame(self.motor_id, LIMIT_TORQUE))
        original = self.wait_param(LIMIT_TORQUE, since)
        self.send(write_param_frame(self.motor_id, RUN_MODE, 0))
        time.sleep(0.01)
        self.original_limit = original
        try:
            self.set_limit(limit)
        except ValueError:
            self.restore_limit()
            raise
        position = self.feedback['position']
        with self.lock:
            self.jog.update(target=position, position=position, velocity=0.0)
            self.send(control_frame(self.motor_id, position, 0.0, 0.0, 0.0, 0.0))
            self.send(enable_frame(self.motor_id))
            self.enabled = True
            self.last_seen = self.clock()

    def restore_limit(self):
        if self.original_limit is None or self.motor_id is None:
            return
        try:
            self.set_limit(self.original_limit)
            self.original_limit = None
        except (ValueError, OSError) as error:
            self.error = f'Torque limit not restored ({error}); power-cycle the motor'

    def disable(self, reason='Stopped', _=None):
        with self.lock:
            was = self.enabled
            self.enabled, self.mode = False, 'idle'
            if self.test is not None and self.test.get('phase') != 'done':
                self.test['phase'] = 'done'
                self.test['stopped'] = reason
            if self.bus is None or self.motor_id is None:
                return reason
            for _ in range(3):
                try:
                    self.send(stop_frame(self.motor_id))
                    break
                except OSError as error:
                    self.error = f'Stop send failed: {error}'
                    time.sleep(0.01)
            self.message = reason
        if was:
            time.sleep(0.05)
            self.restore_limit()
        return reason

    def stop(self, _=None):
        return self.disable('Stopped by operator')

    def clear_faults(self, _=None):
        with self.lock:
            if self.enabled or self.motor_id is None:
                raise ValueError('Select a motor and stop it first')
            self.faults = []
            self.send(stop_frame(self.motor_id, clear_faults=True))
            self.error = ''
        return 'Fault clear sent'

    def set_reference(self, _=None):
        with self.lock:
            if self.feedback is None:
                raise ValueError('No feedback yet')
            self.reference = self.feedback['position']
        return 'Reference set here (host only; nothing written to the motor)'

    # ---- rotation ------------------------------------------------------------------------
    def jog_params(self, data):
        return dict(speed=number(data, 'speed', 0.05, MAX_JOG_SPEED, 1.0), kp=number(data, 'kp', 0.5, MAX_KP, 10.0),
                    kd=number(data, 'kd', 0.0, MAX_KD, 1.0), limit=number(data, 'torque_limit', 0.1, MAX_TORQUE_NM, 2.0))

    def rotate(self, data):
        """Move to ``target_deg`` (relative to the host reference), or by ``delta_deg``."""
        params = self.jog_params(data)
        with self.lock:
            if self.mode == 'torque':
                raise ValueError('Torque test running; stop it first')
        if not self.enabled:
            self.enable(params['limit'])
        with self.lock:
            base = self.jog['target'] if self.mode in ('hold', 'jog') else self.feedback['position']
            if 'delta_deg' in data:
                target = base + math.radians(number(data, 'delta_deg', -720, 720))
            else:
                target = self.reference + math.radians(number(data, 'target_deg', -720, 720))
            if not P_RANGE[0] + 0.1 <= target <= P_RANGE[1] - 0.1:
                raise ValueError('Target outside the motor\'s +/-4 pi position range; set a new reference nearer')
            if self.mode not in ('hold', 'jog'):
                self.jog['position'] = self.feedback['position']
            self.jog.update(params, target=target, velocity=0.0)
            self.mode = 'jog'
            self.message = f'Moving to {math.degrees(target - self.reference):.1f} deg'
        return self.message

    def spin(self, data):
        """Constant speed (kp 0, kd x speed error) until stopped."""
        params = self.jog_params(data)
        velocity = number(data, 'velocity', -MAX_JOG_SPEED, MAX_JOG_SPEED)
        with self.lock:
            if self.mode == 'torque':
                raise ValueError('Torque test running; stop it first')
        if not self.enabled:
            self.enable(params['limit'])
        with self.lock:
            if params['kd'] <= 0:
                raise ValueError('Spin needs kd > 0')
            self.jog.update(params, velocity=velocity)
            self.mode = 'spin'
            self.message = f'Spinning at {velocity:.2f} rad/s'
        return self.message

    # ---- torque test ---------------------------------------------------------------------
    def torque_test(self, data):
        steps = data.get('steps')
        if not isinstance(steps, list) or not 1 <= len(steps) <= 40:
            raise ValueError('steps must be a list of 1-40 torques')
        steps = [number(dict(t=s), 't', 0.05, MAX_TORQUE_NM) for s in steps]
        direction = 1.0 if data.get('direction', 1) in (1, '+', '1') else -1.0
        cfg = dict(steps=steps, direction=direction,
                   hold_s=number(data, 'hold_s', 0.5, MAX_HOLD_S, 3.0),
                   rest_s=number(data, 'rest_s', 0.0, 120.0, 5.0),
                   ramp_nm_s=number(data, 'ramp_nm_s', 0.5, 20.0, 4.0),
                   kd=number(data, 'kd', 0.0, MAX_KD, 0.5),
                   travel_guard_rad=math.radians(number(data, 'travel_guard_deg', 1.0, 90.0, 20.0)),
                   speed_guard=number(data, 'speed_guard', 0.2, 5.0, 1.0),
                   temperature_limit=number(data, 'temperature_limit', 30.0, MAX_TEMPERATURE_C, 45.0),
                   wait_next=data.get('wait_next') is True,
                   lever_m=number(data, 'lever_m', 0.005, 1.0))
        with self.lock:
            if self.enabled:
                raise ValueError('Stop the motor first; the torque test enables with its own limit')
        self.enable(min(MAX_TORQUE_NM, max(steps) + 0.25))
        with self.lock:
            if self.run_dir is None:
                stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
                self.run_dir = RUNS / f'motor-bench-{self.motor_id}-{stamp}'
                self.run_dir.mkdir(parents=True, exist_ok=True)
            self.test = dict(cfg, index=0, phase='ramp_up', command=0.0, phase_started=self.clock(),
                             start_position=self.feedback['position'], samples=[], stopped=None,
                             started_unix=time.time())
            self.mode = 'torque'
            self.message = f'Torque test: {len(steps)} step(s) up to {max(steps):g} Nm'
            (self.run_dir / 'config.json').write_text(json.dumps(dict(cfg, motor_id=self.motor_id,
                                                                      bus=self.channel), indent=2))
        return self.message

    def next_step(self, _=None):
        with self.lock:
            if self.test is None or self.test['phase'] != 'wait':
                raise ValueError('Not waiting for Next')
            self.test['phase'], self.test['phase_started'] = 'rest', self.clock() - self.test['rest_s']
        return 'Continuing'

    def scale(self, data):
        """Operator's scale reading (kg) for a finished step; measured torque = (kg - zero) g r."""
        index = int(number(data, 'step', 0, 1000))
        kg = number(data, 'scale_kg', -1000, 1000)
        zero = number(data, 'zero_kg', -1000, 1000, 0.0)
        with self.lock:
            if not 0 <= index < len(self.results):
                raise ValueError('Unknown step')
            row = self.results[index]
            row.update(scale_kg=kg, zero_kg=zero, measured_nm=(kg - zero) * G * row['lever_m'])
            self.save_results()
        return 'Saved'

    def save_results(self):
        if self.run_dir is not None:
            (self.run_dir / 'results.json').write_text(json.dumps(self.results, indent=2))

    def torque_tick(self, now, fb):
        t = self.test
        target = t['steps'][t['index']]
        elapsed = now - t['phase_started']
        if t['phase'] == 'ramp_up':
            t['command'] = min(target, t['ramp_nm_s'] * elapsed)
            if t['command'] >= target:
                t['phase'], t['phase_started'], t['samples'] = 'hold', now, []
        elif t['phase'] == 'hold':
            t['command'] = target
            t['samples'].append(dict(t=now, torque=fb['torque'] * t['direction'], position=fb['position'],
                                     temperature=fb['temperature'], iq=self.param(IQF)))
            if elapsed >= t['hold_s']:
                row = dict(step=len(self.results), commanded_nm=target, lever_m=t['lever_m'],
                           direction=t['direction'], hold_s=t['hold_s'], unix=time.time(),
                           **summarize(t['samples'], t['hold_s']))
                if row.get('iq_a') is not None:
                    row['iq_a'] *= t['direction']
                self.results.append(row)
                self.save_results()
                t['phase'], t['phase_started'], t['ramp_from'] = 'ramp_down', now, target
        elif t['phase'] == 'ramp_down':
            t['command'] = max(0.0, t['ramp_from'] - t['ramp_nm_s'] * elapsed)
            if t['command'] <= 0:
                last = t['index'] + 1 >= len(t['steps'])
                t['phase'] = 'finish' if last else ('wait' if t['wait_next'] else 'rest')
                t['phase_started'] = now
        elif t['phase'] == 'rest':
            t['command'] = 0.0
            if elapsed >= t['rest_s']:
                t['index'] += 1
                t['phase'], t['phase_started'] = 'ramp_up', now
        elif t['phase'] == 'wait':
            t['command'] = 0.0
        if t['phase'] == 'finish':
            return 'Torque test complete'
        if abs(fb['position'] - t['start_position']) > t['travel_guard_rad']:
            return (f"Output moved {math.degrees(abs(fb['position'] - t['start_position'])):.1f} deg "
                    '(end effector off the scale?)')
        if abs(fb['velocity']) > t['speed_guard']:
            return f"Output speed {fb['velocity']:.2f} rad/s over the {t['speed_guard']:g} guard"
        if fb['temperature'] >= t['temperature_limit']:
            return f"Temperature {fb['temperature']:.1f} C reached the {t['temperature_limit']:g} C limit"
        return None

    def param(self, index):
        value = self.params.get(index)
        return None if value is None or self.clock() - value[1] > 1.0 else value[0]

    # ---- control loop ----------------------------------------------------------------------
    def loop(self):
        tick = 0
        period = 1 / CONTROL_HZ
        while not self.stop_event.wait(period):
            tick += 1
            stop_reason = None
            try:
                with self.lock:
                    if self.motor_id is None:
                        continue
                    now = self.clock()
                    if not self.enabled:
                        if tick % 5 == 0:  # disabled: a stop frame is answered with state
                            self.send(stop_frame(self.motor_id))
                        if tick % 10 == 1:
                            self.send(read_param_frame(self.motor_id, VBUS))
                        continue
                    fb = self.feedback
                    if fb is None or now - fb['received'] > FEEDBACK_STALE_S:
                        stop_reason = 'Feedback lost'
                    elif fb['flags'] or self.faults:
                        stop_reason = f"Motor fault: {', '.join(fb['flags'] + self.faults)}"
                    elif now - self.last_seen > HEARTBEAT_S:
                        stop_reason = 'Dashboard stopped polling (page closed or tunnel down)'
                    elif fb['temperature'] >= MAX_TEMPERATURE_C:
                        stop_reason = f"Temperature {fb['temperature']:.1f} C"
                    elif self.mode == 'torque':
                        stop_reason = self.torque_tick(now, fb)
                        if stop_reason is None:
                            t = self.test
                            self.send(control_frame(self.motor_id, fb['position'], 0.0,
                                                    t['direction'] * t['command'], 0.0, t['kd']))
                    elif self.mode in ('jog', 'hold'):
                        j = self.jog
                        if abs(fb['velocity']) > j['speed'] * 1.5 + 1.0:
                            stop_reason = f"Speed {fb['velocity']:.2f} rad/s over the jog guard"
                        else:
                            gap = j['target'] - j['position']
                            step = math.copysign(min(abs(gap), j['speed'] * period), gap)
                            j['position'] += step
                            velocity = step / period if self.mode == 'jog' else 0.0
                            if self.mode == 'jog' and abs(j['target'] - j['position']) < 1e-9:
                                self.mode, self.message = 'hold', 'Holding at target'
                            self.send(control_frame(self.motor_id, j['position'], velocity, 0.0, j['kp'], j['kd']))
                    elif self.mode == 'spin':
                        j = self.jog
                        if abs(fb['velocity']) > abs(j['velocity']) * 1.5 + 1.0:
                            stop_reason = f"Speed {fb['velocity']:.2f} rad/s over the spin guard"
                        elif abs(fb['position']) > P_RANGE[1] - 0.5:
                            stop_reason = 'Near the +/-4 pi position range end'
                        else:
                            self.send(control_frame(self.motor_id, fb['position'], j['velocity'], 0.0, 0.0, j['kd']))
                    if stop_reason is None and tick % 5 == 0:
                        self.send(read_param_frame(self.motor_id, IQF if tick % 10 else VBUS))
            except OSError as error:
                stop_reason = f'CAN send failed: {error}'
            if stop_reason is not None:
                if not stop_reason.startswith('Torque test complete'):
                    self.error = stop_reason
                self.disable(stop_reason)

    # ---- state ---------------------------------------------------------------------------
    def state(self):
        with self.lock:
            self.last_seen = self.clock()
            fb = self.feedback
            now = self.clock()
            test = None
            if self.test is not None:
                t = self.test
                test = dict(phase=t['phase'], index=t['index'], steps=t['steps'], command=t['command'],
                            direction=t['direction'], stopped=t['stopped'])
            return dict(
                buses=[link_state(b) for b in BUSES], channel=self.channel, motor_id=self.motor_id,
                enabled=self.enabled, mode=self.mode, message=self.message, error=self.error,
                feedback=None if fb is None else dict(
                    {k: fb[k] for k in ('position', 'velocity', 'torque', 'temperature', 'mode', 'flags')},
                    age=now - fb['received'], relative_deg=math.degrees(fb['position'] - self.reference)),
                faults=self.faults, iq_a=self.param(IQF), vbus_v=self.param(VBUS),
                torque_limit=self.param(LIMIT_TORQUE), test=test, results=self.results,
                run_dir=str(self.run_dir) if self.run_dir else None,
                jog=dict(target_deg=None if self.jog['target'] is None else
                         math.degrees(self.jog['target'] - self.reference)))


def clean(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def serve(bench, port):
    token = secrets.token_urlsafe(32)
    origin = f'http://127.0.0.1:{port}'
    actions = {'open': bench.open, 'close': bench.close, 'scan': bench.scan, 'select': bench.select,
               'stop': bench.stop, 'clear_faults': bench.clear_faults, 'reference': bench.set_reference,
               'rotate': bench.rotate, 'spin': bench.spin, 'torque_test': bench.torque_test,
               'next': bench.next_step, 'scale': bench.scale}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, value, status=200, kind='application/json'):
            payload = (json.dumps(clean(value), allow_nan=False) if kind == 'application/json' else value).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.headers.get('Host') != f'127.0.0.1:{port}':
                return self.reply({'error': 'Invalid host'}, 403)
            if self.path == '/':
                page = (ROOT / 'scripts' / 'motor_bench.html').read_text().replace('__TOKEN__', token)
                return self.reply(page, kind='text/html; charset=utf-8')
            if self.path == '/state':
                return self.reply(bench.state())
            return self.reply({'error': 'Not found'}, 404)

        def do_POST(self):
            if (self.headers.get('Origin') != origin or self.headers.get('X-Control-Token') != token or
                    self.headers.get('Host') != f'127.0.0.1:{port}'):
                return self.reply({'error': 'Invalid request origin/token'}, 403)
            try:
                n = int(self.headers.get('Content-Length', '0'))
                if not 0 < n <= 4096:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(n))
                if not isinstance(data, dict):
                    raise ValueError('Expected object')
                action = actions.get(self.path.removeprefix('/'))
                if action is None:
                    raise ValueError('Unknown action')
                result = action(data)
                if self.path != '/stop':
                    bench.error = ''
                self.reply({'ok': True, 'result': result})
            except (ValueError, TypeError, KeyError, RuntimeError, OSError) as error:
                if self.path != '/stop':
                    bench.error = str(error)
                self.reply({'error': str(error)}, 400)

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print(f'Open {origin} through SSH forwarding. No CAN traffic until you open a bus.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bench.close()
        server.server_close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()

    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    serve(Bench(), args.port)


if __name__ == '__main__':
    main()
