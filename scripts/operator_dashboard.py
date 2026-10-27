#!/usr/bin/env python3
"""Local operator dashboard for read-only telemetry and hip/knee homing.

Connect through SSH forwarding, never a public port. Starting the server or
opening the page sends no CAN traffic; only explicit operator actions start the
motor_controller node, reset CAN links or call homing services.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import secrets
import shlex
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src' / 'tocatl_bringup' / 'config'
EXECUTABLE = ROOT / 'install' / 'tocatl_lowlevel' / 'lib' / 'tocatl_lowlevel' / 'motor_controller'
BUSES = ('can0', 'can1')
BITRATE = 1_000_000
LEGS = (1, 2, 3, 4, 5, 6)
# Physically disconnected; never polled or commanded (docs/can-debug-2026-10-25.md).
OUT_OF_SERVICE = (51,)
# Legs whose stop references are operator-declared and unloaded only.
UNVALIDATED_LEGS = (2, 3, 4, 5, 6)
PROFILES = {
    'telemetry': dict(label='Level 1 · Read-only telemetry', params=[CONFIG / 'hardware.yaml'],
                      homing=False, motion=False),
    'homing': dict(label='Level 2 · Hip/knee homing', params=[CONFIG / 'homing-hip-knee.yaml'],
                   homing=True, motion=False),
    'motion': dict(label='Level 3 · Feet-in-air motion',
                   params=[CONFIG / 'homing-hip-knee.yaml', CONFIG / 'motion-feet-air.yaml'],
                   homing=True, motion=True),
}
MOTION_KINDS = ('nudge', 'pose', 'circle')
# Player flags the dashboard may pass, with their types (planner validates the values).
MOTION_FIELDS = {'d_pitch_rad': float, 'd_knee_rad': float, 'r_m': float, 'z_m': float, 'radius_m': float,
                 'loops': int, 'hold_s': float, 'max_speed_rad_s': float}
# Any of these owning a bus makes starting a controller or resetting a link unsafe.
ROBOT_PROCESSES = ('motor_controller', 'cybergear_telemetry', 'motor_telemetry', 'hold_capture',
                   'tocatl_diagnostics', 'run_spider_art_cue', 'joint_endpoint_probe', 'leg_auto_zero')
STALE_AFTER_S = 1.0
# CyberGear output position is ambiguous by whole rotor turns across power cycles (7.75:1).
ROTOR_TURN_RAD = 2 * math.pi / 7.75
RECORD_PERIOD_S = 0.1
RECORD_MAX_S = 120
RECORD_LABEL = re.compile(r'[A-Za-z0-9_-]{1,60}')


def clean(value):
    """JSON-safe copy: NaN/inf become null so the browser can parse the reply."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def controller_command(profile, legs=LEGS, commissioning=False, output_path=None):
    """Exact argv for the controller; homing flags are only ever set on the command line."""
    if profile not in PROFILES:
        raise ValueError('Unknown controller profile')
    args = [sys.executable, str(EXECUTABLE), '--ros-args']
    for params in PROFILES[profile]['params']:
        args += ['--params-file', str(params)]
    if not PROFILES[profile]['homing']:
        return args
    legs = sorted(set(legs))
    if not legs or any(type(leg) is not int or leg not in LEGS for leg in legs):
        raise ValueError('Select at least one leg from 1 through 6')
    if type(commissioning) is not bool:
        raise ValueError('Commissioning must be true or false')
    if output_path is None:
        raise ValueError('Homing needs a session output path')
    selection = '[' + ','.join(map(str, legs)) + ']'
    args += ['-p', 'allow_homing:=true',
             '-p', f'homing.legs:={selection}',
             '-p', f'homing.verify_repeatability:={str(commissioning).lower()}',
             '-p', f'homing.output_path:={output_path}']
    if PROFILES[profile]['motion']:
        # Only the selected legs' hip pitch/knee are polled and commandable: fewer motors keep
        # bus sweeps short, which the 0.1 rad position-step check depends on.
        motors = '[' + ','.join(str(leg * 10 + j) for leg in legs for j in (2, 3)) + ']'
        args += ['-p', 'allow_hardware_motion:=true',
                 '-p', f'actuation_motor_ids:={motors}', '-p', f'telemetry_motor_ids:={motors}']
    return args


def player_command(legs, kind, values, dry_run, actuated_legs):
    """Exact argv for tocatl_play (tocatl_kinematics.player). The controller only accepts
    commands naming every actuated motor, so the session's legs are always passed:
    those not being moved hold their pose."""
    if kind not in MOTION_KINDS:
        raise ValueError(f'Motion must be one of {MOTION_KINDS}')
    if (not isinstance(legs, list) or not legs or len(set(legs)) != len(legs) or
            any(type(leg) is not int or leg not in LEGS for leg in legs)):
        raise ValueError('Choose the legs to move')
    if not set(legs) <= set(actuated_legs):
        raise ValueError('Move only legs this controller session commands')
    args = [sys.executable, '-m', 'tocatl_kinematics.player', '--legs', ','.join(map(str, sorted(legs))),
            '--actuated-legs', ','.join(map(str, sorted(actuated_legs))), '--kind', kind]
    for name, value in values.items():
        kind_of = MOTION_FIELDS.get(name)
        if kind_of is None:
            raise ValueError(f'Unknown motion setting {name}')
        if kind_of is int and type(value) is not int or kind_of is float and (
                isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)):
            raise ValueError(f'{name} must be a finite number')
        args += ['--' + name.replace('_', '-'), str(value)]
    return args + (['--dry-run'] if dry_run else [])


def can_links():
    """Linux state of can0/can1 from ``ip``; reads host counters only, never queries motors."""
    result = subprocess.run(['ip', '-j', '-details', '-statistics', 'link', 'show'],
                            capture_output=True, text=True, check=True, timeout=5)
    return [item for item in json.loads(result.stdout) if item.get('ifname') in BUSES]


def summarize_link(item):
    info = item.get('linkinfo', {}).get('info_data', {})
    stats = item.get('stats64', item.get('stats', {}))
    tx, rx = stats.get('tx', {}), stats.get('rx', {})
    return dict(name=item.get('ifname'), operstate=item.get('operstate'), state=info.get('state'),
                bitrate=info.get('bittiming', {}).get('bitrate'), txqlen=item.get('txqlen'),
                restart_ms=info.get('restart_ms'),
                tx_packets=tx.get('packets'), tx_errors=tx.get('errors'), tx_dropped=tx.get('dropped'),
                rx_packets=rx.get('packets'), rx_errors=rx.get('errors'), rx_dropped=rx.get('dropped'),
                bus_errors=item.get('linkinfo', {}).get('info_xstats', {}).get('bus_error'),
                restarts=item.get('linkinfo', {}).get('info_xstats', {}).get('restarts'))


def can_problems(links, buses=BUSES):
    """Reasons the links cannot carry motor traffic; empty when all are UP, ERROR-ACTIVE, 1 Mbit/s."""
    found = {item['name']: item for item in links}
    problems = []
    for name in buses:
        item = found.get(name)
        if item is None:
            problems.append(f'{name} missing')
        elif item['operstate'] != 'UP' or item['state'] != 'ERROR-ACTIVE':
            problems.append(f"{name} {item['operstate']} {item['state'] or 'unknown'}")
        elif item['bitrate'] != BITRATE:
            problems.append(f"{name} bitrate {item['bitrate']}")
    return problems


def robot_processes(own_pids=()):
    """Command lines of other processes that may own a CAN bus."""
    lines = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True, check=True).stdout
    found = []
    for line in lines.splitlines():
        pid, _, args = line.strip().partition(' ')
        if int(pid) in own_pids or int(pid) == os.getpid() or 'operator_dashboard' in args:
            continue
        if any(name in args for name in ROBOT_PROCESSES):
            found.append(line.strip())
    return found


def homing_results(path):
    """Per-motor summary of a session results file; reference mismatches are surfaced, not hidden."""
    path = Path(path)
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return []
    rows = []
    for leg, record in sorted(data.get('legs', {}).items(), key=lambda item: int(item[0])):
        for motor in record.get('motors', []):
            turns, residual = motor.get('reference_turns'), motor.get('reference_residual_rad')
            delta = motor.get('reference_delta_rad')
            if turns is None and isinstance(delta, (int, float)):  # results written before turn-aware homing
                turns = round(delta / ROTOR_TURN_RAD)
                residual = delta - turns * ROTOR_TURN_RAD
            rows.append(dict(leg=int(leg), mode=record.get('mode'), motor_id=motor.get('motor_id'),
                             turns=turns, residual_rad=residual,
                             home_rad=motor.get('home_position_rad'),
                             reference_rad=motor.get('previous_reference_rad'),
                             delta_rad=motor.get('reference_delta_rad'),
                             within_window=motor.get('reference_within_window'),
                             contacts=len(motor.get('contact_positions_rad', [])),
                             spread_rad=motor.get('spread_rad'),
                             completed_unix_s=record.get('completed_unix_s')))
    return clean(rows)


def robot_state_dict(msg):
    return dict(
        safety_state=msg.safety_state, status_reason=msg.status_reason, backend=msg.backend,
        telemetry_fresh=msg.telemetry_fresh, fault_status_valid=msg.fault_status_valid,
        motors_enabled=msg.motors_enabled, faults=list(msg.faults),
        motors=[dict(motor_id=m.motor_id, joint=m.joint_name, bus=m.bus, valid=m.telemetry_valid,
                     fresh=m.telemetry_fresh, fault_valid=m.fault_status_valid, enabled=m.enabled,
                     mode=m.mode, position=m.position_rad, velocity=m.velocity_rad_s,
                     torque=m.torque_estimate_nm, temperature=m.temperature_c, faults=list(m.faults))
                for m in msg.motors],
        legs=[dict(leg=l.leg_id, motor_ids=list(l.motor_ids), homed=l.homed, ready=l.ready,
                   home_positions=list(l.home_positions_rad), status_reason=l.status_reason)
              for l in msg.legs])


class RosBridge:
    """Subscribes to robot_state and calls controller services. Sends no CAN traffic itself."""

    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from tocatl_interfaces.msg import RobotState
        # main() owns SIGINT/SIGTERM so shutdown always stops the controller first.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy = rclpy
        self.node = rclpy.create_node('tocatl_operator_dashboard')
        self.lock = threading.Lock()
        self.latest = None
        self.received = None
        self.clients = {}
        self.node.create_subscription(RobotState, 'robot_state', self.receive, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def receive(self, msg):
        with self.lock:
            self.latest, self.received = msg, time.monotonic()

    def snapshot(self):
        with self.lock:
            msg, received = self.latest, self.received
        if msg is None:
            return None, None
        return clean(robot_state_dict(msg)), time.monotonic() - received

    def call(self, name, kind, timeout=3.0, **fields):
        from std_srvs.srv import SetBool, Trigger
        service = {'trigger': Trigger, 'setbool': SetBool}[kind]
        with self.lock:
            client = self.clients.get(name)
            if client is None:
                client = self.clients[name] = self.node.create_client(service, name)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{name} unavailable; is the controller running with this profile?')
        request = service.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RuntimeError(f'{name} did not answer within {timeout} s')
        response = future.result()
        return response.success, response.message

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class Dashboard:
    def __init__(self, bridge=None, preview_only=False, runs_root=None):
        self.bridge = bridge
        self.preview_only = preview_only or bridge is None
        self.runs_root = Path(runs_root or Path.home() / 'tocatl-runs')
        self.lock = threading.RLock()
        self.proc = None
        self.profile = None
        self.legs = []
        self.commissioning = False
        self.argv = None
        self.started = None
        self.exit_code = None
        self.run_dir = None
        self.output_path = None
        self.logs = deque(maxlen=400)
        self.events = deque(maxlen=50)
        self.can = []
        self.can_error = ''
        self.busy = ''
        self.recording = None
        self.record_stop = threading.Event()
        self.player = None
        self.player_mode = ''
        self.motion_log = deque(maxlen=300)

    def event(self, text):
        stamp = datetime.now().strftime('%H:%M:%S')
        with self.lock:
            self.events.append(f'{stamp}  {text}')

    def running(self):
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def refresh_can(self):
        try:
            links = [summarize_link(item) for item in can_links()]
            with self.lock:
                self.can, self.can_error = links, ''
        except Exception as error:
            with self.lock:
                self.can_error = f'Could not read CAN links: {error}'
        return self.can

    def plan(self, data):
        profile = data.get('profile')
        legs = data.get('legs', list(LEGS))
        commissioning = data.get('commissioning', False)
        output = (self.runs_root / '<session>' / 'hip-knee-homing.json') if profile in PROFILES and PROFILES[profile]['homing'] else None
        return shlex.join(controller_command(profile, legs, commissioning, output))

    def start_controller(self, data):
        if self.preview_only:
            raise ValueError('Preview-only dashboard cannot start the controller')
        if data.get('checklist_confirmed') is not True:
            raise ValueError('Confirm every item of the physical checklist first')
        profile = data.get('profile')
        if profile in PROFILES and PROFILES[profile]['motion'] and data.get('motion_confirmed') is not True:
            raise ValueError('Confirm the Level 3 motion conditions first')
        legs, commissioning = data.get('legs', list(LEGS)), data.get('commissioning', False)
        controller_command(profile, legs, commissioning, 'check')  # validate before side effects
        if not EXECUTABLE.is_file():
            raise ValueError('motor_controller is not built; run colcon build first')
        with self.lock:
            if self.running() or self.busy:
                raise ValueError('A controller or operation is already active')
            self.busy = 'Starting controller'
        try:
            others = robot_processes()
            if others:
                raise ValueError('Another process may own the bus: ' + '; '.join(others))
            problems = can_problems(self.refresh_can())
            if problems:
                raise ValueError('CAN links not ready (' + ', '.join(problems) + '); reset the link first')
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            run_dir = self.runs_root / f'dashboard-{profile}-{stamp}'
            run_dir.mkdir(parents=True)
            output = run_dir / 'hip-knee-homing.json'
            argv = controller_command(profile, legs, commissioning, output)
            log = open(run_dir / 'controller.log', 'w')
            log.write('$ ' + shlex.join(argv) + '\n')
            log.flush()
            env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0',
                       RCUTILS_COLORIZED_OUTPUT='0')
            proc = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, env=env, start_new_session=True)
            with self.lock:
                self.proc, self.profile, self.argv = proc, profile, argv
                self.legs = sorted(set(legs)) if PROFILES[profile]['homing'] else []
                self.commissioning = bool(commissioning) and PROFILES[profile]['homing']
                self.started, self.exit_code = time.time(), None
                self.run_dir, self.output_path = str(run_dir), str(output) if PROFILES[profile]['homing'] else None
                self.logs.clear()
            threading.Thread(target=self.follow, args=(proc, log), daemon=True).start()
            self.event(f'Started {PROFILES[profile]["label"]} (pid {proc.pid})')
        finally:
            with self.lock:
                self.busy = ''

    def follow(self, proc, log):
        with log:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                with self.lock:
                    self.logs.append(line.rstrip())
            code = proc.wait()
            log.write(f'[exit {code}]\n')
        with self.lock:
            self.exit_code = code
        self.event(f'Controller exited with status {code}')

    def stop_controller(self):
        with self.lock:
            proc = self.proc
            if proc is None or proc.poll() is not None:
                raise ValueError('No controller is running')
            if self.busy:
                raise ValueError(f'{self.busy} in progress')
            self.busy = 'Stopping controller'
        threading.Thread(target=self.shutdown, args=(proc,), daemon=True).start()

    def shutdown(self, proc):
        """Cancel/disarm first, then SIGINT so the node disables and restores limits itself."""
        try:
            self.stop_player()
            try:
                self.disarm()
            except Exception as error:
                self.event(f'Disarm before stop failed: {error}')
            proc.send_signal(signal.SIGINT)
            self.event('Sent SIGINT to controller')
            for sig, wait in ((None, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                if sig is not None and proc.poll() is None:
                    proc.send_signal(sig)
                    self.event(f'Controller did not exit; sent {sig.name}')
                try:
                    proc.wait(wait)
                    break
                except subprocess.TimeoutExpired:
                    continue
        finally:
            with self.lock:
                self.busy = ''

    def disarm(self):
        if not self.running():
            raise ValueError('No controller is running')
        success, message = self.bridge.call('motors/enable', 'setbool', data=False)
        self.event(f'Cancel/disarm: {message}')
        return success, message

    def home(self, data):
        leg = data.get('leg')
        with self.lock:
            if not self.running() or not PROFILES[self.profile]['homing']:
                raise ValueError('Start the homing controller first')
            if data.get('homing_confirmed') is not True:
                raise ValueError('Confirm the homing checklist first')
            if leg is not None and leg not in self.legs:
                raise ValueError('That leg is not in this session\'s homing.legs')
            if any(l in UNVALIDATED_LEGS for l in ([leg] if leg else self.legs)) and data.get(
                    'unvalidated_confirmed') is not True:
                raise ValueError('Acknowledge that legs 2–6 have only unloaded stop references')
        state, age = self.bridge.snapshot()
        if state is None or age > STALE_AFTER_S:
            raise ValueError('robot_state is not arriving; check the controller log')
        name = 'motors/home' if leg is None else f'motors/leg_{leg}/home'
        success, message = self.bridge.call(name, 'trigger')
        self.event(f'{name}: {message}')
        if not success:
            raise ValueError(message)

    def player_running(self):
        with self.lock:
            return self.player is not None and self.player.poll() is None

    def motion(self, data, dry_run):
        """Preview (dry run) or start the feet-in-air player for this Level 3 session."""
        legs = data.get('legs')
        with self.lock:
            session_legs = list(self.legs)
        argv = player_command(legs, data.get('kind'), data.get('values', {}), dry_run, session_legs)
        with self.lock:
            if not self.running() or not PROFILES[self.profile]['motion']:
                raise ValueError('Start the Level 3 motion controller first')
            if any(leg not in self.legs for leg in legs):
                raise ValueError('Move only legs selected for this controller session')
            if self.player_running() or self.busy:
                raise ValueError('A motion or operation is already active')
            if not dry_run and data.get('motion_confirmed') is not True:
                raise ValueError('Confirm the motion checklist first')
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        log = open(Path(self.run_dir) / f'motion-{stamp}{"-dry-run" if dry_run else ""}.log', 'w')
        log.write('$ ' + shlex.join(argv) + '\n')
        proc = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, env=env, start_new_session=True)
        with self.lock:
            self.player, self.player_mode = proc, 'preview' if dry_run else 'run'
            self.motion_log.clear()
        threading.Thread(target=self.follow_player, args=(proc, log), daemon=True).start()
        self.event(f'{"Previewing" if dry_run else "Started"} motion {data.get("kind")} on legs {sorted(legs)}')

    def follow_player(self, proc, log):
        with log:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                with self.lock:
                    self.motion_log.append(line.rstrip())
            code = proc.wait()
            log.write(f'[exit {code}]\n')
        self.event(f'Motion player exited with status {code}')

    def arm(self, data):
        with self.lock:
            if not self.running() or not PROFILES[self.profile]['motion']:
                raise ValueError('Start the Level 3 motion controller first')
            if not self.player_running() or self.player_mode != 'run':
                raise ValueError('Run a motion first; arming needs its commands streaming')
            if data.get('arm_confirmed') is not True:
                raise ValueError('Confirm you are ready to arm')
        success, message = self.bridge.call('motors/enable', 'setbool', data=True)
        self.event(f'Arm: {message}')
        if not success:
            raise ValueError(message)

    def stop_player(self):
        """Disarm, then interrupt the player (it also requests a disarm on its way out)."""
        if self.running():
            try:
                self.disarm()
            except Exception as error:
                self.event(f'Disarm failed: {error}')
        with self.lock:
            proc = self.player
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                proc.kill()

    def reset_can(self, data):
        buses = data.get('buses')
        if self.preview_only:
            raise ValueError('Preview-only dashboard cannot change CAN links')
        if not isinstance(buses, list) or not buses or any(b not in BUSES for b in buses) or len(set(buses)) != len(buses):
            raise ValueError('Choose can0, can1 or both')
        with self.lock:
            if self.running() or self.busy:
                raise ValueError('Stop the controller before resetting CAN')
            self.busy = 'Resetting CAN'
        try:
            others = robot_processes()
            if others:
                raise ValueError('Another process may own the bus: ' + '; '.join(others))
            for bus in buses:
                for step in (['down'], ['type', 'can', 'bitrate', str(BITRATE)], ['up']):
                    result = subprocess.run(['sudo', '-n', 'ip', 'link', 'set', bus, *step],
                                            capture_output=True, text=True, timeout=10)
                    if result.returncode:
                        raise ValueError(f'ip link set {bus} {" ".join(step)} failed: '
                                         f'{result.stderr.strip() or result.returncode}')
            problems = can_problems(self.refresh_can(), buses)
            if problems:
                raise ValueError('Links not ready after reset: ' + ', '.join(problems))
            self.event(f'Reset {", ".join(buses)} at 1 Mbit/s; no motor requests sent')
        finally:
            with self.lock:
                self.busy = ''

    def record(self, data):
        """Sample the robot_state already being received into JSONL; sends nothing to motors."""
        label = data.get('label')
        duration = data.get('duration_s')
        if not isinstance(label, str) or not RECORD_LABEL.fullmatch(label):
            raise ValueError('Label: 1-60 letters, digits, - or _ (e.g. leg2-hip-only)')
        if type(duration) is not int or not 1 <= duration <= RECORD_MAX_S:
            raise ValueError(f'Choose 1 through {RECORD_MAX_S} seconds')
        with self.lock:
            if not self.running() or self.bridge is None:
                raise ValueError('Start a controller first; recording reads its robot_state')
            if self.recording and self.recording['active']:
                raise ValueError('A recording is already running')
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            path = Path(self.run_dir) / 'recordings' / f'{stamp}-{label}.jsonl'
            self.recording = dict(label=label, path=str(path), duration_s=duration, samples=0,
                                  skipped=0, active=True, started=time.time())
            self.record_stop.clear()
        path.parent.mkdir(parents=True, exist_ok=True)
        threading.Thread(target=self.record_worker, args=(path, label, duration), daemon=True).start()
        self.event(f'Recording "{label}" for {duration} s')

    def record_worker(self, path, label, duration):
        start = time.monotonic()
        try:
            with open(path, 'w') as stream:
                while not self.record_stop.is_set() and time.monotonic() - start < duration:
                    robot, age = self.bridge.snapshot()
                    if robot is None or age > STALE_AFTER_S:
                        with self.lock:
                            self.recording['skipped'] += 1
                    else:
                        motors = {m['motor_id']: dict(position=m['position'], velocity=m['velocity'],
                                                      torque=m['torque'], enabled=m['enabled'])
                                  for m in robot['motors'] if m['valid'] and m['fresh']}
                        stream.write(json.dumps(clean(dict(
                            t=round(time.monotonic() - start, 3), unix=time.time(), label=label,
                            robot_age_s=age, motors=motors)), allow_nan=False) + '\n')
                        with self.lock:
                            self.recording['samples'] += 1
                    self.record_stop.wait(RECORD_PERIOD_S)
        finally:
            with self.lock:
                self.recording['active'] = False
                samples, skipped = self.recording['samples'], self.recording['skipped']
            self.event(f'Recording "{label}" saved: {samples} samples'
                       + (f', {skipped} skipped for stale robot_state' if skipped else ''))

    def stop_recording(self):
        self.record_stop.set()

    def recordings(self):
        if not self.run_dir:
            return []
        folder = Path(self.run_dir) / 'recordings'
        return sorted(p.name for p in folder.glob('*.jsonl')) if folder.is_dir() else []

    def state(self):
        robot, age = self.bridge.snapshot() if self.bridge else (None, None)
        with self.lock:
            running = self.running()
            results = homing_results(self.output_path) if self.output_path else []
            recording = dict(self.recording) if self.recording else None
            return clean(dict(
                preview_only=self.preview_only, busy=self.busy, running=running,
                pid=self.proc.pid if self.proc else None, profile=self.profile,
                profile_label=PROFILES[self.profile]['label'] if self.profile else None,
                legs=self.legs, commissioning=self.commissioning, exit_code=self.exit_code,
                command=shlex.join(self.argv) if self.argv else None,
                uptime_s=time.time() - self.started if running and self.started else None,
                run_dir=self.run_dir, output_path=self.output_path, results=results,
                logs=list(self.logs), events=list(self.events), can=self.can, can_error=self.can_error,
                robot=robot if running else None, robot_age_s=age if running else None,
                stale_after_s=STALE_AFTER_S, out_of_service=OUT_OF_SERVICE,
                unvalidated_legs=UNVALIDATED_LEGS, recording=recording, recordings=self.recordings(),
                motion_profile=bool(self.profile and PROFILES[self.profile]['motion']),
                player_running=self.player_running(), player_mode=self.player_mode,
                motion_log=list(self.motion_log), motion_kinds=MOTION_KINDS,
                record_max_s=RECORD_MAX_S,
                profiles={k: v['label'] for k, v in PROFILES.items()}))

    def close(self):
        self.stop_recording()
        if self.running():
            self.shutdown(self.proc)


def serve(dashboard, port):
    token = secrets.token_urlsafe(32)
    origin = f'http://127.0.0.1:{port}'
    actions = {'start': dashboard.start_controller, 'stop': lambda _: dashboard.stop_controller(),
               'disarm': lambda _: dashboard.disarm(), 'home': dashboard.home,
               'can_reset': dashboard.reset_can, 'can_refresh': lambda _: dashboard.refresh_can(),
               'plan': dashboard.plan, 'record': dashboard.record,
               'record_stop': lambda _: dashboard.stop_recording(),
               'motion_preview': lambda data: dashboard.motion(data, dry_run=True),
               'motion_run': lambda data: dashboard.motion(data, dry_run=False),
               'motion_stop': lambda _: dashboard.stop_player(), 'arm': dashboard.arm}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, value, status=200, kind='application/json'):
            payload = (json.dumps(value, allow_nan=False) if kind == 'application/json' else value).encode()
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
                page = (ROOT / 'scripts' / 'operator_dashboard.html').read_text().replace('__TOKEN__', token)
                return self.reply(page, kind='text/html; charset=utf-8')
            if self.path == '/state':
                return self.reply(dashboard.state())
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
                self.reply(clean({'ok': True, 'result': result if isinstance(result, (str, list)) else None}))
            except (ValueError, TypeError, KeyError, RuntimeError) as error:
                self.reply({'error': str(error)}, 400)

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print(f'Open {origin} through SSH forwarding. No CAN traffic until you start a controller.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()
        server.server_close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--preview-only', action='store_true',
                        help='never start a controller or touch CAN links (works without ROS)')
    args = parser.parse_args()
    bridge = None
    if not args.preview_only:
        try:
            bridge = RosBridge()
        except ImportError as error:
            print(f'ROS unavailable ({error}); source ROS and install/setup.bash. Starting preview-only.', flush=True)
    dashboard = Dashboard(bridge, args.preview_only)
    # Under nohup/background shells SIGINT starts ignored and would be inherited by the
    # controller; take both signals explicitly so either one stops the controller cleanly.
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    if not dashboard.preview_only:
        dashboard.refresh_can()
    try:
        serve(dashboard, args.port)
    finally:
        if bridge:
            bridge.close()


if __name__ == '__main__':
    main()
