"""Walk TOCATL: stream whole-body (yaw, hip pitch, knee) targets from the walk generator.

Sequence:
1. HOLD at the measured pose until the operator arms.
2. ENTER: feet blend in a straight line from where they are to the walking stance. From
   resting on the ground at the stance radius, this lifts the body without sliding the feet.
3. WALK: follow the command (a script, or /cmd_vel from teleop).
4. Stop: SETTLE (step until every foot is back at neutral), then EXIT (blend back to the
   starting feet, lowering the body), hold, disarm.

Body height (``walk/height``, Int8 +1 raises, -1 lowers; the D-pad in teleop) moves in 1 cm
steps among heights checked at startup (two each way, the full walk feasibility at each). A
change waits for the feet to settle, then the body moves vertically (ADJUST) and walking
resumes.

Deadman (``walk/deadman``, std_msgs/Bool, from tocatl_teleop: LB held and the pad fresh): while it
reads false, every phase freezes in place (joints held where they are; a tripod is statically
stable at every instant), and LB resumes from there at zero velocity. Without fresh deadman
messages (scripts, hand-published /cmd_vel) the walker ignores it.

Stopping:
- First Ctrl-C (or a ``walk/stop`` message) stops gracefully as above.
- A second Ctrl-C disarms at once. Disarming drops a standing robot, so keep it supported.

Before anything is published, the walk is checked offline at its maximum command, in every
direction and turning (``walk.feasibility``), against the running controller's torque cap
and yaw envelope. The enter/exit blends are checked against the joint limits and the
controller envelopes.

Every tick is lag-aware like tocatl_play: gait time slows as the worst motor falls behind,
targets never lead the measured position by more than the lead, and a stall stops the walk.
``--dry-run`` reports and exits.
"""
from __future__ import annotations

import argparse
from dataclasses import fields, replace
import json
import math
import sys
import time

import numpy as np

from tocatl_kinematics.body import BodyPose, body_fk, joints_from_raw
from tocatl_kinematics.geometry import LEG_IDS, load_geometry
from tocatl_kinematics.joint_map import load_joint_map, motor_ids
from tocatl_kinematics.leg import KinematicsError
from tocatl_kinematics.motions import EnvelopeOffsets
from tocatl_kinematics.player import (ARM_TIMEOUT_S, END_HOLD_S, SLOW_FROM, STALL_RATE, STALL_S, STATE_STALE_S,
                                      ControllerLimits, actuated_legs_from_ids, lead_for_step, query_controller,
                                      session_inputs, undeclared_yaw_legs, _state_dict)
from tocatl_kinematics.walk import GAITS, WalkGenerator, WalkParams, feasibility, solve

BLEND_SPEED_M_S = 0.03     # enter/exit foot speed
HEIGHT_STEP_M = 0.01       # D-pad body-height step
HEIGHT_STEPS = 2           # verified steps each way from the start height
BLEND_MIN_S = 2.0
TICK_S = 0.02
COMMAND_TIMEOUT_S = 0.5    # a /cmd_vel older than this counts as zero
DEADMAN_FRESH_S = 0.5      # walk/deadman messages older than this are ignored (no teleop)
YAW_RADIAL_TOLERANCE = 0.05


def _quintic(s):
    s = min(1.0, max(0.0, s))
    return s ** 3 * (10 - 15 * s + 6 * s * s)


def envelope_problems(session_map, raw: dict, offsets: EnvelopeOffsets):
    problems = []
    for motor_id, value in raw.items():
        low, high = (offsets.hip_yaw, offsets.hip_pitch, offsets.knee)[motor_id % 10 - 1]
        offset = value - session_map.references[motor_id]
        if not low - 1e-9 <= offset <= high + 1e-9:
            problems.append(f'motor {motor_id} target {offset:+.3f} rad from its reference, outside [{low:+.2f}, {high:+.2f}]')
    return problems


class WalkStreamer:
    """Pure decision logic for each tick (the ROS node only feeds it measurements)."""

    def __init__(self, geometry, session_map, params: WalkParams, raw_start: dict, commands,
                 offsets: EnvelopeOffsets = EnvelopeOffsets(), clock=time.monotonic, lead=0.08, legs=LEG_IDS):
        self.geometry, self.session_map, self.commands, self.offsets = geometry, session_map, commands, offsets
        self.clock, self.lead, self.legs = clock, lead, tuple(legs)
        self.gen = WalkGenerator(geometry, params, self.legs)
        self.motor_ids = tuple(m for leg in self.legs for m in motor_ids(leg))
        self.raw_start = {m: float(raw_start[m]) for m in self.motor_ids}
        joints = joints_from_raw(session_map, self.raw_start, self.legs)
        if set(joints) != set(self.legs):
            raise ValueError('Every walking leg needs measured yaw, hip and knee with session references')
        self.start_feet = {leg: np.asarray(p, float) for leg, p in body_fk(geometry, joints, BodyPose()).items()}
        self.neutral = {leg: p.copy() for leg, p in self.gen.neutral.items()}
        distance = max(np.linalg.norm(self.start_feet[l] - self.neutral[l]) for l in self.legs)
        self.blend_s = max(BLEND_MIN_S, distance / BLEND_SPEED_M_S)
        self.phase, self.reason = 'HOLD', ''
        self.waiting_since = clock()
        self.blend = 0.0
        self.rate = 0.0
        self.last_tick = self.last_advance = self.done_at = None
        self.stop_requested = False
        self.deadman = None                       # (held, received) from walk/deadman
        self.frozen = False
        self.pending_gait = None
        self.start_params = params
        self.gaits = (params.gait,)                # gaits verified before streaming
        self.heights = (params.body_height_m,)   # body heights verified before streaming
        self.pending_height = None
        self.adjust = None                        # (from feet, to feet, seconds, generator) while ADJUST
        self.previous = dict(self.raw_start)
        self.max_lag = (None, 0.0)

    # --- operator inputs -------------------------------------------------------------
    def request_stop(self):
        self.stop_requested = True

    def set_deadman(self, held, received):
        self.deadman = (bool(held), received)

    def deadman_released(self, now):
        """True while a fresh walk/deadman says released (teleop running, LB up or pad lost)."""
        return self.deadman is not None and not self.deadman[0] and now - self.deadman[1] <= DEADMAN_FRESH_S

    def allow_gaits(self, gaits):
        """Gaits the operator may switch to (each checked at the start height beforehand)."""
        self.gaits = tuple(sorted(set(gaits) | {self.start_params.gait}))

    def _current(self):
        gait = self.pending_gait or self.gen.params.gait
        height = self.pending_height if self.pending_height is not None else self.gen.params.body_height_m
        return gait, height

    def set_gait(self, name):
        """Switch gait once the feet settle; returns a refusal message, or None if accepted.
        Gaits are verified at the start body height only, so a switch away from the start
        gait needs the start height (and a height change needs the start gait)."""
        if name not in GAITS:
            return f'Unknown gait {name!r} (one of {sorted(GAITS)})'
        if name not in self.gaits:
            return (f'Gait {name} did not pass the startup checks with these settings '
                    f'(verified: {", ".join(self.gaits)})')
        gait, height = self._current()
        if name != self.start_params.gait and abs(height - self.start_params.body_height_m) > 1e-9:
            return f'Return to the start body height ({self.start_params.body_height_m:.3f} m) before switching gait'
        self.pending_gait = name
        return None

    def allow_heights(self, heights):
        """Body heights the D-pad may choose (each checked beforehand), always including the start."""
        self.heights = tuple(sorted(set(heights) | {self.gen.params.body_height_m}))

    def request_height_step(self, direction):
        """Raise (direction > 0) or lower the body to the next verified height, once the feet
        have settled. Body height is the feet's depth below the mounting plane, so raising the
        body means a larger height."""
        gait, current = self._current()
        if gait != self.start_params.gait:
            return None   # heights were verified for the start gait only
        options = [h for h in self.heights if (h > current + 1e-9 if direction > 0 else h < current - 1e-9)]
        if options:
            self.pending_height = min(options) if direction > 0 else max(options)
        return self.pending_height

    def height_problems(self, height):
        """Joint-limit, reach and envelope problems for the neutral stance at ``height``."""
        feet = WalkGenerator(self.geometry, replace(self.gen.params, body_height_m=height), self.legs).neutral
        try:
            solutions = solve(self.geometry, feet)
        except KinematicsError as error:
            return [f'height {height:.3f} m unreachable: {error}']
        problems = [f'height {height:.3f} m: leg {leg} {name} {value:.3f} outside [{low:.2f}, {high:.2f}]'
                    for leg, solution in solutions.items()
                    for name, value, (low, high) in zip(('yaw', 'hip_pitch', 'knee'), solution.q,
                                                         self.geometry.leg(leg).limits.as_tuple())
                    if not low <= value <= high]
        return problems + envelope_problems(self.session_map, self.raw_for(feet), self.offsets)

    # --- planning helpers --------------------------------------------------------------
    def feet_now(self):
        if self.phase in ('HOLD',):
            return self.start_feet
        if self.phase == 'ENTER':
            s = _quintic(self.blend / self.blend_s)
            return {l: self.start_feet[l] + (self.neutral[l] - self.start_feet[l]) * s for l in self.legs}
        if self.phase in ('EXIT', 'END'):
            s = _quintic(self.blend / self.blend_s)
            return {l: self.neutral[l] + (self.start_feet[l] - self.neutral[l]) * s for l in self.legs}
        if self.phase == 'ADJUST':
            start, end, seconds, _ = self.adjust
            s = _quintic(self.blend / seconds)
            return {l: start[l] + (end[l] - start[l]) * s for l in self.legs}
        return self.gen.feet

    def raw_for(self, feet):
        solutions = solve(self.geometry, feet)
        raw = {}
        for leg in self.legs:
            raw.update(zip(motor_ids(leg), self.session_map.to_raw(leg, solutions[leg].q)))
        return raw

    def blend_problems(self, samples=40):
        """Joint-limit and envelope problems along the enter/exit blend (exit is its reverse)."""
        problems = []
        for i in range(samples + 1):
            s = _quintic(i / samples)
            feet = {l: self.start_feet[l] + (self.neutral[l] - self.start_feet[l]) * s for l in self.legs}
            try:
                solutions = solve(self.geometry, feet)
            except KinematicsError as error:
                return [f'enter/exit blend unreachable: {error}']
            for leg, solution in solutions.items():
                for name, value, (low, high) in zip(('yaw', 'hip_pitch', 'knee'), solution.q,
                                                     self.geometry.leg(leg).limits.as_tuple()):
                    if not low <= value <= high:
                        problems.append(f'enter/exit: leg {leg} {name} {value:.3f} outside [{low:.2f}, {high:.2f}]')
            problems += envelope_problems(self.session_map, self.raw_for(feet), self.offsets)
        return sorted(set(problems))

    def yaw_problems(self):
        """Walking assumes each yaw was declared with its leg pointing radially."""
        problems = []
        for leg in self.legs:
            yaw_id = motor_ids(leg)[0]
            offset = self.raw_start[yaw_id] - self.session_map.references[yaw_id]
            if abs(offset) > YAW_RADIAL_TOLERANCE:
                problems.append(f'leg {leg} yaw is {offset:+.3f} rad from its declared (radial) reference')
        return problems

    # --- tick --------------------------------------------------------------------------
    def lag(self, planned, measured):
        worst = (None, 0.0)
        for mid, target in planned.items():
            actual = measured.get(mid)
            gap = float('inf') if actual is None else abs(target - actual)
            if gap > worst[1]:
                worst = (mid, gap)
        return worst

    def playback_rate(self, lag):
        ratio = lag / self.lead
        return 1.0 if ratio <= SLOW_FROM else max(0.0, (1.0 - ratio) / (1.0 - SLOW_FROM))

    def targets(self, raw, measured, dt):
        out = []
        for mid in self.motor_ids:
            position = raw[mid]
            velocity = (position - self.previous[mid]) / dt if dt > 0 else 0.0
            actual = measured.get(mid)
            if actual is not None:
                position = min(actual + self.lead, max(actual - self.lead, position))
            out.append((mid, position, velocity))
        self.previous = dict(raw)
        return out

    def advance(self, dt):
        """Move the plan forward by ``dt`` of plan time (already scaled by the rate)."""
        if self.phase == 'ENTER':
            self.blend = min(self.blend_s, self.blend + dt)
            if self.blend >= self.blend_s:
                self.phase, self.blend = 'WALK', 0.0
        elif self.phase in ('WALK', 'SETTLE'):
            if self.stop_requested and self.phase == 'WALK':
                self.phase = 'SETTLE'
            waiting = self.phase == 'SETTLE' or self.pending_gait or self.pending_height is not None
            command = (0.0, 0.0, 0.0) if waiting else self.commands()
            self.gen.step(dt, command)
            if self.pending_height is not None and self.gen.idle() and self.phase == 'WALK':
                target = WalkGenerator(self.geometry, replace(self.gen.params, body_height_m=self.pending_height),
                                       self.legs)
                distance = max(np.linalg.norm(target.neutral[l] - self.neutral[l]) for l in self.legs)
                self.adjust = ({l: p.copy() for l, p in self.gen.feet.items()}, target.neutral,
                               max(1.0, distance / BLEND_SPEED_M_S), target)
                self.phase, self.blend, self.pending_height = 'ADJUST', 0.0, None
                return
            if self.pending_gait and self.gen.idle():
                feet = self.gen.feet
                self.gen = WalkGenerator(self.geometry, replace(self.gen.params, gait=self.pending_gait), self.legs,
                                         feet={l: p.copy() for l, p in feet.items()})
                self.pending_gait = None
            if self.phase == 'SETTLE' and self.gen.idle():
                distance = max(np.linalg.norm(self.start_feet[l] - self.neutral[l]) for l in self.legs)
                self.phase, self.blend = 'EXIT', 0.0
                self.blend_s = max(BLEND_MIN_S, distance / BLEND_SPEED_M_S)
        elif self.phase == 'ADJUST':
            _, end, seconds, target = self.adjust
            self.blend = min(seconds, self.blend + dt)
            if self.blend >= seconds:
                if self.pending_gait:
                    target = WalkGenerator(self.geometry, replace(target.params, gait=self.pending_gait), self.legs)
                    self.pending_gait = None
                self.gen, self.neutral = target, {l: p.copy() for l, p in end.items()}
                self.phase, self.adjust = 'WALK', None
        elif self.phase == 'EXIT':
            self.blend = min(self.blend_s, self.blend + dt)
            if self.blend >= self.blend_s:
                self.phase = 'END'

    def step(self, safety_state, state_age, measured=None):
        """Targets to publish now, or None to stop publishing (``reason`` says why)."""
        now = self.clock()
        measured = measured or {}
        if state_age is None or state_age > STATE_STALE_S:
            return self.stop('robot_state stale; stopped publishing')
        if self.phase == 'HOLD':
            if safety_state == 'ARMED':
                self.phase, self.last_tick, self.last_advance = 'ENTER', now, now
            elif safety_state == 'FAULT':
                return self.stop('Controller FAULT before arming')
            elif now - self.waiting_since > ARM_TIMEOUT_S:
                return self.stop(f'Not armed within {ARM_TIMEOUT_S:.0f} s')
            elif self.stop_requested:
                return self.stop('Stopped before arming')
            else:
                return self.targets(self.raw_start, measured, 0.0)
        if safety_state != 'ARMED':
            return self.stop(f'Controller left ARMED ({safety_state}) while walking')
        dt, self.last_tick = now - self.last_tick, now
        try:
            planned = self.raw_for(self.feet_now())
        except KinematicsError as error:
            return self.stop(f'Walk target unreachable: {error}')
        lagging = self.lag(planned, measured)
        self.rate = self.playback_rate(lagging[1])
        if lagging[1] != float('inf') and lagging[1] > self.max_lag[1]:
            self.max_lag = lagging
        if self.phase == 'END':
            self.done_at = self.done_at or now
            if now - self.done_at >= END_HOLD_S:
                self.phase, self.reason = 'DONE', 'Walk complete'
                return None
        elif self.deadman_released(now):
            # Deadman: hold every joint where the plan is now; resume from zero velocity.
            if not self.frozen:
                self.frozen = True
                self.gen.command = (0.0, 0.0, 0.0)
            self.last_advance, self.rate = now, 0.0
        else:
            self.frozen = False
            if self.rate >= STALL_RATE:
                self.last_advance = now
            elif now - self.last_advance > STALL_S:
                return self.stop(f'Walk stalled for {STALL_S:.0f} s in {self.phase}: motor {lagging[0]} is '
                                 f'{lagging[1]:.3f} rad behind (lead limit {self.lead:.3f} rad)')
            self.advance(dt * self.rate)
        try:
            raw = self.raw_for(self.feet_now())
        except KinematicsError as error:
            return self.stop(f'Walk target unreachable: {error}')
        problems = envelope_problems(self.session_map, raw, self.offsets)
        if problems:
            return self.stop('Walk target outside the controller envelope: ' + '; '.join(problems[:3]))
        return self.targets(raw, measured, dt)

    def status(self):
        vx, vy, wz = self.gen.command
        return (f'{self.phase} {self.gen.params.gait}: v=({vx:+.3f}, {vy:+.3f}) m/s w={wz:+.3f} rad/s at '
                f'{100 * self.rate:.0f}% speed; max lag {self.max_lag[1]:.3f} rad'
                + (f' (motor {self.max_lag[0]})' if self.max_lag[0] else ''))

    def stop(self, reason):
        self.phase, self.reason = 'STOPPED', reason
        return None


def parse_script(text):
    """'4:0.03,0,0;3:0,0,0.1' -> [(4.0, (0.03, 0.0, 0.0)), (3.0, (0.0, 0.0, 0.1))]."""
    steps = []
    for part in filter(None, (p.strip() for p in text.split(';'))):
        seconds, _, twist = part.partition(':')
        values = tuple(float(v) for v in twist.split(','))
        if len(values) != 3 or not 0 < float(seconds) <= 600:
            raise ValueError(f'script step "{part}" must be seconds:vx,vy,wz')
        steps.append((float(seconds), values))
    if not steps:
        raise ValueError('empty script')
    return steps


class ScriptCommands:
    """Plays a parsed script against a clock, then asks for a stop."""

    def __init__(self, steps, on_done, clock=time.monotonic):
        self.steps, self.on_done, self.clock, self.started = steps, on_done, clock, None

    def __call__(self):
        now = self.clock()
        self.started = self.started if self.started is not None else now
        elapsed = now - self.started
        for seconds, twist in self.steps:
            if elapsed < seconds:
                return twist
            elapsed -= seconds
        self.on_done()
        return (0.0, 0.0, 0.0)


def build_parser():
    parser = argparse.ArgumentParser(prog='tocatl_walk', description=__doc__.splitlines()[0])
    defaults = WalkParams()
    for field in fields(WalkParams):
        parser.add_argument('--' + field.name.replace('_', '-'), type=type(getattr(defaults, field.name)),
                            default=getattr(defaults, field.name))
    parser.add_argument('--script', help='seconds:vx,vy,wz;... (default: follow /cmd_vel until stopped)')
    parser.add_argument('--mass-kg', type=float, default=16.0)
    parser.add_argument('--kp', type=float, default=80.0)
    parser.add_argument('--kd', type=float, default=4.0)
    parser.add_argument('--max-motor-speed-rad-s', type=float, default=5.0,
                        help="the controller's max_motor_velocity_rad_s (report/check only)")
    parser.add_argument('--geometry', default='config/robot-geometry.json')
    parser.add_argument('--joint-map', default='config/joint-map.json')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def offline_report(geometry, joint_map, params, mass, limits: ControllerLimits, max_speed):
    return feasibility(geometry, joint_map, params, mass, torque_limit_nm=limits.torque_limit or 12.0,
                       max_motor_speed_rad_s=max_speed, yaw_envelope=limits.offsets.hip_yaw)


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        params = WalkParams(**{f.name: getattr(args, f.name) for f in fields(WalkParams)}).validate()
        script = parse_script(args.script) if args.script else None
        if not (0 < args.kp <= 500 and 0 <= args.kd <= 5 and 0 < args.mass_kg <= 50):
            raise ValueError('kp must be in (0, 500], kd in [0, 5], mass in (0, 50] kg')
        geometry, joint_map = load_geometry(args.geometry), load_joint_map(args.joint_map)
    except ValueError as error:
        print(f'Refused: {error}', flush=True)
        return 2

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, Empty, Int8, String
    from std_srvs.srv import SetBool
    from tocatl_interfaces.msg import MotorCommand, MotorTarget, RobotState
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('tocatl_walk')
    latest, twist = {}, {'value': (0.0, 0.0, 0.0), 'at': 0.0}
    node.create_subscription(RobotState, 'robot_state', lambda m: latest.update(msg=m, at=time.monotonic()), 10)

    def on_twist(msg):
        twist.update(value=(msg.linear.x, msg.linear.y, msg.angular.z), at=time.monotonic())
    node.create_subscription(Twist, 'cmd_vel', on_twist, 10)
    publisher = node.create_publisher(MotorCommand, 'motor_command', 1)
    enable = node.create_client(SetBool, 'motors/enable')
    streamer, streaming, code = None, False, 1

    def disarm():
        try:
            if not enable.wait_for_service(timeout_sec=1.0):
                print('Disarm service unavailable; the command watchdog will stop the motors.', flush=True)
                return
            future = enable.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            print(f'Disarm requested: {future.result().message if future.result() else "no reply"}', flush=True)
        except Exception as error:
            print(f'Disarm request failed ({error}); the command watchdog will stop the motors.', flush=True)

    def cmd_vel():
        return twist['value'] if time.monotonic() - twist['at'] <= COMMAND_TIMEOUT_S else (0.0, 0.0, 0.0)
    node.create_subscription(Empty, 'walk/stop', lambda _: streamer and streamer.request_stop(), 1)
    def on_gait(msg):
        if streamer:
            refusal = streamer.set_gait(msg.data.strip())
            print(refusal or f'Gait -> {msg.data.strip()} (once the feet settle)', flush=True)
    node.create_subscription(String, 'walk/gait', on_gait, 1)

    def on_height(msg):
        if streamer and msg.data:
            height = streamer.request_height_step(msg.data)
            print(f'Body height -> {height:.3f} m (once the feet settle)' if height is not None else
                  f'No verified height further {"up" if msg.data > 0 else "down"} (heights are verified '
                  'for the start gait only)', flush=True)
    node.create_subscription(Int8, 'walk/height', on_height, 1)
    node.create_subscription(Bool, 'walk/deadman',
                             lambda m: streamer and streamer.set_deadman(m.data, time.monotonic()), 1)
    try:
        deadline = time.monotonic() + 5.0
        while 'msg' not in latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'msg' not in latest:
            raise ValueError('No robot_state received; is the controller running?')
        limits = query_controller(node, rclpy)
        legs, yaw_ids = actuated_legs_from_ids(limits.actuation_ids or ())
        if legs != list(LEG_IDS) or len(yaw_ids) != 6:
            raise ValueError(f'Walking needs all 18 motors actuated; the controller actuates legs {legs}, yaws {yaw_ids}')
        if args.kp > limits.max_kp or args.kd > limits.max_kd:
            raise ValueError(f'kp/kd exceed this profile (kp <= {limits.max_kp:g}, kd <= {limits.max_kd:g})')
        state = _state_dict(latest['msg'])
        missing = undeclared_yaw_legs(state, yaw_ids)
        if missing:
            raise ValueError(f'Declare yaw references first (legs radial): legs {missing} have none this session')
        session_map, _, raw_start = session_inputs(state, legs, joint_map, yaw_ids)
        report = offline_report(geometry, joint_map, params, args.mass_kg, limits, args.max_motor_speed_rad_s)
        stopper = {}
        commands = (ScriptCommands(script, lambda: stopper['s'].request_stop()) if script else cmd_vel)
        streamer = stopper['s'] = WalkStreamer(geometry, session_map, params, raw_start, commands,
                                               offsets=limits.offsets, lead=lead_for_step(limits.max_step))
        report['blend'] = dict(seconds=streamer.blend_s, problems=streamer.blend_problems())
        allowed, rejected = [], {}
        for k in range(1, HEIGHT_STEPS + 1):
            for height in (params.body_height_m - k * HEIGHT_STEP_M, params.body_height_m + k * HEIGHT_STEP_M):
                if not 0.05 <= height <= 0.5:
                    continue
                trial = replace(params, body_height_m=round(height, 4))
                problems = streamer.height_problems(trial.body_height_m)
                if not problems:
                    problems = offline_report(geometry, joint_map, trial, args.mass_kg, limits,
                                              args.max_motor_speed_rad_s)['violations']
                if problems:
                    rejected[trial.body_height_m] = problems[:2]
                else:
                    allowed.append(trial.body_height_m)
        streamer.allow_heights(allowed)
        report['heights'] = dict(allowed=list(streamer.heights), rejected=rejected)
        gaits_ok, gaits_rejected = [], {}
        for gait in sorted(set(GAITS) - {params.gait}):
            problems = offline_report(geometry, joint_map, replace(params, gait=gait), args.mass_kg, limits,
                                      args.max_motor_speed_rad_s)['violations']
            if problems:
                gaits_rejected[gait] = problems[:2]
            else:
                gaits_ok.append(gait)
        streamer.allow_gaits(gaits_ok)
        report['gaits'] = dict(allowed=list(streamer.gaits), rejected=gaits_rejected)
        report['yaw_not_radial'] = streamer.yaw_problems()
        report['controller_limits'] = dict(max_step=limits.max_step, max_kp=limits.max_kp, max_kd=limits.max_kd,
                                           torque_limit_nm=limits.torque_limit, yaw_envelope=limits.offsets.hip_yaw)
        print(json.dumps(report, indent=1, default=float), flush=True)
        problems = report['violations'] + report['blend']['problems'] + report['yaw_not_radial']
        if problems:
            raise ValueError('Walk rejected: ' + '; '.join(problems[:6]))
        if args.dry_run:
            print('Dry run: nothing published.', flush=True)
            return 0
        planned_at = time.monotonic()
        while latest['at'] <= planned_at and time.monotonic() < planned_at + 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        streaming = True
        print(f'Holding all legs. Arm to start: the body rises over {streamer.blend_s:.1f} s, then walks '
              + ('the script' if script else 'from /cmd_vel') + '. Ctrl-C once to stop and sit, twice to disarm.',
              flush=True)
        last_phase, last_report, interrupts = streamer.phase, 0.0, 0
        was_frozen = False
        while True:
            tick = time.monotonic()
            try:
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.0)
                msg = latest.get('msg')
                measured = ({m.motor_id: m.position_rad for m in msg.motors
                             if m.telemetry_fresh and math.isfinite(m.position_rad)} if msg else {})
                targets = streamer.step(msg.safety_state if msg else None,
                                        time.monotonic() - latest['at'] if msg else None, measured)
            except KeyboardInterrupt:
                interrupts += 1
                if interrupts >= 2 or streamer.phase == 'HOLD':
                    raise
                print('Stopping: settling the feet, then lowering. Ctrl-C again to disarm now.', flush=True)
                streamer.request_stop()
                continue
            if streamer.frozen != was_frozen:
                print('Deadman released: holding in place' if streamer.frozen else 'Deadman held: resuming',
                      flush=True)
                was_frozen = streamer.frozen
            if streamer.phase != last_phase:
                print(f'Phase {streamer.phase}', flush=True)
                last_phase = streamer.phase
            if streamer.phase not in ('HOLD',) and time.monotonic() - last_report >= 2.0:
                print(streamer.status(), flush=True)
                last_report = time.monotonic()
            if targets is None:
                print(streamer.reason, flush=True)
                code = 0 if streamer.phase == 'DONE' else 3
                if streamer.phase == 'DONE' or msg is not None and msg.safety_state == 'ARMED':
                    disarm()
                streaming = False
                break
            command = MotorCommand()
            command.header.stamp = node.get_clock().now().to_msg()
            command.motors = [MotorTarget(motor_id=mid, position_rad=pos, velocity_rad_s=vel, torque_nm=0.0,
                                          kp=args.kp, kd=args.kd) for mid, pos, vel in targets]
            publisher.publish(command)
            time.sleep(max(0.0, TICK_S - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print('Interrupted: disarming.', flush=True)
        if streaming:
            disarm()
        code = 130
    except ValueError as error:
        print(f'Refused: {error}', flush=True)
        code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
