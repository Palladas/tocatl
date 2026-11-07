"""Feet-in-air performance: artistic leg cues on demand (Xbox pad via tocatl_teleop, or topics).

The robot must be supported with every foot clear of the ground (motion-feet-air.yaml).
Every cue moves hip pitch and knee in each leg's plane; yaws are only held.

Sequence:
- Start: wait for robot_state with every actuated leg homed and READY. Plan every cue once,
  from a fixed ready pose (feet at READY_R_M, READY_Z_M in each leg plane), and refuse to
  start unless at least one cue passes the planner's joint-limit, envelope and speed checks.
- REST: hold the measured pose while disarmed (the enter move is re-planned as the legs
  are moved by hand). Arming (teleop Start, or the dashboard) plays ENTER: the legs unfold
  to the ready pose.
- READY: hold the ready pose. A cue on ``perform/cue`` (std_msgs/String) plays it; each cue
  starts and ends at the ready pose. Cues that arrive while another plays are ignored.
- ``walk/stop`` (teleop B): after the current cue, EXIT folds the legs from the ready pose to a
  fixed rest pose just off the homed stops (REST_OFFSET_RAD), planned at startup, then the
  performer requests a disarm.
- Disarm or FAULT at any time stops streaming targets and returns to REST. Stale
  robot_state, a motion stall or Ctrl-C stop the node (Ctrl-C requests a disarm first).

``--dry-run`` plans and prints the cue report without publishing.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
import sys
import time

import numpy as np

from tocatl_kinematics.geometry import load_geometry
from tocatl_kinematics.joint_map import load_joint_map, motor_ids
from tocatl_kinematics.motions import (CONTROLLER_MAX_SPEED_RAD_S, EnvelopeOffsets, PlaneMotion, PlaneTrajectory,
                                       plan_plane_motion, plane_ik)
from tocatl_kinematics.player import (STATE_STALE_S, ControllerLimits, Streamer, actuated_legs_from_ids,
                                      add_gain_arguments, check_gains_against, joint_gains, lead_for_step,
                                      query_controller, session_inputs, undeclared_yaw_legs, _state_dict)

# Feet in each leg's plane: r out from the yaw axis, z up from the mounting plane. Every
# target in r 0.15-0.45 m, z -0.10..+0.15 m passed the planner on all six legs with the
# feet-in-air envelopes (2026-11-07, legacy geometry, hip gearbox 2.7:1).
READY_R_M = 0.32
READY_Z_M = 0.02
CUE_END_HOLD_S = 0.3
REPLAN_MOVED_RAD = 0.02     # re-plan the enter move once a motor has moved this far while resting
DEFAULT_SPEED_RAD_S = 1.0   # raw motor rad/s (controller cap 5)
# Gains for every performer move (motion-feet-air.yaml allows kp <= 30, kd <= 2). Lowered from
# 30 / 2 on 2026-11-07 (Pranav: 'reduce the kp gain significantly'). The lag-aware streamer
# slows a move rather than faulting if a leg trails its targets.
DEFAULT_KP = 10.0
DEFAULT_KD = 1.0
# Fold-back target: this far (raw rad) off each homed hip-pitch and knee stop, away from the
# stop (homing directions are -1, so +). A fixed pose, not the pose at arming: re-arming with the
# legs still near the ready pose made the old reversed-unfold fold-back zero length (2026-11-07).
REST_OFFSET_RAD = 0.10

# name -> PlaneMotion overrides. All circles are centred on the ready pose. Sizes raised on
# 2026-11-07 (radius 0.04 -> 0.10 m, bloom 0.42/+0.10 -> 0.45/+0.15, reach -0.08 -> -0.10):
# all pass the planner with the 2026-11-07 references; reach at z -0.12 leaves hip 12's envelope.
CUES = {
    'wave': dict(kind='wave', radius_m=0.10, loops=1, phase_deg=60.0),          # ripple around the body
    'wave_reverse': dict(kind='wave', radius_m=0.10, loops=1, phase_deg=-60.0),  # the other way round
    'circle': dict(kind='circle', radius_m=0.10, loops=1),                      # all feet together
    'alternate': dict(kind='wave', radius_m=0.10, loops=1, phase_deg=180.0),    # tripods in antiphase
    'bloom': dict(kind='pose', r_m=0.45, z_m=0.15, hold_s=1.5),                 # reach out and up
    'reach': dict(kind='pose', r_m=0.30, z_m=-0.10, hold_s=1.0),                # point the feet down
}


def cue_motion(name, speed=DEFAULT_SPEED_RAD_S, dt=0.05):
    values = dict(r_m=READY_R_M, z_m=READY_Z_M, max_speed_rad_s=speed, dt_s=dt, hold_s=0.5)
    values.update(CUES[name])
    return PlaneMotion(**values)


def ready_pose(geometry, legs, branch='knee_down'):
    """{leg: (hip_pitch, knee)} with the foot at the ready point."""
    return {leg: np.array(plane_ik(geometry, leg, READY_R_M, READY_Z_M, branch)) for leg in legs}


def plan_cues(geometry, session, legs, offsets, speed=DEFAULT_SPEED_RAD_S, dt=0.05, names=None):
    """{name: PlaneTrajectory} for each cue, planned from the ready pose."""
    start = ready_pose(geometry, legs)
    return {name: plan_plane_motion(geometry, session, start, cue_motion(name, speed, dt), offsets=offsets)
            for name in (names or CUES)}


def plan_enter(geometry, session, start, offsets, speed=DEFAULT_SPEED_RAD_S, dt=0.05):
    """One-way move from ``start`` ({leg: (pitch, knee)}) to the ready pose."""
    motion = PlaneMotion(kind='goto', r_m=READY_R_M, z_m=READY_Z_M, max_speed_rad_s=speed, dt_s=dt)
    return plan_plane_motion(geometry, session, start, motion, offsets=offsets)


def rest_pose(session, legs):
    """{leg: (hip_pitch, knee)} REST_OFFSET_RAD off each homed stop (session references)."""
    return {leg: session.raw_to_pitch_knee(leg, session.references[leg * 10 + 2] + REST_OFFSET_RAD,
                                           session.references[leg * 10 + 3] + REST_OFFSET_RAD)
            for leg in legs}


def plan_fold(geometry, session, legs, offsets, speed=DEFAULT_SPEED_RAD_S, dt=0.05):
    """Ready pose -> rest pose: the unfold from the rest pose, played backwards."""
    return reversed_trajectory(plan_enter(geometry, session, rest_pose(session, legs), offsets, speed, dt))


def reversed_trajectory(trajectory: PlaneTrajectory) -> PlaneTrajectory:
    """The same path played backwards (exit = enter reversed, ending at the pose it started from)."""
    return replace(trajectory, q={leg: values[::-1].copy() for leg, values in trajectory.q.items()},
                   raw={mid: values[::-1].copy() for mid, values in trajectory.raw.items()},
                   raw_velocity={mid: -values[::-1] for mid, values in trajectory.raw_velocity.items()},
                   loaded=None if trajectory.loaded is None else trajectory.loaded[::-1].copy())


class Performer:
    """Pure decision logic; the ROS node feeds it robot_state and operator input.

    ``plan_enter(measured) -> (trajectory, hold)`` plans the unfold from a measured pose
    ({motor_id: raw}); ``hold`` is {motor_id: raw} for actuated motors that only hold (yaw).
    ``cues``: {name: trajectory} planned from the ready pose (only feasible ones).
    ``fold``: the fold-back, ready pose -> fixed rest pose.
    """

    def __init__(self, cues, plan_enter, motor_ids_, kp, kd, lead, fold, clock=time.monotonic,
                 end_hold=CUE_END_HOLD_S):
        self.cues, self.plan_enter_fn, self.fold = dict(cues), plan_enter, fold
        self.motor_ids = tuple(sorted(motor_ids_))
        self.kp, self.kd, self.lead, self.clock, self.end_hold = kp, kd, lead, clock, end_hold
        self.phase = 'REST'
        self.streamer = None
        self.playing = None          # name of what the streamer plays ('enter', a cue, 'exit')
        self.enter = None            # (trajectory, hold, planned-from {motor_id: raw})
        self.exit = None
        self.ready_targets = None    # {motor_id: raw} held in READY
        self.pending = None
        self.stop_requested = False
        self.events = []             # ('log', text) / ('disarm', reason)
        self.reason = ''
        # Unfold only on a fresh arm: not if the controller was already armed at start, nor
        # in the ticks between a requested disarm (after EXIT or a stall) and the controller
        # leaving ARMED.
        self.need_disarm = True

    # ---- operator input -------------------------------------------------------------------
    def request_cue(self, name):
        if name not in self.cues:
            known = ', '.join(sorted(self.cues))
            self._log(f'Unknown or infeasible cue {name!r} (available: {known})')
        elif self.phase != 'READY' or self.pending is not None:
            self._log(f'Cue {name} ignored: {self._busy()}')
        elif self.stop_requested:
            self._log(f'Cue {name} ignored: folding back')
        else:
            self.pending = name

    def request_stop(self):
        if self.phase in ('ENTER', 'READY', 'CUE'):
            self.stop_requested = True
            self._log('Stop: fold back after the current move, then disarm')

    # ---- per tick ---------------------------------------------------------------------------
    def step(self, safety_state, state_age, measured):
        """Targets [(motor_id, position, velocity)] to publish now, or None to stop the node."""
        if state_age is None or state_age > STATE_STALE_S:
            self.phase, self.reason = 'STOPPED', 'robot_state stale; stopped publishing'
            return None
        if self.streamer is not None:
            return self._step_streamer(safety_state, state_age, measured)
        if safety_state != 'ARMED':
            self.need_disarm = False
            if self.phase != 'REST':
                self._log(f'Controller {safety_state}: back to REST (re-arm to unfold again)')
            self._rest()
            self._replan_enter(measured)
            return self._clamped({mid: measured.get(mid) for mid in self.motor_ids}, measured)
        if self.phase == 'REST' and self.need_disarm:
            return self._clamped({mid: measured.get(mid) for mid in self.motor_ids}, measured)
        if self.phase == 'REST':
            if self.enter is None or self._moved(self.enter[2], measured):
                self._replan_enter(measured)
            if self.enter is None:
                return self._clamped({mid: measured.get(mid) for mid in self.motor_ids}, measured)
            trajectory, hold, _ = self.enter
            self.exit = (self.fold, hold)
            self.ready_targets = {mid: float(values[-1]) for mid, values in trajectory.raw.items()}
            self.ready_targets.update(hold)
            return self._play('ENTER', 'enter', trajectory, hold, safety_state, state_age, measured)
        if self.phase == 'READY':
            if self.stop_requested:
                trajectory, hold = self.exit
                return self._play('EXIT', 'exit', trajectory, hold, safety_state, state_age, measured)
            if self.pending is not None:
                name, self.pending = self.pending, None
                hold = {mid: self.ready_targets[mid] for mid in self.motor_ids if mid not in self.cues[name].raw}
                return self._play('CUE', name, self.cues[name], hold, safety_state, state_age, measured)
            return self._clamped(self.ready_targets, measured)
        return self._clamped({mid: measured.get(mid) for mid in self.motor_ids}, measured)

    # ---- helpers ----------------------------------------------------------------------------
    def _play(self, phase, name, trajectory, hold, safety_state, state_age, measured):
        self.phase, self.playing = phase, name
        self.streamer = Streamer(trajectory, self.kp, self.kd, clock=self.clock, hold=hold, lead=self.lead,
                                 end_hold=self.end_hold)
        self._log(f'{phase}: {name} ({float(trajectory.times[-1]):.1f} s)')
        return self._step_streamer(safety_state, state_age, measured)

    def _step_streamer(self, safety_state, state_age, measured):
        targets = self.streamer.step(safety_state, state_age, measured)
        if targets is not None:
            return targets
        streamer, self.streamer = self.streamer, None
        if streamer.phase == 'DONE':
            if self.phase == 'EXIT':
                self._log('Folded back: requesting disarm')
                self.events.append(('disarm', 'performance finished'))
                self._rest()
                self.need_disarm = True
            else:
                self.phase = 'READY'
            return self.step(safety_state, state_age, measured)
        # Stopped: left ARMED, or stalled while armed (then disarm: the safe direction).
        self._log(f'{self.playing} stopped: {streamer.reason}')
        self._rest()
        if safety_state == 'ARMED':
            self.events.append(('disarm', streamer.reason))
            self.need_disarm = True
        return self._clamped({mid: measured.get(mid) for mid in self.motor_ids}, measured)

    def _rest(self):
        self.phase, self.streamer, self.playing, self.pending = 'REST', None, None, None
        self.stop_requested = False
        self.exit = self.ready_targets = None

    def _replan_enter(self, measured):
        if self.enter is not None and not self._moved(self.enter[2], measured):
            return
        if any(measured.get(mid) is None for mid in self.motor_ids):
            self.enter = None
            return
        try:
            trajectory, hold = self.plan_enter_fn(measured)
        except ValueError as error:
            self._log(f'Unfold not planned: {error}')
            self.enter = None
            return
        if not trajectory.ok:
            self._log('Unfold rejected: ' + '; '.join(trajectory.violations))
            self.enter = None
            return
        self.enter = (trajectory, hold, {mid: measured[mid] for mid in self.motor_ids})

    def _moved(self, planned_from, measured):
        return any(measured.get(mid) is None or abs(measured[mid] - planned_from[mid]) > REPLAN_MOVED_RAD
                   for mid in self.motor_ids)

    def _clamped(self, positions, measured):
        """Hold targets, clamped to within the lead of the measured position; None if unknown."""
        out = []
        for mid in self.motor_ids:
            target, actual = positions.get(mid), measured.get(mid)
            if target is None and actual is None:
                return []
            target = actual if target is None else target
            if actual is not None:
                target = min(actual + self.lead, max(actual - self.lead, target))
            out.append((mid, float(target), 0.0))
        return out

    def _busy(self):
        return {'REST': 'not armed (hold Start to arm and unfold)', 'ENTER': 'still unfolding',
                'CUE': f'{self.playing} is playing', 'EXIT': 'folding back'}.get(self.phase, self.phase)

    def _log(self, text):
        self.events.append(('log', text))

    def take_events(self):
        events, self.events = self.events, []
        return events


def build_parser():
    parser = argparse.ArgumentParser(prog='tocatl_perform', description=__doc__.splitlines()[0])
    parser.add_argument('--speed', type=float, default=DEFAULT_SPEED_RAD_S,
                        help=f'peak raw motor speed in rad/s (default {DEFAULT_SPEED_RAD_S}, cap '
                             f'{CONTROLLER_MAX_SPEED_RAD_S})')
    add_gain_arguments(parser, DEFAULT_KP, DEFAULT_KD)
    parser.add_argument('--geometry', default='config/robot-geometry.json')
    parser.add_argument('--joint-map', default='config/joint-map.json')
    parser.add_argument('--dry-run', action='store_true', help='plan and report only; publish nothing')
    return parser


def cue_report(cues):
    return {name: dict(ok=t.ok, duration_s=round(float(t.times[-1]), 1), violations=t.violations)
            for name, t in cues.items()}


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        if not 0 < args.speed <= CONTROLLER_MAX_SPEED_RAD_S:
            raise ValueError(f'--speed must be in (0, {CONTROLLER_MAX_SPEED_RAD_S}]')
        gains = joint_gains(args)
        geometry, joint_map = load_geometry(args.geometry), load_joint_map(args.joint_map)
    except ValueError as error:
        print(f'Refused: {error}', flush=True)
        return 2

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Empty, String
    from std_srvs.srv import SetBool
    from tocatl_interfaces.msg import MotorCommand, MotorTarget, RobotState
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('tocatl_perform')
    latest = {}

    def receive(msg):
        latest['msg'], latest['at'] = msg, time.monotonic()
    node.create_subscription(RobotState, 'robot_state', receive, 10)
    publisher = node.create_publisher(MotorCommand, 'motor_command', 1)
    enable = node.create_client(SetBool, 'motors/enable')
    performer = None

    def disarm():
        """Request disarm; never raises (the command watchdog is the fallback)."""
        try:
            if not enable.wait_for_service(timeout_sec=1.0):
                print('Disarm service unavailable; the command watchdog will stop the motors.', flush=True)
                return
            future = enable.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            result = future.result()
            print(f'Disarm requested: {result.message if result else "no reply"}', flush=True)
        except Exception as error:
            print(f'Disarm request failed ({error}); the command watchdog will stop the motors.', flush=True)

    code = 1
    try:
        deadline = time.monotonic() + 5.0
        while 'msg' not in latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'msg' not in latest:
            raise ValueError('No robot_state received; is the controller running?')
        limits = query_controller(node, rclpy)
        if not limits.actuation_ids:
            raise ValueError('The controller actuates no motors')
        legs, yaw_ids = actuated_legs_from_ids(limits.actuation_ids)
        check_gains_against(gains, limits, limits.actuation_ids)
        state = _state_dict(latest['msg'])
        session_map, _, _ = session_inputs(state, legs, joint_map, yaw_ids)
        missing_yaw = undeclared_yaw_legs(state, yaw_ids)
        if missing_yaw:
            raise ValueError(f'Declare yaw references first: legs {missing_yaw} have none this session')
        cues = plan_cues(geometry, session_map, legs, limits.offsets, args.speed)
        fold = plan_fold(geometry, session_map, legs, limits.offsets, args.speed)
        report = cue_report(cues)
        print(json.dumps(dict(legs=legs, held_yaw=yaw_ids, ready_pose_m=[READY_R_M, READY_Z_M],
                              speed_rad_s=args.speed,
                              gains={j: dict(kp=gains[i][0], kd=gains[i][1]) for i, j in enumerate(('yaw', 'hip', 'knee'), 1)},
                              fold_back=dict(ok=fold.ok, duration_s=round(float(fold.times[-1]), 1),
                                             rest_offset_rad=REST_OFFSET_RAD, violations=fold.violations),
                              cues=report), indent=1), flush=True)
        if not fold.ok:
            raise ValueError('Fold-back rejected: ' + '; '.join(fold.violations))
        feasible = {name: t for name, t in cues.items() if t.ok}
        if not feasible:
            raise ValueError('No cue passed the planner checks')
        if args.dry_run:
            print('Dry run: nothing published.', flush=True)
            return 0

        pitch_knee_ids = [mid for leg in legs for mid in motor_ids(leg)[1:]]

        def enter_from(measured):
            start = {leg: session_map.raw_to_pitch_knee(leg, measured[leg * 10 + 2], measured[leg * 10 + 3])
                     for leg in legs}
            return (plan_enter(geometry, session_map, start, limits.offsets, args.speed),
                    {mid: measured[mid] for mid in yaw_ids})
        performer = Performer(feasible, enter_from, pitch_knee_ids + list(yaw_ids), args.kp, args.kd,
                              lead_for_step(limits.max_step), fold)
        node.create_subscription(String, 'perform/cue', lambda m: performer.request_cue(m.data.strip()), 10)
        node.create_subscription(Empty, 'walk/stop', lambda m: performer.request_stop(), 10)
        print(f'Ready: cues {sorted(feasible)}. Arm (teleop: hold Start 1 s) to unfold to the ready pose; '
              'B folds back and disarms; Back disarms now.', flush=True)
        last_phase = None
        while True:
            tick = time.monotonic()
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.0)
            msg = latest.get('msg')
            measured = ({m.motor_id: m.position_rad for m in msg.motors
                         if m.telemetry_fresh and math.isfinite(m.position_rad)} if msg else {})
            targets = performer.step(msg.safety_state if msg else None,
                                     time.monotonic() - latest['at'] if msg else None, measured)
            for kind, text in performer.take_events():
                print(text if kind == 'log' else f'Disarm: {text}', flush=True)
                if kind == 'disarm':
                    disarm()
            if performer.phase != last_phase:
                print(f'Phase {performer.phase}', flush=True)
                last_phase = performer.phase
            if targets is None:
                print(performer.reason, flush=True)
                code = 3
                break
            if targets:
                command = MotorCommand()
                command.header.stamp = node.get_clock().now().to_msg()
                command.motors = [MotorTarget(motor_id=mid, position_rad=pos, velocity_rad_s=vel, torque_nm=0.0,
                                              kp=gains[mid % 10][0], kd=gains[mid % 10][1])
                                  for mid, pos, vel in targets]
                publisher.publish(command)
            time.sleep(max(0.0, 0.05 - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print('Interrupted.', flush=True)
        if performer is not None and latest.get('msg') is not None and latest['msg'].safety_state == 'ARMED':
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
