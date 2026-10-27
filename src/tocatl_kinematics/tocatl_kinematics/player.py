"""Stream a planned feet-in-air motion to motor_controller as MotorCommand batches.

Sequence: wait for fresh robot_state with every selected leg homed and READY, read the
measured hip/knee positions, plan from there (``tocatl_kinematics.motions``), refuse on
any violation, then publish HOLD commands (targets = measured start) at 20 Hz. The
player never arms: the operator arms from the dashboard, and only then does the motion
play. Afterwards it holds the start pose for a moment, requests a disarm and exits.
Any FAULT/disarm, stale state or Ctrl-C stops the motion: the player requests a disarm
(the safe direction) before it stops publishing, so ending a motion is an operator-style
disarm, not a watchdog fault (a controller fault clears every leg's homing readiness).
``--dry-run`` plans and prints the report without publishing.

The controller accepts a MotorCommand only if it names exactly its actuation_motor_ids,
so every command carries all of them: the selected legs follow the plan, and the other
actuated legs hold their measured pose (they must be homed and READY too, which the
controller requires before arming anyway). The actuated set comes from
``--actuated-legs`` or, if omitted, the controller's ``actuation_motor_ids`` parameter.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, fields, replace
import json
import math
import sys
import time

import numpy as np

from tocatl_kinematics.geometry import load_geometry
from tocatl_kinematics.joint_map import load_joint_map, motor_ids
from tocatl_kinematics.motions import EnvelopeOffsets, PlaneMotion, plan_plane_motion

STATE_STALE_S = 0.5
ARM_TIMEOUT_S = 120.0
END_HOLD_S = 2.0
# The controller rejects a target more than max_position_step_rad (0.1) from the measured
# position, and 1 s without an accepted command trips its watchdog (a FAULT that clears
# every leg's homing). So targets never lead the measured position by more than this, and
# the plan only advances while every moving motor keeps up.
LEAD_RAD = 0.08         # used when the controller's max_position_step_rad can't be read
LEAD_FRACTION = 0.8     # of max_position_step_rad, leaving room for feedback age
STALL_S = 3.0


@dataclass(frozen=True)
class ControllerLimits:
    """What the running controller enforces; the defaults are motion-feet-air.yaml's."""
    actuation_ids: tuple = None
    max_step: float = 0.1
    max_kp: float = 30.0
    max_kd: float = 2.0
    torque_limit: float = 1.5
    offsets: EnvelopeOffsets = EnvelopeOffsets()


def actuated_legs_from_ids(motor_ids_):
    """(legs whose hip pitch and knee are both actuated, actuated yaw IDs). Yaw is only
    held, so it needs its leg's pitch/knee; half a leg is refused."""
    ids = set(motor_ids_)
    legs = sorted({mid // 10 for mid in ids})
    if not ids:
        raise ValueError('Controller actuates no motors')
    for leg in legs:
        if not {leg * 10 + 2, leg * 10 + 3} <= ids:
            raise ValueError(f'Controller actuates only part of leg {leg}: {sorted(ids)}')
    return legs, sorted(mid for mid in ids if mid % 10 == 1)


def hold_violations(session_map, raw_start, legs, offsets: EnvelopeOffsets):
    """Held legs must also sit inside the controller's home-relative envelope."""
    problems = []
    for leg in legs:
        for motor_id, (low, high) in zip(motor_ids(leg)[1:], (offsets.hip_pitch, offsets.knee)):
            offset = raw_start[motor_id] - session_map.references[motor_id]
            if not low <= offset <= high:
                problems.append(f'held motor {motor_id} sits {offset:+.3f} rad from home, outside '
                                f'the controller envelope [{low:+.2f}, {high:+.2f}]')
    return problems


def session_inputs(state: dict, legs, joint_map, yaw_ids=()):
    """References and measured start pose from a robot_state snapshot; raises if not ready.
    Actuated yaw motors (``yaw_ids``) are only held, so only their measured position is read."""
    by_leg = {leg['leg']: leg for leg in state['legs']}
    motors = {m['motor_id']: m for m in state['motors']}
    refs, start, raw_start = {}, {}, {}
    for leg in legs:
        info = by_leg.get(leg)
        if not info or not info['homed'] or not info['ready']:
            raise ValueError(f'Leg {leg} is not homed and READY in this controller session')
        refs.update(zip(info['motor_ids'], info['home_positions']))
    session_map = joint_map.with_references(refs, 'robot_state session homes')
    for leg in legs:
        _, hip_id, knee_id = motor_ids(leg)
        for motor_id in (hip_id, knee_id):
            m = motors.get(motor_id)
            if not m or not m['fresh'] or m['faults'] or m['position'] is None or not math.isfinite(m['position']):
                raise ValueError(f'Motor {motor_id} feedback is stale, faulted or missing')
            raw_start[motor_id] = m['position']
        start[leg] = session_map.raw_to_pitch_knee(leg, raw_start[hip_id], raw_start[knee_id])
    for motor_id in yaw_ids:
        m = motors.get(motor_id)
        if not m or not m['fresh'] or m['faults'] or m['position'] is None or not math.isfinite(m['position']):
            raise ValueError(f'Yaw motor {motor_id} feedback is stale, faulted or missing')
        raw_start[motor_id] = m['position']
    return session_map, start, raw_start


class Streamer:
    """Pure decision logic for what to publish at each tick; the ROS node only feeds it.

    Playback is lag-aware: plan time advances only while every moving motor would stay
    within LEAD_RAD of its measured position, so a motor that cannot keep up (torque cap,
    load, bus latency) slows the motion instead of faulting it. Every sent target is
    clamped to within LEAD_RAD of the measured position. No progress for STALL_S stops
    the motion with the lagging motor named.
    """

    def __init__(self, trajectory, kp, kd, clock=time.monotonic, hold=None, lead=LEAD_RAD):
        """``hold``: {motor_id: raw} for actuated motors that are not moving. ``lead``: how far a
        target may lead the measured position (below the controller's max_position_step_rad)."""
        if not 0 < lead <= 1.0:
            raise ValueError('lead must be in (0, 1] rad')
        self.trajectory, self.kp, self.kd, self.clock, self.lead = trajectory, kp, kd, clock, lead
        self.hold = dict(hold or {})
        if set(self.hold) & set(trajectory.raw):
            raise ValueError('A motor cannot both move and hold')
        self.motor_ids = tuple(sorted(set(trajectory.raw) | set(self.hold)))
        self.phase = 'HOLD'
        self.started = None
        self.waiting_since = clock()
        self.reason = ''
        self.progress = 0.0        # plan time reached, s
        self.last_tick = None
        self.last_advance = None
        self.finished_at = None
        self.max_lag = (None, 0.0)  # (motor_id, rad) seen while playing

    def index_at(self, plan_time):
        return min(int(np.searchsorted(self.trajectory.times, plan_time)), len(self.trajectory.times) - 1)

    def planned(self, index):
        return {mid: (float(self.hold[mid]), 0.0) if mid in self.hold else
                (float(self.trajectory.raw[mid][index]), float(self.trajectory.raw_velocity[mid][index]))
                for mid in self.motor_ids}

    def targets(self, index, measured=None, moving=True):
        """Planned targets at ``index``, clamped to within LEAD_RAD of ``measured``."""
        out = []
        for mid, (position, velocity) in self.planned(index).items():
            actual = (measured or {}).get(mid)
            if actual is not None:
                position = min(actual + self.lead, max(actual - self.lead, position))
            out.append((mid, position, velocity if moving else 0.0))
        return out

    def lag(self, index, measured):
        """Largest |planned - measured| over moving motors, as (motor_id, rad)."""
        worst = (None, 0.0)
        for mid in self.trajectory.raw:
            actual = measured.get(mid)
            gap = float('inf') if actual is None else abs(float(self.trajectory.raw[mid][index]) - actual)
            if gap > worst[1]:
                worst = (mid, gap)
        return worst

    def step(self, safety_state, state_age, measured=None):
        """Return the targets to publish now, or None to stop publishing (sets reason).

        ``measured``: {motor_id: raw position} from fresh feedback (missing = unknown)."""
        now = self.clock()
        measured = measured or {}
        if state_age is None or state_age > STATE_STALE_S:
            return self.stop('robot_state stale; stopped publishing')
        if self.phase == 'HOLD':
            if safety_state == 'ARMED':
                self.phase, self.started = 'PLAY', now
                self.last_tick = self.last_advance = now
            elif safety_state == 'FAULT':
                return self.stop('Controller FAULT before arming')
            elif now - self.waiting_since > ARM_TIMEOUT_S:
                return self.stop(f'Not armed within {ARM_TIMEOUT_S:.0f} s')
            else:
                return self.targets(0, measured, moving=False)
        if safety_state != 'ARMED':
            return self.stop(f'Controller left ARMED ({safety_state}) during motion')
        duration = float(self.trajectory.times[-1])
        dt, self.last_tick = now - self.last_tick, now
        moving = False
        if self.progress < duration:
            candidate = min(duration, self.progress + dt)
            lagging = self.lag(self.index_at(candidate), measured)
            if lagging[1] <= self.lead:
                self.progress, self.last_advance, moving = candidate, now, True
            if lagging[1] > self.max_lag[1] and lagging[1] != float('inf'):
                self.max_lag = lagging
            if now - self.last_advance > STALL_S:
                held = self.lag(self.index_at(self.progress), measured)
                return self.stop(f'Motion stalled for {STALL_S:.0f} s at {100 * self.progress / duration:.0f}%: '
                                 f'motor {lagging[0]} is {lagging[1]:.3f} rad behind its next target '
                                 f'({held[1]:.3f} rad behind the current one); it cannot follow at this load')
        elif self.finished_at is None:
            self.finished_at = now
        if self.finished_at is not None and now - self.finished_at >= END_HOLD_S:
            self.phase, self.reason = 'DONE', 'Motion complete'
            return None
        return self.targets(self.index_at(self.progress), measured, moving)

    def status(self):
        duration = float(self.trajectory.times[-1])
        return (f'progress {100 * self.progress / duration:.0f}% of {duration:.1f} s plan; '
                f'max lag so far {self.max_lag[1]:.3f} rad' + (f' (motor {self.max_lag[0]})' if self.max_lag[0] else ''))

    def stop(self, reason):
        self.phase, self.reason = 'STOPPED', reason
        return None


def build_parser():
    parser = argparse.ArgumentParser(prog='tocatl_play', description=__doc__.splitlines()[0])
    parser.add_argument('--legs', required=True, help='legs to move, comma-separated, e.g. 1,2')
    parser.add_argument('--actuated-legs',
                        help='legs the controller commands (default: read its actuation_motor_ids); '
                             'actuated legs not in --legs hold their pose')
    defaults = PlaneMotion()
    for field in fields(PlaneMotion):
        parser.add_argument('--' + field.name.replace('_', '-'), type=type(getattr(defaults, field.name)),
                            default=getattr(defaults, field.name))
    parser.add_argument('--kp', type=float, default=30.0)
    parser.add_argument('--kd', type=float, default=2.0)
    parser.add_argument('--geometry', default='config/robot-geometry.json')
    parser.add_argument('--joint-map', default='config/joint-map.json')
    parser.add_argument('--dry-run', action='store_true', help='plan and report only; publish nothing')
    return parser


def parse_legs(text):
    legs = sorted({int(part) for part in text.split(',') if part.strip()})
    if not legs or any(leg not in range(1, 7) for leg in legs):
        raise ValueError('Choose legs from 1 to 6')
    return legs


CONTROLLER_PARAMETERS = ('actuation_motor_ids', 'max_position_step_rad', 'max_motor_kp', 'max_motor_kd',
                         'motion_torque_limit_nm', 'hip_yaw_home_offset_rad', 'hip_pitch_home_offset_rad',
                         'knee_home_offset_rad')


def limits_from_values(values):
    """ControllerLimits from parameter values in CONTROLLER_PARAMETERS order."""
    ids, step, kp, kd, torque, yaw, hip, knee = values
    defaults = EnvelopeOffsets()
    pair = lambda v, default: tuple(v) if len(v) == 2 else default  # unset parameters read as [0.0]
    return ControllerLimits(actuation_ids=tuple(ids), max_step=step, max_kp=kp, max_kd=kd,
                            torque_limit=torque or None,
                            offsets=EnvelopeOffsets(hip_pitch=pair(hip, defaults.hip_pitch),
                                                    knee=pair(knee, defaults.knee), hip_yaw=pair(yaw, defaults.hip_yaw)))


def query_controller(node, rclpy, timeout=3.0):
    """Read the limits the running motor_controller enforces."""
    from rclpy.parameter_client import AsyncParameterClient
    client = AsyncParameterClient(node, 'motor_controller')
    if not client.wait_for_services(timeout_sec=timeout):
        raise ValueError('motor_controller parameters unavailable')
    future = client.get_parameters(list(CONTROLLER_PARAMETERS))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    result = future.result()
    if result is None or len(result.values) != len(CONTROLLER_PARAMETERS):
        raise ValueError('Could not read motor_controller parameters')
    v = result.values
    return limits_from_values([v[0].integer_array_value, v[1].double_value, v[2].double_value, v[3].double_value,
                               v[4].double_value, v[5].double_array_value, v[6].double_array_value,
                               v[7].double_array_value])


def lead_for_step(max_step):
    """Target lead for a controller step limit; the conservative default if unknown."""
    return LEAD_FRACTION * max_step if max_step and 0 < max_step <= 1.0 else LEAD_RAD


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        legs = parse_legs(args.legs)
        actuated = parse_legs(args.actuated_legs) if args.actuated_legs else None
        motion = PlaneMotion(**{f.name: getattr(args, f.name) for f in fields(PlaneMotion)})
        if motion.kind != 'stand':
            motion.validate()
        if not (0 < args.kp <= 500 and 0 <= args.kd <= 5):
            raise ValueError('kp must be in (0, 500] and kd in [0, 5]')
        if actuated is not None and not set(legs) <= set(actuated):
            raise ValueError(f'Legs {sorted(set(legs) - set(actuated))} are not commanded by this controller session')
        geometry, joint_map = load_geometry(args.geometry), load_joint_map(args.joint_map)
    except ValueError as error:
        print(f'Refused: {error}', flush=True)
        return 2

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import SetBool
    from tocatl_interfaces.msg import MotorCommand, MotorTarget, RobotState
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('tocatl_play')
    latest = {}

    def receive(msg):
        latest['msg'], latest['at'] = msg, time.monotonic()
    node.create_subscription(RobotState, 'robot_state', receive, 10)
    publisher = node.create_publisher(MotorCommand, 'motor_command', 1)
    enable = node.create_client(SetBool, 'motors/enable')
    streaming = False

    def disarm():
        """Request disarm while commands are still fresh; never raises."""
        try:
            if not enable.wait_for_service(timeout_sec=1.0):
                print('Disarm service unavailable; the command watchdog will stop the motors.', flush=True)
                return
            future = enable.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            result = future.result()
            print(f'Disarm requested: {result.message if result else "no reply"}', flush=True)
        except Exception as error:  # shutting down; the watchdog is the fallback
            print(f'Disarm request failed ({error}); the command watchdog will stop the motors.', flush=True)
    code = 1
    try:
        deadline = time.monotonic() + 5.0
        while 'msg' not in latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'msg' not in latest:
            raise ValueError('No robot_state received; is the controller running?')
        try:
            limits = query_controller(node, rclpy)
        except ValueError as error:
            if actuated is None:
                raise ValueError(f'{error}; pass --actuated-legs')
            limits = ControllerLimits()
            print(f'Note: {error}; using the feet-in-air profile limits', flush=True)
        yaw_ids = []
        if limits.actuation_ids:
            session_legs, yaw_ids = actuated_legs_from_ids(limits.actuation_ids)
            if actuated is None:
                actuated = session_legs
            elif set(actuated) != set(session_legs):
                raise ValueError(f'--actuated-legs {actuated} does not match the controller session {session_legs}')
        if not set(legs) <= set(actuated):
            raise ValueError(f'Legs {sorted(set(legs) - set(actuated))} are not commanded by this controller session')
        if args.kp > limits.max_kp or args.kd > limits.max_kd:
            raise ValueError(f'kp {args.kp:g} / kd {args.kd:g} exceed this profile (kp <= {limits.max_kp:g}, '
                             f'kd <= {limits.max_kd:g})')
        if limits.torque_limit:
            motion = replace(motion, torque_limit_nm=limits.torque_limit)
        motion.validate()
        lead = lead_for_step(limits.max_step)
        held = [leg for leg in actuated if leg not in legs]
        state = _state_dict(latest['msg'])
        session_map, start, raw_start = session_inputs(state, actuated, joint_map, yaw_ids)
        trajectory = plan_plane_motion(geometry, session_map, {leg: start[leg] for leg in legs}, motion,
                                       offsets=limits.offsets)
        hold = {mid: raw_start[mid] for leg in held for mid in motor_ids(leg)[1:]}
        hold.update({mid: raw_start[mid] for mid in yaw_ids})
        trajectory.violations += hold_violations(session_map, raw_start, held, limits.offsets)
        report = trajectory.summary(limits.offsets, session_map)
        report['held_legs'] = held
        report['held_yaw'] = yaw_ids
        report['controller_limits'] = dict(max_step=limits.max_step, max_kp=limits.max_kp, max_kd=limits.max_kd,
                                           torque_limit_nm=limits.torque_limit)
        print(json.dumps(report, indent=1), flush=True)
        if not trajectory.ok:
            raise ValueError('Plan rejected: ' + '; '.join(trajectory.violations))
        if args.dry_run:
            print('Dry run: nothing published.', flush=True)
            return 0
        streamer = Streamer(trajectory, args.kp, args.kd, hold=hold, lead=lead)
        print(f'Target lead {lead:.3f} rad (controller max_position_step_rad {limits.max_step}).', flush=True)
        streaming = True
        print(f'Holding legs {actuated} at the measured pose. Arm the motors to start '
              f'({report["duration_s"]:.1f} s motion of legs {legs}'
              + (f'; legs {held} keep holding' if held else '') + ').', flush=True)
        last_phase, last_report = streamer.phase, 0.0
        while True:
            tick = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            msg = latest.get('msg')
            measured = ({m.motor_id: m.position_rad for m in msg.motors
                         if m.telemetry_fresh and math.isfinite(m.position_rad)} if msg else {})
            targets = streamer.step(msg.safety_state if msg else None,
                                    time.monotonic() - latest['at'] if msg else None, measured)
            if streamer.phase != last_phase:
                print(f'Phase {streamer.phase}', flush=True)
                last_phase = streamer.phase
            if streamer.phase == 'PLAY' and time.monotonic() - last_report >= 2.0:
                print(streamer.status(), flush=True)
                last_report = time.monotonic()
            if targets is None:
                print(streamer.reason, flush=True)
                if streamer.started is not None:
                    print(streamer.status(), flush=True)
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
            time.sleep(max(0.0, motion.dt_s - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print('Interrupted.', flush=True)
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


def _state_dict(msg):
    return dict(safety_state=msg.safety_state,
                legs=[dict(leg=l.leg_id, homed=l.homed, ready=l.ready, motor_ids=list(l.motor_ids),
                           home_positions=list(l.home_positions_rad)) for l in msg.legs],
                motors=[dict(motor_id=m.motor_id, fresh=m.telemetry_fresh, faults=list(m.faults),
                             position=m.position_rad) for m in msg.motors])


if __name__ == '__main__':
    sys.exit(main())
