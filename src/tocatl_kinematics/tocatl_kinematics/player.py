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
"""
from __future__ import annotations

import argparse
from dataclasses import fields
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


def session_inputs(state: dict, legs, joint_map):
    """References and measured start pose from a robot_state snapshot; raises if not ready."""
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
    return session_map, start, raw_start


class Streamer:
    """Pure decision logic for what to publish at each tick; the ROS node only feeds it."""

    def __init__(self, trajectory, kp, kd, clock=time.monotonic):
        self.trajectory, self.kp, self.kd, self.clock = trajectory, kp, kd, clock
        self.motor_ids = tuple(sorted(trajectory.raw))
        self.phase = 'HOLD'
        self.started = None
        self.waiting_since = clock()
        self.reason = ''

    def targets(self, index):
        return [(mid, float(self.trajectory.raw[mid][index]), float(self.trajectory.raw_velocity[mid][index]))
                for mid in self.motor_ids]

    def step(self, safety_state, state_age):
        """Return the targets to publish now, or None to stop publishing (sets reason)."""
        now = self.clock()
        if state_age is None or state_age > STATE_STALE_S:
            return self.stop('robot_state stale; stopped publishing')
        if self.phase == 'HOLD':
            if safety_state == 'ARMED':
                self.phase, self.started = 'PLAY', now
            elif safety_state == 'FAULT':
                return self.stop('Controller FAULT before arming')
            elif now - self.waiting_since > ARM_TIMEOUT_S:
                return self.stop(f'Not armed within {ARM_TIMEOUT_S:.0f} s')
            else:
                return self.targets(0)
        if safety_state != 'ARMED':
            return self.stop(f'Controller left ARMED ({safety_state}) during motion')
        elapsed = now - self.started
        duration = float(self.trajectory.times[-1])
        if elapsed >= duration + END_HOLD_S:
            self.phase, self.reason = 'DONE', 'Motion complete'
            return None
        index = int(np.searchsorted(self.trajectory.times, min(elapsed, duration)))
        return self.targets(min(index, len(self.trajectory.times) - 1))

    def stop(self, reason):
        self.phase, self.reason = 'STOPPED', reason
        return None


def build_parser():
    parser = argparse.ArgumentParser(prog='tocatl_play', description=__doc__.splitlines()[0])
    parser.add_argument('--legs', required=True, help='comma-separated legs, e.g. 1,2')
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


def main(argv=None):
    args = build_parser().parse_args(argv)
    legs = parse_legs(args.legs)
    motion = PlaneMotion(**{f.name: getattr(args, f.name) for f in fields(PlaneMotion)})
    motion.validate()
    if not (0 < args.kp <= 30 and 0 <= args.kd <= 2):
        raise SystemExit('kp must be in (0, 30] and kd in [0, 2] (motion-feet-air.yaml limits)')
    geometry, joint_map = load_geometry(args.geometry), load_joint_map(args.joint_map)

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
        state = _state_dict(latest['msg'])
        session_map, start, raw_start = session_inputs(state, legs, joint_map)
        trajectory = plan_plane_motion(geometry, session_map, start, motion)
        report = trajectory.summary(EnvelopeOffsets(), session_map)
        print(json.dumps(report, indent=1), flush=True)
        if not trajectory.ok:
            raise ValueError('Plan rejected: ' + '; '.join(trajectory.violations))
        if args.dry_run:
            print('Dry run: nothing published.', flush=True)
            return 0
        streamer = Streamer(trajectory, args.kp, args.kd)
        streaming = True
        print(f'Holding legs {legs} at the measured pose. Arm the motors to start '
              f'({report["duration_s"]:.1f} s motion).', flush=True)
        last_phase = streamer.phase
        while True:
            tick = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            msg = latest.get('msg')
            targets = streamer.step(msg.safety_state if msg else None,
                                    time.monotonic() - latest['at'] if msg else None)
            if streamer.phase != last_phase:
                print(f'Phase {streamer.phase}', flush=True)
                last_phase = streamer.phase
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
