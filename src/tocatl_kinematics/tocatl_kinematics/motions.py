"""Feet-in-air motions in each leg's own plane: hip pitch and knee only (offline planning).

Yaw has no session reference and stays unpowered, so every motion lives in the leg's
vertical plane: foot (r, z) with r measured out from the yaw axis and z up from the
mounting plane. Motions start and end at the measured pose, segments use quintic time
scaling (zero velocity and acceleration at each end), and every segment is timed so no
motor exceeds ``max_speed_rad_s``. Nothing here talks to ROS or CAN.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import pi

import numpy as np

from tocatl_kinematics.geometry import RobotGeometry
from tocatl_kinematics.joint_map import JointMap, motor_ids
from tocatl_kinematics.leg import KinematicsError, ik

KINDS = ('nudge', 'pose', 'circle')
CONTROLLER_MAX_SPEED_RAD_S = 0.21  # max_motor_velocity_rad_s in motion-feet-air.yaml
QUINTIC_PEAK = 1.875  # peak d/dt of the quintic blend, per unit displacement per unit time


@dataclass(frozen=True)
class EnvelopeOffsets:
    """Raw-encoder bounds relative to each homed position; mirrors motion-feet-air.yaml."""
    hip_pitch: tuple[float, float] = (-0.02, 1.6)
    knee: tuple[float, float] = (-1.7, 2.0)


@dataclass(frozen=True)
class PlaneMotion:
    kind: str = 'nudge'
    # nudge: joint-space change from the start pose (negative pitch lowers the thigh,
    # negative knee unfolds it: both move away from the compressed stop).
    d_pitch_rad: float = -0.2
    d_knee_rad: float = -0.2
    # pose / circle: foot target (or circle centre) in the leg plane.
    r_m: float = 0.35
    z_m: float = -0.05
    radius_m: float = 0.03
    loops: int = 1
    hold_s: float = 1.0
    max_speed_rad_s: float = 0.1
    dt_s: float = 0.05
    branch: str = 'knee_down'

    def validate(self):
        problems = []
        if self.kind not in KINDS:
            problems.append(f'kind must be one of {KINDS}')
        if not 0 < self.max_speed_rad_s <= CONTROLLER_MAX_SPEED_RAD_S:
            problems.append(f'max_speed_rad_s must be in (0, {CONTROLLER_MAX_SPEED_RAD_S}]')
        if not 0 < self.dt_s <= 0.1:
            problems.append('dt_s must be in (0, 0.1]')
        if not 0 <= self.hold_s <= 10:
            problems.append('hold_s must be in [0, 10]')
        if self.kind == 'nudge' and not (abs(self.d_pitch_rad) <= 0.8 and abs(self.d_knee_rad) <= 0.8):
            problems.append('nudge changes are limited to 0.8 rad per joint')
        if self.kind == 'circle' and not (0.005 <= self.radius_m <= 0.1 and 1 <= self.loops <= 5):
            problems.append('circle needs radius 0.005-0.1 m and 1-5 loops')
        if self.branch not in ('knee_down', 'knee_up'):
            problems.append('branch must be knee_down or knee_up')
        if problems:
            raise ValueError('; '.join(problems))


@dataclass
class PlaneTrajectory:
    motion: PlaneMotion
    legs: tuple[int, ...]
    times: np.ndarray
    q: dict            # leg -> (n, 2) [hip_pitch, knee] joint radians
    raw: dict          # motor_id -> (n,) raw encoder radians
    raw_velocity: dict  # motor_id -> (n,) raw rad/s
    violations: list

    @property
    def ok(self):
        return not self.violations

    def summary(self, offsets: EnvelopeOffsets, joint_map: JointMap):
        per_motor = {}
        for motor_id, values in self.raw.items():
            ref = joint_map.references[motor_id]
            per_motor[str(motor_id)] = dict(
                start_rad=float(values[0]), min_offset_rad=float(values.min() - ref),
                max_offset_rad=float(values.max() - ref),
                peak_speed_rad_s=float(np.abs(self.raw_velocity[motor_id]).max()))
        return dict(ok=self.ok, violations=self.violations, motion=asdict(self.motion), legs=list(self.legs),
                    duration_s=float(self.times[-1]), samples=len(self.times), envelope_offsets=asdict(offsets),
                    motors=per_motor)


def _blend(fraction):
    s = np.clip(fraction, 0.0, 1.0)
    return s ** 3 * (10 - 15 * s + 6 * s * s)


def plane_ik(geometry: RobotGeometry, leg: int, r: float, z: float, branch: str):
    """Hip pitch and knee placing the foot at (r, z) in leg ``leg``'s plane."""
    solution = ik((r, 0.0, z), geometry.leg(leg), branch, check_limits=False)
    return solution.q[1], solution.q[2]


def plan_plane_motion(geometry: RobotGeometry, joint_map: JointMap, start: dict, motion: PlaneMotion,
                      offsets: EnvelopeOffsets = EnvelopeOffsets()) -> PlaneTrajectory:
    """Plan ``motion`` for every leg in ``start`` ({leg: (hip_pitch, knee)} measured now).

    ``joint_map`` must carry this session's raw references for the legs' hip and knee.
    Problems are collected in ``violations`` rather than raised, so callers can report
    all of them; an unreachable target is one of them.
    """
    motion.validate()
    legs = tuple(sorted(start))
    if not legs:
        raise ValueError('No legs to move')
    violations = []
    start = {leg: np.array(start[leg], float) for leg in legs}

    def to_raw(leg, q):
        return np.array(joint_map.pitch_knee_to_raw(leg, q[0], q[1]))

    # Waypoints / paths per leg, as callables of a 0..1 progress, plus their joint path for timing.
    segments = []  # list of (kind, {leg: path(u) -> (pitch, knee)})
    try:
        if motion.kind == 'nudge':
            target = {leg: start[leg] + [motion.d_pitch_rad, motion.d_knee_rad] for leg in legs}
        elif motion.kind == 'pose':
            target = {leg: np.array(plane_ik(geometry, leg, motion.r_m, motion.z_m, motion.branch)) for leg in legs}
        else:
            target = {leg: np.array(plane_ik(geometry, leg, motion.r_m + motion.radius_m, motion.z_m, motion.branch))
                      for leg in legs}
    except KinematicsError as error:
        violations.append(f'Unreachable target: {error}')
        target = None
    if target is not None:
        segments.append(('move', {leg: (lambda u, a=start[leg], b=target[leg]: a + _blend(u) * (b - a)) for leg in legs}))
        if motion.kind == 'circle':
            def circle(u, leg):
                angle = 2 * pi * motion.loops * _blend(u)
                return np.array(plane_ik(geometry, leg, motion.r_m + motion.radius_m * np.cos(angle),
                                         motion.z_m + motion.radius_m * np.sin(angle), motion.branch))
            try:
                for leg in legs:
                    circle(0.5, leg)
                segments.append(('circle', {leg: (lambda u, leg=leg: circle(u, leg)) for leg in legs}))
            except KinematicsError as error:
                violations.append(f'Circle leaves the workspace: {error}')
        segments.append(('hold', {leg: (lambda u, b=target[leg]: b) for leg in legs}))
        segments.append(('return', {leg: (lambda u, a=target[leg], b=start[leg]: a + _blend(u) * (b - a))
                                    for leg in legs}))

    times, q_rows = [0.0], {leg: [start[leg]] for leg in legs}
    for kind, paths in segments:
        if kind == 'hold':
            duration = motion.hold_s
        else:
            # Raw speed scales as 1/duration: measure the path on a fine grid, then time it.
            grid = np.linspace(0.0, 1.0, 401)
            peak = 0.0
            for leg, path in paths.items():
                raw = np.array([to_raw(leg, path(u)) for u in grid])
                peak = max(peak, float(np.abs(np.diff(raw, axis=0)).max() / (grid[1] - grid[0])))
            duration = max(1.0, peak / motion.max_speed_rad_s * 1.05)
        steps = max(1, int(np.ceil(duration / motion.dt_s)))
        for step in range(1, steps + 1):
            u = step / steps
            times.append(times[-1] + duration / steps)
            for leg, path in paths.items():
                q_rows[leg].append(np.asarray(path(u), float))

    times = np.array(times)
    q = {leg: np.array(rows) for leg, rows in q_rows.items()}
    raw, raw_velocity = {}, {}
    for leg in legs:
        hip_id, knee_id = motor_ids(leg)[1:]
        values = np.array([to_raw(leg, row) for row in q[leg]])
        raw[hip_id], raw[knee_id] = values[:, 0], values[:, 1]
        velocity = np.gradient(values, times, axis=0) if len(times) > 1 else np.zeros_like(values)
        raw_velocity[hip_id], raw_velocity[knee_id] = velocity[:, 0], velocity[:, 1]
        limits = geometry.leg(leg).limits
        for index, (name, (low, high)) in enumerate((('hip_pitch', limits.hip_pitch), ('knee', limits.knee))):
            column = q[leg][:, index]
            if column.min() < low - 1e-9 or column.max() > high + 1e-9:
                violations.append(f'leg {leg} {name} leaves its joint limits [{low:.3f}, {high:.3f}]: '
                                  f'{column.min():.3f}..{column.max():.3f}')
        for motor_id, (low, high) in ((hip_id, offsets.hip_pitch), (knee_id, offsets.knee)):
            offset = raw[motor_id] - joint_map.references[motor_id]
            if offset.min() < low - 1e-9 or offset.max() > high + 1e-9:
                violations.append(f'motor {motor_id} leaves the controller envelope [{low:+.2f}, {high:+.2f}] '
                                  f'from home: {offset.min():+.3f}..{offset.max():+.3f}')
            peak = float(np.abs(raw_velocity[motor_id]).max())
            if peak > CONTROLLER_MAX_SPEED_RAD_S:
                violations.append(f'motor {motor_id} peaks at {peak:.3f} rad/s > {CONTROLLER_MAX_SPEED_RAD_S}')
    return PlaneTrajectory(motion=motion, legs=legs, times=times, q=q, raw=raw, raw_velocity=raw_velocity,
                           violations=violations)
