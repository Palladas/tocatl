"""Tripod gait foot trajectories and their joint-space solution (offline planning only).

Feet are planned in base_link around a neutral stance. Alternate legs (by mount
angle, not ID) form the two tripods, half a cycle apart. Stance feet move backward
in a straight line at the body speed, so they stay fixed in the world; swing feet
follow a cycloid that starts and ends with zero velocity. The stride ramps in and
out with a smoothstep so the trajectory starts and ends in the neutral stance.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import cos, pi, sin

import numpy as np

from tocatl_kinematics.body import body_to_leg, neutral_stance
from tocatl_kinematics.geometry import LEG_IDS, RobotGeometry
from tocatl_kinematics.leg import KinematicsError, ik


@dataclass(frozen=True)
class TripodGait:
    stride_m: float = 0.06
    step_height_m: float = 0.04
    period_s: float = 8.0
    duty: float = 0.5
    heading_rad: float = 0.0
    body_height_m: float = 0.30
    foot_radius_m: float = 0.40
    cycles: int = 2
    ramp_cycles: int = 1
    dt_s: float = 0.05

    def validate(self):
        if not (0 <= self.stride_m and 0 <= self.step_height_m and self.period_s > 0 and 0.5 <= self.duty < 1
                and self.cycles >= 1 and self.ramp_cycles >= 0 and 0 < self.dt_s <= self.period_s / 20):
            raise ValueError('Invalid gait: need stride, height >= 0, period > 0, 0.5 <= duty < 1, cycles >= 1, '
                             'dt <= period / 20')

    @property
    def duration_s(self):
        return (self.cycles + 2 * self.ramp_cycles) * self.period_s


@dataclass
class GaitTrajectory:
    gait: TripodGait
    times: np.ndarray
    amplitude: np.ndarray
    feet_body: dict
    contact: dict
    joints: dict
    reach_margin_m: dict
    condition: dict
    body_travel_m: np.ndarray
    groups: tuple
    failure: str = ''

    def summary(self):
        return dict(gait=asdict(self.gait), samples=len(self.times), groups=self.groups, failure=self.failure,
                    body_travel_m=self.body_travel_m[-1].tolist() if len(self.body_travel_m) else [0.0, 0.0])


def tripod_groups(geometry: RobotGeometry, legs=LEG_IDS):
    ordered = sorted(legs, key=lambda leg: geometry.leg(leg).mount_angle_rad % (2 * pi))
    return tuple(ordered[0::2]), tuple(ordered[1::2])


def _smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def amplitude(gait: TripodGait, t):
    if gait.ramp_cycles == 0:
        return np.ones_like(t)
    ramp = gait.ramp_cycles * gait.period_s
    return np.minimum(_smoothstep(t / ramp), _smoothstep((gait.duration_s - t) / ramp))


def foot_offset(phase, duty, stride, height):
    """Along-heading offset and lift of a foot at cycle phase in [0, 1)."""
    phase = np.mod(phase, 1.0)
    stance = phase < duty
    swing_u = np.where(stance, 0.0, (phase - duty) / (1 - duty))
    along = np.where(stance, stride / 2 - stride * phase / duty,
                     -stride / 2 + stride * (swing_u - np.sin(2 * pi * swing_u) / (2 * pi)))
    lift = np.where(stance, 0.0, height * (1 - np.cos(2 * pi * swing_u)) / 2)
    return along, lift, stance


def plan(geometry: RobotGeometry, gait: TripodGait, legs=LEG_IDS, branch='knee_down') -> GaitTrajectory:
    """Foot paths and IK for every sample. Joint limits are not enforced here (see validate);
    the first unreachable sample stops planning and is reported in ``failure``."""
    gait.validate()
    times = np.arange(0.0, gait.duration_s + gait.dt_s / 2, gait.dt_s)
    amp = amplitude(gait, times)
    groups = tripod_groups(geometry, legs)
    neutral = neutral_stance(geometry, gait.foot_radius_m, gait.body_height_m, legs)
    heading = np.array([cos(gait.heading_rad), sin(gait.heading_rad), 0.0])
    speed = amp * gait.stride_m / (gait.duty * gait.period_s)
    travel = np.cumsum(speed) * gait.dt_s - speed[0] * gait.dt_s
    feet, contact, joints, margins, conditions = {}, {}, {}, {}, {}
    failure = ''
    for leg in legs:
        offset = 0.0 if leg in groups[0] else 0.5
        along, lift, stance = foot_offset(times / gait.period_s + offset, gait.duty, gait.stride_m, gait.step_height_m)
        path = neutral[leg] + np.outer(amp * along, heading) + np.outer(amp * lift, [0.0, 0.0, 1.0])
        feet[leg], contact[leg] = path, stance
        q = np.full((len(times), 3), np.nan)
        margin, condition = np.full(len(times), np.nan), np.full(len(times), np.nan)
        for i, point in enumerate(path):
            try:
                solution = ik(body_to_leg(point, geometry.leg(leg)), geometry.leg(leg), branch, check_limits=False)
            except KinematicsError as error:
                failure = failure or f'leg {leg} at t={times[i]:.2f} s: {error}'
                break
            q[i], margin[i], condition[i] = solution.q, solution.reach_margin_m, solution.condition
        joints[leg], margins[leg], conditions[leg] = q, margin, condition
    return GaitTrajectory(gait=gait, times=times, amplitude=amp, feet_body=feet, contact=contact, joints=joints,
                          reach_margin_m=margins, condition=conditions,
                          body_travel_m=np.outer(travel, heading[:2]), groups=groups, failure=failure)
