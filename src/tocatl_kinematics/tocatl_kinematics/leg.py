"""Single-leg forward/inverse kinematics in the leg_N frame (metres, radians).

Joint convention (docs/kinematics.md):
- hip_yaw: about the leg z axis, counter-clockwise positive.
- hip_pitch: 0 = thigh horizontal, positive raises the thigh.
- knee: 0 = shin in line with the thigh, positive folds the shin downward.

Unlike legacy_geometry.py, IK never clamps: unreachable or out-of-limit targets raise.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, hypot, sin

import numpy as np

from tocatl_kinematics.geometry import JOINTS, LegGeometry

AXIS_TOLERANCE_M = 1e-6


class KinematicsError(ValueError):
    pass


class Unreachable(KinematicsError):
    pass


class OutOfLimits(KinematicsError):
    def __init__(self, message, joints):
        super().__init__(message)
        self.joints = joints


@dataclass(frozen=True)
class IKSolution:
    q: tuple[float, float, float]
    reach_margin_m: float
    limit_margin_rad: float
    condition: float


def _plane(q, geometry: LegGeometry):
    """Foot (radial, vertical) in the yawed leg plane."""
    _, pitch, knee = q
    radial = geometry.coxa_radial_m + geometry.thigh_m * cos(pitch) + geometry.shin_m * cos(pitch - knee)
    vertical = geometry.coxa_vertical_m + geometry.thigh_m * sin(pitch) + geometry.shin_m * sin(pitch - knee)
    return radial, vertical


def fk(q, geometry: LegGeometry) -> np.ndarray:
    """Foot contact point in leg_N for joint angles ``(hip_yaw, hip_pitch, knee)``."""
    radial, vertical = _plane(q, geometry)
    return np.array([radial * cos(q[0]), radial * sin(q[0]), vertical])


def knee_position(q, geometry: LegGeometry) -> np.ndarray:
    radial = geometry.coxa_radial_m + geometry.thigh_m * cos(q[1])
    vertical = geometry.coxa_vertical_m + geometry.thigh_m * sin(q[1])
    return np.array([radial * cos(q[0]), radial * sin(q[0]), vertical])


def pitch_axis_position(q, geometry: LegGeometry) -> np.ndarray:
    return np.array([geometry.coxa_radial_m * cos(q[0]), geometry.coxa_radial_m * sin(q[0]),
                     geometry.coxa_vertical_m])


def jacobian(q, geometry: LegGeometry) -> np.ndarray:
    """d(foot xyz)/d(hip_yaw, hip_pitch, knee)."""
    yaw, pitch, knee = q
    radial, _ = _plane(q, geometry)
    shin = pitch - knee
    dr_dpitch = -geometry.thigh_m * sin(pitch) - geometry.shin_m * sin(shin)
    dr_dknee = geometry.shin_m * sin(shin)
    dz_dpitch = geometry.thigh_m * cos(pitch) + geometry.shin_m * cos(shin)
    dz_dknee = -geometry.shin_m * cos(shin)
    c, s = cos(yaw), sin(yaw)
    return np.array([[-radial * s, c * dr_dpitch, c * dr_dknee],
                     [radial * c, s * dr_dpitch, s * dr_dknee],
                     [0.0, dz_dpitch, dz_dknee]])


def limit_violations(q, geometry: LegGeometry, tolerance=1e-9):
    return [name for name, value, (low, high) in zip(JOINTS, q, geometry.limits.as_tuple())
            if not low - tolerance <= value <= high + tolerance]


def ik(foot, geometry: LegGeometry, branch: str = 'knee_down', check_limits: bool = True) -> IKSolution:
    """Joint angles placing the foot at ``foot`` (leg_N frame, metres).

    ``knee_down`` gives knee > 0 (shin folded below the thigh line), the walking posture.
    """
    if branch not in ('knee_down', 'knee_up'):
        raise ValueError('branch must be knee_down or knee_up')
    x, y, z = (float(v) for v in foot)
    radial = hypot(x, y)
    if radial < AXIS_TOLERANCE_M:
        raise Unreachable('Foot on the yaw axis: yaw is undefined')
    yaw = atan2(y, x)
    u = radial - geometry.coxa_radial_m
    w = z - geometry.coxa_vertical_m
    reach = hypot(u, w)
    upper, lower = geometry.thigh_m, geometry.shin_m
    margin = min(reach - abs(upper - lower), upper + lower - reach)
    if margin < 0:
        raise Unreachable(f'Leg {geometry.leg} cannot reach {reach:.4f} m from the pitch axis '
                          f'(range {abs(upper - lower):.4f}-{upper + lower:.4f} m)')
    cos_knee = max(-1.0, min(1.0, (reach * reach - upper * upper - lower * lower) / (2 * upper * lower)))
    knee = acos(cos_knee) * (1 if branch == 'knee_down' else -1)
    pitch = atan2(w, u) + atan2(lower * sin(knee), upper + lower * cos(knee))
    q = (yaw, pitch, knee)
    limit_margin = min(min(v - low, high - v) for v, (low, high) in zip(q, geometry.limits.as_tuple()))
    if check_limits:
        bad = limit_violations(q, geometry)
        if bad:
            raise OutOfLimits(f'Leg {geometry.leg} target needs {", ".join(bad)} outside limits: '
                              f'q={tuple(round(v, 4) for v in q)}', bad)
    return IKSolution(q=q, reach_margin_m=margin, limit_margin_rad=limit_margin,
                      condition=float(np.linalg.cond(jacobian(q, geometry))))
