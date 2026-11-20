"""Measured TOCATL leg and body geometry, loaded from JSON with provenance.

Pure data and validation: nothing here talks to CAN or commands motors. Units are
SI (metres, radians). Frames follow docs/kinematics.md:

- ``base_link``: REP-103 body frame (x forward, y left, z up), origin at the body
  centre on the hip-yaw mounting plane.
- ``leg_N``: origin on leg N's yaw axis at the mounting plane, z up, x pointing
  radially outward at the mount angle.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from math import isfinite
from pathlib import Path

JOINTS = ('hip_yaw', 'hip_pitch', 'knee')
LEG_IDS = (1, 2, 3, 4, 5, 6)
# "measured": measured on the robot. "accepted": an existing (legacy) value the operator
# accepted as correct without re-measuring. Both count as verified; anything else does not.
TRUSTED = ('measured', 'accepted')


@dataclass(frozen=True)
class JointLimits:
    hip_yaw: tuple[float, float]
    hip_pitch: tuple[float, float]
    knee: tuple[float, float]

    def as_tuple(self):
        return (self.hip_yaw, self.hip_pitch, self.knee)


@dataclass(frozen=True)
class LegGeometry:
    """One leg. ``mount_*`` place the yaw axis in base_link; the rest rotate with yaw.

    coxa_radial_m / coxa_vertical_m: hip-pitch axis position from the yaw axis in the
    yawed leg plane. thigh_m: pitch axis to knee axis. shin_m: knee axis to foot contact.
    """
    leg: int
    mount_x_m: float
    mount_y_m: float
    mount_z_m: float
    mount_angle_rad: float
    coxa_radial_m: float
    coxa_vertical_m: float
    thigh_m: float
    shin_m: float
    limits: JointLimits
    verified: bool = False


@dataclass(frozen=True)
class RobotGeometry:
    legs: dict[int, LegGeometry]
    source: str
    unverified: tuple[str, ...]
    accepted: tuple[str, ...] = ()

    def leg(self, leg_id: int) -> LegGeometry:
        return self.legs[leg_id]


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def _range(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f'{name} must be [min, max]')
    low, high = (_number(v, name) for v in value)
    if not low < high:
        raise ValueError(f'{name} must have min < max')
    return low, high


def _field(entry, defaults, key, leg):
    """Read a field ({"value", "status", ...}) or a bare number, per leg or default -> (value, status)."""
    raw = entry.get(key, defaults.get(key))
    if raw is None:
        raise ValueError(f'leg {leg}: {key} is missing')
    if isinstance(raw, dict):
        return raw.get('value'), raw.get('status', 'unverified')
    return raw, 'unverified'


def geometry_from_dict(data: dict) -> RobotGeometry:
    if data.get('schema_version') != 1:
        raise ValueError('robot geometry schema_version must be 1')
    defaults = data.get('defaults', {})
    legs, unverified, accepted = {}, [], []
    entries = {int(k): v for k, v in data.get('legs', {}).items()}
    if set(entries) != set(LEG_IDS):
        raise ValueError('robot geometry needs exactly legs 1-6')
    for leg_id, entry in sorted(entries.items()):
        values, all_measured = {}, True
        for key in ('mount_x_m', 'mount_y_m', 'mount_z_m', 'mount_angle_rad', 'coxa_radial_m',
                    'coxa_vertical_m', 'thigh_m', 'shin_m'):
            raw, status = _field(entry, defaults, key, leg_id)
            values[key] = _number(raw, f'leg {leg_id} {key}')
            if status not in TRUSTED:
                all_measured = False
                unverified.append(f'leg {leg_id} {key}')
            elif status == 'accepted':
                accepted.append(f'leg {leg_id} {key}')
        for key in ('thigh_m', 'shin_m'):
            if values[key] <= 0:
                raise ValueError(f'leg {leg_id} {key} must be positive')
        limits = {}
        for joint in JOINTS:
            raw, status = _field(entry.get('limits_rad', {}), defaults.get('limits_rad', {}), joint, leg_id)
            limits[joint] = _range(raw, f'leg {leg_id} {joint} limits')
            if status not in TRUSTED:
                all_measured = False
                unverified.append(f'leg {leg_id} {joint} limits')
            elif status == 'accepted':
                accepted.append(f'leg {leg_id} {joint} limits')
        legs[leg_id] = LegGeometry(leg=leg_id, limits=JointLimits(**limits), verified=all_measured, **values)
    return RobotGeometry(legs=legs, source=str(data.get('source', '')), unverified=tuple(unverified),
                         accepted=tuple(accepted))


def load_geometry(path) -> RobotGeometry:
    return geometry_from_dict(json.loads(Path(path).read_text()))
