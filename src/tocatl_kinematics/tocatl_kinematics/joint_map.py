"""Joint angles <-> raw CyberGear encoder radians (the values MotorCommand carries).

Per motor: ``sign`` and ``ratio`` (motor-output turns per joint turn, the old repo's
external_gear_ratio), the joint angle at the reference pose ``q_at_reference_rad``,
and for knees an optional ``coupling_raw_per_hip_rad``: knee encoder travel per
radian of hip pitch with the knee joint itself held still. The model is

    raw_hip  = ref_hip  + sign_h * ratio_h * (q_hip - q_hip_ref)
    raw_knee = ref_knee + sign_k * ratio_k * (q_knee - q_knee_ref) + coupling * (q_hip - q_hip_ref)

Raw references are per power session (homing results, or a declared yaw pose), never
stored in the static map: encoder positions are not persistent across power cycles.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
from math import isfinite
from pathlib import Path

from tocatl_kinematics.geometry import JOINTS, LEG_IDS, TRUSTED


def motor_ids(leg: int) -> tuple[int, int, int]:
    return leg * 10 + 1, leg * 10 + 2, leg * 10 + 3


class MissingReference(ValueError):
    pass


@dataclass(frozen=True)
class MotorJoint:
    motor_id: int
    sign: int
    ratio: float
    q_at_reference_rad: float
    coupling_raw_per_hip_rad: float = 0.0
    verified: bool = False


@dataclass(frozen=True)
class JointMap:
    motors: dict[int, MotorJoint]
    references: dict[int, float]
    reference_sources: dict[int, str]
    unverified: tuple[str, ...] = ()
    accepted: tuple[str, ...] = ()

    def with_references(self, references: dict[int, float], source: str) -> 'JointMap':
        for motor_id, value in references.items():
            if motor_id not in self.motors or not isinstance(value, (int, float)) or not isfinite(value):
                raise ValueError(f'Invalid raw reference for motor {motor_id}')
        return replace(self, references={**self.references, **{int(k): float(v) for k, v in references.items()}},
                       reference_sources={**self.reference_sources, **{int(k): source for k in references}})

    def _ref(self, motor_id):
        if motor_id not in self.references:
            raise MissingReference(f'Motor {motor_id} has no raw reference this session '
                                   '(home the leg, or declare a yaw reference)')
        return self.references[motor_id]

    def to_raw(self, leg: int, q) -> tuple[float, float, float]:
        yaw_id, hip_id, knee_id = motor_ids(leg)
        yaw, hip, knee = (self.motors[i] for i in (yaw_id, hip_id, knee_id))
        hip_delta = q[1] - hip.q_at_reference_rad
        return (self._ref(yaw_id) + yaw.sign * yaw.ratio * (q[0] - yaw.q_at_reference_rad),
                self._ref(hip_id) + hip.sign * hip.ratio * hip_delta,
                self._ref(knee_id) + knee.sign * knee.ratio * (q[2] - knee.q_at_reference_rad)
                + knee.coupling_raw_per_hip_rad * hip_delta)

    def to_joint(self, leg: int, raw) -> tuple[float, float, float]:
        yaw_id, hip_id, knee_id = motor_ids(leg)
        yaw, hip, knee = (self.motors[i] for i in (yaw_id, hip_id, knee_id))
        q_yaw = yaw.q_at_reference_rad + yaw.sign * (raw[0] - self._ref(yaw_id)) / yaw.ratio
        hip_delta = hip.sign * (raw[1] - self._ref(hip_id)) / hip.ratio
        q_knee = knee.q_at_reference_rad + knee.sign * (
            raw[2] - self._ref(knee_id) - knee.coupling_raw_per_hip_rad * hip_delta) / knee.ratio
        return q_yaw, hip.q_at_reference_rad + hip_delta, q_knee

    def velocity_to_raw(self, leg: int, qd) -> tuple[float, float, float]:
        yaw, hip, knee = (self.motors[i] for i in motor_ids(leg))
        return (yaw.sign * yaw.ratio * qd[0], hip.sign * hip.ratio * qd[1],
                knee.sign * knee.ratio * qd[2] + knee.coupling_raw_per_hip_rad * qd[1])


def _value(entry, defaults, key, motor_id):
    raw = entry.get(key, defaults.get(key))
    if raw is None:
        raise ValueError(f'motor {motor_id}: {key} is missing')
    if isinstance(raw, dict):
        return raw.get('value'), raw.get('status', 'unverified')
    return raw, 'unverified'


def joint_map_from_dict(data: dict) -> JointMap:
    if data.get('schema_version') != 1:
        raise ValueError('joint map schema_version must be 1')
    entries = {int(k): v for k, v in data.get('motors', {}).items()}
    expected = {mid for leg in LEG_IDS for mid in motor_ids(leg)}
    if set(entries) != expected:
        raise ValueError('joint map needs exactly the 18 inventory motors')
    defaults = data.get('defaults', {})
    motors, unverified, accepted = {}, [], []
    for motor_id, entry in sorted(entries.items()):
        joint = JOINTS[motor_id % 10 - 1]
        kind = defaults.get(joint, {})
        fields, measured = {}, True
        keys = ['sign', 'ratio', 'q_at_reference_rad'] + (['coupling_raw_per_hip_rad'] if joint == 'knee' else [])
        for key in keys:
            value, status = _value(entry, kind, key, motor_id)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                raise ValueError(f'motor {motor_id}: {key} must be a finite number')
            fields[key] = value
            if status not in TRUSTED:
                measured = False
                unverified.append(f'motor {motor_id} {key}')
            elif status == 'accepted':
                accepted.append(f'motor {motor_id} {key}')
        if fields['sign'] not in (-1, 1):
            raise ValueError(f'motor {motor_id}: sign must be -1 or 1')
        if fields['ratio'] <= 0:
            raise ValueError(f'motor {motor_id}: ratio must be positive')
        motors[motor_id] = MotorJoint(motor_id=motor_id, sign=int(fields['sign']), ratio=float(fields['ratio']),
                                      q_at_reference_rad=float(fields['q_at_reference_rad']),
                                      coupling_raw_per_hip_rad=float(fields.get('coupling_raw_per_hip_rad', 0.0)),
                                      verified=measured)
    return JointMap(motors=motors, references={}, reference_sources={}, unverified=tuple(unverified),
                    accepted=tuple(accepted))


def load_joint_map(path) -> JointMap:
    return joint_map_from_dict(json.loads(Path(path).read_text()))


def references_from_homing(data: dict) -> dict[int, float]:
    """Raw pitch/knee references from a hip_knee homing result file (``{"legs": {...}}``)."""
    refs = {}
    for record in data.get('legs', {}).values():
        for motor in record.get('motors', []):
            refs[int(motor['motor_id'])] = float(motor['home_position_rad'])
    if not refs:
        raise ValueError('Homing result has no completed legs')
    return refs


def references_from_robot_state_legs(legs) -> dict[int, float]:
    """Raw references from RobotState.legs entries that are homed this session."""
    refs = {}
    for leg in legs:
        if leg['homed']:
            refs.update(zip(leg['motor_ids'], leg['home_positions']))
    return refs


def load_references(path) -> tuple[dict[int, float], str]:
    """A homing result file, or a declared reference file ``{"source": ..., "raw_rad": {"11": ...}}``."""
    data = json.loads(Path(path).read_text())
    if 'legs' in data:
        return references_from_homing(data), f'homing {path}'
    if 'raw_rad' in data:
        return {int(k): float(v) for k, v in data['raw_rad'].items()}, f'declared {data.get("source", path)}'
    raise ValueError(f'{path} is neither a homing result nor a declared reference file')
