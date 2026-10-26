"""Fit joint-map terms and check FK against measurements, from dashboard recordings.

Offline only: reads JSONL recordings (scripts/operator_dashboard.py "Record
telemetry") and operator-measured JSON; never touches CAN.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from tocatl_kinematics.body import leg_to_body
from tocatl_kinematics.geometry import RobotGeometry
from tocatl_kinematics.joint_map import JointMap, motor_ids
from tocatl_kinematics.leg import fk

MIN_HIP_TRAVEL_RAD = 0.2
MIN_R_SQUARED = 0.95


def load_recording(path) -> list[dict]:
    samples = []
    for line in Path(path).read_text().splitlines():
        if line.strip():
            sample = json.loads(line)
            sample['motors'] = {int(k): v for k, v in sample['motors'].items()}
            samples.append(sample)
    if not samples:
        raise ValueError(f'{path} has no samples')
    return samples


def series(samples, *ids):
    """Rows where every motor in ``ids`` has a position, as an (n, len(ids)) array."""
    rows = [[s['motors'][i]['position'] for i in ids] for s in samples
            if all(i in s['motors'] and s['motors'][i]['position'] is not None for i in ids)]
    if len(rows) < 3:
        raise ValueError(f'Fewer than 3 samples with motors {ids}')
    return np.array(rows)


def _linear(x, y):
    slope, intercept = np.polyfit(x, y, 1)
    residual = y - (slope * x + intercept)
    total = float(np.sum((y - y.mean()) ** 2))
    return float(slope), float(intercept), 1.0 - float(np.sum(residual ** 2)) / total if total > 0 else 0.0


def fit_coupling(samples, leg: int, joint_map: JointMap) -> dict:
    """Hip-only recording (knee joint held still relative to the thigh): knee raw per hip joint radian."""
    _, hip_id, knee_id = motor_ids(leg)
    data = series(samples, hip_id, knee_id)
    hip_raw, knee_raw = data[:, 0], data[:, 1]
    slope, _, r2 = _linear(hip_raw, knee_raw)
    hip = joint_map.motors[hip_id]
    travel = float(np.ptp(hip_raw))
    warnings = []
    if travel < MIN_HIP_TRAVEL_RAD:
        warnings.append(f'hip moved only {travel:.3f} raw rad; move at least {MIN_HIP_TRAVEL_RAD}')
    if r2 < MIN_R_SQUARED and abs(slope) > 0.05:
        warnings.append(f'knee does not follow hip linearly (r^2 {r2:.3f}); was the knee held still?')
    return dict(leg=leg, samples=len(data), hip_travel_raw_rad=travel, knee_travel_raw_rad=float(np.ptp(knee_raw)),
                knee_per_hip_raw=slope, r_squared=r2,
                coupling_raw_per_hip_rad=slope * hip.ratio * hip.sign, warnings=warnings)


def check_isolation(samples, moved: int, still: int) -> dict:
    """Knee-only (or yaw-only) recording: how much the motor meant to stay still moved."""
    data = series(samples, moved, still)
    moved_travel, still_travel = float(np.ptp(data[:, 0])), float(np.ptp(data[:, 1]))
    return dict(moved=moved, still=still, moved_travel_raw_rad=moved_travel, still_travel_raw_rad=still_travel,
                ratio=still_travel / moved_travel if moved_travel else float('inf'))


def fit_scale(pairs, raw_reference: float | None = None) -> dict:
    """Sign, ratio and joint angle at the reference from (raw, measured joint angle) pairs."""
    raw = np.array([p['raw_rad'] for p in pairs], float)
    angle = np.array([p['angle_rad'] for p in pairs], float)
    if len(raw) < 2 or np.ptp(raw) < 1e-3:
        raise ValueError('Need at least two pairs at different positions')
    slope, intercept, r2 = _linear(angle, raw)  # raw = slope * q + intercept, slope = sign * ratio
    result = dict(sign=1 if slope > 0 else -1, ratio=abs(slope), r_squared=r2, pairs=len(raw))
    if raw_reference is not None:
        result['q_at_reference_rad'] = (raw_reference - intercept) / slope
    return result


def window_mean(samples, t: float, ids, half_width_s: float = 0.25):
    rows = [s for s in samples if abs(s['t'] - t) <= half_width_s]
    if not rows:
        raise ValueError(f'No samples within {half_width_s} s of t={t}')
    return series(rows, *ids).mean(axis=0) if len(rows) >= 3 else np.array(
        [rows[0]['motors'][i]['position'] for i in ids])


def validate_fk(poses, geometry: RobotGeometry, joint_map: JointMap, recordings_dir=None) -> list[dict]:
    """Compare measured base_link foot positions with FK of the recorded raw positions.

    Each pose: {"leg", "foot_body_m": [x, y, z], and either "raw_rad": {id: raw} or
    "recording": file.jsonl + "t": seconds}.
    """
    report = []
    for pose in poses:
        leg = int(pose['leg'])
        ids = motor_ids(leg)
        if 'raw_rad' in pose:
            raw = [float(pose['raw_rad'][str(i)]) for i in ids]
        else:
            path = Path(recordings_dir or '.') / pose['recording']
            raw = list(window_mean(load_recording(path), float(pose['t']), ids))
        q = joint_map.to_joint(leg, raw)
        predicted = leg_to_body(fk(q, geometry.leg(leg)), geometry.leg(leg))
        measured = np.array(pose['foot_body_m'], float)
        report.append(dict(leg=leg, q=[float(v) for v in q], predicted_m=predicted.tolist(),
                           measured_m=measured.tolist(), error_m=float(np.linalg.norm(predicted - measured)),
                           label=pose.get('label', '')))
    return report
