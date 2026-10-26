"""Check a planned trajectory against joint limits and the controller's command limits.

Defaults mirror src/tocatl_bringup/config/hardware.yaml (max_motor_velocity_rad_s 0.21,
max_position_step_rad 0.1) and the out-of-service motor 51. Limits are reported, not
widened: a gait that needs more speed reports the slowest period that would fit.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from tocatl_kinematics.geometry import JOINTS, RobotGeometry
from tocatl_kinematics.joint_map import JointMap, motor_ids


def _finite(values, reduce):
    values = values[np.isfinite(values)]
    return float(reduce(values)) if len(values) else None


@dataclass(frozen=True)
class CommandLimits:
    max_motor_velocity_rad_s: float = 0.21
    max_position_step_rad: float = 0.1
    command_period_s: float = 0.05
    min_reach_margin_m: float = 0.01
    out_of_service: tuple[int, ...] = (51,)
    still_tolerance_rad: float = 1e-3


def validate_trajectory(trajectory, geometry: RobotGeometry, joint_map: JointMap,
                        limits: CommandLimits = CommandLimits()) -> dict:
    """Return {"ok", "violations", "per_motor", "min_feasible_period_s", ...}."""
    violations = []
    if trajectory.failure:
        violations.append(f'Unreachable: {trajectory.failure}')
    dt = float(trajectory.times[1] - trajectory.times[0])
    per_motor, worst_speed = {}, 0.0
    for leg, q in trajectory.joints.items():
        if np.isnan(q).any():
            continue
        for index, (name, (low, high)) in enumerate(zip(JOINTS, geometry.leg(leg).limits.as_tuple())):
            bad = (q[:, index] < low) | (q[:, index] > high)
            if bad.any():
                first = int(np.argmax(bad))
                violations.append(f'leg {leg} {name} outside [{low:.3f}, {high:.3f}] from t={trajectory.times[first]:.2f} s '
                                  f'(reaches {q[bad, index].min() if q[first, index] < low else q[bad, index].max():.3f})')
        qd = np.gradient(q, dt, axis=0)
        raw_velocity = np.array([joint_map.velocity_to_raw(leg, row) for row in qd])
        raw_travel = np.cumsum(raw_velocity * dt, axis=0)
        for index, motor_id in enumerate(motor_ids(leg)):
            peak = float(np.abs(raw_velocity[:, index]).max())
            travel = float(np.ptp(raw_travel[:, index]))
            per_motor[motor_id] = dict(peak_velocity_rad_s=peak, travel_rad=travel,
                                       max_step_rad=peak * limits.command_period_s)
            if motor_id in limits.out_of_service:
                if travel > limits.still_tolerance_rad:
                    violations.append(f'motor {motor_id} is out of service but this gait moves it {travel:.3f} rad')
                continue
            worst_speed = max(worst_speed, peak)
            if peak > limits.max_motor_velocity_rad_s:
                violations.append(f'motor {motor_id} peaks at {peak:.3f} rad/s > {limits.max_motor_velocity_rad_s} rad/s')
            if peak * limits.command_period_s > limits.max_position_step_rad:
                violations.append(f'motor {motor_id} steps {peak * limits.command_period_s:.3f} rad per command')
        margin = float(np.nanmin(trajectory.reach_margin_m[leg]))
        if margin < limits.min_reach_margin_m:
            violations.append(f'leg {leg} comes within {margin * 1000:.1f} mm of its reach limit')
    period = trajectory.gait.period_s
    unverified = len(geometry.unverified) + len(joint_map.unverified)
    return dict(ok=not violations, violations=violations, per_motor=per_motor,
                peak_motor_velocity_rad_s=worst_speed,
                min_feasible_period_s=period * worst_speed / limits.max_motor_velocity_rad_s if worst_speed else 0.0,
                min_reach_margin_m=_finite(np.concatenate(list(trajectory.reach_margin_m.values())), min),
                max_condition=_finite(np.concatenate(list(trajectory.condition.values())), max),
                unverified_values=unverified,
                verified=unverified == 0)
