"""Body-level kinematics: body pose + foot positions -> every leg's joint angles.

The world frame is fixed to the ground, z up. ``BodyPose`` places base_link in it
(roll/pitch/yaw applied as Rz(yaw) Ry(pitch) Rx(roll)).
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin

import numpy as np

from tocatl_kinematics.geometry import LEG_IDS, LegGeometry, RobotGeometry
from tocatl_kinematics.joint_map import JointMap, motor_ids
from tocatl_kinematics.leg import IKSolution, fk, ik


@dataclass(frozen=True)
class BodyPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def rotation(self) -> np.ndarray:
        cr, sr, cp, sp, cy, sy = cos(self.roll), sin(self.roll), cos(self.pitch), sin(self.pitch), cos(self.yaw), sin(self.yaw)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])

    def translation(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


def _mount_rotation(leg: LegGeometry) -> np.ndarray:
    c, s = cos(leg.mount_angle_rad), sin(leg.mount_angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _mount(leg: LegGeometry) -> np.ndarray:
    return np.array([leg.mount_x_m, leg.mount_y_m, leg.mount_z_m])


def body_to_leg(point_body, leg: LegGeometry) -> np.ndarray:
    return _mount_rotation(leg).T @ (np.asarray(point_body, float) - _mount(leg))


def leg_to_body(point_leg, leg: LegGeometry) -> np.ndarray:
    return _mount_rotation(leg) @ np.asarray(point_leg, float) + _mount(leg)


def neutral_stance(geometry: RobotGeometry, foot_radius_m: float, body_height_m: float, legs=LEG_IDS):
    """Feet in base_link: ``foot_radius_m`` out from each yaw axis along its mount angle,
    ``body_height_m`` below the mounting plane."""
    feet = {}
    for leg_id in legs:
        leg = geometry.leg(leg_id)
        feet[leg_id] = _mount(leg) + np.array([foot_radius_m * cos(leg.mount_angle_rad),
                                               foot_radius_m * sin(leg.mount_angle_rad), -body_height_m - leg.mount_z_m])
    return feet


def body_ik(geometry: RobotGeometry, feet_world: dict, pose: BodyPose = BodyPose(),
            branch: str = 'knee_down', check_limits: bool = True) -> dict[int, IKSolution]:
    """Joint solutions for every leg in ``feet_world`` (world-frame foot positions)."""
    rotation, translation = pose.rotation(), pose.translation()
    solutions = {}
    for leg_id, foot in feet_world.items():
        foot_body = rotation.T @ (np.asarray(foot, float) - translation)
        leg = geometry.leg(leg_id)
        solutions[leg_id] = ik(body_to_leg(foot_body, leg), leg, branch, check_limits)
    return solutions


def body_fk(geometry: RobotGeometry, joints: dict, pose: BodyPose = BodyPose()) -> dict[int, np.ndarray]:
    """World-frame foot positions for ``{leg: (hip_yaw, hip_pitch, knee)}``."""
    rotation, translation = pose.rotation(), pose.translation()
    return {leg_id: rotation @ leg_to_body(fk(q, geometry.leg(leg_id)), geometry.leg(leg_id)) + translation
            for leg_id, q in joints.items()}


def raw_targets(joint_map: JointMap, joints: dict) -> dict[int, float]:
    """``{motor_id: raw rad}`` for ``{leg: q}``; raises MissingReference without session references."""
    targets = {}
    for leg_id, q in joints.items():
        targets.update(zip(motor_ids(leg_id), joint_map.to_raw(leg_id, q)))
    return targets


def joints_from_raw(joint_map: JointMap, raw: dict, legs=LEG_IDS) -> dict:
    """Joint angles for legs whose three motors all have raw readings and references."""
    joints = {}
    for leg_id in legs:
        ids = motor_ids(leg_id)
        if all(raw.get(i) is not None and i in joint_map.references for i in ids):
            joints[leg_id] = joint_map.to_joint(leg_id, [raw[i] for i in ids])
    return joints
