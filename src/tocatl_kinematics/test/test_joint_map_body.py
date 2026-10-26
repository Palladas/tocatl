import copy

import numpy as np
import pytest

from tocatl_kinematics.body import (BodyPose, body_fk, body_ik, body_to_leg, joints_from_raw, leg_to_body,
                                    neutral_stance, raw_targets)
from tocatl_kinematics.joint_map import (MissingReference, joint_map_from_dict, load_references, motor_ids,
                                         references_from_homing, references_from_robot_state_legs)
from tocatl_kinematics.leg import ik

REFS = {mid: 0.1 * mid / 7 for leg in range(1, 7) for mid in motor_ids(leg)}


def mapped(joint_map_data, **knee):
    data = copy.deepcopy(joint_map_data)
    data['defaults']['knee'].update(knee)
    data['defaults']['hip_pitch']['ratio'] = 2.8
    return joint_map_from_dict(data).with_references(REFS, 'test')


def test_seeded_map_is_unverified_and_complete(joint_map):
    assert len(joint_map.motors) == 18
    assert not any(m.verified for m in joint_map.motors.values())
    assert joint_map.motors[12].sign == -1 and joint_map.motors[13].coupling_raw_per_hip_rad == 0.0


@pytest.mark.parametrize('coupling', [0.0, 1.0, -0.87])
def test_joint_raw_round_trip_with_ratio_and_coupling(joint_map_data, coupling):
    jm = mapped(joint_map_data, coupling_raw_per_hip_rad=coupling)
    for q in [(0.1, 1.5, 2.4), (-0.3, 0.2, 1.1), (0.4, -0.8, 0.5)]:
        raw = jm.to_raw(3, q)
        assert jm.to_joint(3, raw) == pytest.approx(q, abs=1e-12)


def test_reference_pose_maps_to_reference_raw(joint_map_data):
    jm = mapped(joint_map_data, coupling_raw_per_hip_rad=1.0)
    q_ref = tuple(jm.motors[i].q_at_reference_rad for i in motor_ids(2))
    assert jm.to_raw(2, q_ref) == pytest.approx([REFS[i] for i in motor_ids(2)])


def test_coupling_moves_knee_encoder_with_hip_only(joint_map_data):
    jm = mapped(joint_map_data, coupling_raw_per_hip_rad=1.0)
    a, b = jm.to_raw(1, (0.0, 1.5, 2.0)), jm.to_raw(1, (0.0, 1.0, 2.0))
    assert b[1] - a[1] == pytest.approx(0.5 * 2.8)  # hip lowered 0.5 rad: sign -1, ratio 2.8
    assert b[2] - a[2] == pytest.approx(-0.5)  # knee joint unchanged, encoder follows hip


def test_velocity_mapping_matches_position_derivative(joint_map_data):
    jm = mapped(joint_map_data, coupling_raw_per_hip_rad=0.7)
    q, qd, dt = np.array([0.1, 0.4, 1.3]), np.array([0.2, -0.3, 0.5]), 1e-6
    numeric = (np.array(jm.to_raw(4, q + qd * dt)) - np.array(jm.to_raw(4, q))) / dt
    assert jm.velocity_to_raw(4, qd) == pytest.approx(numeric, abs=1e-6)


def test_missing_reference_and_bad_reference(joint_map):
    with pytest.raises(MissingReference):
        joint_map.to_raw(1, (0, 1, 1))
    with pytest.raises(ValueError):
        joint_map.with_references({99: 0.0}, 'x')
    with pytest.raises(ValueError):
        joint_map.with_references({12: float('nan')}, 'x')


def test_references_from_homing_and_robot_state(tmp_path):
    homing = {'version': 1, 'legs': {'1': {'motors': [{'motor_id': 12, 'home_position_rad': 0.7792},
                                                      {'motor_id': 13, 'home_position_rad': 0.7395}]}}}
    assert references_from_homing(homing) == {12: 0.7792, 13: 0.7395}
    legs = [dict(homed=True, motor_ids=[12, 13], home_positions=[0.7, 0.8]),
            dict(homed=False, motor_ids=[22, 23], home_positions=[])]
    assert references_from_robot_state_legs(legs) == {12: 0.7, 13: 0.8}
    path = tmp_path / 'yaw.json'
    path.write_text('{"source": "jig", "raw_rad": {"11": 1.25}}')
    assert load_references(path) == ({11: 1.25}, 'declared jig')


def test_body_leg_frame_round_trip(geometry):
    point = np.array([0.4, -0.2, -0.3])
    for leg in geometry.legs.values():
        assert leg_to_body(body_to_leg(point, leg), leg) == pytest.approx(point)
    leg1 = geometry.leg(1)
    mount = np.array([leg1.mount_x_m, leg1.mount_y_m, 0.0])
    outward = np.array([np.cos(leg1.mount_angle_rad), np.sin(leg1.mount_angle_rad), 0.0])
    assert body_to_leg(mount + 0.3 * outward, leg1) == pytest.approx([0.3, 0.0, 0.0])


def test_neutral_stance_at_identity_pose_equals_per_leg_ik(geometry):
    feet = neutral_stance(geometry, foot_radius_m=0.4, body_height_m=0.3)
    solutions = body_ik(geometry, feet)
    for leg_id, solution in solutions.items():
        assert solution.q == pytest.approx(ik((0.4, 0.0, -0.3), geometry.leg(leg_id)).q)
        assert solution.q[0] == pytest.approx(0.0)


def test_body_fk_inverts_body_ik_for_random_poses(wide_geometry):
    rng = np.random.default_rng(1)
    stance = neutral_stance(wide_geometry, 0.4, 0.3)
    for _ in range(25):
        pose = BodyPose(*rng.uniform(-0.03, 0.03, 3), *rng.uniform(-0.1, 0.1, 3))
        feet = {leg: p + rng.uniform(-0.03, 0.03, 3) for leg, p in stance.items()}
        joints = {leg: s.q for leg, s in body_ik(wide_geometry, feet, pose).items()}
        for leg, foot in body_fk(wide_geometry, joints, pose).items():
            assert foot == pytest.approx(feet[leg], abs=1e-9)


def test_raising_body_bends_knees_more(geometry):
    stance = neutral_stance(geometry, 0.4, 0.3)
    low = body_ik(geometry, stance, BodyPose())
    high = body_ik(geometry, stance, BodyPose(z=0.03))
    assert all(high[leg].q[2] < low[leg].q[2] for leg in stance)  # feet relatively lower -> knees straighter


def test_raw_targets_round_trip(geometry, joint_map):
    jm = joint_map.with_references(REFS, 'test')
    joints = {leg: s.q for leg, s in body_ik(geometry, neutral_stance(geometry, 0.4, 0.3)).items()}
    raw = raw_targets(jm, joints)
    assert set(raw) == set(REFS)
    recovered = joints_from_raw(jm, raw)
    for leg, q in joints.items():
        assert recovered[leg] == pytest.approx(q)
    del raw[51]
    assert 5 not in joints_from_raw(jm, raw)
