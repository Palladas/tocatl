import math
import random

import pytest

from tocatl_kinematics.leg_model import forward, inverse, compare_measurements
from tocatl_kinematics.legacy_geometry import LegGeometry, Point3


def test_known_straight_and_right_angle_poses():
    g = LegGeometry(hip_vertical_offset_mm=10, hip_horizontal_offset_mm=20,
                    upper_link_mm=100, lower_link_mm=150)
    assert forward((0,0,0), g) == Point3(270,0,10)
    p = forward((0,0,math.pi/2), g)
    assert (p.x_mm,p.y_mm,p.z_mm) == pytest.approx((120,0,-140))


@pytest.mark.parametrize('branch', (-1,1))
def test_both_branches_reconstruct_random_reachable_targets(branch):
    rng = random.Random(42)
    for _ in range(200):
        target = forward((rng.uniform(-2,2),rng.uniform(-1,1),rng.uniform(.1,2.8)))
        reconstructed = forward(inverse(target, branch=branch))
        assert (reconstructed.x_mm,reconstructed.y_mm,reconstructed.z_mm) == pytest.approx(
            (target.x_mm,target.y_mm,target.z_mm),abs=1e-8)


@pytest.mark.parametrize('foot', (Point3(2000,0,0),Point3(45.5561,1,114.25),Point3(float('nan'),0,0)))
def test_rejects_unreachable_and_nonfinite_targets(foot):
    with pytest.raises(ValueError):
        inverse(foot)


def test_yaw_axis_is_not_assigned_an_arbitrary_angle():
    with pytest.raises(ValueError,match='yaw'):
        inverse(Point3(45.5561,0,500))


@pytest.mark.parametrize('length', (0,-1,float('inf')))
def test_rejects_invalid_geometry(length):
    with pytest.raises(ValueError):
        forward((0,0,0),LegGeometry(upper_link_mm=length))


def test_measurement_report_preserves_error_direction_and_magnitude():
    p=forward((0,0,0))
    report=compare_measurements([dict(joints_deg=[0,0,0],foot_mm=[p.x_mm-3,p.y_mm-4,p.z_mm])])
    assert report['poses'][0]['error_mm'] == pytest.approx([3,4,0])
    assert report['rms_position_error_mm'] == pytest.approx(5)
    with pytest.raises(ValueError):
        compare_measurements([])


def test_legacy_motor_mapping_roundtrip_and_hip_knee_coupling():
    from tocatl_kinematics.legacy_geometry import theta_to_legacy_motor, legacy_motor_to_theta
    joints = (.2,-.3,1.1)
    motors = theta_to_legacy_motor(*joints)
    assert legacy_motor_to_theta(*motors) == pytest.approx(joints)
    moved = theta_to_legacy_motor(joints[0],joints[1]+.1,joints[2])
    assert moved[1]-motors[1] == pytest.approx(.1)
    assert moved[2]-motors[2] == pytest.approx(-.1)
