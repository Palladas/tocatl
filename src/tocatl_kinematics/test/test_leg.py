import copy
from itertools import product

import numpy as np
import pytest

from tocatl_kinematics.geometry import geometry_from_dict
from tocatl_kinematics.leg import OutOfLimits, Unreachable, fk, ik, jacobian


def test_committed_geometry_is_accepted_legacy(geometry):
    assert set(geometry.legs) == {1, 2, 3, 4, 5, 6}
    assert all(leg.verified for leg in geometry.legs.values()) and not geometry.unverified
    assert 'leg 1 thigh_m' in geometry.accepted
    assert geometry.leg(1).thigh_m == pytest.approx(0.3303455, abs=1e-6)


def test_unknown_status_is_unverified(geometry_data):
    data = copy.deepcopy(geometry_data)
    data['defaults']['shin_m']['status'] = 'guess'
    geometry = geometry_from_dict(data)
    assert 'leg 2 shin_m' in geometry.unverified and not geometry.leg(2).verified


@pytest.mark.parametrize('change', [
    lambda d: d['legs'].pop('6'),
    lambda d: d['defaults'].__setitem__('thigh_m', -0.1),
    lambda d: d['defaults'].__setitem__('shin_m', float('nan')),
    lambda d: d['defaults']['limits_rad'].__setitem__('knee', [1.0, 0.5]),
    lambda d: d.__setitem__('schema_version', 2),
])
def test_invalid_geometry_rejected(geometry_data, change):
    data = copy.deepcopy(geometry_data)
    change(data)
    with pytest.raises(ValueError):
        geometry_from_dict(data)


def test_measured_status_marks_leg_verified(geometry_data):
    data = copy.deepcopy(geometry_data)
    for value in list(data['defaults'].values()) + list(data['defaults']['limits_rad'].values()):
        if isinstance(value, dict) and 'value' in value:
            value['status'] = 'measured'
    for leg in data['legs'].values():
        for value in leg.values():
            value['status'] = 'measured'
    assert geometry_from_dict(data).leg(3).verified


def test_fk_known_pose(wide_geometry):
    leg = wide_geometry.leg(1)
    foot = fk((0.0, 0.0, 0.0), leg)  # leg straight out horizontally
    assert foot == pytest.approx([leg.coxa_radial_m + leg.thigh_m + leg.shin_m, 0.0, leg.coxa_vertical_m])
    foot = fk((np.pi / 2, 0.0, np.pi / 2), leg)  # yawed left, shin straight down
    assert foot == pytest.approx([0.0, leg.coxa_radial_m + leg.thigh_m, leg.coxa_vertical_m - leg.shin_m])


@pytest.mark.parametrize('branch', ['knee_down', 'knee_up'])
def test_fk_ik_round_trip_grid(wide_geometry, branch):
    leg = wide_geometry.leg(2)
    sign = 1 if branch == 'knee_down' else -1
    for yaw, pitch, knee in product(np.linspace(-1.2, 1.2, 5), np.linspace(-1.0, 1.2, 5), np.linspace(0.2, 2.6, 5)):
        q = (yaw, pitch, sign * knee)
        if fk((0.0, pitch, sign * knee), leg)[0] < 0.05:
            continue  # foot behind the yaw axis: yaw + pi is an equivalent solution, never a stance
        solution = ik(fk(q, leg), leg, branch)
        assert solution.q == pytest.approx(q, abs=1e-9)
        assert fk(solution.q, leg) == pytest.approx(fk(q, leg), abs=1e-12)


def test_unreachable_raises_instead_of_clamping(geometry):
    leg = geometry.leg(1)
    with pytest.raises(Unreachable):
        ik((2.0, 0.0, -0.3), leg)  # beyond full extension
    with pytest.raises(Unreachable):
        ik((leg.coxa_radial_m + 0.01, 0.0, leg.coxa_vertical_m), leg)  # inside the minimum reach
    with pytest.raises(Unreachable):
        ik((0.0, 0.0, -0.3), leg)  # on the yaw axis


def test_out_of_limits_raises_with_joint_names(geometry):
    leg = geometry.leg(1)
    yaw = leg.limits.hip_yaw[1] + 0.2
    target = fk((yaw, 0.0, 1.2), leg)  # yaw just past its limit
    with pytest.raises(OutOfLimits) as error:
        ik(target, leg)
    assert error.value.joints == ['hip_yaw']
    assert ik(target, leg, check_limits=False).q == pytest.approx((yaw, 0.0, 1.2))


def test_jacobian_matches_finite_differences(wide_geometry):
    leg = wide_geometry.leg(4)
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform([-1, -1, 0.1], [1, 1, 2.5])
        numeric = np.column_stack([(fk(q + dq, leg) - fk(q - dq, leg)) / 2e-6 for dq in np.eye(3) * 1e-6])
        assert jacobian(q, leg) == pytest.approx(numeric, abs=1e-6)


def test_solution_reports_margins(geometry):
    leg = geometry.leg(1)
    q = (0.0, 0.3, 1.2)
    solution = ik(fk(q, leg), leg)
    assert solution.reach_margin_m > 0
    expected = min(min(v - low, high - v) for v, (low, high) in zip(q, leg.limits.as_tuple()))
    assert solution.limit_margin_rad == pytest.approx(expected)
    assert 1 <= solution.condition < 100
