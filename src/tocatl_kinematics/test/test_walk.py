from math import cos, pi, sin

import numpy as np
import pytest

from tocatl_kinematics.walk import (GAITS, WalkGenerator, WalkParams, check, feasibility, simulate,
                                    stability_margin, support_forces)


def world(foot_body, pose):
    c, s = cos(pose[2]), sin(pose[2])
    return np.array([pose[0] + c * foot_body[0] - s * foot_body[1], pose[1] + s * foot_body[0] + c * foot_body[1]])


@pytest.mark.parametrize('gait', sorted(GAITS))
def test_stance_feet_stay_fixed_in_the_world_while_walking_and_turning(geometry, gait):
    t = simulate(geometry, WalkParams(gait=gait), [(6.0, (0.04, 0.02, 0.1))], dt=0.005)
    assert not t.failure
    for leg in t.q:
        down = t.contact[leg]
        start = None
        for i in range(1, len(t.times)):
            if down[i] and down[i - 1]:
                if start is None:
                    start = world(t.feet[leg][i - 1], t.body[i - 1])
                assert world(t.feet[leg][i], t.body[i]) == pytest.approx(start, abs=2e-3)
            else:
                start = None
    assert t.body[-1][0] > 0.03 and t.body[-1][2] > 0.1   # it actually went somewhere and turned


@pytest.mark.parametrize('gait,most_up', [('tripod', 3), ('tetrapod', 2), ('ripple', 1)])
def test_gait_lifts_its_groups_and_every_leg_steps(geometry, gait, most_up):
    t = simulate(geometry, WalkParams(gait=gait), [(2 * WalkParams().period_s, (0.03, 0.0, 0.0))])
    up = np.sum([~t.contact[leg] for leg in t.contact], axis=0)
    assert up.max() == most_up and (up == 0).any()          # short all-down overlap at handovers
    assert all((~t.contact[leg]).any() for leg in t.contact)
    if gait == 'tripod':
        assert (t.contact[1] == t.contact[3]).all() and (t.contact[1] != t.contact[2]).any()


def test_commands_are_clamped_to_speed_and_stride(geometry):
    fast = WalkGenerator(geometry, WalkParams(gait='tripod', period_s=1.0))
    assert fast.clamp((1.0, 0.0, 0.0))[0] == pytest.approx(0.05)    # speed limit
    gen = WalkGenerator(geometry, WalkParams(gait='tripod'))
    vx, vy, wz = gen.clamp((1.0, 0.0, 0.0))
    assert vx * gen.params.stance_s == pytest.approx(gen.params.max_stride_m)   # stride limit (3 s cycle)
    vx, vy, wz = gen.clamp((0.0, 0.0, 1.0))
    radius = max(np.hypot(*n[:2]) for n in gen.neutral.values())
    assert wz * radius * gen.params.stance_s == pytest.approx(gen.params.max_stride_m)   # stride limit


def test_zero_command_settles_feet_back_to_neutral_and_stops(geometry):
    gen = WalkGenerator(geometry, WalkParams(gait='ripple'))
    for _ in range(300):
        gen.step(0.02, (0.04, 0.0, 0.0))
    assert not gen.idle()
    for _ in range(int(3 * gen.params.period_s / 0.02)):
        gen.step(0.02, (0.0, 0.0, 0.0))
    assert gen.idle()
    phase = gen.phase
    gen.step(0.02, (0.0, 0.0, 0.0))
    assert gen.phase == phase                                  # stopped stepping
    assert all(np.linalg.norm(gen.feet[l] - gen.neutral[l]) < 0.002 for l in gen.legs)


def test_support_forces_and_stability_margin():
    tripod = {1: (1.0, 0.0), 3: (cos(2 * pi / 3), sin(2 * pi / 3)), 5: (cos(4 * pi / 3), sin(4 * pi / 3))}
    forces = support_forces(tripod, 10.0)
    assert sum(forces.values()) == pytest.approx(98.1) and all(f == pytest.approx(98.1 / 3) for f in forces.values())
    assert stability_margin(tripod) == pytest.approx(0.5)     # inradius of a unit triangle
    assert stability_margin(tripod, (-0.8, 0.0)) < 0
    assert min(support_forces(tripod, 10.0, (-0.8, 0.0)).values()) < 0
    assert stability_margin({1: (0, 0), 2: (1, 0)}) == float('-inf')


def test_default_walks_are_feasible_for_16_kg_and_limits_are_enforced(geometry, joint_map):
    for gait in GAITS:
        report = feasibility(geometry, joint_map, WalkParams(gait=gait), 16.0)
        assert report['ok'], (gait, report['violations'])
        assert report['peak_torque_nm'] < 0.9 * 12.0
    t = simulate(geometry, WalkParams(gait='tripod'), [(6.0, (0.05, 0.0, 0.0))])
    assert any('Nm' in v for v in check(t, geometry, joint_map, 16.0, torque_limit_nm=2.0)['violations'])
    assert any('yaw needs' in v for v in check(t, geometry, joint_map, 16.0, yaw_envelope=(-0.01, 0.01))['violations'])
    assert any('rad/s' in v for v in check(t, geometry, joint_map, 16.0, max_motor_speed_rad_s=0.2)['violations'])


def test_unreachable_stance_is_reported_not_clamped(geometry):
    t = simulate(geometry, WalkParams(body_height_m=0.5, foot_radius_m=0.6), [(1.0, (0.0, 0.0, 0.0))])
    assert t.failure and 'reach' in t.failure


def test_invalid_parameters_refused():
    for bad in (dict(gait='gallop'), dict(period_s=0.1), dict(step_height_m=0.5), dict(max_stride_m=0.0)):
        with pytest.raises(ValueError):
            WalkParams(**bad).validate()
