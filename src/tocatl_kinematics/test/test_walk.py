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
    assert fast.clamp((1.0, 0.0, 0.0))[0] == pytest.approx(fast.params.max_vx_m_s)    # speed limit
    gen = WalkGenerator(geometry, WalkParams(gait='tripod', max_stride_m=0.06))   # stride binds before speed
    vx, vy, wz = gen.clamp((1.0, 0.0, 0.0))
    assert vx * gen.params.stance_s == pytest.approx(gen.params.max_stride_m)   # stride limit (3 s cycle)
    vx, vy, wz = gen.clamp((0.0, 0.0, 1.0))
    radius = max(np.hypot(*n[:2]) for n in gen.neutral.values())
    assert wz * radius * gen.params.stance_s == pytest.approx(gen.params.max_stride_m)   # stride limit


def test_diagonal_commands_are_clamped_to_the_checked_ellipse(geometry):
    # feasibility() walks the ellipse (max_vx cos a, max_vy sin a); a stick in the corner must
    # not exceed it (per-axis clamping gave sqrt(2) x the speed).
    gen = WalkGenerator(geometry, WalkParams(gait='tripod', max_stride_m=1.0))
    p = gen.params
    vx, vy, _ = gen.clamp((1.0, 1.0, 0.0))
    assert vx == pytest.approx(vy) and (vx / p.max_vx_m_s) ** 2 + (vy / p.max_vy_m_s) ** 2 == pytest.approx(1.0)
    assert gen.clamp((0.05, 0.05, 0.0))[:2] == pytest.approx((0.05, 0.05))     # inside: untouched
    no_side = WalkGenerator(geometry, WalkParams(gait='tripod', max_vy_m_s=0.0))
    assert no_side.clamp((0.0, 1.0, 0.0))[1] == 0.0


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


# Walk parameters verified on the robot 2026-11-07, before the Pi defaults were widened (2026-11-13).
VERIFIED = dict(period_s=3.0, step_height_m=0.08, max_stride_m=0.06, max_vx_m_s=0.05, max_vy_m_s=0.05)


def test_verified_walks_are_feasible_for_16_kg_and_limits_are_enforced(geometry, joint_map):
    # 0.08 m steps (2026-11-07): tripod and tetrapod at these values; ripple's 15% swing needs a
    # longer cycle (or a lower step) to stay under the 5 rad/s hip motor speed.
    for params in (WalkParams(gait='tripod', **VERIFIED), WalkParams(gait='tetrapod', **VERIFIED),
                   WalkParams(gait='ripple', **{**VERIFIED, 'period_s': 5.0}),
                   WalkParams(gait='ripple', **{**VERIFIED, 'step_height_m': 0.04})):
        report = feasibility(geometry, joint_map, params, 16.0)
        assert report['ok'], (params, report['violations'])
        assert report['peak_torque_nm'] < 0.9 * 12.0
    ripple = feasibility(geometry, joint_map, WalkParams(gait='ripple', **VERIFIED), 16.0)
    assert not ripple['ok'] and any('rad/s' in v for v in ripple['violations'])



def test_default_walk_is_feasible_with_the_standing_profile(geometry, joint_map):
    # 0.2 m/s tripod (2026-11-13): feasible against motion-standing.yaml (12 Nm, 6.5 rad/s,
    # yaw +/-1.0) but not under the old 5 rad/s cap, which the hip-pitch motor would exceed.
    params = WalkParams()
    assert params.gait == 'tripod'
    report = feasibility(geometry, joint_map, params, 16.0, torque_limit_nm=12.0,
                         max_motor_speed_rad_s=6.5, yaw_envelope=(-1.0, 1.0))
    assert report['ok'], report['violations']
    old = feasibility(geometry, joint_map, params, 16.0, torque_limit_nm=12.0,
                      max_motor_speed_rad_s=5.0, yaw_envelope=(-1.0, 1.0))
    assert not old['ok'] and any('rad/s' in v for v in old['violations'])
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


def test_override_flag_defaults_off():
    from tocatl_kinematics.walk_player import build_parser
    assert not build_parser().parse_args([]).override
    assert build_parser().parse_args(['--override']).override
