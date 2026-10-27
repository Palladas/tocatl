from math import pi

import numpy as np
import pytest

from tocatl_kinematics.joint_map import motor_ids
from tocatl_kinematics.leg import fk
from tocatl_kinematics.motions import (CONTROLLER_MAX_SPEED_RAD_S, EnvelopeOffsets, PlaneMotion,
                                       plan_plane_motion)
from tocatl_kinematics.player import ARM_TIMEOUT_S, END_HOLD_S, STATE_STALE_S, Streamer, parse_legs, session_inputs

# Session homes read from the dashboard on 2026-10-26 (legs 1 and 2).
HOMES = {12: -0.0269, 13: 1.5795, 22: -2.4607, 23: -0.0696}


@pytest.fixture
def session(joint_map):
    return joint_map.with_references(HOMES, 'test session')


def start_pose(session, legs=(1, 2)):
    return {leg: session.raw_to_pitch_knee(leg, HOMES[leg * 10 + 2] + 0.005, HOMES[leg * 10 + 3] + 0.1)
            for leg in legs}


def test_pitch_knee_raw_round_trip(session):
    for q in [(2.3, 2.9), (1.0, 1.5), (0.2, 1.2)]:
        raw = session.pitch_knee_to_raw(1, *q)
        assert session.raw_to_pitch_knee(1, *raw) == pytest.approx(q)
    assert session.pitch_knee_to_raw(1, session.motors[12].q_at_reference_rad,
                                     session.motors[13].q_at_reference_rad) == pytest.approx((HOMES[12], HOMES[13]))


@pytest.mark.parametrize('kind', ['nudge', 'pose', 'circle'])
def test_motions_start_and_end_at_the_measured_pose_within_limits(geometry, session, kind):
    start = start_pose(session)
    trajectory = plan_plane_motion(geometry, session, start, PlaneMotion(kind=kind))
    assert trajectory.ok, trajectory.violations
    for leg in (1, 2):
        assert trajectory.q[leg][0] == pytest.approx(start[leg])
        assert trajectory.q[leg][-1] == pytest.approx(start[leg])
        hip_id, knee_id = motor_ids(leg)[1:]
        assert trajectory.raw[hip_id][0] == pytest.approx(HOMES[hip_id] + 0.005)
        assert trajectory.raw[knee_id][0] == pytest.approx(HOMES[knee_id] + 0.1)
    peak = max(np.abs(v).max() for v in trajectory.raw_velocity.values())
    assert peak <= 0.1 * 1.01  # timed to the requested speed
    assert np.all(np.diff(trajectory.times) > 0)


def test_pose_reaches_the_requested_foot_point(geometry, session):
    motion = PlaneMotion(kind='pose', r_m=0.35, z_m=-0.05, hold_s=1.0)
    trajectory = plan_plane_motion(geometry, session, start_pose(session), motion)
    middle = len(trajectory.times) // 2  # inside the hold
    for leg in (1, 2):
        foot = fk((0.0, *trajectory.q[leg][middle]), geometry.leg(leg))
        assert (foot[0], foot[2]) == pytest.approx((0.35, -0.05), abs=1e-9)


def test_circle_traces_the_requested_radius(geometry, session):
    motion = PlaneMotion(kind='circle', r_m=0.35, z_m=-0.05, radius_m=0.03, loops=2)
    trajectory = plan_plane_motion(geometry, session, start_pose(session, (1,)), motion)
    feet = np.array([fk((0.0, *q), geometry.leg(1))[[0, 2]] for q in trajectory.q[1]])
    distances = np.hypot(feet[:, 0] - 0.35, feet[:, 1] + 0.05)
    on_circle = np.abs(distances - 0.03) < 1e-9
    assert on_circle.sum() > 50
    angles = np.unwrap(np.arctan2(feet[on_circle, 1] + 0.05, feet[on_circle, 0] - 0.35))
    assert angles.max() - angles.min() == pytest.approx(4 * pi, abs=0.05)


def test_slower_speed_lengthens_the_motion(geometry, session):
    fast = plan_plane_motion(geometry, session, start_pose(session), PlaneMotion(kind='pose', max_speed_rad_s=0.2))
    slow = plan_plane_motion(geometry, session, start_pose(session), PlaneMotion(kind='pose', max_speed_rad_s=0.05))
    assert slow.times[-1] > 3 * fast.times[-1]
    assert max(np.abs(v).max() for v in fast.raw_velocity.values()) <= 0.2 * 1.01


def test_violations_reported_not_sent(geometry, session):
    far = plan_plane_motion(geometry, session, start_pose(session), PlaneMotion(kind='pose', r_m=0.55, z_m=-0.3))
    assert not far.ok and any('controller envelope' in v for v in far.violations)
    unreachable = plan_plane_motion(geometry, session, start_pose(session), PlaneMotion(kind='pose', r_m=2.0, z_m=0.0))
    assert any('Unreachable' in v for v in unreachable.violations)
    tight = plan_plane_motion(geometry, session, start_pose(session), PlaneMotion(),
                              offsets=EnvelopeOffsets(hip_pitch=(-0.02, 0.1)))
    assert any('motor 12 leaves the controller envelope' in v for v in tight.violations)


@pytest.mark.parametrize('bad', [dict(kind='walk'), dict(max_speed_rad_s=5.5), dict(max_speed_rad_s=0),
                                 dict(d_pitch_rad=1.0), dict(kind='circle', loops=0), dict(branch='x')])
def test_motion_parameters_validated(geometry, session, bad):
    with pytest.raises(ValueError):
        plan_plane_motion(geometry, session, start_pose(session), PlaneMotion(**bad))


def state(ready=(1, 2), fresh=True):
    return dict(safety_state='DISARMED',
                legs=[dict(leg=leg, homed=leg in ready, ready=leg in ready, motor_ids=list(motor_ids(leg)[1:]),
                           home_positions=[HOMES.get(leg * 10 + 2, 0.0), HOMES.get(leg * 10 + 3, 0.0)]
                           if leg in ready else []) for leg in range(1, 7)],
                motors=[dict(motor_id=mid, fresh=fresh, faults=[], position=HOMES[mid] + 0.01) for mid in HOMES])


def test_session_inputs_need_ready_legs_and_fresh_feedback(joint_map):
    session_map, start, raw = session_inputs(state(), (1, 2), joint_map)
    assert session_map.references == HOMES and set(start) == {1, 2}
    assert raw == {mid: HOMES[mid] + 0.01 for mid in HOMES}
    with pytest.raises(ValueError, match='Leg 2 is not homed and READY'):
        session_inputs(state(ready=(1,)), (1, 2), joint_map)
    with pytest.raises(ValueError, match='stale'):
        session_inputs(state(fresh=False), (1,), joint_map)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def streamer(geometry, session):
    clock = Clock()
    trajectory = plan_plane_motion(geometry, session, start_pose(session, (1,)), PlaneMotion())
    return Streamer(trajectory, 30.0, 2.0, clock=clock), clock, trajectory


def run(s, clock, seconds, measured, follow=1.0, dt=0.05):
    """Step at 20 Hz; each motor moves ``follow`` of the way to its last target per tick."""
    out = None
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        out = s.step('ARMED', 0.02, dict(measured))
        if out is None:
            return None
        for mid, position, _ in out:
            measured[mid] += follow.get(mid, 1.0) * (position - measured[mid]) if isinstance(follow, dict) \
                else follow * (position - measured[mid])
    return out


def test_streamer_holds_until_armed_then_plays_and_finishes(geometry, session):
    s, clock, trajectory = streamer(geometry, session)
    measured = {12: trajectory.raw[12][0], 13: trajectory.raw[13][0]}
    hold = s.step('DISARMED', 0.02, measured)
    assert [t[1] for t in hold] == pytest.approx([trajectory.raw[12][0], trajectory.raw[13][0]])
    clock.t = 5.0
    assert s.step('DISARMED', 0.02, measured) == hold and s.phase == 'HOLD'
    assert s.step('ARMED', 0.02, measured) is not None and s.phase == 'PLAY'
    duration = trajectory.times[-1]
    assert run(s, clock, duration / 2, measured) is not None
    assert s.progress == pytest.approx(duration / 2, abs=0.11)  # following motors: real-time playback
    assert run(s, clock, duration / 2 + END_HOLD_S + 0.2, measured) is None
    assert s.phase == 'DONE' and s.progress == pytest.approx(duration)
    assert measured[12] == pytest.approx(trajectory.raw[12][-1], abs=1e-3)


@pytest.mark.parametrize('safety,age,advance,phase_reason', [
    ('ARMED', STATE_STALE_S + 0.1, 0.0, 'stale'),
    ('FAULT', 0.02, 0.0, 'FAULT before arming'),
    ('DISARMED', 0.02, ARM_TIMEOUT_S + 1, 'Not armed'),
])
def test_streamer_stops_before_arming(geometry, session, safety, age, advance, phase_reason):
    s, clock, _ = streamer(geometry, session)
    clock.t = advance
    assert s.step(safety, age) is None and s.phase == 'STOPPED' and phase_reason in s.reason


def test_lagging_motor_slows_the_plan_and_targets_never_lead_by_more_than_the_limit(geometry, session):
    """2026-10-27 22:26: hip 42 fell >0.1 rad behind, every command was rejected, watchdog FAULT."""
    from tocatl_kinematics.player import LEAD_RAD
    clock = Clock()
    trajectory = plan_plane_motion(geometry, session, start_pose(session, (1,)), PlaneMotion(kind='pose'))
    s = Streamer(trajectory, 30.0, 2.0, clock=clock)
    measured = {12: trajectory.raw[12][0], 13: trajectory.raw[13][0]}
    s.step('ARMED', 0.02, measured)
    sent = []
    for _ in range(400):  # 20 s; the hip covers 2% of each gap per tick (~0.03 rad/s < 0.1 planned)
        clock.t += 0.05
        out = s.step('ARMED', 0.02, dict(measured))
        assert out is not None, s.reason
        for mid, position, _ in out:
            sent.append(abs(position - measured[mid]))
            measured[mid] += (0.02 if mid == 12 else 1.0) * (position - measured[mid])
    assert max(sent) <= LEAD_RAD + 1e-12
    assert 1.0 < s.progress < 15.0  # slowed well below real time, but still advancing
    assert s.max_lag[0] == 12


def test_stalled_motor_stops_with_a_named_reason(geometry, session):
    s, clock, trajectory = streamer(geometry, session)
    measured = {12: trajectory.raw[12][0], 13: trajectory.raw[13][0]}
    s.step('ARMED', 0.02, measured)
    stuck = dict(measured)
    for _ in range(int(10.0 / 0.05)):
        clock.t += 0.05
        out = s.step('ARMED', 0.02, dict(stuck))  # the hip never moves
        if out is None:
            break
        stuck[13] = dict((m, p) for m, p, _ in out)[13]
    assert out is None and s.phase == 'STOPPED'
    assert 'stalled' in s.reason and 'motor 12' in s.reason


def test_held_legs_are_clamped_toward_their_measured_pose(geometry, session):
    from tocatl_kinematics.player import LEAD_RAD
    clock = Clock()
    trajectory = plan_plane_motion(geometry, session, {2: start_pose(session)[2]}, PlaneMotion())
    s = Streamer(trajectory, 30.0, 2.0, clock=clock, hold={12: 0.5, 13: 1.0})
    out = dict((m, p) for m, p, _ in s.step('DISARMED', 0.02, {12: 0.3, 13: 1.0, 22: trajectory.raw[22][0],
                                                               23: trajectory.raw[23][0]}))
    assert out[12] == pytest.approx(0.3 + LEAD_RAD) and out[13] == pytest.approx(1.0)


def test_streamer_stops_if_controller_leaves_armed(geometry, session):
    s, clock, _ = streamer(geometry, session)
    s.step('ARMED', 0.02)
    clock.t = 1.0
    assert s.step('FAULT', 0.02) is None and 'left ARMED (FAULT)' in s.reason


def test_parse_legs():
    assert parse_legs('2,1,2') == [1, 2]
    for bad in ('', '0', '7', '1,8'):
        with pytest.raises(ValueError):
            parse_legs(bad)


def test_actuated_legs_from_controller_ids():
    from tocatl_kinematics.player import actuated_legs_from_ids
    assert actuated_legs_from_ids([12, 13, 22, 23, 32, 33, 42, 43, 52, 53, 62, 63]) == ([1, 2, 3, 4, 5, 6], [])
    assert actuated_legs_from_ids([23, 22]) == ([2], [])
    assert actuated_legs_from_ids([11, 12, 13, 52, 53]) == ([1, 5], [11])  # standing: yaw held, 51 absent
    for bad in ([], [11], [12], [12, 13, 22]):
        with pytest.raises(ValueError):
            actuated_legs_from_ids(bad)


def test_streamer_names_every_actuated_motor_and_holds_the_rest(geometry, session):
    """The 2026-10-27 22:16 run sent leg 2 alone to a 6-leg session: all 2397 commands rejected."""
    clock = Clock()
    start = start_pose(session, (1, 2))
    trajectory = plan_plane_motion(geometry, session, {2: start[2]}, PlaneMotion())
    hold = {12: HOMES[12] + 0.005, 13: HOMES[13] + 0.1}
    s = Streamer(trajectory, 30.0, 2.0, clock=clock, hold=hold)
    measured = {**hold, 22: trajectory.raw[22][0], 23: trajectory.raw[23][0]}
    first = s.step('DISARMED', 0.02, measured)
    assert [t[0] for t in first] == [12, 13, 22, 23]
    s.step('ARMED', 0.02, measured)
    middle = {mid: (pos, vel) for mid, pos, vel in run(s, clock, trajectory.times[-1] / 2, measured)}
    assert middle[12] == (pytest.approx(hold[12]), 0.0) and middle[13] == (pytest.approx(hold[13]), 0.0)
    assert middle[22][0] != pytest.approx(trajectory.raw[22][0])
    with pytest.raises(ValueError):
        Streamer(trajectory, 30.0, 2.0, hold={22: 0.0})


def test_held_legs_must_sit_inside_the_envelope(session):
    from tocatl_kinematics.player import hold_violations
    raw = {12: HOMES[12] + 0.005, 13: HOMES[13] + 0.1}
    assert hold_violations(session, raw, [1], EnvelopeOffsets()) == []
    raw[12] = HOMES[12] + 1.7
    assert 'held motor 12' in hold_violations(session, raw, [1], EnvelopeOffsets())[0]


@pytest.mark.parametrize('argv,message', [
    (['--legs', '2', '--max-speed-rad-s', '6'], 'max_speed_rad_s'),   # out of range: a clean refusal
    (['--legs', '2', '--kp', '600'], 'kp must be'),   # beyond any profile; per-profile limits are read from the controller
    (['--legs', '4', '--actuated-legs', '1,2'], 'not commanded'),
    (['--legs', '9'], 'Choose legs'),
])
def test_bad_inputs_refuse_cleanly_before_ros(argv, message, capsys):
    from tocatl_kinematics.player import main
    assert main(argv) == 2
    out = capsys.readouterr().out
    assert out.startswith('Refused:') and message in out


def test_lead_follows_the_controller_step_limit(geometry, session):
    from tocatl_kinematics.player import LEAD_RAD, lead_for_step
    assert lead_for_step(0.5) == pytest.approx(0.4)
    assert lead_for_step(0.1) == pytest.approx(0.08)
    assert lead_for_step(None) == LEAD_RAD and lead_for_step(5.0) == LEAD_RAD
    clock = Clock()
    trajectory = plan_plane_motion(geometry, session, start_pose(session, (1,)),
                                   PlaneMotion(kind='pose', max_speed_rad_s=3.0))
    s = Streamer(trajectory, 30.0, 2.0, clock=clock, lead=0.4)
    measured = {12: trajectory.raw[12][0], 13: trajectory.raw[13][0]}
    s.step('ARMED', 0.02, measured)
    sent = []
    for _ in range(200):
        clock.t += 0.05
        out = s.step('ARMED', 0.02, dict(measured))
        if out is None:
            break
        for mid, position, _ in out:
            sent.append(abs(position - measured[mid]))
            measured[mid] += 0.3 * (position - measured[mid])
    assert max(sent) <= 0.4 + 1e-12 and max(sent) > 0.08  # uses the wider lead, never beyond it
    with pytest.raises(ValueError):
        Streamer(trajectory, 30.0, 2.0, lead=0.0)


def test_fast_motion_is_timed_to_the_requested_speed(geometry, session):
    fast = plan_plane_motion(geometry, session, start_pose(session), PlaneMotion(kind='pose', max_speed_rad_s=3.0))
    assert fast.ok, fast.violations
    assert max(np.abs(v).max() for v in fast.raw_velocity.values()) <= 3.0 * 1.01
    assert fast.times[-1] < 6.0


def test_controller_limits_from_parameters():
    from tocatl_kinematics.player import limits_from_values
    standing = limits_from_values([[11, 12, 13], 0.5, 100.0, 5.0, 8.0, [-0.3, 0.3], [-0.02, 2.6], [-1.7, 2.0]])
    assert standing.max_kp == 100.0 and standing.torque_limit == 8.0
    assert standing.offsets.hip_pitch == (-0.02, 2.6) and standing.offsets.hip_yaw == (-0.3, 0.3)
    feet_air = limits_from_values([[12, 13], 0.5, 30.0, 2.0, 1.5, [0.0], [-0.02, 1.6], [-1.7, 2.0]])
    assert feet_air.offsets.hip_yaw == EnvelopeOffsets().hip_yaw  # unset parameter reads as [0.0]


def stand_motion(**kw):
    values = dict(kind='stand', r_m=0.25, rest_height_m=0.10, stand_height_m=0.20, mass_kg=12.0,
                  max_speed_rad_s=0.3, torque_limit_nm=8.0)
    values.update(kw)
    return PlaneMotion(**values)


STANDING = EnvelopeOffsets(hip_pitch=(-0.02, 2.6), knee=(-1.7, 2.0))


def six_leg_session(joint_map):
    homes = {12: -0.0269, 13: 1.5795, 22: -2.4607, 23: -0.0696, 32: -0.76, 33: -0.23,
             42: -1.64, 43: -0.06, 52: -0.85, 53: -0.17, 62: -2.45, 63: -0.89}
    session = joint_map.with_references(homes, 'test')
    start = {l: session.raw_to_pitch_knee(l, homes[l * 10 + 2] + 0.005, homes[l * 10 + 3] + 0.1) for l in range(1, 7)}
    return session, start


def test_stand_places_feet_touches_down_rises_and_returns(geometry, joint_map):
    session, start = six_leg_session(joint_map)
    t = plan_plane_motion(geometry, session, start, stand_motion(), offsets=STANDING)
    assert t.ok, t.violations
    leg = geometry.leg(1)
    feet = np.array([fk((0.0, *row), leg)[[0, 2]] for row in t.q[1]])
    loaded_feet = feet[t.loaded]
    assert loaded_feet[:, 0] == pytest.approx(0.25, abs=1e-9)           # feet stay at r while loaded
    assert loaded_feet[:, 1].max() == pytest.approx(-0.10, abs=1e-3)    # ground contact at rest height
    assert loaded_feet[:, 1].min() == pytest.approx(-0.20, abs=1e-9)    # body raised to stand height
    assert t.q[1][0] == pytest.approx(start[1]) and t.q[1][-1] == pytest.approx(start[1])
    hip_peak = np.abs(t.torques[12]).max()
    assert 3.0 < hip_peak < 0.9 * 8.0                                   # ~4.9 Nm for 12 kg on six legs
    assert np.abs(t.torques[12][~t.loaded]).max() == 0.0
    assert any('continuous rating' in w for w in t.warnings)


def test_stand_refuses_loads_beyond_the_cap_and_missing_mass(geometry, joint_map):
    session, start = six_leg_session(joint_map)
    heavy = plan_plane_motion(geometry, session, start, stand_motion(mass_kg=25.0), offsets=STANDING)
    assert any('over 90% of the 8 Nm cap' in v for v in heavy.violations)
    two_legs = plan_plane_motion(geometry, session, {1: start[1], 2: start[2]}, stand_motion(), offsets=STANDING)
    assert any('on 2 legs' in v for v in two_legs.violations)
    narrow = plan_plane_motion(geometry, session, start, stand_motion(), offsets=EnvelopeOffsets())
    assert any('controller envelope' in v for v in narrow.violations)  # feet-in-air hip envelope is too small
    for bad in (dict(mass_kg=0.0), dict(stand_height_m=0.05), dict(clearance_m=0.0)):
        with pytest.raises(ValueError):
            stand_motion(**bad).validate()
