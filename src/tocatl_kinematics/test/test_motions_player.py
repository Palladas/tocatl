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


@pytest.mark.parametrize('bad', [dict(kind='walk'), dict(max_speed_rad_s=0.3), dict(max_speed_rad_s=0),
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


def test_streamer_holds_until_armed_then_plays_and_finishes(geometry, session):
    s, clock, trajectory = streamer(geometry, session)
    hold = s.step('DISARMED', 0.02)
    assert [t[1] for t in hold] == pytest.approx([trajectory.raw[12][0], trajectory.raw[13][0]])
    clock.t = 5.0
    assert s.step('DISARMED', 0.02) == hold and s.phase == 'HOLD'
    assert s.step('ARMED', 0.02) is not None and s.phase == 'PLAY'
    clock.t += trajectory.times[-1] / 2
    middle = s.step('ARMED', 0.02)
    assert middle != hold
    clock.t += trajectory.times[-1] / 2 + END_HOLD_S + 0.01
    assert s.step('ARMED', 0.02) is None and s.phase == 'DONE'


@pytest.mark.parametrize('safety,age,advance,phase_reason', [
    ('ARMED', STATE_STALE_S + 0.1, 0.0, 'stale'),
    ('FAULT', 0.02, 0.0, 'FAULT before arming'),
    ('DISARMED', 0.02, ARM_TIMEOUT_S + 1, 'Not armed'),
])
def test_streamer_stops_before_arming(geometry, session, safety, age, advance, phase_reason):
    s, clock, _ = streamer(geometry, session)
    clock.t = advance
    assert s.step(safety, age) is None and s.phase == 'STOPPED' and phase_reason in s.reason


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
    assert actuated_legs_from_ids([12, 13, 22, 23, 32, 33, 42, 43, 52, 53, 62, 63]) == [1, 2, 3, 4, 5, 6]
    assert actuated_legs_from_ids([23, 22]) == [2]
    for bad in ([], [11, 12, 13], [12], [12, 13, 22]):
        with pytest.raises(ValueError):
            actuated_legs_from_ids(bad)


def test_streamer_names_every_actuated_motor_and_holds_the_rest(geometry, session):
    """The 2026-10-27 22:16 run sent leg 2 alone to a 6-leg session: all 2397 commands rejected."""
    clock = Clock()
    start = start_pose(session, (1, 2))
    trajectory = plan_plane_motion(geometry, session, {2: start[2]}, PlaneMotion())
    hold = {12: HOMES[12] + 0.005, 13: HOMES[13] + 0.1}
    s = Streamer(trajectory, 30.0, 2.0, clock=clock, hold=hold)
    first = s.step('DISARMED', 0.02)
    assert [t[0] for t in first] == [12, 13, 22, 23]
    s.step('ARMED', 0.02)
    clock.t = trajectory.times[-1] / 2
    middle = {mid: (pos, vel) for mid, pos, vel in s.step('ARMED', 0.02)}
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
    (['--legs', '2', '--max-speed-rad-s', '0.4'], 'max_speed_rad_s'),   # 22:16 dry run crashed instead
    (['--legs', '2', '--kp', '50'], 'kp must be'),
    (['--legs', '4', '--actuated-legs', '1,2'], 'not commanded'),
    (['--legs', '9'], 'Choose legs'),
])
def test_bad_inputs_refuse_cleanly_before_ros(argv, message, capsys):
    from tocatl_kinematics.player import main
    assert main(argv) == 2
    out = capsys.readouterr().out
    assert out.startswith('Refused:') and message in out
