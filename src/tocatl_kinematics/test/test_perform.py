import numpy as np
import pytest

from tocatl_kinematics.joint_map import motor_ids
from tocatl_kinematics.motions import EnvelopeOffsets, PlaneMotion, plan_plane_motion
from tocatl_kinematics.perform import (CUES, READY_R_M, READY_Z_M, REST_OFFSET_RAD, Performer, plan_cues,
                                       plan_enter, plan_fold, ready_pose, rest_pose, reversed_trajectory)
from tocatl_kinematics.teleop import CUE_BUTTONS

HOMES = {12: -0.0269, 13: 1.5795, 22: -2.4607, 23: -0.0696, 32: -0.76, 33: -0.23,
         42: -1.64, 43: -0.06, 52: -0.85, 53: -0.17, 62: -2.45, 63: -0.89}
LEGS = tuple(range(1, 7))
FEET_AIR = EnvelopeOffsets()   # motion-feet-air.yaml


@pytest.fixture
def session(joint_map):
    return joint_map.with_references(HOMES, 'test')


def folded_pose(session):
    """Folded legs near the homes, as after homing."""
    return {leg: session.raw_to_pitch_knee(leg, HOMES[leg * 10 + 2] + 0.005, HOMES[leg * 10 + 3] + 0.1) for leg in LEGS}


def test_every_cue_is_feasible_from_the_ready_pose_on_all_legs(geometry, session):
    cues = plan_cues(geometry, session, LEGS, FEET_AIR, speed=1.0)
    for name, trajectory in cues.items():
        assert trajectory.ok, (name, trajectory.violations)
        for mid, values in trajectory.raw.items():           # each cue starts and ends at the ready pose
            assert values[0] == pytest.approx(values[-1], abs=1e-6), (name, mid)
    assert set(cues) == set(CUES)


def test_teleop_buttons_only_send_known_cues():
    assert {cue for _, cue in CUE_BUTTONS} <= set(CUES)


def test_enter_goes_to_the_ready_pose_and_stays_exit_comes_back(geometry, session):
    enter = plan_enter(geometry, session, folded_pose(session), FEET_AIR)
    assert enter.ok, enter.violations
    ready = ready_pose(geometry, LEGS)
    for leg in LEGS:
        assert enter.q[leg][-1] == pytest.approx(ready[leg], abs=1e-9)
        assert enter.q[leg][0] == pytest.approx(folded_pose(session)[leg], abs=1e-9)
    exit_ = reversed_trajectory(enter)
    for mid in enter.raw:
        assert exit_.raw[mid][0] == pytest.approx(enter.raw[mid][-1])
        assert exit_.raw[mid][-1] == pytest.approx(enter.raw[mid][0])
        assert exit_.raw_velocity[mid][0] == pytest.approx(-enter.raw_velocity[mid][-1])


def test_goto_has_no_hold_or_return(geometry, session):
    start = folded_pose(session)
    goto = plan_plane_motion(geometry, session, start, PlaneMotion(kind='goto', r_m=READY_R_M, z_m=READY_Z_M))
    pose = plan_plane_motion(geometry, session, start, PlaneMotion(kind='pose', r_m=READY_R_M, z_m=READY_Z_M))
    assert goto.times[-1] < pose.times[-1] / 2


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


@pytest.fixture
def rig(geometry, session):
    """A Performer on two legs, with a measured pose that follows the targets exactly."""
    legs = (1, 2)
    cues = {name: t for name, t in plan_cues(geometry, session, legs, FEET_AIR, speed=2.0,
                                             names=('wave', 'bloom')).items()}
    ids = [mid for leg in legs for mid in motor_ids(leg)[1:]] + [11, 21]

    def enter_from(measured):
        q = {leg: session.raw_to_pitch_knee(leg, measured[leg * 10 + 2], measured[leg * 10 + 3]) for leg in legs}
        return plan_enter(geometry, session, q, FEET_AIR, speed=2.0), {11: measured[11], 21: measured[21]}
    clock = Clock()
    fold = plan_fold(geometry, session, legs, FEET_AIR, speed=2.0)
    performer = Performer(cues, enter_from, ids, kp=10.0, kd=1.0, lead=0.4, fold=fold, clock=clock)
    measured = {11: 0.1, 21: -0.1}
    start = rest_pose(session, legs)          # resting where the fold-back puts the legs
    for leg in legs:
        measured.update(zip(motor_ids(leg)[1:], session.pitch_knee_to_raw(leg, *start[leg])))
    return performer, clock, measured


def run(performer, clock, measured, safety, seconds, dt=0.05):
    """Step for ``seconds``; the measured pose follows the targets. Returns the phases seen."""
    phases = []
    for _ in range(int(round(seconds / dt))):
        targets = performer.step(safety, 0.02, dict(measured))
        assert targets is not None, performer.reason
        for mid, position, _ in targets:
            measured[mid] = position
        if not phases or phases[-1] != performer.phase:
            phases.append(performer.phase)
        clock.now += dt
    return phases


def test_performance_unfolds_plays_cues_and_folds_back(rig):
    performer, clock, measured = rig
    rest = dict(measured)
    assert run(performer, clock, measured, 'ARMED', 1.0) == ['REST']        # armed at start: no unfold
    assert run(performer, clock, measured, 'DISARMED', 0.5) == ['REST']
    assert run(performer, clock, measured, 'ARMED', 30.0)[-1] == 'READY'
    performer.request_cue('bloom')
    phases = run(performer, clock, measured, 'ARMED', 30.0)
    assert phases[0] == 'CUE' and phases[-1] == 'READY'
    performer.request_cue('nope')
    performer.request_stop()
    phases = run(performer, clock, measured, 'ARMED', 30.0)
    assert 'EXIT' in phases and phases[-1] == 'REST'
    events = performer.take_events()
    assert ('disarm', 'performance finished') in events
    assert any('Unknown or infeasible cue' in text for kind, text in events if kind == 'log')
    for mid in rest:                                                          # folded back where it started
        assert measured[mid] == pytest.approx(rest[mid], abs=1e-6)
    assert run(performer, clock, measured, 'ARMED', 2.0) == ['REST']        # disarm pending: no re-unfold


def test_cues_are_ignored_unless_ready(rig):
    performer, clock, measured = rig
    performer.request_cue('wave')
    run(performer, clock, measured, 'DISARMED', 0.2)
    assert performer.phase == 'REST'
    run(performer, clock, measured, 'ARMED', 0.2)
    assert performer.phase == 'ENTER'
    performer.request_cue('wave')
    assert any('still unfolding' in text for _, text in performer.take_events())


def test_disarm_mid_cue_returns_to_rest_and_a_fresh_arm_unfolds_from_the_new_pose(rig):
    performer, clock, measured = rig
    run(performer, clock, measured, 'DISARMED', 0.1)
    run(performer, clock, measured, 'ARMED', 30.0)
    performer.request_cue('wave')
    run(performer, clock, measured, 'ARMED', 1.0)
    assert performer.phase == 'CUE'
    run(performer, clock, measured, 'DISARMED', 0.2)
    assert performer.phase == 'REST'
    measured[12] -= 0.05                                       # legs sag while limp
    run(performer, clock, measured, 'DISARMED', 0.2)
    assert performer.enter[2][12] == pytest.approx(measured[12])
    assert run(performer, clock, measured, 'ARMED', 30.0)[-1] == 'READY'


def test_stale_state_stops(rig):
    performer, _, measured = rig
    assert performer.step('ARMED', 1.0, measured) is None
    assert 'stale' in performer.reason


def test_yaw_is_held_throughout(rig):
    performer, clock, measured = rig
    run(performer, clock, measured, 'DISARMED', 0.1)
    yaw = (measured[11], measured[21])
    run(performer, clock, measured, 'ARMED', 30.0)
    performer.request_cue('wave')
    run(performer, clock, measured, 'ARMED', 30.0)
    assert (measured[11], measured[21]) == pytest.approx(yaw)


def test_fold_back_goes_to_the_rest_pose_even_after_re_arming_near_the_ready_pose(rig, session):
    """2026-11-07: disarmed and re-armed with the legs still near the ready pose, B 'finished' a
    zero-length fold-back at once and disarmed: the legs went limp without folding."""
    performer, clock, measured = rig
    run(performer, clock, measured, 'DISARMED', 0.1)
    run(performer, clock, measured, 'ARMED', 30.0)
    assert performer.phase == 'READY'
    run(performer, clock, measured, 'DISARMED', 0.2)           # legs stay up (gearbox friction)
    assert run(performer, clock, measured, 'ARMED', 30.0)[-1] == 'READY'   # tiny re-unfold
    performer.take_events()
    performer.request_stop()
    phases = run(performer, clock, measured, 'ARMED', 30.0)
    assert 'EXIT' in phases and phases[-1] == 'REST'
    assert ('disarm', 'performance finished') in performer.take_events()
    for mid in (12, 13, 22, 23):                               # folded to the stops + offset
        assert measured[mid] == pytest.approx(session.references[mid] + REST_OFFSET_RAD, abs=1e-6)


def test_fold_back_is_feasible_on_all_legs(geometry, session):
    fold = plan_fold(geometry, session, LEGS, FEET_AIR)
    assert fold.ok, fold.violations
    ready = ready_pose(geometry, LEGS)
    for leg in LEGS:
        assert fold.q[leg][0] == pytest.approx(ready[leg], abs=1e-9)


def test_releasing_the_deadman_pauses_a_cue_and_holding_it_resumes(rig):
    performer, clock, measured = rig
    run(performer, clock, measured, 'DISARMED', 0.1)
    run(performer, clock, measured, 'ARMED', 30.0)
    performer.request_cue('wave')
    run(performer, clock, measured, 'ARMED', 1.0)
    assert performer.phase == 'CUE'
    progress, held = performer.streamer.progress, dict(measured)
    for _ in range(80):                                   # 4 s released: longer than a stall
        performer.set_deadman(False, clock.now)
        run(performer, clock, measured, 'ARMED', 0.05)
    assert performer.phase == 'CUE' and performer.streamer.progress == progress
    assert max(abs(measured[m] - held[m]) for m in measured) < 1e-9
    for _ in range(10):
        performer.set_deadman(True, clock.now)
        run(performer, clock, measured, 'ARMED', 0.05)
    assert performer.streamer.progress > progress
    texts = [t for _, t in performer.take_events()]
    assert any('paused' in t for t in texts) and any('resumes' in t for t in texts)
