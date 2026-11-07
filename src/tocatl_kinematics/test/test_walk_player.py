import numpy as np
import pytest

from tocatl_kinematics.body import BodyPose, body_fk, joints_from_raw, neutral_stance
from tocatl_kinematics.joint_map import motor_ids
from tocatl_kinematics.motions import EnvelopeOffsets
from tocatl_kinematics.walk import WalkParams, solve
from tocatl_kinematics.walk_player import ScriptCommands, WalkStreamer, parse_script

STANDING = EnvelopeOffsets(hip_yaw=(-0.3, 0.3), hip_pitch=(-0.05, 7.02), knee=(-1.7, 2.0))
ALL = [m for leg in range(1, 7) for m in motor_ids(leg)]


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def resting(geometry, joint_map, height=0.10):
    """Session with every reference at raw 0, and the robot resting on its feet at r 0.25."""
    session = joint_map.with_references({m: 0.0 for m in ALL}, 'test')
    raw = {}
    for leg, solution in solve(geometry, neutral_stance(geometry, 0.25, height)).items():
        raw.update(zip(motor_ids(leg), session.to_raw(leg, solution.q)))
    return session, raw


def run(streamer, clock, seconds, measured, dt=0.02, safety='ARMED'):
    """Tick with motors that follow their targets exactly; returns the published batches."""
    out = []
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        targets = streamer.step(safety, 0.02, dict(measured))
        if targets is None:
            break
        measured.update({mid: pos for mid, pos, _ in targets})
        out.append(targets)
    return out


def feet_from(geometry, session, raw):
    return body_fk(geometry, joints_from_raw(session, raw), BodyPose())


def test_walk_holds_rises_walks_settles_and_lowers(geometry, joint_map):
    session, raw = resting(geometry, joint_map)
    clock = Clock()
    commands = {'v': (0.03, 0.0, 0.0)}
    s = WalkStreamer(geometry, session, WalkParams(gait='tripod'), raw, lambda: commands['v'],
                     offsets=STANDING, clock=clock, lead=0.4)
    assert not s.blend_problems() and not s.yaw_problems()
    measured = dict(raw)
    hold = run(s, clock, 1.0, measured, safety='DISARMED')
    assert s.phase == 'HOLD' and all(pos == pytest.approx(raw[mid]) for mid, pos, _ in hold[-1])
    start = feet_from(geometry, session, raw)
    run(s, clock, s.blend_s / 2, measured)
    assert s.phase == 'ENTER'
    mid_feet = feet_from(geometry, session, measured)
    for leg in start:            # rising: feet only move vertically, so they do not slide
        assert mid_feet[leg][:2] == pytest.approx(start[leg][:2], abs=1e-6)
        assert mid_feet[leg][2] < start[leg][2]
    run(s, clock, s.blend_s / 2 + 6.0, measured)
    assert s.phase == 'WALK' and s.gen.command[0] > 0.02
    s.request_stop()
    batches = run(s, clock, 30.0, measured)
    assert s.phase == 'DONE' and s.reason == 'Walk complete'
    final = feet_from(geometry, session, measured)
    for leg in start:            # back down where it started
        assert final[leg] == pytest.approx(start[leg], abs=1e-6)
    assert {mid for mid, _, _ in batches[0]} == set(ALL)   # every motor, yaw included, every tick


def test_stop_before_arming_and_fault_and_stale_state(geometry, joint_map):
    session, raw = resting(geometry, joint_map)
    clock = Clock()
    s = WalkStreamer(geometry, session, WalkParams(), raw, lambda: (0, 0, 0), offsets=STANDING, clock=clock)
    assert s.step('DISARMED', None, raw) is None and 'stale' in s.reason
    s = WalkStreamer(geometry, session, WalkParams(), raw, lambda: (0, 0, 0), offsets=STANDING, clock=clock)
    assert s.step('FAULT', 0.02, raw) is None and 'FAULT' in s.reason
    s = WalkStreamer(geometry, session, WalkParams(), raw, lambda: (0, 0, 0), offsets=STANDING, clock=clock)
    run(s, clock, 0.5, dict(raw))
    assert s.step('DISARMED', 0.02, raw) is None and 'left ARMED' in s.reason


def test_lagging_motor_slows_the_walk_then_stalls(geometry, joint_map):
    session, raw = resting(geometry, joint_map)
    clock = Clock()
    s = WalkStreamer(geometry, session, WalkParams(), raw, lambda: (0, 0, 0), offsets=STANDING, clock=clock, lead=0.1)
    measured = dict(raw)
    s.step('ARMED', 0.02, measured)
    for _ in range(500):        # motor 12 never moves
        clock.t += 0.02
        targets = s.step('ARMED', 0.02, dict(measured))
        if targets is None:
            break
        for mid, pos, _ in targets:
            if mid != 12:
                measured[mid] = pos
            assert abs(pos - measured[mid]) <= 0.1 + 1e-9
    assert s.phase == 'STOPPED' and 'stalled' in s.reason and 'motor 12' in s.reason


def test_gait_change_waits_until_idle_and_yaw_must_start_radial(geometry, joint_map):
    session, raw = resting(geometry, joint_map)
    clock = Clock()
    commands = {'v': (0.03, 0.0, 0.0)}
    s = WalkStreamer(geometry, session, WalkParams(gait='tripod'), raw, lambda: commands['v'],
                     offsets=STANDING, clock=clock, lead=0.4)
    measured = dict(raw)
    run(s, clock, s.blend_s + 4.0, measured)
    assert 'did not pass the startup checks' in s.set_gait('ripple')   # not verified: refused
    assert s.pending_gait is None
    s.allow_gaits(['ripple'])
    assert s.set_gait('ripple') is None
    run(s, clock, 0.1, measured)
    assert s.gen.params.gait == 'tripod'          # still stepping: switches only once settled
    run(s, clock, 12.0, measured)
    assert s.gen.params.gait == 'ripple' and s.phase == 'WALK'
    assert 'Unknown gait' in s.set_gait('gallop')
    # Heights were verified for the start gait only, and gaits at the start height only.
    s.allow_heights([s.start_params.body_height_m + 0.01])
    assert s.request_height_step(+1) is None      # in ripple now: no height change
    assert s.set_gait('tripod') is None           # back to the start gait is always allowed
    twisted = dict(raw)
    twisted[31] += 0.2
    assert 'leg 3 yaw' in WalkStreamer(geometry, session, WalkParams(), twisted, lambda: (0, 0, 0),
                                       offsets=STANDING).yaw_problems()[0]


def test_envelope_is_enforced_on_every_target(geometry, joint_map):
    session, raw = resting(geometry, joint_map)
    narrow = EnvelopeOffsets(hip_yaw=(-0.3, 0.3), hip_pitch=(-0.05, 4.3), knee=(-1.7, 2.0))
    clock = Clock()
    s = WalkStreamer(geometry, session, WalkParams(), raw, lambda: (0, 0, 0), offsets=narrow, clock=clock, lead=0.4)
    assert any('outside' in p for p in s.blend_problems())   # rising needs more hip than this allows
    run(s, clock, 20.0, dict(raw))
    assert s.phase == 'STOPPED' and 'envelope' in s.reason


def test_script_parsing_and_playback():
    steps = parse_script('2:0.03,0,0; 1:0,0,0.1')
    assert steps == [(2.0, (0.03, 0.0, 0.0)), (1.0, (0.0, 0.0, 0.1))]
    clock, done = Clock(), []
    commands = ScriptCommands(steps, lambda: done.append(True), clock)
    assert commands() == (0.03, 0.0, 0.0)
    clock.t = 2.5
    assert commands() == (0.0, 0.0, 0.1)
    clock.t = 3.5
    assert commands() == (0.0, 0.0, 0.0) and done
    for bad in ('', '2:1,2', 'x:0,0,0', '0:0,0,0'):
        with pytest.raises(ValueError):
            parse_script(bad)


@pytest.mark.parametrize('argv,message', [(['--gait', 'gallop'], 'gait must be'), (['--script', '1:0,0'], 'seconds:vx'),
                                          (['--kp', '900'], 'kp must be'), (['--period-s', '0.1'], 'period_s')])
def test_bad_inputs_refuse_before_ros(argv, message, capsys):
    from tocatl_kinematics.walk_player import main
    assert main(argv) == 2
    out = capsys.readouterr().out
    assert out.startswith('Refused:') and message in out


def test_body_height_steps_wait_for_settled_feet_and_move_vertically(geometry, joint_map):
    session, raw = resting(geometry, joint_map)
    clock = Clock()
    commands = {'v': (0.03, 0.0, 0.0)}
    s = WalkStreamer(geometry, session, WalkParams(gait='ripple'), raw, lambda: commands['v'],
                     offsets=STANDING, clock=clock, lead=0.4)
    assert not s.height_problems(0.16) and not s.height_problems(0.14)
    s.allow_heights([0.14, 0.16])
    assert s.request_height_step(+1) == pytest.approx(0.16)
    assert s.request_height_step(+1) == pytest.approx(0.16)        # nothing verified above
    measured = dict(raw)
    run(s, clock, s.blend_s + 3.0, measured)
    assert s.phase in ('WALK', 'ADJUST')
    run(s, clock, 15.0, measured)
    assert s.phase == 'WALK' and s.gen.params.body_height_m == pytest.approx(0.16)
    feet = feet_from(geometry, session, measured)
    down = [l for l, on_ground in s.gen.contact().items() if on_ground]
    assert len(down) >= 5
    assert all(feet[l][2] == pytest.approx(-0.16 - geometry.leg(l).mount_z_m, abs=1e-6) for l in down)
    s.request_stop()
    run(s, clock, 40.0, measured)
    assert s.phase == 'DONE'
    final = feet_from(geometry, session, measured)
    start = feet_from(geometry, session, raw)
    assert all(final[l] == pytest.approx(start[l], abs=1e-6) for l in start)


def test_robot_state_with_declared_yaw_gives_the_walker_every_joint(joint_map):
    """The controller publishes [yaw, hip, knee] per leg once yaw is declared; session_inputs
    must then carry yaw references so every leg resolves (2026-11-07 refusal)."""
    from tocatl_kinematics.body import joints_from_raw
    from tocatl_kinematics.player import session_inputs
    homes = {m: 0.1 * (m % 10) for m in ALL}
    legs = [dict(leg=leg, homed=True, ready=True, scope='pitch_knee_yaw', motor_ids=list(motor_ids(leg)),
                 home_positions=[homes[m] for m in motor_ids(leg)]) for leg in range(1, 7)]
    motors = [dict(motor_id=m, fresh=True, faults=[], position=homes[m] + 0.05) for m in ALL]
    state = dict(safety_state='DISARMED', legs=legs, motors=motors)
    yaw_ids = [leg * 10 + 1 for leg in range(1, 7)]
    session_map, _, raw = session_inputs(state, list(range(1, 7)), joint_map, yaw_ids)
    assert all(y in session_map.references for y in yaw_ids)
    assert set(joints_from_raw(session_map, raw)) == set(range(1, 7))
    # The old message (pitch/knee only) left the yaws without references: no leg resolved.
    for leg in legs:
        leg['motor_ids'], leg['home_positions'] = leg['motor_ids'][1:], leg['home_positions'][1:]
    session_map, _, raw = session_inputs(state, list(range(1, 7)), joint_map, yaw_ids)
    assert joints_from_raw(session_map, raw) == {}


def test_releasing_the_deadman_freezes_the_walk_in_place_and_resumes(geometry, joint_map):
    """2026-11-07: releasing LB should stop all motion, not finish the stride."""
    from tocatl_kinematics.walk_player import STALL_S
    session, raw = resting(geometry, joint_map)
    clock = Clock()
    s = WalkStreamer(geometry, session, WalkParams(gait='tripod'), raw, lambda: (0.03, 0.0, 0.0),
                     offsets=STANDING, clock=clock, lead=0.4)
    measured = dict(raw)
    s.set_deadman(True, clock.t)
    run(s, clock, s.blend_s + 1.3, measured)
    assert s.phase == 'WALK'
    phase_before = s.gen.phase
    frozen_at = dict(measured)
    for _ in range(int((STALL_S + 1.0) / 0.02)):         # longer than a stall: no stall stop
        s.set_deadman(False, clock.t)
        run(s, clock, 0.02, measured)
    assert s.frozen and s.phase == 'WALK' and s.gen.phase == phase_before
    assert max(abs(measured[m] - frozen_at[m]) for m in measured) < 1e-9
    assert s.gen.command == (0.0, 0.0, 0.0)              # resumes from zero velocity
    for _ in range(25):
        s.set_deadman(True, clock.t)
        run(s, clock, 0.02, measured)
    assert not s.frozen and s.gen.phase != phase_before and s.gen.command[0] > 0


def test_a_stale_deadman_is_ignored(geometry, joint_map):
    session, raw = resting(geometry, joint_map)
    clock = Clock()
    s = WalkStreamer(geometry, session, WalkParams(gait='tripod'), raw, lambda: (0.03, 0.0, 0.0),
                     offsets=STANDING, clock=clock, lead=0.4)
    s.set_deadman(False, clock.t)                        # teleop said released, then went away
    measured = dict(raw)
    run(s, clock, s.blend_s + 1.0, measured)
    assert not s.frozen and s.phase == 'WALK'


def test_direction_changes_take_effect_within_a_fraction_of_a_second(geometry):
    from tocatl_kinematics.walk import WalkGenerator
    gen = WalkGenerator(geometry, WalkParams(gait='tripod'))
    for _ in range(100):
        gen.step(0.02, (0.05, 0.0, 0.0))
    assert gen.command[0] == pytest.approx(gen.clamp((0.05, 0.0, 0.0))[0])
    for tick in range(1, 100):
        gen.step(0.02, (-0.05, 0.0, 0.0))
        if gen.command[0] <= gen.clamp((-0.05, 0.0, 0.0))[0] + 1e-9:
            break
    assert tick * 0.02 <= 0.3                            # full reversal (was 2 s at 0.05 m/s^2)
