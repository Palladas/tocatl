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
    s.set_gait('ripple')
    run(s, clock, 0.1, measured)
    assert s.gen.params.gait == 'tripod'          # still stepping: switches only once settled
    run(s, clock, 12.0, measured)
    assert s.gen.params.gait == 'ripple' and s.phase == 'WALK'
    with pytest.raises(ValueError):
        s.set_gait('gallop')
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
