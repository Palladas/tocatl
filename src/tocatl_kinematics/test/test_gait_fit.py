import copy
import json
from math import pi

import numpy as np
import pytest

from tocatl_kinematics import cli, fit
from tocatl_kinematics.body import body_fk
from tocatl_kinematics.gait import TripodGait, foot_offset, plan, tripod_groups
from tocatl_kinematics.joint_map import joint_map_from_dict, motor_ids
from tocatl_kinematics.validate import CommandLimits, validate_trajectory

REFS = {mid: 0.05 * (mid % 10) for leg in range(1, 7) for mid in motor_ids(leg)}


def test_tripods_alternate_by_mount_angle(geometry):
    groups = tripod_groups(geometry)
    assert groups == ((1, 3, 5), (2, 4, 6))
    assert sorted(groups[0] + groups[1]) == [1, 2, 3, 4, 5, 6]


def test_foot_offset_continuous_over_a_cycle():
    phase = np.linspace(0, 2, 4001)
    along, lift, stance = foot_offset(phase, 0.5, 0.06, 0.04)
    assert np.abs(np.diff(along)).max() < 0.06 * 4 / 4000 * 2
    assert np.abs(np.diff(lift)).max() < 1e-3
    assert lift.max() == pytest.approx(0.04, abs=1e-6)
    assert (lift[stance] == 0).all()


def test_gait_starts_and_ends_in_neutral_and_keeps_stance_feet_fixed(geometry):
    gait = TripodGait(heading_rad=pi / 2)
    trajectory = plan(geometry, gait)
    assert not trajectory.failure
    for leg, feet in trajectory.feet_body.items():
        assert feet[0] == pytest.approx(feet[-1], abs=1e-9)
    # Steady cycles: stance feet stay still in the world as the body advances.
    steady = (trajectory.times > gait.period_s * 1.05) & (trajectory.times < gait.period_s * 2.95)
    world_y = {leg: trajectory.feet_body[leg][:, 1] + trajectory.body_travel_m[:, 1] for leg in trajectory.feet_body}
    for leg in trajectory.feet_body:
        mask = steady & trajectory.contact[leg]
        runs = np.split(np.flatnonzero(mask), np.flatnonzero(np.diff(np.flatnonzero(mask)) > 1) + 1)
        for run in runs:
            assert np.ptp(world_y[leg][run]) < 2e-3
    # The joint solution reproduces the planned feet.
    for i in (0, len(trajectory.times) // 3, len(trajectory.times) - 1):
        joints = {leg: trajectory.joints[leg][i] for leg in trajectory.joints}
        for leg, foot in body_fk(geometry, joints).items():
            assert foot == pytest.approx(trajectory.feet_body[leg][i], abs=1e-9)


def test_validation_flags_speed_cap_and_reports_shortest_period(geometry, joint_map):
    fast = plan(geometry, TripodGait(heading_rad=pi / 2, period_s=2.0))
    report = validate_trajectory(fast, geometry, joint_map)
    assert not report['ok']
    assert any('rad/s >' in v for v in report['violations'])
    slow = plan(geometry, TripodGait(heading_rad=pi / 2, period_s=report['min_feasible_period_s'] * 1.05))
    assert validate_trajectory(slow, geometry, joint_map)['ok']
    assert report['verified'] and report['accepted_values'] == 126


def test_validation_flags_out_of_service_yaw_only_when_it_moves(geometry, joint_map):
    forward = validate_trajectory(plan(geometry, TripodGait(heading_rad=0.0)), geometry, joint_map)
    assert any('motor 51 is out of service' in v for v in forward['violations'])
    sideways = validate_trajectory(plan(geometry, TripodGait(heading_rad=pi / 2)), geometry, joint_map)
    assert not any('51' in v for v in sideways['violations'])


def test_validation_flags_joint_limits_and_unreachable(geometry, joint_map):
    tall = plan(geometry, TripodGait(heading_rad=pi / 2, body_height_m=0.85))
    assert tall.failure and 'Unreachable' in validate_trajectory(tall, geometry, joint_map)['violations'][0]
    wide = plan(geometry, TripodGait(heading_rad=pi / 2, stride_m=0.5, period_s=60))
    assert any('outside' in v for v in validate_trajectory(wide, geometry, joint_map)['violations'])


def test_gait_rejects_bad_parameters(geometry):
    for bad in (dict(duty=0.3), dict(period_s=0), dict(cycles=0), dict(dt_s=1.0)):
        with pytest.raises(ValueError):
            plan(geometry, TripodGait(**bad))


def recording(path, hip, knee, leg=1):
    ids = motor_ids(leg)
    with open(path, 'w') as stream:
        for i, (h, k) in enumerate(zip(hip, knee)):
            stream.write(json.dumps(dict(t=0.1 * i, label='x', motors={str(ids[1]): dict(position=h),
                                                                         str(ids[2]): dict(position=k)})) + '\n')
    return path


def test_fit_coupling_recovers_slope(tmp_path, joint_map):
    hip = np.linspace(0.8, 1.6, 40)
    path = recording(tmp_path / 'r.jsonl', hip, 0.5 - 0.87 * (hip - 0.8))
    result = fit.fit_coupling(fit.load_recording(path), 1, joint_map)
    assert result['knee_per_hip_raw'] == pytest.approx(-0.87)
    assert result['coupling_raw_per_hip_rad'] == pytest.approx(0.87)  # hip sign -1, ratio 1
    assert result['r_squared'] == pytest.approx(1.0) and not result['warnings']
    tiny = recording(tmp_path / 's.jsonl', np.linspace(0.8, 0.85, 10), np.full(10, 0.5))
    assert fit.fit_coupling(fit.load_recording(tiny), 1, joint_map)['warnings']


def test_fit_scale_and_isolation(tmp_path):
    angles = np.radians([0, 30, 60, 90])
    pairs = [dict(raw_rad=1.0 - 2.8 * a, angle_rad=a) for a in angles]
    result = fit.fit_scale(pairs, raw_reference=1.0 - 2.8 * 0.4)
    assert (result['sign'], result['ratio'], result['q_at_reference_rad']) == (-1, pytest.approx(2.8), pytest.approx(0.4))
    path = recording(tmp_path / 'k.jsonl', np.full(10, 0.8) + np.linspace(0, 0.01, 10), np.linspace(0.2, 1.2, 10))
    isolation = fit.check_isolation(fit.load_recording(path), moved=13, still=12)
    assert isolation['ratio'] == pytest.approx(0.01)


def test_validate_fk_against_measured_feet(tmp_path, geometry, joint_map_data):
    jm = joint_map_from_dict(joint_map_data).with_references(REFS, 'test')
    q = (0.1, 0.3, 1.4)
    raw = jm.to_raw(2, q)
    from tocatl_kinematics.body import leg_to_body
    from tocatl_kinematics.leg import fk
    truth = leg_to_body(fk(q, geometry.leg(2)), geometry.leg(2))
    poses = [dict(leg=2, raw_rad={str(i): r for i, r in zip(motor_ids(2), raw)}, foot_body_m=(truth + [0.003, 0, 0]).tolist())]
    report = fit.validate_fk(poses, geometry, jm)
    assert report[0]['error_m'] == pytest.approx(0.003)


def test_cli_ik_and_gait_plan(tmp_path, capsys):
    common = ['--geometry', str(cli_repo('config/robot-geometry.json')), '--joint-map', str(cli_repo('config/joint-map.json'))]
    assert cli.ik_main(['stance', *common]) == 0
    stance = json.loads(capsys.readouterr().out)
    assert set(stance) == {'1', '2', '3', '4', '5', '6'} and stance['1']['raw_rad'] is None
    refs = tmp_path / 'refs.json'
    refs.write_text(json.dumps({'source': 'test', 'raw_rad': {str(k): v for k, v in REFS.items()}}))
    assert cli.ik_main(['leg', '--leg', '2', '--foot', '0.0', '0.75', '-0.3', '--references', str(refs), *common]) == 0
    assert len(json.loads(capsys.readouterr().out)['2']['raw_rad']) == 3
    out = tmp_path / 'gait.json'
    assert cli.gait_main(['plan', '--heading-deg', '90', '--out', str(out), '--references', str(refs), *common]) == 0
    document = json.loads(out.read_text())
    assert document['validation']['ok'] and len(document['raw_rad']['12']) == len(document['times_s'])
    assert cli.gait_main(['plan', '--heading-deg', '0', '--out', str(out), *common]) == 2
    assert json.loads(out.read_text())['raw_rad'] is None
    assert cli.gait_main(['plan', '--heading-deg', '90', '--body-height-m', '0.85', '--out', str(out), *common]) == 2
    assert json.loads(out.read_text())['joints_rad']['1'][0] is None


def cli_repo(path):
    from conftest import REPO
    return REPO / path
