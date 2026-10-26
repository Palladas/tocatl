"""Command-line tools: tocatl_ik, tocatl_gait, tocatl_fit. Offline only; nothing opens CAN."""
from __future__ import annotations

import argparse
from dataclasses import fields
import json
from math import radians
from pathlib import Path
import sys

import numpy as np

from tocatl_kinematics.body import BodyPose, body_ik, body_to_leg, leg_to_body, neutral_stance, raw_targets
from tocatl_kinematics.gait import TripodGait, plan
from tocatl_kinematics.geometry import load_geometry
from tocatl_kinematics.joint_map import load_joint_map, load_references, motor_ids
from tocatl_kinematics.leg import KinematicsError, fk, ik, knee_position, pitch_axis_position
from tocatl_kinematics.validate import CommandLimits, validate_trajectory

# Reference palette (dataviz skill references/palette.md), light surface.
SURFACE, INK, INK_2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
SERIES = ('#2a78d6', '#eb6834', '#1baf7a')


def _common(parser):
    parser.add_argument('--geometry', default='config/robot-geometry.json')
    parser.add_argument('--joint-map', default='config/joint-map.json')
    parser.add_argument('--references', action='append', default=[],
                        help='session raw references: a homing result or declared {"raw_rad": {...}} file (repeatable)')


def _load(args):
    geometry, joint_map = load_geometry(args.geometry), load_joint_map(args.joint_map)
    for path in args.references:
        refs, source = load_references(path)
        joint_map = joint_map.with_references(refs, source)
    unverified = len(geometry.unverified) + len(joint_map.unverified)
    accepted = len(geometry.accepted) + len(joint_map.accepted)
    if unverified:
        print(f'WARNING: {unverified} geometry/joint-map values are unverified; results are not fit for motion.',
              file=sys.stderr)
    if accepted:
        print(f'Note: {accepted} geometry/joint-map values are accepted legacy values, not re-measured.', file=sys.stderr)
    return geometry, joint_map


def _raw_or_none(joint_map, leg, q):
    try:
        return [round(v, 5) for v in joint_map.to_raw(leg, q)]
    except ValueError:
        return None


def ik_main(argv=None):
    parser = argparse.ArgumentParser(prog='tocatl_ik', description='Joint angles (and raw targets) for foot positions.')
    sub = parser.add_subparsers(dest='command', required=True)
    leg = sub.add_parser('leg', help='one leg: foot position in base_link (or leg_N with --leg-frame)')
    leg.add_argument('--leg', type=int, required=True)
    leg.add_argument('--foot', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
    leg.add_argument('--leg-frame', action='store_true')
    stance = sub.add_parser('stance', help='all legs: neutral stance with an optional body pose')
    stance.add_argument('--radius', type=float, default=0.40, help='foot distance from each yaw axis (m)')
    stance.add_argument('--height', type=float, default=0.30, help='mounting plane above ground (m)')
    for name in ('x', 'y', 'z'):
        stance.add_argument(f'--body-{name}', type=float, default=0.0, help='body shift (m)')
    for name in ('roll', 'pitch', 'yaw'):
        stance.add_argument(f'--body-{name}', type=float, default=0.0, help='body rotation (deg)')
    for p in (leg, stance):
        _common(p)
        p.add_argument('--branch', choices=('knee_down', 'knee_up'), default='knee_down')
    args = parser.parse_args(argv)
    geometry, joint_map = _load(args)
    try:
        if args.command == 'leg':
            geo = geometry.leg(args.leg)
            foot = np.array(args.foot) if args.leg_frame else body_to_leg(args.foot, geo)
            solutions = {args.leg: ik(foot, geo, args.branch)}
        else:
            pose = BodyPose(args.body_x, args.body_y, args.body_z,
                            radians(args.body_roll), radians(args.body_pitch), radians(args.body_yaw))
            solutions = body_ik(geometry, neutral_stance(geometry, args.radius, args.height), pose, args.branch)
    except KinematicsError as error:
        print(f'IK failed: {error}', file=sys.stderr)
        return 1
    print(json.dumps({str(leg): dict(q_rad=[round(v, 5) for v in s.q], raw_rad=_raw_or_none(joint_map, leg, s.q),
                                     motor_ids=list(motor_ids(leg)), reach_margin_m=round(s.reach_margin_m, 4),
                                     limit_margin_rad=round(s.limit_margin_rad, 4), condition=round(s.condition, 2))
                      for leg, s in solutions.items()}, indent=2))
    return 0


def _gait_args(parser):
    defaults = TripodGait()
    for field in fields(TripodGait):
        flag = '--' + field.name.replace('_', '-')
        parser.add_argument(flag, type=type(getattr(defaults, field.name)), default=getattr(defaults, field.name))
    parser.add_argument('--heading-deg', type=float, help='walking direction in base_link (overrides --heading-rad)')
    parser.add_argument('--max-motor-velocity', type=float, default=CommandLimits.max_motor_velocity_rad_s,
                        help='rad/s; mirrors hardware.yaml. Report only: this tool never changes controller limits')
    _common(parser)


def _gait_from(args):
    values = {f.name: getattr(args, f.name) for f in fields(TripodGait)}
    if args.heading_deg is not None:
        values['heading_rad'] = radians(args.heading_deg)
    return TripodGait(**values)


def trajectory_document(trajectory, report, joint_map):
    raw = {}
    for leg, q in trajectory.joints.items():
        rows = [_raw_or_none(joint_map, leg, row) if not np.isnan(row).any() else None for row in q]
        if all(rows):
            for index, motor_id in enumerate(motor_ids(leg)):
                raw[str(motor_id)] = [row[index] for row in rows]
    return dict(summary=trajectory.summary(), validation=report, times_s=np.round(trajectory.times, 4).tolist(),
                joints_rad={str(l): [None if np.isnan(r).any() else np.round(r, 5).tolist() for r in q]
                            for l, q in trajectory.joints.items()},
                raw_rad=raw or None,
                raw_note=None if raw else 'No raw targets: pass --references for this session (homing + yaw).')


def gait_main(argv=None):
    parser = argparse.ArgumentParser(prog='tocatl_gait', description='Plan, validate and preview a tripod gait.')
    sub = parser.add_subparsers(dest='command', required=True)
    plan_parser = sub.add_parser('plan', help='write trajectory JSON + validation report')
    plan_parser.add_argument('--out', type=Path, required=True)
    preview_parser = sub.add_parser('preview', help='write a PNG preview')
    preview_parser.add_argument('--out', type=Path, required=True)
    for p in (plan_parser, preview_parser):
        _gait_args(p)
    args = parser.parse_args(argv)
    geometry, joint_map = _load(args)
    gait = _gait_from(args)
    trajectory = plan(geometry, gait)
    report = validate_trajectory(trajectory, geometry, joint_map,
                                 CommandLimits(max_motor_velocity_rad_s=args.max_motor_velocity))
    if args.command == 'plan':
        args.out.write_text(json.dumps(trajectory_document(trajectory, report, joint_map), indent=1, allow_nan=False) + '\n')
    else:
        preview(trajectory, report, geometry, args.out)
    print(f"{'OK' if report['ok'] else 'NOT OK'}: peak motor velocity {report['peak_motor_velocity_rad_s']:.3f} rad/s, "
          f"shortest feasible period {report['min_feasible_period_s']:.2f} s, "
          f"min reach margin {(report['min_reach_margin_m'] or 0) * 1000:.0f} mm, groups {trajectory.groups}")
    for violation in report['violations']:
        print('  - ' + violation)
    if not report['verified']:
        print(f"  ({report['unverified_values']} unverified geometry/joint-map values)")
    print(f'Wrote {args.out}')
    return 0 if report['ok'] else 2


def preview(trajectory, report, geometry, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 9, 'text.color': INK, 'axes.labelcolor': INK_2, 'xtick.color': INK_2,
                         'ytick.color': INK_2, 'axes.edgecolor': GRID, 'axes.facecolor': SURFACE,
                         'figure.facecolor': SURFACE, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
                         'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False})
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    top, height, joints, speed = axes.flat
    group_of = {leg: index for index, group in enumerate(trajectory.groups) for leg in group}
    mounts = np.array([[geometry.leg(l).mount_x_m, geometry.leg(l).mount_y_m] for l in sorted(geometry.legs)])
    order = np.argsort(np.arctan2(mounts[:, 1], mounts[:, 0]))
    outline = mounts[order]
    top.fill(outline[:, 0], outline[:, 1], color=GRID, alpha=0.6, linewidth=0)
    for leg, feet in trajectory.feet_body.items():
        color = SERIES[group_of[leg]]
        top.plot(feet[:, 0], feet[:, 1], color=color, linewidth=2)
        q0 = trajectory.joints[leg][0]
        if not np.isnan(q0).any():
            geo = geometry.leg(leg)
            chain = [leg_to_body(p, geo) for p in ((0, 0, 0), pitch_axis_position(q0, geo), knee_position(q0, geo), fk(q0, geo))]
            chain = np.array(chain)
            top.plot(chain[:, 0], chain[:, 1], color=INK_2, linewidth=1, marker='o', markersize=3)
        end = feet[0]
        top.annotate(f'leg {leg}', end[:2], xytext=(6, 6), textcoords='offset points', color=INK)
    top.plot([], [], color=SERIES[0], linewidth=2, label=f'tripod {trajectory.groups[0]}')
    top.plot([], [], color=SERIES[1], linewidth=2, label=f'tripod {trajectory.groups[1]}')
    top.arrow(0, 0, 0.12 * np.cos(trajectory.gait.heading_rad), 0.12 * np.sin(trajectory.gait.heading_rad),
              width=0.004, color=INK, length_includes_head=True)
    top.set(aspect='equal', title='Top view (base_link): foot paths, legs at t = 0, heading arrow',
            xlabel='x forward (m)', ylabel='y left (m)')
    top.legend(loc='upper left')
    for index, leg in enumerate(trajectory.groups[i][0] for i in range(2)):
        feet = trajectory.feet_body[leg]
        height.plot(trajectory.times, (feet[:, 2] - feet[0, 2]) * 1000, color=SERIES[index], linewidth=2,
                    label=f'leg {leg} (tripod {index + 1})')
    height.set(title='Foot lift', xlabel='time (s)', ylabel='lift above stance (mm)')
    height.legend(loc='upper right')
    leg = trajectory.groups[0][0]
    for index, name in enumerate(('hip yaw', 'hip pitch', 'knee')):
        joints.plot(trajectory.times, trajectory.joints[leg][:, index], color=SERIES[index], linewidth=2, label=name)
    joints.set(title=f'Leg {leg} joint angles', xlabel='time (s)', ylabel='joint angle (rad)')
    joints.legend(loc='center right')
    per_motor = report['per_motor']
    ids = sorted(per_motor)
    limit = CommandLimits.max_motor_velocity_rad_s
    peaks = [per_motor[i]['peak_velocity_rad_s'] for i in ids]
    retired = set(CommandLimits.out_of_service)
    bars = speed.bar([str(i) for i in ids], peaks, color=[GRID if i in retired else SERIES[0] for i in ids], width=0.6)
    for bar, motor_id in zip(bars, ids):
        if motor_id in retired:
            bar.set_hatch('//')
            bar.set_edgecolor(INK_2)
            speed.annotate('out of\nservice', (bar.get_x() + bar.get_width() / 2, bar.get_height()), xytext=(0, 3),
                           textcoords='offset points', ha='center', color=INK_2, fontsize=8)
    speed.axhline(limit, color=INK_2, linestyle='--', linewidth=1)
    speed.annotate(f'controller cap {limit} rad/s', (0, limit), xytext=(0, 4), textcoords='offset points', color=INK_2)
    speed.set(title='Peak raw motor velocity', xlabel='motor ID', ylabel='rad/s')
    status = 'OK' if report['ok'] else 'NOT OK: ' + '; '.join(report['violations'][:2])
    fig.suptitle(f"Tripod gait: stride {trajectory.gait.stride_m * 100:.0f} cm, period {trajectory.gait.period_s:g} s, "
                 f"heading {np.degrees(trajectory.gait.heading_rad):.0f}°  |  {status}"
                 + ('' if report['verified'] else '  |  UNVERIFIED geometry'), color=INK, fontsize=11)
    fig.savefig(out, dpi=120)
    plt.close(fig)


def fit_main(argv=None):
    from tocatl_kinematics import fit
    parser = argparse.ArgumentParser(prog='tocatl_fit', description='Fit joint-map terms from dashboard recordings.')
    sub = parser.add_subparsers(dest='command', required=True)
    coupling = sub.add_parser('coupling', help='hip-only recording -> knee coupling')
    coupling.add_argument('--recording', type=Path, required=True)
    coupling.add_argument('--leg', type=int, required=True)
    isolation = sub.add_parser('isolation', help='single-joint recording -> how much another motor moved')
    isolation.add_argument('--recording', type=Path, required=True)
    isolation.add_argument('--moved', type=int, required=True)
    isolation.add_argument('--still', type=int, required=True)
    scale = sub.add_parser('scale', help='(raw, measured angle) pairs -> sign, ratio, q at reference')
    scale.add_argument('--pairs', type=Path, required=True,
                       help='JSON [{"raw_rad": .., "angle_deg": ..}, ...] (or angle_rad)')
    scale.add_argument('--raw-reference', type=float)
    check = sub.add_parser('fk', help='measured foot positions vs FK of recorded raw positions')
    check.add_argument('--poses', type=Path, required=True)
    check.add_argument('--recordings-dir', type=Path)
    for p in (coupling, isolation, scale, check):
        _common(p)
    args = parser.parse_args(argv)
    geometry, joint_map = _load(args)
    if args.command == 'coupling':
        result = fit.fit_coupling(fit.load_recording(args.recording), args.leg, joint_map)
    elif args.command == 'isolation':
        result = fit.check_isolation(fit.load_recording(args.recording), args.moved, args.still)
    elif args.command == 'scale':
        pairs = [dict(raw_rad=p['raw_rad'], angle_rad=p['angle_rad'] if 'angle_rad' in p else radians(p['angle_deg']))
                 for p in json.loads(args.pairs.read_text())]
        result = fit.fit_scale(pairs, args.raw_reference)
    else:
        data = json.loads(args.poses.read_text())
        result = fit.validate_fk(data['poses'], geometry, joint_map, args.recordings_dir or args.poses.parent)
        errors = [r['error_m'] for r in result]
        result = dict(poses=result, max_error_mm=max(errors) * 1000, mean_error_mm=float(np.mean(errors)) * 1000)
    print(json.dumps(result, indent=2))
    return 0
