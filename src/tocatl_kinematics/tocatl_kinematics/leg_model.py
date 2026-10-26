"""Offline assessment of the recovered leg model; no hardware dependencies."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from .legacy_geometry import LegGeometry, Point3, foot_position_from_joint_angles


def check_geometry(geometry):
    if not all(math.isfinite(v) for v in asdict(geometry).values()):
        raise ValueError('Geometry must be finite')
    if geometry.upper_link_mm <= 0 or geometry.lower_link_mm <= 0:
        raise ValueError('Link lengths must be positive')


def forward(joints_rad, geometry=LegGeometry()):
    check_geometry(geometry)
    if len(joints_rad) != 3 or not all(math.isfinite(v) for v in joints_rad):
        raise ValueError('Provide three finite joint angles')
    return foot_position_from_joint_angles(*joints_rad, geometry)


def inverse(foot, geometry=LegGeometry(), branch=1):
    check_geometry(geometry)
    if branch not in (-1, 1):
        raise ValueError('Knee branch must be -1 or +1')
    if not all(math.isfinite(v) for v in asdict(foot).values()):
        raise ValueError('Foot coordinates must be finite')
    x, y = foot.x_mm-geometry.hip_horizontal_offset_mm, foot.y_mm
    z = foot.z_mm-geometry.hip_vertical_offset_mm
    radial = math.hypot(x, y)
    distance = math.hypot(radial, z)
    upper, lower = geometry.upper_link_mm, geometry.lower_link_mm
    if distance < abs(upper-lower)-1e-9 or distance > upper+lower+1e-9:
        raise ValueError('Foot target is outside the geometric workspace')
    if radial < 1e-9:
        raise ValueError('Foot target is on the yaw axis; yaw is underdetermined')
    cosine = (distance*distance-upper*upper-lower*lower)/(2*upper*lower)
    knee = branch*math.acos(max(-1., min(1., cosine)))
    hip = math.atan2(-z, radial)-math.atan2(lower*math.sin(knee), upper+lower*math.cos(knee))
    return math.atan2(y, x), hip, knee


def compare_measurements(samples, geometry=LegGeometry()):
    if not samples:
        raise ValueError('Provide at least one measured pose')
    rows = []
    for sample in samples:
        joints = tuple(math.radians(v) for v in sample['joints_deg'])
        predicted = asdict(forward(joints, geometry))
        measured = sample['foot_mm']
        if len(measured) != 3 or not all(math.isfinite(v) for v in measured):
            raise ValueError('Provide three finite measured foot coordinates')
        error = [p-m for p,m in zip(predicted.values(), measured)]
        rows.append(dict(predicted_foot_mm=list(predicted.values()), measured_foot_mm=measured,
                         error_mm=error, error_norm_mm=math.sqrt(sum(e*e for e in error))))
    return dict(poses=rows, rms_position_error_mm=math.sqrt(sum(r['error_norm_mm']**2 for r in rows)/len(rows)),
                maximum_position_error_mm=max(r['error_norm_mm'] for r in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, help='JSON overrides for LegGeometry fields, in mm')
    modes = parser.add_subparsers(dest='mode', required=True)
    fk = modes.add_parser('fk')
    fk.add_argument('--joints-deg', type=float, nargs=3, required=True, metavar=('YAW','HIP','KNEE'))
    ik = modes.add_parser('ik')
    ik.add_argument('--foot-mm', type=float, nargs=3, required=True, metavar=('X','Y','Z'))
    ik.add_argument('--branch', type=int, choices=(-1,1), default=1)
    validate = modes.add_parser('validate')
    validate.add_argument('measurements', type=Path, help='JSON list of joints_deg / foot_mm measured poses')
    args = parser.parse_args()
    try:
        geometry = LegGeometry(**json.loads(args.geometry.read_text())) if args.geometry else LegGeometry()
        check_geometry(geometry)
        if args.mode == 'fk':
            result = dict(foot_mm=asdict(forward(tuple(map(math.radians, args.joints_deg)), geometry)))
        elif args.mode == 'ik':
            target = Point3(*args.foot_mm)
            joints = inverse(target, geometry, args.branch)
            result = dict(joints_deg=list(map(math.degrees,joints)),
                          reconstructed_foot_mm=asdict(forward(joints, geometry)))
        else:
            result = compare_measurements(json.loads(args.measurements.read_text()), geometry)
        print(json.dumps(dict(model='unvalidated_legacy_leg', geometry_mm=asdict(geometry), **result), indent=2, allow_nan=False))
    except (ValueError, TypeError, KeyError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
