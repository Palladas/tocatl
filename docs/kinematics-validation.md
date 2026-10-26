# One-leg kinematics validation

The first implementation is an offline tool. It does not read CAN, move motors,
apply homing offsets, select a standing pose, or assert a validated calibration.
The historical module remains unchanged. `leg_model.py` adds finite-input and
geometry validation, explicit unreachable-target rejection, both knee branches,
and measured-position error reporting.

## Model assumptions to verify

Recovered geometry, in millimeters:

| Quantity | Value |
| --- | ---: |
| Upper pitch-pivot to knee-pivot distance | 330.3455 |
| Knee-pivot to modeled foot point distance | 458.0 |
| Fixed X offset to pitch-chain origin | 45.5561 |
| Fixed Z offset to pitch-chain origin | 114.25 |

Measure pivot centers, not plastic link outlines. The foot point needs to be
chosen consistently, including whether the contact pad adds an offset.

The legacy local frame has pitch-chain origin `(45.5561, 0, 114.25)` mm.
At yaw=hip=knee=0, both links point along +X and the foot is
`(833.9016, 0, 114.25)` mm. Yaw rotates link directions about +Z; positive
hip pitch rotates them toward -Z. Geometric knee angle is relative to the upper
link, so lower-link pitch is hip+knee. These are model joint angles, not raw
encoder angles or displacements from the homing endstop.

The fixed X offset is outside the yaw rotation in the recovered equations.
Verify its actual physical interpretation rather than assuming the frame is
the yaw motor center. Body mount transforms and opposite-leg mirroring are
not validated by this one-leg model.

The recovered motor conversion uses hip angle plus knee angle for the knee
motor, with a sign inversion and historical offsets. Its mathematical inverse
is consistent, but current raw encoder directions/offsets are not established
by those historical constants. A fresh software home does not establish the
straight-link geometric pose. Motor/linkage ratios and absolute-angle coupling
must be checked against the physical transmission.

## Offline commands

From the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tocatl_kinematics
source install/setup.bash
ros2 run tocatl_kinematics leg_model fk --joints-deg 0 0 90
ros2 run tocatl_kinematics leg_model ik --foot-mm 500 0 -300
ros2 run tocatl_kinematics leg_model ik --foot-mm 500 0 -300 --branch -1
```

These are mathematical examples, not recommended hardware targets. Either
geometric branch may violate physical limits or use the wrong linkage
configuration. The tool rejects targets outside the link-length workspace and
on the underdetermined yaw axis, but does not check collisions or motor limits.
Straight/folded workspace boundaries are also singular for motion planning.

Override measured geometry with `--geometry /path/to/geometry.json` before the
subcommand. JSON keys match `LegGeometry` fields; omitted fields use legacy
values. For example:

```json
{"upper_link_mm": 330.3455, "lower_link_mm": 458.0}
```

## First physical measurements

Start with one supported leg, without commanding a new pose. First confirm the
two link lengths and identify the physical origin/axes described above. Then
record at least three distinct reachable poses, measuring geometric yaw, hip
and relative knee angle independently of encoder zero, plus the foot coordinates
in the same local frame. Avoid straight/folded singular poses for calibration.
Also record the three raw encoder angles at each pose to establish the current
motor-to-joint mapping afterward. This tool's first comparison uses measured
geometric joint angles; it deliberately does not infer them from saved homes.

Create a JSON list with one object per measured pose:

- `joints_deg`: `[yaw, hip, relative_knee]` in degrees.
- `foot_mm`: `[x, y, z]` measured in millimeters.

Then run:

```bash
ros2 run tocatl_kinematics leg_model validate /path/to/measured-poses.json
```

The report gives signed predicted-minus-measured errors, each pose's Euclidean
position error, RMS position error and maximum error. It does not automatically
fit dimensions or declare physical accuracy from a fixed tolerance. Compare
errors with measurement uncertainty. Once this model agrees with measurements,
calibrate raw encoders against geometric angles, establish measured limits,
and test small one-leg foot displacements before whole-body standing.
