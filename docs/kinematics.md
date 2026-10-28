# Kinematics

`tocatl_kinematics` turns body poses and foot positions into joint angles and raw motor
targets, and plans a tripod gait. It is offline: nothing in it opens CAN or sends a
`MotorCommand`. Executing a trajectory (Level 3: `allow_hardware_motion`, measured
envelopes, homed legs) is a separate step and is not implemented yet.

**Status (2026-10-26):** Pranav accepted the legacy measurements as correct without re-measuring
them. Every geometry and joint-map value has `"status": "accepted"`. That counts as verified, and
the tools print how many values are accepted rather than measured.

The accepted model reproduces the legacy FK and the legacy motor mapping exactly
(`test_legacy_equivalence`):
- **Coxa offset.** The legacy FK applies the 45.56 mm offset before yaw. So the yaw axis sits
  0.3456 m from the body centre, and the pitch axis lies on the yaw axis (`coxa_radial_m = 0`),
  114.25 mm above the mounting plane.
- **Reference pose.** The joint angles at the homing (compressed) stop are taken from the legacy
  motor-zero pose: hip pitch θ = 134.56°, knee φ = 168.82°.
- **Joint limits.** These come from the old 2026-08-25 calibration. The knee range is exact only
  with the hip at its reference, because that calibration limited knee encoder travel.

Use the measurement procedure below to replace any value with a `measured` one.

## Conventions

Units are SI: metres and radians.

| Frame | Definition |
|---|---|
| world | Fixed to the ground, z up |
| `base_link` | REP-103 body frame (x forward, y left, z up). Origin at the body centre on the hip-yaw mounting plane |
| `leg_N` | Origin on leg N's yaw axis at the mounting plane, z up. x points radially outward along the mount angle |

| Joint | Zero | Positive |
|---|---|---|
| `hip_yaw` | Leg along its mount angle | Counter-clockwise about z |
| `hip_pitch` | Thigh horizontal | Thigh raised |
| `knee` | Shin in line with the thigh | Shin folds downward (walking posture, `knee_down` branch) |

The leg model is a yaw joint, then a coxa offset that rotates with yaw (`coxa_radial_m`,
`coxa_vertical_m` to the pitch axis), then a thigh (`thigh_m`, pitch axis to knee axis),
then a shin (`shin_m`, knee axis to foot contact point).

### Joint angles and raw encoder radians

`config/joint-map.json` holds four values per motor: `sign`, `ratio` (motor turns per joint
turn), `q_at_reference_rad`, and for knees `coupling_raw_per_hip_rad`. The mapping is:

```
raw_hip  = ref_hip  + sign_h * ratio_h * (q_hip  - q_hip_ref)
raw_knee = ref_knee + sign_k * ratio_k * (q_knee - q_knee_ref) + coupling * (q_hip - q_hip_ref)
```

The raw references `ref_*` are re-anchored every power session. They are not stored in the map:
- **Pitch/knee:** the session's homing result. The reference pose is the compressed homing stop.
- **Yaw:** a declared reference pose with each yaw at 0, in a file like
  `{"source": "...", "raw_rad": {"11": ..., "21": ...}}`. There is no yaw homing yet.

Pass these files to the tools with `--references` (repeatable). Without them the tools output
joint angles only.

### Knee coupling

The knee *joint* is independent of the hip, but under the accepted legacy mapping the knee
*encoder* follows the hip 1:1 (`coupling = 1`): `motor_knee = −(pitch + knee) + K`. That fits a
knee motor that reads the shin's absolute angle through a linkage. A manual hip-only move on
2026-10-23 read hip +47° and knee −41° raw, a ratio of about 0.87. A hip-only recording run
through `tocatl_fit coupling` confirms or corrects the value.

## Tools

Run these from the workspace root. Geometry and joint-map paths default to `config/`.

```bash
# One leg: foot position in base_link (or --leg-frame)
ros2 run tocatl_kinematics tocatl_ik leg --leg 2 --foot 0.0 0.75 -0.3
# All legs: neutral stance, optionally with a body shift/rotation (degrees)
ros2 run tocatl_kinematics tocatl_ik stance --radius 0.40 --height 0.30 --body-roll 5
# Tripod gait: trajectory JSON + validation, or a PNG preview
ros2 run tocatl_kinematics tocatl_gait plan --heading-deg 90 --out /tmp/gait.json \
  --references ~/tocatl-runs/<session>/hip-knee-homing.json --references yaw-refs.json
ros2 run tocatl_kinematics tocatl_gait preview --heading-deg 90 --out /tmp/gait.png
```

On the Mac, without ROS:
`PYTHONPATH=src/tocatl_kinematics python3 -c "from tocatl_kinematics.cli import gait_main; gait_main()" preview ...`.

### Validation

Validation checks each trajectory against the following. It reports problems and never changes limits.
- The joint limits.
- The workspace reach margin (≥ 10 mm).
- The controller's `max_motor_velocity_rad_s` (0.21, from `hardware.yaml`) and `max_position_step_rad` (0.1).
- Any out-of-service motors (`CommandLimits.out_of_service`; none since 51 returned on 2026-10-27).

It also reports the **shortest feasible period** under the velocity cap.

**Note:** walking forward moves leg 5's yaw. That ruled out forward gaits while 51 was
disconnected. Since 51 returned (2026-10-27), every heading is allowed, but yaw needs a
declared reference before it can be commanded.

## Measurement procedure

Prepare before measuring:
- Run the Level 1 telemetry controller from the operator dashboard. Motors stay **disabled** throughout.
- Support the robot with its legs clear.
- Use **Record telemetry** on the dashboard for every recording. It saves JSONL to
  `<run folder>/recordings/` and adds no CAN traffic.

Commit only the measured JSON values, each with `"status": "measured"`, `measured_by`, `date`
and `method`. Raw recordings stay in `~/tocatl-runs`.

1. **Body layout.** Number the legs around the body. Measure each yaw axis position from the
   body centre (`mount_x_m`, `mount_y_m`) and the leg's outward direction (`mount_angle_rad`).
2. **Link dimensions,** axis to axis with calipers or tape:
   - yaw axis → pitch axis, horizontal and vertical (`coxa_radial_m`, `coxa_vertical_m`)
   - pitch axis → knee axis (`thigh_m`)
   - knee axis → foot contact point (`shin_m`)
3. **Coupling.** For each leg:
   1. Hold the shin fixed relative to the thigh (by hand or with tape).
   2. Record about 20 s while swinging only the hip through at least 0.3 rad. Use a label like `leg2-hip-only`.
   3. Run `tocatl_fit coupling --recording <file> --leg 2`.
   4. Then record a knee-only move and run `tocatl_fit isolation --recording <file> --moved 23 --still 22`.
      The hip should barely move.
4. **Sign, ratio, reference angle.**
   1. Home the leg (Level 2) so its compressed-stop raw reference exists.
   2. Then, with motors disabled, set the joint to 3–4 angles measured with an inclinometer or protractor.
   3. Note the raw value at each angle from a recording, in a pairs file like
      `[{"raw_rad": .., "angle_deg": ..}]`.
   4. Run `tocatl_fit scale --pairs pairs.json --raw-reference <homed raw>`.
   5. Yaw: declare the yaw-0 pose and capture its raw value as that motor's reference.
5. **Joint limits.** Record the joint angles at the mechanical stops. Set the limits with margin
   inside them.
6. **FK check.**
   1. Record 4–6 poses per leg (hold each still for 2 s).
   2. Tape-measure each foot contact point in `base_link`.
   3. Write `{"poses": [{"leg", "recording", "t", "foot_body_m": [x, y, z]}]}`.
   4. Run `tocatl_fit fk --poses poses.json --references ...`.
   5. **Acceptance:** under 10 mm error per pose. Fix the geometry first if any pose exceeds it.

## Tests

```bash
PYTHONPATH=src/tocatl_kinematics python3 -m pytest -q src/tocatl_kinematics/test
```

The tests cover:
- FK/IK round trips, and that targets that are unreachable or out of limits raise.
- The analytic Jacobian.
- The joint map: round trip, ratio, coupling and velocities.
- Body IK/FK.
- Gait continuity, with stance feet fixed in the world.
- Validation of the speed cap, out-of-service motors, limits and reach.
- The fit routines and the CLIs.
- The restored legacy geometry tests.

## Running motions on the robot (Level 3, feet in the air)

`tocatl_kinematics.motions` plans nudge, pose and circle motions in each leg's own plane,
using hip pitch and knee only. Yaw has no session reference, so yaw is never commanded.
`tocatl_play` (`tocatl_kinematics.player`) does four things:
1. Reads the session homes and the measured pose from `/robot_state`.
2. Plans the motion and refuses if anything would break the joint limits, the controller
   envelope or the speed cap.
3. Streams `MotorCommand` holds until the operator arms.
4. Plays the motion, then disarms.

Every command names all the motors the controller actuates: the selected legs follow the
plan, and the other actuated legs hold their measured pose. The actuated set comes from
`--actuated-legs`, or from the controller's `actuation_motor_ids` parameter if that's
omitted.

Run it from the operator dashboard's Level 3 panel (see `operator-dashboard.md`). To preview
without sending anything:

```bash
ros2 run tocatl_kinematics tocatl_play --legs 1 --actuated-legs 1,2 --kind nudge --dry-run
```

Durations are set so no motor exceeds `--max-speed-rad-s` (default 0.1). For example,
moving the foot from the folded stop to (r 0.35, z −0.05) takes about 55 s at 0.1 rad/s.

### Wave (2026-10-28)

`--kind wave` is a circle whose phase lags by `--phase-deg` per leg number (default 60°).
Because the legs are numbered around the body, the circles ripple around the robot. A
negative phase reverses the ripple's direction. Every leg starts at its own phase point
and returns to its start pose.

Example: about 20 s on all six legs, feet in the air. Planned peak hip offset is 1.42 rad,
inside the feet-in-air envelope of 1.6.

```bash
ros2 run tocatl_kinematics tocatl_play --legs 1,2,3,4,5,6 --actuated-legs 1,2,3,4,5,6 --kind wave \
  --r-m 0.32 --z-m 0.02 --radius-m 0.08 --loops 5 --phase-deg 60 --max-speed-rad-s 1.8 --hold-s 1.0 --kp 30 --kd 2
```
