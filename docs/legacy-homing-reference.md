# Legacy homing reference and leg 1 — 2026-10-23

Inspected `~/romela-tocatl/legacy/deployed-2026-03-15/tocatl_testing/` as source
code, without executing its startup or zero-writing routines.

## What the original code commands

| Operation | Feedforward torque | Position control and damping | Source |
|---|---:|---|---|
| Hip pitch toward shell stop | 0 Nm | target = initial - 1.5 rad, kp=1.75, kd=2; wait 3 s | `tocatl_intbootup.py:184–205` |
| Hip pitch at shell contact | stop, then device zero | re-enable afterward | same |
| Knee zero | no independent contact search | stop and zero at the shell-contact pose | same |
| Initial deployment, all legs | 0 Nm | hip target 1.40 rad, knee -0.6 rad; kp=3, kd=1 | `TEST_tocatl_bootup_STREAMER.py:284–289` |
| Loaded deployment phase | 8.5 Nm hip; 0 Nm knee | hip target 1.70 rad, kp=3, kd=2 | `TEST_tocatl_bootup_STREAMER.py:301–309` |
| Older deployment example | 8.55 Nm, commented out | hip target 1.170 rad, kp=3, kd=1 | `TEST_tocatl_intbootup_.py:247–251` |

In operation control, the position and velocity terms contribute torque in
addition to feedforward: `kp*(target-position) + kd*(target_velocity-velocity)`.
Thus the hip-stop command initially requests about 1.75*1.5 = 2.625 Nm in
magnitude from the position term when stationary. Zero feedforward is not zero
actuation. Damping opposes motion in the drive's fast control loop.

The 8.5 Nm value is explicitly a deployment/load command, not a measured homing
contact threshold. These files do not establish a three-contact repeatability
procedure or a calibrated stop-force limit. Static legacy source is evidence of
intended commands, not proof of physical validation on the current assembly.

The prior experiment used zero drive kp/kd and host-side torque control, with
very small initial caps and speed cutoffs. It was not reproducing this legacy
control method. For subsequent leg 1 work, use these gains/torque scales as the
reference and distinguish lifting from physical-stop contact. Do not copy the
all-motor startup/zero behavior into a single-leg test.

## Leg 1 target and reference

The original two-bus streamer maps leg 1 hip pitch to motor 12 and knee to motor
13 on can1 (`TEST_tocatl_bootup_STREAMER.py:62–64`); motor 11 is yaw and remains
outside this task. Homing worker support now accepts IDs 12 and 13 as well as
32 and 33, derives result bus from the canonical map, and checks hip 12's pose
when homing knee 13. Tests verify that a leg 1 worker touches only its selected
motor and does not send mode/control/enable/stop operations to leg 3.

On 2026-10-23 both CAN buses were found down and were brought up at 1 Mbps for
read-only telemetry. Motors 12 and 13 returned valid, fresh, fault-free feedback
and reported disabled. Motor 11 did not return feedback in the inspected sample;
its physical enable state is unknown. The existing worker's all-motor readiness
preflight has not been bypassed to ignore missing replies.

The operator confirmed leg 1 was against both stops. Saved observed raw reference:

- Motor 12 hip pitch: 0.80281529 rad.
- Motor 13 knee: 0.87529564 rad.
- Both on can1; last-six-observation span 0.00038148 rad each.

`config/leg1-home-reference-2026-10-23.json` preserves this as an operator-declared
reference (`calibration_valid: false`). These observations are not three separate
powered contact trials, and no device zero or persistent offset was written.

Live `/robot_state` monitoring and raw-position recording are running, with
observations in `/tmp/tocatl-leg1-identification-20261023.jsonl`. A manual hip
backoff will physically confirm ID/direction before powered leg 1 work. Leg 3's
raw home angles and measured signs must not be transferred to leg 1 by assumption.
