# Motor bench: single-motor CAN ID check, rotation and maximum-torque test

`scripts/motor_bench.py` + `.html` test one CyberGear that is **not** one of the robot's 18.
It is independent of the robot stack: no ROS, no `motor_controller`, its own port (8766),
and no changes to the operator dashboard or spider control. It refuses to open a bus while
any robot CAN process runs (a running robot controller included). It never writes zeros or
CAN IDs.

## Start (Pranav, on the Pi)

```bash
cd ~/tocatl && .venv/bin/python scripts/motor_bench.py      # prints the URL; no CAN traffic yet
# on the Mac:
ssh -N -L 8766:127.0.0.1:8766 tocatl@tocatl.local           # then open http://127.0.0.1:8766
```

Checklist before **Open bus**: motor clamped, end effector resting on the scale, power cutoff
in reach, fused harness OK, no robot controller running. If robot motors share the bus, keep
them unpowered or make sure the test motor's ID differs from theirs.

## 1. Bus and CAN ID

- Choose `can0` or `can1` and **Open bus**. The link must be UP at 1 Mbit/s.
- **Scan for motors** asks IDs 1–127 for their device ID (comm type 0, read-only). It lists
  each responder with its MCU UID. A factory CyberGear is ID 127.
- Robot IDs (11–63) need the acknowledgement checkbox. Two UIDs answering the same ID is
  reported as a conflict and refused.

## 2. Rotation

- Each move enables operation control. Before enabling, the bench writes the drive torque
  limit (0x700B), checks it reads back, and restores the original value after stopping.
- Moves ramp at the set speed (up to 10 rad/s) with kp ≤ 100 and kd ≤ 5. **Spin** is constant
  speed (kp 0).
- Positions are shown relative to a host-side reference (**Set position reference here**).
  The motor's own position range is ±4π rad.

## 3. Maximum-torque test

The output is blocked by the scale. Torque is commanded as feedforward with kp 0, so all of
it goes into the scale.

Formula: measured torque = (scale kg − zero kg) × 9.80665 × r. Here r is the distance from
the motor axis to the contact point, and the force must be perpendicular to the arm.

1. **Scale zero:** with the motor off and the effector resting on the scale, record the
   reading in *Scale zero*.
2. **Direction check:** 0.3 Nm for 2 s. Confirm the scale reading rises. If it doesn't, flip
   *Press direction*.
3. **Sweep:** the default steps are 1–12 Nm with a 3 s hold, 5 s rest and a 4 Nm/s ramp. The
   drive limit is set to the largest step + 0.25 Nm (at most 12 Nm, the CyberGear peak).
   Tick *Wait for Next* to pause after each step.
4. Enter each scale reading in the results table. The table shows:
   - reported torque (the motor's own estimate)
   - iq
   - Nm/A
   - measured torque
   - measured ÷ commanded

### Stop conditions

The motor stops (disabled) on any of the following:
- the output moves more than the travel guard (default 20°) or faster than the speed guard
  (1 rad/s), which means the effector slipped off the scale
- any motor fault flag
- the temperature limit (default 45 °C, at most 60 °C)
- feedback older than 0.25 s
- the page stops polling for 1.5 s
- **STOP MOTOR**

If the server process itself dies, the motor keeps its last command. The power cutoff is
the real stop.

### Results

Results go to `~/tocatl-runs/motor-bench-<id>-<UTC time>/`:
- `config.json`
- `results.json`, rewritten whenever a scale reading is entered

The page can also download a CSV.

Above 4 Nm the CyberGear is past its continuous rating. Keep holds short and watch the
temperature.
