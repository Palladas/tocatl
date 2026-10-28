# TOCATL command guide

Find what you want to do in the first table, then copy the commands from that section.
Everything here runs on the Pi in `~/tocatl`. Commands that open CAN or move a motor are run
by Pranav.

**Before any CAN or motion command:**
- robot supported, legs clear
- power cutoff in reach
- fused harness OK
- no other process owning the bus

**The power cutoff is the real stop.**

## What do you want to do?

| I want to… | Go to |
|---|---|
| Open a terminal ready for ROS | [A. Terminal setup](#a-terminal-setup) |
| Bring CAN up after a reboot or a freeze | [B. CAN links](#b-can-links) |
| Just watch the motors (no motion) | [C1. Level 1 telemetry](#c1-level-1--read-only-telemetry) |
| Home legs | [C2. Level 2 homing](#c2-level-2--homing) + [D. Homing](#d-homing) |
| Move legs in the air (nudge, pose, circle, wave) | [C3. Level 3](#c3-level-3--feet-in-the-air) + [E. Motions](#e-motions) |
| Stand up | [C4. Level 4](#c4-level-4--standing) + [E5. Stand](#e5-stand-level-4) |
| Arm, disarm, or stop | [F. Arm / disarm / stop](#f-arm--disarm--stop) |
| See status, faults, why something stopped | [G. Watch and debug](#g-watch-and-debug) |
| Record telemetry | [G. Watch and debug](#g-watch-and-debug) |
| Test a separate motor (CAN ID, rotation, max torque) | [H. Motor bench](#h-motor-bench-separate-motor) |
| Get the latest code onto the Pi | [I. Update the Pi](#i-update-the-pi) |

**The three terminals:**

| Terminal | Runs | Behaviour |
|---|---|---|
| **T1** | the controller | stays running |
| **T2** | the player | stays running during a motion |
| **T3** | services: home, arm, disarm, status | one-shot commands |

Start every terminal with [A](#a-terminal-setup).

---

## A. Terminal setup
```bash
ssh tocatl@tocatl.local
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## B. CAN links
Run this after every reboot, or after a bus freeze, with the controller stopped:
```bash
for b in can0 can1; do sudo ip link set $b down; sudo ip link set $b type can bitrate 1000000; sudo ip link set $b up; done
ip -details -statistics link show can0 | head -5      # want: UP, ERROR-ACTIVE, bitrate 1000000
ip -details -statistics link show can1 | head -5
ps -eo pid,args | grep -E "[m]otor_controller|[t]ocatl_play|[m]otor_bench"   # must print nothing before starting
```

## C. Start a controller (T1)
Run only one controller at a time. Ctrl-C stops it; the node disables the motors and restores
their limits on the way out. Set these two variables first:
```bash
C=src/tocatl_bringup/config
RUN=~/tocatl-runs/cli-$(date -u +%Y%m%dT%H%M%SZ); mkdir -p $RUN
```

### C1. Level 1 · read-only telemetry
```bash
ros2 run tocatl_lowlevel motor_controller --ros-args --params-file $C/hardware.yaml 2>&1 | tee $RUN/controller.log
```

### C2. Level 2 · homing
Homing only, no motion. Edit the legs list as needed:
```bash
ros2 run tocatl_lowlevel motor_controller --ros-args --params-file $C/homing-hip-knee.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false \
  -p homing.output_path:=$RUN/hip-knee-homing.json 2>&1 | tee $RUN/controller.log
```

### C3. Level 3 · feet in the air
Hip pitch and knee only; the yaws stay unpowered. Limits: kp ≤ 30, kd ≤ 2, 1.5 Nm cap.

All six legs:
```bash
IDS="[12,13,22,23,32,33,42,43,52,53,62,63]"
ros2 run tocatl_lowlevel motor_controller --ros-args --params-file $C/homing-hip-knee.yaml --params-file $C/motion-feet-air.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false -p homing.output_path:=$RUN/hip-knee-homing.json \
  -p allow_hardware_motion:=true -p "actuation_motor_ids:=$IDS" -p "telemetry_motor_ids:=$IDS" 2>&1 | tee $RUN/controller.log
```
For fewer legs, change `homing.legs` and `IDS` together. For example, legs 1 and 2 are
`[1,2]` and `IDS="[12,13,22,23]"`.

### C4. Level 4 · standing
All 18 motors, including the yaws. Limits: kp ≤ 100, kd ≤ 5, 12 Nm cap.
```bash
IDS="[11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63]"
ros2 run tocatl_lowlevel motor_controller --ros-args --params-file $C/homing-hip-knee.yaml --params-file $C/motion-standing.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false -p homing.output_path:=$RUN/hip-knee-homing.json \
  -p allow_hardware_motion:=true -p "actuation_motor_ids:=$IDS" -p "telemetry_motor_ids:=$IDS" 2>&1 | tee $RUN/controller.log
```

## D. Homing (T3)
Use this with a C2, C3 or C4 controller running, robot supported, legs clear.

**Home legs one at a time.** Home-all currently fails on the leg 1/4 pair, a freshness
regression from `8c0b041`:
```bash
for n in 1 2 3 4 5 6; do ros2 service call /motors/leg_$n/home std_srvs/srv/Trigger; done
```
- Each call returns straight away and homing runs in the background.
- Wait for `Leg N pitch/knee READY` in T1's log before calling the next leg. Alternatively,
  run them one by one:
  ```bash
  ros2 service call /motors/leg_1/home std_srvs/srv/Trigger
  ```

**Level 4 only.** After every leg is READY, declare the yaw references. This holds the yaws
where they are now:
```bash
ros2 service call /motors/declare_yaw_reference std_srvs/srv/Trigger
```

**Check readiness:**
```bash
ros2 topic echo /robot_state --field legs --once | grep -E "leg_id|ready|status_reason"
```

## E. Motions (T2 player, then arm in T3)
Every motion follows the same pattern:
1. **Dry run.** Add `--dry-run`. It plans and checks only; nothing is sent.
2. **Real run.** Remove `--dry-run`. The player holds the start pose and waits up to 120 s.
3. **Arm in T3** ([F](#f-arm--disarm--stop)). The motion plays.
4. **When it finishes**, the player disarms by itself and exits.

**Rules:**
- `--actuated-legs` must equal the controller's legs.
- `--legs` lists the legs that move; any others hold their pose.
- Every leg must be homed first.

### E1. Nudge
Small joint change and back. Use it as the first test:
```bash
ros2 run tocatl_kinematics tocatl_play --legs 1,2,3,4,5,6 --actuated-legs 1,2,3,4,5,6 --kind nudge \
  --d-pitch-rad -0.2 --d-knee-rad -0.2 --max-speed-rad-s 0.5 --kp 30 --kd 2
```

### E2. Pose
Foot to (r, z), hold, and back:
```bash
ros2 run tocatl_kinematics tocatl_play --legs 1,2,3,4,5,6 --actuated-legs 1,2,3,4,5,6 --kind pose \
  --r-m 0.32 --z-m 0.02 --hold-s 2 --max-speed-rad-s 1.0 --kp 30 --kd 2
```

### E3. Circle
All legs in sync, about 20 s:
```bash
ros2 run tocatl_kinematics tocatl_play --legs 1,2,3,4,5,6 --actuated-legs 1,2,3,4,5,6 --kind circle \
  --r-m 0.32 --z-m 0.02 --radius-m 0.08 --loops 5 --max-speed-rad-s 1.8 --hold-s 1.0 --kp 30 --kd 2
```

### E4. Wave
Circles rippling around the body, about 20 s:
```bash
ros2 run tocatl_kinematics tocatl_play --legs 1,2,3,4,5,6 --actuated-legs 1,2,3,4,5,6 --kind wave \
  --r-m 0.32 --z-m 0.02 --radius-m 0.08 --loops 5 --phase-deg 60 --max-speed-rad-s 1.8 --hold-s 1.0 --kp 30 --kd 2
```
- `--phase-deg` sets the lag between neighbouring legs: 60 gives one ripple around the body,
  120 gives two, and a negative value reverses the direction.
- These settings peak at about 1.42 rad hip offset, inside the 1.6 feet-in-air limit.

### E5. Stand (Level 4)
Needs every leg homed and the yaws declared:
```bash
ros2 run tocatl_kinematics tocatl_play --legs 1,2,3,4,5,6 --actuated-legs 1,2,3,4,5,6 --kind stand \
  --kp 80 --kd 4 --r-m 0.25 --rest-height-m 0.10 --stand-height-m 0.15 --mass-kg 16 --max-speed-rad-s 0.5
```

| Setting | What it changes |
|---|---|
| `--r-m` | foot distance from the yaw axis (m); smaller means less hip torque |
| `--z-m` | foot height from the mounting plane (m); negative is down |
| `--radius-m` | circle or wave size, 0.005–0.1 m |
| `--loops` | 1–5 |
| `--max-speed-rad-s` | at most 5; lower is gentler |
| `--hold-s` | hold time at the target |
| `--kp`, `--kd` | gains; Level 3 allows at most 30 / 2, Level 4 at most 100 / 5 |

## F. Arm / disarm / stop (T3)
```bash
ros2 service call /motors/enable std_srvs/srv/SetBool "{data: true}"    # ARM: the waiting motion starts
ros2 service call /motors/enable std_srvs/srv/SetBool "{data: false}"   # DISARM
```
**To stop**, in this order:
1. Ctrl-C the player (T2).
2. DISARM (T3).
3. Ctrl-C the controller (T1), if you're finished.
4. If anything looks wrong, use the power cutoff.

## G. Watch and debug (T3)
```bash
ros2 topic echo /robot_state --field safety_state        # DISABLED / ARMED / FAULT
ros2 topic echo /robot_state --field status_reason       # why it faulted
ros2 topic hz /robot_state                               # is telemetry flowing?
grep -E "Readiness cleared|Motion fault|READY|ERROR|WARN" $RUN/controller.log | tail -20
ls -td ~/tocatl-runs/*/ | head -3                       # newest run folders
ros2 bag record /robot_state -o $RUN/bag                 # record telemetry; Ctrl-C to stop
```
`$RUN` only exists in the terminal where you set it. In other terminals, use the newest folder
from `ls -td ~/tocatl-runs/*/`.

## H. Motor bench (separate motor)
This tests a motor that is **not** one of the robot's 18. It needs no ROS and no robot
controller, and refuses to run while one is up. Details are in `docs/motor-bench.md`.
```bash
cd ~/tocatl && .venv/bin/python scripts/motor_bench.py               # on the Pi
ssh -N -L 8766:127.0.0.1:8766 tocatl@tocatl.local                    # on the Mac, then open http://127.0.0.1:8766
```
In the page:
1. Choose the bus.
2. Click **Open bus**, then **Scan**.
3. Use the motor's ID.
4. Use rotation control to bring the end effector onto the scale.
5. Run **Direction check**.
6. Run **Start torque sweep** and type in the scale readings.

## I. Update the Pi
Pull with the robot idle and the tree clean:
```bash
cd ~/tocatl && git status --short        # must be empty
git pull --ff-only
# Rebuild only if a pull changes package.xml, setup.py, msgs, or adds new files:
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

## Offline tools (no CAN)
```bash
ros2 run tocatl_kinematics tocatl_ik --help
ros2 run tocatl_kinematics tocatl_gait plan --out /tmp/gait.json
ros2 run tocatl_kinematics leg_model
```
