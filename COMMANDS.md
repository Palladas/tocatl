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
| Check which motors are plugged in, and on which bus | [B2. Which motors are on which bus](#b2-which-motors-are-on-which-bus) |
| Check the CAN adapter names after a reboot | [B3. Adapter names](#b3-adapter-names) |
| Just watch the motors (no motion) | [C1. Level 1 telemetry](#c1-level-1--read-only-telemetry) |
| Home legs | [C2. Level 2 homing](#c2-level-2--homing) + [D. Homing](#d-homing) |
| Move legs in the air (nudge, pose, circle, wave) | [C3. Level 3](#c3-level-3--feet-in-the-air) + [E. Motions](#e-motions) |
| Stand up | [C4. Level 4](#c4-level-4--standing) + [E5. Stand](#e5-stand-level-4) |
| Walk | [C4. Level 4](#c4-level-4--standing) + [E6. Walk](#e6-walk-level-4) |
| Artistic leg cues from the Xbox pad (feet in the air) | [E8. Perform](#e8-perform-feet-in-the-air-xbox-cues) |
| Arm, disarm, or stop | [F. Arm / disarm / stop](#f-arm--disarm--stop) |
| See status, faults, why something stopped | [G. Watch and debug](#g-watch-and-debug) |
| Record telemetry | [G. Watch and debug](#g-watch-and-debug) |
| Test a separate motor (CAN ID, rotation, max torque) | [H. Motor bench](#h-motor-bench-separate-motor) |
| Get the latest code onto the Pi | [I. Update the Pi](#i-update-the-pi) |
| Pair or check the Xbox pad, CAN naming service, other one-time Pi setup | [J. One-time Pi setup](#j-one-time-pi-setup) |
| Run the tests | [K. Tests](#k-tests) |

**The three terminals:**

| Terminal | Runs | Behaviour |
|---|---|---|
| **T1** | the controller | stays running |
| **T2** | the player | stays running during a motion |
| **T3** | services: home, arm, disarm, status | one-shot commands |

Start every terminal with [A](#a-terminal-setup).

---

**Shortcuts** (in the repo, so they arrive with every pull; run from any directory):

| Script | Does | Opens CAN | Moves motors | Section |
|---|---|---|---|---|
| `~/tocatl/scripts/cmds/can-scan.sh` | which motors answer on which bus | get-ID only | no | [B2](#b2-which-motors-are-on-which-bus) |
| `~/tocatl/scripts/cmds/level1.sh` | Level 1 read-only telemetry, all buses | yes | no | [C1](#c1-level-1--read-only-telemetry) |
| `~/tocatl/scripts/cmds/state.sh` | one snapshot: safety, reason, motors, CAN counters | no | no | [G](#g-watch-and-debug) |
| `~/tocatl/scripts/cmds/level3-perform.sh` | Level 3 controller, all six legs feet in the air (homing + motion) | yes | when armed | [C3](#c3-level-3--feet-in-the-air) |
| `~/tocatl/scripts/cmds/perform.sh` | feet-in-air cues (`tocatl_perform`); `--dry-run` plans only | no | via the controller | [E8](#e8-perform-feet-in-the-air-xbox-cues) |
| `~/tocatl/scripts/cmds/level4-walk.sh` | Level 4 controller for standing/walking (homing + motion, `fault_response:=soften`) | yes | when armed | [C4](#c4-level-4--standing) |
| `~/tocatl/scripts/cmds/teleop.sh` | Xbox controller (joy_node + tocatl_teleop) | no | via walker/performer | [E7](#e7-walk-with-an-xbox-controller) |

- The controller scripts write a run folder, `~/tocatl-runs/cli-<UTC time>/`, containing
  `controller.log`.
- Extra parameters go after the script, e.g. `level1.sh -p can_transport:=v2`, or
  `perform.sh --dry-run`.

## A. Terminal setup
```bash
ssh tocatl@tocatl.local
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## B. CAN links
Run this after every reboot, or after a bus freeze, with the controller stopped:
```bash
for b in can0 can1 can2; do sudo ip link set $b down; sudo ip link set $b type can bitrate 1000000; sudo ip link set $b up; done
# fewer buses connected: list only those, e.g. "for b in can2"
ip -br link | grep can      # want every bus UP
ip -details link show can0 | grep -oE "can state [A-Z-]+|bitrate [0-9]+"   # want ERROR-ACTIVE, 1000000; repeat for can1, can2
ps -eo pid,args | grep -E "[m]otor_controller|[t]ocatl_play|[t]ocatl_walk|[t]ocatl_perform|[m]otor_bench"   # must print nothing before starting
```

### B2. Which motors are on which bus
Read-only: it sends only get-ID (comm type 0) to IDs 1–127. Run it with the buses UP (B) and
no controller or bench running:
```bash
~/tocatl/scripts/cmds/can-scan.sh            # all three buses
~/tocatl/scripts/cmds/can-scan.sh can2       # one bus
cd ~/tocatl && .venv/bin/python scripts/can_scan.py can0 can1   # same thing, without the running-process check
```

| Output | Meaning |
|---|---|
| `answered [...]` | motors that replied on this bus |
| `missing [...]` | mapped to this bus but silent: unpowered, unplugged or wrong ID |
| `ON THE WRONG BUS` | the cable is in the wrong chain, or the adapter names swapped (B3) |
| `send failed … nothing acknowledges frames` | no powered motor on that bus: check power, CAN-H/L, termination (about 60 Ω across H/L, power off) |

The map: can0 = legs 1/2 (11–13, 21–23), can1 = legs 3/4 (31–33, 41–43), can2 = legs 5/6
(51–53, 61–63).

A controller does the same check at startup (`verify_can_wiring`). It prints
`CAN wiring check: can0: [...]` and refuses to start if a motor answers on the wrong bus.

### B3. Adapter names
A boot service pins each adapter's name to its USB serial (setup in J1):

| Name | Legs | Adapter serial |
|---|---|---|
| can0 | 1, 2 | `003A00665743500D20353341` |
| can1 | 3, 4 | `0035005C4633500E20303035` |
| can2 | 5, 6 | `0036006D4633500E20303035` |

Check the names after a reboot:
```bash
for b in can0 can1 can2; do echo "$b $(cat /sys/class/net/$b/device/../serial)"; done   # must match the table
journalctl -b -t tocatl-can-names --no-pager          # what the service renamed this boot
journalctl -b -u systemd-udevd --no-pager | grep -i rename   # any rename errors
```
To fix names now, without rebooting (links must be DOWN):
```bash
for b in can0 can1 can2; do sudo ip link set $b down; done
sudo /usr/local/sbin/tocatl-can-names
```
To map buses for a single run instead, add `-p can1_channel:=can2 -p can2_channel:=can1` (any
pair) to the controller.

## C. Start a controller (T1)
Run only one controller at a time. Ctrl-C stops it; the node disables the motors and restores
their limits on the way out. Set these two variables first:
```bash
C=src/tocatl_bringup/config
RUN=~/tocatl-runs/cli-$(date -u +%Y%m%dT%H%M%SZ); mkdir -p $RUN
CTL=".venv/bin/python install/tocatl_lowlevel/lib/tocatl_lowlevel/motor_controller"
```
The controller must run under `.venv` Python, which has `python-can` and `CyberGearDriver`.
`ros2 run tocatl_lowlevel motor_controller` uses the system Python and fails with
`No module named 'can'`.

**Buses:** can0 carries legs 1/2, can1 carries legs 3/4, and can2 carries legs 5/6.

**Not every bus connected:** add `-p "can_buses:=[...]"` and limit the motor IDs and legs to
those buses. For example, Level 1 with only can2:
`-p "can_buses:=[can2]" -p "telemetry_motor_ids:=[51,52,53,61,62,63]"`.

**Adapter got a different name:** point a bus at another interface, e.g.
`-p can2_channel:=can0`. Linux names adapters in the order it finds them, so check
`ip -br link` after every reboot.

**Optional controller parameters** (add to any C command, or after a script):

| Parameter | Effect |
|---|---|
| `-p can_transport:=v2` | faster CAN sends, without the driver's 10 ms pause (opt-in; rollout below) |
| `-p control_period_s:=0.01` | pause between command passes (default 0.02); try only with v2 |
| `-p fault_response:=soften` | on a fault, ramp kp to zero over `fault_soften_s` (3 s), instead of dropping at once |
| `-p verify_can_wiring:=false` | skip the startup get-ID wiring check |
| `-p canN_channel:=canM` | point logical bus N at interface M |

**Transport v2 rollout.** Change one step at a time. Stop at the first `No buffer space
available` or rising `dropped`, reset the bus (B), go back to v1, and note the step.
1. Run the Pi tests, including the timing test (K).
2. Level 1, one leg, with `-p can_transport:=v2`. While it runs:
   ```bash
   ros2 topic hz /robot_state
   ip -s link show can0 | sed -n 4p; sleep 5; ip -s link show can0 | sed -n 4p   # TX rising, dropped not rising
   ```
3. Level 1, all legs. Same checks as step 2.
4. Level 3, one leg: nudge (E1) with v2.
5. Level 3, all legs: circle and wave (E3, E4). Then try `-p control_period_s:=0.01`.

Details are in `docs/can-transport.md`.

### C1. Level 1 · read-only telemetry
```bash
$CTL --ros-args --params-file $C/hardware.yaml 2>&1 | tee $RUN/controller.log
```

### C2. Level 2 · homing
Homing only, no motion. Edit the legs list as needed:
```bash
$CTL --ros-args --params-file $C/homing-hip-knee.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false \
  -p homing.output_path:=$RUN/hip-knee-homing.json 2>&1 | tee $RUN/controller.log
```

### C3. Level 3 · feet in the air
Hip pitch and knee only; the yaws stay unpowered. Limits: kp ≤ 30, kd ≤ 2, 1.5 Nm cap.

All six legs:
```bash
IDS="[12,13,22,23,32,33,42,43,52,53,62,63]"
$CTL --ros-args --params-file $C/homing-hip-knee.yaml --params-file $C/motion-feet-air.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false -p homing.output_path:=$RUN/hip-knee-homing.json \
  -p allow_hardware_motion:=true -p "actuation_motor_ids:=$IDS" -p "telemetry_motor_ids:=$IDS" 2>&1 | tee $RUN/controller.log
```
For fewer legs, change `homing.legs` and `IDS` together. For example, legs 1 and 2 are
`[1,2]` and `IDS="[12,13,22,23]"`.

### C4. Level 4 · standing
All 18 motors, including the yaws. Limits: kp ≤ 100, kd ≤ 5, 12 Nm cap.
```bash
IDS="[11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63]"
$CTL --ros-args --params-file $C/homing-hip-knee.yaml --params-file $C/motion-standing.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false -p homing.output_path:=$RUN/hip-knee-homing.json \
  -p allow_hardware_motion:=true -p "actuation_motor_ids:=$IDS" -p "telemetry_motor_ids:=$IDS" 2>&1 | tee $RUN/controller.log
```

## D. Homing (T3)
Use this with a C2, C3 or C4 controller running, robot supported, legs clear.

**Home in tripods:** legs 1, 3, 5 together (one leg per bus), then 2, 4, 6.
```bash
ros2 service call /motors/home std_srvs/srv/Trigger               # both tripods: 1/3/5, then 2/4/6
ros2 service call /motors/tripod_135/home std_srvs/srv/Trigger    # one tripod only
ros2 service call /motors/tripod_246/home std_srvs/srv/Trigger
```
- Each call returns straight away and homing runs in the background. Wait for
  `Legs 1, 3, 5 pitch/knee READY` (or a fault) in T1's log before the next call. A call while
  homing is running is refused with `Leg homing already active`.
- A fault in one leg cancels the other legs of its tripod. Legs that already finished stay READY.
- **Robot test status:** the tripod batching passes the timing model, but has **not yet run on
  the robot**. For the first run, home one tripod (`tripod_135`), and watch T1 for
  `fault-status age` refusals before running the second.
- Only legs in the controller's `homing.legs` are homed. A tripod service homes the part of its
  tripod that is selected.

**Hip homing torque** (the gearbox multiplies it 2.7× at the joint). The defaults are 1.5 Nm
drive limit and 1.0 Nm contact threshold, motor side. To lower them for one run, add to the
controller command:
```bash
-p homing.hip_torque_limit_nm:=0.8 -p homing.hip_contact_effort_nm:=0.5
```
- Both must satisfy contact < limit ≤ 1.5.
- Choose from data: after any homing run, read each hip's free-travel effort. The contact
  threshold should sit well above it.
  ```bash
  R=$(ls -td ~/tocatl-runs/*/ | head -1); grep -h -E '"motor_id"|peak_seek_effort_nm|contact_effort_nm|reference_within_window' $R/hip-knee-homing.motor-*2.json
  ```
- Hip contacts must land within 0.03 rad (turn-aware) of `hip_stop_rad`, or the hip refuses
  with `Loaded contact outside expected stop window`. That means an obstruction, or a stop that
  moved. Check the leg, and update `homing-hip-knee.yaml` only if the new stop is real.
- **Re-learning references:** start the controller with `-p homing.enforce_hip_window:=false`.
  Hips then report a stop outside the window instead of refusing it. Home, update the YAML from
  that run, then restart without the flag.

**One leg at a time** (proven on the robot; the fallback if a tripod run refuses):
```bash
ros2 service call /motors/leg_1/home std_srvs/srv/Trigger    # wait for "Leg 1 pitch/knee READY", then leg_2, ...
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

### E6. Walk (Level 4)
Needs a C4 controller (all 18 motors), every leg homed, and the yaws declared **with every leg
pointing straight out (radial)**. Start resting on the feet at about 0.25 m from each yaw axis,
or fully supported with the feet clear for the first runs. After the pull that added
`tocatl_walk`, rebuild once: `colcon build --symlink-install`.

Scripted walk: forward for 6 s, then turn for 6 s, then settle and sit. Dry run first:
```bash
ros2 run tocatl_kinematics tocatl_walk --gait ripple --script "6:0.02,0,0;6:0,0,0.08" --dry-run
```
Drive it live from `/cmd_vel` instead (teleop, or by hand in T3):
```bash
ros2 run tocatl_kinematics tocatl_walk --gait ripple
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.02}}"    # T3; Ctrl-C = stop walking
ros2 topic pub --once /walk/stop std_msgs/msg/Empty                              # settle and sit
ros2 topic pub --once /walk/gait std_msgs/msg/String "{data: tripod}"            # switches once settled
ros2 topic pub --once /walk/height std_msgs/msg/Int8 "{data: 1}"                 # body up 1 cm (-1 down), once settled
```
- Arm in T3 ([F](#f-arm--disarm--stop)). The body rises, then it walks.
- **Recommended for walking:** add `-p fault_response:=soften` to the C4 controller command. A
  fault then holds every healthy motor's last target while its stiffness (kp) ramps to zero over
  `fault_soften_s` (3 s), so the body sinks under damping instead of dropping, and then everything
  stops. A motor that reports its own fault is stopped at once. A DISARM during softening is still
  the immediate hard stop. Not yet tested on the robot: try it first with the robot supported.
- **Stopping:** Ctrl-C once settles the feet and lowers the body, then disarms. Ctrl-C twice
  disarms at once, which **drops** the robot.
- Gaits: `ripple` (most stable), `tetrapod`, `tripod` (fastest). Speeds are clamped to
  `--max-vx-m-s`, `--max-vy-m-s` (0.05) and `--max-wz-rad-s` (0.2), and to the stride.

### E7. Walk with an Xbox controller
Pairing, checking the pad and checking the button layout are in [J2](#j2-xbox-pad). Press the
Xbox button to turn the pad on: it reconnects by itself.

Run it with a C4 controller (T1), using `-p fault_response:=soften`. Use one terminal for each of
these three:
```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0     # T4: reads the pad
ros2 run tocatl_kinematics tocatl_teleop                      # T5: pad -> /cmd_vel, stop, gait, arm
ros2 run tocatl_kinematics tocatl_walk --gait ripple          # T2: the walker
```
`~/tocatl/scripts/cmds/teleop.sh` starts T4 and T5 together; Ctrl-C stops both. Over USB, add
`-p axis_dpad_y:=7` to teleop (e.g. `teleop.sh -p axis_dpad_y:=7`).

| Pad | Action |
|---|---|
| **LB held** | **deadman**: it only walks while held |
| Left stick | walk in any direction |
| Right stick left/right | turn |
| Right trigger | speed: 30% released, 100% pressed |
| Start held 1 s | ARM: the body rises, then it follows the sticks |
| B | stop: settle the feet, lower the body, disarm |
| X | next gait (ripple, then tetrapod, then tripod), applied once the feet settle |
| D-pad up/down | body height ±1 cm, among the heights the walker checked at startup (two steps each way) |
| Back / Xbox button | **DISARM now**, the hard stop; a standing robot drops |

If the pad stops sending for 0.25 s, the walk command goes to zero and the feet settle.

### E8. Perform: feet in the air, Xbox cues
Artistic leg cues played on demand. **The robot must be supported with every foot clear of the
ground.** Hip pitch and knee only; the yaws stay unpowered (Level 3). Never run it together with
`tocatl_walk`.

1. T1: `scripts/cmds/level3-perform.sh` (C3, all six legs). Home every leg (D).
2. T2: `scripts/cmds/perform.sh --dry-run`. Check that every cue reports `ok`. Then run
   `scripts/cmds/perform.sh`. It plans every cue and holds the measured pose.
3. T4: `scripts/cmds/teleop.sh`.

| Pad | Action |
|---|---|
| Start held 1 s | ARM: the legs unfold to the ready pose (feet at r 0.32 m, z +0.02 m) |
| **LB held** + A | `wave`: circles rippling around the body |
| **LB held** + Y | `bloom`: all feet reach out and up, then return |
| **LB held** + RB | `circle`: all feet circle together |
| **LB held** + left-stick click | `alternate`: the two tripods circle in antiphase |
| **LB held** + right-stick click | `wave_reverse`: the ripple runs the other way |
| B | fold back along the unfold path after the current cue, then disarm |
| Back / Xbox button | **DISARM now**: the legs go limp |

- Cues only fire while LB is held. A cue that arrives while another is playing is ignored.
- `reach` (feet point down) has no button. Run it with
  `ros2 topic pub --once /perform/cue std_msgs/msg/String "{data: reach}"`.
- `--speed` sets the peak raw motor speed (default 1.0 rad/s, cap 5). Start slower, e.g. `--speed 0.5`.
- A disarm or fault goes back to REST. The next arm unfolds again from wherever the legs are.

The same steps without the scripts (C3 controller in T1):
```bash
ros2 run tocatl_kinematics tocatl_perform --dry-run                  # T2: plan and report every cue
ros2 run tocatl_kinematics tocatl_perform --speed 0.5                # T2: run (speed in raw motor rad/s)
ros2 topic pub --once /perform/cue std_msgs/msg/String "{data: wave}"   # T3: any cue by name
ros2 topic pub --once /walk/stop std_msgs/msg/Empty                  # T3: fold back and disarm
```
Cue names: `wave`, `wave_reverse`, `circle`, `alternate`, `bloom`, `reach`.

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
~/tocatl/scripts/cmds/state.sh                           # one snapshot: safety, reason, motors, CAN counters
for b in can0 can1 can2; do echo $b; ip -s link show $b | sed -n 4p; done   # TX counters: packets, errors, dropped
ip -details link show can0 | grep -oE "can state [A-Z-]+"   # ERROR-ACTIVE good; ERROR-PASSIVE / BUS-OFF = wiring/termination
ps -eo pid,args | grep -E "[m]otor_controller|[t]ocatl_|[j]oy_node|[m]otor_bench"   # what is running
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
source install/setup.bash && ros2 pkg executables tocatl_kinematics   # new commands listed?
```
If `git status` shows local edits on the Pi, save them first, then decide whether to keep them
(commit them from the Mac) or drop them:
```bash
mkdir -p ~/patches && git diff > ~/patches/pi-local-$(date +%Y%m%d).patch   # save every local edit
git checkout -- <file> ...                                                   # drop them (after saving)
git apply ~/patches/pi-local-<date>.patch                                   # bring them back if needed
```
The Pi fetches with a read-only deploy key (`git fetch` works without a passphrase). It pushes
with Pranav's key, so commit and push from the Mac.

## J. One-time Pi setup
Already done on the Pi. Rerun these only to repair or redo the setup.

### J1. CAN adapter names service
Pins `can0`–`can2` to the adapters' USB serials (B3). A plain udev `NAME=` rule fails here: all
three adapters appear at once, and renaming into a name that's still taken fails with
`File exists`. So a service renames them in two passes, and never renames a link that is UP.
Install or update it:
```bash
cd ~/tocatl
sudo install -m 755 scripts/udev/tocatl-can-names /usr/local/sbin/
sudo install -m 644 scripts/udev/tocatl-can-names.service /etc/systemd/system/
sudo install -m 644 scripts/udev/70-tocatl-can.rules /etc/udev/rules.d/
sudo systemctl daemon-reload && sudo systemctl enable tocatl-can-names.service
sudo udevadm control --reload
```
Add or replace an adapter:
```bash
cat /sys/class/net/can0/device/../serial        # read the new adapter's serial (use its current name)
nano scripts/udev/tocatl-can-names              # edit MAP, commit from the Mac, then reinstall as above
```
Simulate a boot (every adapter re-enumerates at once; links must be DOWN, no controller):
```bash
for d in $(for b in can0 can1 can2; do basename $(readlink -f /sys/class/net/$b/device/..); done); do echo 0 | sudo tee /sys/bus/usb/devices/$d/authorized >/dev/null; echo 1 | sudo tee /sys/bus/usb/devices/$d/authorized >/dev/null; done
sleep 5; for b in can0 can1 can2; do echo "$b $(cat /sys/class/net/$b/device/../serial)"; done
```
The old 2025 rule is disabled as `/etc/udev/rules.d/88-canable.rules.disabled-20261106`.

### J2. Xbox pad
Paired 2026-11-07: Xbox Series pad `AC:8E:BD:64:DD:20` (driver `xpadneo`), bonded and trusted.
Another pad, `40:8E:2C:2A:6D:84`, is also paired.

**Once per Pi:**
```bash
sudo apt install ros-jazzy-joy
sudo usermod -aG input tocatl      # then log in again: joy_node (SDL2) can't read /dev/input/event* without it
```

**Check the pad:**
```bash
bluetoothctl info AC:8E:BD:64:DD:20 | grep -E "Name|Paired|Bonded|Trusted|Connected"   # all yes
bluetoothctl devices Paired                       # every paired device
ls -l /dev/input/js*                              # want /dev/input/js0
grep -A4 'Name="Xbox' /proc/bus/input/devices | grep -E "^N:|^H:"
id -nG | grep -w input && test -r /dev/input/event5 && echo readable    # access over SSH
```
`Paired`/`Bonded`/`Trusted: yes` with `Connected: no` means the pad is off. Press the Xbox
button and it reconnects by itself.

**Pair a new pad** (plugging in by USB does not pair it for Bluetooth):
```bash
bluetoothctl
# inside bluetoothctl:
agent NoInputNoOutput
default-agent
scan on
# hold the pad's pair button until the logo flashes fast; wait for "Xbox Wireless Controller" and its MAC
pair <MAC>
trust <MAC>
connect <MAC>
exit
```
- Without the agent, pairing fails with `AuthenticationCanceled`. The pad may still connect, but
  nothing is bonded, so it won't reconnect by itself.
- To start over, run `bluetoothctl remove <MAC>`.

**Check the button layout** (terminal 1: `joy_node`; terminal 2: echo):
```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
ros2 topic echo /joy          # press each button; watch which index changes
```
- **Recorded over Bluetooth:** LB 4, Start 7, B 1, X 2, Back 6, D-pad up/down axis 8, left
  stick axes 1 (forward +) and 0 (left +), right stick axis 3 (left +), RT axis 5 (1 released,
  −1 pressed).
- **Not yet recorded:** A 0, Y 3, RB 5 and the stick clicks 9/10.
- **Over USB** (`xpad`), the D-pad up/down is axis 7.
- To change an index: `ros2 run tocatl_kinematics tocatl_teleop --ros-args -p button_lb:=4 -p axis_dpad_y:=8`.

## K. Tests
On the Mac (no ROS needed):
```bash
PYTHONPATH=src/tocatl_lowlevel python3.12 -m pytest -q -p no:cacheprovider src/tocatl_lowlevel/test
PYTHONPATH=src/tocatl_kinematics python3.12 -m pytest -q -p no:cacheprovider src/tocatl_kinematics/test
PYTHONPATH=scripts python3.12 -m pytest -q -p no:cacheprovider test
```
On the Pi (also runs the ROS-only tests the Mac skips):
```bash
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
for d in src/tocatl_lowlevel/test src/tocatl_kinematics/test test; do .venv/bin/python -m pytest -q -p no:cacheprovider $d; done
TOCATL_TIMING_TESTS=1 .venv/bin/python -m pytest -q src/tocatl_lowlevel/test/test_homing_timing.py   # homing timing model
```

## Offline tools (no CAN)
```bash
ros2 run tocatl_kinematics tocatl_ik --help
ros2 run tocatl_kinematics tocatl_gait plan --out /tmp/gait.json
ros2 run tocatl_kinematics leg_model
```
