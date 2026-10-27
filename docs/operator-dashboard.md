# Operator dashboard

A local web page for the two operator levels that exist today:

- **Level 1 · Read-only telemetry** starts `motor_controller` with `hardware.yaml`.
- **Level 2 · Hip/knee homing** starts it with `homing-hip-knee.yaml -p allow_homing:=true`, then
  homes legs through `/motors/leg_N/home` or `/motors/home`.

Normal motor-space actuation (`allow_hardware_motion`) is not offered. Starting the server or opening
the page sends no CAN traffic. Only Start controller, Reset CAN and the Home buttons act.

## Start on the Pi

```bash
cd ~/tocatl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python scripts/operator_dashboard.py
```

On the laptop, keep a tunnel open and browse to <http://127.0.0.1:8765>:

```bash
ssh -N -L 8765:127.0.0.1:8765 tocatl@tocatl.local
```

The server binds to loopback only, checks Host/Origin, and requires a per-start control token for
every action. It is a single shared operator session.

`--preview-only` (or starting without ROS sourced) serves the page without the ability to start a
controller or change CAN links. This is useful on a laptop.

## Workflow

1. **CAN links** shows can0/can1 from `ip -details -statistics` (state, bitrate, TX queue, counters).
   - **Reset** brings a link down/up at 1 Mbit/s with `sudo -n ip link set`. It changes nothing else.
   - Reset is refused while a controller or another bus-owning process runs.
   - After a TX freeze (ENOBUFS), stop the controller, reset can1, and start a fresh controller.
2. **Start controller**: choose a level and, for homing, the legs and normal or commissioning mode.
   - The page shows the exact command before it runs.
   - Confirm all five physical checklist items. Starting is refused if CAN links are not UP /
     ERROR-ACTIVE at 1 Mbit/s or if another process may own the bus.
3. **Homing** (Level 2 only): confirm the homing checklist, then press **Home leg N** or **Home all
   selected legs in order**.
   - Legs 2–6 also require acknowledging that their references are unloaded and operator-declared.
   - **Cancel homing** calls `/motors/enable false`.
4. **Stop controller** cancels/disarms first, then sends SIGINT. The node then disables motors and
   restores torque limits itself. SIGTERM and SIGKILL follow only if it does not exit.

Each controller start creates `~/tocatl-runs/dashboard-<profile>-<UTC stamp>/` holding
`controller.log` (full output) and, for homing, `hip-knee-homing.json`.

- Homing results never write to the tracked `config/` file.
- The page lists each homed motor's reference delta and flags out-of-window results. Normal mode
  accepts those results anyway, so check the leg physically.

## Limits

- Software stops depend on ROS and CAN communication. The physical power disconnect is the real stop.
- Closing the page does not stop the controller.
- Stopping the dashboard (Ctrl-C) stops its controller. If the dashboard is killed outright, the
  controller keeps running. The next start then refuses until that process is gone.
- Leg readiness is session-only, as in the controller.
- Not yet run against hardware. Offline tests: `python3 -m pytest test/test_operator_dashboard.py`.

## Homing all legs together

**Home all selected legs** homes opposite pairs 1+4, then 2+5, then 3+6: four motors at
once. Three legs at once (tripods 1-3-5 / 2-4-6) were tried on 2026-10-27 and removed.
Every CAN message costs the driver 10 ms, and one lock serializes both buses, so six motors'
fault-status feedback ages past the 0.25 s freshness limit. That was the cause of the
robot's 22:08 and 22:12 failures, and the timing model in
`src/tocatl_lowlevel/test/test_homing_timing.py` (run with `TOCATL_TIMING_TESTS=1`)
reproduces it. Pairs fit, but close to the limit.

## Level 3 · Feet-in-air motion

The Level 3 profile loads `homing-hip-knee.yaml` and then `motion-feet-air.yaml` in one
controller, so the same session can home and move. The legs you select become
`homing.legs`, the polled motors and the commandable motors. Only their hip pitch and knee
are included. Yaw stays unpowered.

The limits were authorized by Pranav on 2026-10-26 for this profile only:
- kp ≤ 30 and kd ≤ 2.
- Every drive is capped at **1.5 Nm**. The cap is written and read back before the first
  enable, and restored only after the drive confirms it's disabled.
- Feedforward ≤ 0.5 Nm, speed ≤ 0.21 rad/s, and each target within 0.1 rad of the measured
  position.
- Targets must stay inside a raw envelope measured from **this session's** homed position.
  The hip may go from −0.02 to +1.6 rad from its home, and the knee from −1.7 to +2.0.

**Workflow**
1. Pick legs, tick the checklist and the Level 3 confirmation, then **Start controller**.
2. Home those legs. Each must be homed and READY.
3. In **Feet-in-air motion**, choose legs and a motion:
   - **Nudge:** a joint change and back, the safest first test.
   - **Pose:** the foot to (r, z) in the leg plane and back.
   - **Circle:** loops around (r, z).
4. **Preview plan** plans from the live homes and reports duration, motor offsets and peak
   speeds. It sends nothing.
5. Tick the path checklist, then **Run motion**. The player streams hold commands at the
   measured pose.
6. Press **Arm motors**. The motion plays, holds the start pose for 2 s, and the player
   disarms.

**Commands always name every leg in the session.** The controller rejects any
`MotorCommand` that doesn't name exactly its `actuation_motor_ids`, which are all the legs
the Level 3 session was started with. So the player commands all of them: the legs you
tick move, and the rest hold their measured pose. Every session leg must therefore be
homed and READY before a motion runs.

On 2026-10-27 at 22:16, a leg-2-only motion in a six-leg session had all 2397 commands
rejected for this reason and never armed. Fixed in the player the same day.

**A motor that can't keep up slows the motion; it doesn't fault it.**
- The player keeps every target within 0.08 rad of the motor's measured position, inside
  the controller's 0.1 rad step check, and advances the plan only while every moving motor
  keeps up.
- If nothing advances for 3 s, the player stops, names the lagging motor, and disarms. That
  is a disarm, not a FAULT, so homing is kept.
- On 2026-10-27 at 22:26, a six-leg pose instead faulted the controller. Hip 42 fell more
  than 0.1 rad behind, holding the leg against gravity at the 1.5 Nm cap. Every command was
  then rejected, and the watchdog tripped after 1 s.
- The player reports progress and the worst lag every 2 s.

**Prefer two legs at a time.**
- Two-leg sessions work.
- A six-leg nudge at 22:19 completed, but 9 of its commands were rejected. Legs 5 and 6
  lagged their targets by more than the 0.1 rad step limit, because twelve motors make the
  bus loop slow.
- Start the Level 3 session with only the legs you plan to move.

**Stop motion** disarms and interrupts the player at any point. The player also disarms on
its own if the controller faults or leaves ARMED, or if `robot_state` goes stale. Disarming,
not a watchdog timeout, keeps the legs' homing readiness. A controller FAULT clears every
leg's readiness, so a rehome is needed after one.

Not yet run on the robot. Start with one leg, a nudge, and 0.05–0.1 rad/s.

## Level 4 · Standing

The Level 4 profile loads `homing-hip-knee.yaml` and then `motion-standing.yaml`. Pranav
authorized these limits on 2026-10-27:
- Every drive is capped at **8 Nm**, twice the CyberGear's 4 Nm continuous rating. The cap is
  written and read back before enable, and restored after a confirmed disable.
- kp ≤ 100, kd ≤ 5, speed ≤ 5 rad/s, each target within 0.5 rad of the measured position.
- Feedforward torque is unchanged at 0.5 Nm. The 45 °C readiness guard still stops motion.

The session commands the selected legs' **yaw** motors too, all six including 51, which is
back in service since 2026-10-27. 51 has passed read-only polling but hasn't been commanded
yet; if can1 stalls, stop, reset can1 and see `docs/motor51-diagnosis-2026-10-26.md`.

**Workflow**
1. Rest the robot on flat, clear ground. Tick the checklist, the motion confirmation and the
   standing confirmation, then **Start controller**.
2. Home every selected leg, in normal mode.
3. Pose the legs, then press **Declare yaw references**. This records each yaw's current
   position as its reference, and only if the motor is disabled, fault-free and fresh. Arming
   needs it. A rehome, lost or faulted feedback, or a controller fault clears it, and it must
   then be declared again.
   Each leg card shows **yaw held** or **yaw not declared**. Once every session leg is
   homed, the button is highlighted until all yaws are declared, and **Arm** is refused until
   then. Homing a leg clears its yaw reference, so declare after the last homing.
4. In **Leg motion**, choose **Stand** and select all session legs. Enter:
   - **r:** foot distance out from the yaw axis.
   - **Resting height:** the mounting plane's height above the ground while the body rests.
   - **Standing height.**
   - **Clearance.**
   - **The robot's mass,** which you get by weighing it.

   Use kp around 80 and kd around 4.
5. **Preview.** The plan is refused if any joint's static load torque (mass shared equally by
   the moving legs) exceeds 90% of the 8 Nm cap. It warns above the 4 Nm rating.
   - Example: 12 kg on six legs at r 0.25 m needs about 4.9 Nm per hip and 1 Nm per knee.
   - Moving the feet in (smaller r) lowers hip torque and raises knee torque.
6. **Run motion**, then **Arm**. The sequence: place the feet, touch down, rise, hold, lower,
   lift off, return.

**Not modelled:**
- dynamic loads
- foot slip
- unequal load sharing
- the controller's command loop, at about 7 Hz with all motors

Begin with a low standing height and a short hold. Stopping the motion (stall, Stop or a
fault) disables the motors and lowers the body under gravity, so keep the body over soft
blocks.

**Homes stay put during homing (fixed 2026-10-27).** While a pair homes, other motors are
polled in the background. In Level 4, with 14–16 motors sharing that slot, homed legs' feedback
aged past 1.5 s, and readiness cleared them as `feedback lost`. As a result, homing any leg
dropped the earlier ones. The background rotation now covers only the motors readiness checks
(homed hip/knee and declared yaws), so at most 8 motors share it during a batch, the same as
Levels 2 and 3.
