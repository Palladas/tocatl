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

## Homing all legs as tripods

**Home all as tripods** homes legs 1, 3 and 5 at the same time, then 2, 4 and 6 (service
`motors/home_tripods`). **Home all selected legs** keeps the opposite pairs 1+4, 2+5, 3+6.

Six motors is the most that fits at once. Every CAN message costs the driver 10 ms, and one
lock serializes both buses. Polling 12 motors takes about 240 ms, which leaves no margin
under the 0.25 s feedback freshness guard. Tripods have not been powered-tested.

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

**Stop motion** disarms and interrupts the player at any point. The player also disarms on
its own if the controller faults or leaves ARMED, or if `robot_state` goes stale. Disarming,
not a watchdog timeout, keeps the legs' homing readiness. A controller FAULT clears every
leg's readiness, so a rehome is needed after one.

Not yet run on the robot. Start with one leg, a nudge, and 0.05–0.1 rad/s.
