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
