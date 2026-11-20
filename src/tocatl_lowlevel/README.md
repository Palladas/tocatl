# Low-level robot control

Six modules own the controller's runtime responsibilities:

| Module | Responsibility |
| --- | --- |
| `motor_controller.py` | ROS node, services, subscriptions, and robot state publishing |
| `motor_commands.py` | Motor target validation, arming, watchdog, hardware execution, and mock backend |
| `hardware.py` | CAN transport, address overrides, telemetry polling and caching |
| `motor_map.py` | Canonical motor IDs, buses, and joint names |
| `homing.py` | Bounded torque/contact homing, cancellation, and result persistence |
| `calibration.py` | Calibration documents and offline validation CLI |

The public executables remain `motor_controller` and `validate_calibration`.
Mock is still the default backend. Hardware motion requires explicit configuration,
fresh telemetry, a complete validated command, and operator arming. Homing remains
exclusive with normal motor control and never writes device zeros.

Homing methods: `torque`, `legacy_impedance`, `active_endstop`, and
`active_endstop_hip_hold`. Their existing YAML profiles remain in
`tocatl_bringup/config`.

The October 2026 cleanup removed the standalone hold/sweep experiments and their
YAML profiles, along with unused prototype command limiter and safety state
machine code. The active validation and arming checks live in `motor_commands.py`.
`basic_motors` was merged into `motor_commands`, `robot_state` into
`motor_controller`, and `impedance_homing` into `homing`. The read-only CyberGear
adapter now lives in `tocatl_diagnostics`, its only consumer. Imports should use
these consolidated modules; compatibility shim files are intentionally omitted.
Historical bench instructions in `docs/` describe retired experiments.

Run offline regression checks from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/tocatl_lowlevel/test
```

## Hip/knee homing for all six legs

`homing-hip-knee.yaml` selects `hip_knee` and scopes telemetry/actuation to all
12 pitch/knee motors. All yaw motors are excluded, including disconnected 51.
`homing.legs` chooses which legs the services can home. Each leg uses the same
worker: seek hip with a damping-only following knee, then seek knee with hip
holding. Normal homing needs one loaded contact per joint, stable within 0.003
rad for 0.6 s, with a 0.2 rad/s reference and 60-second motion budget **per leg**.

The controller provides `/motors/leg_1/home` through `/motors/leg_6/home` for
individual legs, and `/motors/home` for opposite-leg groups: (1,4), then (2,5),
then (3,6). Both legs in each group home concurrently; each leg still homes hip
before knee. A partial selection pairs only selected opposites and otherwise
runs a leg alone. Only one batch may run at a time. Any active-leg failure
cancels its partner and prevents later groups from starting, including while a
failed drive is retrying stop. Earlier completed groups remain ready. Cancel
stops both active legs. Both legs retain telemetry priority through confirmed
stop; shared result writes are serialized. Homing and normal actuation remain exclusive.

Each leg's `homing.leg_N.hip_stop_rad`, `knee_stop_rad`, `hip_direction`, and
`knee_direction` are independent. Directions accept +1 or -1. The all-leg YAML
retains leg 1's tested references. Legs 2–6 now contain operator-declared unloaded
stop references captured on 2026-10-25, with manually checked home directions
(-1 for all ten motors). These hand-set references still need powered loaded-stop
validation; they are not a completed homing or readiness record. Missing
references/directions or missing telemetry drivers reject the request before
motor writes. Archived session software-zero offsets were not substituted for
loaded stop references. `/motors/home` validates the entire configured list
before starting; individual configured legs can be homed independently.

`homing-leg1-pair.yaml` remains available. Its existing `leg1_pair` method is an
alias for the same implementation with leg 1 selected, preserving existing test
commands. The all-leg server enables homing services by default; motion still
requires an explicit service request.

For commissioning only, `homing.verify_repeatability:=true` selects three
independent contacts per joint, 5-degree backoffs, <=0.01 rad contact spread, a
0.05 rad/s reference rate and a default 600-second overall budget. The optional
`homing.timeout_s` parameter overrides either mode's budget.

Normal homing has no start-derived position corridor and does not reject a
contact for differing from the old/manual stop reference. It seeks in the
configured direction and records a stable loaded contact as home. Prior-reference
position, delta and window agreement are recorded as comparison metadata. Only
the encoder wire-format range [-12.5,12.5] remains as a position validity bound;
normal actuation position limits are unchanged. The knee phase holds the freshly
observed relaxed hip pose, without requiring spring-back to stay within 0.03 rad
of the loaded hip contact. Commissioning mode retains strict prior-reference
comparison and repeated-contact checks.

A torque/stability-based contact cannot independently distinguish a physical
endstop from an obstruction; normal homing assumes a clear path to the intended
stop. Moving feedback under load does not count as a stable contact.

Active-joint native MIT gains remain kp=1.75/kd=2. The supporting hip during knee
homing retains kp=150/kd=3 and a 0.01 rad drift guard. Both drives' volatile runtime
torque limits are set to 1.5 Nm with readback before enable, and their original
limits are restored only after disabled feedback is confirmed. No device zero
is written. Stable contact requires 1.0 Nm inward effort. After the final contact,
the worker removes spring error over 1.5 seconds with damping retained, then
fades both motors' impedance to zero over another 1.5 seconds before disable.
Contact positions are saved from the loaded contact, before unloading. Faults
and cancellation still request immediate stop. The 3.5 Nm measured-effort abort, 0.5 rad/s encoder speed guard,
0.25 s feedback/fault freshness and 45 C thermal guard remain unchanged. Startup
has no fixed sleep: it verifies the torque cap and waits only for fresh state,
fault and enable acknowledgements. Slow reference loading and commissioning
repeats caused the earlier multi-minute procedure, not those checks.

The controller terminal logs homing start and completion/failure. During homing,
`RobotState.status_reason` reports the phase and contact count. Successful
completion remains visible as `Leg N pitch/knee READY; yaw excluded; motors
disabled`. Each completed leg
reports `homed=true`, `ready=true`, `scope=pitch_knee` in `RobotState.legs`. The service response only acknowledges starting the run.
Readiness is session-only and is cleared by restart, rehoming, cancellation,
faults or lost feedback. Saved results never restore readiness by themselves.
Rehoming or a local motor/homing fault invalidates that pair without discarding
other completed legs. Global transport or normal-actuation faults clear all
readiness. The robot-wide safety state remains DISARMED after homing, and normal
actuation remains separately opt-in.

`actuation_motor_ids` selects the exact command batch IDs. `telemetry_motor_ids`
selects the motors queried on configured buses; unpolled motors remain unknown.
The leg 1 profile uses `[12,13]` for both; the all-leg profile uses all 12
pitch/knee IDs. The general hardware profile excludes
51 after read-only probes associated traffic to that ID with CAN transmit stalls;
the operator subsequently disconnected leg 5 yaw. No live motion was initiated
for the normal/commissioning split or the faster reference rate.

Historical powered test results and CAN findings:

- [Commissioning record](../../docs/leg1-homing-2026-10-25.md)
- [CAN investigation](../../docs/can-debug-2026-10-25.md)


Start the lowlevel hardware server (with the hardware dependencies in the active venv):

```bash
cd ~/tocatl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate
ros2 launch tocatl_bringup lowlevel.launch.py
```

The server exposes homing by default and waits for a service request before moving.
It selects the twelve hip/knee motors and uses the lighter contact/gradual release
profile. `allow_homing:=false` disables the homing services. Normal actuation
remains disabled. Stop any previous controller before launching this one.

From another sourced terminal, request one leg or the configured list:

```bash
ros2 service call /motors/leg_2/home std_srvs/srv/Trigger '{}'
ros2 service call /motors/home std_srvs/srv/Trigger '{}'
```

The batch command is an alternative to an individual request, not a simultaneous
second operation. Monitor `/robot_state --field legs`; cancel with
`/motors/enable` (`SetBool`, `data: false`). No new all-leg powered test was
performed for this software extension.
