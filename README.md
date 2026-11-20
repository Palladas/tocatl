# TOCATL

ROS 2 Jazzy workspace for the TOCATL hexapod. This is a small starting point
extracted from `~/romela-tocatl`; the source project is left intact.

| Package | Responsibility |
| --- | --- |
| `tocatl_lowlevel` | Basic motor ROS interface, mock backend, canonical motor map, reusable calibration and command validation |
| `tocatl_interfaces` | Shared robot, motor, and foot state ROS messages |
| `tocatl_bringup` | Launch files and runtime parameter YAML |
| `tocatl_diagnostics` | Inventory and optional read-only CyberGear CAN telemetry |
| `tocatl_kinematics` | Offline leg/body IK, tripod gait planning and measurement fitting ([docs/kinematics.md](docs/kinematics.md)); legacy geometry reference |

## Build and run

Use system Python with ROS 2 Jazzy installed. No virtual environment or CAN
packages are needed for the default launch.

```bash
cd ~/tocatl
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch tocatl_bringup tocatl.launch.py
```

In another terminal, source both setup files, then inspect feedback:

```bash
ros2 topic echo /robot_state
ros2 topic echo /joint_states
```

Enable the mock motors and publish commands continuously (the watchdog disables
them after one second without an accepted command):

```bash
ros2 service call /motors/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState \
  '{name: [leg_1_hip_yaw], position: [0.1]}'
```

Disable with `ros2 service call /motors/enable std_srvs/srv/SetBool '{data: false}'`.
Joint names follow `leg_<1..6>_<hip_yaw|hip_pitch|knee>`. Commands may address a
subset of joints; positions are radians. Duplicate/unknown names, nonfinite or
out-of-range positions, and velocity/effort commands are rejected atomically.
The mock starts disabled, tracks positions instantly, and uses +/- pi bounds
solely for software testing. It is not a physical simulation or calibrated model.

See the [low-level package guide](src/tocatl_lowlevel/README.md) for the consolidated module layout and supported homing modes. Historical hold/sweep experiments have been removed.

Configuration: `src/tocatl_bringup/config/motors.yaml`.
The default is `backend: mock`; `backend: hardware` enables read-only CAN telemetry.

## Tests and tools

```bash
colcon test
colcon test-result --verbose
ros2 run tocatl_diagnostics motor_map
ros2 run tocatl_kinematics geometry_reference
ros2 run tocatl_diagnostics motor_telemetry  # offline schema/inventory by default
ros2 run tocatl_lowlevel validate_calibration config/calibration-template.json
```

The calibration template intentionally fails validation until measured values are
provided. Geometry retains legacy units (millimeters) and behavior, including IK
clamping for unreachable targets; it is reference math, not a motion controller.

## Hardware and next steps

No default node opens CAN, enables real motors, or writes motor zeros. The
optional telemetry CLI was carried over for explicit read-only CAN sessions;
`--live` transmits requests. See [motor map](docs/motor-map.md) and
[driver notes](docs/cybergear-driver-dependency.md). Live dependencies are pinned
in `requirements-pi-live.txt` and are not required for mock operation.

Live position control is deferred. Timestamped read-only hardware telemetry is
available. Before adding motion, connect the existing calibration/command limiter
and verify disable/timeout behavior on the robot. The standalone
validation helpers are not yet connected to the mock node. Walking, joystick
control, gait state machines, dashboard, automatic standing, and zeroing are
outside this initial version.

See [migration notes](docs/migration.md) for provenance and scope.

## Robot state contract

`tocatl_lowlevel` publishes `tocatl_interfaces/msg/RobotState` on `robot_state`
at 50 Hz alongside the existing `joint_states` topic. Topics are relative so ROS
namespaces work normally. The publisher uses reliable, volatile QoS with depth 10;
consumers should detect a stopped publisher using the header timestamp and their
own receive timeout. `sequence` increases per snapshot and resets on node restart.
The `body_frame` parameter defaults to `base_link`.

The contract contains joint feedback, per-motor CAN identity and telemetry,
command age/timeout, motor enable state, safety state, calibration/E-stop/fault
status, and optional odometry, IMU, battery, and foot contact/force measurements.
Nested headers retain acquisition time and coordinate frame; republishing cached
hardware telemetry must not refresh its acquisition stamp. Values use SI units
except temperature in Celsius and voltage in volts. Unknown motor readings are
NaN, unknown joint velocity/effort arrays are empty, and optional sensor payloads
must be ignored unless their validity flag is true. Each motor has separate
telemetry freshness and fault-status validity. An empty foot array means no foot
measurements are available. Consumers must check validity as well as freshness.

The mock reports simulated positions and software enable/watchdog status only.
It leaves physical safety, calibration, fault, and sensor validity false, and
keeps `safety_state` DISARMED even when mock motors are enabled. Before the first
accepted command, command age is NaN; enabling starts the watchdog grace period.
A timeout stays reported until an explicit enable/disable request resets it.
This topic is observation, not an arming command or proof of hardware safety.

After the package rename, rebuild and source `install/setup.bash` in a fresh
terminal. The old `tocatl_control` build/install directories, if present, are
stale artifacts and can be removed.

## Read-only hardware telemetry

The hardware backend polls all 18 motors on both buses without enabling, stopping,
zeroing, or changing modes. Motion is disabled by default. The `motor_command` execution path is described
below. Existing motor enable state is observed from feedback when available.
`motors_enabled_valid` / per-motor `enabled_valid` distinguish unknown from disabled.

Install optional dependencies into a system-site-packages venv so ROS Python
packages remain available:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-pi-live.txt
```

Configure each SocketCAN interface at 1 Mbps (once, before starting the server):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

Then source ROS and the built workspace and start using the venv interpreter:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python install/tocatl_lowlevel/lib/tocatl_lowlevel/motor_controller \
  --ros-args --params-file src/tocatl_bringup/config/hardware.yaml
```

If the driver dependencies are already installed for the ROS executable's Python,
`ros2 launch tocatl_bringup hardware.launch.py` is equivalent. Inspect feedback with
`ros2 topic echo /robot_state`. Channel names, polling period, and freshness timeout
are configured in `src/tocatl_bringup/config/hardware.yaml`.

Hardware motor positions are raw encoder radians, not calibrated joint positions;
`joints` remains empty and hardware mode does not publish `/joint_states` until
calibration conversion is implemented. Feedback includes velocity, estimated torque,
temperature, numeric mode status (0 reset, 1 calibration, 2 running), and faults.
Voltage, IMU, battery, odometry, physical E-stop and calibration validity remain
unavailable. Nonresponding motors have invalid telemetry and NaN readings; cached
readings keep their original receive timestamp and become stale after the timeout.
The snapshot header is publication time; motor headers are local receive time in
the ROS clock, not device acquisition time. Fault validity expires independently.
With the pinned driver's 10 ms delay per request, a complete 36-request sweep takes
at least 360 ms; 50 Hz snapshots do not imply 50 Hz hardware measurements.

## Motor-space commands

`motor_command` uses `tocatl_interfaces/msg/MotorCommand`. Every batch contains
exactly one `MotorTarget` for every motor ID: `11,12,13,21,22,23,31,32,33,41,42,43,
51,52,53,61,62,63`. Ordering is arbitrary; duplicate, missing, and unknown IDs
reject the whole batch. The bus comes from the canonical motor map.

Each target specifies raw `position_rad`, `velocity_rad_s`, feedforward `torque_nm`,
`kp` (Nm/rad), and `kd` (Nm/(rad/s)). CyberGear operation control uses all five
values. No IK, joint zero/sign transform, or mechanical zeroing occurs here.
`max_motor_torque_nm` limits feedforward torque; impedance feedback can add torque.
The mock tracks position instantly and does not simulate velocity/torque dynamics.

Set `header.stamp` to the publishing node's current ROS time and leave `frame_id`
empty. Stamps must increase and be no more than `command_max_age_s` old (0.5 s by
default); zero/future/replayed stamps are rejected. Use a single command publisher.
The subscriber is reliable/volatile with depth 1. Publish continuously: the last
accepted desired state is executed until replaced or the command watchdog expires
(default 1 s of local monotonic time). Invalid batches do not refresh the watchdog.
`RobotState.accepted_motor_command_stamp` acknowledges accepted input; execution
is reported separately through `safety_state`, `status_reason`, and motor feedback.

For a mock test, launch the default mock backend, enable it with `/motors/enable`,
and use a Python publisher like this (Ctrl-C stops publishing):

```python
import rclpy
from tocatl_interfaces.msg import MotorCommand, MotorTarget

rclpy.init()
node = rclpy.create_node('motor_targets')
pub = node.create_publisher(MotorCommand, 'motor_command', 1)
ids = [leg * 10 + joint for leg in range(1, 7) for joint in range(1, 4)]

def publish():
    msg = MotorCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.motors = [MotorTarget(motor_id=i, position_rad=0.1,
                             velocity_rad_s=0.0, torque_nm=0.0, kp=1.0, kd=0.1)
                  for i in ids]
    pub.publish(msg)

timer = node.create_timer(0.05, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

For hardware execution, configure `allow_hardware_motion: true` and **measured**
`motor_min_position_rad` and `motor_max_position_rad` arrays (18 entries each in
ascending motor ID order) in the hardware YAML. Unset envelopes prevent arming.
These limits are raw motor coordinates and need no IK or joint offset transform.
Targets must fit those envelopes and the configured velocity, torque, and gain
bounds. The default maximum step from measured position is 0.1 radians.

Stream a full command close to current measured positions, then call
`/motors/enable` with `{data: true}`. Success acknowledges an arm request; watch
`RobotState.safety_state` for ARMED or FAULT. The worker sets operation-control
mode, preloads each target, enables motors, and continues sending desired states.
Calling with `{data: false}` requests stops for motors touched by this controller.
A timeout, stale/missing feedback or fault status, reported fault, excessive step,
or CAN send error disarms and sends stop requests. Stop transmission failures are
reported and retried while the server runs. A new accepted command and explicit
enable request are required after a fault/timeout. Shutdown also attempts stops.
Batch validation is atomic; CAN writes across multiple motors are sequential.
The driver delays each send by 10 ms, so execution of 18 targets takes at least
180 ms plus telemetry contention, regardless of the ROS publishing rate.

The physical server remains configured with motion disabled. The command path was
verified with mock/fake drivers and virtual CAN; physical motor motion was not
used to test it. Physical E-stop state is still unavailable to this software.
The legacy `joint_commands` topic continues to work only with the mock backend.

## Leg 3 hard-stop homing

A separate opt-in single-motor homing path supports hip 32 and knee 33. It uses
three independent torque-limited contacts, backoffs, feedback/travel/time guards,
and an atomic software reference file. It does not write device zeros or enable
other motors. See [homing procedure and limitations](docs/homing.md) and the
intentionally disabled `src/tocatl_bringup/config/homing-leg3.yaml` template.
Physical commissioning and selection of a mechanically safe torque cap remain
required; the implementation has only been tested with synthetic/fake hardware.
