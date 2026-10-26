# Snapshot publication time; header.frame_id is the robot body frame.
# Nested sensor headers retain their own acquisition times and frames.
std_msgs/Header header
uint64 sequence
string backend
bool simulated
string DISARMED=DISARMED
string SELF_TEST=SELF_TEST
string CALIBRATION_CHECK=CALIBRATION_CHECK
string READY=READY
string ARMED=ARMED
string FAULT=FAULT
# Mock motor enabling does not imply physical safety readiness or ARMED.
string safety_state
string status_reason
bool motors_enabled_valid
bool motors_enabled
# Timestamp of the most recently accepted motor_command (zero if none).
# Acceptance does not imply execution; inspect safety_state and status_reason.
builtin_interfaces/Time accepted_motor_command_stamp
bool command_received
# Age of last accepted command, seconds; NaN until the first command.
float64 command_age_s
float64 command_timeout_s
bool command_timed_out
bool telemetry_fresh
bool calibration_valid
bool estop_status_valid
bool estop_engaged
bool fault_status_valid
string[] faults
# Stable canonical joint ordering; unknown velocity/effort arrays are empty.
sensor_msgs/JointState joints
MotorState[] motors
# Optional measurements MUST be ignored unless their validity flag is true.
bool odometry_valid
nav_msgs/Odometry odometry
bool imu_valid
sensor_msgs/Imu imu
bool battery_valid
sensor_msgs/BatteryState battery
FootState[] feet

# Per-leg session homing readiness; inspect scope before actuating.
LegState[] legs
