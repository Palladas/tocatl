# Last actual sample time, not the time a cached sample was republished.
std_msgs/Header header
string joint_name
uint16 motor_id
string bus
bool telemetry_valid
bool telemetry_fresh
bool enabled_valid
bool enabled
# SI units; unavailable numeric readings MUST be NaN.
# Hardware values are raw encoder coordinates until calibration_valid is true.
float64 position_rad
float64 velocity_rad_s
float64 torque_estimate_nm
float64 temperature_c
float64 bus_voltage_v
string mode
# Empty faults only means fault-free when fault_status_valid is true.
bool fault_status_valid
string[] faults
