# Raw motor/encoder coordinates. No IK, joint offsets, signs, or unit conversion.
uint16 motor_id
float64 position_rad
float64 velocity_rad_s
float64 torque_nm
# CyberGear operation-control impedance gains, Nm/rad and Nm/(rad/s).
float64 kp
float64 kd
