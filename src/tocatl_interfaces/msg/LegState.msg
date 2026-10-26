# Readiness scope is explicit: pitch_knee excludes yaw and whole-leg calibration.
uint8 leg_id
string scope
uint8[] motor_ids
bool homed
bool ready
# Software raw encoder references in motor_ids order; empty until homed.
float64[] home_positions_rad
string status_reason
