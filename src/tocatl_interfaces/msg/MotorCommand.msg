# Publisher ROS timestamp. Must be nonzero, recent, and strictly increasing.
# frame_id must be empty: values are individual raw motor coordinates.
std_msgs/Header header
# Exactly one target for every inventory motor (18); ordering is arbitrary.
# A rejected batch never replaces the previous accepted desired state.
MotorTarget[] motors
