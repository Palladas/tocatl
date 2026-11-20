#!/bin/bash
# One snapshot: safety state and reason, per-motor validity and position, CAN counters.
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /robot_state --field safety_state --once
ros2 topic echo /robot_state --field status_reason --once
ros2 topic echo /robot_state --once | grep -E "motor_id|bus:|telemetry_valid|position_rad" | paste - - - -
for b in can0 can1 can2; do ip -br link show $b 2>/dev/null; ip -s link show $b 2>/dev/null | sed -n 4p; done
