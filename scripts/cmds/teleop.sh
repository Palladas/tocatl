#!/bin/bash
# Xbox controller: joy_node in the background, tocatl_teleop in the foreground (Ctrl-C stops both).
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0 &
JOY=$!
trap 'kill $JOY 2>/dev/null' EXIT
ros2 run tocatl_kinematics tocatl_teleop "$@"
