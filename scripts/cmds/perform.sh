#!/bin/bash
# Feet-in-air performance cues (tocatl_perform). Needs a running Level 3 controller with every
# leg homed (level3-perform.sh). Plans and reports every cue first. Options after the script,
# e.g. --dry-run or --speed 0.5 (raw motor rad/s, default 1.0).
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run tocatl_kinematics tocatl_perform "$@"
