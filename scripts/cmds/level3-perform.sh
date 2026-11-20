#!/bin/bash
# Level 3 controller for feet-in-air performance: hip pitch + knee of all six legs (yaws
# unpowered), homing allowed, motion allowed (motion-feet-air.yaml: kp <= 30, kd <= 2, 1.5 Nm).
# Robot supported, every foot clear of the ground. Then home, run perform.sh and teleop.sh
# (COMMANDS.md E8). Pass extra controller parameters after the script.
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
C=src/tocatl_bringup/config
RUN=~/tocatl-runs/cli-$(date -u +%Y%m%dT%H%M%SZ); mkdir -p "$RUN"; echo "Run folder: $RUN"
IDS="[12,13,22,23,32,33,42,43,52,53,62,63]"
.venv/bin/python install/tocatl_lowlevel/lib/tocatl_lowlevel/motor_controller --ros-args \
  --params-file $C/homing-hip-knee.yaml --params-file $C/motion-feet-air.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false \
  -p homing.output_path:="$RUN/hip-knee-homing.json" \
  -p allow_hardware_motion:=true -p "actuation_motor_ids:=$IDS" -p "telemetry_motor_ids:=$IDS" \
  "$@" 2>&1 | tee "$RUN/controller.log"
