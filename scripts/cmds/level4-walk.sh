#!/bin/bash
# Level 4 controller for standing and walking: all 18 motors, homing allowed, motion allowed,
# soft fault response. Then home (tripods or one leg at a time), declare yaw with every leg
# radial, and run tocatl_walk or the teleop (COMMANDS.md E6/E7).
# Pass extra controller parameters after the script, e.g. -p can_transport:=v2
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
C=src/tocatl_bringup/config
RUN=~/tocatl-runs/cli-$(date -u +%Y%m%dT%H%M%SZ); mkdir -p "$RUN"; echo "Run folder: $RUN"
IDS="[11,12,13,21,22,23,31,32,33,41,42,43,51,52,53,61,62,63]"
.venv/bin/python install/tocatl_lowlevel/lib/tocatl_lowlevel/motor_controller --ros-args \
  --params-file $C/homing-hip-knee.yaml --params-file $C/motion-standing.yaml \
  -p allow_homing:=true -p "homing.legs:=[1,2,3,4,5,6]" -p homing.verify_repeatability:=false \
  -p homing.output_path:="$RUN/hip-knee-homing.json" \
  -p allow_hardware_motion:=true -p "actuation_motor_ids:=$IDS" -p "telemetry_motor_ids:=$IDS" \
  -p fault_response:=soften "$@" 2>&1 | tee "$RUN/controller.log"
