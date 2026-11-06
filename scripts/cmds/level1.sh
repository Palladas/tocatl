#!/bin/bash
# Level 1: read-only telemetry for every motor on can0/can1/can2 (motion and homing off).
# Pass extra controller parameters after the script, e.g. -p can_transport:=v2
cd ~/tocatl && source /opt/ros/jazzy/setup.bash && source install/setup.bash
RUN=~/tocatl-runs/cli-$(date -u +%Y%m%dT%H%M%SZ); mkdir -p "$RUN"; echo "Run folder: $RUN"
.venv/bin/python install/tocatl_lowlevel/lib/tocatl_lowlevel/motor_controller --ros-args \
  --params-file src/tocatl_bringup/config/hardware.yaml "$@" 2>&1 | tee "$RUN/controller.log"
