#!/bin/bash
# Read-only: which motors answer on which CAN bus (get-ID only). Buses must be UP (COMMANDS.md B)
# and no controller or bench running. Optional bus names after the script, e.g. can-scan.sh can2
cd ~/tocatl
if ps -eo args | grep -qE "[m]otor_controller|[m]otor_bench|[t]ocatl_play|[t]ocatl_perform|[t]ocatl_walk"; then
  echo "A controller/bench is running and owns the buses: stop it first." >&2; exit 1
fi
.venv/bin/python scripts/can_scan.py "$@"
