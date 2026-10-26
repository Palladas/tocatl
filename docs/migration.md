# Migration scope

Source: `/home/tocatl/romela-tocatl`, Git HEAD
`ee193c53d317fce67e3c7e53cdf681148566eae6` at extraction. Files were copied from
the working tree; the original project was not modified.

Retained the pure calibration, safety-state, command-limiter and CyberGear
adapter helpers with their tests; legacy geometry and its tests; motor inventory;
and the optional telemetry CLI. The canonical inventory now lives in
`tocatl_lowlevel.motor_map`, with a diagnostics re-export for compatibility.

Replaced package metadata with minimal declared dependencies, added installed
bringup configuration and launch files, and put tests inside their packages.
Removed placeholder state-manager behavior from the new project. Added an
explicit mock-only position controller and watchdog tests.

The original legacy scripts, service files, gait references, deployment tooling,
virtual environment, and generated files were not migrated. They remain in the
source repository. Copied driver documentation is historical reference; use the
new workspace README for current build and run commands.

The source packages declare MIT licensing and retain their existing maintainer.
No upstream driver source is vendored here.

Validation: all four packages built with ROS 2 Jazzy; 49 package tests passed.
An installed-launch smoke test verified 18-joint feedback, explicit enable,
position tracking, and rejection after command timeout. No hardware was used.
