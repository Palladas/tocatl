# CAN transport v1 / v2

## Why v1 is slow
- `CyberGearDriver` 0.2.0 sleeps 10 ms (`PAUSE_AFTER_SEND`) after every frame it sends.
- One lock serializes all buses.
- So a command pass over 18 motors takes about 180 ms, roughly 5 updates per second per motor,
  however many buses there are.
- This was the main cause of jerky motion. It also limits how fast homing can poll fault status.

## What v2 changes (`-p can_transport:=v2`, opt-in)
- **Same frames, no pause.** v2 sends exactly the frames the driver builds (a subclass that keeps
  the driver's `_send` without the sleep).
- **Fuller TX queue is handled.** A frame lasts about 130 µs at 1 Mbit/s, so a burst can briefly
  fill a bus's 10-frame TX queue (`ENOBUFS`). v2 retries for up to 20 ms.
- **A frozen bus still faults.** A queue that never drains, because nothing acknowledges (the
  2026-10-25 freeze, or power/wiring/termination problems), still raises and faults as before.
- **Update rate is set by `control_period_s`.** This is the pause between command passes
  (default 0.02 s). With v2 a pass over 18 motors takes a few milliseconds, so 0.01 s gives about
  100 Hz per motor, and each bus stays under about 20% load.
- **Safety paths unchanged:**
  - torque-limit write and readback before enable, and restore after disable
  - watchdog, freshness and thermal guards
  - wiring check
  - homing priority

Tests:
- `test_v2_transport_sends_the_drivers_frames_without_its_pause` checks the frames are unchanged.
- On the Pi, `test_v2_frames_match_the_installed_cyber_gear_driver` checks them against the
  installed driver.
- `test_v2_send_retries_a_briefly_full_tx_queue_then_gives_up` checks the retry.

## Rollout (Pranav runs each step; one change at a time)
1. **Pi tests.** Run `colcon build`, then the tests. Include
   `TOCATL_TIMING_TESTS=1 python3 -m pytest src/tocatl_lowlevel/test/test_homing_timing.py`.
2. **Motor bench.** No change; the bench has its own transport.
3. **Level 1, one leg.** Add `-p can_transport:=v2`. Watch `ros2 topic hz /robot_state`, and run
   `ip -s link show canN` twice to check that TX/RX are rising with no `dropped`.
4. **Level 1, all legs.** Same as step 3.
5. **Level 3, one leg.** Nudge, with `-p can_transport:=v2`.
6. **Level 3, all legs.** Circle and wave. Then try `-p control_period_s:=0.01`.
7. **Make it the default.** Only after these pass, change the default to v2 in code and record it
   in this file.

If any step shows `No buffer space available` or rising `dropped` counts, stop the controller.
Reset that bus (COMMANDS.md B), go back to v1, and note the step that failed.
