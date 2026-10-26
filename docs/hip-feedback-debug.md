# Hip feedback interruption investigation

The observed hold failure is preceded by hip effort approaching the decoded
12 Nm feedback limit and loss of hip replies. The knee continues reporting.
The software stops when hip fault-status age exceeds 250 ms, and fresh disabled
acknowledgement is delayed roughly 1.7–1.8 seconds. A returning packet sometimes
reports 0 C transiently. This is consistent with several failure modes; it is
not sufficient to diagnose a reset, undervoltage or wiring fault.

## Current captures

- `log/homing-leg1-20261023/hip-hold-failure-watch.jsonl`: ROS states and reasons,
  including intermediate faults before stop-retry status changes.
- `log/homing-leg1-20261023/hip-power-can-debug-1.jsonl`: raw CAN data and queries,
  with kernel timestamps and host receipt timestamps. This captures hip/knee
  traffic independently of the controller's callback and polling logic.

The probe queries only allowlisted registers using command type 9 (parameter
read). It sends no enable, stop, control, zero, mode or configuration write.
Voltage (0x3007, UINT16 millivolts), tick (0x301c, UINT32), warning status
(0x3023, UINT32) and driver fault (0x3024, UINT16) follow the installed
CyberGearDriver parameter definitions. Voltage/tick reads cover motors 12 and
13, with warning/driver fault reads on motor 12. Existing fault replies from
controller polling are also recorded. Tick units are not assumed.

Baseline while both motors were disabled: hip 21.901–21.914 V, knee
21.965–21.978 V, no tick counter backwards jumps, no nonzero fault/warning
replies. CAN interface reported no receive/transmit errors or bus-off events.
No new powered trial was started as part of setting up these diagnostics.

## Running the probe

Use a new output filename for every capture. From the workspace root:

```bash
PYTHONPATH=src/tocatl_diagnostics .venv/bin/python -m \
  tocatl_diagnostics.hold_capture --query \
  --output log/homing-leg1-20261023/hip-power-can-debug-2.jsonl
```

Omit `--query` for a completely passive capture. Ctrl+C stops the recorder only;
it does not send any motor command. `--duration 60` limits recording to a minute.
Do not run multiple query probes concurrently.

Analyze a completed capture:

```bash
PYTHONPATH=src/tocatl_diagnostics .venv/bin/python -m \
  tocatl_diagnostics.hold_capture --analyze \
  log/homing-leg1-20261023/hip-power-can-debug-1.jsonl
```

The analyzer reports voltage ranges, raw tick counter backwards jumps,
nonzero fault/warning values, and gaps over 250 ms with host read counts and
other-motor activity in the same interval. Readonly polling normally has gaps
above 250 ms; interpret gaps relative to the active hold and polling requests.
Counter discontinuities alone do not establish reset semantics. Host receipt
lags versus kernel timestamps help distinguish a reader backlog from a bus gap.

## Next reproduction

The operator owns controller startup and test initiation. Restart the controller
to load the source change preserving the original failure reason, then repeat
the 30-degree hold with the capture and ROS recorder running. No watchdog,
force, temperature or travel setting is relaxed by this probe. Compare the last
valid voltage/tick values, first returning tick, hip and knee reply gaps, and
continued host requests. Record the supply type, voltage/current setting and
whether a bench supply enters current limiting.

Register voltage sampling cannot rule out a short supply collapse while the
motor cannot reply. A measurement at the actuator power input may still be
needed. The probe itself adds only read requests but changes bus traffic; if
behavior changes, compare against passive-only capture.

Validation: 12 diagnostics tests passed, including query allowlisting,
register decode/padding and reply-gap correlation.


## Later startup failure

The controller was absent and the hold YAML had been edited to `sweep.effort_limit_nm: 20.0` with proportional limit 10.5. Offline construction reproduced `Unsupported hip sweep torque profile`. Restored the previously authorized driver ceiling to 12.0; validated the actual YAML offline and improved the exception to show supplied and supported limits. No controller or powered test was started. Raw CAN showed a preceding stop and immediate disabled acknowledgement; fresh state-read queries confirmed motors 12 and 13 in mode 0.

The capture also contains both tick counters returning near zero and a voltage transition from about 21.9 V to about 24.7 V. These observations cannot be attributed to a fault without knowing whether the operator power-cycled or changed the supply. They are separate from the confirmed startup validation error.


## Replacement motor on bench

A read-only ID scan found the replacement at physical CAN ID 127, with ID 12
absent. The operator confirmed it is a loose motor on the bench, not installed
in the leg. The old leg home reference therefore must not be used.

`hold-bench-replacement-30.yaml` selects `bench_position_hold` and a temporary
`leg1_hip_can_id: 127` transport override. Logical motor 12 in ROS and controller
ownership maps to physical driver address 127; no stored motor ID is changed.
Replies from physical 127 are routed back to logical 12. Overrides reject
same-bus address collisions and invalid IDs. The normal leg profile is unchanged.

Bench mode owns only the replacement and does not depend on passive knee
feedback. It still rejects other observed running motors. Its target is 30
encoder degrees above a fresh disabled bench reference, with the same native
PD gains, effort slew and existing non-position guards. The reference is not
homing calibration. Start refuses if the motor moved more than 2 degrees from
the captured reference before enable. `hold.reference_rad` is deliberately
nonfinite until the housing is secured and a reference is captured. No powered
bench test has been run; fixture confirmation is pending.

Diagnostics now support `--hip-id 127` and capture reads/replies for 127 and 13.
Read-only replacement capture: `replacement-127-power-can-debug-1.jsonl`.
Validation: 82 hardware/homing/diagnostics checks passed for the address routing
change; 22 focused checks passed after bench mode and capture support were added.
Three ROS packages built successfully with symlink install.


### Secured replacement bench test completed startup

The operator confirmed the replacement housing was secured and the shaft clear.
Initial CAN reads encountered a full transmit queue. Cycling can1 down/up did
not restore replies. With no controller running, unbinding/rebinding only
USB interface `4-2:1.0` from gs_usb and restoring can1 at 1 Mbit/s recovered
immediate disabled replies from physical IDs 127 and 13. This establishes a
host/adapter recovery method, not a proven cause of the earlier loaded failures.

Captured twelve stable disabled position readings from ID 127: reference
-0.6486991684 rad, target -0.1251003928 rad (+30 encoder degrees). Set only
the temporary bench profile's reference; the original leg profile and persistent
motor CAN ID/calibration were unchanged. The replacement controller started with
logical hip 12 mapped to physical ID 127, ramped to +29.94 degrees and remained
holding with fresh fault-free feedback at about 27.3 C. The knee stayed disabled.
The hold is intentionally left active until operator cancellation or a guard;
confirmation JSON is a timestamped snapshot, not live state.

Live log: `bench-127-hold-state.jsonl`. Raw capture:
`replacement-127-power-can-debug-2.jsonl`. Reference and startup confirmation:
`replacement-bench-reference.json`, `replacement-bench-hold-confirmation.json`.
Stop via `/motors/enable` SetBool `data: false`. Diagnostic query probe and
bench state recorder are running; they send no motion commands.


Current-centered bench sweep update: each trigger captures the selected motor's fresh encoder position as center, without requiring the old 30-degree hold or reference file. `sweep-bench-replacement-15.yaml` runs three cycles center−15 to center+15 degrees, then returns to and continuously holds the captured center. `sweep.amplitude_deg` sets the excursion; the older 10-degree profile remains available with a 10-degree excursion but also captures a fresh center. Endpoints use eased ramps and settling checks. Encoder-range headroom is checked before enabling. Ten focused tests passed, including arbitrary starting positions, fresh center on each trigger, three dynamic cycles, final center hold and stall/stale feedback rejection. The old background controller was stopped after disabled feedback confirmation. No powered test was initiated for this source change.
