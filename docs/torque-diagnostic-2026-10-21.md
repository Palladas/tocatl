# Leg 3 hip torque diagnostic — 2026-10-21

Motor 32, can0; knee and all other motors disabled. Tests performed at the hip's
manually identified stop. The nonzero ramp was directed away from that stop.

## Observations

Disabled read-only queries returned zero for `iq` (0x3020), filtered `iqf`
(0x301e), and `torque_fdb` (0x302d). `run_mode` (0x7005) was zero, operation
control. UINT8 mode replies must use byte 4 only; higher reply bytes are padding,
not part of the mode value. Fault register 0x3022 returned zero.

A two-second enabled zero-torque trial captured 42 operation-control frames.
Every frame encoded approximately -0.000183 Nm (nearest representable zero),
kp=0, kd=0. Encoder span was 0.0003815 rad. During the enabled interval:

| Reading | Minimum | Maximum |
|---|---:|---:|
| Raw iq (A) | -0.7023 | +0.4988 |
| Filtered iqf (A) | -0.1084 | +0.1607 |
| State-frame torque estimate (Nm) | -0.000916 | +0.006409 |
| torque_fdb register | 0 | 0 |

The next trial used a 0.002 Nm/s ramp with a 0.02 Nm command cap. It stopped on
its existing 0.1 Nm immediate estimated-torque cutoff before breakaway:

- 99 captured operation-control frames; all kp=kd=0.
- Largest wire-decoded command: +0.01007095 Nm.
- At feedback timestamp 1792621459.991179, state torque was +0.1950103 Nm.
- The preceding command, at 1792621459.9806604, was +0.01007095 Nm. Raw CAN
  command ID: `0x01801b20`; payload: `7fff7fff00000000`.
- Peak feedback ID: `0x02802000`; payload: `787c800b82140103`.
- Enabled encoder span remained 0.0003815 rad, below the 0.003 rad breakaway rule.
- Raw iq ranged -0.8392 to +0.5800 A; filtered iqf -0.1903 to +0.1619 A.
- Current/register queries were asynchronous (roughly five readings/s per
  register), so they cannot be paired exactly with the torque spike.
- `torque_fdb` was predominantly zero, with one negative reading; its usefulness
  as an independent estimate is not established. No conversion from current to
  output torque has been validated here.
- The selected motor's disabled state was confirmed after stopping.

## Conclusion and limits

The captured commands exclude an oversized CAN torque command or nonzero
impedance gains as the explanation for this trial. The discrepancy exists between
the correctly encoded small command and motor-originated feedback/current.
These observations cannot distinguish actual current-control fluctuations from
feedback noise, firmware behavior, or a sensor/calibration/scaling problem.
A static encoder does not prove zero physical torque. No safe-contact torque or
minimum breakaway torque was established. Do not raise abort limits based on
this diagnostic alone.

Before further powered ramps, inspect motor calibration/firmware and current-loop
configuration using the appropriate motor documentation and commissioning tools;
validate current and torque response with adequate sampling and known unloaded
conditions. No motor calibration, tuning, zero, or persistent configuration was
changed during this diagnostic.

Raw captures and the read-only capture script are saved under
`log/torque-diagnostic-20261021/` (normally git-ignored). The script transmits
parameter-read requests, never motor enable/control/zero commands. Read-only
hardware monitoring was restored after the diagnostic.

## Requested repeat ramp

A subsequent repeat retained the 0.002 Nm/s ramp, 0.02 Nm command cap,
0.003 rad motion criterion, and 0.1 Nm immediate estimated-torque cutoff.
Largest captured command was +0.01007095 Nm before feedback reached
+0.1481346 Nm and the controller aborted. Enabled encoder span was
0.0007630 rad, insufficient for breakaway. Disable was confirmed and read-only
monitoring restored. Trace: `log/torque-diagnostic-20261021/tocatl-hip-ramp-repeat.jsonl`.
This trial did not reach the configured command cap and does not establish a
minimum breakaway torque or a lower bound of 0.02 Nm.

## Increased-limit trial

The operator-requested increased-limit preset was tested: 0.05 Nm command cap,
0.002 Nm/s ramp, 0.2 Nm immediate feedback cutoff, 0.15 Nm sustained for 0.15 s,
and 30 s timeout. It aborted at +0.2532387 Nm reported torque; the largest
wire-decoded command was only +0.01007095 Nm. Encoder span was 0.0007630 rad,
below the 0.003 rad movement criterion. The selected motor's disabled state was
confirmed. No home reference or breakaway estimate was written. Read-only
monitoring was restored. Capture:
`log/torque-diagnostic-20261021/tocatl-hip-ramp-increased-wire.jsonl`.

This trial did not reach the command cap. Further automatic increases of the
feedback cutoff are not justified by the data. Supporting the leg to reduce
external load, without restricting the tested joint, provides a useful next
comparison; external loading remains a hypothesis, not an established explanation
of the feedback spikes.

## 0.3 Nm cutoff trial and load clarification

At the operator's request, the away-only preset was changed to a 0.3 Nm
immediate cutoff and 0.2 Nm sustained cutoff (0.15 s); the command cap remained
0.05 Nm, ramp 0.002 Nm/s, timeout 30 s. Validation ceiling was updated to 0.3 Nm;
93 lowlevel tests passed before the run. It aborted at +0.3008469 Nm feedback.
The largest captured command was only +0.01922637 Nm; encoder span was
0.0007630 rad. Motor disable was confirmed and read-only monitoring restored.
Trace: `log/torque-diagnostic-20261021/tocatl-hip-ramp-point3-wire.jsonl`.

The operator clarified that the legs are heavy and described the actuators as
18 Nm. The previous command caps are small compared with that rating, and none
of the aborting trials establishes the gravity-loaded breakaway requirement.
The installed driver's encoded torque range is +/-12 Nm, so verify actuator
model and torque scaling before selecting a larger lifting-test range. No
assumption that motor torque rating is the printed-stop load rating is warranted.

## Operator-supplied model and configuration readback

The operator supplied label text `Xiaomi MIQDD PG061202A`. An exact public
model-number match was not verified. A read-only query of hip motor 32 returned:

- 0x2007 configured torque limit: 12.0.
- 0x200f configured internal gear ratio: 7.75.
- 0x2019 configured current limit: 27.0.
- 0x302e rated-current register: 8.0.
- 0x302f limit-current register: 27.0.

All were float-type replies. These are firmware configuration values, not
independent certified physical ratings. No settings were written and no motor
was enabled. Raw readback: `log/torque-diagnostic-20261021/hip32-config-readback.json`.
The 7.75 internal reduction is distinct from the operator's direct mounting of
the actuator output to the hip; do not multiply commanded output torque by it
again. Standard CyberGear documentation describes +/-12 Nm CAN torque encoding;
this readback supports that family identification but does not prove a particular
variant's peak or continuous rating.

## Sequential command-limit trial

Implemented away-only command stages 0.05, 0.10, 0.20, 0.35, 0.50 Nm, with
0.02 Nm/s ramps and one-second holds. No-motion feedback advances stages rather
than ending the run at each intermediate cap. A bounded 35 s phase timeout,
1.0 Nm immediate feedback cutoff and 0.75 Nm sustained cutoff (0.15 s) remain.
Full homing and zero-baseline checks are unchanged. 94 lowlevel tests passed.

The trial reached all five stages without a torque trip. Highest wire-decoded
command was 0.4997024 Nm; peak state-frame torque was 0.7450675 Nm. It timed out
at 35 s without reaching the 0.003 rad relative movement criterion. Position
changed from the preflight reference about -0.7337682 to -0.7318608 rad
(+0.0019074 rad, 0.109 degrees). The full enabled encoder range, including its
one-count initial fluctuation, was 0.0022889 rad.

After disabling, position returned to approximately -0.7333867 rad. Most loaded
displacement therefore relaxed; it cannot be classified as confirmed breakaway
rather than elasticity/backlash/settling from these observations alone. This
trial establishes a small encoder response under 0.5 Nm command, not a reliable
minimum breakaway or safe contact torque. No home offset was saved. Disable was
confirmed and read-only monitoring restored. Trace:
`log/torque-diagnostic-20261021/tocatl-hip-ramp-staged-wire.jsonl`.

## Sequential ramp extended to 1 Nm

At the operator's request, away-only staged control was extended through 0.75
and 1.00 Nm, retaining the 0.02 Nm/s ramp, one-second holds, encoder-based speed,
0.003 rad movement stop, 0.02 rad travel bound, and telemetry/fault guards. This
preset uses 1.5 Nm immediate and 1.25 Nm sustained (0.15 s) estimated-torque
cutoffs, with a 65 s phase timeout. Larger command/feedback ceilings apply only
to staged away-only trials; full homing limits are unchanged. 95 lowlevel tests
passed before the run.

The movement criterion was reached before the 1 Nm cap:

- Previous commanded torque at detection: 0.7937905 Nm (pre-encoding).
- State-frame torque estimate at detection: 0.8424811 Nm.
- Initial encoder position: -0.7333867 rad.
- Detection encoder position: -0.7303349 rad.
- Displacement: +0.0030518 rad (0.17485 degrees).
- Torque was removed and disabled feedback confirmed. Position relaxed back
  toward the starting pose (approximately -0.7330053 rad shortly after disable).

This is the first successful measurement of torque reaching the configured
small-displacement criterion in this pose. Because much of the displacement
relaxes, it is not proof of sustained free motion or a calibrated static-friction
breakaway torque; elasticity, backlash, gravity loading and encoder location
remain relevant. It does not establish an appropriate hard-stop contact torque.
No home offset or device zero was written. Read-only monitoring was restored.

Result: `log/torque-diagnostic-20261021/hip-breakaway-one-nm-result.json`.
Wire trace: `log/torque-diagnostic-20261021/tocatl-hip-ramp-one-nm-wire.jsonl`.

## Sequential ramp with 2 Nm maximum

At the operator's request, staged away-only ceilings were raised to 2 Nm
commanded and 2.5 Nm immediate feedback (2.25 Nm sustained for 0.15 s). Stages
now include 1.5 and 2.0 Nm, retaining 0.02 Nm/s ramps, one-second holds, the
0.003 rad displacement stop and other existing motion/freshness/fault guards.
Phase timeout is 120 s. Full homing bounds are unchanged. 96 lowlevel tests passed.

The repeat stopped on the movement criterion before reaching the new cap:
commanded torque 0.8521272 Nm, state torque estimate 0.8168460 Nm, initial raw
position -0.7330053 rad, detection position -0.7299535 rad (0.0030518 rad,
0.17485 degrees). It disabled successfully and relaxed near the starting pose.
This is a repeat of the small-displacement response previously reached at
0.7937905 Nm; neither trial establishes free-motion breakaway or safe hard-stop
contact torque. The motor was not driven at 2 Nm. No home/device zero was written.
Read-only monitoring was restored.

Result: `log/torque-diagnostic-20261021/hip-breakaway-two-nm-result.json`.
Capture: `log/torque-diagnostic-20261021/tocatl-hip-ramp-two-nm-wire.jsonl`.

## Sequential ramp with 3 Nm maximum

The operator requested a 3 Nm cap and explicitly retained the 0.175 degree
movement stop. Staged-only ceilings were updated to 3 Nm command and 3.5 Nm
immediate feedback; the preset uses 3.25 Nm sustained for 0.15 s, 175 s phase
and 200 s total timeouts. The total timeout is now exposed as
`homing.total_timeout_s`. Stages add 2.5 and 3.0 Nm while retaining 0.02 Nm/s
ramps and one-second holds. Non-staged limits are unchanged. 97 tests passed.

The repeat reached the movement criterion at 0.8285997 Nm commanded and
0.8607919 Nm reported. Raw position changed from -0.7330053 to -0.7299535 rad,
0.0030518 rad (0.17485 degrees). It stopped and confirmed disable before the
3 Nm cap, relaxing to approximately -0.7330053 rad. No 3 Nm command, homing
contact, device zero or home offset was applied. Read-only monitoring restored.
This repeats the earlier approximately 0.8 Nm small-displacement response; it
remains distinct from demonstrated sustained free-motion breakaway.

Result: `log/torque-diagnostic-20261021/hip-breakaway-three-nm-result.json`.
Capture: `log/torque-diagnostic-20261021/tocatl-hip-ramp-three-nm-wire.jsonl`.

## 5 Nm maximum, 1 degree movement target

The operator requested a 5 Nm cap and 1 degree movement. Added an independent
`homing.breakaway_motion_rad` parameter, default 0.003 rad, to avoid changing
baseline drift or homing contact stability tolerances. The away-only preset now
uses 0.01745329 rad, retains the 0.02 rad travel guard and 0.02 Nm/s ramp, and
adds stages through 5 Nm. Staged-only command/feedback ceilings are 5 / 5.5 Nm;
the sustained cutoff is 5.25 Nm for 0.15 s, phase timeout 280 s, total 300 s.
Non-staged limits are unchanged. 98 lowlevel tests passed before the run.

The run reached its target before 1 Nm commanded:

- Commanded torque at detection: 0.9777316 Nm.
- Reported torque: 1.0274205 Nm.
- Initial raw position: -0.7326238 rad.
- Detection position: -0.7143130 rad.
- Displacement: 0.0183108 rad, 1.04913 degrees.
- Zero command and stop followed, with disabled feedback confirmed. The leg
  moved back toward the stop when unloaded; state immediately after disable was
  approximately -0.7368200 rad. No holding command was applied afterward.

This is torque reaching the 1 degree encoder criterion under this specific leg
load/pose. It is not a safe-contact torque or general calibration for all poses,
nor proof of the sustained torque needed to hold the leg. No 5 Nm command or
home/device zero was applied. Read-only monitoring restored.

Result: `log/torque-diagnostic-20261021/hip-breakaway-five-nm-one-degree-result.json`.
Capture: `log/torque-diagnostic-20261021/tocatl-hip-ramp-five-nm-one-degree-wire.jsonl`.

## Requested 5 degree test with slow return

Implemented a bounded supporting-torque trajectory for return, preserving support
at turnaround and unloading at the starting pose. Initial aggressive simulation
gains were corrected before physical operation. The final code adds outward
trajectory control after initial movement for targets >0.02 rad. 111 lowlevel
tests pass, including model-based lift/return tests and aborts on abrupt loads.

A smaller commissioning run reached approximately 1 degree at 0.9574730 Nm
commanded, 1.0523232 Nm reported. Raw positions: start -0.7345312, target
-0.7169833, returned -0.7326238 rad. The return took about 40 s, unloaded to
zero torque, and confirmed disable. Result `slow_return_completed` was true.
The returned encoder was within 0.0019074 rad (0.1093 degrees) of the initial
pose, consistent with the configured settling/unloading tolerances.

The first 5 degree run, with only the open-loop outward ramp, aborted on the
0.12 rad/s encoder-derived overspeed guard at about 2.4 degrees before target.
The second, using outward trajectory control after initial movement, aborted on
the 0.06 rad/s trajectory-speed guard while lifting, before reaching 1 degree.
The error string at the time read 'Slow return overspeed' because lift and return
share a controller; this was subsequently corrected to identify the lift phase.
Both aborts confirmed disable. No successful 5 degree result was saved. Outward
control needs further physical tuning; the speed guard was not raised to force
completion. Read-only monitoring was restored after the attempts.

Artifacts in `log/torque-diagnostic-20261021/`:

- `hip-one-degree-slow-return-result.json`
- `tocatl-hip-one-degree-slow-return-wire.jsonl`
- `tocatl-hip-five-degree-slow-return-wire.jsonl`
- `tocatl-hip-five-degree-controlled-roundtrip-wire.jsonl`

No motor zero, homing offset, firmware calibration or driver tuning was written.

## Higher trajectory-speed cutoff retry

At the operator's request, the trajectory cutoff was exposed as
`homing.trajectory_overspeed_rad_s` and the preset raised from 0.06 to 0.10 rad/s.
The outer 0.12 rad/s guard, 0.01 rad/s reference rate, 5 degree target and 5 Nm
command cap were unchanged. 112 lowlevel tests passed before the run.

The run entered LIFT_SLOWLY, then aborted on 'Slow lift overspeed'. Initial raw
position was -0.7333867 rad; the fault snapshot position was -0.7085908 rad
(approximately 1.42 degrees from start), with 0.8974 Nm estimated torque.
Disabled feedback was confirmed. No successful roundtrip result was written;
no homing/device zero was applied. Read-only monitoring was restored.
Capture: `log/torque-diagnostic-20261021/tocatl-hip-five-degree-speed-point1-wire.jsonl`.
This shows the higher cutoff still tripped during outward acceleration, not
successful tracking of the requested slow trajectory or a completed 5 degree test.
