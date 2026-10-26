# Leg 3 hard-stop homing

This implementation has synthetic-feedback tests, not physical validation. It
operates one selected motor (32 hip pitch or 33 knee), with all others disabled.
Normal 18-motor arming and homing are mutually exclusive. No mechanical-zero
command is sent. Results are software raw-encoder references, not automatically
applied joint calibration. `RobotState.calibration_valid` remains false.

The 2026-10-21 manual observations establish negative encoder homing direction
for both motors. Hip stop was approximately -0.7330 rad; knee stop approximately
-0.8173 rad at the then-current hip pose. The knee linkage makes its reference
dependent on hip pose. These observations are not power-cycle validation.

## Sequence

1. Require fresh mode feedback showing every motor disabled, plus fresh selected
   motor position, velocity, torque, temperature, and separate fault status.
2. Enable only the selected motor in operation-control mode with zero torque and
   zero impedance gains. Wait for fresh enable acknowledgement.
3. Move 0.05 rad away from the stop using ramped, capped torque. Record an estimate
   of breakaway torque at first displacement; no automatic increase of the cap.
4. Approach with a host PI velocity servo targeting 0.03 rad/s, with torque
   clamped to the configured limit. Accept contact only within 0.04 rad of the
   expected stop, with torque at least 80% of the cap, speed <=0.015 rad/s and
   position variation <=0.003 rad for one second and at least three new samples.
5. Unload and back off 0.05 rad between contacts. Collect three contacts; require
   their spread <=0.01 rad. After the third, back off again and disable the motor.
6. Require fresh disabled acknowledgement, then atomically save the three contact
   positions, median home, spread, settings, and timestamp. Preserve the other
   motor's record. An aborted run does not replace a successful result.

The initial retreat is the breakaway trial. If insufficient torque prevents
movement it times out; the operator must assess before selecting a new cap.
A printed stop's safe load cannot be inferred from the motor's rated torque.
The 0.5 Nm software ceiling is not a certified safe contact torque.

Abort on stale/nonfinite feedback, missing fault status, faults, overspeed
(0.12 rad/s), torque above cap plus 0.02 Nm measurement allowance, temperature
>=45 C, raw envelope/travel violations, phase/total timeout, inconsistent
contacts, operator cancellation, or a stall outside the known stop window.
The default maximum travel is 0.5 rad, phase timeout 20 s, total timeout 150 s.
Contact cannot distinguish an obstruction within that window from a real stop.

During homing, telemetry prioritizes the selected motor and polls other motors
round robin. Fresh selected feedback and fault reports are required within 0.25 s.
If this firmware does not return fault reports, preflight refuses to enable.
CAN/stop errors latch a fault and retry stopping until fresh disabled feedback
arrives. A stop that cannot be confirmed keeps the worker alive, including during
shutdown. Physical power removal is the fallback for an unresponsive drive.
A process/host failure cannot be covered by this host-side watchdog alone.

## Supervised commissioning

Use `src/tocatl_bringup/config/homing-leg3.yaml` as a template. It deliberately
leaves `allow_homing: false` and the torque limit unset (zero). Set an explicitly
chosen low cap, verify the raw envelope and expected stop for this session, then
set `allow_homing: true`. Start with the supported robot and clear joint travel.
Only one controller may own CAN commands. Stop the existing controller before
launching the replacement; do not run two hardware controllers concurrently.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python install/tocatl_lowlevel/lib/tocatl_lowlevel/motor_controller \
  --ros-args --params-file src/tocatl_bringup/config/homing-leg3.yaml
```

Starting the node does not start motion. Start one sequence explicitly:

```bash
ros2 service call /motors/home std_srvs/srv/Trigger '{}'
ros2 topic echo /robot_state
```

Service success acknowledges the request, not completed homing. Watch
`status_reason` for phases, completion or failure. Cancel with:

```bash
ros2 service call /motors/enable std_srvs/srv/SetBool '{data: false}'
```

Home the hip first with knee disabled. For knee homing, configure motor 33,
its raw envelope and expected stop, plus `homing.knee_hip_reference_rad`.
Place/support the hip at that reference: it remains disabled; drift >0.01 rad
aborts knee homing. Re-establish the knee stop reference at this exact hip pose
before running. The current implementation homes one joint per configuration;
it does not automatically hold the hip or launch a second joint's sequence.

Results default to `leg3-homing.json` relative to the controller working directory.
Set an absolute `homing.output_path` for a stable location. Back up the result
before mechanical changes. Homing does not discover the full joint travel range.

## Breakaway-only commissioning

Set `homing.breakaway_only: true` to retreat only and stop at the first 0.003 rad
of displacement. This does not approach a stop or create a home offset. Set
`homing.torque_ramp_nm_s` explicitly for a slower ramp. For example, the first
supervised hip trial used a 0.02 Nm cap, 0.002 Nm/s ramp, and 12 s timeout.
These are trial settings, not measured minimum torque or a stop safety rating.
Breakaway results live under `breakaway_tests` in the output JSON and cannot
replace saved `motors` home references.

On the connected firmware, type-21 fault requests did not return reports. The
transport now reads `faultSta` (0x3022, type 9), taking its uint32 from bytes 4–7,
while still accepting unsolicited type-21 fault reports. Unrelated parameter
responses do not refresh fault validity.

The first powered trial on 2026-10-21 aborted with measured torque approximately
-0.0526 Nm, above its 0.04 Nm measured-torque cutoff, without breakaway. Disable
was confirmed. No minimum homing torque was established; investigate enabled
feedback noise and torque response before raising the cap. Live trial telemetry
was saved to `/tmp/tocatl-hip-breakaway-telemetry.jsonl`.

### Follow-up baseline and bounded ramp

A two-second zero-command trial collected 40 fresh enabled samples with zero
impedance gains. Position span was 0.0003815 rad. Estimated torque ranged from
-0.0650 to +0.0141 Nm (median -0.000183 Nm); reported speed varied despite nearly
constant encoder position. This demonstrates isolated feedback excursions, not a
validated torque calibration or proof that larger readings are harmless.

`homing.baseline_only` now collects zero-command feedback for two seconds; it
aborts on 0.003 rad drift, raw overspeed, or estimated torque magnitude >0.1 Nm.
Its results are stored separately under `baseline_tests`. Away-only breakaway
now uses a direct fixed-cap ramp and encoder-derived speed. It stops on 0.003 rad
away displacement, 0.003 rad wrong-direction displacement, 0.02 rad maximum
travel, stale feedback/faults, timeout, torque magnitude >0.1 Nm immediately, or
magnitude above cap+0.02 Nm sustained for 0.15 s. Full homing retains its original
more conservative torque and raw-velocity checks.

The repeat ramp (0.002 Nm/s, 0.02 Nm cap) aborted at estimated torque +0.13385 Nm,
when the requested ramp was only about 0.011 Nm. Position did not reach the
breakaway criterion and returned to its starting value after confirmed disable.
No minimum torque, lower bound at the cap, or safe contact torque was established.
Do not increase the threshold based on these observations. Compare current and
estimated torque with command/wire traces before another powered ramp.

## Increased-limit away-only preset

At the operator's request, `src/tocatl_bringup/config/breakaway-leg3.yaml`
provides a separate hip breakaway preset: 0.05 Nm command cap, 0.2 Nm immediate
estimated-torque cutoff, and 0.15 Nm estimated torque sustained for 0.15 s.
The unchanged 0.002 Nm/s ramp needs 25 seconds to reach the cap, so this preset
allows 30 seconds total for the phase. It retains the 0.003 rad movement stop,
0.02 rad maximum travel, wrong-direction/encoder-speed/freshness/fault guards,
and selected-motor-only operation. These limits are not a measured safe load
rating. `allow_homing` remains false and no live settings are changed by editing
this preset. Normal full homing and baseline-mode torque checks remain unchanged.

The new ROS parameters are `homing.breakaway_hard_torque_nm` (default 0.1,
ceiling 0.3) and `homing.breakaway_sustained_torque_nm` (default zero, meaning
command cap plus 0.02). Away-only settings must satisfy command cap < sustained
cutoff <= immediate cutoff. The preset is for movement away from the stop and
must not be used to infer a suitable hard-stop contact torque.

## Sequential away-only command limits

The current breakaway preset uses `homing.breakaway_staged: true`. It ramps at
0.02 Nm/s through 0.05, 0.10, 0.20, 0.35 and 0.50 Nm command stages with one-second
holds. No movement advances to the next stage. It still stops on the 0.003 rad
movement criterion, 0.02 rad travel bound, wrong-direction/encoder-speed limits,
freshness/fault violations, 35 s phase timeout, or a fixed feedback limit (1.0 Nm
immediate; 0.75 Nm sustained for 0.15 s). It does not increase fault cutoffs after
a trip. `allow_homing` remains false in the saved preset. The earlier 0.05 Nm
preset values above are historical; the YAML is the current commissioning setup.
Staged mode is restricted to away-only trials, with command cap <=0.5 Nm and
feedback cutoff <=1.0 Nm. These are bounded commissioning limits, not validated
mechanical ratings, and must not be transferred to hard-stop approaches.

The operator subsequently requested a 1 Nm away-only trial. The current YAML
extends the stages through 0.75 and 1.00 Nm, allows 65 seconds, and uses 1.5 Nm
immediate / 1.25 Nm sustained feedback cutoffs. The staged-only code ceilings are
now 1 Nm commanded and 1.5 Nm reported torque; all non-staged mode ceilings are
unchanged. The first run reached the 0.003 rad movement criterion at about
0.794 Nm commanded. Most displacement relaxed after disable, so treat this as
an initial torque-for-displacement measurement, not validated free-motion or
hard-stop homing calibration. See the diagnostic report for recorded evidence.

The current operator-requested staged maximum is 2 Nm, adding 1.5 and 2.0 Nm
stages; the preset keeps 0.02 Nm/s and one-second holds, with 120 s phase timeout.
Away-only staged code ceilings are 2 Nm command and 2.5 Nm immediate feedback;
the preset's sustained cutoff is 2.25 Nm for 0.15 s. Non-staged full homing and
baseline limits remain unchanged. The repeat reached the movement criterion at
0.852 Nm and stopped, so no 2 Nm command was applied. Prior numerical preset
values in this document describe earlier commissioning runs, not the current YAML.

The latest operator-selected staged cap is 3 Nm, with the original 0.003 rad
movement stop explicitly retained. Current YAML adds 2.5 and 3.0 Nm stages,
uses 3.5 Nm immediate / 3.25 Nm sustained feedback cutoffs, 175 s phase and
200 s total timeouts. `homing.total_timeout_s` exposes the latter. Non-staged
limits are unchanged. The repeat stopped at 0.829 Nm commanded, before the cap,
and relaxed after disabling. This is a repeated small-displacement measurement,
not validation of hard-stop contact torque or sustained free motion.

The latest preset reflects the operator's 5 Nm cap and 1 degree away-only target.
`homing.breakaway_motion_rad` is independently configurable (default 0.003 rad;
preset 0.01745329), leaving baseline/contact tolerances unchanged. It must remain
below the 0.02 rad travel guard. Staged-only ceilings are now 5 Nm command and
5.5 Nm immediate feedback, with preset 5.25 Nm sustained feedback for 0.15 s;
phase/total timeouts are 280/300 s. The run reached 1.049 degrees at 0.978 Nm
commanded (1.027 Nm reported), then stopped and relaxed. No 5 Nm command was
applied. All previously described hard-stop/full-homing bounds are unchanged.

## Slow return and 5 degree commissioning status

The operator selected a slow return rather than releasing/holding at the target.
`homing.breakaway_return_slow: true` enables this; `homing.return_speed_rad_s`
defaults to 0.01 rad/s (0.57 degrees/s). The current preset targets 5 degrees,
with a 0.12 rad total-travel guard instead of the small-trial 0.02 rad guard.
The target is restricted to <=5.1 degrees in this mode. Normal full-homing and
baseline bounds remain unchanged.

On target, the worker keeps the supporting torque and follows a descending host
position reference. It never commands torque toward the stop. Torque changes
are limited to 0.2 Nm/s. Near the origin, it requires position within 0.002 rad
and low filtered encoder velocity for 0.5 s, then gradually unloads to zero.
Drift >0.005 rad while unloading aborts. Disable and result saving follow only
after successful unload. Results include `slow_return_completed` and
`returned_position_rad`. A fault still requests immediate motor stop; an automatic
return is not attempted using feedback that has failed a guard.

The trajectory controller checks 0.06 rad/s encoder-derived speed, 0.04 rad
tracking error, a 60 s trajectory timeout and a 0.005 rad lower-origin bound.
The outer telemetry, torque, temperature and travel checks continue to apply.
The preset's 280 s outbound phase plus up to 60 s return fits the updated 360 s
total timeout. For targets larger than 0.02 rad, initial displacement switches
from open-loop ramping to a slow outward position trajectory, then return.
The outward target tolerance is 0.001 rad.

Physical status: a 1 degree commissioning test completed its slow return and
confirmed disable near the starting pose. Two 5 degree attempts aborted during
the outward motion on encoder-derived speed (the second used trajectory control).
Neither completed 5 degrees or the return from that angle. Do not treat the larger
roundtrip as physically validated. The saved preset remains disabled; tuning the
outward controller against measured robot dynamics is required before claiming
successful 5 degree operation. 111 lowlevel tests cover gravity/elastic models,
turnaround support, unload, and guard failures; those tests do not replace the
missing larger-angle physical validation.

The operator subsequently requested an increased speed cutoff. The new
`homing.trajectory_overspeed_rad_s` parameter defaults to 0.06 rad/s; the current
preset sets 0.10 rad/s. Validation requires return reference speed < trajectory
cutoff <= the outer speed cutoff (0.12 rad/s by default). The commanded trajectory
rate remains 0.01 rad/s. The higher-cutoff physical retry aborted while lifting
at about 1.4 degrees on trajectory overspeed, confirmed disable, and did not
complete the 5 degree roundtrip. The 0.12 rad/s outer guard was retained. Further
outward-controller tuning is still required; no larger-angle return was validated.
112 lowlevel tests pass.


### Leg 1 active knee endstop search (2026-10-23)

`homing-leg1-knee.yaml` selects `active_endstop` for motor 13 on can1.
This mode measures three independent approaches: back off 5 degrees of motor
encoder travel, seek inward with native PD (kp 1.75, kd 2), and accept a contact
only after inward effort >=0.5 Nm and <=0.003 rad position span over 0.6 seconds.
The hand-set raw angle does not determine contact. Travel is bounded to
0.2753–1.4753 rad, measured effort to 3.5 Nm, encoder speed to 0.5 rad/s,
phase duration to 25 seconds and total duration to 90 seconds. Contact spread
must be <=0.01 rad. Software reference is saved only after three contacts and
confirmed disable; no motor zero is written. Three repeatable stalls alone
cannot distinguish an intended endstop from another physical obstruction.

The first active trial backed off from 0.8753 to 0.9665 rad, then sought inward.
Before a stable contact was accepted, hip 12 shifted from 0.7906 to 0.8169 rad,
exceeding the 0.01 rad hip-reference guard. The worker aborted and confirmed
both motors disabled. No knee home was recorded. A stationary hip is required
before retrying; do not relax the hip-reference guard to hide linkage movement.
Telemetry and raw CAN capture are in `log/homing-leg1-20261023/`.
Validation: 133 tests passed, including unknown-stop search with three actual
backoffs, nonrepeatable contact rejection, timeout and travel bounds.


### Powered hip holding during leg 1 knee homing

`homing-leg1-knee-held.yaml` selects `active_endstop_hip_hold`. This worker
owns motors 12 and 13 exclusively, prioritizes feedback for both, and rejects
freshness/fault/temperature violations on either. It preloads both drives before
motion, requires fresh enabled acknowledgements, and stops both on cancellation,
failure or completion. Stop transmission failures are retried for both; results
are saved only after fresh disabled acknowledgements from both.

Hip target is the current preflight pose (configured reference must match within
0.01 rad). Native hip gains are kp=150, kd=3, zero feedforward/velocity, with
3.5 Nm measured-effort cutoff, 0.5 rad/s encoder speed guard and 0.01 rad
position deviation guard. No extra pressure into the hip stop is requested.
The initial kp=40, kd=2 trial tripped the unchanged hip position guard near
1.5 Nm knee effort. Both motors disabled and no result was saved from that trial.

With kp=150, kd=3 the powered trial completed in about 26 seconds. The three
independent knee contacts were 0.7391088731, 0.7402532998 and 0.7394903487 rad;
median 0.7394903487 rad and spread 0.0011444266 rad (0.0656 degrees).
Hip target was 0.7932784009 rad. Logged hip displacement while enabled stayed
within 0.00725 rad of target. Contact transition efforts were about -2.2 Nm for
the knee and -1.0 to -1.2 Nm for the hip. Both motors were confirmed disabled
before saving. The readonly controller was restored after the trial.

This is a loaded endstop reference; the knee springs back toward 0.90 rad
when disabled. It depends on hip geometry and contact load, and should not be
interpreted as an unloaded joint angle. No device encoder zero was written;
global calibration remains invalid, including because hip's earlier three
samples were from a single contact rather than independent reapproaches.
137 tests pass, including paired cancellation, hip fault preflight, stop retry
on one drive while still stopping the other, and save-after-both-disable.


### Leg 1 ramped 15-degree encoder sweep (2026-10-23)

**Superseded physical interpretation:** the knee is mechanically coupled to hip motion. These encoder tracking results do not establish unconstrained hip travel. The fixed-knee sweep entry point is now blocked; use the hip-only/free-knee test described below.

`sweep-leg1.yaml` / `leg1_sweep` uses the paired enable/stop lifecycle and a separate result file. It preserves homing records and writes no device zero. Four-second quintic position ramps ease in/out, with 0.5 s stable settling within 1 degree. The knee sweeps first with the hip held at its homing reference, then parks 15 degrees away while the hip sweeps. The hip reference is restored before the final knee return. These are raw motor encoder angles; physical knee linkage conversion remains unverified.

Native gains are kp=150, kd=3, zero velocity/feedforward. Each virtual target is clamped to 2.5 Nm proportional effort; measured effort cutoff is 3.5 Nm, encoder-derived overspeed 0.5 rad/s, telemetry/fault freshness 0.25 s, temperature cutoff 45 C. All endpoints require fresh samples; failure to settle aborts instead of claiming success. Position envelopes are [home−0.05, home+15 degrees+0.15] rad to accommodate measured elastic relaxation.

The hardware trial completed all nine approach/sweep/return/parking endpoints in about 42 seconds. Both motors were confirmed disabled before saving `log/homing-leg1-20261023/sweep-result.json`; readonly telemetry was restored. Encoder traces are in `sweep-robot-state.jsonl`, with raw CAN appended to `can1-wire.jsonl`.

knee: outward encoder position 14.906 degrees above loaded home, return error 0.634 degrees; peak measured effort 1.839 Nm.

hip: outward encoder position 14.688 degrees above loaded home, return error 0.437 degrees; peak measured effort 1.424 Nm.

The return offsets are consistent with the different contact loads and observed compliance; a position-controlled return did not reproduce the loaded homing angle exactly. The knee relaxes away from home again after disabling; this test does not establish an unloaded joint zero or absolute calibration. Validation: 144 tests passed, including eased endpoints, a dynamic plant sweep, effort clamp, stalled-drive timeout and feedback guards.


The same ramped leg-1 sweep was repeated successfully without guard trips. All nine endpoints settled; both motors were confirmed disabled before saving `log/homing-leg1-20261023/sweep-repeat-result.json`. The first sweep result and home references were preserved. Readonly hardware telemetry was restored.


Manual hip-direction check: operator reports free physical hip rotation away from the stop. Recorded hip excursion was +32.48 encoder degrees from the disabled baseline; at peak hip travel the knee encoder changed in the opposite direction. This confirms positive hip encoder direction corresponds to physical travel away, but does not validate the earlier sweep's fixed knee encoder constraint or clearance to 45 degrees. Trace/result saved as `hip-manual-direction-*.json`. Readonly telemetry had crashed on integer temperature feedback; hardware ingestion now normalizes position, velocity, torque and temperature to floats. A ROS serialization regression test reproduces the integer-temperature case; 145 tests pass. Readonly telemetry restored and both motors confirmed disabled.

After readonly telemetry was restored, a further fresh disabled reading showed hip +47.36 degrees and knee −41.35 degrees relative to the manual baseline. The operator may still be moving the leg; these are observed encoder excursions, not a measured physical knee/hip angle calibration.


### Hip-only 45-degree sweep with passive knee (2026-10-23)

The operator confirmed that manually moving only the hip also rotates the knee
motor through the linkage. Fixed knee-encoder holding during hip motion is not
an appropriate independent hip test. `leg1_sweep` is now explicitly rejected.

`sweep-leg1-hip-free-knee.yaml` / `hip_free_knee_sweep` owns and commands only
hip motor 12 on can1. Knee 13 receives no mode, control, enable or stop command;
its state/fault feedback is prioritized alongside hip feedback and it must stay
disabled. Passive knee raw travel is monitored over [-0.1, 1.1] rad, covering
the observed manual check. Both motors' feedback/fault status must be fresh;
a knee fault or unexpected enable aborts the hip test.

The hip ramps through 5, 15, 30, 45 and 5 degrees above its saved loaded home.
Quintic segments last at least 4 seconds, scaled to nominal reference peak speed
0.15 rad/s. Native gains kp=150, kd=3, zero feedforward; proportional error is
clamped to 2.5 Nm, measured effort cutoff 3.5 Nm, observed encoder overspeed
0.5 rad/s, temperature cutoff 45 C. Each target requires stable feedback within
1 degree for 0.5 seconds; settling and total timeout abort instead of claiming
completion. Return stops at 5 degrees rather than reloading the physical stop.

The hardware trial reached encoder offsets 4.59, 14.69, 29.81, 44.92 and 4.79
degrees. Peak measured hip effort was 1.287 Nm and peak encoder-derived speed
0.287 rad/s. Knee 13 remained disabled in the recorded feedback. These readings
verify encoder trajectory only; visual confirmation of no bending/binding is
pending. Motors were confirmed disabled and readonly telemetry was restored.
Result: `log/homing-leg1-20261023/hip-free-knee-sweep-result.json`.
Homing records and device encoder zeros were unchanged. 149 tests passed,
including a dynamic hip ramp, stall rejection, passive-knee preflight and a
worker test proving all motor-write calls target only motor 12.


Hip-only repeat for operator visual observation: reached the 45-degree stage, but aborted during return when passive knee raw encoder exceeded the 1.1 rad upper travel guard (reported 1.16369 rad). Knee stayed disabled throughout. Both motors confirmed disabled; no completed sweep result or home reference was written. Fault/repeat states saved to `hip-free-knee-sweep-repeat-abort.json`; readonly hardware telemetry restored. Do not widen the passive range or claim the return completed based on this run.


Slow-return revision: final return reference peak speed reduced from 0.15 to 0.05 rad/s (~26 seconds for 40 degrees). Monitoring bounds and effort limits unchanged; knee remains disabled. An initial attempt aborted on passive knee feedback freshness immediately after enable. Startup now explicitly queries state/faults for all priority motors and requires their post-enable readings, without sending knee motion commands. The retry reached about 23 degrees but failed to settle at the outward 30-degree waypoint (~2.6 Nm measured hip effort), so the slower return was not exercised. Both motors disabled and readonly telemetry restored. No completed sweep/home result written. Abort details: `hip-free-knee-slow-return-abort.json`. Operator observation is required before retrying or increasing force. 150 tests passed.


### Leg 1 continuous 30-degree hip hold

`hold-leg1-hip-30.yaml` / `hip_position_hold` ramps only motor 12 to saved hip
home +30 encoder degrees and continues holding until cancelled or a guard trips.
Motor 13 remains disabled and is only queried for feedback. The quintic approach
has nominal peak speed <=0.12 rad/s and duration >=6 seconds. A stable 0.5 s
window within 1 degree changes status to `HOLDING_30_DEG`; no elapsed time cutoff
applies in that phase. A 3-degree tracking error lasting 1 second aborts.
Approach settling has an 8-second allowance after the ramp.

Transient measured effort cutoff 12 Nm, proportional budget 10.5 Nm, 4 Nm/s
proportional slew, native kp=150/kd=3. The initial 3.5 Nm/1-second sustained
effort trip was subsequently removed at the operator's explicit request.
Effort above 3.5 Nm alone no longer stops this hold. State/fault freshness remains
0.25 seconds; temperature cutoff 45 C and encoder-derived overspeed 0.5 rad/s.
Passive knee range monitoring [-0.1, 1.1] rad remains active. The prior high-load
CAN trace showed the hip ceased replying while the knee continued; its root
cause is unresolved, so freshness protection remains mandatory.

The actual hold reached about 29.8 degrees with ~0.45 Nm hip effort at 28.6 C,
with fresh fault-free feedback and knee disabled. Unlike completed sweeps, this
request intentionally leaves the hip enabled under local controller supervision.
Telemetry: `log/homing-leg1-20261023/hip-30-hold-state.jsonl`; startup confirmation
is a dated snapshot in `hip-30-hold-start-confirmation.json`, not live state.
No encoder zero or homing record is written by a hold.

To stop, request chat cancellation or call `/motors/enable` (std_srvs/SetBool)
with `data: false`, then verify fresh disabled feedback. Normal arming stays
unavailable in this diagnostic mode. 155 tests passed, including hold beyond
one hour of simulated elapsed time, sustained load rejection, stale feedback,
approach timeout, tracking drift and cancellation with hip-only motor writes.


Hold update: operator requested removal of the 3.5 Nm sustained-effort stop.
That guard and its config fields were removed. The controller was explicitly
cancelled and both motors confirmed disabled before restart; the hip was then
ramped back toward 30 degrees with the knee disabled. Existing 12 Nm transient
cutoff, 45 C temperature cutoff, 0.25 s state/fault freshness, encoder speed,
travel, settling and 3-degree tracking-error guards remain. A regression test
feeds 9 Nm feedback for 20 seconds without a sustained-effort trip and confirms
that >12 Nm still aborts. 155 tests passed. No device parameters or homing
references were changed.


Operator-requested source change: removed the absolute hip position-range trip (`home−0.02` through `home+34 degrees`) from `hip_hold.py`. The separate >3-degree target tracking error lasting 1 second remains, along with overspeed, effort, thermal, feedback/fault and passive-knee checks. 156 tests passed, including a brief 35-degree excursion without the removed range trip. No controller restart or hardware test was performed for this change; an already-running Python controller retains its loaded implementation until restarted.
