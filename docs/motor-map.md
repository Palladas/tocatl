# TOCATL Motor and CAN Map

## Original leg 1 hip reconnected — 2026-10-25

After the operator reported reconnecting the original hip motor, a read-only
state-request scan of IDs 1–127 on `can1` found 11, 12, 13, 51, 52, 53,
61, 62, and 63. IDs 27 and 127 did not respond. ID 12 reported disabled,
fault-free feedback at approximately 0.795186 rad; knee 13 reported disabled,
fault-free feedback near 0.876 rad. The existing leg 1 hip-pitch mapping
(logical 12 to physical 12) already matches the responding hardware, including
the bench profiles' `leg1_hip_can_id: 12`. No CAN ID write, zero write, or
motion command was needed or sent. This scan confirms responding addresses;
physical identity relies on the operator's reconnection report and prior mapping.

This map is recovered from the controller deployed on the Raspberry Pi on
2026-03-15. It is an evidence-based starting point, not yet a physical wiring
verification.

| Leg | CAN bus | Hip yaw | Hip pitch | Knee |
|---:|:---:|---:|---:|---:|
| 1 | `can1` | 11 | 12 | 13 |
| 2 | `can0` | 21 | 22 | 23 |
| 3 | `can0` | 31 | 32 | 33 |
| 4 | `can0` | 41 | 42 | 43 |
| 5 | `can1` | 51 | 52 | 53 |
| 6 | `can1` | 61 | 62 | 63 |

- Bus bitrate: 1,000,000 bit/s.
- Motor family: Xiaomi CyberGear, based on the motor label and deployed driver.
- IDs ending in 1, 2, and 3 represent hip yaw, hip pitch, and knee.

## Leg 3 manual identification — 2026-10-21

Observed live hardware `/robot_state` while the operator manually moved leg 3;
hardware motion was disabled and feedback was valid and fresh.

- Knee: motor **33**, `can0`. The knee-only movement changed its raw encoder
  approximately +0.824 rad while motors 31 and 32 remained stationary.
- Hip pitch: motor **32**, `can0`. The first hip movement changed its encoder
  approximately +0.810 rad. A repeat included a hip excursion of approximately
  -0.308 rad with motor 33 changing less than 0.001 rad, supporting identification
  independently of the knee. Motor 31 stayed stationary during the repeat.
- Hip yaw/base rotation (31): not physically tested in this session.

These observations identify motors; they do not establish joint zero offsets,
positive joint directions, transmission ratios, or homing limits. Some manual
repositioning moved both pitch and knee encoders; this alone does not establish
mechanical coupling.

Run the offline-only inventory with:

```bash
PYTHONPATH=src/tocatl_diagnostics python3 -m tocatl_diagnostics.motor_map
```

The module has no CAN dependency and cannot send, enable, zero, or command a
motor. Any future live telemetry command must be a separate, explicitly reviewed
tool because even a state request transmits a CAN frame.


## Leg 1 setup — 2026-10-23

The operator selected leg 1 for subsequent homing work and confirmed hip/knee at
their physical home stops. IDs 12 (hip pitch) and 13 (knee) on can1 returned fresh,
fault-free telemetry and reported disabled. Observed raw stop references were
approximately 0.80281529 and 0.87529564 rad respectively, recorded in
`config/leg1-home-reference-2026-10-23.json`. These are declared-pose observations,
not a completed three-contact calibration or physical ID/direction verification.
See [legacy homing reference](legacy-homing-reference.md) for original gains and
commands. No leg 1 motion or zero-writing was performed during this setup.

The subsequent manual hip backoff confirmed motor 12 as leg 1 hip pitch. Its raw
encoder increased away from the stop (initial isolated portion near 0.890 rad,
compared with the 0.802815 rad stop reference), so its home approach is negative
encoder direction. Knee readings changed later in the manipulation; that does
not independently verify knee direction. Reference metadata records the hip
identification without changing the original stop observation or device zero.

Manual knee backoff then confirmed motor 13 as leg 1 knee. Its raw encoder
increased to approximately 1.348325 rad while hip motor 12 remained stable at
0.802815 rad during the isolated portion. Knee home approach is negative encoder
direction toward the observed 0.875296 rad stop at that hip pose. Both motors
reported disabled with fresh fault-free feedback. Thus both pitch/knee directions
are physically checked for leg 1; no powered homing or zero write has occurred.


Leg 1 linkage correction: operator confirmed on 2026-10-23 that moving only the hip mechanically rotates both hip and knee motors. A fixed knee encoder during a hip sweep can constrain this linkage. The hip-only diagnostic leaves knee motor 13 disabled and monitors its passive motion. Coordinated joint control still needs a verified linkage transform; encoder travel alone does not verify rigid physical joint rotation.


## Leg 5 yaw excluded — 2026-10-25

The operator chose to leave leg 5 yaw (ID 51) unconnected after read-only polling
associated traffic to that ID with transmit blockage. The hardware profile now
excludes 51 from both `telemetry_motor_ids` and `actuation_motor_ids`. The leg 1
pair profile already limits both lists to 12/13. The canonical inventory retains
51 so its identity is preserved; exclusion does not imply that it is homed,
healthy or ready. Physical disconnection is an operator action, not verified by
this configuration change. Change wiring only with motor power off and preserve
CAN termination. See [CAN investigation](can-debug-2026-10-25.md).


## Manual hip/knee stop references — 2026-10-25

The operator placed legs 2–6 at their endstops. A repeated full-stop capture
recorded eight distinct stable samples per motor, with all ten drives disabled
and fault-free. Home directions were established from operator-declared backoff
poses; every remaining hip/knee motor homes in the negative encoder direction.

| Leg | Hip pitch stop (raw rad) | Knee stop (raw rad) | Hip/knee home direction |
| --- | ---: | ---: | --- |
| 2 | -1.6466392004 | 0.0524528878 | -1 / -1 |
| 3 | -1.5510795758 | 0.0123979553 | -1 / -1 |
| 4 | -1.6397726406 | 0.0711451896 | -1 / -1 |
| 5 | -0.0368123903 | -0.0200274662 | -1 / -1 |
| 6 | -1.6325246052 | 0.0463492790 | -1 / -1 |

Earlier partial captures were superseded after the operator clarified the final
hip-endstop pose. Motor 32 initially gave no replies, including during a can0
ID scan, but later recovered fresh feedback after the operator's power cycling/
repositioning. A separate leg-3 hip backoff increased its encoder from -1.551080
to -0.839055 rad, confirming its negative home direction. A specific cause of
its earlier disappearance was not established.

These are operator-declared unloaded stop positions, not loaded contact
calibration. No enable, motion, zero or ID write was performed by the assistant
in this capture. No leg was marked ready. The all-leg YAML now contains the
values; restart the running controller to load them. Its existing parameter
objects do not reload edits to the YAML file automatically. Powered home trials
still need to confirm the loaded contact locations against these references.

Reference artifact: `config/manual-stop-references-2026-10-25.json`.
Capture evidence: `log/all-leg-references-20261025/`.
