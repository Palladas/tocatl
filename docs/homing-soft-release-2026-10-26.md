# Reduced contact load and gradual release

Hip/knee homing now uses a 1.5 Nm verified runtime drive cap (previously
2.625 Nm), a 1.0 Nm stable-contact threshold (previously 2.3 Nm), and a
spring-error lead capped to the new command budget. Final contact starts a
three-second UNLOADING phase: spring error ramps to zero for 1.5 seconds with
native damping retained, then both drives' impedance fades to zero for 1.5
seconds. Loaded home samples are preserved. Fault/cancel stops remain immediate;
feedback, effort, encoder-speed, hip drift, thermal and timeout guards remain.
This reduces commanded load; it is not an independent torque calibration.
Legacy single-joint profiles retain their existing behavior.

A leg-1-only powered trial passed a ten-second read-only preflight, acquired
both contacts, completed both unloads and confirmed both motors disabled and
fault-free. Both original 12 Nm runtime limits were restored. The test process
then closed; its session READY state does not persist into another controller.
The operator did not observe the motion, so visual improvement remains unverified.
No all-leg powered validation was performed.

Recorded loaded hip/knee contacts were -0.0268940261 / 1.5791180285 rad,
respectively. These differ from the historical profile references by about
-0.8297 / +0.8396 rad. Normal homing records these differences rather than gating
on historical references. Stable contact alone cannot prove the physical stop;
confirm the intended stop visually before treating these as new calibration.
No checked-in reference or device zero was changed.

Trial telemetry, test output and separate result files:
`log/homing-soft-release-20261026/`.
Offline validation with ROS Jazzy sourced: 93 lowlevel tests passed, including
both release directions, monotonic spring unloading, zero final gains and
retained stale-feedback/effort/speed rejection.

## Opposite-leg batch revision

The subsequent operator-requested profile lowers the verified drive cap to
1.0 Nm on both motors and the inward stable-contact threshold to 0.7 Nm.
The spring lead is limited to 1.0/kp rad. The three-second release is retained.
These supersede the earlier trial settings above; that trial did not validate
the new cap.

Batch homing groups selected opposite legs: (1,4), (2,5), (3,6) for the default
selection. Each leg homes hip then knee independently. Partial selections never
add an unrequested opposite. A coordinator retains telemetry priority for all
four motors until both leg workers finish, including stop recovery. Workers
accept enabled feedback only from their owned pair and its scheduled opposite;
unrelated enabled motors still reject preflight. CAN writes remain serialized.
The existing parent lock serializes combined result-file updates and readiness.

A worker fault cancels its partner immediately, including if the failed worker
is still retrying stop. Later groups do not start; readiness from earlier groups
is preserved. Both active legs are invalidated on cancellation/failure. Each
leg retains its finite motion deadline, freshness, speed, temperature and
measured-effort guards and confirmed-disable/torque-limit restoration rules.
The 0.25-second fault freshness threshold is unchanged. The prior all-motor
fault freshness incident has not been established as resolved by this change.

Validation: 99 offline lowlevel tests pass with warnings treated as errors.
New cases prove actual overlap for each opposite pair, group order, partial
selections, persistence of both concurrent results, partner cancellation and
stop recovery before acknowledgement is available. No powered parallel test
or hardware test of the 1 Nm profile was initiated for this revision. Restart
the lowlevel server to load it; service names and launch command are unchanged.

## Restored 1.5 Nm profile

The operator subsequently requested restoring the 1.5 Nm verified drive cap.
The stable-contact threshold returns to 1.0 Nm and spring lead to 1.5/kp rad.
Opposite-leg concurrency and the three-second release remain enabled.
Restart the server to load these settings. No powered test was initiated for
this restoration.
