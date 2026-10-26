# Motor 51 reconnection diagnosis — 2026-10-26

The operator reconnected motor 51 without changing its cable, power connection,
CAN ID or firmware. The previous lowlevel server was disarmed successfully and
stopped before probing. No enable, control, zero or parameter-write command was
sent by the diagnostic probes. The server was not restarted afterward; session
readiness from that process is invalid. Motor 51 remains excluded from the
checked-in production profiles.

## Current observations

| Read-only test on can1 | Duration | Requests | Missing replies / TX completions |
| --- | ---: | ---: | ---: |
| Known-good motor 12, state and faults | 10 s | 99 | 0 / 0 |
| Motor 51, state only | 10 s | 50 | 0 / 0 |
| Motor 51, fault register only | 10 s | 50 | 0 / 0 |
| All nine can1 motors including 51 | 30 s | 587 | 0 / 0 |
| Same inventory excluding 51 | 15 s | 293 | 0 / 0 |
| All nine including 51, faster paced requests | 60 s | 2,873 | 0 / 0 |

These request counts exclude the separate identity/voltage detail probe and
production-transport soak. Every received state response reported disabled and
no state faults; observed fault registers were zero. Across the table's tests,
51 answered 486 requests, with a maximum measured application receive latency
of 9.98 ms. Its position span was one encoder increment (0.0003815 rad), and
reported temperature ranged from 27.3 to 27.9 C.

Ten repeated identity requests to each of 12 and 51 returned one consistent
payload per motor, with different payloads between motors. Motor 51 returned
`468f303538363812`, and its CAN-ID register returned 51. This provides no positive
evidence of a duplicate ID, but does not conclusively rule one out.
Reported idle voltage was 22.068 V at 51 and 22.081–22.094 V at 12, using the
float voltage register, cross-checked against the millivolt register. This is
an idle snapshot, not a supply-transient measurement during actuation.

Probes explicitly received local send echoes to distinguish an unanswered
request from missing host TX completion, and stopped sending if completion
was absent for 0.25 seconds. The Linux documentation describes this echo
mechanism and its dependence on driver support:
[SocketCAN returned message flags](https://docs.kernel.org/networking/can.html#returned-message-flags).
On this adapter all requests in the table produced completion echoes. No CAN
error frames were received. Interface counters must be interpreted separately;
zero reported CAN errors alone cannot certify physical signal integrity.

## Production transport check

The actual HardwareTelemetry transport also passed 30 seconds of full can1
inventory polling and 30 seconds with four priority motors plus round-robin
background polling, including 51. No transport/fault/mode/freshness failures
occurred. Inventory polling delivered 60 state updates per motor. In the
priority phase, the four selected motors delivered 281–282 updates each;
maximum selected state/fault age was 0.1148 s, below the 0.25 s guard. Background
motor 51 delivered 57 observed updates, with maximum state/fault age 0.5454 s,
within the background 1.5 s bound. This was disabled telemetry only, without
actuation command traffic.

The interface remained ERROR-ACTIVE, with no increase in reported bus errors or
TX drops (90 historical drops before and after). No interface reset was needed.
The adapter serial remained 0035005C4633500E20303035, matching the earlier report.

## Assessment

The previously recorded failure is real: the 2026-10-25 comparison reproduced
transmit blockage after reintroducing traffic to 51, and 51 then answered only
8/39 state and 5/39 fault requests. Today's tests did not reproduce it.

There is currently insufficient evidence to condemn or clear the motor. A
reseated/intermittent connector or power/CAN path is plausible given that only
reconnection changed, but motor electronics, duplicate addressing and adapter
behavior remain alternatives. No firmware update, ID change, permanent
configuration change, wiring swap or loaded test was performed.

If the fault recurs, the next discriminating physical test is to power off,
move the suspect motor to a known-good cable/power/CAN connection, and test a
known-good motor on the suspect connection. Preserve the established bus
termination and unique addresses. Repeat the same read-only probe after each
change. A fault that follows the motor across known-good connections is evidence
for motor/drive repair or replacement. A fault that stays with the connection
points toward its wiring/power/bus path. If it follows neither, compare the USB
adapter/host and firmware only after recording the failing setup. Physical
changes require operator execution; none were attempted remotely.

Raw frames, per-phase counts, voltage/identity responses, production snapshots,
interface counters and the exact probe programs are saved under
`log/motor51-diagnosis-20261026/`.
