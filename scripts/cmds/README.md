# Operator shortcuts (`scripts/cmds/`)

One-line launchers for the Pi. They live in the repo, so every `git pull` updates them.
Run them from any directory as `~/tocatl/scripts/cmds/<name>.sh`. COMMANDS.md has the full,
long-form commands they wrap.

**Before any script that opens CAN or moves a motor:**
1. Robot supported, legs clear.
2. Motor-power cutoff in reach.
3. Fused harness OK.
4. No other process owns the bus.
5. Out-of-service motors identified (none known since 2026-11-06).

Only one process may own the CAN buses at a time. Stop a controller with Ctrl-C before
starting another, or before `can-scan.sh`.

| Script | Opens CAN | Moves motors | Needs |
|---|---|---|---|
| [`can-scan.sh`](#can-scansh) | yes (get-ID only) | no | buses UP, nothing else running |
| [`level1.sh`](#level1sh) | yes | no | buses UP |
| [`state.sh`](#statesh) | no | no | a running controller |
| [`level3-perform.sh`](#level3-performsh) | yes | when armed | robot supported, every foot clear of the ground |
| [`perform.sh`](#performsh) | no | via the controller | Level 3 controller, all legs homed |
| [`level4-walk.sh`](#level4-walksh) | yes | when armed | robot supported for first tests |
| [`teleop.sh`](#teleopsh) | no | via the walker/performer | Xbox pad connected, `tocatl` in the `input` group |

## Typical sessions

**After a reboot, check the wiring:**
```bash
for b in can0 can1 can2; do sudo ip link set $b down; sudo ip link set $b type can bitrate 1000000; sudo ip link set $b up; done
~/tocatl/scripts/cmds/can-scan.sh
```

**Feet-in-air performance from the pad**, with three terminals:
```bash
~/tocatl/scripts/cmds/level3-perform.sh      # T1: controller; then home every leg (COMMANDS.md D)
~/tocatl/scripts/cmds/perform.sh --dry-run   # T2: check every cue is ok, then run it without --dry-run
~/tocatl/scripts/cmds/teleop.sh              # T4: the pad
```

**Walking from the pad:**
```bash
~/tocatl/scripts/cmds/level4-walk.sh         # T1: controller; home, declare yaw (COMMANDS.md D)
ros2 run tocatl_kinematics tocatl_walk --gait ripple   # T2: the walker (COMMANDS.md E6)
~/tocatl/scripts/cmds/teleop.sh              # T4: the pad
```

---

## `can-scan.sh`
Lists which CyberGear motors answer on which CAN bus. It's **read-only**: it sends only comm
type 0 (get ID) to IDs 1–127 on each bus, with no enable, mode, parameter or zero frames.

```bash
~/tocatl/scripts/cmds/can-scan.sh            # all three buses
~/tocatl/scripts/cmds/can-scan.sh can2       # one bus
```

Needs the buses UP (COMMANDS.md B). It refuses to run while a controller, the bench, or a
player/walker/performer is running.

Example output:
```
can0: answered [11, 12, 13, 21, 22, 23]
      missing none of [11, 12, 13, 21, 22, 23]
can1: answered [31, 41]
      missing [32, 33, 42, 43] of [31, 32, 33, 41, 42, 43]
Not answering anywhere: [32, 33, 42, 43]
```

| Line | Meaning |
|---|---|
| `answered` | motors that replied on this bus |
| `missing` | motors the motor map expects on this bus that did not reply: unpowered, unplugged or a wrong ID |
| `ON THE WRONG BUS` | a motor replied on a bus other than its mapped one: a cable is in the wrong chain, or the adapter names swapped (see [CAN adapter names](#can-adapter-names-pinned)) |
| `send failed … nothing acknowledges frames` | no powered motor on that bus at all: check motor power, CAN-H/L wiring and termination (about 60 Ω across H/L with power off) |

IDs outside 11–63 (for example a replacement motor still on factory ID 127) also show up
under `answered`.

The motor map is in `src/tocatl_lowlevel/tocatl_lowlevel/motor_map.py`: ID = leg × 10 + joint
(1 hip yaw, 2 hip pitch, 3 knee).

| Bus | Legs | Motors |
|---|---|---|
| can0 | 1, 2 | 11–13, 21–23 |
| can1 | 3, 4 | 31–33, 41–43 |
| can2 | 5, 6 | 51–53, 61–63 |

## `level1.sh`
Starts a Level 1 controller: **read-only telemetry** for all 18 motors on all three buses.
Motion and homing are off.
- At startup it runs the wiring check (`verify_can_wiring`): a get-ID per motor. It **refuses
  to start** if a motor answers on the wrong bus, and warns about silent motors.
- Then it polls every motor and publishes `/robot_state`.
- Writes a run folder `~/tocatl-runs/cli-<UTC time>/` with `controller.log`.
- Extra controller parameters go after the script, e.g. `level1.sh -p can_transport:=v2`.
- Stop it with Ctrl-C.

## `state.sh`
Prints one snapshot from a running controller, opening no CAN itself:
- the safety state and its reason
- per motor: ID, bus, whether telemetry is valid, and position
- CAN counters

## `level3-perform.sh`
Starts a Level 3 controller for feet-in-air motion: hip pitch and knee of all six legs,
yaws unpowered.
- **Profile:** `homing-hip-knee.yaml` and `motion-feet-air.yaml`.
- **Limits:** kp ≤ 30, kd ≤ 2, 1.5 Nm drive torque cap, 5 rad/s.
- **Homing and motion are allowed for this run only.** The committed YAML keeps both off.
- **The robot must be supported with every foot clear of the ground.**
- **Next:** home every leg (COMMANDS.md D), then run `perform.sh` (or `tocatl_play`, COMMANDS.md E1–E4).
- Writes a run folder like `level1.sh`. Extra parameters go after the script.

## `perform.sh`
Runs `tocatl_perform`: artistic leg cues played on demand, with the feet in the air.
- Needs `level3-perform.sh` running with every leg homed and READY.
- Plans every cue once at startup, from a fixed ready pose (feet at r 0.32 m, z +0.02 m in each
  leg's plane). Each cue is checked against joint limits, the controller envelopes and motor
  speed. It prints a JSON report, drops any cue that fails, and refuses to start if none pass.
- Holds the measured pose until armed. Arming (pad: Start held 1 s) unfolds the legs to the
  ready pose. Each cue starts and ends there.

```bash
~/tocatl/scripts/cmds/perform.sh --dry-run      # plan and report only
~/tocatl/scripts/cmds/perform.sh --speed 0.5    # slower: peak raw motor rad/s (default 1.0, cap 5)
```

| Cue | Pad (LB held) | Motion |
|---|---|---|
| `wave` | A | small circles that ripple around the body, leg by leg |
| `bloom` | Y | all feet reach out and up, hold 1.5 s, return |
| `circle` | RB | all feet circle together |
| `alternate` | left-stick click | the two tripods (1,3,5 / 2,4,6) circle in antiphase |
| `wave_reverse` | right-stick click | the ripple runs the other way round |
| `reach` | (no button) | feet point down: `ros2 topic pub --once /perform/cue std_msgs/msg/String "{data: reach}"` |

- **B (`walk/stop`):** after the current cue, the legs fold back along the reversed unfold
  path, then it requests a disarm.
- **Back / Xbox button:** disarm now; the legs go limp.
- A disarm or fault returns it to rest. The next arm unfolds again from wherever the legs
  are, re-planned while they were disarmed.
- A cue that arrives while another is playing is ignored and logged.
- Never run it together with `tocatl_walk`: both would command the same motors.

## `level4-walk.sh`
Starts a Level 4 controller for standing and walking: all 18 motors.
- **Profile:** `homing-hip-knee.yaml` and `motion-standing.yaml`.
- **Limits:** 12 Nm cap, kp ≤ 100, 5 rad/s.
- **Fault response:** `fault_response:=soften`. On a fault it holds the last targets and ramps
  kp to zero over 3 s, instead of dropping the robot.
- **Next:** home (tripods, or one leg at a time until home-all is tested on the robot), declare
  yaw with every leg radial, then `tocatl_walk` (COMMANDS.md E6) and `teleop.sh` (E7).
- Writes a run folder like `level1.sh`.
- To get the old behaviour, pass `-p fault_response:=disable`.

## `teleop.sh`
Starts `joy_node` (reads the pad and publishes `/joy`) and `tocatl_teleop` (turns `/joy` into
walk and perform commands). Ctrl-C stops both.

| Pad | Walking (`tocatl_walk`) | Performing (`tocatl_perform`) |
|---|---|---|
| **LB held** | **deadman**: velocity only while held | required for every cue button |
| Left stick | walk in any direction | — |
| Right stick left/right | turn | — |
| Right trigger | speed 30–100% | — |
| Start held 1 s | arm: rise, then walk | arm: unfold to the ready pose |
| B | settle, lower, disarm | fold back after the cue, disarm |
| X | next gait (ripple → tetrapod → tripod) | — |
| D-pad up/down | body height ±1 cm | — |
| A / Y / RB / stick clicks (with LB) | — | cues (see `perform.sh`) |
| Back / Xbox button | **disarm now** | **disarm now** |

- If `/joy` stops for 0.25 s (pad out of range or off), velocity goes to zero and the walker
  settles.
- Button and axis numbers are ROS parameters of `tocatl_teleop` (`button_*`, `axis_*`).
- **Recorded layout** (2026-11-07, Bluetooth, `xpadneo`): LB 4, Start 7, B 1, X 2, Back 6, D-pad
  up/down axis 8, left stick axes 1/0, right stick axis 3, RT axis 5.
- **Not yet recorded:** A 0, Y 3, RB 5 and the stick clicks 9/10 follow the standard SDL
  numbering. Check them with `ros2 topic echo /joy`.
- **Over USB** (`xpad`), the D-pad up/down is axis 7: run `teleop.sh -p axis_dpad_y:=7`.

---

## Setup notes (one-time, already done on the Pi)

### CAN adapter names, pinned
Linux names the three canable2 adapters `can0`–`can2` in the order it finds them at boot. On
this Pi that order puts the legs 5/6 adapter first, so without the fix its legs would land on
the wrong bus. Since 2026-11-06, a boot service renames each adapter by its USB serial:

| Name | Legs | Adapter serial |
|---|---|---|
| can0 | 1, 2 | `003A00665743500D20353341` |
| can1 | 3, 4 | `0035005C4633500E20303035` |
| can2 | 5, 6 | `0036006D4633500E20303035` |

**Files** (source in `scripts/udev/`):
- `/etc/udev/rules.d/70-tocatl-can.rules`
- `/etc/systemd/system/tocatl-can-names.service` (enabled)
- `/usr/local/sbin/tocatl-can-names`

**Why a service, not a udev rule:** a plain udev `NAME=` rule fails here with "File exists",
because all three adapters appear at once with `can0`–`can2` already taken. The script
renames in two passes through temporary names, and never renames a link that is UP.

**Verified twice:** after a simulated replug of all three adapters, and after a real reboot on
2026-11-07.

**Logs:** `journalctl -t tocatl-can-names`.

**Adding or replacing an adapter:** read its serial with
`cat /sys/class/net/canN/device/../serial`, edit `MAP` in the script, and reinstall it
(`docs/can-adapter-names.md`).

The old `88-canable.rules` is disabled as `88-canable.rules.disabled-20261106`.

### Xbox controller
Paired 2026-11-07: Xbox Series pad `AC:8E:BD:64:DD:20` (USB ID 045E:0B13, driver `xpadneo`).

- **State:** bonded and trusted, so pressing the Xbox button reconnects it, including after a
  Pi reboot.
- **Check it:** `bluetoothctl info AC:8E:BD:64:DD:20`. You want `Paired`, `Bonded`, `Trusted`
  and `Connected` all `yes`, and `ls /dev/input/js0` to exist.
- **Reading it over SSH:** `tocatl` is in the `input` group. Without that, `joy_node` (SDL2)
  cannot read `/dev/input/event*` over SSH and silently publishes nothing.

**Pairing a new pad:**
1. Run `bluetoothctl`, then `agent NoInputNoOutput`, `default-agent` and `scan on`.
2. Hold the pad's pair button until the logo flashes fast.
3. When it appears, run `pair <MAC>`, `trust <MAC>` and `connect <MAC>`.

Without the agent, pairing fails with `AuthenticationCanceled`. The pad may still connect,
but nothing is bonded, so it won't reconnect by itself.

Plugging the pad in by USB does **not** pair it for Bluetooth.

Another pad, `40:8E:2C:2A:6D:84`, is also paired from before.

### Motor 51
Back in service (Pranav, 2026-11-06). It answered get-ID on the legs 5/6 adapter, and has been
in every profile since 2026-10-27.

The old can1 ENOBUFS freeze it was linked to (`docs/can-debug-2026-10-25.md`) has not recurred
in a scan, but a full Level 1 poll of the 5/6 chain is the real test.
