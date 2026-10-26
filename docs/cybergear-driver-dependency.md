# CyberGearDriver Dependency Decision

Date: 2026-07-10

## Decision

Do **not** vendor the recovered `CyberGearDriver` source into this repository
right now.

Use the upstream package as a pinned optional Pi dependency instead:

```text
CyberGearDriver==0.2.0
python-can==4.6.1
```

Those live-CAN dependencies are listed in
[`requirements-pi-live.txt`](../requirements-pi-live.txt). They are not needed
for laptop dry runs, unit tests, calibration validation, legacy geometry/gait
reference work, or schema-only telemetry output.

## Why not vendor it?

The recovered Pi package source is small and useful, but vendoring it would make
TOCATL responsible for maintaining a motor-driver fork that exposes dangerous
motion APIs. The upstream package is already public, versioned, and MIT
licensed, so pinning the dependency is cleaner than copying site-packages source
into `legacy/` or `src/`.

Keep the recovered archive as evidence and fallback:

```text
TOCATL Spiderbot/05_Testing + Media/code-recovery/tocatl-driver-recovery-20260710-2008.tar.gz
```

That archive confirmed the Pi was using `CyberGearDriver` version `0.2.0` by
Jeremy Gillick, installed inside:

```text
/home/tocatl/Desktop/dev/romela_tocatl/.venv/lib/python3.12/site-packages/CyberGearDriver
```

## Upstream evidence

The upstream `jgillick/python-cybergear` `pyproject.toml` identifies:

- package name: `CyberGearDriver`
- version: `0.2.0`
- author: Jeremy Gillick
- Python requirement: `>=3.8`
- license: `MIT`
- homepage: `https://github.com/jgillick/python-cybergear`

The upstream license file is MIT. If upstream disappears or TOCATL must patch
driver behavior, the recovered archive can support a future vendored fork, but
that should be a deliberate PR with the MIT license preserved.

## Safety boundary

The driver exposes methods that must not be called casually:

- `enable()`
- `control(...)`
- `set_zero_position()`
- `change_motor_id(...)`
- `set_parameter(...)`

All TOCATL code should keep using
[`tocatl_lowlevel.cybergear_adapter`](../src/tocatl_lowlevel/tocatl_lowlevel/cybergear_adapter.py)
as the boundary. The current live telemetry helper is intentionally read-only:

- `request_motor_state()`
- `request_motor_fault_status()`

Do not let ROS nodes, gait planners, calibration tools, or one-off scripts
instantiate raw `CyberGearMotor` objects directly unless they are inside a
reviewed adapter/command-sender module with safety-state and calibration gates.

## Compatibility notes from the recovered source

The recovered package exports the names imported by the legacy deployed scripts:

- `CyberGearMotor`
- `RunMode`
- `CyberMotorMessage`

The driver's state parser decodes:

- position
- velocity
- torque
- temperature
- fault bits such as over/under-voltage, over-temperature and over-current

One source wart is worth remembering: `CyberGearDriver/state.py` defines the
state tuple for torque as `(T_MIN, T_MIN)`, while `motor_controller.py` decodes
torque using `(T_MIN, T_MAX)`. This appears to be an unused typo, but it is a
good reason to keep adapter tests around our normalized telemetry instead of
trusting driver internals blindly.

## Install on the Pi

From the project root on the Pi:

```bash
cd ~/Desktop/dev/romela_tocatl
. .venv/bin/activate
python -m pip install -r requirements-pi-live.txt
```

Then run the normal dry-run gate before any live CAN read:

```bash
bash scripts/pi_dry_run_checks.sh
```

Live telemetry still belongs after the physical protected-harness/no-motion gate
passes:

```bash
PYTHONPATH=src/tocatl_lowlevel:src/tocatl_diagnostics \
  python3 -m tocatl_diagnostics.cybergear_telemetry \
  --live --interface socketcan --channel can0 --bus can0 \
  --csv telemetry-can0.csv
```

Repeat separately for `can1`.
