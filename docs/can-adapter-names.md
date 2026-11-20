# Stable CAN adapter names

TOCATL has three canable2 (gs_usb) adapters:

| Name | Legs | Motors |
|---|---|---|
| can0 | 1, 2 | 11–13, 21–23 |
| can1 | 3, 4 | 31–33, 41–43 |
| can2 | 5, 6 | 51–53, 61–63 |

Linux names CAN adapters in the order it finds them at boot, so the names can move. For
example, on 2026-11-02 the only adapter plugged in came up as `can1`. All three adapters are
the same USB device (`1d50:606f`), so only the **serial number** tells them apart. A boot script
pins each serial number to its name (section 3).

## 1. Detect a wrong name (already built in)

When the controller starts, it sends a read-only "get ID" request (comm type 0) to every
selected motor on every configured bus. It refuses to start if a motor answers only on a
different bus, for example `12 on can1 (expected can0)`. Motors that don't answer at all are
logged as a warning and left to the normal freshness checks. To turn the check off, set
`-p verify_can_wiring:=false`.

## 2. Remap for one run

Point a logical bus at a different interface:

```bash
-p can0_channel:=can1 -p can1_channel:=can0
```

## 3. Pinned names (installed on the Pi 2026-11-06)

| Name | Legs | Adapter serial |
|---|---|---|
| can0 | 1, 2 | `003A00665743500D20353341` |
| can1 | 3, 4 | `0035005C4633500E20303035` |
| can2 | 5, 6 | `0036006D4633500E20303035` |

Serials were matched to legs with a read-only get-ID probe on 2026-11-06 (only hip-yaw motors
connected). Before the fix, Linux had named the legs 5/6 adapter `can1` and the legs 3/4 adapter `can2`.

**A plain udev `NAME=` rule does not work here.** The kernel names all three adapters
`can0`–`can2` in the same instant. Renaming one to a name another adapter still holds fails with
`Failed to rename network interface … File exists`. That is what happened to the old
`88-canable.rules` from 2025, which is now disabled as `88-canable.rules.disabled-20261106`.

The fix uses three files in `scripts/udev/`:
- `70-tocatl-can.rules` starts the service whenever a gs_usb interface appears.
- `tocatl-can-names.service` waits 1 s for all adapters, then runs the script. It also runs at boot.
- `tocatl-can-names` renames in two passes, through temporary `tcan_canN` names, so no rename
  collides. It never renames an interface that is up.

Commands to install, update, check and extend it are in COMMANDS.md:
[B3. Adapter names](../COMMANDS.md#b3-adapter-names) and
[J1. CAN adapter names service](../COMMANDS.md#j1-can-adapter-names-service).

Bring the links up as described in COMMANDS.md section B. udev only sets the names; it doesn't
set the bitrate.
