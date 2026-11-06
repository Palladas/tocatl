# Stable CAN adapter names

TOCATL has three canable2 (gs_usb) adapters:

| Name | Legs | Motors |
|---|---|---|
| can0 | 1, 2 | 11–13, 21–23 |
| can1 | 3, 4 | 31–33, 41–43 |
| can2 | 5, 6 | 51–53, 61–63 |

Linux names CAN adapters in the order it finds them at boot, so the names can move. For
example, on 2026-11-02 the only adapter plugged in came up as `can1`. All three adapters are
the same USB device (`1d50:606f`), so only the **serial number** tells them apart. A udev rule
can pin each serial number to its name.

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

## 3. Pin the names with udev (run once on the Pi)

Plug in **one adapter at a time**. For each one, note which chain it's wired to, label it, and
read its serial number:

```bash
for n in /sys/class/net/can*; do echo "$(basename $n): $(udevadm info -q property -p $n | grep ID_SERIAL_SHORT)"; done
```

Write `/etc/udev/rules.d/70-tocatl-can.rules` with your three serial numbers in place of the
placeholders:

```
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="SERIAL_FOR_LEGS_1_2", NAME="can0"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="SERIAL_FOR_LEGS_3_4", NAME="can1"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="SERIAL_FOR_LEGS_5_6", NAME="can2"
```

Apply it by running `sudo udevadm control --reload` and then replugging the adapters, or by
rebooting. Check the result with `ip -br link | grep can`. After that, start a Level 1
controller: the wiring check should list every leg on its expected bus.

Bring the links up as described in COMMANDS.md section B. udev only sets the names; it doesn't
set the bitrate.
