"""Which CyberGear motors answer on which CAN bus: read-only.

Sends only comm type 0 (get ID) to IDs 1-127 on each bus and lists who replies. No enable,
mode, parameter or zero frames. Run with no controller or bench owning the buses.
Usage: .venv/bin/python scripts/can_scan.py [can0 can1 can2]
"""
import sys
import time

import can

EXPECT = {1: 'can0', 2: 'can0', 3: 'can1', 4: 'can1', 5: 'can2', 6: 'can2'}   # motor_map.py
MOTORS = [leg * 10 + joint for leg in range(1, 7) for joint in (1, 2, 3)]
GET_ID_REPLY = 0xFE


def scan(name):
    """(motor IDs that answered, error text)."""
    try:
        bus = can.Bus(interface='socketcan', channel=name)
    except Exception as error:
        return set(), f'cannot open: {error}'
    found, error = set(), ''

    def drain(until):
        while time.monotonic() < until:
            msg = bus.recv(timeout=0.01)
            if (msg and msg.is_extended_id and (msg.arbitration_id >> 24) & 0x1F == 0
                    and msg.arbitration_id & 0xFF == GET_ID_REPLY):
                found.add((msg.arbitration_id >> 8) & 0xFF)
    try:
        for motor_id in range(1, 128):
            try:
                bus.send(can.Message(arbitration_id=motor_id, data=bytes(8), is_extended_id=True), timeout=0.05)
            except can.CanError as send_error:
                error = (f'send failed at ID {motor_id} ({send_error}): nothing acknowledges frames; '
                         'check motor power, CAN-H/L wiring and termination')
                break
            drain(time.monotonic() + 0.002)
        drain(time.monotonic() + 0.3)
    finally:
        bus.shutdown()
    return found, error


def main(buses):
    seen = {}
    for name in buses:
        ids, error = scan(name)
        seen[name] = ids
        expected = sorted(m for m in MOTORS if EXPECT[m // 10] == name)
        wrong = sorted(m for m in ids if m in MOTORS and EXPECT[m // 10] != name)
        print(f'{name}: answered {sorted(ids) or "nothing"}')
        print(f'      missing {sorted(set(expected) - ids) or "none"} of {expected}')
        if wrong:
            print(f'      ON THE WRONG BUS (map expects elsewhere): {wrong}')
        if error:
            print(f'      {error}')
    everywhere = set().union(*seen.values())
    print(f'Not answering anywhere: {sorted(set(MOTORS) - everywhere) or "none"}')


if __name__ == '__main__':
    main(sys.argv[1:] or ['can0', 'can1', 'can2'])
