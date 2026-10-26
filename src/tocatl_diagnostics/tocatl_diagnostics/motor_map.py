"""Inventory CLI; canonical map belongs to the motor-control package."""
from tocatl_lowlevel.motor_map import CAN_BITRATE, MOTORS, Motor, as_records, main

if __name__ == "__main__":
    main()
