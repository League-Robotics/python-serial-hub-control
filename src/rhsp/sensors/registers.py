"""Sensor register constants and calibration values.

These constants are used by sensor drivers (ticket 012+) and are kept
here so they can be imported independently of any device class.

P7-f fix: ``OSC_CALIBRATE_VAL`` must be ``0xF8``; the vendor SDK used
an incorrect value.
"""

from __future__ import annotations

__all__ = [
    "OSC_CALIBRATE_VAL",
    # APDS-9151 registers
    "APDS9151_MAIN_CTRL",
    "APDS9151_LS_MEAS_RATE",
    "APDS9151_LS_GAIN",
    "APDS9151_PART_ID",
    "APDS9151_MAIN_STATUS",
    "APDS9151_LS_DATA_IR_0",
    "APDS9151_LS_DATA_IR_1",
    "APDS9151_LS_DATA_IR_2",
    "APDS9151_LS_DATA_GREEN_0",
    "APDS9151_LS_DATA_GREEN_1",
    "APDS9151_LS_DATA_GREEN_2",
    "APDS9151_LS_DATA_BLUE_0",
    "APDS9151_LS_DATA_BLUE_1",
    "APDS9151_LS_DATA_BLUE_2",
    "APDS9151_LS_DATA_RED_0",
    "APDS9151_LS_DATA_RED_1",
    "APDS9151_LS_DATA_RED_2",
]

# ---------------------------------------------------------------------------
# Oscillator calibration value
# P7-f: must be 0xF8 — vendor incorrectly used a different value.
# ---------------------------------------------------------------------------

OSC_CALIBRATE_VAL: int = 0xF8

# ---------------------------------------------------------------------------
# APDS-9151 light sensor register addresses
# ---------------------------------------------------------------------------

APDS9151_MAIN_CTRL: int = 0x00
APDS9151_LS_MEAS_RATE: int = 0x04
APDS9151_LS_GAIN: int = 0x05
APDS9151_PART_ID: int = 0x06
APDS9151_MAIN_STATUS: int = 0x07
APDS9151_LS_DATA_IR_0: int = 0x0A
APDS9151_LS_DATA_IR_1: int = 0x0B
APDS9151_LS_DATA_IR_2: int = 0x0C
APDS9151_LS_DATA_GREEN_0: int = 0x0D
APDS9151_LS_DATA_GREEN_1: int = 0x0E
APDS9151_LS_DATA_GREEN_2: int = 0x0F
APDS9151_LS_DATA_BLUE_0: int = 0x10
APDS9151_LS_DATA_BLUE_1: int = 0x11
APDS9151_LS_DATA_BLUE_2: int = 0x12
APDS9151_LS_DATA_RED_0: int = 0x13
APDS9151_LS_DATA_RED_1: int = 0x14
APDS9151_LS_DATA_RED_2: int = 0x15
