"""Sensor register constants and calibration values.

These constants are used by sensor drivers (ticket 012+) and are kept
here so they can be imported independently of any device class.

P7-f fix: ``OSC_CALIBRATE_VAL`` must be ``0xF8``; the vendor SDK used
an incorrect value.
"""

from __future__ import annotations

__all__ = [
    # Oscillator calibration
    "OSC_CALIBRATE_VAL",
    # APDS-9960 (ColorSensor, APDS) registers
    "COMMAND_BIT",
    "MULTI_BYTE_BIT",
    "APDS9960_ENABLE",
    "APDS9960_ATIME",
    "APDS9960_PPULSE",
    "APDS9960_ID",
    "APDS9960_CDATAL",
    "APDS9960_RDATAL",
    "APDS9960_GDATAL",
    "APDS9960_BDATAL",
    "APDS9960_DEVICE_ID",
    "APDS9960_ADDRESS",
    # REV Color Sensor V3 (APDS-9151) registers
    "APDS9151_MAIN_CTRL",
    "APDS9151_PS_LED",
    "APDS9151_PS_PULSES",
    "APDS9151_PS_MEAS_RATE",
    "APDS9151_LS_MEAS_RATE",
    "APDS9151_LS_GAIN",
    "APDS9151_PART_ID",
    "APDS9151_MAIN_STATUS",
    "APDS9151_PS_DATA",
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
    "APDS9151_DEVICE_ID",
    "APDS9151_ADDRESS",
    # APDS-9151 MAIN_CTRL bits
    "APDS9151_PS_EN",
    "APDS9151_LS_EN",
    "APDS9151_RGB_MODE",
    # APDS-9151 measurement rate options
    "APDS9151_PS_MEAS_RATE_100ms",
    "APDS9151_PS_RES_11_BIT",
    "APDS9151_LS_GAIN_9",
    # VL53L0X (Distance2m) registers
    "VL53L0X_ADDRESS",
    "SYSRANGE_START",
    "SYSTEM_SEQUENCE_CONFIG",
    "SYSTEM_INTERRUPT_CONFIG_GPIO",
    "GPIO_HV_MUX_ACTIVE_HIGH",
    "SYSTEM_INTERRUPT_CLEAR",
    "RESULT_INTERRUPT_STATUS",
    "RESULT_RANGE_STATUS",
    "MSRC_CONFIG_CONTROL",
    "FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT",
    "FINAL_RANGE_CONFIG_VCSEL_PERIOD",
    "FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI",
    "PRE_RANGE_CONFIG_VCSEL_PERIOD",
    "PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI",
    "MSRC_CONFIG_TIMEOUT_MACROP",
    "GLOBAL_CONFIG_SPAD_ENABLES_REF_0",
    "GLOBAL_CONFIG_REF_EN_START_SELECT",
    "DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD",
    "DYNAMIC_SPAD_REF_EN_START_OFFSET",
    "IDENTIFICATION_MODEL_ID",
    # BNO055 (IMU) registers
    "BNO055_ADDRESS",
    "BNO055_OPR_MODE",
    "BNO055_PWR_MODE",
    "BNO055_SYS_TRIGGER",
    "BNO055_PAGE_ID",
    "BNO055_UNIT_SEL",
    "BNO055_SYS_STAT",
    "BNO055_CHIP_ID",
    "BNO055_ACC_DATA_X_LSB",
    "BNO055_EUL_H_LSB",
    "BNO055_GRV_DATA_X_LSB",
    # BNO055 operation modes
    "BNO055_CONFIGMODE",
    "BNO055_IMUMODE",
    # BNO055 power modes
    "BNO055_NORMAL",
]

# ---------------------------------------------------------------------------
# Oscillator calibration value
# P7-f: must be 0xF8 — vendor incorrectly used a different value.
# ---------------------------------------------------------------------------

OSC_CALIBRATE_VAL: int = 0xF8

# ---------------------------------------------------------------------------
# APDS-9960 (ColorSensor) register constants
# Ported from vendor/rhsp/internal/i2c.py (I2CConstants) and color.py
# ---------------------------------------------------------------------------

# I2C command modifier bits for APDS-9960
COMMAND_BIT: int = 0x80     # Must be set when reading APDS-9960 registers
MULTI_BYTE_BIT: int = 0x20  # Set for multi-byte (auto-increment) reads

# APDS-9960 device I2C address (7-bit)
APDS9960_ADDRESS: int = 0x39  # = 57 decimal

# APDS-9960 register addresses
APDS9960_ENABLE: int = 0x00   # Enable register (PON, AEN, PEN)
APDS9960_ATIME: int = 0x01    # RGBC integration time
APDS9960_PPULSE: int = 0x0E   # Proximity pulse count
APDS9960_ID: int = 0x12       # Device ID register

# APDS-9960 color data registers (CDATAL = start of 8-byte RGBC block)
APDS9960_CDATAL: int = 0x14   # Clear data low byte
APDS9960_RDATAL: int = 0x16   # Red data low byte
APDS9960_GDATAL: int = 0x18   # Green data low byte
APDS9960_BDATAL: int = 0x1A   # Blue data low byte

# APDS-9960 expected device ID
APDS9960_DEVICE_ID: int = 0x60  # = 96 decimal

# ---------------------------------------------------------------------------
# REV Color Sensor V3 (APDS-9151) register constants
# Ported from vendor/rhsp/color.py (ColorSensorV3)
# ---------------------------------------------------------------------------

# APDS-9151 device I2C address (7-bit)
APDS9151_ADDRESS: int = 0x52  # = 82 decimal

# APDS-9151 expected device part ID
APDS9151_DEVICE_ID: int = 0xC2  # = 194 decimal

# APDS-9151 register addresses
APDS9151_MAIN_CTRL: int = 0x00
APDS9151_PS_LED: int = 0x01
APDS9151_PS_PULSES: int = 0x02
APDS9151_PS_MEAS_RATE: int = 0x03
APDS9151_LS_MEAS_RATE: int = 0x04
APDS9151_LS_GAIN: int = 0x05
APDS9151_PART_ID: int = 0x06
APDS9151_MAIN_STATUS: int = 0x07
APDS9151_PS_DATA: int = 0x08
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

# APDS-9151 MAIN_CTRL enable bits
APDS9151_PS_EN: int = 0x01    # Proximity sensor enable
APDS9151_LS_EN: int = 0x02    # Light sensor enable
APDS9151_RGB_MODE: int = 0x04  # RGB mode (vs ambient light)

# APDS-9151 PS measurement rate options (PS_MEAS_RATE register)
APDS9151_PS_MEAS_RATE_100ms: int = 0x05
APDS9151_PS_RES_11_BIT: int = 0x18   # 11-bit PS resolution = 24

# APDS-9151 LS gain options (LS_GAIN register)
APDS9151_LS_GAIN_9: int = 0x03

# ---------------------------------------------------------------------------
# VL53L0X (Distance2m) register constants
# Ported from vendor/rhsp/distance.py (Distance2m class attributes)
# ---------------------------------------------------------------------------

VL53L0X_ADDRESS: int = 0x29  # = 41 decimal

# Range control
SYSRANGE_START: int = 0x00
SYSTEM_SEQUENCE_CONFIG: int = 0x01
SYSTEM_INTERRUPT_CONFIG_GPIO: int = 0x0A
GPIO_HV_MUX_ACTIVE_HIGH: int = 0x84
SYSTEM_INTERRUPT_CLEAR: int = 0x0B

# Result registers
RESULT_INTERRUPT_STATUS: int = 0x13
RESULT_RANGE_STATUS: int = 0x14

# Measurement configuration
MSRC_CONFIG_CONTROL: int = 0x60
MSRC_CONFIG_TIMEOUT_MACROP: int = 0x46
FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT: int = 0x44
FINAL_RANGE_CONFIG_VCSEL_PERIOD: int = 0x70
FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI: int = 0x71
PRE_RANGE_CONFIG_VCSEL_PERIOD: int = 0x50
PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI: int = 0x51

# SPAD configuration
GLOBAL_CONFIG_SPAD_ENABLES_REF_0: int = 0xB0
GLOBAL_CONFIG_REF_EN_START_SELECT: int = 0xB6
DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD: int = 0x4E
DYNAMIC_SPAD_REF_EN_START_OFFSET: int = 0x4F

# Identification
IDENTIFICATION_MODEL_ID: int = 0xC0

# ---------------------------------------------------------------------------
# BNO055 (IMU) register constants
# Ported from vendor/rhsp/internal/imu.py (IMUConstants)
# ---------------------------------------------------------------------------

BNO055_ADDRESS: int = 0x28  # = 40 decimal

# BNO055 identification register
BNO055_CHIP_ID: int = 0x00

# BNO055 data registers (start addresses for block reads)
BNO055_ACC_DATA_X_LSB: int = 0x08
BNO055_EUL_H_LSB: int = 0x1A
BNO055_GRV_DATA_X_LSB: int = 0x2E

# BNO055 status / control registers
BNO055_PAGE_ID: int = 0x07
BNO055_UNIT_SEL: int = 0x3B
BNO055_OPR_MODE: int = 0x3D
BNO055_PWR_MODE: int = 0x3E
BNO055_SYS_TRIGGER: int = 0x3F
BNO055_SYS_STAT: int = 0x39

# BNO055 operation modes
BNO055_CONFIGMODE: int = 0x00
BNO055_IMUMODE: int = 0x08

# BNO055 power modes
BNO055_NORMAL: int = 0x00
