"""Integer enum types for the rhsp library.

All values are derived from the ``enums`` block of ``protocol.json`` (the
authoritative source).  No hub interaction occurs here — this module is a
foundation leaf with no imports from other rhsp modules.
"""

from __future__ import annotations

from enum import IntEnum, IntFlag

__all__ = [
    "MotorMode",
    "ZeroPowerBehavior",
    "ClosedLoopMode",
    "DIODirection",
    "ADCChannel",
    "LEDColor",
    "I2CSpeedCode",
    "NackCode",
    "ModuleStatusBits",
    "MotorStatusBits",
]


class MotorMode(IntEnum):
    """Motor run mode (protocol.json → enums.MotorMode)."""

    CONSTANT_POWER = 0
    CONSTANT_VELOCITY = 1
    POSITION_TARGET = 2
    CONSTANT_CURRENT = 3


class ZeroPowerBehavior(IntEnum):
    """Behaviour when motor power is set to zero (protocol.json → enums.ZeroPowerBehavior)."""

    BRAKE_AT_ZERO = 0
    FLOAT_AT_ZERO = 1


class ClosedLoopMode(IntEnum):
    """PID closed-loop mode selector (protocol.json → enums.ClosedLoopMode).

    Used as the ``mode`` field in Set/GetMotorPIDCoefficients.
    """

    VELOCITY = 1
    POSITION = 2
    CURRENT = 3


class DIODirection(IntEnum):
    """Digital I/O pin direction (protocol.json → enums.DIODirection)."""

    INPUT = 0
    OUTPUT = 1


class ADCChannel(IntEnum):
    """ADC channel identifiers (protocol.json → enums.ADCChannel).

    Channels 0–3 are the four external analogue inputs.  Higher indices select
    internal monitoring rails.
    """

    ADC_0 = 0
    ADC_1 = 1
    ADC_2 = 2
    ADC_3 = 3
    ADC_GPIO = 4
    ADC_I2C = 5
    ADC_Servo = 6
    ADC_Battery = 7
    ADC_Motor0 = 8
    ADC_Motor1 = 9
    ADC_Motor2 = 10
    ADC_Motor3 = 11
    ADC_5VMonitor = 12
    ADC_BatteryMonitor = 13
    ADC_CPUTemp = 14


class LEDColor(IntEnum):
    """LED colour selector (protocol.json → enums.LEDColor)."""

    Red = 0
    Yellow = 1
    Green = 2


class I2CSpeedCode(IntEnum):
    """I2C bus speed (protocol.json → enums.I2CSpeedCode)."""

    STANDARD_100KHZ = 0
    FAST_400KHZ = 1


# ---------------------------------------------------------------------------
# NACK code descriptions keyed by integer code.
# Ranges from the spec are expanded to individual entries.
# ---------------------------------------------------------------------------

_NACK_DESCRIPTIONS: dict[int, str] = {
    **{n: f"Parameter #{n} out of range" for n in range(0, 10)},   # 0–9
    **{10 + n: f"GPIO #{n} not configured for output" for n in range(8)},  # 10–17
    18: "No GPIO pins configured for output",
    **{20 + n: f"GPIO #{n} not configured for input" for n in range(8)},   # 20–27
    28: "No GPIO pins configured for input",
    30: "Servo not fully configured before enabled",
    31: "Battery voltage too low to run servo",
    40: "I2C master busy (command rejected)",
    41: "I2C operation in progress (poll again)",
    42: "I2C no results pending",
    43: "I2C query mismatch",
    44: "I2C timeout - SDA stuck",
    45: "I2C timeout - SCK stuck",
    46: "I2C timeout",
    50: "Motor not fully configured for mode before enabled",
    51: "Command not valid for selected motor mode",
    52: "Battery voltage too low to run motor",
    253: "Command implementation pending",
    254: "Command routing error",
    255: "Packet Type ID unknown",
}


class NackCode(IntEnum):
    """NACK codes returned by the hub when it rejects a command (§2.12).

    Codes 19, 29, 32–39, 47–49, and 53–252 are reserved and not represented
    as named members.  Use ``NackCode.describe(code)`` to get the description
    for any raw integer code, including unnamed/reserved ones.
    """

    # 0–9: parameter out of range
    PARAM_0_OUT_OF_RANGE = 0
    PARAM_1_OUT_OF_RANGE = 1
    PARAM_2_OUT_OF_RANGE = 2
    PARAM_3_OUT_OF_RANGE = 3
    PARAM_4_OUT_OF_RANGE = 4
    PARAM_5_OUT_OF_RANGE = 5
    PARAM_6_OUT_OF_RANGE = 6
    PARAM_7_OUT_OF_RANGE = 7
    PARAM_8_OUT_OF_RANGE = 8
    PARAM_9_OUT_OF_RANGE = 9

    # 10–17: GPIO not configured for output
    GPIO_0_NOT_OUTPUT = 10
    GPIO_1_NOT_OUTPUT = 11
    GPIO_2_NOT_OUTPUT = 12
    GPIO_3_NOT_OUTPUT = 13
    GPIO_4_NOT_OUTPUT = 14
    GPIO_5_NOT_OUTPUT = 15
    GPIO_6_NOT_OUTPUT = 16
    GPIO_7_NOT_OUTPUT = 17

    NO_GPIO_OUTPUT = 18

    # 20–27: GPIO not configured for input
    GPIO_0_NOT_INPUT = 20
    GPIO_1_NOT_INPUT = 21
    GPIO_2_NOT_INPUT = 22
    GPIO_3_NOT_INPUT = 23
    GPIO_4_NOT_INPUT = 24
    GPIO_5_NOT_INPUT = 25
    GPIO_6_NOT_INPUT = 26
    GPIO_7_NOT_INPUT = 27

    NO_GPIO_INPUT = 28

    SERVO_NOT_CONFIGURED = 30
    SERVO_BATTERY_LOW = 31

    I2C_MASTER_BUSY = 40
    I2C_IN_PROGRESS = 41
    I2C_NO_RESULTS = 42
    I2C_QUERY_MISMATCH = 43
    I2C_TIMEOUT_SDA = 44
    I2C_TIMEOUT_SCK = 45
    I2C_TIMEOUT = 46

    MOTOR_NOT_CONFIGURED = 50
    MOTOR_INVALID_MODE = 51
    MOTOR_BATTERY_LOW = 52

    COMMAND_PENDING = 253
    COMMAND_ROUTING_ERROR = 254
    UNKNOWN_PACKET_TYPE = 255

    @classmethod
    def describe(cls, code: int) -> str:
        """Return a human-readable description for any NACK code integer.

        Works for both named members and reserved/unnamed codes.  Falls back to
        a generic string for codes not listed in the spec.
        """
        if code in _NACK_DESCRIPTIONS:
            return _NACK_DESCRIPTIONS[code]
        return f"Unknown NACK code {code}"


class ModuleStatusBits(IntFlag):
    """Module-level status bits returned by GetModuleStatus (§2.13, byte 0).

    Bits 6–7 are reserved.
    """

    KeepAliveTimeout = 1 << 0
    DeviceReset = 1 << 1
    FailSafe = 1 << 2
    ControllerOverTemp = 1 << 3
    BatteryLow = 1 << 4
    HIBFault = 1 << 5


class MotorStatusBits(IntFlag):
    """Motor alert bits returned by GetModuleStatus (§2.13, byte 1).

    Bits 0–3: encoder lost-count alerts; bits 4–7: driver overheat alerts.
    """

    Motor0LostCounts = 1 << 0
    Motor1LostCounts = 1 << 1
    Motor2LostCounts = 1 << 2
    Motor3LostCounts = 1 << 3
    Motor0DriverOverheat = 1 << 4
    Motor1DriverOverheat = 1 << 5
    Motor2DriverOverheat = 1 << 6
    Motor3DriverOverheat = 1 << 7
