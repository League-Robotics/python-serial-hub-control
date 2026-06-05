"""Tests for the rhsp foundation layer: errors, enums, and packaged protocol.json.

These are smoke tests and value assertions — no hub hardware required.
"""

import importlib.resources
import json

import rhsp
import rhsp.errors as errors_mod
import rhsp.enums as enums_mod
from rhsp.errors import (
    ChecksumError,
    NackError,
    ProtocolError,
    RhspError,
    RhspTimeoutError,
)
from rhsp.enums import (
    ADCChannel,
    ClosedLoopMode,
    DIODirection,
    I2CSpeedCode,
    MotorMode,
    MotorStatusBits,
    ModuleStatusBits,
    NackCode,
    ZeroPowerBehavior,
)


# ---------------------------------------------------------------------------
# Verify new src/ package is loaded (not vendor/)
# ---------------------------------------------------------------------------


def test_rhsp_package_is_from_src() -> None:
    """rhsp.__file__ must live under src/, not vendor/."""
    assert rhsp.__file__ is not None
    assert "src/rhsp" in rhsp.__file__ or "src" in rhsp.__file__, (
        f"Expected src/rhsp to be loaded, got: {rhsp.__file__}"
    )


# ---------------------------------------------------------------------------
# Exception hierarchy
# ---------------------------------------------------------------------------


def test_exception_mro_rhsp_error() -> None:
    assert issubclass(RhspError, Exception)


def test_exception_mro_protocol_error() -> None:
    assert issubclass(ProtocolError, RhspError)


def test_exception_mro_checksum_error() -> None:
    assert issubclass(ChecksumError, ProtocolError)
    assert issubclass(ChecksumError, RhspError)


def test_exception_mro_timeout_error() -> None:
    assert issubclass(RhspTimeoutError, RhspError)


def test_exception_mro_nack_error() -> None:
    assert issubclass(NackError, RhspError)


def test_nack_error_attributes() -> None:
    err = NackError(50, "Motor not configured")
    assert err.code == 50
    assert err.description == "Motor not configured"


def test_nack_error_str_contains_code() -> None:
    err = NackError(41, "I2C in progress")
    assert "41" in str(err)


def test_nack_error_is_catchable_as_rhsp_error() -> None:
    with __import__("pytest").raises(RhspError):
        raise NackError(0, "param 0 out of range")


# ---------------------------------------------------------------------------
# MotorMode
# ---------------------------------------------------------------------------


def test_motor_mode_constant_power() -> None:
    assert MotorMode.CONSTANT_POWER == 0


def test_motor_mode_constant_velocity() -> None:
    assert MotorMode.CONSTANT_VELOCITY == 1


def test_motor_mode_position_target() -> None:
    assert MotorMode.POSITION_TARGET == 2


def test_motor_mode_constant_current() -> None:
    assert MotorMode.CONSTANT_CURRENT == 3


# ---------------------------------------------------------------------------
# ZeroPowerBehavior
# ---------------------------------------------------------------------------


def test_zero_power_behavior_brake() -> None:
    assert ZeroPowerBehavior.BRAKE_AT_ZERO == 0


def test_zero_power_behavior_float() -> None:
    assert ZeroPowerBehavior.FLOAT_AT_ZERO == 1


# ---------------------------------------------------------------------------
# NackCode — spot-check named members and describe()
# ---------------------------------------------------------------------------


def test_nack_code_param_range() -> None:
    for n in range(10):
        member = NackCode(n)
        assert member.value == n


def test_nack_code_i2c_in_progress() -> None:
    assert NackCode.I2C_IN_PROGRESS == 41


def test_nack_code_motor_not_configured() -> None:
    assert NackCode.MOTOR_NOT_CONFIGURED == 50


def test_nack_code_unknown_packet_type() -> None:
    assert NackCode.UNKNOWN_PACKET_TYPE == 255


def test_nack_code_count_named_members() -> None:
    # Spec defines:
    # 10 param codes (0–9) + 8 gpio-out (10–17) + 1 (18) + 8 gpio-in (20–27)
    # + 1 (28) + 2 servo + 6 I2C + 3 motor + 3 system = 42 named members
    assert len(NackCode) >= 40, f"Expected ≥40 NackCode members, got {len(NackCode)}"


def test_nack_code_describe_named() -> None:
    desc = NackCode.describe(50)
    assert "Motor" in desc or "motor" in desc


def test_nack_code_describe_range() -> None:
    # Code 3 is "Parameter #3 out of range"
    desc = NackCode.describe(3)
    assert "3" in desc


def test_nack_code_describe_unknown() -> None:
    # 200 is reserved; should return a fallback string
    desc = NackCode.describe(200)
    assert "200" in desc or "Unknown" in desc


# ---------------------------------------------------------------------------
# ADCChannel — 15 channels (0–3, 4–14)
# ---------------------------------------------------------------------------


def test_adc_channel_count() -> None:
    assert len(ADCChannel) == 15


def test_adc_channel_values() -> None:
    assert ADCChannel.ADC_0 == 0
    assert ADCChannel.ADC_CPUTemp == 14


# ---------------------------------------------------------------------------
# I2CSpeedCode
# ---------------------------------------------------------------------------


def test_i2c_speed_standard() -> None:
    assert I2CSpeedCode.STANDARD_100KHZ == 0


def test_i2c_speed_fast() -> None:
    assert I2CSpeedCode.FAST_400KHZ == 1


# ---------------------------------------------------------------------------
# ModuleStatusBits — IntFlag
# ---------------------------------------------------------------------------


def test_module_status_bits_is_int_flag() -> None:
    from enum import IntFlag

    assert issubclass(ModuleStatusBits, IntFlag)


def test_module_status_bits_values() -> None:
    assert ModuleStatusBits.KeepAliveTimeout == 1
    assert ModuleStatusBits.DeviceReset == 2
    assert ModuleStatusBits.FailSafe == 4
    assert ModuleStatusBits.ControllerOverTemp == 8
    assert ModuleStatusBits.BatteryLow == 16
    assert ModuleStatusBits.HIBFault == 32


def test_module_status_bits_combination() -> None:
    combined = ModuleStatusBits.KeepAliveTimeout | ModuleStatusBits.BatteryLow
    assert ModuleStatusBits.KeepAliveTimeout in combined
    assert ModuleStatusBits.BatteryLow in combined
    assert ModuleStatusBits.FailSafe not in combined


# ---------------------------------------------------------------------------
# MotorStatusBits — IntFlag
# ---------------------------------------------------------------------------


def test_motor_status_bits_is_int_flag() -> None:
    from enum import IntFlag

    assert issubclass(MotorStatusBits, IntFlag)


def test_motor_status_bits_values() -> None:
    assert MotorStatusBits.Motor0LostCounts == 1
    assert MotorStatusBits.Motor3LostCounts == 8
    assert MotorStatusBits.Motor0DriverOverheat == 16
    assert MotorStatusBits.Motor3DriverOverheat == 128


# ---------------------------------------------------------------------------
# protocol.json accessible via importlib.resources
# ---------------------------------------------------------------------------


def test_protocol_json_loadable() -> None:
    data = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    assert len(data) > 0


def test_protocol_json_is_valid_json() -> None:
    data = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    parsed = json.loads(data)
    assert isinstance(parsed, dict)


def test_protocol_json_has_enums_block() -> None:
    data = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    parsed = json.loads(data)
    assert "enums" in parsed


def test_protocol_json_has_commands() -> None:
    data = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    parsed = json.loads(data)
    assert "commands" in parsed
    assert len(parsed["commands"]) > 0


# ---------------------------------------------------------------------------
# Top-level __init__ re-exports
# ---------------------------------------------------------------------------


def test_top_level_imports_errors() -> None:
    from rhsp import ChecksumError, NackError, ProtocolError, RhspError, RhspTimeoutError  # noqa: F401


def test_top_level_imports_enums() -> None:
    from rhsp import MotorMode, NackCode  # noqa: F401
