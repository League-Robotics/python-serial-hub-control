"""BulkInputData and ModuleStatus dataclasses.

``BulkInputData`` decodes the ``GetBulkInputData`` response payload into a
named dataclass.  All 47 fields from the response are represented with correct
Python types:

- Encoder fields (motor0Encoder … motor3Encoder) are signed 32-bit integers.
- Velocity fields (motor0Velocity … motor3Velocity) are signed 16-bit integers.
- The monotonic time field has the vendor typo ``mototonicTime`` in the JSON;
  ``BulkInputData`` exposes it as ``monotonic_time_ms``.

``ModuleStatus`` decodes the two-byte ``GetModuleStatus`` response.
"""

from __future__ import annotations

from dataclasses import dataclass

from rhsp.enums import ModuleStatusBits, MotorStatusBits

__all__ = ["BulkInputData", "ModuleStatus"]


@dataclass
class ModuleStatus:
    """Decoded ``GetModuleStatus`` response.

    Attributes:
        status_bits:  Module-level status flags (see :class:`~rhsp.enums.ModuleStatusBits`).
        motor_alerts: Per-motor alert flags (see :class:`~rhsp.enums.MotorStatusBits`).
    """

    status_bits: ModuleStatusBits
    motor_alerts: MotorStatusBits

    @classmethod
    def from_response(cls, rsp: dict) -> "ModuleStatus":
        """Construct a ``ModuleStatus`` from a decoded transaction response dict.

        Parameters:
            rsp: Dict returned by ``session.transaction("GetModuleStatus", ...)``
                 with keys ``"statusWord"`` and ``"motorAlerts"``.
        """
        status_word = rsp.get("statusWord", b"")
        motor_word = rsp.get("motorAlerts", b"")

        status_int = _coerce_int(status_word, nbytes=1, signed=False)
        motor_int = _coerce_int(motor_word, nbytes=1, signed=False)

        return cls(
            status_bits=ModuleStatusBits(status_int),
            motor_alerts=MotorStatusBits(motor_int),
        )


@dataclass
class BulkInputData:
    """Decoded ``GetBulkInputData`` response — all 47+ fields.

    Encoder fields are signed 32-bit integers (reinterpreted here because the
    JSON overlay omits the ``signed`` flag for these fields).  Velocity fields
    are signed 16-bit integers for the same reason.

    The vendor typo ``mototonicTime`` is normalised to ``monotonic_time_ms``.
    """

    # DIO
    digital_inputs: int

    # Motor encoders (signed 32-bit)
    motor0_encoder: int
    motor1_encoder: int
    motor2_encoder: int
    motor3_encoder: int

    # Motor status bitmask
    motor_status: int

    # Motor velocities (signed 16-bit, encoder counts/s)
    motor0_velocity: int
    motor1_velocity: int
    motor2_velocity: int
    motor3_velocity: int

    # Motor modes
    motor0_mode: int
    motor1_mode: int
    motor2_mode: int
    motor3_mode: int

    # ADC inputs (mV/mA depending on channel)
    analog_input0: int
    analog_input1: int
    analog_input2: int
    analog_input3: int

    # Current monitors (mA)
    gpio_current_ma: int
    i2c_current_ma: int
    servo_current_ma: int
    battery_current_ma: int
    motor0_current_ma: int
    motor1_current_ma: int
    motor2_current_ma: int
    motor3_current_ma: int

    # Voltage monitors (mV)
    mon5v_mv: int
    battery_voltage_mv: int

    # Servo pulse widths (us)
    servo0_cmd: int
    servo1_cmd: int
    servo2_cmd: int
    servo3_cmd: int
    servo4_cmd: int
    servo5_cmd: int

    # Servo frame periods (us)
    servo0_frame_period_us: int
    servo1_frame_period_us: int
    servo2_frame_period_us: int
    servo3_frame_period_us: int
    servo4_frame_period_us: int
    servo5_frame_period_us: int

    # I2C block read data (10 bytes each)
    i2c0_data: bytes
    i2c1_data: bytes
    i2c2_data: bytes
    i2c3_data: bytes
    imu_block: bytes

    # I2C status bytes
    i2c0_status: int
    i2c1_status: int
    i2c2_status: int
    i2c3_status: int
    imu_status: int

    # Monotonic timestamp (vendor typo: "mototonicTime" in JSON)
    monotonic_time_ms: int

    @classmethod
    def from_response(cls, rsp: dict) -> "BulkInputData":
        """Construct a ``BulkInputData`` from a decoded transaction response dict.

        Signed reinterpretation is applied here for encoder (4-byte signed) and
        velocity (2-byte signed) fields because the JSON overlay omits the
        ``signed`` flag for these fields.

        Parameters:
            rsp: Dict returned by ``session.transaction("GetBulkInputData", ...)``
                 with all the ``GetBulkInputData_RSP`` field keys.
        """
        def _s32(v: object) -> int:
            return _coerce_int(v, nbytes=4, signed=True)

        def _s16(v: object) -> int:
            return _coerce_int(v, nbytes=2, signed=True)

        def _u8(v: object) -> int:
            return _coerce_int(v, nbytes=1, signed=False)

        def _u16(v: object) -> int:
            return _coerce_int(v, nbytes=2, signed=False)

        def _u32(v: object) -> int:
            return _coerce_int(v, nbytes=4, signed=False)

        def _raw(v: object, n: int) -> bytes:
            if isinstance(v, (bytes, bytearray)):
                return bytes(v)
            # If already int (shouldn't happen for these fields), encode it.
            return int(v).to_bytes(n, "little")

        return cls(
            digital_inputs=_u8(rsp.get("digitalInputs", 0)),
            motor0_encoder=_s32(rsp.get("motor0Encoder", 0)),
            motor1_encoder=_s32(rsp.get("motor1Encoder", 0)),
            motor2_encoder=_s32(rsp.get("motor2Encoder", 0)),
            motor3_encoder=_s32(rsp.get("motor3Encoder", 0)),
            motor_status=_u8(rsp.get("motorStatus", 0)),
            motor0_velocity=_s16(rsp.get("motor0Velocity", 0)),
            motor1_velocity=_s16(rsp.get("motor1Velocity", 0)),
            motor2_velocity=_s16(rsp.get("motor2Velocity", 0)),
            motor3_velocity=_s16(rsp.get("motor3Velocity", 0)),
            motor0_mode=_u8(rsp.get("motor0mode", 0)),
            motor1_mode=_u8(rsp.get("motor1mode", 0)),
            motor2_mode=_u8(rsp.get("motor2mode", 0)),
            motor3_mode=_u8(rsp.get("motor3mode", 0)),
            analog_input0=_u16(rsp.get("analogInput0", 0)),
            analog_input1=_u16(rsp.get("analogInput1", 0)),
            analog_input2=_u16(rsp.get("analogInput2", 0)),
            analog_input3=_u16(rsp.get("analogInput3", 0)),
            gpio_current_ma=_u16(rsp.get("gpioCurrent_mA", 0)),
            i2c_current_ma=_u16(rsp.get("i2cCurrent_mA", 0)),
            servo_current_ma=_u16(rsp.get("servoCurrent_mA", 0)),
            battery_current_ma=_u16(rsp.get("batteryCurrent_mA", 0)),
            motor0_current_ma=_u16(rsp.get("motor0current_mA", 0)),
            motor1_current_ma=_u16(rsp.get("motor1current_mA", 0)),
            motor2_current_ma=_u16(rsp.get("motor2current_mA", 0)),
            motor3_current_ma=_u16(rsp.get("motor3current_mA", 0)),
            mon5v_mv=_u16(rsp.get("mon5v_mV", 0)),
            battery_voltage_mv=_u16(rsp.get("batteryVoltage_mV", 0)),
            servo0_cmd=_u16(rsp.get("servo0cmd", 0)),
            servo1_cmd=_u16(rsp.get("servo1cmd", 0)),
            servo2_cmd=_u16(rsp.get("servo2cmd", 0)),
            servo3_cmd=_u16(rsp.get("servo3cmd", 0)),
            servo4_cmd=_u16(rsp.get("servo4cmd", 0)),
            servo5_cmd=_u16(rsp.get("servo5cmd", 0)),
            servo0_frame_period_us=_u16(rsp.get("servo0framePeriod_us", 0)),
            servo1_frame_period_us=_u16(rsp.get("servo1framePeriod_us", 0)),
            servo2_frame_period_us=_u16(rsp.get("servo2framePeriod_us", 0)),
            servo3_frame_period_us=_u16(rsp.get("servo3framePeriod_us", 0)),
            servo4_frame_period_us=_u16(rsp.get("servo4framePeriod_us", 0)),
            servo5_frame_period_us=_u16(rsp.get("servo5framePeriod_us", 0)),
            i2c0_data=_raw(rsp.get("i2c0data", b"\x00" * 10), 10),
            i2c1_data=_raw(rsp.get("i2c1data", b"\x00" * 10), 10),
            i2c2_data=_raw(rsp.get("i2c2data", b"\x00" * 10), 10),
            i2c3_data=_raw(rsp.get("i2c3data", b"\x00" * 10), 10),
            imu_block=_raw(rsp.get("imuBlock", b"\x00" * 10), 10),
            i2c0_status=_u8(rsp.get("i2c0Status", 0)),
            i2c1_status=_u8(rsp.get("i2c1Status", 0)),
            i2c2_status=_u8(rsp.get("i2c2Status", 0)),
            i2c3_status=_u8(rsp.get("i2c3Status", 0)),
            imu_status=_u8(rsp.get("imuStatus", 0)),
            # Vendor typo: "mototonicTime" in JSON
            monotonic_time_ms=_u32(rsp.get("mototonicTime", 0)),
        )


# ---------------------------------------------------------------------------
# Internal helper
# ---------------------------------------------------------------------------


def _coerce_int(value: object, *, nbytes: int, signed: bool) -> int:
    """Coerce a value that may be bytes or int to a proper Python int.

    The codec's last-field rule returns raw bytes for single-field responses.
    For multi-field responses, the codec decodes fields without the ``signed``
    flag for encoder/velocity fields (which omit it in the JSON overlay);
    the decoded value is an unsigned int.  When *signed* is True, this helper
    reinterprets the unsigned int as two's-complement if it is out of the
    signed range.

    Parameters:
        value:  The value from the decoded response dict (bytes or int).
        nbytes: Expected byte width (used when *value* is bytes or for sign
                reinterpretation).
        signed: Whether to interpret the bytes/int as two's-complement signed.
    """
    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)
        # Use only the first *nbytes* bytes (guard against extra trailing bytes).
        raw = raw[:nbytes]
        # Pad to *nbytes* if shorter.
        raw = raw.ljust(nbytes, b"\x00")
        return int.from_bytes(raw, "little", signed=signed)
    raw_int = int(value)
    if signed:
        # Re-encode as bytes then decode signed to handle unsigned→signed reinterpretation.
        # This is a no-op when the value is already in the signed range.
        try:
            raw_bytes = raw_int.to_bytes(nbytes, "little", signed=False)
            return int.from_bytes(raw_bytes, "little", signed=True)
        except OverflowError:
            # Value is negative and already a signed int; pass through.
            return raw_int
    return raw_int
