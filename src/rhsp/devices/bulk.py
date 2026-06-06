"""BulkInputData and ModuleStatus dataclasses.

``BulkInputData`` decodes the ``GetBulkInputData`` response payload into a
named dataclass.

Hardware note — fw 1.8.2 payload truncation
--------------------------------------------
On REV Hub firmware 1.8.2 the ``GetBulkInputData`` response is only ~34 bytes.
It ends after: digital_inputs (1 B), four motor encoders (16 B), motor_status
(1 B), four velocities (8 B), four modes (4 B), analog_input0 (2 B),
analog_input1 (2 B) — total 34 B.

Fields beyond offset 34 (analog_input2, analog_input3, servo command/period
fields, I2C data blocks, monotonic timestamp) are zero-padded by the tolerant
``hub.bulk_input()`` path and will therefore read as 0, not as real hub
values.  Callers should not rely on those fields for meaningful data on
fw 1.8.2.

The current and voltage monitor fields that were previously included
(``motor0_current_ma`` … ``motor3_current_ma``, ``battery_current_ma``,
``battery_voltage_mv``, ``mon5v_mv``, ``gpio_current_ma``, ``i2c_current_ma``,
``servo_current_ma``) have been **removed** from ``BulkInputData`` because the
hub never sends them on fw 1.8.2 and returning 0 was misleading.  Use
``Motor.get_current_ma()``, ``Hub.battery_voltage_mv()``, and
``Hub.battery_current_ma()`` to read those values via ``GetADC`` instead.

Encoding notes
--------------
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
    """Decoded ``GetBulkInputData`` response.

    Encoder fields are signed 32-bit integers (reinterpreted here because the
    JSON overlay omits the ``signed`` flag for these fields).  Velocity fields
    are signed 16-bit integers for the same reason.

    The vendor typo ``mototonicTime`` is normalised to ``monotonic_time_ms``.

    .. note::
        Current and voltage monitor fields (``motor*_current_ma``,
        ``battery_current_ma``, ``battery_voltage_mv``, ``mon5v_mv``,
        ``gpio_current_ma``, ``i2c_current_ma``, ``servo_current_ma``) are
        **not present in this dataclass**.  On fw 1.8.2 the hub response is
        only ~34 bytes and those fields are never transmitted.  Read current
        and voltage via ``Motor.get_current_ma()``,
        ``Hub.battery_voltage_mv()``, and ``Hub.battery_current_ma()``
        (all use ``GetADC``).

        Fields beyond offset 34 (analog_input2/3, servo cmd/period fields,
        I2C data, monotonic timestamp) are zero-padded by the tolerant decode
        path on fw 1.8.2 and should not be relied upon for meaningful data.
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

    # ADC inputs (mV/mA depending on channel).
    # NOTE: analog_input0 and analog_input1 are within the fw 1.8.2 response
    # window.  analog_input2 and analog_input3 are zero-padded on fw 1.8.2
    # (not read from the hub).
    analog_input0: int
    analog_input1: int
    analog_input2: int
    analog_input3: int

    # Servo pulse widths (us) — zero-padded on fw 1.8.2
    servo0_cmd: int
    servo1_cmd: int
    servo2_cmd: int
    servo3_cmd: int
    servo4_cmd: int
    servo5_cmd: int

    # Servo frame periods (us) — zero-padded on fw 1.8.2
    servo0_frame_period_us: int
    servo1_frame_period_us: int
    servo2_frame_period_us: int
    servo3_frame_period_us: int
    servo4_frame_period_us: int
    servo5_frame_period_us: int

    # I2C block read data (10 bytes each) — zero-padded on fw 1.8.2
    i2c0_data: bytes
    i2c1_data: bytes
    i2c2_data: bytes
    i2c3_data: bytes
    imu_block: bytes

    # I2C status bytes — zero-padded on fw 1.8.2
    i2c0_status: int
    i2c1_status: int
    i2c2_status: int
    i2c3_status: int
    imu_status: int

    # Monotonic timestamp (vendor typo: "mototonicTime" in JSON)
    # Zero-padded on fw 1.8.2
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
            # analog_input2/3 are beyond the fw 1.8.2 response window (offset 34)
            # and will read as 0 when the tolerant padding path is used.
            analog_input2=_u16(rsp.get("analogInput2", 0)),
            analog_input3=_u16(rsp.get("analogInput3", 0)),
            # Servo, I2C, and timestamp fields are zero-padded on fw 1.8.2.
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
