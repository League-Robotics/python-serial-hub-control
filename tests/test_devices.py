"""Tests for Session typed methods — byte-exact payload assertions and P7 bug regressions.

All tests are hardware-free, using FakeHub over LoopbackTransport.

Coverage:
- set_servo_pulse_width(0, 1500)       → payload bytes 00 DC 05
- set_motor_constant_power(0, 16000)   → payload bytes 00 80 3E
- set_motor_channel_mode(0, CONSTANT_POWER, float_at_zero=True) → payload 00 00 01
- get_dio_direction                    → returns decoded bool (P7-b fix)
- get_pwm_pulse_width                  → decodes 2-byte response (P7-g fix)
- i2c_configure_query                  → uses I2CConfigureQuery_RSP (P7-e fix)
- get_adc                              → returns int (not bytes)
- get_motor_encoder_position           → returns signed 32-bit int
- get_bulk_input_data                  → BulkInputData with correct types + negative encoder
- query_interface                      → stores deka_base, returns (packet_id, num_values)
- keep_alive / fail_safe               → send correct packet types (ACK)
- set_module_led_color                 → payload bytes r g b
- read_version_string                  → returns ASCII string
"""

from __future__ import annotations

import struct
from typing import Any

import pytest

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
from rhsp.codec import encode_payload
from rhsp.enums import MotorMode, ZeroPowerBehavior
from rhsp.framing import build_frame
from rhsp.session import Session
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# Packet-type constants
# ---------------------------------------------------------------------------

_ACK_TYPE = 0x7F01
_KEEPALIVE_TYPE = 0x7F04
_FAILSAFE_TYPE = 0x7F05
_DEKA_BASE = 0x1000


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_session(
    *,
    timeout: float = 0.5,
) -> tuple[LoopbackTransport, FakeHub, Session]:
    """Return a wired-up (transport, hub, session) triple."""
    transport = LoopbackTransport()
    hub = FakeHub(transport)
    session = Session(transport, timeout=timeout)
    return transport, hub, session


# ---------------------------------------------------------------------------
# Payload byte-exact assertions — set_servo_pulse_width
# ---------------------------------------------------------------------------


class TestSetServoPulseWidth:
    """set_servo_pulse_width(dest, channel, pulse_width) byte-exact payload."""

    def test_payload_bytes_channel0_1500(self) -> None:
        """set_servo_pulse_width(dest=1, channel=0, pulse_width=1500) → 00 DC 05."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_servo_pulse_width(dest=1, channel=0, pulse_width=1500)
        finally:
            hub.stop()

        # Verify the raw payload bytes: 0x00 (channel), 0xDC 0x05 (1500 LE).
        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x21  # SetServoPulseWidth = 0x1021
        )
        assert req.payload == bytes([0x00, 0xDC, 0x05])


# ---------------------------------------------------------------------------
# Payload byte-exact assertions — set_motor_constant_power
# ---------------------------------------------------------------------------


class TestSetMotorConstantPower:
    """set_motor_constant_power(dest, channel, power) byte-exact payload."""

    def test_payload_bytes_channel0_power16000(self) -> None:
        """set_motor_constant_power(dest=1, channel=0, power=16000) → 00 80 3E."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_motor_constant_power(dest=1, channel=0, power=16000)
        finally:
            hub.stop()

        # Raw payload: 0x00, then 16000 as signed LE 2-byte = 0x80 0x3E
        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x0F  # SetMotorConstantPower = 0x100F
        )
        assert req.payload == bytes([0x00, 0x80, 0x3E])


# ---------------------------------------------------------------------------
# Payload byte-exact assertions — set_motor_channel_mode
# ---------------------------------------------------------------------------


class TestSetMotorChannelMode:
    """set_motor_channel_mode byte-exact payload."""

    def test_payload_bytes_mode_constant_power_float_at_zero(self) -> None:
        """set_motor_channel_mode(dest=1, channel=0, CONSTANT_POWER, float_at_zero=True) → 00 00 01."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_motor_channel_mode(
                dest=1,
                channel=0,
                mode=MotorMode.CONSTANT_POWER,
                float_at_zero=True,
            )
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x08  # SetMotorChannelMode = 0x1008
        )
        # channel=0, mode=CONSTANT_POWER=0, float_at_zero=True → 1
        assert req.payload == bytes([0x00, 0x00, 0x01])


# ---------------------------------------------------------------------------
# P7-b fix — get_dio_direction returns a decoded value
# ---------------------------------------------------------------------------


class TestGetDioDirection:
    """get_dio_direction returns a bool decoded from the RSP (P7-b fix)."""

    def test_get_dio_direction_returns_bool(self) -> None:
        """get_dio_direction(dest, pin) must return a Python bool."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetDIODirection", directionOutput=1)
        hub.run_in_thread()
        try:
            result = session.get_dio_direction(dest=1, pin=3)
        finally:
            hub.stop()

        assert isinstance(result, bool)
        assert result is True

    def test_get_dio_direction_returns_false_for_input(self) -> None:
        """get_dio_direction returns False when the pin is configured as input."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetDIODirection", directionOutput=0)
        hub.run_in_thread()
        try:
            result = session.get_dio_direction(dest=1, pin=0)
        finally:
            hub.stop()

        assert result is False


# ---------------------------------------------------------------------------
# P7-g fix — get_pwm_pulse_width decodes 2-byte response
# ---------------------------------------------------------------------------


class TestGetPwmPulseWidth:
    """get_pwm_pulse_width decodes a 2-byte response (P7-g fix)."""

    def test_get_pwm_pulse_width_returns_int(self) -> None:
        """get_pwm_pulse_width must return an int, not bytes."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetPWNPulseWidth", pulseWidth=1000)
        hub.run_in_thread()
        try:
            result = session.get_pwm_pulse_width(dest=1, channel=0)
        finally:
            hub.stop()

        assert isinstance(result, int)
        assert result == 1000

    def test_get_pwm_pulse_width_large_value(self) -> None:
        """get_pwm_pulse_width can return values > 255 (2-byte field)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetPWNPulseWidth", pulseWidth=2000)
        hub.run_in_thread()
        try:
            result = session.get_pwm_pulse_width(dest=1, channel=0)
        finally:
            hub.stop()

        assert result == 2000


# ---------------------------------------------------------------------------
# P7-e fix — i2c_configure_query uses I2CConfigureQuery_RSP
# ---------------------------------------------------------------------------


class TestI2CConfigureQuery:
    """i2c_configure_query uses I2CConfigureQuery_RSP from RESPONSES_BY_ID (P7-e fix)."""

    def test_i2c_configure_query_response_registered(self) -> None:
        """I2CConfigureQuery_RSP must exist in RESPONSES_BY_ID."""
        assert 0x902F in RESPONSES_BY_ID
        rsp = RESPONSES_BY_ID[0x902F]
        assert rsp.name == "I2CConfigureQuery_RSP"

    def test_i2c_configure_query_returns_int(self) -> None:
        """i2c_configure_query must return an int speed code."""
        transport, hub, session = _make_session()
        hub.set_rsp("I2CConfigureQuery", speedCode=1)
        hub.run_in_thread()
        try:
            result = session.i2c_configure_query(dest=1, i2c_ch=0)
        finally:
            hub.stop()

        assert isinstance(result, int)
        assert result == 1  # FAST_400KHZ


# ---------------------------------------------------------------------------
# get_adc returns int (not bytes)
# ---------------------------------------------------------------------------


class TestGetADC:
    """get_adc returns a plain int (codec last-field bytes→int coercion)."""

    def test_get_adc_returns_int(self) -> None:
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=11934)
        hub.run_in_thread()
        try:
            result = session.get_adc(dest=1, channel=0)
        finally:
            hub.stop()

        assert isinstance(result, int)
        assert result == 11934

    def test_get_adc_raw_mode(self) -> None:
        """get_adc(raw=True) sends rawMode=1 in the payload."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=500)
        hub.run_in_thread()
        try:
            result = session.get_adc(dest=1, channel=2, raw=True)
        finally:
            hub.stop()

        assert result == 500
        # GetADC has 2 fields (adcChannel, rawMode); last field rawMode returns bytes.
        # Check raw payload instead.
        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x07  # GetADC = 0x1007
        )
        assert req.payload[1] == 1  # rawMode byte at offset 1


# ---------------------------------------------------------------------------
# get_motor_encoder_position returns signed 32-bit int
# ---------------------------------------------------------------------------


class TestGetMotorEncoderPosition:
    """get_motor_encoder_position returns a signed 32-bit int."""

    def test_positive_encoder(self) -> None:
        transport, hub, session = _make_session()
        hub.set_rsp("GetMotorEncoderPosition", currentPosition=28)
        hub.run_in_thread()
        try:
            result = session.get_motor_encoder_position(dest=1, channel=0)
        finally:
            hub.stop()

        assert result == 28

    def test_negative_encoder(self) -> None:
        """Encoder position -1 must decode as -1 (not 4294967295)."""
        transport, hub, session = _make_session()
        # Encode -1 as a signed 32-bit integer → 0xFFFFFFFF
        hub.set_rsp("GetMotorEncoderPosition", currentPosition=-1)
        hub.run_in_thread()
        try:
            result = session.get_motor_encoder_position(dest=1, channel=0)
        finally:
            hub.stop()

        assert isinstance(result, int)
        assert result == -1

    def test_large_negative_encoder(self) -> None:
        """get_motor_encoder_position(-123456) round-trips correctly."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetMotorEncoderPosition", currentPosition=-123456)
        hub.run_in_thread()
        try:
            result = session.get_motor_encoder_position(dest=1, channel=0)
        finally:
            hub.stop()

        assert result == -123456


# ---------------------------------------------------------------------------
# get_bulk_input_data returns BulkInputData
# ---------------------------------------------------------------------------


class TestGetBulkInputData:
    """get_bulk_input_data returns a BulkInputData with correct types."""

    def _make_bulk_rsp_payload(self, *, motor0_encoder: int = 0) -> bytes:
        """Build a 141-byte GetBulkInputData_RSP payload with a given encoder value."""
        # Build by hand: 141 bytes of zeros except for motor0Encoder (bytes 1–4).
        data = bytearray(141)
        # motor0Encoder at offset 1, 4 bytes, little-endian signed
        enc_bytes = motor0_encoder.to_bytes(4, "little", signed=True)
        data[1:5] = enc_bytes
        return bytes(data)

    def test_get_bulk_input_data_returns_dataclass(self) -> None:
        """get_bulk_input_data must return a BulkInputData instance."""
        from rhsp.devices.bulk import BulkInputData

        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            # Inject a raw RSP frame manually (141 bytes of zeros).
            payload = self._make_bulk_rsp_payload()
            rsp_id = COMMANDS["GetBulkInputData"].reply_id
            # We'll send the session request and have FakeHub process it.
            # Then manually inject the RSP.
            import threading
            import time

            def _inject_after_request() -> None:
                time.sleep(0.02)
                rsp_frame = build_frame(
                    dest=0,
                    src=1,
                    msg=0,
                    ref=session._msg_num,  # current msg_num will be sent
                    ptype=rsp_id,
                    payload=payload,
                )
                transport.inject(rsp_frame)

            # We need to know msg_num before session sends it.
            expected_ref = session._msg_num

            t = threading.Thread(target=lambda: (
                time.sleep(0.02),
                transport.inject(
                    build_frame(
                        dest=0, src=1, msg=0, ref=expected_ref,
                        ptype=rsp_id, payload=payload,
                    )
                )
            ), daemon=True)
            t.start()

            result = session.get_bulk_input_data(dest=1)
            t.join(timeout=1.0)
        finally:
            hub.stop()

        assert isinstance(result, BulkInputData)

    def test_get_bulk_input_data_negative_encoder(self) -> None:
        """BulkInputData.motor0_encoder is a signed 32-bit int."""
        from rhsp.devices.bulk import BulkInputData
        import threading
        import time

        transport, hub, session = _make_session()
        payload = self._make_bulk_rsp_payload(motor0_encoder=-999)
        rsp_id = COMMANDS["GetBulkInputData"].reply_id

        hub.run_in_thread()
        try:
            expected_ref = session._msg_num

            def _inject() -> None:
                time.sleep(0.02)
                transport.inject(
                    build_frame(
                        dest=0, src=1, msg=0, ref=expected_ref,
                        ptype=rsp_id, payload=payload,
                    )
                )

            t = threading.Thread(target=_inject, daemon=True)
            t.start()
            result = session.get_bulk_input_data(dest=1)
            t.join(timeout=1.0)
        finally:
            hub.stop()

        assert isinstance(result, BulkInputData)
        assert result.motor0_encoder == -999


# ---------------------------------------------------------------------------
# System commands
# ---------------------------------------------------------------------------


class TestSystemCommands:
    """keep_alive, fail_safe, set_module_led_color, read_version_string."""

    def test_keep_alive_sends_ack_command(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.keep_alive(dest=1)
        finally:
            hub.stop()

        assert any(r.packet_type == _KEEPALIVE_TYPE for r in hub.requests)

    def test_fail_safe_sends_correct_packet(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.fail_safe(dest=1)
        finally:
            hub.stop()

        assert any(r.packet_type == _FAILSAFE_TYPE for r in hub.requests)

    def test_set_module_led_color_payload(self) -> None:
        """set_module_led_color emits r, g, b bytes."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_module_led_color(dest=1, r=10, g=20, b=30)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == 0x7F0A  # SetModuleLEDColor
        )
        assert req.payload == bytes([10, 20, 30])

    def test_read_version_string_returns_str(self) -> None:
        """read_version_string returns a Python str."""
        transport, hub, session = _make_session()
        version = "HW: 20, Maj: 1"
        version_bytes = version.encode("ascii")
        hub.set_rsp(
            "ReadVersionString",
            length=len(version_bytes),
            versionString=version_bytes,
        )
        hub.run_in_thread()
        try:
            result = session.read_version_string(dest=1)
        finally:
            hub.stop()

        assert isinstance(result, str)
        assert result == version

    def test_query_interface_updates_deka_base(self) -> None:
        """query_interface('DEKA') stores the returned packetID in session.deka_base."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            packet_id, num_values = session.query_interface(dest=1, name="DEKA")
        finally:
            hub.stop()

        assert packet_id == 0x1000
        assert num_values == 49
        assert session.deka_base == 0x1000

    def test_query_interface_returns_tuple(self) -> None:
        """query_interface returns (packet_id, num_values) tuple."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            result = session.query_interface(dest=1, name="DEKA")
        finally:
            hub.stop()

        assert isinstance(result, tuple)
        assert len(result) == 2


# ---------------------------------------------------------------------------
# Motor typed methods — basic round-trips
# ---------------------------------------------------------------------------


class TestMotorTypedMethods:
    """Basic motor method tests (ACK commands — payload byte-exact)."""

    def test_set_motor_channel_enable(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_motor_channel_enable(dest=1, channel=2, enabled=True)
        finally:
            hub.stop()

        # Raw payload: channel=2, enabled=1.
        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x0A  # SetMotorChannelEnable = 0x100A
        )
        assert req.payload == bytes([0x02, 0x01])

    def test_reset_motor_encoder(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.reset_motor_encoder(dest=1, channel=0)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x0E  # ResetMotorEncoder = 0x100E
        )
        assert req.payload == bytes([0x00])

    def test_set_motor_target_velocity(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_motor_target_velocity(dest=1, channel=1, velocity=-500)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x11  # SetMotorTargetVelocity = 0x1011
        )
        # channel=1, velocity=-500 LE signed 16-bit = 0x0C 0xFE
        assert req.payload[0] == 0x01  # channel 1
        assert req.payload[1:] == (-500).to_bytes(2, "little", signed=True)

    def test_set_motor_target_position(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_motor_target_position(
                dest=1, channel=0, position=1000, tolerance=20
            )
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x13  # SetMotorTargetPosition = 0x1013
        )
        assert req.payload[0] == 0x00  # channel
        assert req.payload[1:5] == (1000).to_bytes(4, "little", signed=True)
        assert req.payload[5:7] == (20).to_bytes(2, "little")


# ---------------------------------------------------------------------------
# I2C typed methods — ACK commands payload
# ---------------------------------------------------------------------------


class TestI2CTypedMethods:
    """Basic I2C method tests."""

    def test_i2c_configure_channel(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.i2c_configure_channel(dest=1, i2c_ch=0, speed_code=1)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x2B  # I2CConfigureChannel = 0x102B
        )
        assert req.payload == bytes([0x00, 0x01])

    def test_i2c_write_single_byte(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.i2c_write_single_byte(dest=1, i2c_ch=0, address=0x50, byte=0xFF)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x25  # I2CWriteSingleByte = 0x1025
        )
        assert req.payload == bytes([0x00, 0x50, 0xFF])


# ---------------------------------------------------------------------------
# DIO typed methods
# ---------------------------------------------------------------------------


class TestDIOTypedMethods:
    """DIO method payload assertions."""

    def test_set_dio_direction_output(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_dio_direction(dest=1, pin=3, output=True)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x03  # SetDIODirection = 0x1003
        )
        assert req.payload == bytes([0x03, 0x01])

    def test_set_single_dio_output(self) -> None:
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_single_dio_output(dest=1, pin=0, value=True)
        finally:
            hub.stop()

        req = next(
            r for r in hub.requests
            if r.packet_type == _DEKA_BASE + 0x01  # SetSingleDIOOutput = 0x1001
        )
        assert req.payload == bytes([0x00, 0x01])

    def test_get_single_dio_input(self) -> None:
        transport, hub, session = _make_session()
        hub.set_rsp("GetSingleDIOInput", inputValue=1)
        hub.run_in_thread()
        try:
            result = session.get_single_dio_input(dest=1, pin=5)
        finally:
            hub.stop()

        assert result is True
