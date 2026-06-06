"""Tests for device classes and P7 bug-fix regressions.

All tests are hardware-free, using FakeHub over LoopbackTransport.

Coverage:
- Motor.set_power(16000)              → FakeHub payload bytes 00 80 3E
- Servo.set_pulse_width(1500)         → FakeHub payload bytes 00 DC 05
- Servo.set_angle(90)                 → pulse_width ≈ 1500 → payload 00 DC 05
- Servo.set_pulse_width(499)          → raises ValueError (no transaction)
- DIOPin.get_direction()              → returns decoded bool (P7-b)
- Motor.get_velocity(bulk)            → reads from BulkInputData, no new transaction
- session.set_motor_pid_coefficients  → FakeHub receives SetMotorPIDCoefficients (P7-d)
- BulkInputData.from_response()       → negative encoder decodes correctly
- I2CDevice.read_register()           → write + poll sequence (NACK-41 × 2, then data)
- I2CChannel.configure(speed_code)    → session arg is not dropped (P7-a)
- OSC_CALIBRATE_VAL == 0xF8          → P7-f
- set_servo_pulse_width(0,1500)       → payload bytes 00 DC 05
- set_motor_constant_power(0,16000)   → payload bytes 00 80 3E
- get_dio_direction                   → returns decoded bool (P7-b via session)
- set_motor_pid_coefficients          → sends SetMotorPIDCoefficients (P7-d)
- I2CConfigureQuery_RSP registered    → P7-e (also tested in test_session.py)
"""

from __future__ import annotations

import threading
import time
from typing import Any

import pytest

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
from rhsp.codec import encode_payload
from rhsp.devices.adc import ADCPin
from rhsp.devices.bulk import BulkInputData, ModuleStatus
from rhsp.devices.dio import DIOPin
from rhsp.devices.i2c import I2CChannel, I2CDevice
from rhsp.devices.motor import Motor
from rhsp.devices.servo import Servo
from rhsp.framing import build_frame
from rhsp.session import Session
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_DEKA_BASE = 0x1000
_ACK_TYPE = 0x7F01

# SetServoPulseWidth = DEKA 0x21
_SET_SERVO_PW_TYPE = _DEKA_BASE + 0x21

# SetMotorConstantPower = DEKA 0x0F
_SET_MOTOR_POWER_TYPE = _DEKA_BASE + 0x0F

# SetMotorPIDCoefficients = DEKA 0x17
_SET_MOTOR_PID_TYPE = _DEKA_BASE + 0x17

# GetMotorPIDCoefficients = DEKA 0x18
_GET_MOTOR_PID_TYPE = _DEKA_BASE + 0x18


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_session(
    timeout: float = 0.5,
) -> tuple[LoopbackTransport, FakeHub, Session]:
    """Return (transport, hub, session) triple."""
    transport = LoopbackTransport()
    hub = FakeHub(transport)
    session = Session(transport, timeout=timeout)
    return transport, hub, session


def _inject_bulk_rsp(
    transport: LoopbackTransport,
    session: Session,
    *,
    motor0_velocity: int = 0,
    motor0_encoder: int = 0,
) -> "BulkInputData":
    """Synchronously inject a GetBulkInputData_RSP and decode it."""
    data = bytearray(141)
    # motor0Encoder at offset 1, 4 bytes LE signed
    data[1:5] = motor0_encoder.to_bytes(4, "little", signed=True)
    # motor0Velocity at offset 18, 2 bytes LE signed
    data[18:20] = motor0_velocity.to_bytes(2, "little", signed=True)
    payload = bytes(data)

    rsp_id = COMMANDS["GetBulkInputData"].reply_id
    expected_ref = session._msg_num

    result_holder: list[BulkInputData] = []

    def _inject() -> None:
        time.sleep(0.02)
        transport.inject(
            build_frame(dest=0, src=1, msg=0, ref=expected_ref, ptype=rsp_id, payload=payload)
        )

    t = threading.Thread(target=_inject, daemon=True)
    t.start()
    bulk = session.get_bulk_input_data(dest=1)
    t.join(timeout=1.0)
    return bulk


# ===========================================================================
# Motor device class
# ===========================================================================


class TestMotorDevice:
    """Motor device class tests."""

    def test_set_power_payload_bytes(self) -> None:
        """Motor.set_power(16000) → FakeHub payload bytes 00 80 3E."""
        transport, hub, session = _make_session()
        motor = Motor(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            motor.set_power(16000)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _SET_MOTOR_POWER_TYPE)
        assert req.payload == bytes([0x00, 0x80, 0x3E])

    def test_enable_calls_set_channel_enable_true(self) -> None:
        transport, hub, session = _make_session()
        motor = Motor(session, channel=2, address=1)
        hub.run_in_thread()
        try:
            motor.enable()
        finally:
            hub.stop()

        # SetMotorChannelEnable = DEKA 0x0A
        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x0A)
        assert req.payload == bytes([0x02, 0x01])  # channel=2, enabled=1

    def test_disable_calls_set_channel_enable_false(self) -> None:
        transport, hub, session = _make_session()
        motor = Motor(session, channel=1, address=1)
        hub.run_in_thread()
        try:
            motor.disable()
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x0A)
        assert req.payload == bytes([0x01, 0x00])  # channel=1, enabled=0

    def test_set_mode_constant_power(self) -> None:
        from rhsp.enums import MotorMode

        transport, hub, session = _make_session()
        motor = Motor(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            motor.set_mode(MotorMode.CONSTANT_POWER, float_at_zero=True)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x08)
        assert req.payload == bytes([0x00, 0x00, 0x01])

    def test_reset_encoder(self) -> None:
        transport, hub, session = _make_session()
        motor = Motor(session, channel=3, address=1)
        hub.run_in_thread()
        try:
            motor.reset_encoder()
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x0E)
        assert req.payload == bytes([0x03])

    def test_get_velocity_reads_from_bulk_no_new_transaction(self) -> None:
        """Motor.get_velocity(bulk) reads from BulkInputData — no new transaction."""
        transport, hub, session = _make_session()
        motor = Motor(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            bulk = _inject_bulk_rsp(transport, session, motor0_velocity=300)
        finally:
            hub.stop()

        # Count transactions issued so far (the get_bulk_input_data itself).
        n_before = len(hub.requests)

        # get_velocity must not issue a new transaction.
        vel = motor.get_velocity(bulk)

        n_after = len(hub.requests)
        assert n_after == n_before, (
            f"get_velocity issued {n_after - n_before} new transaction(s)"
        )
        assert vel == 300

    def test_get_velocity_returns_correct_channel(self) -> None:
        """Motor.get_velocity reads motorN_velocity for the correct channel."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            bulk = _inject_bulk_rsp(transport, session, motor0_velocity=-250)
        finally:
            hub.stop()

        motor0 = Motor(session, channel=0, address=1)
        motor1 = Motor(session, channel=1, address=1)

        # motor0 velocity from bulk
        assert motor0.get_velocity(bulk) == -250
        # motor1 velocity was 0 in our injected payload
        assert motor1.get_velocity(bulk) == 0

    def test_get_velocity_does_not_call_get_bulk_motor_data(self) -> None:
        """get_velocity must NOT use GetBulkMotorData (0x37)."""
        # GetBulkMotorData = DEKA 0x37
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            bulk = _inject_bulk_rsp(transport, session)
        finally:
            hub.stop()

        motor = Motor(session, channel=0, address=1)
        motor.get_velocity(bulk)

        # Assert no GetBulkMotorData request was ever issued.
        get_bulk_motor_data_type = _DEKA_BASE + 0x37
        assert not any(r.packet_type == get_bulk_motor_data_type for r in hub.requests)


# ===========================================================================
# Servo device class
# ===========================================================================


class TestServoDevice:
    """Servo device class tests."""

    def test_set_pulse_width_payload_bytes(self) -> None:
        """Servo.set_pulse_width(1500) → FakeHub payload bytes 00 DC 05."""
        transport, hub, session = _make_session()
        servo = Servo(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            servo.set_pulse_width(1500)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _SET_SERVO_PW_TYPE)
        assert req.payload == bytes([0x00, 0xDC, 0x05])

    def test_set_angle_90_payload_bytes(self) -> None:
        """Servo.set_angle(90) → pulse_width ≈ 1500 → payload 00 DC 05."""
        transport, hub, session = _make_session()
        servo = Servo(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            servo.set_angle(90.0)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _SET_SERVO_PW_TYPE)
        # 500 + 90 * 2000/180 = 1500.0 exactly
        assert req.payload == bytes([0x00, 0xDC, 0x05])

    def test_set_pulse_width_499_raises_value_error(self) -> None:
        """Servo.set_pulse_width(499) raises ValueError before any transaction."""
        transport, hub, session = _make_session()
        servo = Servo(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            with pytest.raises(ValueError, match="out of range"):
                servo.set_pulse_width(499)
        finally:
            hub.stop()

        # No servo pulse-width frame should have been sent.
        assert not any(r.packet_type == _SET_SERVO_PW_TYPE for r in hub.requests)

    def test_set_pulse_width_2501_raises_value_error(self) -> None:
        """Servo.set_pulse_width(2501) raises ValueError."""
        transport, hub, session = _make_session()
        servo = Servo(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            with pytest.raises(ValueError):
                servo.set_pulse_width(2501)
        finally:
            hub.stop()

    def test_set_pulse_width_boundaries(self) -> None:
        """Servo.set_pulse_width(500) and (2500) succeed (boundary values)."""
        transport, hub, session = _make_session()
        servo = Servo(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            servo.set_pulse_width(500)
            servo.set_pulse_width(2500)
        finally:
            hub.stop()

    def test_enable_disable(self) -> None:
        transport, hub, session = _make_session()
        servo = Servo(session, channel=1, address=1)
        hub.run_in_thread()
        try:
            servo.enable()
            servo.disable()
        finally:
            hub.stop()

        # SetServoEnable = DEKA 0x23
        reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x23]
        assert len(reqs) == 2
        assert reqs[0].payload == bytes([0x01, 0x01])  # enable
        assert reqs[1].payload == bytes([0x01, 0x00])  # disable

    def test_set_configuration(self) -> None:
        transport, hub, session = _make_session()
        servo = Servo(session, channel=2, address=1)
        hub.run_in_thread()
        try:
            servo.set_configuration(20000)
        finally:
            hub.stop()

        # SetServoConfiguration = DEKA 0x1F
        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x1F)
        # channel=2, frame_period=20000 LE 2-byte = 0x20 0x4E
        assert req.payload[0] == 0x02
        assert req.payload[1:3] == (20000).to_bytes(2, "little")


# ===========================================================================
# DIOPin device class — P7-b
# ===========================================================================


class TestDIOPinDevice:
    """DIOPin device class tests (including P7-b get_direction return)."""

    def test_get_direction_returns_true_when_output(self) -> None:
        """DIOPin.get_direction() returns True when directionOutput=1 (P7-b)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetDIODirection", directionOutput=1)
        pin = DIOPin(session, pin=3, address=1)
        hub.run_in_thread()
        try:
            result = pin.get_direction()
        finally:
            hub.stop()

        assert isinstance(result, bool)
        assert result is True

    def test_get_direction_returns_false_when_input(self) -> None:
        """DIOPin.get_direction() returns False when directionOutput=0 (P7-b)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetDIODirection", directionOutput=0)
        pin = DIOPin(session, pin=0, address=1)
        hub.run_in_thread()
        try:
            result = pin.get_direction()
        finally:
            hub.stop()

        assert result is False

    def test_set_direction_output(self) -> None:
        transport, hub, session = _make_session()
        pin = DIOPin(session, pin=5, address=1)
        hub.run_in_thread()
        try:
            pin.set_direction(True)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x03)
        assert req.payload == bytes([0x05, 0x01])

    def test_write_and_read(self) -> None:
        transport, hub, session = _make_session()
        hub.set_rsp("GetSingleDIOInput", inputValue=1)
        pin = DIOPin(session, pin=2, address=1)
        hub.run_in_thread()
        try:
            pin.write(True)
            result = pin.read()
        finally:
            hub.stop()

        assert result is True


# ===========================================================================
# ADCPin device class
# ===========================================================================


class TestADCPinDevice:
    """ADCPin device class tests."""

    def test_read_returns_int(self) -> None:
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=3300)
        pin = ADCPin(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            result = pin.read()
        finally:
            hub.stop()

        assert isinstance(result, int)
        assert result == 3300

    def test_read_raw_mode_payload(self) -> None:
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=512)
        pin = ADCPin(session, channel=1, address=1)
        hub.run_in_thread()
        try:
            pin.read(raw=True)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x07)
        assert req.payload[1] == 1  # rawMode=1


# ===========================================================================
# I2CChannel device class — P7-a
# ===========================================================================


class TestI2CChannelDevice:
    """I2CChannel.configure() passes session correctly (P7-a)."""

    def test_configure_passes_session_correctly(self) -> None:
        """I2CChannel.configure(1) calls session.i2c_configure_channel (P7-a)."""
        transport, hub, session = _make_session()
        i2c = I2CChannel(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            i2c.configure(1)  # FAST_400KHZ
        finally:
            hub.stop()

        # I2CConfigureChannel = DEKA 0x2B
        req = next(r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x2B)
        assert req.payload == bytes([0x00, 0x01])  # i2c_channel=0, speed_code=1

    def test_device_returns_i2c_device(self) -> None:
        transport, hub, session = _make_session()
        i2c = I2CChannel(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            dev = i2c.device(0x29)
        finally:
            hub.stop()

        assert isinstance(dev, I2CDevice)
        assert dev.address == 0x29
        assert dev.i2c_channel == 0
        assert dev.dest == 1


# ===========================================================================
# I2CDevice — read_register two-phase poll
# ===========================================================================


class TestI2CDeviceReadRegister:
    """I2CDevice.read_register() issues write-then-poll sequence."""

    def _make_i2c_device(self) -> tuple[LoopbackTransport, FakeHub, Session, I2CDevice]:
        transport, hub, session = _make_session()
        dev = I2CDevice(session, i2c_channel=0, dest=1, address=0x29)
        return transport, hub, session, dev

    def test_write_multiple_bytes_then_read_issued(self) -> None:
        """read_register issues I2CWriteMultipleBytes then I2CReadMultipleBytes."""
        transport, hub, session, dev = self._make_i2c_device()
        # Set the read status to return data on the first poll.
        hub.set_rsp("I2CReadStatusQuery", i2cStatus=0, byteRead=2, payloadBytes=bytes([0xAB, 0xCD]))
        hub.run_in_thread()
        try:
            data = dev.read_register(0x0D, 2)
        finally:
            hub.stop()

        # Verify write was issued (I2CWriteMultipleBytes = DEKA 0x26).
        write_reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x26]
        assert len(write_reqs) >= 1

        # Verify read was issued (I2CReadMultipleBytes = DEKA 0x28).
        read_reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x28]
        assert len(read_reqs) >= 1

    def test_poll_nack41_twice_then_data(self) -> None:
        """FakeHub returns NACK-41 twice then data — 3 total status query calls."""
        transport, hub, session = _make_session()
        dev = I2CDevice(session, i2c_channel=0, dest=1, address=0x29)

        # Simulate NACK-41 twice, then data on third poll.
        poll_count = [0]

        original_handle = hub._handle

        def _patched_handle(pkt: Any) -> None:
            # Intercept I2CReadStatusQuery packets.
            cmd_name_map = {cmd.id: name for name, cmd in COMMANDS.items()}
            cname = cmd_name_map.get(pkt.packet_type)
            if cname == "I2CReadStatusQuery":
                poll_count[0] += 1
                if poll_count[0] <= 2:
                    # Return i2cStatus=41 (in progress).
                    hub._rsp_config["I2CReadStatusQuery"] = {
                        "i2cStatus": 41, "byteRead": 0, "payloadBytes": b""
                    }
                else:
                    # Return actual data.
                    hub._rsp_config["I2CReadStatusQuery"] = {
                        "i2cStatus": 0, "byteRead": 3, "payloadBytes": bytes([0x01, 0x02, 0x03])
                    }
            original_handle(pkt)

        hub._handle = _patched_handle
        hub.run_in_thread()
        try:
            data = dev.read_register(0x0D, 3)
        finally:
            hub.stop()

        assert poll_count[0] == 3
        assert data[:3] == bytes([0x01, 0x02, 0x03])

    def test_write_register_payload(self) -> None:
        """I2CDevice.write_register(reg, data) encodes reg+data as one write."""
        transport, hub, session = _make_session()
        dev = I2CDevice(session, i2c_channel=1, dest=1, address=0x50)
        hub.run_in_thread()
        try:
            dev.write_register(0x03, bytes([0xAA, 0xBB]))
        finally:
            hub.stop()

        # I2CWriteMultipleBytes = DEKA 0x26
        write_reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x26]
        assert len(write_reqs) >= 1
        # payload: i2c_channel=1, address=0x50, num_bytes=3, bytes=[0x03, 0xAA, 0xBB]
        payload = write_reqs[0].payload
        assert payload[0] == 0x01   # i2c_channel=1
        assert payload[1] == 0x50   # address=0x50
        # num_bytes = 3
        assert payload[2] == 0x03
        assert payload[3:6] == bytes([0x03, 0xAA, 0xBB])


# ===========================================================================
# P7-d — set_motor_pid_coefficients calls SetMotorPIDCoefficients (the setter)
# ===========================================================================


class TestSetMotorPIDCoefficientsBug:
    """P7-d: set_motor_pid_coefficients calls SetMotorPIDCoefficients, not the getter."""

    def test_pid_coefficients_sends_setter_packet(self) -> None:
        """session.set_motor_pid_coefficients sends SetMotorPIDCoefficients (DEKA 0x17)."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_motor_pid_coefficients(
                dest=1, channel=0, mode=0, p=1.5, i=0.0, d=0.0
            )
        finally:
            hub.stop()

        # Verify the SETTER type was issued.
        setter_reqs = [r for r in hub.requests if r.packet_type == _SET_MOTOR_PID_TYPE]
        getter_reqs = [r for r in hub.requests if r.packet_type == _GET_MOTOR_PID_TYPE]
        assert len(setter_reqs) == 1, f"Expected 1 setter packet, got {len(setter_reqs)}"
        assert len(getter_reqs) == 0, f"Expected 0 getter packets, got {len(getter_reqs)}"


# ===========================================================================
# BulkInputData.from_response() — negative encoder / velocity
# ===========================================================================


class TestBulkInputDataFromResponse:
    """BulkInputData.from_response() correctly decodes signed fields."""

    def test_negative_encoder_minus_one(self) -> None:
        """Encoder value 0xFFFFFFFF decodes as -1 (signed 32-bit)."""
        rsp: dict = {
            "digitalInputs": 0,
            "motor0Encoder": 0xFFFFFFFF,
            "motor1Encoder": 0,
            "motor2Encoder": 0,
            "motor3Encoder": 0,
            "motorStatus": 0,
            "motor0Velocity": 0,
            "motor1Velocity": 0,
            "motor2Velocity": 0,
            "motor3Velocity": 0,
            "motor0mode": 0,
            "motor1mode": 0,
            "motor2mode": 0,
            "motor3mode": 0,
            "analogInput0": 0,
            "analogInput1": 0,
            "analogInput2": 0,
            "analogInput3": 0,
            "gpioCurrent_mA": 0,
            "i2cCurrent_mA": 0,
            "servoCurrent_mA": 0,
            "batteryCurrent_mA": 0,
            "motor0current_mA": 0,
            "motor1current_mA": 0,
            "motor2current_mA": 0,
            "motor3current_mA": 0,
            "mon5v_mV": 0,
            "batteryVoltage_mV": 0,
            "servo0cmd": 0,
            "servo1cmd": 0,
            "servo2cmd": 0,
            "servo3cmd": 0,
            "servo4cmd": 0,
            "servo5cmd": 0,
            "servo0framePeriod_us": 0,
            "servo1framePeriod_us": 0,
            "servo2framePeriod_us": 0,
            "servo3framePeriod_us": 0,
            "servo4framePeriod_us": 0,
            "servo5framePeriod_us": 0,
            "i2c0data": b"\x00" * 10,
            "i2c1data": b"\x00" * 10,
            "i2c2data": b"\x00" * 10,
            "i2c3data": b"\x00" * 10,
            "imuBlock": b"\x00" * 10,
            "i2c0Status": 0,
            "i2c1Status": 0,
            "i2c2Status": 0,
            "i2c3Status": 0,
            "imuStatus": 0,
            "mototonicTime": 0,
        }
        bulk = BulkInputData.from_response(rsp)
        assert bulk.motor0_encoder == -1

    def test_negative_velocity(self) -> None:
        """Velocity value 0xFFFE decodes as -2 (signed 16-bit)."""
        rsp: dict = {
            "digitalInputs": 0,
            "motor0Encoder": 0,
            "motor1Encoder": 0,
            "motor2Encoder": 0,
            "motor3Encoder": 0,
            "motorStatus": 0,
            "motor0Velocity": 0xFFFE,  # -2 as unsigned 16-bit
            "motor1Velocity": 0,
            "motor2Velocity": 0,
            "motor3Velocity": 0,
            "motor0mode": 0,
            "motor1mode": 0,
            "motor2mode": 0,
            "motor3mode": 0,
            "analogInput0": 0,
            "analogInput1": 0,
            "analogInput2": 0,
            "analogInput3": 0,
            "gpioCurrent_mA": 0,
            "i2cCurrent_mA": 0,
            "servoCurrent_mA": 0,
            "batteryCurrent_mA": 0,
            "motor0current_mA": 0,
            "motor1current_mA": 0,
            "motor2current_mA": 0,
            "motor3current_mA": 0,
            "mon5v_mV": 0,
            "batteryVoltage_mV": 0,
            "servo0cmd": 0,
            "servo1cmd": 0,
            "servo2cmd": 0,
            "servo3cmd": 0,
            "servo4cmd": 0,
            "servo5cmd": 0,
            "servo0framePeriod_us": 0,
            "servo1framePeriod_us": 0,
            "servo2framePeriod_us": 0,
            "servo3framePeriod_us": 0,
            "servo4framePeriod_us": 0,
            "servo5framePeriod_us": 0,
            "i2c0data": b"\x00" * 10,
            "i2c1data": b"\x00" * 10,
            "i2c2data": b"\x00" * 10,
            "i2c3data": b"\x00" * 10,
            "imuBlock": b"\x00" * 10,
            "i2c0Status": 0,
            "i2c1Status": 0,
            "i2c2Status": 0,
            "i2c3Status": 0,
            "imuStatus": 0,
            "mototonicTime": 0,
        }
        bulk = BulkInputData.from_response(rsp)
        assert bulk.motor0_velocity == -2

    def test_monotonic_time_typo_handled(self) -> None:
        """BulkInputData.from_response handles the 'mototonicTime' vendor typo."""
        rsp: dict = {
            "digitalInputs": 0,
            "motor0Encoder": 0,
            "motor1Encoder": 0,
            "motor2Encoder": 0,
            "motor3Encoder": 0,
            "motorStatus": 0,
            "motor0Velocity": 0,
            "motor1Velocity": 0,
            "motor2Velocity": 0,
            "motor3Velocity": 0,
            "motor0mode": 0,
            "motor1mode": 0,
            "motor2mode": 0,
            "motor3mode": 0,
            "analogInput0": 0,
            "analogInput1": 0,
            "analogInput2": 0,
            "analogInput3": 0,
            "gpioCurrent_mA": 0,
            "i2cCurrent_mA": 0,
            "servoCurrent_mA": 0,
            "batteryCurrent_mA": 0,
            "motor0current_mA": 0,
            "motor1current_mA": 0,
            "motor2current_mA": 0,
            "motor3current_mA": 0,
            "mon5v_mV": 0,
            "batteryVoltage_mV": 0,
            "servo0cmd": 0,
            "servo1cmd": 0,
            "servo2cmd": 0,
            "servo3cmd": 0,
            "servo4cmd": 0,
            "servo5cmd": 0,
            "servo0framePeriod_us": 0,
            "servo1framePeriod_us": 0,
            "servo2framePeriod_us": 0,
            "servo3framePeriod_us": 0,
            "servo4framePeriod_us": 0,
            "servo5framePeriod_us": 0,
            "i2c0data": b"\x00" * 10,
            "i2c1data": b"\x00" * 10,
            "i2c2data": b"\x00" * 10,
            "i2c3data": b"\x00" * 10,
            "imuBlock": b"\x00" * 10,
            "i2c0Status": 0,
            "i2c1Status": 0,
            "i2c2Status": 0,
            "i2c3Status": 0,
            "imuStatus": 0,
            "mototonicTime": 12345,  # vendor typo key
        }
        bulk = BulkInputData.from_response(rsp)
        assert bulk.monotonic_time_ms == 12345


# ===========================================================================
# P7-e — I2CConfigureQuery_RSP registered in RESPONSES_BY_ID
# ===========================================================================


class TestI2CConfigureQueryRegistration:
    """P7-e: I2CConfigureQuery_RSP is registered in RESPONSES_BY_ID."""

    def test_response_registered(self) -> None:
        assert 0x902F in RESPONSES_BY_ID
        rsp = RESPONSES_BY_ID[0x902F]
        assert rsp.name == "I2CConfigureQuery_RSP"

    def test_i2c_configure_query_decodes_response(self) -> None:
        """session.i2c_configure_query() correctly decodes the response."""
        transport, hub, session = _make_session()
        hub.set_rsp("I2CConfigureQuery", speedCode=0)
        hub.run_in_thread()
        try:
            result = session.i2c_configure_query(dest=1, i2c_ch=2)
        finally:
            hub.stop()

        assert isinstance(result, int)
        assert result == 0  # STANDARD_100KHZ


# ===========================================================================
# P7-f — OSC_CALIBRATE_VAL == 0xF8
# ===========================================================================


class TestOscCalibrateVal:
    """P7-f: OSC_CALIBRATE_VAL == 0xF8."""

    def test_osc_calibrate_val_is_0xf8(self) -> None:
        from rhsp.sensors.registers import OSC_CALIBRATE_VAL

        assert OSC_CALIBRATE_VAL == 0xF8


# ===========================================================================
# Session payload byte-exact assertions (also proved in test_session.py;
# duplicated here per ticket requirement)
# ===========================================================================


class TestSessionPayloadByteExact:
    """Byte-exact payload assertions for session typed methods."""

    def test_set_servo_pulse_width_payload(self) -> None:
        """set_servo_pulse_width(dest=1, channel=0, pulse_width=1500) → 00 DC 05."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_servo_pulse_width(dest=1, channel=0, pulse_width=1500)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _SET_SERVO_PW_TYPE)
        assert req.payload == bytes([0x00, 0xDC, 0x05])

    def test_set_motor_constant_power_payload(self) -> None:
        """set_motor_constant_power(dest=1, channel=0, power=16000) → 00 80 3E."""
        transport, hub, session = _make_session()
        hub.run_in_thread()
        try:
            session.set_motor_constant_power(dest=1, channel=0, power=16000)
        finally:
            hub.stop()

        req = next(r for r in hub.requests if r.packet_type == _SET_MOTOR_POWER_TYPE)
        assert req.payload == bytes([0x00, 0x80, 0x3E])

    def test_get_dio_direction_returns_bool_p7b(self) -> None:
        """get_dio_direction returns decoded bool (P7-b)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetDIODirection", directionOutput=1)
        hub.run_in_thread()
        try:
            result = session.get_dio_direction(dest=1, pin=0)
        finally:
            hub.stop()

        assert isinstance(result, bool)
        assert result is True


# ===========================================================================
# Motor.get_current_ma — 003-006
# ===========================================================================


class TestMotorGetCurrentMa:
    """Motor.get_current_ma() reads from GetADC(ADC_Motor0 + channel)."""

    def test_channel0_reads_adc_motor0(self) -> None:
        """Motor ch 0 issues GetADC for channel 8 (ADC_Motor0)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=3)
        motor = Motor(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            result = motor.get_current_ma()
        finally:
            hub.stop()

        # Verify the GetADC request used adcChannel=8 (ADC_Motor0).
        # GetADC = DEKA 0x07.
        adc_reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x07]
        assert len(adc_reqs) == 1
        # payload[0] = adcChannel, payload[1] = rawMode
        assert adc_reqs[0].payload[0] == 8  # ADC_Motor0
        assert adc_reqs[0].payload[1] == 0  # engineering units
        assert isinstance(result, int)
        assert result == 3

    def test_channel1_reads_adc_motor1(self) -> None:
        """Motor ch 1 issues GetADC for channel 9 (ADC_Motor1)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=5)
        motor = Motor(session, channel=1, address=1)
        hub.run_in_thread()
        try:
            result = motor.get_current_ma()
        finally:
            hub.stop()

        adc_reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x07]
        assert adc_reqs[0].payload[0] == 9  # ADC_Motor1
        assert result == 5

    def test_channel2_reads_adc_motor2(self) -> None:
        """Motor ch 2 issues GetADC for channel 10 (ADC_Motor2)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=0)
        motor = Motor(session, channel=2, address=1)
        hub.run_in_thread()
        try:
            motor.get_current_ma()
        finally:
            hub.stop()

        adc_reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x07]
        assert adc_reqs[0].payload[0] == 10  # ADC_Motor2

    def test_channel3_reads_adc_motor3(self) -> None:
        """Motor ch 3 issues GetADC for channel 11 (ADC_Motor3)."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=2)
        motor = Motor(session, channel=3, address=1)
        hub.run_in_thread()
        try:
            result = motor.get_current_ma()
        finally:
            hub.stop()

        adc_reqs = [r for r in hub.requests if r.packet_type == _DEKA_BASE + 0x07]
        assert adc_reqs[0].payload[0] == 11  # ADC_Motor3
        assert result == 2

    def test_returns_int(self) -> None:
        """get_current_ma() always returns a plain int."""
        transport, hub, session = _make_session()
        hub.set_rsp("GetADC", adcValue=42)
        motor = Motor(session, channel=0, address=1)
        hub.run_in_thread()
        try:
            result = motor.get_current_ma()
        finally:
            hub.stop()

        assert type(result) is int


# ===========================================================================
# Hub.battery_voltage_mv and Hub.battery_current_ma — 003-006
# ===========================================================================


class TestHubBatteryGetters:
    """Hub battery voltage and current helpers read from the correct ADC channels."""

    def _make_hub_obj(
        self, *, adc_value: int = 0
    ) -> tuple[LoopbackTransport, "FakeHub", Session, object]:
        """Return (transport, fake_hub, session, hub) with FakeHub running."""
        from rhsp.hub import Hub

        transport, fake, session = _make_session()
        fake.set_rsp("GetADC", adcValue=adc_value)
        hub = Hub(session=session, address=1)
        fake.run_in_thread()
        return transport, fake, session, hub

    def test_battery_voltage_mv_reads_adc_channel_13(self) -> None:
        """Hub.battery_voltage_mv() issues GetADC for channel 13 (ADC_BatteryMonitor)."""
        from rhsp.hub import Hub

        transport, fake, session = _make_session()
        fake.set_rsp("GetADC", adcValue=11934)
        hub = Hub(session=session, address=1)
        fake.run_in_thread()
        try:
            result = hub.battery_voltage_mv()
        finally:
            fake.stop()

        adc_reqs = [r for r in fake.requests if r.packet_type == _DEKA_BASE + 0x07]
        assert len(adc_reqs) == 1
        assert adc_reqs[0].payload[0] == 13  # ADC_BatteryMonitor
        assert adc_reqs[0].payload[1] == 0   # engineering units
        assert isinstance(result, int)
        assert result == 11934

    def test_battery_current_ma_reads_adc_channel_7(self) -> None:
        """Hub.battery_current_ma() issues GetADC for channel 7 (ADC_Battery)."""
        from rhsp.hub import Hub

        transport, fake, session = _make_session()
        fake.set_rsp("GetADC", adcValue=0)
        hub = Hub(session=session, address=1)
        fake.run_in_thread()
        try:
            result = hub.battery_current_ma()
        finally:
            fake.stop()

        adc_reqs = [r for r in fake.requests if r.packet_type == _DEKA_BASE + 0x07]
        assert len(adc_reqs) == 1
        assert adc_reqs[0].payload[0] == 7  # ADC_Battery
        assert isinstance(result, int)
        assert result == 0

    def test_battery_voltage_mv_returns_int(self) -> None:
        """battery_voltage_mv() always returns a plain int."""
        from rhsp.hub import Hub

        transport, fake, session = _make_session()
        fake.set_rsp("GetADC", adcValue=12100)
        hub = Hub(session=session, address=1)
        fake.run_in_thread()
        try:
            result = hub.battery_voltage_mv()
        finally:
            fake.stop()

        assert type(result) is int
        assert result == 12100

    def test_battery_current_ma_returns_int(self) -> None:
        """battery_current_ma() always returns a plain int."""
        from rhsp.hub import Hub

        transport, fake, session = _make_session()
        fake.set_rsp("GetADC", adcValue=150)
        hub = Hub(session=session, address=1)
        fake.run_in_thread()
        try:
            result = hub.battery_current_ma()
        finally:
            fake.stop()

        assert type(result) is int
        assert result == 150


# ===========================================================================
# BulkInputData — current/voltage fields removed (003-006)
# ===========================================================================


class TestBulkCurrentVoltageFieldsRemoved:
    """Confirm that removed current/voltage fields are no longer on BulkInputData."""

    def test_motor_current_ma_attributes_absent(self) -> None:
        """motor*_current_ma attributes must not exist on BulkInputData."""
        from rhsp.devices.bulk import BulkInputData

        bulk = BulkInputData.from_response({})
        for ch in range(4):
            assert not hasattr(bulk, f"motor{ch}_current_ma"), (
                f"motor{ch}_current_ma must have been removed from BulkInputData"
            )

    def test_battery_voltage_mv_attribute_absent(self) -> None:
        """battery_voltage_mv must not exist on BulkInputData."""
        from rhsp.devices.bulk import BulkInputData

        bulk = BulkInputData.from_response({})
        assert not hasattr(bulk, "battery_voltage_mv")

    def test_battery_current_ma_attribute_absent(self) -> None:
        """battery_current_ma must not exist on BulkInputData."""
        from rhsp.devices.bulk import BulkInputData

        bulk = BulkInputData.from_response({})
        assert not hasattr(bulk, "battery_current_ma")

    def test_mon5v_mv_attribute_absent(self) -> None:
        """mon5v_mv must not exist on BulkInputData."""
        from rhsp.devices.bulk import BulkInputData

        bulk = BulkInputData.from_response({})
        assert not hasattr(bulk, "mon5v_mv")

    def test_bulk_input_data_still_decodes_encoders_velocities_modes(self) -> None:
        """BulkInputData.from_response still correctly decodes encoders, velocities, and modes."""
        from rhsp.devices.bulk import BulkInputData

        rsp = {
            "digitalInputs": 0b10101010,
            "motor0Encoder": 0xFFFFFFFF,  # -1 signed
            "motor1Encoder": 1000,
            "motor2Encoder": 0,
            "motor3Encoder": -500,
            "motorStatus": 0,
            "motor0Velocity": 0xFFFE,  # -2 signed
            "motor1Velocity": 300,
            "motor2Velocity": 0,
            "motor3Velocity": 0,
            "motor0mode": 1,
            "motor1mode": 0,
            "motor2mode": 0,
            "motor3mode": 0,
            "analogInput0": 3300,
            "analogInput1": 0,
        }
        bulk = BulkInputData.from_response(rsp)

        assert bulk.digital_inputs == 0b10101010
        assert bulk.motor0_encoder == -1
        assert bulk.motor1_encoder == 1000
        assert bulk.motor0_velocity == -2
        assert bulk.motor1_velocity == 300
        assert bulk.motor0_mode == 1
        assert bulk.analog_input0 == 3300
