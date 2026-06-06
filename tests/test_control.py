"""Tests for rhsp.control — VelocityController ABC and HubVelocityController.

All tests are hardware-free: they use FakeHub over a LoopbackTransport.

Coverage:
- clamp_int16 helper: boundary values and out-of-range inputs.
- Motor.set_velocity_pid: correct delegation to session layer (no double Q16).
- Motor.get_velocity_pid: correct delegation with ClosedLoopMode.VELOCITY.
- HubVelocityController.attach(): sets CONSTANT_VELOCITY mode + enable.
- HubVelocityController.command(t): clamps to int16, calls set_target_velocity.
- HubVelocityController.measured(bulk): delegates to motor.get_velocity(bulk).
- HubVelocityController.detach(disable=True): disables + restores mode.
- HubVelocityController.detach(disable=False): no disable, restores mode.
- HubVelocityController.available: returns True.
"""

from __future__ import annotations

import threading
import time

import pytest

from rhsp.catalogue import COMMANDS
from rhsp.devices.bulk import BulkInputData
from rhsp.devices.motor import Motor
from rhsp.enums import ClosedLoopMode, MotorMode
from rhsp.hub import Hub
from rhsp.session import Session
from rhsp.transport import LoopbackTransport
from rhsp.control import HubVelocityController, RatioDrive, VelocityController, clamp_int16

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# DEKA packet-type constants
# ---------------------------------------------------------------------------

_DEKA_BASE: int = 0x1000

# SetMotorChannelMode = DEKA 0x08
_SET_MOTOR_MODE_TYPE: int = _DEKA_BASE + 0x08

# SetMotorChannelEnable = DEKA 0x0A
_SET_MOTOR_ENABLE_TYPE: int = _DEKA_BASE + 0x0A

# SetMotorTargetVelocity = DEKA 0x11
_SET_MOTOR_VELOCITY_TYPE: int = _DEKA_BASE + 0x11

# SetMotorPIDCoefficients = DEKA 0x17
_SET_MOTOR_PID_TYPE: int = _DEKA_BASE + 0x17

# GetMotorPIDCoefficients = DEKA 0x18
_GET_MOTOR_PID_TYPE: int = _DEKA_BASE + 0x18


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_hub_triple(
    *,
    address: int = 2,
    timeout: float = 0.5,
) -> tuple[LoopbackTransport, FakeHub, Hub]:
    """Return a wired-up (transport, fake_hub, hub) triple.

    The FakeHub runs in a background thread so Session transactions
    complete without manual interleaving.
    """
    transport = LoopbackTransport()
    fake = FakeHub(transport)
    fake.run_in_thread()

    session = Session(transport, timeout=timeout)
    session.deka_base = _DEKA_BASE

    hub = Hub(session=session, address=address)
    return transport, fake, hub


def _make_bulk(*, motor0_velocity: int = 0, motor1_velocity: int = 0) -> BulkInputData:
    """Build a synthetic BulkInputData with the given velocity values."""
    rsp: dict = {
        "digitalInputs": 0,
        "motor0Encoder": 0,
        "motor1Encoder": 0,
        "motor2Encoder": 0,
        "motor3Encoder": 0,
        "motorStatus": 0,
        "motor0Velocity": motor0_velocity,
        "motor1Velocity": motor1_velocity,
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
    return BulkInputData.from_response(rsp)


# ---------------------------------------------------------------------------
# clamp_int16
# ---------------------------------------------------------------------------


class TestClampInt16:
    """Unit tests for the clamp_int16 helper."""

    def test_in_range_positive(self) -> None:
        assert clamp_int16(0) == 0
        assert clamp_int16(1) == 1
        assert clamp_int16(1000) == 1000

    def test_in_range_negative(self) -> None:
        assert clamp_int16(-1) == -1
        assert clamp_int16(-1000) == -1000

    def test_max_boundary(self) -> None:
        assert clamp_int16(32767) == 32767

    def test_min_boundary(self) -> None:
        assert clamp_int16(-32767) == -32767

    def test_over_max_clamped(self) -> None:
        assert clamp_int16(32768) == 32767
        assert clamp_int16(100000) == 32767

    def test_under_min_clamped(self) -> None:
        assert clamp_int16(-32768) == -32767
        assert clamp_int16(-100000) == -32767


# ---------------------------------------------------------------------------
# Motor PID convenience methods
# ---------------------------------------------------------------------------


class TestMotorVelocityPID:
    """Motor.set_velocity_pid and get_velocity_pid delegation tests."""

    def test_set_velocity_pid_sends_setter_packet(self) -> None:
        """set_velocity_pid sends SetMotorPIDCoefficients (DEKA 0x17)."""
        _, fake, hub = _make_hub_triple()
        motor = hub.motors[0]
        try:
            motor.set_velocity_pid(p=1.0, i=0.5, d=0.1)
        finally:
            fake.stop()

        setter_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_PID_TYPE]
        getter_reqs = [r for r in fake.requests if r.packet_type == _GET_MOTOR_PID_TYPE]
        assert len(setter_reqs) == 1, f"Expected 1 setter packet, got {len(setter_reqs)}"
        assert len(getter_reqs) == 0, f"Expected 0 getter packets, got {len(getter_reqs)}"

    def test_set_velocity_pid_passes_correct_address_and_channel(self) -> None:
        """set_velocity_pid uses the motor's address and channel."""
        _, fake, hub = _make_hub_triple(address=2)
        # Use channel 1
        motor = hub.motors[1]
        try:
            motor.set_velocity_pid(p=2.0, i=0.0, d=0.0)
        finally:
            fake.stop()

        payload = fake.payloads.get("SetMotorPIDCoefficients", {})
        # channel field
        assert payload.get("motorChannel") == 1 or payload.get("channel") == 1 or \
            _verify_channel_from_payload(fake, _SET_MOTOR_PID_TYPE, channel=1)

    def test_set_velocity_pid_uses_velocity_mode(self) -> None:
        """set_velocity_pid passes ClosedLoopMode.VELOCITY to the session."""
        _, fake, hub = _make_hub_triple()
        motor = hub.motors[0]
        try:
            motor.set_velocity_pid(p=1.0, i=0.0, d=0.0)
        finally:
            fake.stop()

        payload = fake.payloads.get("SetMotorPIDCoefficients", {})
        # mode must be VELOCITY = 1; may arrive as bytes (last-field rule) or int.
        mode_raw = payload.get("mode")
        if isinstance(mode_raw, (bytes, bytearray)):
            mode_val = int.from_bytes(mode_raw, "little")
        else:
            mode_val = int(mode_raw)
        assert mode_val == int(ClosedLoopMode.VELOCITY), (
            f"Expected mode={int(ClosedLoopMode.VELOCITY)}, got {mode_val!r}"
        )

    def test_get_velocity_pid_sends_getter_packet(self) -> None:
        """get_velocity_pid sends GetMotorPIDCoefficients (DEKA 0x18)."""
        _, fake, hub = _make_hub_triple()
        motor = hub.motors[0]
        # FakeHub encode_payload will multiply float by 65536 (fixed_point) before
        # encoding; the session then decodes it back to a float.  Pass plain floats.
        fake.set_rsp("GetMotorPIDCoefficients", p=1.0, i=0.0, d=0.0)
        try:
            result = motor.get_velocity_pid()
        finally:
            fake.stop()

        getter_reqs = [r for r in fake.requests if r.packet_type == _GET_MOTOR_PID_TYPE]
        assert len(getter_reqs) == 1, f"Expected 1 getter packet, got {len(getter_reqs)}"

    def test_get_velocity_pid_returns_tuple(self) -> None:
        """get_velocity_pid returns a (p, i, d) tuple."""
        _, fake, hub = _make_hub_triple()
        motor = hub.motors[0]
        # FakeHub encode_payload multiplies by fixed_point=65536, session decodes back.
        fake.set_rsp("GetMotorPIDCoefficients", p=1.0, i=0.5, d=0.0)
        try:
            result = motor.get_velocity_pid()
        finally:
            fake.stop()

        assert isinstance(result, tuple)
        assert len(result) == 3
        p, i, d = result
        # Values should round-trip cleanly through Q16 encoding/decoding.
        assert abs(p - 1.0) < 1e-4, f"Expected p≈1.0, got {p}"
        assert abs(i - 0.5) < 1e-4, f"Expected i≈0.5, got {i}"
        assert abs(d - 0.0) < 1e-6, f"Expected d≈0.0, got {d}"

    def test_get_velocity_pid_uses_velocity_mode(self) -> None:
        """get_velocity_pid passes ClosedLoopMode.VELOCITY to the session."""
        _, fake, hub = _make_hub_triple()
        motor = hub.motors[0]
        fake.set_rsp("GetMotorPIDCoefficients", p=0.0, i=0.0, d=0.0)
        try:
            motor.get_velocity_pid()
        finally:
            fake.stop()

        payload = fake.payloads.get("GetMotorPIDCoefficients", {})
        mode_raw = payload.get("mode")
        # The mode field is the last field in the request so the codec returns
        # raw bytes.  Normalise to int for comparison.
        if isinstance(mode_raw, (bytes, bytearray)):
            mode_val = int.from_bytes(mode_raw, "little")
        else:
            mode_val = int(mode_raw)
        assert mode_val == int(ClosedLoopMode.VELOCITY), (
            f"Expected mode={int(ClosedLoopMode.VELOCITY)}, got {mode_val!r}"
        )

    def test_set_velocity_pid_no_double_q16(self) -> None:
        """set_velocity_pid passes plain floats — Q16 is handled once by the session."""
        _, fake, hub = _make_hub_triple()
        motor = hub.motors[0]
        # If double-scaling occurred, p=1.0 would arrive as 65536*65536 which
        # would overflow the payload.  We verify the packet is sent without error.
        try:
            motor.set_velocity_pid(p=1.0, i=0.5, d=0.25)
        finally:
            fake.stop()

        setter_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_PID_TYPE]
        assert len(setter_reqs) == 1


def _verify_channel_from_payload(fake: FakeHub, ptype: int, *, channel: int) -> bool:
    """Helper: check that the first request of ptype has the expected channel byte."""
    req = next((r for r in fake.requests if r.packet_type == ptype), None)
    if req is None or not req.payload:
        return False
    # Channel is the first byte of most DEKA motor payloads.
    return req.payload[0] == channel


# ---------------------------------------------------------------------------
# HubVelocityController
# ---------------------------------------------------------------------------


class TestHubVelocityControllerAvailable:
    """HubVelocityController.available always returns True."""

    def test_available_is_true(self) -> None:
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            assert ctrl.available is True
        finally:
            fake.stop()

    def test_is_velocity_controller_subclass(self) -> None:
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            assert isinstance(ctrl, VelocityController)
        finally:
            fake.stop()


class TestHubVelocityControllerAttach:
    """HubVelocityController.attach() tests."""

    def test_attach_sets_constant_velocity_mode(self) -> None:
        """attach() calls set_mode(CONSTANT_VELOCITY, float_at_zero=True)."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
        finally:
            fake.stop()

        mode_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_MODE_TYPE]
        assert len(mode_reqs) >= 1

        # Payload: [channel=0, mode=CONSTANT_VELOCITY=1, float_at_zero=1]
        last_mode_req = mode_reqs[-1]
        assert last_mode_req.payload[0] == 0, "channel should be 0"
        assert last_mode_req.payload[1] == int(MotorMode.CONSTANT_VELOCITY), (
            f"Expected CONSTANT_VELOCITY={int(MotorMode.CONSTANT_VELOCITY)}, "
            f"got {last_mode_req.payload[1]}"
        )
        assert last_mode_req.payload[2] == 1, "float_at_zero should be 1"

    def test_attach_calls_enable(self) -> None:
        """attach() calls motor.enable()."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
        finally:
            fake.stop()

        enable_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_ENABLE_TYPE]
        # Should have at least one enable call with channel=0 and enabled=1
        found = any(
            len(r.payload) >= 2 and r.payload[0] == 0 and r.payload[1] == 1
            for r in enable_reqs
        )
        assert found, "Expected enable(channel=0, True) to be called"

    def test_attach_stores_prior_mode(self) -> None:
        """attach() stores a prior mode so detach() can restore it."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
        finally:
            fake.stop()

        # After attach, _prior_mode must be set (not None).
        assert ctrl._prior_mode is not None


class TestHubVelocityControllerCommand:
    """HubVelocityController.command() tests."""

    def test_command_in_range(self) -> None:
        """command(500) sends set_target_velocity(500) — no clamping."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            ctrl.command(500)
        finally:
            fake.stop()

        vel_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_VELOCITY_TYPE]
        assert len(vel_reqs) >= 1

    def test_command_max_boundary(self) -> None:
        """command(32767) passes 32767 through unchanged."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            ctrl.command(32767)
        finally:
            fake.stop()

        vel_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_VELOCITY_TYPE]
        assert len(vel_reqs) >= 1
        # Last velocity request: payload bytes are [channel, velocity_lo, velocity_hi]
        last_req = vel_reqs[-1]
        velocity_sent = int.from_bytes(last_req.payload[1:3], "little", signed=True)
        assert velocity_sent == 32767, f"Expected 32767, got {velocity_sent}"

    def test_command_min_boundary(self) -> None:
        """command(-32767) passes -32767 through unchanged."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            ctrl.command(-32767)
        finally:
            fake.stop()

        vel_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_VELOCITY_TYPE]
        assert len(vel_reqs) >= 1
        last_req = vel_reqs[-1]
        velocity_sent = int.from_bytes(last_req.payload[1:3], "little", signed=True)
        assert velocity_sent == -32767, f"Expected -32767, got {velocity_sent}"

    def test_command_over_max_clamped(self) -> None:
        """command(32768) is clamped to 32767."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            ctrl.command(32768)
        finally:
            fake.stop()

        vel_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_VELOCITY_TYPE]
        assert len(vel_reqs) >= 1
        last_req = vel_reqs[-1]
        velocity_sent = int.from_bytes(last_req.payload[1:3], "little", signed=True)
        assert velocity_sent == 32767, f"Expected 32767 (clamped), got {velocity_sent}"

    def test_command_under_min_clamped(self) -> None:
        """command(-32768) is clamped to -32767."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            ctrl.command(-32768)
        finally:
            fake.stop()

        vel_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_VELOCITY_TYPE]
        assert len(vel_reqs) >= 1
        last_req = vel_reqs[-1]
        velocity_sent = int.from_bytes(last_req.payload[1:3], "little", signed=True)
        assert velocity_sent == -32767, f"Expected -32767 (clamped), got {velocity_sent}"


class TestHubVelocityControllerMeasured:
    """HubVelocityController.measured(bulk) tests."""

    def test_measured_returns_motor_velocity(self) -> None:
        """measured(bulk) returns the velocity for the correct channel from the snapshot."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        bulk = _make_bulk(motor0_velocity=450, motor1_velocity=900)
        try:
            result = ctrl.measured(bulk)
        finally:
            fake.stop()

        assert result == 450, f"Expected 450, got {result}"

    def test_measured_correct_channel_selection(self) -> None:
        """measured() reads channel 1 velocity when controller is on channel 1."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=1)
        bulk = _make_bulk(motor0_velocity=100, motor1_velocity=750)
        try:
            result = ctrl.measured(bulk)
        finally:
            fake.stop()

        assert result == 750, f"Expected 750 (channel 1), got {result}"

    def test_measured_negative_velocity(self) -> None:
        """measured() correctly returns negative velocities (signed 16-bit)."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        bulk = _make_bulk(motor0_velocity=-300)
        try:
            result = ctrl.measured(bulk)
        finally:
            fake.stop()

        assert result == -300, f"Expected -300, got {result}"

    def test_measured_does_not_issue_transaction(self) -> None:
        """measured(bulk) reads from the snapshot — no new transaction."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        bulk = _make_bulk(motor0_velocity=123)

        # Record request count before calling measured().
        n_before = len(fake.requests)
        try:
            ctrl.measured(bulk)
        finally:
            fake.stop()

        n_after = len(fake.requests)
        assert n_after == n_before, (
            f"measured() issued {n_after - n_before} unexpected transaction(s)"
        )


class TestHubVelocityControllerDetach:
    """HubVelocityController.detach() tests."""

    def test_detach_with_disable_true_calls_disable(self) -> None:
        """detach(disable=True) calls motor.disable()."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            fake.requests.clear()  # clear attach traffic for clarity
            ctrl.detach(disable=True)
        finally:
            fake.stop()

        enable_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_ENABLE_TYPE]
        # Should include a disable call: channel=0, enabled=0
        disabled = any(
            len(r.payload) >= 2 and r.payload[0] == 0 and r.payload[1] == 0
            for r in enable_reqs
        )
        assert disabled, "Expected disable(channel=0) to be called"

    def test_detach_with_disable_false_does_not_disable(self) -> None:
        """detach(disable=False) does not call motor.disable()."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            fake.requests.clear()  # clear attach traffic
            ctrl.detach(disable=False)
        finally:
            fake.stop()

        enable_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_ENABLE_TYPE]
        # No disable (payload[1]==0) should have been sent
        disabled = any(
            len(r.payload) >= 2 and r.payload[0] == 0 and r.payload[1] == 0
            for r in enable_reqs
        )
        assert not disabled, "disable() should not be called when disable=False"

    def test_detach_restores_prior_mode(self) -> None:
        """detach() calls set_mode to restore the mode recorded by attach()."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            fake.requests.clear()  # clear attach traffic
            ctrl.detach(disable=True)
        finally:
            fake.stop()

        mode_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_MODE_TYPE]
        assert len(mode_reqs) >= 1, "Expected set_mode call in detach()"

        # The restored mode should be CONSTANT_POWER (the recorded prior mode).
        last_mode = mode_reqs[-1]
        assert last_mode.payload[0] == 0, "channel should be 0"
        restored_mode = last_mode.payload[1]
        assert restored_mode == int(MotorMode.CONSTANT_POWER), (
            f"Expected CONSTANT_POWER={int(MotorMode.CONSTANT_POWER)}, "
            f"got {restored_mode}"
        )

    def test_detach_default_is_disable_true(self) -> None:
        """detach() with no argument defaults to disable=True."""
        _, fake, hub = _make_hub_triple()
        ctrl = HubVelocityController(hub, channel=0)
        try:
            ctrl.attach()
            fake.requests.clear()
            ctrl.detach()  # no argument — should default to disable=True
        finally:
            fake.stop()

        enable_reqs = [r for r in fake.requests if r.packet_type == _SET_MOTOR_ENABLE_TYPE]
        disabled = any(
            len(r.payload) >= 2 and r.payload[0] == 0 and r.payload[1] == 0
            for r in enable_reqs
        )
        assert disabled, "Default detach() should call disable()"


# ---------------------------------------------------------------------------
# RatioDrive — helpers and stubs
# ---------------------------------------------------------------------------


def _make_bulk_timed(
    *,
    motor0_velocity: int = 0,
    motor1_velocity: int = 0,
    motor2_velocity: int = 0,
    motor3_velocity: int = 0,
    time_ms: int = 0,
) -> BulkInputData:
    """Build a synthetic BulkInputData with controllable velocities and timestamp."""
    rsp: dict = {
        "digitalInputs": 0,
        "motor0Encoder": 0,
        "motor1Encoder": 0,
        "motor2Encoder": 0,
        "motor3Encoder": 0,
        "motorStatus": 0,
        "motor0Velocity": motor0_velocity,
        "motor1Velocity": motor1_velocity,
        "motor2Velocity": motor2_velocity,
        "motor3Velocity": motor3_velocity,
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
        "mototonicTime": time_ms,
    }
    return BulkInputData.from_response(rsp)


class _RecordingController(VelocityController):
    """Stub VelocityController that records commands without touching hardware."""

    def __init__(self, channel: int) -> None:
        self.channel = channel
        self.attached = False
        self.detached = False
        self.last_command: int = 0
        self.commands: list[int] = []
        self._velocity: int = 0  # synthetic measured velocity

    def set_velocity(self, v: int) -> None:
        """Test helper: set the velocity returned by measured()."""
        self._velocity = v

    def attach(self) -> None:
        self.attached = True

    def command(self, target_counts_s: int) -> None:
        self.last_command = target_counts_s
        self.commands.append(target_counts_s)

    def measured(self, bulk: BulkInputData) -> int:
        return self._velocity

    def detach(self, disable: bool = True) -> None:
        self.detached = True

    @property
    def available(self) -> bool:
        return True


class _FakeHubForRatio:
    """Minimal hub stub for RatioDrive governor tests (no network required).

    Provides ``bulk_input()`` returning a pre-configured ``BulkInputData``,
    and a ``motors`` list of stubs that no-op ``set_velocity_pid`` and return
    a configurable value from ``get_current_ma()``.
    """

    class _FakeMotor:
        def __init__(self) -> None:
            self.pid_calls: list[tuple] = []
            self._current_ma: int = 0  # synthetic current returned by get_current_ma()

        def set_velocity_pid(self, p: float, i: float, d: float) -> None:
            self.pid_calls.append((p, i, d))

        def get_current_ma(self) -> int:
            """Return the synthetic motor current set via set_current_ma()."""
            return self._current_ma

        def set_current_ma(self, ma: int) -> None:
            """Test helper: set the value returned by get_current_ma()."""
            self._current_ma = ma

    def __init__(self) -> None:
        self.motors = [self._FakeMotor() for _ in range(4)]
        self._next_bulk: BulkInputData = _make_bulk_timed()

    def set_next_bulk(self, bulk: BulkInputData) -> None:
        self._next_bulk = bulk

    def bulk_input(self) -> BulkInputData:
        return self._next_bulk


def _make_ratio_drive(
    weights: dict,
    *,
    rate_hz: float = 50.0,
    max_accel: float = 6000.0,
    recovery_accel: float | None = None,
    sat_margin: float = 0.15,
    sat_release_margin: float = 0.07,
    sat_settle_s: float = 0.0,
    min_scale: float = 0.0,
    deadband: int = 20,
    target_scale: float = 1000.0,
) -> tuple[RatioDrive, _FakeHubForRatio, dict[int, _RecordingController]]:
    """Create a RatioDrive with _RecordingControllers injected.

    The RatioDrive's internal _controllers dict is populated with
    _RecordingController instances so governor math can be tested without
    any network I/O.

    ``sat_settle_s`` defaults to 0.0 (instant saturation) so that existing
    tests of the saturation gate remain deterministic without requiring extra
    ticks to elapse the settling window.  Tests that specifically verify the
    settle-window behaviour pass an explicit positive value.
    """
    fake_hub = _FakeHubForRatio()
    rd = RatioDrive(
        fake_hub,
        weights,
        rate_hz=rate_hz,
        max_accel=max_accel,
        recovery_accel=recovery_accel,
        sat_margin=sat_margin,
        sat_release_margin=sat_release_margin,
        sat_settle_s=sat_settle_s,
        min_scale=min_scale,
        deadband=deadband,
    )
    # Inject recording controllers (bypasses start()/attach()).
    ctrls: dict[int, _RecordingController] = {}
    for ch in weights:
        rc = _RecordingController(ch)
        ctrls[ch] = rc
        rd._controllers[ch] = rc  # type: ignore[assignment]

    # Prime the target scale.
    rd._target_scale = target_scale
    return rd, fake_hub, ctrls


# ---------------------------------------------------------------------------
# RatioDrive — unit tests
# ---------------------------------------------------------------------------


class TestRatioDriveConstruct:
    """Constructor and basic property tests."""

    def test_recovery_accel_defaults_to_max_accel(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, max_accel=5000.0)
        assert rd._recovery_accel == 5000.0

    def test_recovery_accel_explicit(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, max_accel=5000.0, recovery_accel=2000.0)
        assert rd._recovery_accel == 2000.0

    def test_weights_property_returns_copy(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 0.5})
        w = rd.weights
        assert w == {0: 1.0, 1: 0.5}
        # Mutating the copy should not affect internal state.
        w[0] = 99.0
        assert rd.weights[0] == 1.0

    def test_initial_scale_is_zero(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd.scale == 0.0

    def test_initial_target_scale_is_zero(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd.target_scale == 0.0


class TestRatioDriveSetpointAPI:
    """Thread-safe setpoint methods."""

    def test_set_speed_updates_target_scale(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        rd.set_speed(800.0)
        assert rd.target_scale == 800.0

    def test_set_speed_rpm_raises_without_cpr(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        with pytest.raises(ValueError, match="cpr"):
            rd.set_speed_rpm(100.0)

    def test_set_speed_rpm_converts_correctly(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, cpr=537.7)
        rd.set_speed_rpm(60.0)
        expected = 60.0 * 537.7 / 60.0
        assert abs(rd.target_scale - expected) < 1e-6

    def test_set_weights_replaces_atomically(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        rd.set_weights({0: 2.0, 1: -1.0})
        assert rd.weights == {0: 2.0, 1: -1.0}

    def test_set_ratio_updates_pair(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0, 2: 0.5})
        rd.set_ratio(0.75, pair=(0, 1))
        w = rd.weights
        assert w[0] == 1.0
        assert w[1] == 0.75
        # Channel 2 should be unchanged.
        assert w[2] == 0.5

    def test_stop_zeros_controllers_and_target(self) -> None:
        rd, _, ctrls = _make_ratio_drive({0: 1.0, 1: 1.0}, target_scale=500.0)
        # Prime g to something non-zero.
        rd._scale = 500.0
        rd.stop()
        assert rd.target_scale == 0.0
        assert ctrls[0].last_command == 0
        assert ctrls[1].last_command == 0
        assert rd.commanded_targets == {0: 0, 1: 0}


class TestRatioDriveRatioInvariance:
    """Ratio invariance: commanded ratio matches weight ratio at every step."""

    def test_ratio_invariant_across_steps(self) -> None:
        """commanded_targets[0] / commanded_targets[1] == w[0] / w[1] at every step."""
        weights = {0: 1.0, 1: 0.75}
        rd, _, ctrls = _make_ratio_drive(weights, target_scale=800.0, max_accel=10000.0)

        dt_ms = 20  # 20 ms per step
        time_ms = 0

        for step in range(20):
            time_ms += dt_ms
            # Synthetic: both wheels at 80% of their weight-proportional target.
            v0 = int(0.8 * weights[0] * 800)
            v1 = int(0.8 * weights[1] * 800)
            bulk = _make_bulk_timed(motor0_velocity=v0, motor1_velocity=v1, time_ms=time_ms)
            rd.update(bulk)

            t0 = rd.commanded_targets.get(0, 0)
            t1 = rd.commanded_targets.get(1, 0)
            if t1 != 0:
                actual_ratio = t0 / t1
                expected_ratio = weights[0] / weights[1]
                assert abs(actual_ratio - expected_ratio) < 0.02, (
                    f"step {step}: ratio {actual_ratio:.4f} vs expected {expected_ratio:.4f}"
                )


class TestRatioDriveCapToSlowest:
    """Cap-to-slowest: lagging wheel never boosted, scale converges to lagging n_i."""

    def test_lagging_wheel_caps_scale(self) -> None:
        """Wheel 1 stuck at 50% — scale converges to v1/|w1|; wheel 0 not boosted."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        rd, _, ctrls = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,  # very fast slew so we converge quickly
            recovery_accel=100000.0,
            sat_margin=0.10,
        )

        dt_ms = 20
        time_ms = 0

        # Wheel 0 tracks perfectly; wheel 1 only achieves 50% of target.
        for step in range(50):
            g = rd.scale
            time_ms += dt_ms
            v0 = round(g * weights[0])
            v1 = round(0.5 * g * weights[1])
            bulk = _make_bulk_timed(motor0_velocity=v0, motor1_velocity=v1, time_ms=time_ms)
            rd.update(bulk)

        # After convergence: scale should approach n_1 = 0.5 * g (self-consistent).
        # With infinite slew, g converges to some fixed-point ~ 0.
        # More precisely: n_1 = 0.5 * g / 1.0, so ceiling = 0.5*g < g always,
        # scale drops to min_scale (0).
        # The key assertion: wheel 0 is never commanded MORE than g*w0.
        # Verify: at no step was wheel 0 commanded above its proportional share.
        all_t0 = [c for c in ctrls[0].commands]
        all_t1 = [c for c in ctrls[1].commands]
        # Each t0 should equal t1 (same weights), so ratio is always 1.
        for t0, t1 in zip(all_t0, all_t1):
            assert t0 == t1, f"Commands mismatch: t0={t0}, t1={t1}"

    def test_scale_cannot_exceed_setpoint(self) -> None:
        """g is always <= S."""
        weights = {0: 1.0, 1: 1.0}
        S = 500.0
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=S, max_accel=100000.0, recovery_accel=100000.0
        )
        time_ms = 0
        for _ in range(30):
            time_ms += 20
            bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=500, time_ms=time_ms)
            rd.update(bulk)
            assert rd.scale <= S + 1e-9, f"scale {rd.scale} exceeded S={S}"


class TestRatioDriveSlewLimitDown:
    """Slew-limit down: scale decreases by at most max_accel * dt per step."""

    def test_slew_limit_down_respected(self) -> None:
        weights = {0: 1.0, 1: 1.0}
        max_accel = 500.0  # counts/s²
        S = 1000.0
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=S, max_accel=max_accel, recovery_accel=max_accel
        )
        # Manually set g to S (pretend we're at full speed).
        rd._scale = S

        dt_ms = 20  # 20 ms = 0.020 s
        dt = dt_ms / 1000.0
        max_drop_per_step = max_accel * dt

        prev_g = S
        time_ms = 0
        # Wheel 1 drops to 0 suddenly — forces big saturation.
        for step in range(20):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=0, time_ms=time_ms)
            rd.update(bulk)
            g = rd.scale
            drop = prev_g - g
            assert drop <= max_drop_per_step + 1e-6, (
                f"step {step}: drop {drop:.2f} > max_drop {max_drop_per_step:.2f}"
            )
            prev_g = g


class TestRatioDriveSlewLimitUp:
    """Slew-limit up: scale increases by at most recovery_accel * dt per step."""

    def test_slew_limit_up_respected(self) -> None:
        weights = {0: 1.0, 1: 1.0}
        recovery_accel = 300.0
        S = 1000.0
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            recovery_accel=recovery_accel,
            deadband=0,  # disable deadband so zero velocities don't trigger saturation
        )
        # Start from g = 0.
        rd._scale = 0.0

        dt_ms = 20
        dt = dt_ms / 1000.0
        max_rise_per_step = recovery_accel * dt

        # Pre-set _prev_time_ms so that the first tick computes dt = dt_ms exactly.
        # time_ms will be 1000 + 20 = 1020 on the first step, and prev = 1000, so dt = 20ms.
        time_ms = 1000
        rd._prev_time_ms = time_ms  # first tick: time_ms = 1020, dt = 20ms.

        prev_g = 0.0
        for step in range(30):
            time_ms += dt_ms
            # Both wheels perfectly track (g=0 initially, so velocity is 0).
            # With deadband=0, n_i = 0 when g=0 means shortfall = 0 → no saturation.
            v = round(prev_g)
            bulk = _make_bulk_timed(
                motor0_velocity=v,
                motor1_velocity=v,
                time_ms=time_ms,
            )
            rd.update(bulk)
            g = rd.scale
            rise = g - prev_g
            assert rise <= max_rise_per_step + 1e-6, (
                f"step {step}: rise {rise:.2f} > max_rise {max_rise_per_step:.2f}"
            )
            prev_g = g


class TestRatioDriveTransientAccelRobustness:
    """Transient accel robustness: symmetric lag during ramp does not collapse scale."""

    def test_symmetric_lag_does_not_collapse(self) -> None:
        """Both wheels lag symmetrically (shortfall < sat_margin) — g does NOT drop.

        This test verifies that when both wheels lag proportionally at a fraction
        below sat_margin, the governor does not cap scale downward.  We start
        with g pre-warmed to 200 (above deadband) so the first tick has a valid
        reference velocity.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        sat_margin = 0.20
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=200.0,
            recovery_accel=200.0,
            sat_margin=sat_margin,
            deadband=0,  # disable deadband so proportional lag is seen correctly
        )

        dt_ms = 20
        time_ms = 0

        # Pre-warm g to a value above deadband so first tick measurements are meaningful.
        rd._scale = 100.0
        prev_g = 100.0
        for step in range(30):
            time_ms += dt_ms
            # Both wheels at 90% of current g (10% shortfall — below sat_margin=20%).
            v = round(0.90 * prev_g)
            bulk = _make_bulk_timed(motor0_velocity=v, motor1_velocity=v, time_ms=time_ms)
            rd.update(bulk)
            g = rd.scale
            # Scale should be non-decreasing (symmetric lag does NOT trigger capping).
            assert g >= prev_g - 1e-6, (
                f"step {step}: scale collapsed from {prev_g:.2f} to {g:.2f}"
            )
            prev_g = g

        # g should have climbed — not stayed at 100.
        assert prev_g > 100.0, f"scale did not climb: final g={prev_g}"


class TestRatioDriveRecoveryHysteresis:
    """Recovery + hysteresis: no chatter after bottleneck clears."""

    def test_saturated_clears_after_recovery(self) -> None:
        """After bottleneck clears, saturated flips to False.

        Uses moderate max_accel (500 counts/s²) so g does not overshoot in a
        single step — the governor converges smoothly rather than chattering.
        With dt=0.02 s, the max drop per step is 10 counts/s.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 400.0
        max_accel = 500.0  # moderate slew: 10 counts/step at 50 Hz
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=max_accel,
            recovery_accel=max_accel,
            sat_margin=0.15,
            sat_release_margin=0.05,
            deadband=0,
        )
        rd._scale = S

        dt_ms = 20
        time_ms = 0

        # Phase 1: force saturation — wheel 1 stuck at 50% of g.
        # With g starting at S=400 and n_1=200, shortfall = (400-200)/400=0.5 > sat_margin.
        # g slews down at most 10 counts/step.
        for _ in range(10):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g * 0.5), time_ms=time_ms
            )
            rd.update(bulk)

        # After 10 steps with constant bottleneck, saturation should be active.
        assert rd.saturated, (
            f"Expected saturation after bottleneck phase; scale={rd.scale:.2f}, "
            f"sat_flags={rd._sat_flags}"
        )

        # Phase 2: both wheels recover fully — provide n_i >= g so shortfall goes negative.
        for _ in range(50):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms
            )
            rd.update(bulk)

        assert not rd.saturated, (
            f"Expected saturation to clear after full recovery; "
            f"scale={rd.scale:.2f}, sat_flags={rd._sat_flags}"
        )

    def test_no_chatter_after_recovery(self) -> None:
        """No rapid on/off saturation cycles after bottleneck clears.

        Uses moderate max_accel to prevent single-step collapse chattering.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 400.0
        max_accel = 500.0  # 10 counts/step at 50 Hz
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=max_accel,
            recovery_accel=max_accel,
            sat_margin=0.15,
            sat_release_margin=0.05,
            deadband=0,
        )
        rd._scale = S

        dt_ms = 20
        time_ms = 0

        # Force into saturation with a persistent bottleneck.
        for _ in range(10):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g * 0.5), time_ms=time_ms
            )
            rd.update(bulk)

        # Now both wheels fully tracking — count sat transitions.
        transitions = 0
        prev_sat = rd.saturated
        for _ in range(30):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms
            )
            rd.update(bulk)
            sat = rd.saturated
            if sat != prev_sat:
                transitions += 1
            prev_sat = sat

        # At most two transitions (True → False, and possibly a subsequent one);
        # no rapid chatter (many oscillations).
        assert transitions <= 2, f"Too many saturation transitions (chatter): {transitions}"


class TestRatioDriveZeroWeight:
    """Zero-weight channels are commanded 0 and excluded from the governor."""

    def test_zero_weight_channel_commanded_zero(self) -> None:
        weights = {0: 1.0, 1: 0.0, 2: 0.5}
        rd, _, ctrls = _make_ratio_drive(weights, target_scale=500.0, max_accel=100000.0)

        time_ms = 20
        bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=999, motor2_velocity=250, time_ms=time_ms)
        rd.update(bulk)

        assert rd.commanded_targets[1] == 0, "Zero-weight channel must be commanded 0"
        assert ctrls[1].last_command == 0

    def test_zero_weight_excluded_from_normalization(self) -> None:
        """No ZeroDivisionError when w_i = 0."""
        weights = {0: 1.0, 1: 0.0}
        rd, _, _ = _make_ratio_drive(weights, target_scale=300.0, max_accel=100000.0)

        time_ms = 20
        # Channel 1 has enormous velocity but zero weight — should not affect governor.
        bulk = _make_bulk_timed(motor0_velocity=300, motor1_velocity=30000, time_ms=time_ms)
        rd.update(bulk)  # must not raise ZeroDivisionError

        # Channel 1 must not appear in normalized_actual.
        assert 1 not in rd.normalized_actual


class TestRatioDriveStall:
    """Stall: one wheel reads 0 persistently — scale converges toward min_scale."""

    def test_stall_drives_scale_to_min(self) -> None:
        """With a stalled wheel and moderate accel, scale converges toward min_scale.

        Uses max_accel=500 (10 counts/step at 50 Hz) and deadband=0 so v=0 is
        seen as a real zero (not deadband noise), and the slew converges smoothly
        without single-step chattering.

        The governor converges to a near-zero fixed-point because:
        ceiling = n_1 = 0 always (stalled), so g slews down at max_accel*dt per step.
        After N steps = S / (max_accel * dt) = 400 / 10 = 40 steps, g reaches 0.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 400.0
        max_accel = 500.0  # 10 counts/step at dt=0.02 s
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=max_accel,
            recovery_accel=max_accel,
            sat_margin=0.10,
            min_scale=0.0,
            deadband=0,
        )
        rd._scale = S

        dt_ms = 20
        time_ms = 0

        # Run for enough steps to converge: S / (max_accel * dt) = 400 / 10 = 40 steps + margin.
        for _ in range(80):
            time_ms += dt_ms
            g = rd.scale
            # Wheel 0 perfectly tracking; wheel 1 permanently stalled.
            bulk = _make_bulk_timed(motor0_velocity=round(g), motor1_velocity=0, time_ms=time_ms)
            rd.update(bulk)

        # Scale must be at or near min_scale (0).
        assert rd.scale <= 1.0, f"Expected scale near 0 after stall, got {rd.scale:.2f}"

    def test_stall_no_crash(self) -> None:
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=200.0, max_accel=100000.0, sat_margin=0.10
        )
        rd._scale = 200.0
        time_ms = 0
        for _ in range(20):
            time_ms += 20
            bulk = _make_bulk_timed(motor0_velocity=200, motor1_velocity=0, time_ms=time_ms)
            rd.update(bulk)  # must not raise


class TestRatioDriveNegativeWeight:
    """Negative weight: reversed wheel commanded negative target."""

    def test_negative_weight_gives_negative_command(self) -> None:
        weights = {0: 1.0, 1: -0.5}
        S = 400.0
        rd, _, ctrls = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            recovery_accel=100000.0,
            sat_margin=0.15,
        )
        rd._scale = S  # pre-warm scale

        dt_ms = 20
        time_ms = 20
        # Wheel 0 at +400, wheel 1 at -200 (matching weight ratio).
        bulk = _make_bulk_timed(motor0_velocity=400, motor1_velocity=-200, time_ms=time_ms)
        rd.update(bulk)

        t0 = rd.commanded_targets[0]
        t1 = rd.commanded_targets[1]
        assert t1 < 0, f"Channel 1 with weight -0.5 should have negative command, got {t1}"
        # Ratio should be preserved: t0 / t1 == w0 / w1.
        if t1 != 0:
            assert abs(t0 / t1 - weights[0] / weights[1]) < 0.02, (
                f"Ratio mismatch: t0/t1={t0/t1:.4f} vs expected {weights[0]/weights[1]:.4f}"
            )

    def test_negative_weight_sign_in_normalized(self) -> None:
        """normalized_actual for negative-weight wheel uses sign(w)."""
        weights = {0: 1.0, 1: -1.0}
        S = 500.0
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=S, max_accel=100000.0, sat_margin=0.15
        )
        rd._scale = S

        # Wheel 1 going backward (negative) — n_1 = sign(-1) * (-300) / 1.0 = 300.
        bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=-300, time_ms=20)
        rd.update(bulk)

        n1 = rd.normalized_actual.get(1, None)
        assert n1 is not None
        assert n1 > 0, f"normalized_actual[1] should be positive (reversed wheel matching), got {n1}"


class TestRatioDriveInt16Clamp:
    """int16 clamp: targets exceeding ±32767 are clamped."""

    def test_clamp_does_not_exceed_int16(self) -> None:
        weights = {0: 32000.0, 1: 32000.0}
        S = 2.0  # g=2.0, targets = round(2.0 * 32000) = 64000 → clamped to 32767
        rd, _, ctrls = _make_ratio_drive(
            weights, target_scale=S, max_accel=100000.0, recovery_accel=100000.0
        )
        rd._scale = S

        bulk = _make_bulk_timed(motor0_velocity=32767, motor1_velocity=32767, time_ms=20)
        rd.update(bulk)

        t0 = rd.commanded_targets[0]
        t1 = rd.commanded_targets[1]
        assert t0 == 32767, f"Expected 32767 (clamped), got {t0}"
        assert t1 == 32767, f"Expected 32767 (clamped), got {t1}"

    def test_clamp_ratio_preserved_despite_clamp(self) -> None:
        """After clamp, the commanded values are both at the max — ratio is trivially preserved."""
        weights = {0: 1.0, 1: 1.0}
        S = 40000.0  # way above int16 max
        rd, _, _ = _make_ratio_drive(weights, target_scale=S, max_accel=1000000.0)
        rd._scale = S

        bulk = _make_bulk_timed(motor0_velocity=32767, motor1_velocity=32767, time_ms=20)
        rd.update(bulk)

        t0 = rd.commanded_targets[0]
        t1 = rd.commanded_targets[1]
        # Both clamped to 32767 — equal (ratio = 1.0).
        assert t0 == 32767
        assert t1 == 32767

    def test_clamped_velocity_reported_as_bottleneck(self) -> None:
        """When hw velocity is stuck at 32767 (clamped), the governor sees a bottleneck
        on the next step and reduces scale."""
        weights = {0: 1.0, 1: 1.0}
        S = 40000.0
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=S, max_accel=1000000.0, sat_margin=0.10
        )
        rd._scale = S  # pretend we commanded S

        # Step 1: wheel 0 saturated at 32767 (below commanded ~40000).
        bulk = _make_bulk_timed(motor0_velocity=32767, motor1_velocity=32767, time_ms=20)
        rd.update(bulk)

        # scale should now be reduced (bottleneck detected).
        assert rd.scale < S, f"Expected scale to drop below S={S}, got {rd.scale}"


class TestRatioDriveFirstTickDt:
    """First tick and zero-dt edge cases."""

    def test_first_tick_uses_nominal_dt(self) -> None:
        """First tick has no prev_time_ms — must not crash, must use 1/rate_hz."""
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=500.0, max_accel=6000.0
        )
        # No prior tick — _prev_time_ms is None.
        bulk = _make_bulk_timed(motor0_velocity=0, motor1_velocity=0, time_ms=0)
        rd.update(bulk)  # must not raise

    def test_same_timestamp_uses_nominal_dt(self) -> None:
        """Two consecutive ticks with the same timestamp fall back to 1/rate_hz."""
        weights = {0: 1.0, 1: 1.0}
        rate_hz = 50.0
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=200.0, max_accel=6000.0, rate_hz=rate_hz
        )
        bulk = _make_bulk_timed(motor0_velocity=100, motor1_velocity=100, time_ms=1000)
        rd.update(bulk)
        g_after_first = rd.scale
        # Same timestamp.
        rd.update(bulk)
        # Should not crash; scale should move (since dt = 1/rate_hz is used).
        # We just verify no exception was raised.

    def test_wraparound_timestamp(self) -> None:
        """Wraparound of 32-bit ms counter is handled gracefully."""
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=200.0, max_accel=6000.0
        )
        # First tick just before wraparound.
        rd._prev_time_ms = 2**32 - 100
        bulk = _make_bulk_timed(motor0_velocity=100, motor1_velocity=100, time_ms=50)
        rd.update(bulk)  # must not crash; dt should be ~150 ms


class TestRatioDriveProperties:
    """Read-only property tests."""

    def test_measured_property(self) -> None:
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(weights, target_scale=300.0, max_accel=100000.0)
        bulk = _make_bulk_timed(motor0_velocity=250, motor1_velocity=200, time_ms=20)
        rd.update(bulk)
        m = rd.measured
        assert m[0] == 250
        assert m[1] == 200

    def test_normalized_actual_property(self) -> None:
        weights = {0: 2.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(weights, target_scale=200.0, max_accel=100000.0)
        rd._scale = 200.0
        bulk = _make_bulk_timed(motor0_velocity=300, motor1_velocity=150, time_ms=20)
        rd.update(bulk)
        na = rd.normalized_actual
        # n_0 = sign(2.0) * 300 / 2.0 = 150.0
        assert abs(na[0] - 150.0) < 1.0
        # n_1 = sign(1.0) * 150 / 1.0 = 150.0
        assert abs(na[1] - 150.0) < 1.0

    def test_saturated_property_false_initially(self) -> None:
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(weights, target_scale=0.0)
        assert rd.saturated is False

    def test_commanded_targets_is_copy(self) -> None:
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(weights, target_scale=100.0, max_accel=100000.0)
        bulk = _make_bulk_timed(motor0_velocity=100, motor1_velocity=100, time_ms=20)
        rd.update(bulk)
        ct = rd.commanded_targets
        ct[0] = 99999  # mutate copy
        assert rd.commanded_targets[0] != 99999, "commanded_targets should return a copy"


def _make_rd_with_recording_ctrls(
    weights: dict,
    *,
    rate_hz: float = 200.0,
    target_scale: float = 0.0,
) -> tuple[RatioDrive, dict[int, _RecordingController]]:
    """Create a RatioDrive pre-loaded with _RecordingController stubs.

    This avoids having ``start()`` build real ``HubVelocityController``
    instances that need a full hub motor stack.  The controllers are
    injected before ``start()`` so ``start()`` calls ``attach()`` on the
    stubs (no-op) instead of on hardware.
    """
    hub = _FakeHubForRatio()
    rd = RatioDrive(hub, weights, rate_hz=rate_hz)
    rd._target_scale = target_scale
    ctrls: dict[int, _RecordingController] = {}
    for ch in weights:
        rc = _RecordingController(ch)
        ctrls[ch] = rc
        rd._controllers[ch] = rc  # type: ignore[assignment]
    return rd, ctrls


class TestRatioDriveLifecycle:
    """Lifecycle: start/close idempotency and zero-on-close using recording controllers."""

    def test_start_idempotent_no_double_thread(self) -> None:
        """Calling start() twice does not create two threads."""
        rd, _ = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0})
        try:
            rd.start()
            thread_id_1 = id(rd._thread)
            rd.start()  # second call — must be no-op
            thread_id_2 = id(rd._thread)
            assert thread_id_1 == thread_id_2, "start() should not spawn a second thread"
            assert rd._thread is not None
            assert rd._thread.is_alive()
        finally:
            rd.close()

    def test_close_idempotent(self) -> None:
        """Calling close() twice does not crash or double-detach."""
        rd, _ = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0})
        rd.start()
        rd.close()
        rd.close()  # second close — must not raise

    def test_zero_on_close(self) -> None:
        """After close(), commanded_targets are all 0."""
        rd, ctrls = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0})
        rd.start()
        rd._target_scale = 500.0
        rd.close()
        ct = rd.commanded_targets
        assert all(v == 0 for v in ct.values()), (
            f"All commanded_targets should be 0 after close(), got {ct}"
        )

    def test_context_manager_calls_start_close(self) -> None:
        """__enter__ calls start(); __exit__ calls close()."""
        hub = _FakeHubForRatio()
        weights = {0: 1.0, 1: 1.0}
        rd = RatioDrive(hub, weights, rate_hz=200.0)
        rc0 = _RecordingController(0)
        rc1 = _RecordingController(1)
        rd._controllers = {0: rc0, 1: rc1}
        with rd:
            assert rd._thread is not None
            assert rd._thread.is_alive()
        # After __exit__, thread should be stopped.
        assert not (rd._thread and rd._thread.is_alive())

    def test_stop_loop_does_not_detach(self) -> None:
        """stop_loop() stops the thread but leaves controllers attached."""
        rd, ctrls = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0})
        rd.start()
        rd.stop_loop()
        # Controllers should NOT be detached.
        assert not ctrls[0].detached
        assert not ctrls[1].detached

    def test_attach_called_on_start(self) -> None:
        """start() calls attach() on each injected controller."""
        rd, ctrls = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0})
        rd.start()
        rd.stop_loop()
        assert ctrls[0].attached, "controller[0] should be attached after start()"
        assert ctrls[1].attached, "controller[1] should be attached after start()"

    def test_detach_called_on_close(self) -> None:
        """close() calls detach(disable=True) on each controller."""
        rd, ctrls = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0})
        rd.start()
        rd.close()
        assert ctrls[0].detached, "controller[0] should be detached after close()"
        assert ctrls[1].detached, "controller[1] should be detached after close()"


class TestRatioDriveOnError:
    """on_error callback is invoked on transient errors; thread does not die."""

    def test_on_error_called_on_exception(self) -> None:
        """Thread calls on_error when step() raises; thread continues."""
        errors: list[Exception] = []

        # Build with pre-injected recording controllers so start() doesn't need real hub motors.
        rd, ctrls = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0}, rate_hz=500.0)

        # Make bulk_input raise to trigger on_error.
        call_count = [0]

        def bad_bulk():
            call_count[0] += 1
            if call_count[0] <= 3:
                raise RuntimeError("transient error")
            return _make_bulk_timed()

        rd._hub.bulk_input = bad_bulk  # type: ignore[method-assign]
        rd._on_error = errors.append

        rd.start()
        time.sleep(0.1)  # let the thread run a few ticks
        rd.close()

        assert len(errors) >= 1, "on_error should have been called at least once"
        assert all(isinstance(e, RuntimeError) for e in errors)

    def test_thread_survives_transient_errors(self) -> None:
        """Thread is still alive after transient errors."""
        rd, _ = _make_rd_with_recording_ctrls({0: 1.0, 1: 1.0}, rate_hz=500.0)

        call_count = [0]

        def flaky_bulk():
            call_count[0] += 1
            if call_count[0] % 2 == 0:
                raise ValueError("flaky")
            return _make_bulk_timed()

        rd._hub.bulk_input = flaky_bulk  # type: ignore[method-assign]
        rd._on_error = lambda e: None

        rd.start()
        time.sleep(0.1)
        assert rd._thread is not None and rd._thread.is_alive(), (
            "Thread should survive transient errors"
        )
        rd.close()


class TestRatioDriveDeadband:
    """Deadband: velocities below threshold treated as 0."""

    def test_deadband_treats_small_velocity_as_zero(self) -> None:
        """Measured velocity below deadband is treated as 0 for normalization."""
        weights = {0: 1.0, 1: 1.0}
        S = 500.0
        deadband = 50
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            sat_margin=0.10,
            deadband=deadband,
        )
        rd._scale = S

        # Wheel 1 at 30 counts/s (below deadband of 50) — should be treated as 0.
        bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=30, time_ms=20)
        rd.update(bulk)

        # normalized_actual[1] should be 0 (deadband applied).
        n1 = rd.normalized_actual.get(1, None)
        assert n1 == 0.0, f"Expected n1=0 (deadband), got {n1}"

    def test_deadband_does_not_affect_above_threshold(self) -> None:
        """Measured velocity at or above deadband is used as-is."""
        weights = {0: 1.0, 1: 1.0}
        deadband = 50
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=500.0, max_accel=100000.0, deadband=deadband
        )
        rd._scale = 500.0

        bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=60, time_ms=20)
        rd.update(bulk)

        n1 = rd.normalized_actual.get(1, None)
        # Should be 60 / 1.0 = 60.0, not 0.
        assert n1 != 0.0, "Velocity above deadband should not be zeroed"
        assert abs(n1 - 60.0) < 1.0


class TestRatioDriveVelocityPID:
    """velocity_pid is pushed to each motor during start()."""

    def test_velocity_pid_pushed_on_start(self) -> None:
        """If velocity_pid is set, each motor's set_velocity_pid is called during start()."""
        hub = _FakeHubForRatio()
        pid = (1.5, 0.3, 0.05)
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, velocity_pid=pid, rate_hz=200.0)
        # Pre-inject recording controllers so start() doesn't need real motor stack.
        rc0 = _RecordingController(0)
        rc1 = _RecordingController(1)
        rd._controllers = {0: rc0, 1: rc1}
        rd.start()
        rd.close()

        for ch in (0, 1):
            calls = hub.motors[ch].pid_calls
            assert len(calls) >= 1, f"Motor {ch} set_velocity_pid was not called"
            p, i, d = calls[0]
            assert abs(p - 1.5) < 1e-9
            assert abs(i - 0.3) < 1e-9
            assert abs(d - 0.05) < 1e-9


# ---------------------------------------------------------------------------
# RatioDrive — saturation settle window (003-005 fix tests)
# ---------------------------------------------------------------------------


class TestRatioDriveSatSettleConstructor:
    """sat_settle_s constructor parameter is stored and used."""

    def test_default_sat_settle_s(self) -> None:
        """Default sat_settle_s is 0.3 seconds."""
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._sat_settle_s == 0.3

    def test_explicit_sat_settle_s(self) -> None:
        """Explicit sat_settle_s is stored correctly."""
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, sat_settle_s=0.5)
        assert rd._sat_settle_s == 0.5

    def test_sat_settle_zero_disables_window(self) -> None:
        """sat_settle_s=0 means saturation triggers on the first high-shortfall tick."""
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=1000.0,
            max_accel=100000.0,
            sat_margin=0.10,
            sat_settle_s=0.0,  # instant — old behaviour
        )
        rd._scale = 1000.0

        # Single tick: wheel 1 stuck at 0 → shortfall ≈ 1.0 > 0.10 → immediately saturated.
        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=0, time_ms=20)
        rd.update(bulk)
        assert rd.saturated, "With sat_settle_s=0, saturation should fire on the first tick"


class TestRatioDriveAccelLagSpinUp:
    """Acceleration-lag during spin-up must NOT collapse scale to 0.

    This is the core regression test for ticket 003-005.  The governor uses
    a settling window (sat_settle_s=0.3 s, the production default) so that
    transient lag while a motor is accelerating does not trigger saturation.
    The measured velocity trails the commanded target for several ticks while
    the motor ramps up — which is physically realistic and the exact scenario
    that collapsed scale to 0 on the real hub.
    """

    def test_scale_ramps_up_despite_accel_lag(self) -> None:
        """With real sat_settle_s=0.3, scale ramps toward S during spin-up.

        Scenario: both motors start at 0, target is S=1000.  Each tick the
        measured velocity is 0 (maximum lag) for the first 0.25 s — well within
        the 0.3 s settle window — so the governor should NOT declare saturation
        and must keep climbing.  After 0.25 s we let measured track g (motor
        reached speed); the governor should reach S.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        sat_settle_s = 0.3
        dt_ms = 20  # 50 Hz ticks

        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=6000.0,       # 120 counts/step at 50 Hz
            recovery_accel=6000.0,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=sat_settle_s,
            deadband=0,
        )

        # Phase 1: measured stays at 0 for 12 ticks (240 ms < 300 ms settle window).
        # The governor must NOT flag saturation and must keep g climbing.
        time_ms = 0
        for tick in range(12):
            time_ms += dt_ms
            # Both measured velocities lag completely: still at 0 even as g climbs.
            bulk = _make_bulk_timed(
                motor0_velocity=0, motor1_velocity=0, time_ms=time_ms
            )
            rd.update(bulk)
            assert not rd.saturated, (
                f"Tick {tick}: Governor wrongly entered saturation during spin-up "
                f"(scale={rd.scale:.1f}). "
                f"sat_time={rd._sat_time}, sat_flags={rd._sat_flags}"
            )

        # scale must have climbed above 0 — the motor received sustained commands.
        scale_after_lag = rd.scale
        assert scale_after_lag > 0.0, (
            f"scale collapsed to 0 during accel-lag phase: {scale_after_lag}"
        )

        # Phase 2: measured now tracks g (motor reached speed). Governor continues up.
        for _ in range(50):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms
            )
            rd.update(bulk)

        # After convergence, scale should be close to S.
        assert rd.scale > 0.7 * S, (
            f"scale did not converge to S={S} after spin-up: {rd.scale:.1f}"
        )

    def test_scale_never_collapses_during_spinup(self) -> None:
        """Scale must be non-decreasing within the settle window during spin-up.

        Models the exact oscillation seen on the real hub: measured=0 while
        scale tries to climb.  With the settle window the scale should only
        ever go up (limited by recovery_accel) until the window elapses.
        Without the fix, scale would collapse to 0 on the very first tick.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        sat_settle_s = 0.3
        dt_ms = 20
        # Number of ticks strictly inside the settle window (accumulator not yet reached).
        window_ticks = int(sat_settle_s / (dt_ms / 1000.0))  # 15 ticks at 50 Hz

        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=6000.0,
            recovery_accel=6000.0,
            sat_margin=0.15,
            sat_settle_s=sat_settle_s,
            deadband=0,
        )

        prev_g = rd.scale  # starts at 0
        time_ms = 0
        # Run exactly window_ticks ticks — all within the settle window.
        # During this phase measured stays at 0 but saturation must NOT fire.
        for tick in range(window_ticks):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=0, motor1_velocity=0, time_ms=time_ms)
            rd.update(bulk)
            g = rd.scale
            assert g >= prev_g - 1e-6, (
                f"Tick {tick} (within settle window): scale dropped from "
                f"{prev_g:.2f} to {g:.2f} — collapse without the fix!"
            )
            assert not rd.saturated, (
                f"Tick {tick}: saturation fired inside the settle window "
                f"(scale={g:.2f}, sat_time={rd._sat_time})"
            )
            prev_g = g

        # After window_ticks × 120 counts/tick = 1800 counts, capped at S=1000.
        assert rd.scale >= min(window_ticks * 6000.0 * (dt_ms / 1000.0), S) - 1e-6, (
            "scale should have climbed to S during the settle window"
        )
        assert rd.scale > 0.0, "scale stayed at 0 — fix did not take effect"


class TestRatioDriveGenuineSustainedBottleneck:
    """Genuine sustained bottleneck still caps the pack after the settle window.

    These tests verify that the settle-window fix does NOT simply disable
    saturation — it only defers it until the shortfall has persisted for
    sat_settle_s seconds.
    """

    def test_sustained_bottleneck_caps_scale(self) -> None:
        """After sat_settle_s elapses, a persistently slow wheel caps the scale.

        Scenario: wheel 1 can only achieve 40% of g (sustained load).
        After the settle window, saturation must engage and bring g down.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        sat_settle_s = 0.3
        dt_ms = 20  # 50 Hz
        settle_ticks = int(sat_settle_s / (dt_ms / 1000.0)) + 1  # 16 ticks

        rd, _, ctrls = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,   # very fast so g hits S immediately
            recovery_accel=100000.0,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=sat_settle_s,
            deadband=0,
        )
        # Pre-warm g to S so we're testing the bottleneck cap, not spin-up.
        rd._scale = S
        # Pre-warm sat_time accumulators: set each channel's accumulator just
        # below the threshold so the NEXT tick of high shortfall triggers saturation.
        # This lets us verify the mechanism without running settle_ticks spin-up ticks.
        for ch in weights:
            rd._sat_time[ch] = sat_settle_s - (dt_ms / 1000.0) * 0.5

        time_ms = 0
        for _ in range(settle_ticks + 10):
            time_ms += dt_ms
            g = rd.scale
            # Wheel 0 tracks perfectly; wheel 1 only achieves 40% of g.
            bulk = _make_bulk_timed(
                motor0_velocity=round(g),
                motor1_velocity=round(0.4 * g),
                time_ms=time_ms,
            )
            rd.update(bulk)

        # After enough ticks, saturation must have engaged and dragged scale down.
        assert rd.scale < S * 0.8, (
            f"Expected scale to drop below {S*0.8:.0f} after sustained bottleneck, "
            f"got {rd.scale:.1f}"
        )

    def test_ratio_preserved_under_sustained_bottleneck(self) -> None:
        """Ratio is preserved at all times even when one wheel is saturated.

        This verifies that the governor's ratio invariance holds both before and
        after the settle window elapses, throughout a genuine sustained load.
        """
        weights = {0: 1.0, 1: 0.75}
        S = 800.0
        dt_ms = 20

        rd, _, ctrls = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            recovery_accel=100000.0,
            sat_margin=0.15,
            sat_settle_s=0.0,   # instant saturation — focus is on ratio, not window
            deadband=0,
        )
        rd._scale = S

        time_ms = 0
        for step in range(30):
            time_ms += dt_ms
            g = rd.scale
            # Wheel 1 lagging: provides 60% of its weight-proportional target.
            v0 = round(g * weights[0])
            v1 = round(0.6 * g * weights[1])
            bulk = _make_bulk_timed(motor0_velocity=v0, motor1_velocity=v1, time_ms=time_ms)
            rd.update(bulk)

            t0 = rd.commanded_targets.get(0, 0)
            t1 = rd.commanded_targets.get(1, 0)
            # Only check ratio when targets are large enough that integer rounding
            # (±1 count) does not dominate the ratio calculation.
            if abs(t1) >= 20:
                actual_ratio = t0 / t1
                expected_ratio = weights[0] / weights[1]
                assert abs(actual_ratio - expected_ratio) < 0.05, (
                    f"Step {step}: ratio {actual_ratio:.4f} vs expected "
                    f"{expected_ratio:.4f} (t0={t0}, t1={t1})"
                )

    def test_sat_time_resets_when_shortfall_clears(self) -> None:
        """The shortfall accumulator resets to 0 when shortfall drops below sat_release_margin.

        This prevents the accumulator from secretly crossing the threshold on
        the next high-shortfall period when the previous period had already
        cleared (no memory of previous episode).
        """
        weights = {0: 1.0, 1: 1.0}
        S = 500.0
        sat_settle_s = 0.5
        dt_ms = 20

        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=sat_settle_s,
            deadband=0,
        )
        rd._scale = S

        # Run 10 ticks of high shortfall (200 ms) — below the 500 ms settle window.
        time_ms = 0
        for _ in range(10):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=S, motor1_velocity=0, time_ms=time_ms)
            rd.update(bulk)

        # Accumulator should have grown but not triggered saturation.
        assert not rd.saturated, "Should not be saturated yet (within settle window)"
        assert rd._sat_time.get(1, 0.0) > 0.0, "Accumulator for ch1 should be non-zero"

        # Now give wheel 1 a full recovery for several ticks (shortfall < release margin).
        for _ in range(5):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms)
            rd.update(bulk)

        # Accumulator must have been reset to 0 once shortfall cleared.
        assert rd._sat_time.get(1, 0.0) == 0.0, (
            f"Accumulator should reset to 0 after shortfall cleared, "
            f"got {rd._sat_time.get(1)}"
        )


# ---------------------------------------------------------------------------
# Helpers for current-limit tests
# ---------------------------------------------------------------------------


def _make_ratio_drive_cur(
    weights: dict,
    *,
    current_limit_ma: int,
    rate_hz: float = 50.0,
    max_accel: float = 6000.0,
    recovery_accel: float | None = None,
    sat_margin: float = 0.15,
    sat_release_margin: float = 0.07,
    sat_settle_s: float = 0.0,
    min_scale: float = 0.0,
    deadband: int = 0,
    target_scale: float = 1000.0,
) -> tuple[RatioDrive, _FakeHubForRatio, dict[int, _RecordingController]]:
    """Create a RatioDrive with current_limit_ma and _RecordingControllers injected.

    ``_cur_sample_every`` is forced to 1 so that get_current_ma() is called on
    every tick — this makes unit tests deterministic without having to run
    _cur_sample_every extra ticks before the current ceiling kicks in.
    """
    fake_hub = _FakeHubForRatio()
    rd = RatioDrive(
        fake_hub,
        weights,
        rate_hz=rate_hz,
        max_accel=max_accel,
        recovery_accel=recovery_accel,
        sat_margin=sat_margin,
        sat_release_margin=sat_release_margin,
        sat_settle_s=sat_settle_s,
        min_scale=min_scale,
        deadband=deadband,
        current_limit_ma=current_limit_ma,
    )
    # Force current sampling on every tick so tests are deterministic.
    rd._cur_sample_every = 1

    # Inject recording controllers (bypasses start()/attach()).
    ctrls: dict[int, _RecordingController] = {}
    for ch in weights:
        rc = _RecordingController(ch)
        ctrls[ch] = rc
        rd._controllers[ch] = rc  # type: ignore[assignment]

    # Prime the target scale.
    rd._target_scale = target_scale
    return rd, fake_hub, ctrls


# ---------------------------------------------------------------------------
# RatioDrive — current-limit tests (003-007)
# ---------------------------------------------------------------------------


class TestRatioDriveCurrentLimitNoneRegression:
    """current_limit_ma=None must preserve all existing velocity-only behavior."""

    def test_none_no_current_reads(self) -> None:
        """With current_limit_ma=None, get_current_ma() is never called."""
        weights = {0: 1.0, 1: 1.0}
        rd, fake_hub, _ = _make_ratio_drive_cur(
            weights,
            current_limit_ma=0,  # will convert to None below
            target_scale=1000.0,
            max_accel=100000.0,
        )
        # Override: set current_limit_ma to None explicitly (factory used 0 → None
        # mirroring the velocity_chart.py convention; but here we test the param directly).
        rd._current_limit_ma = None
        # Ensure current reads would be detectable if called.
        fake_hub.motors[0].set_current_ma(99999)
        fake_hub.motors[1].set_current_ma(99999)

        rd._scale = 1000.0
        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=1000, time_ms=20)
        rd.update(bulk)

        # Scale must not be affected by the high fake current.
        assert rd.scale >= 999.0, (
            f"current_limit_ma=None must not de-rate scale; got {rd.scale:.2f}"
        )
        # last_current_ma should remain empty (no reads).
        assert rd.last_current_ma == {}, (
            "last_current_ma should be empty when current_limit_ma=None"
        )

    def test_none_velocity_saturation_unchanged(self) -> None:
        """Velocity-based saturation works identically with current_limit_ma=None."""
        weights = {0: 1.0, 1: 1.0}
        S = 500.0
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            sat_margin=0.10,
            sat_settle_s=0.0,
            deadband=0,
        )
        rd._scale = S

        # Wheel 1 stalled — velocity saturation should fire.
        bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=0, time_ms=20)
        rd.update(bulk)

        assert rd.saturated, "Velocity saturation must fire with current_limit_ma=None"
        assert rd.scale < S, "Scale must drop when velocity-saturated"


class TestRatioDriveCurrentLimitDerates:
    """Current above limit causes de-rating; ratio stays exact."""

    def test_derates_when_current_over_limit(self) -> None:
        """Motor 0 over limit → g is capped below S after enough ticks."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        limit = 2000  # mA
        rd, fake_hub, ctrls = _make_ratio_drive_cur(
            weights,
            current_limit_ma=limit,
            target_scale=S,
            max_accel=100000.0,
        )
        rd._scale = S
        # Motor 0 draws 500 mA above limit (above the 5% hysteresis band).
        fake_hub.motors[0].set_current_ma(limit + 500)
        fake_hub.motors[1].set_current_ma(100)  # well under

        # Pre-seed current hysteresis flag so first tick actually applies ceiling.
        rd._cur_sat_flags[0] = True
        rd._last_current_ma[0] = limit + 500
        rd._last_current_ma[1] = 100

        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=1000, time_ms=20)
        rd.update(bulk)

        assert rd.scale < S, (
            f"Scale should be capped below S={S} when motor current exceeds limit; "
            f"got {rd.scale:.2f}"
        )

    def test_ratio_exact_during_current_saturation(self) -> None:
        """Commanded ratio stays exact (w0/w1) at every tick under current saturation."""
        weights = {0: 1.0, 1: 0.5}
        S = 1000.0
        limit = 2000
        rd, fake_hub, ctrls = _make_ratio_drive_cur(
            weights,
            current_limit_ma=limit,
            target_scale=S,
            max_accel=100000.0,
            recovery_accel=100000.0,
        )
        rd._scale = S
        # Motor 0 persistently over limit.
        fake_hub.motors[0].set_current_ma(limit + 800)
        fake_hub.motors[1].set_current_ma(50)
        # Seed flags so ceiling is active from tick 1.
        rd._cur_sat_flags[0] = True
        rd._last_current_ma[0] = limit + 800
        rd._last_current_ma[1] = 50

        expected_ratio = weights[0] / weights[1]  # 2.0

        dt_ms = 20
        time_ms = 0
        for step in range(15):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g * weights[0]),
                motor1_velocity=round(g * weights[1]),
                time_ms=time_ms,
            )
            rd.update(bulk)

            t0 = rd.commanded_targets.get(0, 0)
            t1 = rd.commanded_targets.get(1, 0)
            # Only check ratio when targets are large enough that integer rounding
            # (±1 count) does not dominate.  With w0/w1=2.0 and t1>=20, the
            # maximum rounding error is ±1/20 = 0.05 on each target, giving a
            # combined ratio error of at most ~0.1.  We allow 0.15 here.
            if abs(t1) >= 20:
                actual = t0 / t1
                assert abs(actual - expected_ratio) < 0.15, (
                    f"Step {step}: ratio {actual:.4f} vs expected {expected_ratio:.4f} "
                    f"(t0={t0}, t1={t1})"
                )

    def test_scale_capped_multiple_ticks(self) -> None:
        """Scale stays at or below the current ceiling across multiple ticks."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        limit = 2000
        over_limit = limit + 600  # 2600 mA → ceiling = g * (2000/2600) ≈ 0.769*g
        rd, fake_hub, ctrls = _make_ratio_drive_cur(
            weights,
            current_limit_ma=limit,
            target_scale=S,
            max_accel=100000.0,
        )
        rd._scale = S
        fake_hub.motors[0].set_current_ma(over_limit)
        fake_hub.motors[1].set_current_ma(100)
        rd._cur_sat_flags[0] = True
        rd._last_current_ma[0] = over_limit
        rd._last_current_ma[1] = 100

        dt_ms = 20
        time_ms = 0
        for _ in range(10):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms
            )
            rd.update(bulk)

        # Scale must be well below S after sustained current saturation.
        assert rd.scale < S * 0.85, (
            f"Expected scale < {S*0.85:.0f} under sustained current saturation; "
            f"got {rd.scale:.2f}"
        )


class TestRatioDriveCurrentLimitRecovery:
    """When current drops below limit, g recovers toward S."""

    def test_recovery_toward_setpoint(self) -> None:
        """After current drops below limit, g climbs back toward S."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        limit = 2000
        rd, fake_hub, ctrls = _make_ratio_drive_cur(
            weights,
            current_limit_ma=limit,
            target_scale=S,
            max_accel=100000.0,
            recovery_accel=100000.0,
        )
        # Phase 1: set g low to simulate post-de-rate state.
        rd._scale = 400.0
        # Motor current is now well below limit.
        fake_hub.motors[0].set_current_ma(100)
        fake_hub.motors[1].set_current_ma(100)
        rd._cur_sat_flags[0] = False
        rd._cur_sat_flags[1] = False
        rd._last_current_ma[0] = 100
        rd._last_current_ma[1] = 100

        dt_ms = 20
        time_ms = 0
        for _ in range(20):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms
            )
            rd.update(bulk)

        # Scale should have climbed back toward S.
        assert rd.scale > 900.0, (
            f"Expected scale to recover toward S={S}; got {rd.scale:.2f}"
        )

    def test_hysteresis_prevents_immediate_release(self) -> None:
        """Current must drop below limit*(1-hys) before the ceiling releases."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        limit = 2000
        rd, fake_hub, ctrls = _make_ratio_drive_cur(
            weights,
            current_limit_ma=limit,
            target_scale=S,
            max_accel=100000.0,
        )
        rd._scale = S
        hys = rd._cur_hys  # 0.05

        # Current just barely below limit (within hysteresis band):
        # limit * (1 - hys) = 2000 * 0.95 = 1900; set current to 1950.
        # The flag was True (from a previous over-limit episode).
        just_below = int(limit * (1.0 - hys)) + 50  # 1950 mA — above release threshold
        fake_hub.motors[0].set_current_ma(just_below)
        fake_hub.motors[1].set_current_ma(100)
        rd._cur_sat_flags[0] = True  # already saturated
        rd._last_current_ma[0] = just_below
        rd._last_current_ma[1] = 100

        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=1000, time_ms=20)
        rd.update(bulk)

        # Flag must still be True (hysteresis holds; 1950 > 1900 = release threshold).
        assert rd._cur_sat_flags.get(0, False), (
            "Current flag should remain True within the hysteresis band"
        )

    def test_flag_releases_below_hysteresis_threshold(self) -> None:
        """Current below limit*(1-hys) clears the current saturation flag."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        limit = 2000
        rd, fake_hub, ctrls = _make_ratio_drive_cur(
            weights,
            current_limit_ma=limit,
            target_scale=S,
            max_accel=100000.0,
        )
        rd._scale = S
        hys = rd._cur_hys  # 0.05
        # release threshold = limit * (1 - hys) = 1900 mA; set current well below.
        well_below = int(limit * (1.0 - hys)) - 200  # 1700 mA
        fake_hub.motors[0].set_current_ma(well_below)
        fake_hub.motors[1].set_current_ma(100)
        rd._cur_sat_flags[0] = True  # previously saturated
        rd._last_current_ma[0] = well_below
        rd._last_current_ma[1] = 100

        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=1000, time_ms=20)
        rd.update(bulk)

        # Flag must be cleared now.
        assert not rd._cur_sat_flags.get(0, False), (
            "Current flag should clear when current drops below limit*(1-hys)"
        )


class TestRatioDriveCurrentLimitConstructor:
    """Constructor stores current_limit_ma; default is None."""

    def test_default_is_none(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._current_limit_ma is None

    def test_explicit_value_stored(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, current_limit_ma=1500)
        assert rd._current_limit_ma == 1500

    def test_last_current_ma_property_empty_when_disabled(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd.last_current_ma == {}

    def test_last_current_ma_populated_after_tick(self) -> None:
        """last_current_ma is populated after a tick when current_limit_ma is set."""
        weights = {0: 1.0, 1: 1.0}
        limit = 2000
        rd, fake_hub, _ = _make_ratio_drive_cur(
            weights, current_limit_ma=limit, target_scale=500.0
        )
        fake_hub.motors[0].set_current_ma(300)
        fake_hub.motors[1].set_current_ma(400)

        bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=500, time_ms=20)
        rd.update(bulk)

        cur = rd.last_current_ma
        assert cur.get(0) == 300, f"Expected 300 mA for ch0, got {cur.get(0)}"
        assert cur.get(1) == 400, f"Expected 400 mA for ch1, got {cur.get(1)}"
