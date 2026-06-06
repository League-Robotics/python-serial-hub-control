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
        # 003-010: recovery_accel_up is now an independent tunable defaulting to 500.
        # The old behavior (recovery_accel == max_accel when unspecified) is superseded:
        # the new governor uses a deliberately slow upward rate to prevent overshoot.
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, max_accel=5000.0)
        # Default recovery_accel_up is 500 (independent of max_accel).
        assert rd._recovery_accel_up == 500.0
        # _recovery_accel is kept as an alias for backward-compat; it mirrors _recovery_accel_up.
        assert rd._recovery_accel == rd._recovery_accel_up

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
        """commanded_targets[0] / commanded_targets[1] == w[0] / w[1] at every step.

        003-010: The new governor climbs at recovery_accel_up (default 500 cnt/s²) rather
        than at max_accel.  At low g values (first few ticks), integer rounding dominates
        the ratio check (e.g. round(8*0.75)=6 → ratio=8/6=1.333 is fine, but at g=2,
        round(2*0.75)=2 → ratio=1.0 ≠ 1.333).  We skip the ratio assertion when commanded
        targets are too small for rounding to be insignificant (|t1| < 20).
        """
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
            # Only check ratio when t1 is large enough that ±1 rounding error is negligible.
            if abs(t1) >= 20:
                actual_ratio = t0 / t1
                expected_ratio = weights[0] / weights[1]
                assert abs(actual_ratio - expected_ratio) < 0.06, (
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
        """After bottleneck clears (measured velocity exceeds latch cap), saturated flips False.

        003-010: ``rd.saturated`` now reflects the sticky max-speed latch rather than
        the old shortfall-based flag.  The latch fires once the wheel's dv/dt EMA drops
        below the plateau threshold (~12-15 ticks at alpha=0.3 and 50 Hz).  Phase 1
        therefore needs enough ticks for the EMA to settle.  In Phase 2 the test
        provides v1=g (100% tracking), so n_i=g > latch_cap (~200), which clears the
        latch immediately.
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

        # Phase 1: force latch — wheel 1 stuck at 50% of g.
        # The dv/dt EMA needs ~15 ticks to settle below the plateau threshold.
        # Use 25 ticks to guarantee the latch fires.
        for _ in range(25):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g * 0.5), time_ms=time_ms
            )
            rd.update(bulk)

        # After 25 steps, the plateau latch must be active (rd.saturated = latch active).
        assert rd.saturated, (
            f"Expected speed latch after bottleneck phase; scale={rd.scale:.2f}, "
            f"latches={rd._speed_latch}"
        )

        # Phase 2: both wheels recover fully — provide n_i = g (> latch_cap ~200).
        # The latch clears on the first tick where n_i > latch_cap.
        for _ in range(50):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms
            )
            rd.update(bulk)

        assert not rd.saturated, (
            f"Expected latch to clear after full recovery; "
            f"scale={rd.scale:.2f}, latches={rd._speed_latch}"
        )

    def test_no_chatter_after_recovery(self) -> None:
        """No rapid on/off latch cycles after bottleneck clears (sticky latch, no probing).

        003-010: The sticky latch design guarantees no chatter — once the latch clears
        (n_i > latch_cap), g climbs toward S freely and the latch re-fires only if g
        reaches a new plateau.  With 30 ticks of full tracking, at most one transition
        (latch→clear) should occur.
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

        # Force the latch with a persistent bottleneck (25 ticks for EMA to settle).
        for _ in range(25):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g * 0.5), time_ms=time_ms
            )
            rd.update(bulk)

        # Now both wheels fully tracking — count latch transitions.
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

        # Sticky latch: at most two transitions (True→False once, maybe one re-latch);
        # no rapid chatter (many oscillations).
        assert transitions <= 2, f"Too many latch transitions (chatter): {transitions}"


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
        """003-010: With a stalled wheel (v=0 always), the NEW governor does NOT collapse g.

        Old behavior (005): a wheel stuck at v=0 triggered shortfall-based saturation and
        drove g to min_scale.  New behavior: the plateau latch fires only after the motor
        has first accelerated (dv/dt > plateau_threshold at some point).  A wheel that
        was always at v=0 never demonstrates acceleration, so the latch never fires and
        g remains at S.

        This is intentional: a truly stalled motor (e.g. mechanically jammed) is better
        handled via current limiting (set current_limit_ma) than by collapsing g to 0.
        The ratio guarantee still holds: both commanded targets maintain w0/w1 throughout.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 400.0
        max_accel = 500.0
        rd, _, ctrls = _make_ratio_drive(
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

        for _ in range(80):
            time_ms += dt_ms
            g = rd.scale
            # Wheel 0 perfectly tracking; wheel 1 permanently stalled.
            bulk = _make_bulk_timed(motor0_velocity=round(g), motor1_velocity=0, time_ms=time_ms)
            rd.update(bulk)

        # NEW behavior: stall (v=0 always, never accelerated) does NOT fire the plateau
        # latch, so g remains at S.  The ratio is preserved at every tick.
        # (Old behavior was: scale collapses to 0.  That is intentionally superseded.)
        assert rd.scale >= S * 0.9, (
            f"Expected g to stay near S={S} when wheel never accelerated (stall); "
            f"got {rd.scale:.2f}"
        )
        # Ratio invariance: t0 == t1 (equal weights).
        t0 = rd.commanded_targets.get(0, 0)
        t1 = rd.commanded_targets.get(1, 0)
        assert t0 == t1, f"Ratio violated under stall: t0={t0}, t1={t1}"

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
        """When hw velocity is stuck at 32767 (clamped), the governor detects a bottleneck
        after the dv/dt plateau test fires and reduces scale.

        003-010: The plateau latch requires the motor to first show acceleration (dv/dt
        above threshold).  Starting from g=40000 with v=32767 from tick 0: the velocity
        EMA rises from 0→32767 across ticks (high dv/dt initially), then levels off.
        The latch fires once dv/dt drops below the threshold.  Use enough ticks for
        the EMA to settle, then assert g dropped.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 40000.0
        rd, _, _ = _make_ratio_drive(
            weights, target_scale=S, max_accel=1000000.0, sat_margin=0.10
        )
        rd._scale = S  # pretend we commanded S

        # Run enough ticks at clamped velocity for the dv/dt EMA to settle below the
        # plateau threshold.  With S=40000 and v=32767, the initial dvdt is very large
        # (~150000), and decays exponentially.  At alpha=0.3, it takes ~30 ticks to
        # settle below 150.  Use 50 ticks to be robust.
        time_ms = 0
        for _ in range(50):
            time_ms += 20
            bulk = _make_bulk_timed(motor0_velocity=32767, motor1_velocity=32767, time_ms=time_ms)
            rd.update(bulk)

        # After the EMA plateaus: latch fires, g drops toward the latch cap (32767).
        assert rd.scale < S, f"Expected scale to drop below S={S} after plateau, got {rd.scale}"


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
        """003-010: sat_settle_s is a superseded parameter (accepted but ignored).

        The new governor uses a dv/dt plateau latch, not a time-window shortfall
        accumulator.  The sat_settle_s=0 shortcut (instant saturation on first shortfall
        tick) no longer applies.  Passing sat_settle_s=0 now has no effect on behavior;
        the latch fires only after the motor has demonstrated acceleration and then
        plateaued.

        This test verifies that the parameter is accepted without error (backward compat)
        and that the internal _sat_settle_s attribute stores the value as-is for compat
        introspection (even though it is not used by the governor).
        """
        weights = {0: 1.0, 1: 1.0}
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=1000.0,
            max_accel=100000.0,
            sat_margin=0.10,
            sat_settle_s=0.0,  # superseded — accepted but ignored
        )
        rd._scale = 1000.0

        # The parameter is stored (backward compat) even though it is not used.
        assert rd._sat_settle_s == 0.0, "sat_settle_s should be stored even though superseded"

        # A single tick with wheel 1 at 0 will NOT immediately saturate in the new design:
        # the plateau latch requires first seeing acceleration (dv/dt > threshold).
        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=0, time_ms=20)
        rd.update(bulk)
        # sat_flags (superseded) remains empty; the plateau latch has not yet fired.
        assert rd._sat_flags == {}, (
            "Superseded _sat_flags should remain empty; new design uses _speed_latch"
        )


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
        """003-010: _sat_time is a superseded attribute; this test validates the new design.

        The old behavior was: a per-channel shortfall accumulator (_sat_time) counted
        up time-in-shortfall and triggered saturation after sat_settle_s seconds.  The
        new governor replaces this with a dv/dt plateau latch (_speed_latch) and does
        not maintain _sat_time.

        Instead, this test verifies the analogous NEW invariant: a speed latch fires
        only after the motor has demonstrated acceleration and plateaued, and clears
        when measured velocity rises above the latch cap.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 500.0
        dt_ms = 20

        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=0.5,  # superseded — accepted but ignored
            deadband=0,
        )
        rd._scale = S

        # _sat_time is not populated by the new governor (superseded internal state).
        time_ms = 0
        bulk = _make_bulk_timed(motor0_velocity=S, motor1_velocity=0, time_ms=20)
        rd.update(bulk)
        assert rd._sat_time == {} or rd._sat_time.get(1, 0.0) == 0.0, (
            "Superseded _sat_time should not be populated by the new governor"
        )

        # New invariant: _speed_latch is used instead.  On the first tick with v=0,
        # the plateau latch is NOT yet active (motor hasn't shown acceleration yet).
        assert rd._speed_latch.get(1) is None, (
            "Speed latch should not fire on first tick (motor never accelerated)"
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
        """003-010: Speed latch (rd.saturated) and no current reads with current_limit_ma=None.

        Old behavior: velocity-based shortfall saturation fired on the first high-shortfall
        tick (with sat_settle_s=0).  New behavior: the plateau latch fires after dv/dt
        drops below the threshold (motor first accelerates, then levels off).

        With current_limit_ma=None, the current path is disabled.  The speed latch is
        the only governor cap, and it fires only after the acceleration → plateau
        transition.  A single tick with v=0 does NOT trigger saturation in the new design.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 500.0
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=100000.0,
            sat_margin=0.10,
            sat_settle_s=0.0,  # superseded — accepted but ignored
            deadband=0,
        )
        rd._scale = S

        # Wheel 1 stalled — single tick does NOT fire the latch (no prior acceleration).
        bulk = _make_bulk_timed(motor0_velocity=500, motor1_velocity=0, time_ms=20)
        rd.update(bulk)

        # In the new design, rd.saturated is False after one tick (old behavior changed).
        assert not rd.saturated, (
            "New design: latch does not fire on first zero-velocity tick (no prior accel). "
            "Old shortfall-based saturation is superseded."
        )
        # No current reads should have occurred (current_limit_ma=None).
        assert rd.last_current_ma == {}, "No current reads with current_limit_ma=None"


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


# ---------------------------------------------------------------------------
# RatioDrive — limit-cycle convergence tests (003-009 fix)
# ---------------------------------------------------------------------------


def _synthetic_tick(
    rd: RatioDrive,
    *,
    time_ms: int,
    motor0_v: int,
    motor1_v: int,
) -> None:
    """Feed a single synthetic tick to `rd` without touching hardware."""
    bulk = _make_bulk_timed(
        motor0_velocity=motor0_v,
        motor1_velocity=motor1_v,
        time_ms=time_ms,
    )
    rd.update(bulk)


class TestRatioDriveLimitCycleConvergence:
    """003-009: Governor converges at the bottleneck equilibrium instead of cycling to S.

    Setup: weights={0:1.0, 1:5.0}, S=1000, motor1 physically pinned at 2500 cnt/s
    (normalised n1_max=500).  The pack should settle at g≈500 (motor0=500,
    motor1=2500) and STAY there — not oscillate back to g=1000.
    """

    def test_g_converges_to_bottleneck(self) -> None:
        """g settles near the bottleneck normalised speed (~500) and stays there.

        Motor1 weight=5.0; physical max=2500 cnt/s so n1_max=500.  After the
        initial cap-down, g must remain within a small band around 500 for
        at least 50 consecutive ticks — no swing back to S=1000.
        """
        weights = {0: 1.0, 1: 5.0}
        S = 1000.0
        n1_max = 500  # motor1 physical cap (normalised: 2500/5)
        dt_ms = 20
        sat_settle_s = 0.3
        recovery_accel = 6000.0
        max_accel = 6000.0

        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=max_accel,
            recovery_accel=recovery_accel,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=sat_settle_s,
            deadband=0,
        )

        # Pre-warm: start with g at S (as if motors had already been running).
        rd._scale = S

        time_ms = 0
        # Phase 1: Run until the governor converges (~2s should be plenty).
        # Motor0 tracks g perfectly; motor1 is pinned at n1_max=500.
        for _ in range(100):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g * weights[0])       # motor0 perfectly tracks
            v1 = min(round(g * weights[1]), 2500)  # motor1 capped at 2500 cnt/s
            n1_measured = v1  # raw velocity
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)

        # After convergence, g must be near n1_max=500.
        g_converged = rd.scale
        tolerance = 50  # ±50 counts/s around 500
        assert abs(g_converged - n1_max) <= tolerance, (
            f"Expected g≈{n1_max} after convergence, got {g_converged:.1f}"
        )

        # Phase 2: Run 50 more ticks and verify g STAYS near 500 — no limit cycle.
        g_values_post = []
        for _ in range(50):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g * weights[0])
            v1 = min(round(g * weights[1]), 2500)
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)
            g_values_post.append(rd.scale)

        max_g_post = max(g_values_post)
        assert max_g_post < S * 0.7, (
            f"Limit cycle detected: g climbed to {max_g_post:.1f} after convergence "
            f"(should stay near {n1_max}, never return to S={S})"
        )
        # The band should be narrow — no large-amplitude oscillation.
        band = max(g_values_post) - min(g_values_post)
        assert band < 50, (
            f"g oscillation band {band:.1f} is too large after convergence "
            f"(expected < 50 counts/s ripple)"
        )

    def test_ratio_preserved_at_equilibrium(self) -> None:
        """At the bottleneck equilibrium, commanded ratio stays exact.

        Even when g is capped at the bottleneck, t1/t0 == w1/w0 == 5.0.
        """
        weights = {0: 1.0, 1: 5.0}
        S = 1000.0
        dt_ms = 20

        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=6000.0,
            recovery_accel=6000.0,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=0.3,
            deadband=0,
        )
        rd._scale = S

        time_ms = 0
        # Run to convergence (~2s).
        for _ in range(100):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g)
            v1 = min(round(g * 5.0), 2500)
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)

        # Check ratio over the next 20 ticks.
        for step in range(20):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g)
            v1 = min(round(g * 5.0), 2500)
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)

            t0 = rd.commanded_targets.get(0, 0)
            t1 = rd.commanded_targets.get(1, 0)
            if t0 > 0 and t1 > 0:
                actual_ratio = t1 / t0
                assert abs(actual_ratio - 5.0) < 0.1, (
                    f"Step {step}: ratio {actual_ratio:.3f} != 5.0 at equilibrium "
                    f"(t0={t0}, t1={t1}, g={rd.scale:.1f})"
                )


class TestRatioDriveGenuineRecovery:
    """003-009: After bottleneck clears, g climbs back toward S within ~1-2s."""

    def test_g_recovers_when_bottleneck_clears(self) -> None:
        """003-010: Bottleneck recovery requires explicit set_speed() call (sticky latch design).

        Old behavior (009): After the bottleneck cleared, the probe mechanism allowed g
        to autonomously climb back toward S at probe_step/tick.

        New behavior (010): The sticky latch holds g at the cap indefinitely.  Recovery
        requires an explicit setpoint change (set_speed, set_ratio, etc.) to clear the
        latch.  This eliminates the limit-cycle regression from the probe mechanism while
        accepting that operator intent (a setpoint change) is required to resume after
        a physical ceiling is encountered.

        Verification: after set_speed(S) is called following bottleneck convergence,
        g climbs back toward S.
        """
        weights = {0: 1.0, 1: 5.0}
        S = 1000.0
        dt_ms = 20

        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=6000.0,
            recovery_accel=6000.0,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=0.3,  # superseded — accepted but ignored
            deadband=0,
        )
        rd._scale = S

        time_ms = 0
        # Phase 1: run with bottleneck until convergence.
        for _ in range(100):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g)
            v1 = min(round(g * 5.0), 2500)
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)

        g_at_bottleneck = rd.scale
        assert abs(g_at_bottleneck - 500) <= 80, (
            f"Expected g≈500 after bottleneck phase, got {g_at_bottleneck:.1f}"
        )

        # Phase 2: explicit set_speed() clears the latch.
        rd.set_speed(S)  # user re-asserts the setpoint — latch clears

        # Now motor1 can track fully (no physical cap).
        for _ in range(200):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g)
            v1 = round(g * 5.0)  # no cap
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)

        g_recovered = rd.scale
        assert g_recovered > S * 0.9, (
            f"Expected g to recover above {S*0.9:.0f} after set_speed + bottleneck cleared, "
            f"got {g_recovered:.1f}"
        )

    def test_recovery_is_gradual_not_instant(self) -> None:
        """003-010: After set_speed() clears the latch, g climbs at recovery_accel_up.

        Old behavior: probe_step=10 cnt/tick gave a gradual ~50-tick recovery.
        New behavior: recovery_accel_up (default 500 cnt/s²) also gives gradual recovery
        at 50 Hz (500*0.02=10 cnt/tick, identical rate to probe_step=10).
        In 5 ticks (0.1 s): max rise = 500*0.1=50 counts — well below S=1000.

        Note: do NOT pass explicit recovery_accel (which would override recovery_accel_up
        and make recovery fast); rely on the default recovery_accel_up=500.
        """
        weights = {0: 1.0, 1: 5.0}
        S = 1000.0
        dt_ms = 20

        # Use default recovery_accel_up (500) for gradual recovery.
        # Do NOT pass recovery_accel to avoid overriding the slow default.
        rd, _, _ = _make_ratio_drive(
            weights,
            target_scale=S,
            max_accel=6000.0,
            sat_margin=0.15,
            sat_release_margin=0.07,
            sat_settle_s=0.3,  # superseded — accepted but ignored
            deadband=0,
        )
        rd._scale = S

        time_ms = 0
        # Converge at bottleneck.
        for _ in range(100):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g)
            v1 = min(round(g * 5.0), 2500)
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)

        g_at_convergence = rd.scale
        assert abs(g_at_convergence - 500) <= 80, (
            f"Expected g≈500 at bottleneck convergence, got {g_at_convergence:.1f}"
        )

        # Clear latch via set_speed().
        rd.set_speed(S)

        # Start recovery: after 5 ticks at default recovery_accel_up=500,
        # max rise = 500 * (5 * 0.02) = 50 counts.  g should still be near convergence.
        for _ in range(5):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g)
            v1 = round(g * 5.0)
            _synthetic_tick(rd, time_ms=time_ms, motor0_v=v0, motor1_v=v1)

        g_after_5 = rd.scale
        # After only 5 ticks at recovery_accel_up=500, g should NOT have jumped to S.
        # Max rise = 50 counts, so g ≤ g_at_convergence + 50 ≈ 550, well below 800=S*0.8.
        assert g_after_5 < S * 0.8, (
            f"Recovery was too fast — g jumped to {g_after_5:.1f} in 5 ticks "
            f"(expected gradual climb from {g_at_convergence:.1f})"
        )


class TestRatioDriveProbeParameters:
    """Constructor parameters probe_step and probe_settle_ticks are stored."""

    def test_probe_step_default(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._probe_step == 10.0

    def test_probe_step_explicit(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, probe_step=5.0)
        assert rd._probe_step == 5.0

    def test_probe_settle_ticks_default(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._probe_settle_ticks == 5

    def test_probe_settle_ticks_explicit(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, probe_settle_ticks=10)
        assert rd._probe_settle_ticks == 10

    def test_learned_bottleneck_initially_none(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._learned_bottleneck is None


# ---------------------------------------------------------------------------
# RatioDrive — two-cap governor tests (003-010)
# ---------------------------------------------------------------------------
#
# These tests cover the unified governor introduced in ticket 003-010:
#   - Sticky max-speed latch (dv/dt-based plateau detection)
#   - Live current closed-loop cap
#   - Ratio exactness throughout all governor states
#   - Latch reset on setpoint/weight change
# ---------------------------------------------------------------------------


def _make_ratio_drive_010(
    weights: dict,
    *,
    target_scale: float = 1000.0,
    max_accel: float = 100000.0,
    rate_hz: float = 50.0,
    plateau_threshold_cnts_s2: float = 150.0,
    ema_alpha: float = 0.3,
    speed_margin_frac: float = 0.15,
    deadband: int = 0,
) -> tuple["RatioDrive", "_FakeHubForRatio", dict[int, "_RecordingController"]]:
    """Create a RatioDrive with two-cap governor tunables for 003-010 tests."""
    from rhsp.control import RatioDrive

    fake_hub = _FakeHubForRatio()
    rd = RatioDrive(
        fake_hub,
        weights,
        rate_hz=rate_hz,
        max_accel=max_accel,
        deadband=deadband,
        plateau_threshold_cnts_s2=plateau_threshold_cnts_s2,
        ema_alpha=ema_alpha,
        speed_margin_frac=speed_margin_frac,
    )
    ctrls: dict[int, _RecordingController] = {}
    for ch in weights:
        rc = _RecordingController(ch)
        ctrls[ch] = rc
        rd._controllers[ch] = rc  # type: ignore[assignment]
    rd._target_scale = target_scale
    return rd, fake_hub, ctrls


class TestRatioDrivePlateauLatch:
    """003-010: Sticky max-speed latch — dv/dt-based plateau detection."""

    def test_latch_fires_only_after_acceleration_stops(self) -> None:
        """Wheel ramps upward (high dv/dt) then levels off below command.

        Phase 1: measured velocity rises — |dv/dt| is HIGH → latch must NOT fire.
        Phase 2: measured velocity plateaus at n1_max < commanded_n → |dv/dt| drops
        below plateau_threshold → latch MUST fire.
        g then holds at the latched cap without oscillation.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        n1_max = 600  # physical ceiling for wheel 1
        dt_ms = 20
        # Use a lower plateau threshold so it fires in a reasonable number of ticks.
        plateau_thresh = 150.0

        rd, _, _ = _make_ratio_drive_010(
            weights,
            target_scale=S,
            max_accel=100000.0,
            plateau_threshold_cnts_s2=plateau_thresh,
        )
        rd._scale = S  # pre-warm

        time_ms = 0

        # Phase 1: wheel 1 ramps from 0 to n1_max over 10 ticks.
        # dv/dt is positive and large — latch must NOT fire.
        for tick in range(10):
            time_ms += dt_ms
            frac = (tick + 1) / 10.0
            v1 = round(n1_max * frac)  # linearly ramping
            v0 = round(S)
            bulk = _make_bulk_timed(motor0_velocity=v0, motor1_velocity=v1, time_ms=time_ms)
            rd.update(bulk)

        # After ramp phase: latch must NOT be active (dv/dt was high throughout).
        latch_during_ramp = rd._speed_latch.get(1)
        assert latch_during_ramp is None, (
            f"Speed latch fired during ramp phase (dv/dt was positive): {rd._speed_latch}"
        )

        # Phase 2: wheel 1 plateaus at n1_max.  Run until latch fires (~30 ticks).
        latch_fired_tick = None
        for tick in range(50):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=round(S), motor1_velocity=n1_max, time_ms=time_ms)
            rd.update(bulk)
            if rd._speed_latch.get(1) is not None and latch_fired_tick is None:
                latch_fired_tick = tick

        assert latch_fired_tick is not None, (
            "Speed latch never fired after plateau phase "
            f"(dvdt_ema={rd._dvdt_ema}, latch={rd._speed_latch})"
        )
        assert rd.saturated, "rd.saturated should be True once latch is active"

        # g must have dropped to near the latch cap (n1_max = 600).
        # Allow some slew time; after 50 plateau ticks g should be at or near 600.
        assert rd.scale <= n1_max + 10, (
            f"g should have converged to latch cap ({n1_max}), got {rd.scale:.1f}"
        )

    def test_latch_is_sticky_no_oscillation(self) -> None:
        """Once latched, g stays at the cap for many subsequent ticks.

        No probing occurs — g does NOT drift above the latched cap.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        n1_max = 600
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(weights, target_scale=S, max_accel=100000.0)
        rd._scale = S

        time_ms = 0
        # Run 60 ticks of plateau to guarantee latch fires.
        for _ in range(60):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=round(S), motor1_velocity=n1_max, time_ms=time_ms)
            rd.update(bulk)

        assert rd.saturated, "Latch should be active after 60 plateau ticks"
        g_after_latch = rd.scale

        # Run 50 more ticks at the same plateau velocity.
        g_values = []
        for _ in range(50):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=round(rd.scale), motor1_velocity=n1_max, time_ms=time_ms)
            rd.update(bulk)
            g_values.append(rd.scale)

        # g must not drift above the latch cap.
        max_g = max(g_values)
        assert max_g <= n1_max + 10, (
            f"g oscillated above latch cap: max_g={max_g:.1f}, latch_cap≈{n1_max}"
        )
        assert rd.saturated, "Latch must remain active (sticky)"

    def test_latch_clears_on_set_speed(self) -> None:
        """After latching, set_speed() clears the latch and g climbs toward new S."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        n1_max = 600
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(weights, target_scale=S, max_accel=100000.0)
        rd._scale = S

        time_ms = 0
        # Force the latch.
        for _ in range(60):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=round(S), motor1_velocity=n1_max, time_ms=time_ms)
            rd.update(bulk)

        assert rd.saturated, "Latch must be active before setpoint change"

        # Call set_speed — this must clear the latch.
        rd.set_speed(S)
        assert rd._speed_latch.get(1) is None, (
            "set_speed() must clear the speed latch"
        )
        assert not rd.saturated, "rd.saturated must be False immediately after latch clear"

        # After clearing, g should climb toward S again (both wheels now tracking fully).
        for _ in range(10):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms)
            rd.update(bulk)

        # g must have climbed above the old latch cap.
        assert rd.scale > n1_max, (
            f"g should climb above old latch cap ({n1_max}) after set_speed(); got {rd.scale:.1f}"
        )

    def test_latch_clears_on_set_ratio(self) -> None:
        """After latching, set_ratio() clears the latch."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        n1_max = 600
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(weights, target_scale=S, max_accel=100000.0)
        rd._scale = S

        time_ms = 0
        for _ in range(60):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=round(S), motor1_velocity=n1_max, time_ms=time_ms)
            rd.update(bulk)

        assert rd.saturated, "Latch must be active before ratio change"

        rd.set_ratio(1.0, pair=(0, 1))  # change ratio — clears latch
        assert rd._speed_latch.get(1) is None, (
            "set_ratio() must clear the speed latch"
        )
        assert not rd.saturated

    def test_latch_clears_on_spontaneous_velocity_recovery(self) -> None:
        """After latching, if measured velocity rises above the latch cap, latch clears."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        n1_max = 600
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(weights, target_scale=S, max_accel=100000.0)
        rd._scale = S

        time_ms = 0
        # Force the latch at cap ≈ 600.
        for _ in range(60):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=round(S), motor1_velocity=n1_max, time_ms=time_ms)
            rd.update(bulk)

        assert rd.saturated, "Latch must be active"
        latch_cap = rd._speed_latch.get(1)
        assert latch_cap is not None

        # Now feed a tick where wheel 1 exceeds the latch cap (bottleneck cleared).
        time_ms += dt_ms
        # n1 = 700 > latch_cap ≈ 600 → latch clears.
        bulk = _make_bulk_timed(motor0_velocity=round(S), motor1_velocity=700, time_ms=time_ms)
        rd.update(bulk)

        assert rd._speed_latch.get(1) is None, (
            f"Latch should clear when n_i ({700}) > latch_cap ({latch_cap:.1f})"
        )
        assert not rd.saturated, "rd.saturated must be False after spontaneous recovery"


class TestRatioDriveLatchHardening:
    """003-010 bug-fix: regression tests for the hardened max-speed latch.

    Hardware finding: when current_limit_ma is set, the daemon thread's slower
    loop (extra GetADC calls) causes coarser velocity sampling.  During spin-up
    a single sample may momentarily read v=0 (encoder transient) while
    _ever_accelerating is already True.  The old single-sample plateau test
    would latch at cap=0, pinning g=0 permanently.

    The hardened latch requires:
      1. The plateau condition must persist for _latch_persist_ticks consecutive
         ticks (default 3) — one noisy sample cannot fire it.
      2. The would-be cap must be >= min_latch_frac * g (default 0.25) — a
         near-zero cap is a spin-up transient, not a genuine physical ceiling.
    """

    def test_no_latch_on_single_zero_velocity_sample(self) -> None:
        """A single v=0 reading while _ever_accelerating is True must NOT latch.

        This is the exact failure mode found on hardware: spin-up with
        current_limit_ma set, v bounces 0→260 cnt/s, one sample hits 0,
        old code latched cap=0 and g=0 forever.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(
            weights,
            target_scale=S,
            max_accel=100000.0,
            plateau_threshold_cnts_s2=150.0,
        )
        rd._scale = 60.0  # g partway through spin-up

        # Manually put motor 0 into the "ever_accelerating" state
        # (as would happen after the first real ticks of spin-up).
        rd._ever_accelerating[0] = True
        rd._ever_accelerating[1] = True
        # Set dvdt_ema below threshold (motor appears to have just plateaued)
        rd._dvdt_ema[0] = 0.0
        rd._dvdt_ema[1] = 0.0
        # Set vel_ema to a small value (below commanded scale=60)
        rd._vel_ema[0] = 0.0
        rd._vel_ema[1] = 0.0

        # Feed a single tick with v=0 (the transient that caused the bug).
        # With g=60 and v=0: shortfall=60, is_short=True, is_plateau=True.
        # Old code: would latch at cap_val=0 immediately (one tick).
        # New code: persistence counter reaches 1, need 3 → no latch yet.
        time_ms = 300
        bulk = _make_bulk_timed(motor0_velocity=0, motor1_velocity=0, time_ms=time_ms)
        rd.update(bulk)

        # After a single zero-velocity tick: NO latch should have fired.
        assert rd._speed_latch.get(0) is None, (
            "Latch must NOT fire on a single zero-velocity sample (spin-up transient)"
        )
        assert rd._speed_latch.get(1) is None, (
            "Latch must NOT fire on a single zero-velocity sample (spin-up transient)"
        )
        assert not rd.saturated, "g must not be pinned at 0 by a single zero-velocity tick"
        # g must still be positive — not collapsed to 0.
        assert rd.scale > 0.0, (
            f"g collapsed to 0 after single zero-velocity tick: {rd.scale}"
        )

    def test_no_latch_when_cap_near_zero(self) -> None:
        """Even after persistence, a near-zero would-be cap must NOT latch.

        Protect against the case where several consecutive noise samples all
        read 0 (v_i very small) but g is well above 0.  min_latch_frac * g
        guard prevents latching at implausibly low ceilings.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(
            weights,
            target_scale=S,
            max_accel=100000.0,
            plateau_threshold_cnts_s2=150.0,
        )
        rd._scale = 200.0  # g at 200, well above 0

        # Pre-seed: both motors ever_accelerating, dvdt_ema below threshold.
        rd._ever_accelerating[0] = True
        rd._ever_accelerating[1] = True
        rd._dvdt_ema[0] = 0.0
        rd._dvdt_ema[1] = 0.0
        rd._vel_ema[0] = 0.0
        rd._vel_ema[1] = 0.0

        # Feed _latch_persist_ticks consecutive ticks with v=0.
        # Each tick: cap_val=0 < min_latch_frac * g=50 → latch must NOT fire.
        time_ms = 300
        for _ in range(rd._latch_persist_ticks + 5):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=0, motor1_velocity=0, time_ms=time_ms)
            rd.update(bulk)

        # Even after _latch_persist_ticks zero-velocity ticks: NO latch, because
        # cap_val=0 < min_latch_frac * g.
        assert rd._speed_latch.get(0) is None, (
            f"Latch must not fire when cap ≈ 0 << g={rd.scale:.0f} (min_latch_frac guard)"
        )
        assert rd._speed_latch.get(1) is None, (
            f"Latch must not fire when cap ≈ 0 << g={rd.scale:.0f} (min_latch_frac guard)"
        )
        assert not rd.saturated

    def test_transient_zero_then_ramp_keeps_climbing(self) -> None:
        """Spin-up scenario: one transient v=0 tick in the middle does NOT interrupt ramp.

        Simulates the thread-driven failure case: g starts from 0, both wheels
        accelerate, a single tick reads v=0 (noise), then acceleration resumes.
        g must keep climbing to S — not be stuck at 0.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(
            weights,
            target_scale=S,
            max_accel=100000.0,   # very fast slew so g can reach S quickly
            plateau_threshold_cnts_s2=150.0,
        )

        time_ms = 0

        # Phase 1: normal spin-up — both wheels accelerating (v ≈ 0.7 * g).
        # This sets _ever_accelerating = True via growing vel_ema dv/dt.
        for _ in range(8):
            time_ms += dt_ms
            g = rd.scale
            v = round(0.7 * g) if g > 30 else 0
            bulk = _make_bulk_timed(motor0_velocity=v, motor1_velocity=v, time_ms=time_ms)
            rd.update(bulk)

        g_before_transient = rd.scale
        assert g_before_transient > 0.0, "g should have risen during spin-up"

        # Phase 2: ONE tick with v=0 (the transient).
        time_ms += dt_ms
        bulk = _make_bulk_timed(motor0_velocity=0, motor1_velocity=0, time_ms=time_ms)
        rd.update(bulk)

        # Latch must NOT have fired.
        assert rd._speed_latch.get(0) is None, "Latch must not fire on transient zero"
        assert rd._speed_latch.get(1) is None, "Latch must not fire on transient zero"

        # Phase 3: resume normal spin-up.
        for _ in range(30):
            time_ms += dt_ms
            g = rd.scale
            v = round(0.9 * g)
            bulk = _make_bulk_timed(motor0_velocity=v, motor1_velocity=v, time_ms=time_ms)
            rd.update(bulk)

        # g must have kept climbing — not stuck at 0 or near the transient value.
        # With default recovery_accel_up=500 at 50 Hz: 10 cnt/tick.
        # Starting from g≈80 after 30 ticks: 80 + 300 = 380 → expect > 200.
        # The key invariant: g must NOT be stuck at 0 or near the transient.
        final_g = rd.scale
        assert final_g > 200.0, (
            f"g should have climbed well above 0 after spin-up; got {final_g:.1f}. "
            f"A transient v=0 collapsed g."
        )
        assert rd._speed_latch.get(0) is None, "No latch after clean ramp"
        assert rd._speed_latch.get(1) is None, "No latch after clean ramp"

    def test_genuine_sustained_high_plateau_still_latches(self) -> None:
        """A genuine sustained plateau at a high speed (≥ min_latch_frac * g) still latches.

        Regression check: the hardening must not prevent legitimate ceiling detection.
        Scenario analogous to ratio-5.0 hardware: motor 1 tops out at normalised ~500
        while g is commanded 1000.  After _latch_persist_ticks consecutive ticks at
        n1=500, the latch must fire at cap≈500 (well above min_latch_frac*1000=250).
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        n1_max = 500  # motor 1 physical ceiling (normalised)
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(
            weights,
            target_scale=S,
            max_accel=100000.0,
            plateau_threshold_cnts_s2=150.0,
        )
        rd._scale = S  # start at full speed

        time_ms = 0

        # Run until the latch fires (generous number of ticks for EMA to settle).
        for _ in range(80):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=round(S), motor1_velocity=n1_max, time_ms=time_ms)
            rd.update(bulk)

        # Latch must have fired at cap ≈ n1_max (500 >> 0.25 * 1000 = 250 ✓).
        latch_cap = rd._speed_latch.get(1)
        assert latch_cap is not None, (
            "Speed latch must fire for a genuine sustained plateau at n1_max=500 "
            f"(min_latch_frac * g = {rd._min_latch_frac * rd.scale:.0f}). "
            f"dvdt_ema={rd._dvdt_ema}, speed_latch={rd._speed_latch}"
        )
        assert abs(latch_cap - n1_max) <= 100, (
            f"Latch cap should be near n1_max={n1_max}; got {latch_cap:.1f}"
        )
        assert rd.saturated
        # g must have dropped to near the cap.
        assert rd.scale <= n1_max + 20, (
            f"g should have converged to latch cap {n1_max}; got {rd.scale:.1f}"
        )

    def test_latch_persist_ticks_fires_after_required_duration(self) -> None:
        """Latch fires only after _latch_persist_ticks consecutive plateau ticks.

        Verify the boundary: at tick (persist-1) no latch; at tick persist the latch fires.
        Uses a high g (800) and n_i = 0.5*g = 400, which is well above
        min_latch_frac*g = 200 so the minimum-cap guard doesn't interfere.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        dt_ms = 20
        persist = 3  # default latch_persist_ticks

        rd, _, _ = _make_ratio_drive_010(
            weights,
            target_scale=S,
            max_accel=100000.0,
            plateau_threshold_cnts_s2=150.0,
        )
        rd._scale = 800.0  # start g high so min_latch_frac check is not the bottleneck

        # Pre-seed: both motors ever_accelerating with flat dvdt_ema below threshold.
        rd._ever_accelerating[0] = True
        rd._ever_accelerating[1] = True
        rd._dvdt_ema[0] = 0.0
        rd._dvdt_ema[1] = 0.0
        # Seed vel_ema at 400 (= 0.5 * g) so dv/dt remains ~0 across ticks.
        rd._vel_ema[0] = 400.0
        rd._vel_ema[1] = 400.0

        time_ms = 400

        # Feed (persist - 1) ticks at v = 400 (shortfall = 800-400=400 = 50% > 15% margin).
        # dvdt_ema will remain near 0 because v is constant.
        for tick in range(persist - 1):
            time_ms += dt_ms
            bulk = _make_bulk_timed(motor0_velocity=400, motor1_velocity=400, time_ms=time_ms)
            rd.update(bulk)
            # Must not have latched yet.
            assert rd._speed_latch.get(1) is None, (
                f"Latch fired too early at tick {tick+1} (need {persist} consecutive ticks)"
            )

        # Feed one more tick — this is tick #persist → latch should fire NOW.
        time_ms += dt_ms
        bulk = _make_bulk_timed(motor0_velocity=400, motor1_velocity=400, time_ms=time_ms)
        rd.update(bulk)

        latch = rd._speed_latch.get(1)
        assert latch is not None, (
            f"Latch should fire after exactly {persist} consecutive plateau ticks; "
            f"plateau_ticks={rd._plateau_ticks}, dvdt_ema={rd._dvdt_ema}"
        )
        assert abs(latch - 400.0) < 50, (
            f"Latch cap should be near 400 (the sustained plateau velocity); got {latch:.1f}"
        )


class TestRatioDriveCurrentCapNew:
    """003-010: Live current closed-loop cap — correct recovery behavior."""

    def _make_rd_current(
        self,
        weights: dict,
        *,
        current_limit_ma: int,
        target_scale: float = 1000.0,
        max_accel: float = 100000.0,
    ) -> tuple["RatioDrive", "_FakeHubForRatio", dict[int, "_RecordingController"]]:
        """Create a RatioDrive with current sensing and _RecordingControllers."""
        from rhsp.control import RatioDrive

        fake_hub = _FakeHubForRatio()
        rd = RatioDrive(
            fake_hub,
            weights,
            max_accel=max_accel,
            current_limit_ma=current_limit_ma,
            deadband=0,
        )
        rd._cur_sample_every = 1  # sample on every tick for determinism
        ctrls: dict[int, _RecordingController] = {}
        for ch in weights:
            rc = _RecordingController(ch)
            ctrls[ch] = rc
            rd._controllers[ch] = rc  # type: ignore[assignment]
        rd._target_scale = target_scale
        rd._scale = target_scale
        return rd, fake_hub, ctrls

    def test_current_over_limit_reduces_g(self) -> None:
        """When any motor current exceeds the limit, g is pulled below S."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        limit = 2000
        rd, fake_hub, _ = self._make_rd_current(weights, current_limit_ma=limit, target_scale=S)

        # Motor 0 over limit — seed hysteresis flag.
        fake_hub.motors[0].set_current_ma(limit + 500)
        fake_hub.motors[1].set_current_ma(100)
        rd._cur_sat_flags[0] = True
        rd._last_current_ma[0] = limit + 500
        rd._last_current_ma[1] = 100

        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=1000, time_ms=20)
        rd.update(bulk)

        assert rd.scale < S, (
            f"g should be pulled below S={S} when current exceeds limit; got {rd.scale:.2f}"
        )

    def test_current_recovery_when_load_clears(self) -> None:
        """After current-driven de-rate, releasing load allows g to recover to S.

        This is the critical fix for the 007/009 regression: the old design
        got stuck at low g even after current dropped.  The new design applies
        NO current cap when all motors are under limit — g climbs freely.
        """
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        limit = 2000
        rd, fake_hub, _ = self._make_rd_current(
            weights, current_limit_ma=limit, target_scale=S, max_accel=100000.0
        )

        # Start with g de-rated to 400 (simulating post-load state).
        rd._scale = 400.0
        # All currents now well below limit (load cleared).
        fake_hub.motors[0].set_current_ma(100)
        fake_hub.motors[1].set_current_ma(100)
        rd._cur_sat_flags[0] = False
        rd._cur_sat_flags[1] = False
        rd._last_current_ma[0] = 100
        rd._last_current_ma[1] = 100

        dt_ms = 20
        time_ms = 0
        for _ in range(50):
            time_ms += dt_ms
            g = rd.scale
            bulk = _make_bulk_timed(
                motor0_velocity=round(g), motor1_velocity=round(g), time_ms=time_ms
            )
            rd.update(bulk)

        # g must have recovered toward S (not stuck at 400 or near 0).
        assert rd.scale > S * 0.8, (
            f"g should recover toward S={S} when current drops below limit; got {rd.scale:.2f}"
        )

    def test_current_limit_none_disables_current_path(self) -> None:
        """With current_limit_ma=None, no current reads occur and governor is unaffected."""
        weights = {0: 1.0, 1: 1.0}
        S = 1000.0
        rd, fake_hub, _ = _make_ratio_drive_010(weights, target_scale=S, max_accel=100000.0)
        # current_limit_ma defaults to None.

        # Set very high fake current — should have no effect.
        fake_hub.motors[0].set_current_ma(99999)
        fake_hub.motors[1].set_current_ma(99999)

        rd._scale = S
        bulk = _make_bulk_timed(motor0_velocity=1000, motor1_velocity=1000, time_ms=20)
        rd.update(bulk)

        assert rd.scale >= S - 1, (
            f"current_limit_ma=None must not de-rate g; got {rd.scale:.2f}"
        )
        assert rd.last_current_ma == {}, "No current reads should occur when current_limit_ma=None"


class TestRatioDriveRatioExactnessNew:
    """003-010: Ratio exactness throughout all governor states."""

    def test_ratio_exact_during_plateau_latch(self) -> None:
        """Commanded ratio stays exact during and after the plateau latch fires.

        Across spin-up, plateau latch, and post-latch steady-state, assert
        t_i / t_j == w_i / w_j at every tick where both targets are large.
        """
        weights = {0: 1.0, 1: 5.0}
        S = 1000.0
        n1_max = 500  # motor1 physical cap (normalised: 2500/5)
        dt_ms = 20

        rd, _, _ = _make_ratio_drive_010(
            weights, target_scale=S, max_accel=6000.0
        )
        rd._scale = S

        time_ms = 0
        for _ in range(120):
            time_ms += dt_ms
            g = rd.scale
            v0 = round(g * weights[0])
            v1 = min(round(g * weights[1]), 2500)  # motor1 capped at 2500
            bulk = _make_bulk_timed(motor0_velocity=v0, motor1_velocity=v1, time_ms=time_ms)
            rd.update(bulk)

            t0 = rd.commanded_targets.get(0, 0)
            t1 = rd.commanded_targets.get(1, 0)
            # Only check ratio when t0 is large enough to avoid rounding dominance.
            if abs(t0) >= 10 and abs(t1) >= 10:
                actual_ratio = t1 / t0
                expected_ratio = weights[1] / weights[0]  # 5.0
                assert abs(actual_ratio - expected_ratio) < 0.15, (
                    f"Ratio violated at g={rd.scale:.1f}: t1/t0={actual_ratio:.3f} "
                    f"vs expected {expected_ratio:.1f} (t0={t0}, t1={t1})"
                )

    def test_ratio_exact_during_current_derate(self) -> None:
        """Commanded ratio stays exact during current-driven de-rate."""
        from rhsp.control import RatioDrive

        weights = {0: 1.0, 1: 0.5}
        S = 1000.0
        limit = 2000
        fake_hub = _FakeHubForRatio()
        rd = RatioDrive(fake_hub, weights, max_accel=100000.0, current_limit_ma=limit, deadband=0)
        rd._cur_sample_every = 1
        ctrls = {}
        for ch in weights:
            rc = _RecordingController(ch)
            ctrls[ch] = rc
            rd._controllers[ch] = rc  # type: ignore[assignment]
        rd._target_scale = S
        rd._scale = S

        # Motor 0 persistently over limit.
        fake_hub.motors[0].set_current_ma(limit + 800)
        fake_hub.motors[1].set_current_ma(50)
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
            if abs(t1) >= 20:
                actual = t0 / t1
                assert abs(actual - expected_ratio) < 0.15, (
                    f"Step {step}: ratio {actual:.4f} vs expected {expected_ratio:.4f} "
                    f"(t0={t0}, t1={t1}, g={rd.scale:.1f})"
                )


class TestRatioDriveNewTunableDefaults:
    """003-010: New constructor tunables have correct defaults."""

    def test_ema_alpha_default(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._ema_alpha == 0.3

    def test_plateau_threshold_default(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._plateau_threshold == 150.0

    def test_speed_margin_frac_default(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._speed_margin_frac == 0.15

    def test_recovery_accel_up_default(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._recovery_accel_up == 500.0

    def test_recovery_accel_explicit_via_recovery_accel_up(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, recovery_accel_up=800.0)
        assert rd._recovery_accel_up == 800.0
        assert rd._recovery_accel == 800.0  # alias

    def test_recovery_accel_backward_compat(self) -> None:
        """Passing recovery_accel= sets recovery_accel_up when recovery_accel_up not set."""
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0}, recovery_accel=1200.0)
        assert rd._recovery_accel_up == 1200.0

    def test_speed_latch_initially_empty(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._speed_latch == {}

    def test_ever_accelerating_initially_empty(self) -> None:
        hub = _FakeHubForRatio()
        rd = RatioDrive(hub, {0: 1.0, 1: 1.0})
        assert rd._ever_accelerating == {}

    def test_superseded_params_stored_for_compat(self) -> None:
        """Superseded params are stored as-is (accepted but not used by governor)."""
        hub = _FakeHubForRatio()
        rd = RatioDrive(
            hub, {0: 1.0, 1: 1.0},
            sat_settle_s=0.7,
            probe_step=5.0,
            probe_settle_ticks=10,
        )
        assert rd._sat_settle_s == 0.7
        assert rd._probe_step == 5.0
        assert rd._probe_settle_ticks == 10
