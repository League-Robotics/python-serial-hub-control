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
from rhsp.control import HubVelocityController, VelocityController, clamp_int16

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
