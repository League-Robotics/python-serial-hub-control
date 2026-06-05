"""Tests for rhsp.hub — Hub init_peripherals, keep_alive, rollback.

All tests are hardware-free: they use FakeHub over a LoopbackTransport.

Coverage:
- ``init_peripherals()`` emits the correct byte sequence:
    4 × (SetMotorChannelMode + SetMotorConstantPower)
    + 6 × SetServoConfiguration
    in that exact order.
- Mid-sequence NackError in ``init_peripherals()`` causes ``fail_safe()``
    to be sent and the NackError to be re-raised.
- ``hub.address`` is a single int; no ``module`` or ``destinationModule``.
- ``keep_alive()`` sends a KeepAlive frame (0x7F04) and receives ACK.
- Peripheral list lengths are correct (4 motors, 6 servos, 8 dio, 4 adc, 4 i2c).
"""

from __future__ import annotations

import pytest

from rhsp.catalogue import COMMANDS, runtime_packet_id
from rhsp.errors import NackError
from rhsp.hub import Hub
from rhsp.session import Session
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# Packet-type constants
# ---------------------------------------------------------------------------

_DEKA_BASE: int = 0x1000

_SET_MOTOR_CHANNEL_MODE_TYPE: int = runtime_packet_id(
    COMMANDS["SetMotorChannelMode"], _DEKA_BASE
)
_SET_MOTOR_CONSTANT_POWER_TYPE: int = runtime_packet_id(
    COMMANDS["SetMotorConstantPower"], _DEKA_BASE
)
_SET_SERVO_CONFIGURATION_TYPE: int = runtime_packet_id(
    COMMANDS["SetServoConfiguration"], _DEKA_BASE
)
_KEEPALIVE_TYPE: int = 0x7F04
_FAILSAFE_TYPE: int = 0x7F05


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_hub(
    *,
    address: int = 2,
    retries: int = 0,
    timeout: float = 0.5,
) -> tuple[LoopbackTransport, FakeHub, Hub]:
    """Return a wired-up (transport, fake_hub, hub) triple.

    The FakeHub runs in a background thread so that Session transactions
    complete without manual interleaving.
    """
    transport = LoopbackTransport()
    fake = FakeHub(transport)
    fake.run_in_thread()

    session = Session(transport, retries=retries, timeout=timeout)
    # Set deka_base so runtime_packet_id works correctly.
    session.deka_base = _DEKA_BASE

    hub = Hub(session=session, address=address)
    return transport, fake, hub


# ---------------------------------------------------------------------------
# Hub construction
# ---------------------------------------------------------------------------


class TestHubConstruction:
    """Hub.__init__ must build device lists without issuing transactions."""

    def test_address_attribute(self) -> None:
        _, fake, hub = _make_hub(address=2)
        try:
            assert hub.address == 2
            assert isinstance(hub.address, int)
        finally:
            fake.stop()

    def test_no_module_attribute(self) -> None:
        _, fake, hub = _make_hub()
        try:
            assert not hasattr(hub, "module")
            assert not hasattr(hub, "destinationModule")
        finally:
            fake.stop()

    def test_motor_count(self) -> None:
        _, fake, hub = _make_hub()
        try:
            assert len(hub.motors) == 4
        finally:
            fake.stop()

    def test_servo_count(self) -> None:
        _, fake, hub = _make_hub()
        try:
            assert len(hub.servos) == 6
        finally:
            fake.stop()

    def test_dio_count(self) -> None:
        _, fake, hub = _make_hub()
        try:
            assert len(hub.dio) == 8
        finally:
            fake.stop()

    def test_adc_count(self) -> None:
        _, fake, hub = _make_hub()
        try:
            assert len(hub.adc) == 4
        finally:
            fake.stop()

    def test_i2c_count(self) -> None:
        _, fake, hub = _make_hub()
        try:
            assert len(hub.i2c) == 4
        finally:
            fake.stop()

    def test_no_transactions_on_construction(self) -> None:
        """__init__ must not issue any hub transactions."""
        _, fake, hub = _make_hub()
        try:
            assert len(fake.requests) == 0
        finally:
            fake.stop()

    def test_parent_flag_default_false(self) -> None:
        _, fake, hub = _make_hub()
        try:
            assert hub.parent is False
        finally:
            fake.stop()

    def test_parent_flag_set(self) -> None:
        transport = LoopbackTransport()
        fake = FakeHub(transport)
        session = Session(transport, retries=0, timeout=0.5)
        hub = Hub(session=session, address=3, parent=True)
        assert hub.parent is True


# ---------------------------------------------------------------------------
# init_peripherals — command order
# ---------------------------------------------------------------------------


class TestInitPeripherals:
    """init_peripherals() must emit motor + servo commands in exact order."""

    def test_command_sequence_and_count(self) -> None:
        """4×(SetMotorChannelMode + SetMotorConstantPower) + 6×SetServoConfiguration."""
        _, fake, hub = _make_hub()
        try:
            hub.init_peripherals()
        finally:
            fake.stop()

        # Expected command sequence:
        # ch 0: SetMotorChannelMode, SetMotorConstantPower
        # ch 1: SetMotorChannelMode, SetMotorConstantPower
        # ch 2: SetMotorChannelMode, SetMotorConstantPower
        # ch 3: SetMotorChannelMode, SetMotorConstantPower
        # ch 0–5: SetServoConfiguration × 6
        expected_types = (
            [_SET_MOTOR_CHANNEL_MODE_TYPE, _SET_MOTOR_CONSTANT_POWER_TYPE] * 4
            + [_SET_SERVO_CONFIGURATION_TYPE] * 6
        )

        actual_types = [pkt.packet_type for pkt in fake.requests]
        assert actual_types == expected_types, (
            f"Expected {[hex(t) for t in expected_types]}, "
            f"got {[hex(t) for t in actual_types]}"
        )

    def test_motor_channel_mode_payload(self) -> None:
        """Each SetMotorChannelMode must use CONSTANT_POWER (0) and float_at_zero=1."""
        _, fake, hub = _make_hub()
        try:
            hub.init_peripherals()
        finally:
            fake.stop()

        # Indices 0, 2, 4, 6 in the request list are SetMotorChannelMode packets.
        from rhsp.codec import decode_payload
        cmd = COMMANDS["SetMotorChannelMode"]
        for motor_idx, req_idx in enumerate([0, 2, 4, 6]):
            pkt = fake.requests[req_idx]
            decoded = decode_payload(cmd.fields, pkt.payload)
            # The codec may return raw bytes for the last field; coerce to int.
            def _to_int(v: object, nbytes: int = 1) -> int:
                if isinstance(v, (bytes, bytearray)):
                    return int.from_bytes(bytes(v)[:nbytes].ljust(nbytes, b"\x00"), "little")
                return int(v)  # type: ignore[arg-type]

            assert _to_int(decoded["motorChannel"]) == motor_idx, (
                f"Motor {motor_idx}: expected channel {motor_idx}, "
                f"got {decoded['motorChannel']}"
            )
            assert _to_int(decoded["motorMode"]) == 0, (
                f"Motor {motor_idx}: expected CONSTANT_POWER (0), "
                f"got {decoded['motorMode']}"
            )
            assert _to_int(decoded["floatAtZero"]) == 1, (
                f"Motor {motor_idx}: expected floatAtZero=1, "
                f"got {decoded['floatAtZero']}"
            )

    def test_motor_constant_power_zero(self) -> None:
        """Each SetMotorConstantPower must set power = 0."""
        _, fake, hub = _make_hub()
        try:
            hub.init_peripherals()
        finally:
            fake.stop()

        from rhsp.codec import decode_payload
        cmd = COMMANDS["SetMotorConstantPower"]

        def _to_int(v: object, nbytes: int = 1, signed: bool = False) -> int:
            if isinstance(v, (bytes, bytearray)):
                return int.from_bytes(bytes(v)[:nbytes].ljust(nbytes, b"\x00"), "little", signed=signed)
            return int(v)  # type: ignore[arg-type]

        for motor_idx, req_idx in enumerate([1, 3, 5, 7]):
            pkt = fake.requests[req_idx]
            decoded = decode_payload(cmd.fields, pkt.payload)
            assert _to_int(decoded["motorChannel"]) == motor_idx
            assert _to_int(decoded["powerLevel"], nbytes=2, signed=True) == 0

    def test_servo_configuration_frame_period(self) -> None:
        """Each SetServoConfiguration must use framePeriod = 20000."""
        _, fake, hub = _make_hub()
        try:
            hub.init_peripherals()
        finally:
            fake.stop()

        from rhsp.codec import decode_payload
        cmd = COMMANDS["SetServoConfiguration"]

        def _to_int(v: object, nbytes: int = 1) -> int:
            if isinstance(v, (bytes, bytearray)):
                return int.from_bytes(bytes(v)[:nbytes].ljust(nbytes, b"\x00"), "little")
            return int(v)  # type: ignore[arg-type]

        for servo_idx, req_idx in enumerate(range(8, 14)):
            pkt = fake.requests[req_idx]
            decoded = decode_payload(cmd.fields, pkt.payload)
            assert _to_int(decoded["servoChannel"]) == servo_idx
            assert _to_int(decoded["framePeriod"], nbytes=2) == 20000


# ---------------------------------------------------------------------------
# init_peripherals — rollback on error
# ---------------------------------------------------------------------------


class TestInitPeripheralsRollback:
    """A NACK mid-sequence must trigger fail_safe() and re-raise NackError."""

    def test_nack_triggers_failsafe_and_reraises(self) -> None:
        """NACK on SetMotorConstantPower for channel 2 → FailSafe sent + NackError raised."""
        _, fake, hub = _make_hub()
        try:
            fake.set_nack("SetMotorConstantPower", nack_code=1)
            with pytest.raises(NackError):
                hub.init_peripherals()
        finally:
            fake.stop()

        # After the NackError, FailSafe must have been sent.
        failsafe_types = [
            pkt.packet_type
            for pkt in fake.requests
            if pkt.packet_type == _FAILSAFE_TYPE
        ]
        assert len(failsafe_types) >= 1, (
            "Expected at least one FailSafe frame after NackError in init_peripherals"
        )

    def test_exception_is_reraised_unchanged(self) -> None:
        """The original NackError is re-raised (not wrapped)."""
        _, fake, hub = _make_hub()
        try:
            fake.set_nack("SetMotorChannelMode", nack_code=5)
            with pytest.raises(NackError) as exc_info:
                hub.init_peripherals()
        finally:
            fake.stop()

        assert exc_info.value.code == 5


# ---------------------------------------------------------------------------
# keep_alive
# ---------------------------------------------------------------------------


class TestKeepAlive:
    """keep_alive() must send KeepAlive (0x7F04) and receive ACK."""

    def test_keep_alive_sends_keepalive_frame(self) -> None:
        _, fake, hub = _make_hub()
        try:
            hub.keep_alive()
        finally:
            fake.stop()

        ka_packets = [
            pkt for pkt in fake.requests if pkt.packet_type == _KEEPALIVE_TYPE
        ]
        assert len(ka_packets) == 1, (
            f"Expected 1 KeepAlive frame, got {len(ka_packets)}"
        )

    def test_keep_alive_no_exception(self) -> None:
        """keep_alive() must complete without raising."""
        _, fake, hub = _make_hub()
        try:
            hub.keep_alive()  # Must not raise.
        finally:
            fake.stop()
