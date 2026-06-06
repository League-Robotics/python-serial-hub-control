"""Tests for Hub-managed LED: heartbeat re-assert + status-clear on connect.

These tests exercise the behaviour added in ticket 002-001:

(a) Heartbeat re-asserts a stored LED pattern after a simulated
    ``KeepAliveTimeout | FailSafe`` latch.
(b) ``init_peripherals()`` issues ``GetModuleStatus(clear=True)`` before any
    motor or servo init command.
(c) A simulated non-routine fault (over-temp / battery-low) is NOT cleared;
    ``SetModuleLEDPattern`` is NOT re-called; a WARNING is logged; the
    ``on_error`` callback is invoked with the fault flags.

Additional coverage:
- ``set_led_color()`` stores ``_desired_led_pattern`` and sends immediately.
- ``clear_led_color()`` sets ``_desired_led_pattern = None``.
- Heartbeat skips the status read when no pattern is stored.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

import pytest

from rhsp.catalogue import COMMANDS, runtime_packet_id
from rhsp.enums import ModuleStatusBits
from rhsp.hub import Hub
from rhsp.session import Session
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# Packet-type constants
# ---------------------------------------------------------------------------

_DEKA_BASE: int = 0x1000

_GET_MODULE_STATUS_TYPE: int = 0x7F03
_SET_MODULE_LED_PATTERN_TYPE: int = runtime_packet_id(
    COMMANDS["SetModuleLEDPattern"], _DEKA_BASE
)
_SET_MOTOR_CHANNEL_MODE_TYPE: int = runtime_packet_id(
    COMMANDS["SetMotorChannelMode"], _DEKA_BASE
)

# ModuleStatusBits values used in tests.
_KA_TIMEOUT = int(ModuleStatusBits.KeepAliveTimeout)
_FAIL_SAFE = int(ModuleStatusBits.FailSafe)
_ROUTINE_LATCH = _KA_TIMEOUT | _FAIL_SAFE
_OVER_TEMP = int(ModuleStatusBits.ControllerOverTemp)
_BATTERY_LOW = int(ModuleStatusBits.BatteryLow)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_hub(
    *,
    address: int = 2,
    timeout: float = 0.5,
) -> tuple[LoopbackTransport, FakeHub, Hub]:
    """Return a wired-up (transport, fake_hub, hub) triple.

    The FakeHub runs in a background thread so that Session transactions
    complete without manual interleaving.
    """
    transport = LoopbackTransport()
    fake = FakeHub(transport)
    fake.run_in_thread()

    session = Session(transport, retries=0, timeout=timeout)
    session.deka_base = _DEKA_BASE

    hub = Hub(session=session, address=address)
    return transport, fake, hub


# ---------------------------------------------------------------------------
# set_led_color / clear_led_color — basic storage behaviour
# ---------------------------------------------------------------------------


class TestSetAndClearLedColor:
    """set_led_color() stores the pattern; clear_led_color() removes it."""

    def test_set_led_color_stores_desired_pattern(self) -> None:
        """After set_led_color(r, g, b) the 16-step pattern is stored."""
        _, fake, hub = _make_hub()
        try:
            hub.set_led_color(255, 128, 0)
        finally:
            fake.stop()

        assert hub._desired_led_pattern is not None
        assert len(hub._desired_led_pattern) == 16
        # Each step is (r, g, b, t=1).
        for step in hub._desired_led_pattern:
            assert step == (255, 128, 0, 1)

    def test_set_led_color_sends_pattern_immediately(self) -> None:
        """set_led_color() sends SetModuleLEDPattern without waiting for heartbeat."""
        _, fake, hub = _make_hub()
        try:
            hub.set_led_color(0, 0, 255)
        finally:
            fake.stop()

        led_pkts = [
            pkt for pkt in fake.requests
            if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        ]
        assert len(led_pkts) >= 1

    def test_set_led_color_clears_status_before_pattern(self) -> None:
        """set_led_color() calls GetModuleStatus(clear=True) before SetModuleLEDPattern."""
        _, fake, hub = _make_hub()
        try:
            hub.set_led_color(0, 255, 0)
        finally:
            fake.stop()

        # At least one GetModuleStatus before the LED pattern send.
        names = fake.call_log
        assert "GetModuleStatus" in names
        assert "SetModuleLEDPattern" in names
        gms_idx = names.index("GetModuleStatus")
        led_idx = names.index("SetModuleLEDPattern")
        assert gms_idx < led_idx

    def test_desired_pattern_is_none_initially(self) -> None:
        """_desired_led_pattern starts as None."""
        _, fake, hub = _make_hub()
        try:
            assert hub._desired_led_pattern is None
        finally:
            fake.stop()

    def test_clear_led_color_sets_pattern_to_none(self) -> None:
        """clear_led_color() removes the stored pattern without sending any packet."""
        _, fake, hub = _make_hub()
        try:
            hub.set_led_color(255, 0, 0)
            request_count_before = len(fake.requests)
            hub.clear_led_color()
        finally:
            fake.stop()

        assert hub._desired_led_pattern is None
        # No additional packet should have been sent.
        assert len(fake.requests) == request_count_before


# ---------------------------------------------------------------------------
# Test (b) — init_peripherals call order
# ---------------------------------------------------------------------------


class TestInitPeripheralsStatusClear:
    """init_peripherals() must issue GetModuleStatus(clear=True) before motor/servo init."""

    def test_get_module_status_before_motor_init(self) -> None:
        """GetModuleStatus appears in call_log before the first SetMotorChannelMode."""
        _, fake, hub = _make_hub()
        try:
            hub.init_peripherals()
        finally:
            fake.stop()

        names = fake.call_log
        assert "GetModuleStatus" in names, "GetModuleStatus not found in call log"
        assert "SetMotorChannelMode" in names, "SetMotorChannelMode not found in call log"

        gms_idx = names.index("GetModuleStatus")
        motor_idx = names.index("SetMotorChannelMode")
        assert gms_idx < motor_idx, (
            f"GetModuleStatus (idx {gms_idx}) must appear before "
            f"SetMotorChannelMode (idx {motor_idx})"
        )

    def test_get_module_status_before_servo_init(self) -> None:
        """GetModuleStatus appears in call_log before the first SetServoConfiguration."""
        _, fake, hub = _make_hub()
        try:
            hub.init_peripherals()
        finally:
            fake.stop()

        names = fake.call_log
        assert "GetModuleStatus" in names
        assert "SetServoConfiguration" in names

        gms_idx = names.index("GetModuleStatus")
        servo_idx = names.index("SetServoConfiguration")
        assert gms_idx < servo_idx

    def test_get_module_status_is_first_call(self) -> None:
        """GetModuleStatus is literally the first command in init_peripherals."""
        _, fake, hub = _make_hub()
        try:
            hub.init_peripherals()
        finally:
            fake.stop()

        assert fake.call_log[0] == "GetModuleStatus", (
            f"Expected first call to be GetModuleStatus, got {fake.call_log[0]!r}"
        )


# ---------------------------------------------------------------------------
# Test (a) — heartbeat re-asserts LED pattern after routine latch
# ---------------------------------------------------------------------------


class TestHeartbeatReassert:
    """After a KeepAliveTimeout|FailSafe latch, the heartbeat re-asserts the stored LED pattern."""

    def test_heartbeat_reasserts_pattern_on_routine_latch(self) -> None:
        """Heartbeat re-sends SetModuleLEDPattern when status is routine latch only."""
        _, fake, hub = _make_hub()
        try:
            # Store a desired pattern without starting the background thread.
            hub._desired_led_pattern = [(0, 255, 0, 1)] * 16

            # Script the FakeHub: two GetModuleStatus calls.
            # First read (clear=False): return KeepAliveTimeout | FailSafe.
            # Second read (clear=True): return 0 (cleared).
            fake.set_module_status(_ROUTINE_LATCH)
            fake.set_module_status(0)

            # Record how many SetModuleLEDPattern calls existed before.
            pre_count = sum(
                1 for pkt in fake.requests
                if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
            )

            # Invoke the post-keepalive logic directly (no live thread needed).
            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        # Exactly one additional SetModuleLEDPattern must have been sent.
        post_count = sum(
            1 for pkt in fake.requests
            if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        )
        assert post_count == pre_count + 1, (
            f"Expected one re-assert (total {pre_count + 1}), "
            f"got {post_count}"
        )

    def test_heartbeat_does_not_reassert_when_no_pattern(self) -> None:
        """No status read or LED send occurs when _desired_led_pattern is None."""
        _, fake, hub = _make_hub()
        try:
            assert hub._desired_led_pattern is None
            fake.set_module_status(_ROUTINE_LATCH)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        # No GetModuleStatus should have been sent.
        gms_pkts = [
            pkt for pkt in fake.requests
            if pkt.packet_type == _GET_MODULE_STATUS_TYPE
        ]
        assert len(gms_pkts) == 0

    def test_heartbeat_no_reassert_when_status_is_zero(self) -> None:
        """When module status is 0, SetModuleLEDPattern is NOT re-sent."""
        _, fake, hub = _make_hub()
        try:
            hub._desired_led_pattern = [(255, 0, 0, 1)] * 16
            # Status reads zero (no latch).
            fake.set_module_status(0)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        led_pkts = [
            pkt for pkt in fake.requests
            if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        ]
        assert len(led_pkts) == 0, (
            "SetModuleLEDPattern should not be sent when status is 0"
        )

    def test_full_keepalive_cycle_via_background_thread(self) -> None:
        """Integration: with heartbeat running, LED is re-asserted without caller intervention.

        Timeline of GetModuleStatus calls:
          1. set_led_color() issues one GetModuleStatus(clear=True) — consumes entry #1.
          2. Heartbeat _heartbeat_post_keepalive issues GetModuleStatus(clear=False) — entry #2.
          3. Heartbeat _heartbeat_post_keepalive issues GetModuleStatus(clear=True) — entry #3.
        We queue: [0 (for set_led_color), ROUTINE_LATCH (for heartbeat read), 0 (for clear)].
        """
        _, fake, hub = _make_hub()
        try:
            # Queue:
            #  entry 1 (consumed by set_led_color's clear): status=0 (no latch)
            #  entry 2 (consumed by heartbeat read clear=False): ROUTINE_LATCH
            #  entry 3 (sticky; consumed by heartbeat clear=True): 0
            fake.set_module_status(0)
            fake.set_module_status(_ROUTINE_LATCH)
            fake.set_module_status(0)

            hub.set_led_color(128, 0, 255)  # stores pattern + sends immediately
            # Count LED sends after set_led_color (before heartbeat fires).
            initial_led_count = sum(
                1 for pkt in fake.requests
                if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
            )

            # Start keepalive with a short interval; wait for at least one tick.
            hub.start_keepalive(interval=0.1)
            time.sleep(0.5)  # allow at least one heartbeat to fire
            hub.stop_keepalive()
        finally:
            fake.stop()

        total_led_count = sum(
            1 for pkt in fake.requests
            if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        )
        assert total_led_count > initial_led_count, (
            "Expected heartbeat to re-assert LED pattern at least once"
        )


# ---------------------------------------------------------------------------
# Test (c) — non-routine fault suppresses re-assert, logs WARNING, calls on_error
# ---------------------------------------------------------------------------


class TestNonRoutineFaultHandling:
    """Non-routine faults must NOT clear/re-assert, must log WARNING, must call on_error."""

    def test_overtemp_does_not_clear_status(self) -> None:
        """When ControllerOverTemp is set, GetModuleStatus(clear=True) is NOT called."""
        _, fake, hub = _make_hub()
        try:
            hub._desired_led_pattern = [(255, 0, 0, 1)] * 16
            fake.set_module_status(_OVER_TEMP)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        # Only one GetModuleStatus should have been sent (the read with clear=False).
        gms_pkts = [
            pkt for pkt in fake.requests
            if pkt.packet_type == _GET_MODULE_STATUS_TYPE
        ]
        assert len(gms_pkts) == 1, (
            f"Expected 1 GetModuleStatus (no clear), got {len(gms_pkts)}"
        )

    def test_overtemp_does_not_reassert_led(self) -> None:
        """When ControllerOverTemp is set, SetModuleLEDPattern is NOT re-sent."""
        _, fake, hub = _make_hub()
        try:
            hub._desired_led_pattern = [(0, 0, 255, 1)] * 16
            fake.set_module_status(_OVER_TEMP)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        led_pkts = [
            pkt for pkt in fake.requests
            if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        ]
        assert len(led_pkts) == 0, (
            "SetModuleLEDPattern must NOT be sent when ControllerOverTemp is set"
        )

    def test_battery_low_does_not_reassert_led(self) -> None:
        """When BatteryLow is set, SetModuleLEDPattern is NOT re-sent."""
        _, fake, hub = _make_hub()
        try:
            hub._desired_led_pattern = [(0, 255, 255, 1)] * 16
            fake.set_module_status(_BATTERY_LOW)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        led_pkts = [
            pkt for pkt in fake.requests
            if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        ]
        assert len(led_pkts) == 0

    def test_overtemp_logs_warning(self, caplog: pytest.LogCaptureFixture) -> None:
        """A WARNING log is emitted when ControllerOverTemp is detected."""
        _, fake, hub = _make_hub()
        try:
            hub._desired_led_pattern = [(255, 0, 0, 1)] * 16
            fake.set_module_status(_OVER_TEMP)

            with caplog.at_level(logging.WARNING, logger="rhsp.hub"):
                hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        assert any(
            "non-routine hub fault" in record.message.lower()
            or "non-routine hub fault" in record.getMessage().lower()
            for record in caplog.records
            if record.levelno >= logging.WARNING
        ), f"Expected WARNING log for non-routine fault; got: {[r.getMessage() for r in caplog.records]}"

    def test_overtemp_invokes_on_error_callback(self) -> None:
        """The on_error callback is invoked with the fault flags integer."""
        _, fake, hub = _make_hub()
        received_flags: list[int] = []

        def _on_error(flags: int) -> None:
            received_flags.append(flags)

        try:
            hub._desired_led_pattern = [(255, 0, 0, 1)] * 16
            hub._on_error = _on_error
            fake.set_module_status(_OVER_TEMP)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        assert len(received_flags) == 1, (
            f"Expected on_error called once; got {received_flags}"
        )
        assert received_flags[0] & _OVER_TEMP, (
            f"Expected ControllerOverTemp bit in flags; got 0x{received_flags[0]:02x}"
        )

    def test_battery_low_invokes_on_error_callback(self) -> None:
        """on_error is invoked with BatteryLow flags."""
        _, fake, hub = _make_hub()
        received_flags: list[int] = []

        try:
            hub._desired_led_pattern = [(255, 255, 0, 1)] * 16

            # Register on_error via start_keepalive (ensures the kwarg path works).
            hub.start_keepalive(interval=2.0, on_error=lambda f: received_flags.append(f))
            hub.stop_keepalive()

            # Now invoke directly.
            fake.set_module_status(_BATTERY_LOW)
            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        assert len(received_flags) == 1
        assert received_flags[0] & _BATTERY_LOW

    def test_on_error_not_called_for_routine_latch(self) -> None:
        """on_error is NOT invoked when only routine KeepAliveTimeout|FailSafe bits are set."""
        _, fake, hub = _make_hub()
        called: list[int] = []

        try:
            hub._desired_led_pattern = [(0, 128, 0, 1)] * 16
            hub._on_error = lambda f: called.append(f)
            # Routine latch only.
            fake.set_module_status(_ROUTINE_LATCH)
            fake.set_module_status(0)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        assert called == [], (
            "on_error must NOT be called for KeepAliveTimeout|FailSafe only"
        )

    def test_combined_routine_and_nonroutine_triggers_fault_path(self) -> None:
        """When both routine and non-routine bits are set, the fault path is taken."""
        _, fake, hub = _make_hub()
        received_flags: list[int] = []

        try:
            hub._desired_led_pattern = [(255, 0, 0, 1)] * 16
            hub._on_error = lambda f: received_flags.append(f)
            # Combined: KeepAliveTimeout + ControllerOverTemp.
            fake.set_module_status(_KA_TIMEOUT | _OVER_TEMP)

            hub._heartbeat_post_keepalive()
        finally:
            fake.stop()

        # on_error must be called (non-routine bit present).
        assert len(received_flags) == 1
        # SetModuleLEDPattern must NOT have been sent.
        led_pkts = [
            pkt for pkt in fake.requests
            if pkt.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        ]
        assert len(led_pkts) == 0
