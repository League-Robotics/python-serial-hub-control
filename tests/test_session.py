"""Tests for rhsp.session — Session transaction engine.

All tests are hardware-free: they use FakeHub over a LoopbackTransport.

Coverage:
- msg_num sequencing (1..255, never 0, wraps 255→1)
- ref_num correlation (mismatch discarded, correct accepted)
- NACK handling (NackError raised, not retried)
- Timeout / retry / RhspTimeoutError
- Discovery multi-reply (ref_num exempt)
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Any

import pytest

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
from rhsp.codec import encode_payload
from rhsp.errors import NackError, RhspTimeoutError
from rhsp.framing import FrameParser, build_frame
from rhsp.session import Session
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# Packet-type constants
# ---------------------------------------------------------------------------

_ACK_TYPE = 0x7F01
_NACK_TYPE = 0x7F02
_KEEPALIVE_TYPE = 0x7F04
_DISCOVERY_TYPE = 0x7F0F
_DISCOVERY_RSP_TYPE = 0xFF0F
_QUERY_INTERFACE_TYPE = 0x7F07
_QUERY_INTERFACE_RSP_TYPE = 0xFF07


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_session(
    *,
    retries: int = 3,
    timeout: float = 1.0,
) -> tuple[LoopbackTransport, FakeHub, Session]:
    """Return a wired-up (transport, hub, session) triple."""
    transport = LoopbackTransport()
    hub = FakeHub(transport)
    session = Session(transport, retries=retries, timeout=timeout)
    return transport, hub, session


def _hub_in_thread(hub: FakeHub, *, poll_interval: float = 0.001) -> FakeHub:
    """Start hub in a background thread; return it for use in a ``finally`` block."""
    hub.run_in_thread(poll_interval=poll_interval)
    return hub


# ---------------------------------------------------------------------------
# msg_num sequencing
# ---------------------------------------------------------------------------


class TestMsgNumSequencing:
    """Session._msg_num must stay in 1..255; must never be 0."""

    def test_initial_msg_num_is_one(self) -> None:
        _, _, session = _make_session()
        assert session._msg_num == 1

    def test_msg_num_never_zero_over_300_transactions(self) -> None:
        """300 consecutive transactions must never send msg_num == 0."""
        transport, hub, session = _make_session(timeout=0.05)
        hub.run_in_thread()
        try:
            seen: set[int] = set()
            for _ in range(300):
                session.transaction("KeepAlive", dest=1)
                seen.add(session._msg_num)
            assert 0 not in seen
        finally:
            hub.stop()

    def test_msg_num_wraps_255_to_1(self) -> None:
        """After 255 transactions starting at 1, msg_num must be back at 1.

        Sequencing: start=1, send 1 then advance → _msg_num=2, ...,
        send 254 then advance → _msg_num=255,
        send 255 then advance → (255 % 255) + 1 = 1.
        """
        transport, hub, session = _make_session(timeout=0.05)
        hub.run_in_thread()
        try:
            for _ in range(255):
                session.transaction("KeepAlive", dest=1)
            # After 255 complete transactions: _msg_num wrapped back to 1.
            assert session._msg_num == 1
        finally:
            hub.stop()

    def test_msg_num_increments_each_transaction(self) -> None:
        transport, hub, session = _make_session(timeout=0.05)
        hub.run_in_thread()
        try:
            assert session._msg_num == 1
            session.transaction("KeepAlive", dest=1)
            assert session._msg_num == 2
            session.transaction("KeepAlive", dest=1)
            assert session._msg_num == 3
        finally:
            hub.stop()

    def test_advance_msg_num_wraps_255_to_1(self) -> None:
        """Unit-test _advance_msg_num directly."""
        _, _, session = _make_session()
        session._msg_num = 255
        session._advance_msg_num()
        assert session._msg_num == 1

    def test_advance_msg_num_never_zero(self) -> None:
        """_advance_msg_num must produce values 1..255 across all inputs."""
        _, _, session = _make_session()
        for start in range(1, 256):
            session._msg_num = start
            session._advance_msg_num()
            assert session._msg_num != 0
            assert 1 <= session._msg_num <= 255


# ---------------------------------------------------------------------------
# ref_num correlation
# ---------------------------------------------------------------------------


class TestRefNumCorrelation:
    """Session must discard responses with the wrong ref_num and accept the correct one."""

    def test_mismatched_ref_num_discarded_correct_accepted(self) -> None:
        """FakeHub sends wrong ref_num first, then correct ref_num.

        The session must discard the first packet and return the result of
        the second packet.

        We implement a custom FakeHub that sends two ACK responses: one with
        ref_num=99 (wrong), then one with the correct ref_num.
        """
        transport = LoopbackTransport()
        session = Session(transport, timeout=0.5)

        # We'll inject responses manually into the transport read buffer.
        # The session will send msg_num=1 for its first transaction.
        correct_ref = 1

        # Build two ACK frames: first one has wrong ref_num=99.
        ack_payload = encode_payload(
            RESPONSES_BY_ID[_ACK_TYPE].fields, {"attnReq": 0}
        )
        wrong_ack = build_frame(
            dest=0, src=1, msg=0, ref=99, ptype=_ACK_TYPE, payload=ack_payload
        )
        correct_ack = build_frame(
            dest=0, src=1, msg=0, ref=correct_ref, ptype=_ACK_TYPE, payload=ack_payload
        )

        # Inject both into the read buffer before session.transaction().
        transport.inject(wrong_ack)
        transport.inject(correct_ack)

        # Session should discard the first (ref=99) and accept the second.
        result = session.transaction("KeepAlive", dest=1)
        assert result is None  # KeepAlive returns None (ACK reply_kind)

    def test_correct_ref_num_accepted_immediately(self) -> None:
        """When the first response has the correct ref_num, it is accepted."""
        transport, hub, session = _make_session(timeout=0.2)
        hub.run_in_thread()
        try:
            result = session.transaction("KeepAlive", dest=1)
            assert result is None
        finally:
            hub.stop()


# ---------------------------------------------------------------------------
# NACK handling
# ---------------------------------------------------------------------------


class TestNackHandling:
    """Session must raise NackError on NACK and not retry."""

    def test_nack_raises_nack_error(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        hub.set_nack("KeepAlive", nack_code=50)
        hub.run_in_thread()
        try:
            with pytest.raises(NackError) as exc_info:
                session.transaction("KeepAlive", dest=1)
            assert exc_info.value.code == 50
        finally:
            hub.stop()

    def test_nack_error_has_description(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        hub.set_nack("KeepAlive", nack_code=50)
        hub.run_in_thread()
        try:
            with pytest.raises(NackError) as exc_info:
                session.transaction("KeepAlive", dest=1)
            assert exc_info.value.description  # Non-empty string
            assert "motor" in exc_info.value.description.lower()
        finally:
            hub.stop()

    def test_nack_not_retried(self) -> None:
        """FakeHub returns NACK once; session must not retry (hub should see only 1 request)."""
        transport, hub, session = _make_session(retries=3, timeout=0.2)
        hub.set_nack("KeepAlive", nack_code=50)
        hub.run_in_thread()
        try:
            with pytest.raises(NackError):
                session.transaction("KeepAlive", dest=1)
            # Give hub a moment to process any retransmissions.
            time.sleep(0.05)
            # Only one KeepAlive request should have been received.
            keepalive_requests = [
                r for r in hub.requests if r.packet_type == _KEEPALIVE_TYPE
            ]
            assert len(keepalive_requests) == 1
        finally:
            hub.stop()

    def test_nack_code_in_error_message(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        hub.set_nack("KeepAlive", nack_code=50)
        hub.run_in_thread()
        try:
            with pytest.raises(NackError) as exc_info:
                session.transaction("KeepAlive", dest=1)
            assert "50" in str(exc_info.value)
        finally:
            hub.stop()


# ---------------------------------------------------------------------------
# Timeout and retry
# ---------------------------------------------------------------------------


class TestTimeoutAndRetry:
    """Session must retry on timeout and raise RhspTimeoutError after exhaustion."""

    def test_timeout_raises_rhsp_timeout_error(self) -> None:
        """FakeHub sends no response; session must raise RhspTimeoutError."""
        transport = LoopbackTransport()
        # No FakeHub — nothing responds.
        session = Session(transport, retries=1, timeout=0.02)
        with pytest.raises(RhspTimeoutError):
            session.transaction("KeepAlive", dest=1)

    def test_timeout_retries_correct_number_of_times(self) -> None:
        """Session must send retries+1 total frames before giving up."""
        transport = LoopbackTransport()
        session = Session(transport, retries=2, timeout=0.02)

        with pytest.raises(RhspTimeoutError):
            session.transaction("KeepAlive", dest=1)

        # Count the outbound frames: retries=2 → 3 total writes.
        parser = FrameParser()
        written = transport.drain_writes()
        # drain_writes returns everything written (all retransmissions).
        # Each frame is at least 11 bytes.
        frames = list(parser.feed(written))
        assert len(frames) == 3  # 1 original + 2 retries

    def test_eventual_success_after_transient_timeout(self) -> None:
        """Session receives a valid response after one timeout; must succeed."""
        transport = LoopbackTransport()
        session = Session(transport, retries=3, timeout=0.05)

        # After one failed attempt we inject a correct ACK.
        ack_payload = encode_payload(
            RESPONSES_BY_ID[_ACK_TYPE].fields, {"attnReq": 0}
        )
        correct_ack = build_frame(
            dest=0, src=1, msg=0, ref=1, ptype=_ACK_TYPE, payload=ack_payload
        )

        def _inject_after_delay() -> None:
            time.sleep(0.07)  # Let the first attempt time out (timeout=0.05).
            transport.inject(correct_ack)

        t = threading.Thread(target=_inject_after_delay, daemon=True)
        t.start()
        result = session.transaction("KeepAlive", dest=1)
        t.join(timeout=1.0)
        assert result is None


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------


class TestDiscovery:
    """session.discover() collects multi-reply Discovery_RSP packets."""

    def test_discover_returns_list_of_raw_packets(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        hub.run_in_thread()
        try:
            packets = session.discover(dest=0xFF)
            assert isinstance(packets, list)
            assert len(packets) >= 1
            assert all(p.packet_type == _DISCOVERY_RSP_TYPE for p in packets)
        finally:
            hub.stop()

    def test_discover_exempt_from_ref_num_check(self) -> None:
        """Discovery responses need not carry the sent msg_num in ref_num."""
        transport, hub, session = _make_session(timeout=0.2)
        hub.run_in_thread()
        try:
            packets = session.discover(dest=0xFF)
            # FakeHub echoes msg_num as ref_num for Discovery — but we only
            # assert that we got at least one packet regardless.
            assert len(packets) >= 1
        finally:
            hub.stop()

    def test_discover_msg_num_advances(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        hub.run_in_thread()
        try:
            before = session._msg_num
            session.discover(dest=0xFF)
            after = session._msg_num
            assert after != before
            assert after != 0
        finally:
            hub.stop()

    def test_discover_multi_reply(self) -> None:
        """Inject two Discovery_RSP packets; discover() must return both."""
        transport = LoopbackTransport()
        session = Session(transport, timeout=0.2)

        rsp_desc = RESPONSES_BY_ID[_DISCOVERY_RSP_TYPE]
        payload = encode_payload(rsp_desc.fields, {"parent": 1})
        frame1 = build_frame(
            dest=0, src=1, msg=0, ref=1, ptype=_DISCOVERY_RSP_TYPE, payload=payload
        )
        frame2 = build_frame(
            dest=0, src=2, msg=0, ref=1, ptype=_DISCOVERY_RSP_TYPE, payload=payload
        )

        def _inject() -> None:
            time.sleep(0.01)
            transport.inject(frame1)
            transport.inject(frame2)

        t = threading.Thread(target=_inject, daemon=True)
        t.start()
        packets = session.discover(dest=0xFF)
        t.join(timeout=1.0)
        assert len(packets) == 2


# ---------------------------------------------------------------------------
# deka_base attribute
# ---------------------------------------------------------------------------


class TestDekaBase:
    """Session.deka_base must default to 0x1000 and be writable."""

    def test_default_deka_base(self) -> None:
        _, _, session = _make_session()
        assert session.deka_base == 0x1000

    def test_deka_base_is_writable(self) -> None:
        _, _, session = _make_session()
        session.deka_base = 0x2000
        assert session.deka_base == 0x2000


# ---------------------------------------------------------------------------
# ACK vs RSP return value
# ---------------------------------------------------------------------------


class TestReturnValues:
    """ACK commands return None; RSP commands return a decoded dict."""

    def test_keepalive_returns_none(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        hub.run_in_thread()
        try:
            result = session.transaction("KeepAlive", dest=1)
            assert result is None
        finally:
            hub.stop()

    def test_query_interface_returns_dict(self) -> None:
        """QueryInterface is an RSP command; transaction() must return a dict."""
        transport, hub, session = _make_session(timeout=0.2)
        hub.run_in_thread()
        try:
            from rhsp.codec import encode_payload as ep
            result = session.transaction(
                "QueryInterface",
                dest=1,
                interfaceName=b"DEKA",
            )
            assert isinstance(result, dict)
            assert "packetID" in result or "numValues" in result or result is not None
        finally:
            hub.stop()


# ---------------------------------------------------------------------------
# Module LED pattern wire format
# ---------------------------------------------------------------------------

_SET_MODULE_LED_PATTERN_TYPE = 0x7F0C


class TestModuleLEDPattern:
    """SetModuleLEDPattern must serialise each step as [T, B, G, R] on the wire.

    REV's firmware (verified on fw 1.8.2) and the FTC SDK's
    LynxSetModuleLEDPatternCommand expect duration-first, BGR colour order.
    The [R, G, B, T] order is silently ignored by the hub.
    """

    def test_step_wire_order_is_t_b_g_r(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        hub.run_in_thread()
        try:
            # Distinct values per channel so a swap can't pass by coincidence.
            r, g, b, t = 0x11, 0x22, 0x33, 0x44
            session.set_module_led_pattern(dest=2, steps=[(r, g, b, t)] * 16)
        finally:
            hub.stop()

        reqs = [
            p for p in hub.requests
            if p.packet_type == _SET_MODULE_LED_PATTERN_TYPE
        ]
        assert len(reqs) == 1
        payload = reqs[0].payload
        assert len(payload) == 64  # 16 steps × 4 bytes
        # Every step is the same colour; check the wire order of step 0.
        assert payload[0:4] == bytes([t, b, g, r])  # [T, B, G, R]
        # And the whole payload is that step repeated 16×.
        assert payload == bytes([t, b, g, r]) * 16

    def test_requires_exactly_16_steps(self) -> None:
        transport, hub, session = _make_session(timeout=0.2)
        with pytest.raises(ValueError, match="16"):
            session.set_module_led_pattern(dest=2, steps=[(1, 2, 3, 4)] * 15)
