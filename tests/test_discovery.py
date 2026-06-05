"""Tests for rhsp.discovery — enumerate_hubs, connect, discover.

All tests are hardware-free: they use FakeHub over LoopbackTransport.

Coverage:
- connect() with a single-reply FakeHub → Hub with correct address and deka_base
- connect() with a two-reply FakeHub → parent Hub + one child
- connect(None) raises RhspError when enumerate_hubs() returns []
- discover() does not use time.sleep() with argument >= 0.1
"""

from __future__ import annotations

import threading
import time
from typing import Any
from unittest.mock import patch

import pytest

from rhsp.codec import encode_payload
from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
from rhsp.errors import RhspError
from rhsp.framing import build_frame, FrameParser
from rhsp.hub import Hub
from rhsp.session import Session
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

_DISCOVERY_TYPE: int = 0x7F0F
_DISCOVERY_RSP_TYPE: int = 0xFF0F
_QUERY_INTERFACE_TYPE: int = 0x7F07
_QUERY_INTERFACE_RSP_TYPE: int = 0xFF07
_DEKA_BASE: int = 0x1000


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_loopback() -> tuple[LoopbackTransport, Session]:
    transport = LoopbackTransport()
    session = Session(transport, retries=1, timeout=0.5)
    return transport, session


def _build_discovery_rsp(src_addr: int, ref_num: int, parent: int) -> bytes:
    """Build a well-formed Discovery_RSP frame from *src_addr*."""
    rsp_desc = RESPONSES_BY_ID[_DISCOVERY_RSP_TYPE]
    payload = encode_payload(rsp_desc.fields, {"parent": parent})
    return build_frame(
        dest=0,
        src=src_addr,
        msg=0,
        ref=ref_num,
        ptype=_DISCOVERY_RSP_TYPE,
        payload=payload,
    )


def _build_query_interface_rsp(req_src: int, ref_num: int) -> bytes:
    """Build a QueryInterface_RSP frame with DEKA base."""
    rsp_desc = RESPONSES_BY_ID[_QUERY_INTERFACE_RSP_TYPE]
    payload = encode_payload(
        rsp_desc.fields,
        {"packetID": _DEKA_BASE, "numValues": 49},
    )
    return build_frame(
        dest=req_src,
        src=2,
        msg=0,
        ref=ref_num,
        ptype=_QUERY_INTERFACE_RSP_TYPE,
        payload=payload,
    )


# ---------------------------------------------------------------------------
# A FakeHub extension that injects two Discovery_RSP replies for multi-hub tests
# ---------------------------------------------------------------------------


class TwoHubFakeHub(FakeHub):
    """FakeHub that sends two Discovery_RSP packets (src=2 parent, src=3 child)."""

    def _reply_discovery(self, req: Any) -> None:
        """Inject two Discovery_RSP frames back-to-back."""
        # Parent at address 2.
        self._transport.inject(_build_discovery_rsp(src_addr=2, ref_num=req.msg_num, parent=1))
        # Child at address 3.
        self._transport.inject(_build_discovery_rsp(src_addr=3, ref_num=req.msg_num, parent=0))


# ---------------------------------------------------------------------------
# Test: connect() with a single-reply FakeHub
# ---------------------------------------------------------------------------


class TestConnectSingleHub:
    """connect() with a FakeHub that sends one Discovery_RSP → one Hub."""

    def _make_connected_hub(self) -> Hub:
        """Build a Hub via connect() using a loopback transport + FakeHub thread."""
        from rhsp.discovery import connect

        transport = LoopbackTransport()
        hub_sim = FakeHub(transport, src_addr=2)
        hub_sim.run_in_thread()

        # Monkey-patch SerialTransport so connect() uses our LoopbackTransport.
        import rhsp.discovery as discovery_mod

        original_st = discovery_mod.SerialTransport

        class _FakeSerial:
            def __new__(cls, port: str, **kwargs: Any) -> Any:
                return transport

        try:
            discovery_mod.SerialTransport = _FakeSerial  # type: ignore[assignment]
            hub = connect(port="/dev/fake")
        finally:
            discovery_mod.SerialTransport = original_st  # type: ignore[assignment]
            hub_sim.stop()

        return hub

    def test_returns_hub_instance(self) -> None:
        hub = self._make_connected_hub()
        assert isinstance(hub, Hub)

    def test_hub_address_is_parent(self) -> None:
        """The returned hub's address should be the FakeHub's source address (2)."""
        hub = self._make_connected_hub()
        assert hub.address == 2

    def test_deka_base_resolved(self) -> None:
        """session.deka_base must equal 0x1000 after QueryInterface."""
        hub = self._make_connected_hub()
        assert hub.deka_base == 0x1000

    def test_session_deka_base_matches(self) -> None:
        hub = self._make_connected_hub()
        assert hub.session.deka_base == 0x1000


# ---------------------------------------------------------------------------
# Test: connect() with a two-reply FakeHub
# ---------------------------------------------------------------------------


class TestConnectTwoHubs:
    """connect() with TwoHubFakeHub → parent Hub with one child."""

    def _make_two_hub_result(self) -> Hub:
        from rhsp.discovery import connect

        transport = LoopbackTransport()
        hub_sim = TwoHubFakeHub(transport, src_addr=2)
        hub_sim.run_in_thread()

        import rhsp.discovery as discovery_mod

        original_st = discovery_mod.SerialTransport

        class _FakeSerial:
            def __new__(cls, port: str, **kwargs: Any) -> Any:
                return transport

        try:
            discovery_mod.SerialTransport = _FakeSerial  # type: ignore[assignment]
            hub = connect(port="/dev/fake")
        finally:
            discovery_mod.SerialTransport = original_st  # type: ignore[assignment]
            hub_sim.stop()

        return hub

    def test_parent_hub_returned(self) -> None:
        hub = self._make_two_hub_result()
        assert hub.address == 2

    def test_child_hub_discoverable(self) -> None:
        """Second Discovery_RSP should produce one child Hub at address 3."""
        hub = self._make_two_hub_result()
        assert len(hub.children) == 1
        assert hub.children[0].address == 3

    def test_deka_base_set(self) -> None:
        hub = self._make_two_hub_result()
        assert hub.deka_base == 0x1000


# ---------------------------------------------------------------------------
# Test: connect(None) raises RhspError when no hub found
# ---------------------------------------------------------------------------


class TestConnectNoHub:
    def test_raises_when_no_ports(self) -> None:
        """connect(None) must raise RhspError('No hub found') when enumerate_hubs() is empty."""
        from rhsp.discovery import connect
        import rhsp.discovery as discovery_mod

        with patch.object(discovery_mod, "enumerate_hubs", return_value=[]):
            with pytest.raises(RhspError, match="No hub found"):
                connect(None)


# ---------------------------------------------------------------------------
# Test: discover() does not use time.sleep() with argument >= 0.1
# ---------------------------------------------------------------------------


class TestDiscoverNoLongSleep:
    """discover() must not call time.sleep() with argument >= 0.1 s."""

    def test_no_long_sleep_in_discover(self) -> None:
        """Patch time.sleep and verify no call uses argument >= 0.1."""
        transport, session = _make_loopback()

        # Inject a Discovery_RSP immediately so discover() exits quickly.
        rsp_desc = RESPONSES_BY_ID[_DISCOVERY_RSP_TYPE]
        payload = encode_payload(rsp_desc.fields, {"parent": 1})
        frame = build_frame(dest=0, src=2, msg=0, ref=1, ptype=_DISCOVERY_RSP_TYPE, payload=payload)
        transport.inject(frame)

        sleep_args: list[float] = []
        original_sleep = time.sleep

        def _tracking_sleep(secs: float) -> None:
            sleep_args.append(secs)
            # Actually sleep the (short) amount so timing-dependent loops work.
            original_sleep(secs)

        with patch("time.sleep", side_effect=_tracking_sleep):
            from rhsp.discovery import discover
            discover(session, dest=0xFF)

        long_sleeps = [s for s in sleep_args if s >= 0.1]
        assert long_sleeps == [], f"discover() called time.sleep({long_sleeps}) — must not use long sleeps"


# ---------------------------------------------------------------------------
# Test: enumerate_hubs — unit tests via mocking
# ---------------------------------------------------------------------------


class TestEnumerateHubs:
    def test_filters_by_d_prefix(self) -> None:
        """enumerate_hubs() should return only ports with serial number starting with 'D'."""
        from rhsp.discovery import enumerate_hubs

        class _MockPort:
            def __init__(self, device: str, serial_number: str | None) -> None:
                self.device = device
                self.serial_number = serial_number

        mock_ports = [
            _MockPort("/dev/ttyACM0", "DQ3M375O"),   # REV Hub — should be included
            _MockPort("/dev/ttyACM1", "ABC123"),      # Not REV — excluded
            _MockPort("/dev/ttyACM2", None),           # No serial number — excluded
            _MockPort("/dev/ttyACM3", "D00001"),      # REV Hub — included
        ]

        with patch("serial.tools.list_ports.comports", return_value=mock_ports):
            result = enumerate_hubs()

        assert result == ["/dev/ttyACM0", "/dev/ttyACM3"]

    def test_returns_empty_list_when_none_match(self) -> None:
        from rhsp.discovery import enumerate_hubs

        class _MockPort:
            def __init__(self, device: str, serial_number: str | None) -> None:
                self.device = device
                self.serial_number = serial_number

        mock_ports = [
            _MockPort("/dev/ttyACM0", "ABC123"),
            _MockPort("/dev/ttyACM1", None),
        ]

        with patch("serial.tools.list_ports.comports", return_value=mock_ports):
            result = enumerate_hubs()

        assert result == []

    def test_returns_empty_list_when_no_ports(self) -> None:
        from rhsp.discovery import enumerate_hubs

        with patch("serial.tools.list_ports.comports", return_value=[]):
            result = enumerate_hubs()

        assert result == []
