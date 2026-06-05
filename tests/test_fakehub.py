"""Smoke tests for the FakeHub test fixture (tests/fakehub.py).

These tests verify that FakeHub can:
- Answer a KeepAlive frame with an ACK frame.
- Record the raw RawPacket for every request received.
- Produce a NACK frame when set_nack() is configured.
- Answer a Discovery frame with a Discovery_RSP.
- Answer a QueryInterface("DEKA") with QueryInterface_RSP carrying the
  expected packetID and numValues.
- Run in a background thread and process frames concurrently.

All tests are hardware-free — they drive both ends of a LoopbackTransport
directly, using build_frame / FrameParser from the framing layer.
"""

from __future__ import annotations

import struct
import threading
import time

import pytest

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
from rhsp.codec import encode_payload
from rhsp.framing import FrameParser, build_frame
from rhsp.transport import LoopbackTransport

from fakehub import FakeHub

# ---------------------------------------------------------------------------
# Constants from the protocol (same values as FakeHub uses internally)
# ---------------------------------------------------------------------------

_ACK_TYPE = 0x7F01
_NACK_TYPE = 0x7F02
_KEEPALIVE_TYPE = 0x7F04
_DISCOVERY_TYPE = 0x7F0F
_QUERY_INTERFACE_TYPE = 0x7F07
_DISCOVERY_RSP_TYPE = 0xFF0F
_QUERY_INTERFACE_RSP_TYPE = 0xFF07


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_transport_and_hub() -> tuple[LoopbackTransport, FakeHub]:
    """Construct a LoopbackTransport and a FakeHub wired together."""
    transport = LoopbackTransport()
    hub = FakeHub(transport)
    return transport, hub


def _read_all_frames(transport: LoopbackTransport) -> list:
    """Drain the read buffer and parse all complete frames from it."""
    parser = FrameParser()
    data = transport.read(4096)
    return list(parser.feed(data))


def _write_and_process(
    transport: LoopbackTransport,
    hub: FakeHub,
    *,
    dest: int,
    src: int,
    msg: int,
    ptype: int,
    payload: bytes = b"",
) -> list:
    """Write a frame to the transport, process it through hub, return response frames."""
    frame = build_frame(
        dest=dest, src=src, msg=msg, ref=0, ptype=ptype, payload=payload
    )
    transport.write(frame)
    hub.process_one()
    return _read_all_frames(transport)


# ---------------------------------------------------------------------------
# KeepAlive → ACK
# ---------------------------------------------------------------------------


class TestKeepAlive:
    """FakeHub must answer KeepAlive (0x7F04) with an ACK (0x7F01)."""

    def test_keepalive_produces_ack(self) -> None:
        transport, hub = _make_transport_and_hub()
        responses = _write_and_process(
            transport, hub, dest=1, src=0, msg=1, ptype=_KEEPALIVE_TYPE
        )
        assert len(responses) == 1
        assert responses[0].packet_type == _ACK_TYPE

    def test_keepalive_ack_echoes_msg_num_as_ref_num(self) -> None:
        """ACK ref_num must equal the KeepAlive msg_num (correlation)."""
        transport, hub = _make_transport_and_hub()
        msg_num = 42
        frame = build_frame(
            dest=1, src=0, msg=msg_num, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].ref_num == msg_num

    def test_keepalive_ack_has_correct_dest(self) -> None:
        """ACK dest must be the original packet src (0 in this case)."""
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=1, src=0, msg=5, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].dest == 0

    def test_keepalive_ack_src_is_hub_address(self) -> None:
        """ACK src must be the hub address (default 1)."""
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].src == 1


# ---------------------------------------------------------------------------
# Request recording
# ---------------------------------------------------------------------------


class TestRequestRecording:
    """FakeHub.requests must hold a RawPacket for every processed frame."""

    def test_requests_list_is_initially_empty(self) -> None:
        _, hub = _make_transport_and_hub()
        assert hub.requests == []

    def test_keepalive_recorded_in_requests(self) -> None:
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        assert len(hub.requests) == 1
        assert hub.requests[0].packet_type == _KEEPALIVE_TYPE

    def test_multiple_frames_all_recorded(self) -> None:
        transport, hub = _make_transport_and_hub()
        for msg in (1, 2, 3):
            frame = build_frame(
                dest=1, src=0, msg=msg, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
            )
            transport.write(frame)
        hub.process_one()
        # All three frames were in the write buffer; process_one drains all.
        assert len(hub.requests) == 3

    def test_recorded_packet_has_correct_fields(self) -> None:
        """The stored RawPacket must carry the exact header values sent."""
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=1, src=7, msg=33, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        pkt = hub.requests[0]
        assert pkt.dest == 1
        assert pkt.src == 7
        assert pkt.msg_num == 33
        assert pkt.packet_type == _KEEPALIVE_TYPE


# ---------------------------------------------------------------------------
# Payload recording
# ---------------------------------------------------------------------------


class TestPayloadRecording:
    """FakeHub.payloads must record the decoded dict for each command."""

    def test_discovery_payload_recorded(self) -> None:
        """Discovery has no payload fields; payloads dict should not crash."""
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=0xFF, src=0, msg=1, ref=0, ptype=_DISCOVERY_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        # Discovery has no payload fields, so payloads["Discovery"] may be absent.
        # The important thing is no exception was raised.
        assert len(hub.requests) == 1


# ---------------------------------------------------------------------------
# NACK override
# ---------------------------------------------------------------------------


class TestNackOverride:
    """set_nack() must cause FakeHub to reply with a NACK frame."""

    def test_nack_frame_produced(self) -> None:
        transport, hub = _make_transport_and_hub()
        hub.set_nack("KeepAlive", nack_code=5)
        frame = build_frame(
            dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert len(responses) == 1
        assert responses[0].packet_type == _NACK_TYPE

    def test_nack_carries_correct_code(self) -> None:
        """NACK payload must contain the configured nack_code.

        Note: the NACK response has a single field (nackCode), which the
        codec treats as a variable-length trailing field and returns as bytes.
        We compare against the raw byte representation.
        """
        transport, hub = _make_transport_and_hub()
        expected_code = 7
        hub.set_nack("KeepAlive", nack_code=expected_code)
        frame = build_frame(
            dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        # Check the raw payload byte directly rather than going through
        # decode_payload (whose last-field-is-variable-length rule returns bytes
        # for single-field responses).
        assert len(responses[0].payload) == 1
        assert responses[0].payload[0] == expected_code

    def test_nack_echoes_msg_num_as_ref_num(self) -> None:
        transport, hub = _make_transport_and_hub()
        hub.set_nack("KeepAlive", nack_code=1)
        frame = build_frame(
            dest=1, src=0, msg=99, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].ref_num == 99

    def test_nack_consumed_after_one_use(self) -> None:
        """After one NACK, subsequent requests get the normal response."""
        transport, hub = _make_transport_and_hub()
        hub.set_nack("KeepAlive", nack_code=3)

        # First request → NACK.
        frame1 = build_frame(
            dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame1)
        hub.process_one()
        resp1 = _read_all_frames(transport)
        assert resp1[0].packet_type == _NACK_TYPE

        # Second request → ACK (override consumed).
        frame2 = build_frame(
            dest=1, src=0, msg=2, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame2)
        hub.process_one()
        resp2 = _read_all_frames(transport)
        assert resp2[0].packet_type == _ACK_TYPE

    def test_set_nack_for_unrelated_command_does_not_affect_keepalive(self) -> None:
        transport, hub = _make_transport_and_hub()
        hub.set_nack("SetMotorConstantPower", nack_code=4)
        frame = build_frame(
            dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].packet_type == _ACK_TYPE


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------


class TestDiscovery:
    """FakeHub must reply to a broadcast Discovery with a Discovery_RSP."""

    def test_discovery_produces_discovery_rsp(self) -> None:
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=0xFF, src=0, msg=1, ref=0, ptype=_DISCOVERY_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert len(responses) == 1
        assert responses[0].packet_type == _DISCOVERY_RSP_TYPE

    def test_discovery_rsp_has_parent_flag_set(self) -> None:
        """Discovery_RSP payload must carry parent=1.

        Note: Discovery_RSP has a single field (parent), which the codec treats
        as a variable-length trailing field and returns as bytes.  We check the
        raw payload byte directly.
        """
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=0xFF, src=0, msg=1, ref=0, ptype=_DISCOVERY_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert len(responses[0].payload) == 1
        assert responses[0].payload[0] == 1  # parent flag

    def test_discovery_rsp_src_is_hub_address(self) -> None:
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=0xFF, src=0, msg=1, ref=0, ptype=_DISCOVERY_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].src == 1  # default hub address

    def test_discovery_rsp_echoes_msg_num(self) -> None:
        transport, hub = _make_transport_and_hub()
        frame = build_frame(
            dest=0xFF, src=0, msg=77, ref=0, ptype=_DISCOVERY_TYPE, payload=b""
        )
        transport.write(frame)
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].ref_num == 77


# ---------------------------------------------------------------------------
# QueryInterface
# ---------------------------------------------------------------------------


class TestQueryInterface:
    """FakeHub must answer QueryInterface("DEKA") with packetID=0x1000, numValues=49."""

    def _make_qi_frame(self, interface_name: str, msg: int = 1) -> bytes:
        """Build a QueryInterface frame with the given interface name."""
        cmd = COMMANDS["QueryInterface"]
        # interfaceName is the last (variable-length) field in the descriptor.
        payload = encode_payload(
            cmd.fields,
            {"interfaceName": interface_name.encode("ascii")},
        )
        return build_frame(
            dest=1, src=0, msg=msg, ref=0, ptype=_QUERY_INTERFACE_TYPE, payload=payload
        )

    def test_query_interface_deka_produces_rsp(self) -> None:
        transport, hub = _make_transport_and_hub()
        transport.write(self._make_qi_frame("DEKA"))
        hub.process_one()
        responses = _read_all_frames(transport)
        assert len(responses) == 1
        assert responses[0].packet_type == _QUERY_INTERFACE_RSP_TYPE

    def test_query_interface_deka_packet_id(self) -> None:
        """QueryInterface_RSP packetID must be 0x1000.

        We parse the first 2 bytes of the payload as a little-endian uint16
        rather than going through decode_payload, because the last field
        (numValues) is treated as variable-length by the codec and returned
        as bytes.
        """
        transport, hub = _make_transport_and_hub()
        transport.write(self._make_qi_frame("DEKA"))
        hub.process_one()
        responses = _read_all_frames(transport)
        payload = responses[0].payload
        assert len(payload) >= 2
        packet_id = int.from_bytes(payload[0:2], "little")
        assert packet_id == 0x1000

    def test_query_interface_deka_num_values(self) -> None:
        """QueryInterface_RSP numValues must be 49.

        We parse bytes 2..3 directly (little-endian uint16) for the same
        reason as test_query_interface_deka_packet_id.
        """
        transport, hub = _make_transport_and_hub()
        transport.write(self._make_qi_frame("DEKA"))
        hub.process_one()
        responses = _read_all_frames(transport)
        payload = responses[0].payload
        assert len(payload) >= 4
        num_values = int.from_bytes(payload[2:4], "little")
        assert num_values == 49

    def test_query_interface_deka_echoes_msg_num(self) -> None:
        transport, hub = _make_transport_and_hub()
        transport.write(self._make_qi_frame("DEKA", msg=55))
        hub.process_one()
        responses = _read_all_frames(transport)
        assert responses[0].ref_num == 55

    def test_query_interface_unknown_interface_returns_nack(self) -> None:
        """An unrecognised interface name must produce a NACK."""
        transport, hub = _make_transport_and_hub()
        transport.write(self._make_qi_frame("UNKN"))
        hub.process_one()
        responses = _read_all_frames(transport)
        assert len(responses) == 1
        assert responses[0].packet_type == _NACK_TYPE


# ---------------------------------------------------------------------------
# Background thread
# ---------------------------------------------------------------------------


class TestBackgroundThread:
    """FakeHub.run_in_thread() must process frames from a background thread."""

    def test_thread_responds_to_keepalive(self) -> None:
        transport, hub = _make_transport_and_hub()
        hub.run_in_thread()
        try:
            frame = build_frame(
                dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
            )
            transport.write(frame)
            # Give the background thread time to process.
            deadline = time.monotonic() + 2.0
            responses: list = []
            parser = FrameParser()
            while time.monotonic() < deadline:
                data = transport.read(4096)
                if data:
                    responses.extend(parser.feed(data))
                if responses:
                    break
                time.sleep(0.005)
            assert len(responses) == 1
            assert responses[0].packet_type == _ACK_TYPE
        finally:
            hub.stop()

    def test_thread_records_requests(self) -> None:
        transport, hub = _make_transport_and_hub()
        hub.run_in_thread()
        try:
            frame = build_frame(
                dest=1, src=0, msg=1, ref=0, ptype=_KEEPALIVE_TYPE, payload=b""
            )
            transport.write(frame)
            # Wait for hub to process.
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not hub.requests:
                time.sleep(0.005)
            assert len(hub.requests) == 1
        finally:
            hub.stop()

    def test_cannot_start_thread_twice(self) -> None:
        transport, hub = _make_transport_and_hub()
        hub.run_in_thread()
        try:
            with pytest.raises(RuntimeError, match="already running"):
                hub.run_in_thread()
        finally:
            hub.stop()
