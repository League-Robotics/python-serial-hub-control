"""FakeHub — a minimal RHSP hub simulator for hardware-free testing.

``FakeHub`` sits behind a ``LoopbackTransport`` and acts as a faithful
protocol peer.  It parses inbound frames using ``FrameParser``, decodes
payloads using the same catalogue/codec the client uses, and injects
correctly-framed responses into the transport's read buffer.

Typical usage in a test::

    from rhsp.transport import LoopbackTransport
    from fakehub import FakeHub

    transport = LoopbackTransport()
    hub = FakeHub(transport)

    # Write a KeepAlive frame to the transport and call process_one():
    transport.write(build_frame(..., ptype=0x7F04, payload=b""))
    hub.process_one()

    # Inspect what was produced:
    assert hub.requests[0].packet_type == 0x7F04

``FakeHub`` can also run in a background thread (for integration tests
that use a real ``Session``)::

    hub.run_in_thread()
    session.keep_alive()   # the thread responds automatically
    hub.stop()
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Any

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
from rhsp.codec import decode_payload, encode_payload
from rhsp.framing import FrameParser, RawPacket, build_frame

__all__ = ["FakeHub"]

# ---------------------------------------------------------------------------
# Packet-type constants (absolute ids, same as protocol.json)
# ---------------------------------------------------------------------------

_ACK_TYPE: int = 0x7F01
_NACK_TYPE: int = 0x7F02
_KEEPALIVE_TYPE: int = 0x7F04
_DISCOVERY_TYPE: int = 0x7F0F
_QUERY_INTERFACE_TYPE: int = 0x7F07
_DISCOVERY_RSP_TYPE: int = 0xFF0F
_QUERY_INTERFACE_RSP_TYPE: int = 0xFF07

# Default hub source address used in all responses.
_HUB_SRC: int = 1

# DEKA base address returned by QueryInterface("DEKA").
_DEKA_BASE: int = 0x1000

# Number of DEKA interface values (numValues in QueryInterface_RSP).
_DEKA_NUM_VALUES: int = 49

# Build a reverse map from packet-type id → command name for decoding.
_PACKET_TYPE_TO_CMD: dict[int, str] = {
    cmd.id: name for name, cmd in COMMANDS.items()
}


class FakeHub:
    """Minimal hub simulator over a :class:`~rhsp.transport.LoopbackTransport`.

    Parameters
    ----------
    transport:
        The ``LoopbackTransport`` both ends share.  The session writes to
        ``transport`` (which FakeHub drains via ``drain_writes``) and FakeHub
        injects responses (which the session reads via ``transport.read``).
    src_addr:
        Hub source address used in response frame headers (default 1).
    """

    def __init__(self, transport: Any, *, src_addr: int = _HUB_SRC) -> None:
        self._transport = transport
        self._src_addr = src_addr
        self._parser = FrameParser()

        # Public attributes for test assertions.
        self.requests: list[RawPacket] = []
        self.payloads: dict[str, dict] = {}

        # NACK overrides: command_name → nack_code.
        self._nack_config: dict[str, int] = {}

        # Background thread state.
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # Public configuration API
    # ------------------------------------------------------------------

    def set_nack(self, command_name: str, nack_code: int) -> None:
        """Configure a NACK reply for the next transaction of *command_name*.

        When FakeHub receives a frame whose packet_type matches
        *command_name*, it replies with a NACK frame carrying *nack_code*
        instead of the normal response.  The override is consumed after one
        use.

        Parameters
        ----------
        command_name:
            The catalogue command name (e.g. ``"SetMotorConstantPower"``).
        nack_code:
            Numeric NACK code to include in the NACK frame payload.
        """
        self._nack_config[command_name] = nack_code

    # ------------------------------------------------------------------
    # Processing
    # ------------------------------------------------------------------

    def process_one(self) -> bool:
        """Drain the write buffer once and handle all complete frames in it.

        Returns ``True`` if at least one frame was processed, ``False``
        if the write buffer was empty (or contained only a partial frame).

        This method is synchronous; call it from the test after writing a
        frame to the transport.
        """
        data = self._transport.drain_writes()
        if not data:
            return False

        processed = False
        for pkt in self._parser.feed(data):
            self._handle(pkt)
            processed = True
        return processed

    def process_all(self, *, pause: float = 0.001, max_rounds: int = 100) -> None:
        """Drain and handle frames in a tight loop until the buffer is empty.

        Useful in tests that write multiple frames before calling the hub.
        """
        for _ in range(max_rounds):
            if not self.process_one():
                break
            time.sleep(pause)

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def run_in_thread(self, *, poll_interval: float = 0.001) -> None:
        """Start a background thread that continuously calls :meth:`process_one`.

        The thread runs until :meth:`stop` is called.  Use this for
        integration tests that exercise a real ``Session`` object without
        manually interleaving calls to ``process_one``.

        Parameters
        ----------
        poll_interval:
            Seconds to sleep between polls when the buffer is empty.
        """
        if self._thread is not None:
            raise RuntimeError("FakeHub thread is already running")
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop,
            args=(poll_interval,),
            daemon=True,
            name="FakeHub",
        )
        self._thread.start()

    def stop(self, *, timeout: float = 2.0) -> None:
        """Signal the background thread to stop and wait for it to exit."""
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=timeout)
        self._thread = None

    def _run_loop(self, poll_interval: float) -> None:
        """Background thread body."""
        while not self._stop_event.is_set():
            if not self.process_one():
                time.sleep(poll_interval)

    # ------------------------------------------------------------------
    # Internal frame handling
    # ------------------------------------------------------------------

    def _handle(self, pkt: RawPacket) -> None:
        """Decode *pkt*, record it, and inject an appropriate response."""
        self.requests.append(pkt)

        # Resolve the command name (may be None for unknown packet types).
        cmd_name = _PACKET_TYPE_TO_CMD.get(pkt.packet_type)

        # Decode the payload when a matching command descriptor exists.
        if cmd_name and cmd_name in COMMANDS:
            cmd = COMMANDS[cmd_name]
            if cmd.fields:
                try:
                    decoded = decode_payload(cmd.fields, pkt.payload)
                    self.payloads[cmd_name] = decoded
                except Exception:
                    pass  # Incomplete payload — still record the raw packet.

        # Check for a configured NACK override.
        if cmd_name and cmd_name in self._nack_config:
            nack_code = self._nack_config.pop(cmd_name)
            self._inject_nack(pkt, nack_code)
            return

        # Dispatch to the per-type handler.
        if pkt.packet_type == _KEEPALIVE_TYPE:
            self._reply_ack(pkt)
        elif pkt.packet_type == _DISCOVERY_TYPE:
            self._reply_discovery(pkt)
        elif pkt.packet_type == _QUERY_INTERFACE_TYPE:
            self._reply_query_interface(pkt)
        else:
            # Default: ACK for any command whose catalogue reply_kind is "ack";
            # for unknown or "response" commands, also send ACK as a safe default.
            self._reply_ack(pkt)

    # ------------------------------------------------------------------
    # Response builders
    # ------------------------------------------------------------------

    def _reply_ack(self, req: RawPacket) -> None:
        """Inject an ACK frame correlated to *req*."""
        # ACK payload: attnReq = 0.
        payload = encode_payload(
            RESPONSES_BY_ID[_ACK_TYPE].fields,
            {"attnReq": 0},
        )
        frame = build_frame(
            dest=req.src,
            src=self._src_addr,
            msg=0,
            ref=req.msg_num,
            ptype=_ACK_TYPE,
            payload=payload,
        )
        self._transport.inject(frame)

    def _inject_nack(self, req: RawPacket, nack_code: int) -> None:
        """Inject a NACK frame with *nack_code* correlated to *req*."""
        payload = encode_payload(
            RESPONSES_BY_ID[_NACK_TYPE].fields,
            {"nackCode": nack_code},
        )
        frame = build_frame(
            dest=req.src,
            src=self._src_addr,
            msg=0,
            ref=req.msg_num,
            ptype=_NACK_TYPE,
            payload=payload,
        )
        self._transport.inject(frame)

    def _reply_discovery(self, req: RawPacket) -> None:
        """Inject a Discovery_RSP frame.

        The ticket specifies: ``parent=True`` (1), ``src=1`` (hub address).
        """
        rsp_desc = RESPONSES_BY_ID[_DISCOVERY_RSP_TYPE]
        payload = encode_payload(rsp_desc.fields, {"parent": 1})
        frame = build_frame(
            dest=req.src if req.src != 0xFF else 0,
            src=self._src_addr,
            msg=0,
            ref=req.msg_num,
            ptype=_DISCOVERY_RSP_TYPE,
            payload=payload,
        )
        self._transport.inject(frame)

    def _reply_query_interface(self, req: RawPacket) -> None:
        """Inject a QueryInterface_RSP frame.

        When the decoded ``interfaceName`` starts with ``b"DEKA"`` (case
        insensitive), reply with ``packetID=0x1000`` and ``numValues=49``.
        For all other interfaces, reply with NACK code 2 (unknown interface).
        """
        # Decode the request payload to extract interfaceName.
        cmd = COMMANDS["QueryInterface"]
        interface_name = b""
        if req.payload:
            try:
                decoded = decode_payload(cmd.fields, req.payload)
                raw = decoded.get("interfaceName", b"")
                # Strip null bytes and whitespace.
                if isinstance(raw, (bytes, bytearray)):
                    interface_name = raw.rstrip(b"\x00").upper()
                else:
                    interface_name = str(raw).encode("ascii").upper()
            except Exception:
                pass

        if interface_name.startswith(b"DEKA"):
            rsp_desc = RESPONSES_BY_ID[_QUERY_INTERFACE_RSP_TYPE]
            payload = encode_payload(
                rsp_desc.fields,
                {"packetID": _DEKA_BASE, "numValues": _DEKA_NUM_VALUES},
            )
            frame = build_frame(
                dest=req.src,
                src=self._src_addr,
                msg=0,
                ref=req.msg_num,
                ptype=_QUERY_INTERFACE_RSP_TYPE,
                payload=payload,
            )
            self._transport.inject(frame)
        else:
            # Unknown interface — NACK with code 2.
            self._inject_nack(req, nack_code=2)
