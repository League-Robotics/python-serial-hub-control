"""Session — RHSP transaction engine.

``Session`` is the single-threaded serialisation point between the
application and the hub.  Exactly one transaction is outstanding at any
time; there are no background threads.

Usage::

    from rhsp.transport import SerialTransport
    from rhsp.session import Session

    transport = SerialTransport("/dev/tty.usbmodem…")
    session = Session(transport)
    session.transaction("KeepAlive", dest=1)

Protocol state maintained here:
- ``_msg_num`` — a counter 1..255 that increments on every outbound
  frame and wraps from 255 back to 1 (never 0, as required by the spec).
- ``deka_base`` — the DEKA interface base address returned by
  ``QueryInterface``; defaults to 0x1000, overwritten by
  :meth:`query_interface` (added in ticket 008).

Threading note
--------------
``Session`` is **not thread-safe**.  Use one ``Session`` per thread, or
serialise access externally.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID, response_for, runtime_packet_id
from rhsp.codec import decode_payload, encode_payload
from rhsp.errors import ChecksumError, NackError, RhspTimeoutError
from rhsp.enums import NackCode
from rhsp.framing import FrameParser, RawPacket, build_frame

if TYPE_CHECKING:
    from rhsp.transport import Transport

__all__ = ["Session"]

# Absolute packet-type ids for system frames (not DEKA-relative).
_NACK_TYPE: int = 0x7F02
_DISCOVERY_TYPE: int = 0x7F0F
_DISCOVERY_RSP_TYPE: int = 0xFF0F

# Quiet-window duration for discover() (seconds).
_DISCOVER_QUIET_MS: float = 0.050

# Number of bytes to read per attempt from the transport.
_READ_CHUNK: int = 512


class Session:
    """Transaction engine for the RHSP protocol.

    Parameters
    ----------
    transport:
        The byte-level I/O object the session communicates through.
    retries:
        Number of *additional* attempts after the first timeout or
        checksum error.  With the default of 3 the session will try
        up to 4 times before raising :exc:`~rhsp.errors.RhspTimeoutError`.
    timeout:
        Per-attempt read deadline in seconds.  Each call to
        :meth:`transaction` reads from the transport until a valid
        response arrives or ``timeout`` seconds elapse.
    """

    def __init__(
        self,
        transport: "Transport",
        *,
        retries: int = 3,
        timeout: float = 1.0,
    ) -> None:
        self._transport = transport
        self._retries = retries
        self._timeout = timeout
        # msg_num: 1..255, never 0.  Starts at 1.
        self._msg_num: int = 1
        # DEKA base address; overwritten later by QueryInterface.
        self.deka_base: int = 0x1000
        # Shared frame parser (reused across transactions).
        self._parser: FrameParser = FrameParser()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def transaction(
        self,
        command_name: str,
        dest: int,
        **fields: object,
    ) -> dict | None:
        """Execute a single RHSP request/response transaction.

        Looks up *command_name* in the catalogue, encodes ``**fields`` as
        the request payload, frames and sends it, then waits for the
        matching response.

        Parameters
        ----------
        command_name:
            Catalogue command name (e.g. ``"KeepAlive"``).
        dest:
            Destination module address (0..255).
        **fields:
            Keyword arguments matching the command's payload fields.

        Returns
        -------
        dict | None
            Decoded response dict for ``reply_kind == "rsp"`` commands;
            ``None`` for ``reply_kind == "ack"`` commands (ACK carries no
            semantic payload).

        Raises
        ------
        KeyError
            If *command_name* is not in the catalogue.
        NackError
            If the hub replies with a NACK frame.  NACKs are never retried.
        RhspTimeoutError
            If no valid response arrives within ``retries + 1`` attempts.
        """
        cmd = COMMANDS[command_name]
        ptype = runtime_packet_id(cmd, self.deka_base)
        payload = encode_payload(cmd.fields, fields) if fields else (
            encode_payload(cmd.fields, {}) if cmd.fields else b""
        )

        # Capture the msg_num we are about to send, then advance.
        sent_msg_num = self._msg_num
        self._advance_msg_num()

        frame = build_frame(dest, 0, sent_msg_num, 0, ptype, payload)

        # Expected response packet type for this command.
        expected_reply_id: int = cmd.reply_id

        # Discovery is exempt from ref_num correlation (multi-reply, broadcast).
        is_discovery = (ptype == _DISCOVERY_TYPE)

        last_exc: Exception = RhspTimeoutError(
            f"No response for {command_name!r} after {self._retries + 1} attempt(s)"
        )

        for attempt in range(self._retries + 1):
            # Write the frame on the first attempt; retransmit on subsequent.
            self._transport.write(frame)

            try:
                pkt = self._read_response(
                    expected_reply_id=expected_reply_id,
                    sent_msg_num=sent_msg_num,
                    is_discovery=is_discovery,
                )
            except (RhspTimeoutError, ChecksumError) as exc:
                last_exc = exc
                continue  # Retry on timeout or checksum error.

            # NACK: raise immediately, no retry.
            if pkt.packet_type == _NACK_TYPE:
                nack_code = _parse_nack_code(pkt)
                raise NackError(nack_code, NackCode.describe(nack_code))

            # ACK response.
            if cmd.reply_kind == "ack":
                return None

            # RSP response: decode and return.
            rsp_desc = RESPONSES_BY_ID.get(pkt.packet_type)
            if rsp_desc is None or not rsp_desc.fields:
                return {}
            return decode_payload(rsp_desc.fields, pkt.payload)

        raise last_exc

    def discover(self, dest: int = 0xFF) -> list[RawPacket]:
        """Send a Discovery broadcast and collect all replies.

        Sends one Discovery frame to *dest* (typically the broadcast
        address ``0xFF``) then accumulates incoming ``Discovery_RSP``
        packets until no bytes arrive for approximately 50 ms.

        Parameters
        ----------
        dest:
            Destination address for the Discovery frame (default ``0xFF``).

        Returns
        -------
        list[RawPacket]
            All ``Discovery_RSP`` packets received within the quiet window.
            Not subject to ``ref_num`` correlation (the spec permits any
            number of hubs to reply).

        Notes
        -----
        This method is exempt from the single-outstanding-transaction
        constraint: multiple replies are collected from a broadcast.
        """
        cmd = COMMANDS["Discovery"]
        ptype = runtime_packet_id(cmd, self.deka_base)
        payload = b""

        sent_msg_num = self._msg_num
        self._advance_msg_num()

        frame = build_frame(dest, 0, sent_msg_num, 0, ptype, payload)
        self._transport.write(frame)

        # Collect replies until the quiet window expires.
        packets: list[RawPacket] = []
        deadline = time.monotonic() + self._timeout
        quiet_deadline = time.monotonic() + _DISCOVER_QUIET_MS

        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            chunk = self._transport.read(_READ_CHUNK)
            if chunk:
                for pkt in self._parser.feed(chunk):
                    if pkt.packet_type == _DISCOVERY_RSP_TYPE:
                        packets.append(pkt)
                # Reset the quiet window whenever bytes arrive.
                quiet_deadline = time.monotonic() + _DISCOVER_QUIET_MS
            else:
                if time.monotonic() >= quiet_deadline:
                    break
                # Yield the CPU briefly so the OS or a test thread can
                # inject data before we poll again.
                time.sleep(0.001)

        return packets

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _advance_msg_num(self) -> None:
        """Increment ``_msg_num`` with wrap-around; never produces 0."""
        self._msg_num = (self._msg_num % 255) + 1

    def _read_response(
        self,
        *,
        expected_reply_id: int,
        sent_msg_num: int,
        is_discovery: bool,
    ) -> RawPacket:
        """Read from the transport until an acceptable packet is found.

        An acceptable packet is one where:
        - ``packet_type == expected_reply_id``  (or ``== _NACK_TYPE``)
        - ``ref_num == sent_msg_num``           (unless *is_discovery*)

        Packets that do not match are silently discarded; the loop
        continues reading until the timeout expires.

        Raises
        ------
        RhspTimeoutError
            When ``self._timeout`` seconds elapse without a matching packet.
        ChecksumError
            When a corrupt frame is detected (propagated from
            :class:`~rhsp.framing.FrameParser`); the caller retries.
        """
        deadline = time.monotonic() + self._timeout

        while time.monotonic() < deadline:
            chunk = self._transport.read(_READ_CHUNK)
            if not chunk:
                # Yield the CPU briefly so a FakeHub background thread (in tests)
                # or the OS serial buffer can produce data before we poll again.
                time.sleep(0.001)
                continue  # Transport returned nothing; keep polling.

            for pkt in self._parser.feed(chunk):
                # Check if this is a NACK intended for us.
                if pkt.packet_type == _NACK_TYPE:
                    if is_discovery or pkt.ref_num == sent_msg_num:
                        return pkt
                    # NACK for a different msg_num — discard.
                    continue

                # Check if this is the expected response type.
                if pkt.packet_type != expected_reply_id:
                    continue  # Unexpected packet type — discard.

                # Validate ref_num correlation (Discovery is exempt).
                if is_discovery or pkt.ref_num == sent_msg_num:
                    return pkt
                # ref_num mismatch — discard and keep waiting.

        raise RhspTimeoutError(
            f"Timed out waiting for packet type 0x{expected_reply_id:04X}"
        )


def _parse_nack_code(pkt: RawPacket) -> int:
    """Extract the NACK code integer from a raw NACK packet payload.

    The NACK response payload is a single byte ``nackCode``.  We parse
    it directly from the raw bytes to avoid the codec's variable-length
    trailing-field rule (which returns bytes rather than an int).
    """
    if pkt.payload:
        return pkt.payload[0]
    return 0
