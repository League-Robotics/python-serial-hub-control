"""Framing layer for the RHSP protocol.

Implements packet serialization and streaming deserialization. This module
knows only byte layout; it is unaware of command semantics.

Frame structure (little-endian):
    [0:2]  FRAME magic bytes b"DK"
    [2:4]  length  (uint16): total packet size = 10 + len(payload) + 1
    [4]    dest    (uint8):  destination module address
    [5]    src     (uint8):  source module address
    [6]    msg     (uint8):  message number (session counter; 1..255, never 0)
    [7]    ref     (uint8):  reference number (correlates request to response)
    [8:10] ptype   (uint16): packet type (command/response id)
    [10:N] payload (bytes):  command-specific payload (0..512 bytes)
    [N]    checksum (uint8): sum of all preceding bytes & 0xFF
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator

from rhsp.errors import ChecksumError, ProtocolError

__all__ = [
    "FRAME",
    "HEADER",
    "MAX_PACKET",
    "MAX_PAYLOAD",
    "checksum",
    "build_frame",
    "RawPacket",
    "FrameParser",
]

# Magic bytes at the start of every frame.
FRAME: bytes = b"DK"

# Header struct: frame(2s) + length(H) + dest(B) + src(B) + msg(B) + ref(B) + ptype(H)
# All fields little-endian. Total: 10 bytes.
HEADER: struct.Struct = struct.Struct("<2sHBBBBH")

# Maximum payload size (bytes).
MAX_PAYLOAD: int = 512

# Maximum total packet size: 10-byte header + 512-byte payload + 1-byte checksum.
MAX_PACKET: int = MAX_PAYLOAD + 10 + 1  # 523


def checksum(buf: bytes) -> int:
    """Return the 8-bit checksum: sum of all bytes in *buf*, masked to 8 bits."""
    return sum(buf) & 0xFF


def build_frame(
    dest: int,
    src: int,
    msg: int,
    ref: int,
    ptype: int,
    payload: bytes,
) -> bytes:
    """Assemble a complete RHSP frame and return it as bytes.

    Args:
        dest:    Destination module address (0..255).
        src:     Source module address (0..255).
        msg:     Message number (1..255; the session must never send 0).
        ref:     Reference number for request/response correlation (0..255).
        ptype:   Packet type identifier (0..65535).
        payload: Command-specific payload (0..512 bytes).

    Returns:
        Complete frame bytes including header, payload, and checksum.

    Raises:
        ValueError: If payload exceeds MAX_PAYLOAD bytes.
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(
            f"Payload length {len(payload)} exceeds MAX_PAYLOAD {MAX_PAYLOAD}"
        )
    length = 10 + len(payload) + 1
    header = HEADER.pack(FRAME, length, dest, src, msg, ref, ptype)
    buf = header + payload
    cs = checksum(buf)
    return buf + bytes([cs])


@dataclass(frozen=True)
class RawPacket:
    """A parsed RHSP packet with raw field values (no semantic interpretation).

    Attributes:
        dest:        Destination module address.
        src:         Source module address.
        msg_num:     Message number from the frame header.
        ref_num:     Reference number from the frame header.
        packet_type: Packet type identifier.
        payload:     Raw payload bytes (0..512 bytes).
    """

    dest: int
    src: int
    msg_num: int
    ref_num: int
    packet_type: int
    payload: bytes


class FrameParser:
    """Streaming parser that yields :class:`RawPacket` objects from raw bytes.

    Maintains an internal buffer across :meth:`feed` calls, so fragmented
    input is handled correctly. Resyncs on the ``DK`` magic bytes after
    noise or a corrupt packet.

    Usage::

        parser = FrameParser()
        for chunk in stream:
            for packet in parser.feed(chunk):
                handle(packet)
    """

    def __init__(self) -> None:
        self._buf: bytearray = bytearray()

    def feed(self, data: bytes) -> Iterator[RawPacket]:
        """Append *data* to the internal buffer and yield any complete packets.

        Args:
            data: Raw bytes received from the transport.

        Yields:
            :class:`RawPacket` for each complete, valid frame found.

        Raises:
            ChecksumError:  If a complete frame has an incorrect checksum.
                            The parser resyncs and continues after the corrupt
                            packet.
            ProtocolError:  If a complete frame has an oversized payload
                            (> 512 bytes).
        """
        self._buf.extend(data)

        while True:
            # Step 1: Find the DK magic bytes; discard anything before them.
            idx = self._buf.find(FRAME)
            if idx == -1:
                # No magic bytes in the buffer at all; keep the last byte in
                # case it is the first byte of an upcoming DK pair.
                self._buf = self._buf[-1:] if self._buf else bytearray()
                return
            if idx > 0:
                # Discard noise before the magic bytes.
                del self._buf[:idx]

            # Step 2: Need at least 4 bytes to read the length field.
            if len(self._buf) < 4:
                return

            # Step 3: Parse the length field (bytes 2..3, little-endian uint16).
            (length,) = struct.unpack_from("<H", self._buf, 2)

            # Sanity-check: length must be at least 11 (10-byte header + checksum)
            # and at most MAX_PACKET.
            if length < 11 or length > MAX_PACKET:
                # Malformed length — skip the DK and resync.
                del self._buf[:2]
                continue

            # Step 4: Wait for the full packet.
            if len(self._buf) < length:
                return

            # Step 5: Extract the packet bytes.
            packet_bytes = bytes(self._buf[:length])

            # Step 6: Validate checksum.
            expected_cs = checksum(packet_bytes[:-1])
            actual_cs = packet_bytes[-1]
            if actual_cs != expected_cs:
                # Advance past this corrupt frame and resync.
                del self._buf[:length]
                raise ChecksumError(
                    f"Checksum mismatch: expected 0x{expected_cs:02X}, "
                    f"got 0x{actual_cs:02X}"
                )

            # Step 7: Validate payload size.
            payload_len = length - 11  # total - header(10) - checksum(1)
            if payload_len > MAX_PAYLOAD:
                del self._buf[:length]
                raise ProtocolError(
                    f"Payload length {payload_len} exceeds MAX_PAYLOAD {MAX_PAYLOAD}"
                )

            # Step 8: Unpack header and yield the packet.
            (
                _frame,
                _length,
                dest,
                src,
                msg_num,
                ref_num,
                packet_type,
            ) = HEADER.unpack_from(packet_bytes)
            payload = packet_bytes[10:-1]

            del self._buf[:length]
            yield RawPacket(
                dest=dest,
                src=src,
                msg_num=msg_num,
                ref_num=ref_num,
                packet_type=packet_type,
                payload=payload,
            )
