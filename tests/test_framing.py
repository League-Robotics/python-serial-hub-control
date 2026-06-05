"""Tests for src/rhsp/framing.py.

Covers:
- Golden vector byte-exact assertions (ticket 001-003 §9.5 vectors).
- FrameParser round-trips for each golden vector.
- Fragmented-input handling (split frame across two feed() calls).
- ChecksumError on a corrupted last byte.
- MAX_PACKET constant.
- checksum() function.
"""

from __future__ import annotations

import struct
from typing import List

import pytest

from rhsp.errors import ChecksumError, ProtocolError
from rhsp.framing import (
    FRAME,
    HEADER,
    MAX_PACKET,
    RawPacket,
    FrameParser,
    build_frame,
    checksum,
)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------


def test_frame_constant() -> None:
    assert FRAME == b"DK"


def test_header_size() -> None:
    assert HEADER.size == 10


def test_max_packet() -> None:
    assert MAX_PACKET == 523


# ---------------------------------------------------------------------------
# checksum()
# ---------------------------------------------------------------------------


def test_checksum_keepalive_header() -> None:
    """checksum of KeepAlive msg=1 header (without checksum byte) == 0x1F."""
    buf = bytes.fromhex("444B0B0001000100047F")
    assert checksum(buf) == 0x1F


def test_checksum_wraps_to_8bits() -> None:
    assert checksum(bytes([0xFF, 0x01])) == 0x00
    assert checksum(bytes([0xFF, 0xFF])) == 0xFE


# ---------------------------------------------------------------------------
# build_frame() — golden vectors
# ---------------------------------------------------------------------------

# Each entry: (name, kwargs, expected_hex)
GOLDEN_VECTORS = [
    (
        "KeepAlive_msg1",
        dict(dest=1, src=0, msg=1, ref=0, ptype=0x7F04, payload=b""),
        "444B0B0001000100047F1F",
    ),
    (
        "KeepAlive_msg0",
        dict(dest=1, src=0, msg=0, ref=0, ptype=0x7F04, payload=b""),
        "444B0B0001000000047F1E",
    ),
    (
        "Discovery_msg1",
        dict(dest=255, src=0, msg=1, ref=0, ptype=0x7F0F, payload=b""),
        "444B0B00FF0001000F7F28",
    ),
    (
        "SetServoPulseWidth_ch0_pw1500_msg1",
        dict(
            dest=1,
            src=0,
            msg=1,
            ref=0,
            ptype=0x1021,
            payload=struct.pack("<BH", 0, 1500),
        ),
        "444B0E0001000100211000DC05B1",
    ),
    (
        "SetMotorConstantPower_ch0_pwr16000_msg1",
        dict(
            dest=1,
            src=0,
            msg=1,
            ref=0,
            ptype=0x100F,
            payload=struct.pack("<Bh", 0, 16000),
        ),
        "444B0E00010001000F1000803E7C",
    ),
]


@pytest.mark.parametrize("name,kwargs,expected_hex", GOLDEN_VECTORS, ids=[v[0] for v in GOLDEN_VECTORS])
def test_build_frame_golden_vector(name: str, kwargs: dict, expected_hex: str) -> None:
    """build_frame() must produce byte-exact output for every golden vector."""
    result = build_frame(**kwargs)
    assert result == bytes.fromhex(expected_hex), (
        f"[{name}] got {result.hex().upper()}, expected {expected_hex}"
    )


# ---------------------------------------------------------------------------
# build_frame() — payload too large
# ---------------------------------------------------------------------------


def test_build_frame_payload_too_large() -> None:
    with pytest.raises(ValueError, match="MAX_PAYLOAD"):
        build_frame(dest=1, src=0, msg=1, ref=0, ptype=0x7F04, payload=b"\x00" * 513)


# ---------------------------------------------------------------------------
# FrameParser — round-trip
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,kwargs,expected_hex", GOLDEN_VECTORS, ids=[v[0] for v in GOLDEN_VECTORS])
def test_frame_parser_roundtrip(name: str, kwargs: dict, expected_hex: str) -> None:
    """FrameParser must parse a built frame back to a RawPacket with correct fields."""
    raw = bytes.fromhex(expected_hex)
    parser = FrameParser()
    packets: List[RawPacket] = list(parser.feed(raw))
    assert len(packets) == 1, f"[{name}] expected 1 packet, got {len(packets)}"
    pkt = packets[0]
    assert pkt.dest == kwargs["dest"]
    assert pkt.src == kwargs["src"]
    assert pkt.msg_num == kwargs["msg"]
    assert pkt.ref_num == kwargs["ref"]
    assert pkt.packet_type == kwargs["ptype"]
    assert pkt.payload == kwargs["payload"]


# ---------------------------------------------------------------------------
# FrameParser — fragmented input
# ---------------------------------------------------------------------------


def test_frame_parser_fragmented_input() -> None:
    """Split a golden frame into two halves; the parser must yield one packet."""
    raw = bytes.fromhex("444B0B0001000100047F1F")
    midpoint = len(raw) // 2
    parser = FrameParser()
    first_half = list(parser.feed(raw[:midpoint]))
    assert first_half == [], "Should not yield a packet before receiving the full frame"
    second_half = list(parser.feed(raw[midpoint:]))
    assert len(second_half) == 1
    pkt = second_half[0]
    assert pkt.dest == 1
    assert pkt.packet_type == 0x7F04


def test_frame_parser_two_frames_back_to_back() -> None:
    """Concatenated golden frames must each parse independently."""
    frame1 = bytes.fromhex("444B0B0001000100047F1F")
    frame2 = bytes.fromhex("444B0B00FF0001000F7F28")
    parser = FrameParser()
    packets = list(parser.feed(frame1 + frame2))
    assert len(packets) == 2
    assert packets[0].dest == 1
    assert packets[1].dest == 255


# ---------------------------------------------------------------------------
# FrameParser — noise / resync
# ---------------------------------------------------------------------------


def test_frame_parser_noise_before_frame() -> None:
    """Bytes before the DK magic are discarded and the frame is still parsed."""
    noise = b"\x00\x01\xFF\xAB"
    raw = bytes.fromhex("444B0B0001000100047F1F")
    parser = FrameParser()
    packets = list(parser.feed(noise + raw))
    assert len(packets) == 1
    assert packets[0].packet_type == 0x7F04


# ---------------------------------------------------------------------------
# FrameParser — ChecksumError
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,kwargs,expected_hex", GOLDEN_VECTORS, ids=[v[0] for v in GOLDEN_VECTORS])
def test_frame_parser_checksum_error(name: str, kwargs: dict, expected_hex: str) -> None:
    """Flipping the last byte (checksum) by 1 must raise ChecksumError."""
    raw = bytearray(bytes.fromhex(expected_hex))
    raw[-1] ^= 0x01  # flip one bit of the checksum byte
    parser = FrameParser()
    with pytest.raises(ChecksumError):
        list(parser.feed(bytes(raw)))


# ---------------------------------------------------------------------------
# FrameParser — oversized length
# ---------------------------------------------------------------------------


def test_frame_parser_oversized_length_resyncs() -> None:
    """A packet whose length field exceeds MAX_PACKET is skipped without crashing."""
    # Craft a frame with length = 600 (> 523), followed by a valid frame.
    bad_header = struct.pack("<2sHBBBBH", b"DK", 600, 1, 0, 1, 0, 0x7F04)
    good_frame = bytes.fromhex("444B0B0001000100047F1F")
    parser = FrameParser()
    # The oversized frame will be rejected; the good frame should parse.
    packets = list(parser.feed(bad_header + good_frame))
    # We expect the good frame to eventually be parsed (after resync).
    assert len(packets) == 1
    assert packets[0].packet_type == 0x7F04
