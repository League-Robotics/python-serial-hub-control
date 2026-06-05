---
id: '001-003'
title: Framing — build_frame, FrameParser, golden vectors
status: open
use-cases:
  - SUC-002
depends-on:
  - '001-001'
---

# Ticket 003: Framing — build_frame, FrameParser, golden vectors

## Description

Implement `src/rhsp/framing.py`: the constants, `checksum` function,
`build_frame` function, `RawPacket` dataclass, and `FrameParser` streaming
parser. Verify with byte-exact assertions against the §2.2 golden vectors
(msg_num=1 versions).

This module is the boundary between the session and the transport. It knows
only byte layout; it is unaware of command semantics.

## Acceptance Criteria

- [ ] `FRAME: bytes = b"DK"`
- [ ] `HEADER = struct.Struct("<2sHBBBBH")` (10 bytes: frame(2) + length(2) + dest(1) + src(1) + msg(1) + ref(1) + ptype(2))
- [ ] `MAX_PACKET: int = 523` (512 payload + 10 header + 1 checksum)
- [ ] `checksum(buf: bytes) -> int` returns `sum(buf) & 0xFF`
- [ ] `build_frame(dest, src, msg, ref, ptype, payload) -> bytes`:
  - length field = `10 + len(payload) + 1`
  - appends checksum of the entire packet excluding the checksum byte
- [ ] `RawPacket` is a frozen dataclass: `dest: int`, `src: int`, `msg_num: int`, `ref_num: int`, `packet_type: int`, `payload: bytes`
- [ ] `FrameParser.feed(data: bytes) -> Iterator[RawPacket]`:
  - Resyncs on `DK` magic bytes after noise or corruption
  - Bounds payload to 512 bytes
  - Raises `ChecksumError` on checksum mismatch (then resyncs)
  - Handles fragmented input across multiple `feed()` calls
- [ ] Golden vector assertions pass:
  - `KeepAlive` dest=1 msg=1 → `44 4B 0B 00 01 00 01 00 04 7F 1F`
  - `Discovery` dest=255 msg=1 → `44 4B 0B 00 FF 00 01 00 0F 7F 28`
  - `SetServoPulseWidth` ch=0 pw=1500 dest=1 msg=1 → `44 4B 0E 00 01 00 01 00 21 10 00 DC 05 B1`
  - `SetMotorConstantPower` ch=0 pwr=16000 dest=1 msg=1 → `44 4B 0E 00 01 00 01 00 0F 10 00 80 3E 7C`
- [ ] `FrameParser` round-trips: parse each golden vector → `RawPacket` with correct fields
- [ ] `FrameParser` raises `ChecksumError` when the last byte is flipped by 1

## Implementation Plan

### Approach

The HEADER struct packs exactly as `<2sHBBBBH`. Build the header with `HEADER.pack(FRAME, length, dest, src, msg, ref, ptype)`, append payload, compute checksum of the complete packet so far, append checksum byte.

`FrameParser` maintains an internal buffer. Each `feed()` call appends to the buffer, then loops:
1. Find `b"DK"` in the buffer. Discard any bytes before it.
2. If fewer than 4 bytes available (can't read length field), wait.
3. Parse length. If fewer than `length` bytes available, wait.
4. Extract the packet bytes. Verify checksum of bytes 0..length-2 against byte length-1.
5. If checksum fails: raise `ChecksumError`, advance buffer past the corrupt packet, resync.
6. If payload > 512 bytes: raise `ProtocolError`.
7. Unpack header; yield `RawPacket`.

### Files to Create

- `src/rhsp/framing.py` — all framing logic
- `tests/test_framing.py` — golden vector tests, round-trip tests, ChecksumError test

### Files to Modify

None.

### Testing Plan

`tests/test_framing.py`:
- Assert each golden vector byte-for-byte using `bytes.fromhex(...)`.
- Assert `FrameParser(feed(golden_bytes)).next()` produces `RawPacket` with expected field values.
- Assert partial input (split golden bytes into two halves) yields one packet across two `feed()` calls.
- Assert a packet with the last byte XOR'd by 1 raises `ChecksumError`.
- Assert `MAX_PACKET == 523`.
- Assert `checksum(b"\x44\x4B\x0B\x00\x01\x00\x01\x00\x04\x7F") == 0x1F`.

### Documentation Updates

None required for this ticket.
