---
id: 001-007
title: "Session \u2014 transaction engine, msg_num sequencing, NACK, retry"
status: done
use-cases:
- SUC-004
- UC-016
- UC-017
depends-on:
- 001-004
- 001-006
---

# Ticket 007: Session — transaction engine, msg_num sequencing, NACK, retry

## Description

Implement the core `Session` class in `src/rhsp/session.py`. This ticket covers
only the `transaction()` method and the supporting protocol machinery: msg_num
sequencing, response validation, NACK promotion, retry, and timeout. Typed
convenience methods are added in the next ticket (008).

`Session` is the single-threaded serialization point. One outstanding transaction
at a time. No background threads.

## Acceptance Criteria

- [x] `Session.__init__(transport: Transport, retries: int = 3, timeout: float = 1.0)`:
  - `_msg_num: int = 1` (internal counter)
  - `deka_base: int = 0` (set later by QueryInterface; not hard-coded)
- [x] `_msg_num` increments on each transaction; wraps from 255 back to 1 (never 0)
- [x] `_msg_num` is never 0 at any point (including initialization and wrap)
- [x] `transaction(command_name: str, dest: int, **fields) -> dict | None`:
  - Looks up command in `catalogue.COMMANDS`
  - Calls `encode_payload` to build the request payload
  - Calls `runtime_packet_id(cmd, self.deka_base)` to get the packet type id
  - Calls `build_frame(dest, 0, self._msg_num, 0, ptype, payload)` and writes to transport
  - Increments `_msg_num` (with wrap)
  - Reads response bytes and feeds through `FrameParser`
  - Accepts response only when `packet_type == expected_response_id` AND `ref_num == sent_msg_num`
    (Discovery is exempt from `ref_num` correlation)
  - On NACK (`0x7F02`): raises `NackError(code, NackCode.describe(code))` — does NOT retry
  - On timeout or `ChecksumError`: retries up to `retries` times; raises `RhspTimeoutError` after exhaustion
  - On ACK response (reply_kind == "ack"): returns `None`
  - On RSP response (reply_kind == "rsp"): decodes payload via `decode_payload` and returns dict
  - Mismatched response discarded; session waits for next packet or times out
- [x] `discover(dest: int = 0xFF) -> list[RawPacket]`:
  - Sends Discovery frame to `0xFF`
  - Collects all replies until a quiet window (no bytes for ~50 ms)
  - Returns list of `RawPacket` objects (not subject to ref_num correlation)
  - Exempt from single-outstanding-transaction constraint (multi-reply)

## Acceptance Criteria (test assertions)

- [x] 300 consecutive `transaction()` calls: `_msg_num` never equals 0
- [x] After 254 transactions starting at 1, `_msg_num` wraps to 1 (not 0)
- [x] FakeHub returns mismatched `ref_num=99` first, then correct `ref_num`; session discards first and accepts second
- [x] FakeHub returns NACK code 50; `transaction()` raises `NackError(50, ...)`; not retried
- [x] FakeHub returns no response for all retries; `transaction()` raises `RhspTimeoutError`
- [x] FakeHub answers QueryInterface with `packetID=0x1000`; after calling `session.query_interface("DEKA")`, `session.deka_base == 0x1000` (note: `query_interface` is a typed method added in ticket 008, but the `deka_base` attribute is part of the Session state established here)

## Implementation Plan

### Approach

Session holds `_msg_num = 1`, `_parser = FrameParser()`. `transaction()` is a
straightforward request/response loop:

```python
sent_msg_num = self._msg_num
self._advance_msg_num()
frame = build_frame(dest, 0, sent_msg_num, 0, ptype, payload)
self._transport.write(frame)
for attempt in range(self._retries + 1):
    # read with timeout; feed to parser; check packet type + ref_num
    # handle NACK, mismatch, timeout
```

`_advance_msg_num()`: `self._msg_num = (self._msg_num % 255) + 1` — this maps
255 → 1 and any value 1–254 → value+1. Never produces 0.

For `discover()`, send one frame then loop reading until a quiet window
(no bytes received for `quiet_ms=50`).

### Files to Create

- `src/rhsp/session.py` — `Session` class (transaction() + discover() + _msg_num machinery)
- `tests/test_session.py` — full protocol correctness tests using FakeHub

### Files to Modify

None.

### Testing Plan

`tests/test_session.py`:
- msg_num sequencing test: 300 transactions, assert `_msg_num != 0` each time.
- Wrap test: run 254 transactions; assert final `_msg_num == 1`.
- ref_num mismatch: FakeHub sends wrong ref_num once, then correct; assert first discarded.
- NACK test: FakeHub sends NACK 50; assert `NackError` raised, not retried.
- Timeout test: FakeHub sends nothing; assert `RhspTimeoutError` after 3 retries.
- Multi-reply discovery: FakeHub sends 2 Discovery_RSP packets; assert 2 `RawPacket`s returned.

### Documentation Updates

None required for this ticket.
