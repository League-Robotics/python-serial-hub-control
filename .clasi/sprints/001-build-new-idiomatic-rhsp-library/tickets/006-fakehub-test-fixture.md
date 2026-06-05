---
id: 001-006
title: FakeHub test fixture
status: done
use-cases:
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
depends-on:
- 001-004
- 001-005
---

# Ticket 006: FakeHub test fixture

## Description

Implement `tests/fakehub.py`: a `FakeHub` class that sits behind a
`LoopbackTransport` and acts as a minimal hub simulator for the hardware-free
test suite. `FakeHub` answers protocol requests and lets tests assert the exact
bytes sent by the session/device/sensor layers.

`FakeHub` is a test-only artifact — it lives in `tests/`, not `src/rhsp/`.

## Acceptance Criteria

- [x] `FakeHub` is constructed with a `LoopbackTransport` and exposes a `Session`-compatible interface
- [x] `FakeHub` answers `KeepAlive` (0x7F04) with an ACK (0x7F01) response
- [x] `FakeHub` answers `Discovery` (0x7F0F) with a `Discovery_RSP` packet carrying `parent=True` and `src=1`
- [x] `FakeHub` answers `QueryInterface("DEKA")` with `packetID=0x1000, numValues=49`
- [x] `FakeHub` supports configuring a NACK response for a specific command:
  `fake.set_nack(command_name, nack_code)` — causes the next transaction for that command to return NACK
- [x] `FakeHub` records all requests received: `fake.requests` is a list of `RawPacket` objects
- [x] `FakeHub` records all payloads decoded by command name: `fake.payloads[command_name]` returns the last decoded dict
- [x] `FakeHub.run_in_thread()` starts a background thread that processes requests on the `LoopbackTransport`
- [x] `FakeHub` correctly echoes the `msg_num` from the request as `ref_num` in the response
- [x] `tests/test_fakehub.py` asserts: FakeHub answers KeepAlive with ACK; FakeHub records the request; NACK path triggers `NackError` on the session side

## Implementation Plan

### Approach

`FakeHub` runs a processing loop (in a thread for async use, or called explicitly
by tests for synchronous use). The loop:
1. Calls `transport.drain_writes()` to get bytes written by the session.
2. Feeds bytes through a `FrameParser` to extract `RawPacket`s.
3. Looks up the packet type in the command table to decode the payload.
4. Dispatches to a handler method (or default ACK).
5. Builds a response frame and calls `transport.inject(response_bytes)`.

For simple command handlers (KeepAlive → ACK), the handler builds an ACK frame
using `build_frame(dest=pkt.src, src=1, msg=0, ref=pkt.msg_num, ptype=0x7F01, payload=b"")`.

The NACK configuration dict maps command name → nack_code. When set, the handler
returns a NACK frame instead of the normal response.

For the `QueryInterface("DEKA")` handler, inspect the decoded `interfaceName`
field; if it equals `"DEKA"`, respond with `packetID=0x1000, numValues=49` encoded
as 4 bytes LE.

For `Discovery`, FakeHub responds with a `Discovery_RSP` frame containing `parent=1`
(or `0x01`). `src` in the response header is `1` (hub address).

### Files to Create

- `tests/fakehub.py` — `FakeHub` class
- `tests/test_fakehub.py` — basic smoke tests for the fixture itself

### Files to Modify

None.

### Testing Plan

`tests/test_fakehub.py`:
- Construct FakeHub + LoopbackTransport.
- Build a KeepAlive frame and inject it to FakeHub; assert ACK is produced.
- Assert `fake.requests` has one entry.
- Configure NACK for "SetMotorConstantPower"; assert response is NACK frame.

### Documentation Updates

None required for this ticket.
