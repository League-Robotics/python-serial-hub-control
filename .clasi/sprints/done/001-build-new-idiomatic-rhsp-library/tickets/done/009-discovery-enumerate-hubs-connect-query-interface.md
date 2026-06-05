---
id: 001-009
title: "Discovery \u2014 enumerate_hubs, connect, drained discover, QueryInterface"
status: done
use-cases:
- SUC-005
- UC-001
- UC-002
- UC-003
depends-on:
- 001-008
---

# Ticket 009: Discovery — enumerate_hubs, connect, drained discover, QueryInterface

## Description

Implement `src/rhsp/discovery.py`: the three public functions `enumerate_hubs()`,
`connect()`, and `discover()`, plus the `QueryInterface` integration that sets
`session.deka_base`. This module bridges the transport and session layers to the
Hub object.

`discover()` must use a quiet-window drain to collect multi-reply Discovery
responses — it must not use a hard `time.sleep(2)`.

## Acceptance Criteria

- [x] `enumerate_hubs() -> list[str]`:
  - Scans available serial ports using `serial.tools.list_ports`
  - Returns port names (e.g. `/dev/ttyACM0`) whose USB serial number field starts with `"D"`
  - Returns an empty list if no matching port is found
  - No hub connection is made
- [x] `connect(port: str | None = None) -> Hub`:
  - If `port is None`: calls `enumerate_hubs()` and selects the first result; raises `RhspError("No hub found")` if empty
  - Opens `SerialTransport(port, baud=460800)`
  - Creates `Session(transport)`
  - Calls `discover(dest=0xFF)` to collect all `Discovery_RSP` replies
  - Calls `session.query_interface(parent_address, "DEKA")` to obtain and store `deka_base`
  - Constructs and returns the parent `Hub` object; child hubs are accessible from the parent
- [x] `discover(session: Session, dest: int = 0xFF) -> list[RawPacket]`:
  - Sends `Discovery` (0x7F0F) to `dest`
  - Reads replies until a quiet window (~50 ms of no new bytes)
  - Returns all `Discovery_RSP` packets received
  - No `time.sleep(2)` or equivalent fixed delay
  - Exempt from `ref_num` correlation (multi-reply broadcast)
- [x] FakeHub test: `connect()` using a FakeHub-backed LoopbackTransport:
  - `enumerate_hubs()` is not called (port is passed explicitly)
  - Discovery returns 1 reply → 1 Hub constructed
  - `session.deka_base == 0x1000` after `QueryInterface`
- [x] FakeHub test: FakeHub sends 2 Discovery_RSP replies → `connect()` returns parent Hub; second hub address is discoverable
- [x] `connect(None)` raises `RhspError("No hub found")` when `enumerate_hubs()` returns `[]`
- [x] `discover()` does not use `time.sleep()` with argument ≥ 0.1

## Implementation Plan

### Approach

`enumerate_hubs()` uses `serial.tools.list_ports.comports()` and filters by
`port.serial_number` starting with `"D"`. Returns `[port.device for port in matches]`.

`connect()` is the main entry point. After opening transport and session, it calls
`discover()` which sends a single frame to `0xFF` and reads until quiet. It then
identifies the parent hub (the one with `parent=True` in the Discovery_RSP payload)
and calls `QueryInterface("DEKA")` on its address. Then constructs Hub objects.

`discover()` maintains a loop: read bytes, feed to parser, collect packets, check
if no bytes arrived for `quiet_ms=50`. A `transport.read(512)` with timeout 0.05 s
returns empty bytes when the quiet window elapses.

### Files to Create

- `src/rhsp/discovery.py` — `enumerate_hubs`, `connect`, `discover`
- `tests/test_discovery.py` — FakeHub-backed connect tests

### Files to Modify

None (Hub class is created in ticket 010; `connect()` constructs it but the
module import is forward-compatible once ticket 010 is complete).

### Testing Plan

`tests/test_discovery.py`:
- FakeHub single reply: `connect(port="loopback")` → Hub with correct address; `deka_base == 0x1000`.
- FakeHub 2 replies: assert 2 Hub objects constructed.
- Quiet-window test: assert no `time.sleep` with arg ≥ 0.1 (inspect or mock).
- Empty enumeration: mock `serial.tools.list_ports.comports` to return []; assert `RhspError`.

### Documentation Updates

None required for this ticket.
