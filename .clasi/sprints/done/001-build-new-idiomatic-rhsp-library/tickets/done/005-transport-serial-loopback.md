---
id: 001-005
title: "Transport \u2014 SerialTransport and LoopbackTransport"
status: done
use-cases:
- SUC-004
depends-on:
- 001-001
---

# Ticket 005: Transport — SerialTransport and LoopbackTransport

## Description

Implement `src/rhsp/transport.py`: the `Transport` structural Protocol, the
`SerialTransport` (pyserial-backed, blocking I/O), and `LoopbackTransport`
(in-memory, for tests). These are the only modules that touch I/O.

`LoopbackTransport` is critical for the test suite: it allows `FakeHub` (Ticket
006) and all session/device/sensor tests to run without a physical hub.

## Acceptance Criteria

- [x] `Transport` is a `typing.Protocol` with methods: `write(data: bytes) -> None`,
  `read(n: int) -> bytes`, `reset_input() -> None`, `close() -> None`
- [x] `SerialTransport.__init__(port: str, baud: int = 460800, timeout: float = 1.0, inter_byte_timeout: float = 0.01)`:
  - Opens a pyserial `Serial` object in 8N1, no hardware flow control
  - Uses blocking `read` with `timeout` and `inter_byte_timeout` (kills busy-wait)
  - `read(n)` returns up to `n` bytes (may return fewer on timeout)
  - `reset_input()` calls `serial.reset_input_buffer()`
  - `close()` closes the port
- [x] `LoopbackTransport`:
  - Backed by two in-memory byte buffers: one for "host writes → hub reads", one for "hub writes → host reads"
  - `write(data)` appends to the write buffer
  - `read(n)` pops up to `n` bytes from the read buffer; blocks (or returns empty) if none available
  - `reset_input()` clears the read buffer
  - `close()` is a no-op
  - Exposes `inject(data: bytes)` to add bytes to the read buffer (FakeHub uses this)
  - Exposes `drain_writes() -> bytes` to read what was written by the session (FakeHub uses this)
- [x] `LoopbackTransport` echo test: `write(b"hello")` then `inject(b"hello")` then `read(5)` returns `b"hello"`
- [x] `SerialTransport` satisfies the `Transport` Protocol (mypy/pyright structural check passes)
- [x] `LoopbackTransport` satisfies the `Transport` Protocol

## Implementation Plan

### Approach

`Transport` is a runtime-checkable `Protocol` (`@runtime_checkable`). `SerialTransport`
wraps pyserial; uses `timeout` on reads to avoid the vendor's busy-wait `while inWaiting()==0`.
`inter_byte_timeout=0.01` cuts the packet reassembly latency without spinning.

`LoopbackTransport` uses two `bytearray`s or a simple queue. `inject()` and `drain_writes()`
are test-only helpers; they're not part of the `Transport` Protocol and should not be
type-annotated as such.

### Files to Create

- `src/rhsp/transport.py` — Transport Protocol, SerialTransport, LoopbackTransport
- `tests/test_transport.py` — LoopbackTransport unit tests

### Files to Modify

None.

### Testing Plan

`tests/test_transport.py`:
- `LoopbackTransport`: write + inject + read round-trip.
- `reset_input()` clears the read buffer.
- `drain_writes()` returns what was written.
- Assert `LoopbackTransport` satisfies `Transport` Protocol via `isinstance` check.

### Documentation Updates

None required for this ticket.
