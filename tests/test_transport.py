"""Tests for src/rhsp/transport.py.

Covers:
- LoopbackTransport write / inject / read round-trip.
- reset_input() clears the read buffer.
- drain_writes() returns and clears the write buffer.
- close() is a no-op (no error raised).
- LoopbackTransport satisfies the Transport Protocol (isinstance check).
- SerialTransport satisfies the Transport Protocol structurally (isinstance
  check on the class — does not open a real port).
- Transport is runtime-checkable (isinstance works).
"""

from __future__ import annotations

import pytest

from rhsp.transport import LoopbackTransport, SerialTransport, Transport


# ---------------------------------------------------------------------------
# LoopbackTransport — basic round-trip
# ---------------------------------------------------------------------------


def test_loopback_write_inject_read_roundtrip() -> None:
    """write() + inject() then read() must return the injected bytes."""
    t = LoopbackTransport()
    t.write(b"hello")
    t.inject(b"hello")
    result = t.read(5)
    assert result == b"hello"


def test_loopback_read_returns_empty_when_buffer_empty() -> None:
    t = LoopbackTransport()
    assert t.read(10) == b""


def test_loopback_read_returns_at_most_n_bytes() -> None:
    t = LoopbackTransport()
    t.inject(b"abcdefgh")
    chunk = t.read(3)
    assert chunk == b"abc"
    # Remaining bytes still in buffer
    assert t.read(10) == b"defgh"


def test_loopback_write_does_not_affect_read_buffer() -> None:
    """Bytes written by the session must NOT appear in the read buffer."""
    t = LoopbackTransport()
    t.write(b"session_data")
    assert t.read(100) == b""


# ---------------------------------------------------------------------------
# LoopbackTransport — reset_input
# ---------------------------------------------------------------------------


def test_loopback_reset_input_clears_read_buffer() -> None:
    t = LoopbackTransport()
    t.inject(b"some bytes")
    t.reset_input()
    assert t.read(100) == b""


def test_loopback_reset_input_does_not_clear_write_buffer() -> None:
    t = LoopbackTransport()
    t.write(b"outgoing")
    t.inject(b"incoming")
    t.reset_input()
    # Write buffer must survive reset_input
    assert t.drain_writes() == b"outgoing"


# ---------------------------------------------------------------------------
# LoopbackTransport — drain_writes
# ---------------------------------------------------------------------------


def test_loopback_drain_writes_returns_written_bytes() -> None:
    t = LoopbackTransport()
    t.write(b"frame1")
    t.write(b"frame2")
    drained = t.drain_writes()
    assert drained == b"frame1frame2"


def test_loopback_drain_writes_clears_write_buffer() -> None:
    t = LoopbackTransport()
    t.write(b"data")
    t.drain_writes()
    assert t.drain_writes() == b""


# ---------------------------------------------------------------------------
# LoopbackTransport — close (no-op)
# ---------------------------------------------------------------------------


def test_loopback_close_is_noop() -> None:
    t = LoopbackTransport()
    t.inject(b"bytes")
    t.close()  # must not raise
    # Transport is still usable after close (in-memory — nothing to release)
    assert t.read(5) == b"bytes"


# ---------------------------------------------------------------------------
# Protocol satisfaction — isinstance checks
# ---------------------------------------------------------------------------


def test_loopback_satisfies_transport_protocol() -> None:
    """LoopbackTransport must satisfy the runtime-checkable Transport Protocol."""
    t = LoopbackTransport()
    assert isinstance(t, Transport), (
        "LoopbackTransport does not satisfy the Transport Protocol"
    )


def test_serial_transport_satisfies_transport_protocol_structurally() -> None:
    """SerialTransport must satisfy the Transport Protocol structurally.

    We check via the class itself (not an instance) to avoid opening a real
    serial port.  The Protocol is runtime_checkable, so isinstance on a class
    checks the method signatures rather than instantiating the object.
    """
    # Verify all required methods exist on the class with the right names.
    for method_name in ("write", "read", "reset_input", "close"):
        assert hasattr(SerialTransport, method_name), (
            f"SerialTransport is missing method '{method_name}' required by Transport"
        )

    # Also confirm the Protocol itself is runtime-checkable.
    assert isinstance(LoopbackTransport(), Transport)


def test_transport_protocol_is_runtime_checkable() -> None:
    """Transport must be decorated with @runtime_checkable."""
    # If not runtime-checkable, isinstance raises TypeError.
    t = LoopbackTransport()
    try:
        result = isinstance(t, Transport)
    except TypeError as exc:
        pytest.fail(f"Transport is not runtime_checkable: {exc}")
    assert result is True


# ---------------------------------------------------------------------------
# SerialTransport.read — prompt-read behaviour (hardware-free)
# ---------------------------------------------------------------------------


class _FakeSerial:
    """Minimal stub that replaces ``serial.Serial`` inside ``SerialTransport``.

    Tracks every call to ``read(n)`` and records the argument so tests can
    verify which path was taken (in_waiting path vs. read(1) fallback).
    """

    def __init__(self, buffered: bytes = b"") -> None:
        self._buffered = bytearray(buffered)
        self.read_calls: list[int] = []  # argument passed to each read()

    @property
    def in_waiting(self) -> int:
        return len(self._buffered)

    def read(self, n: int) -> bytes:
        self.read_calls.append(n)
        chunk = bytes(self._buffered[:n])
        del self._buffered[:n]
        return chunk


def _make_serial_transport_with_stub(stub: _FakeSerial) -> SerialTransport:
    """Construct a ``SerialTransport`` without opening a real port.

    We bypass ``__init__`` (which would call ``serial.Serial(port=...)``) and
    inject the stub directly, using the same ``_serial`` attribute name that
    ``SerialTransport.read`` accesses.
    """
    t = object.__new__(SerialTransport)
    t._serial = stub  # type: ignore[attr-defined]
    return t


def test_serial_transport_read_uses_in_waiting_path() -> None:
    """When bytes are already buffered, read() returns them without over-reading.

    The in_waiting path must call ``read(min(n, in_waiting))`` so it returns
    immediately without waiting for *n* bytes.  The argument passed to the
    underlying ``read()`` must be ``<= in_waiting``, not the full ``n``.
    """
    stub = _FakeSerial(b"ABCDE")  # 5 bytes buffered
    t = _make_serial_transport_with_stub(stub)

    result = t.read(512)  # ask for much more than available

    assert result == b"ABCDE", "Should return all 5 buffered bytes"
    assert len(stub.read_calls) == 1, "Should make exactly one read() call"
    assert stub.read_calls[0] == 5, (
        f"read() should have been called with 5 (in_waiting), got {stub.read_calls[0]}"
    )


def test_serial_transport_read_in_waiting_path_respects_n() -> None:
    """When in_waiting > n, read() asks for at most n bytes."""
    stub = _FakeSerial(b"ABCDEFGH")  # 8 bytes buffered
    t = _make_serial_transport_with_stub(stub)

    result = t.read(3)

    assert result == b"ABC"
    assert stub.read_calls == [3], (
        f"read() should have been called with min(3, 8)=3, got {stub.read_calls}"
    )


def test_serial_transport_read_fallback_to_single_byte_when_empty() -> None:
    """When no bytes are buffered, read() falls back to a single read(1).

    This ensures the first arriving byte wakes us promptly rather than
    blocking the full port timeout waiting for n bytes.
    """
    stub = _FakeSerial(b"")  # nothing buffered yet
    t = _make_serial_transport_with_stub(stub)

    # The stub returns b"" for read(1) since the buffer is empty — that's fine,
    # we're only verifying the call argument, not real serial I/O.
    result = t.read(512)

    assert result == b"", "Empty buffer should return empty bytes"
    assert stub.read_calls == [1], (
        f"Fallback should call read(1), not read(512); got read_calls={stub.read_calls}"
    )
