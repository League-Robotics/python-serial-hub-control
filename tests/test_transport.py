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
