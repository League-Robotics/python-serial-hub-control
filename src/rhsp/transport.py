"""I/O transport layer for the RHSP library.

Defines the ``Transport`` structural Protocol and two implementations:

- ``SerialTransport`` — backed by pyserial; blocking read with ``timeout``
  and ``inter_byte_timeout`` (no CPU busy-wait).
- ``LoopbackTransport`` — in-memory; used by the test suite so hardware is
  not required.
"""

from __future__ import annotations

import serial
from typing import Protocol, runtime_checkable


# ---------------------------------------------------------------------------
# Protocol
# ---------------------------------------------------------------------------


@runtime_checkable
class Transport(Protocol):
    """Structural protocol for byte-level I/O with a hub.

    All implementations must be safe to use from a single thread; no
    concurrency guarantees are provided or required.
    """

    def write(self, data: bytes) -> None:
        """Write *data* to the transport (blocking until all bytes sent)."""
        ...

    def read(self, n: int) -> bytes:
        """Read up to *n* bytes.

        Blocks until at least one byte arrives or the implementation's timeout
        expires.  Returns an empty ``bytes`` object on timeout.  May return
        fewer than *n* bytes even if more are available (e.g. inter-byte
        timeout fired).
        """
        ...

    def reset_input(self) -> None:
        """Discard any bytes waiting in the inbound buffer."""
        ...

    def close(self) -> None:
        """Release any underlying resources."""
        ...


# ---------------------------------------------------------------------------
# SerialTransport
# ---------------------------------------------------------------------------


class SerialTransport:
    """pyserial-backed transport for a physical RHSP hub.

    Opens the port immediately on construction.  Reads are blocking; the
    combination of ``timeout`` and ``inter_byte_timeout`` allows pyserial to
    return partial packets as soon as the inter-byte gap fires rather than
    waiting for the full ``n``-byte read — eliminating the busy-wait loop
    present in the vendor implementation.

    Parameters
    ----------
    port:
        OS device path (e.g. ``"/dev/tty.usbmodem…"`` or ``"COM3"``).
    baud:
        Baud rate.  REV hubs use 460 800.
    timeout:
        Overall read timeout in seconds.  ``read()`` returns ``b""`` after
        this long with no data.
    inter_byte_timeout:
        Timeout between consecutive bytes in seconds.  When the gap exceeds
        this value pyserial returns the bytes already received.  Keeps packet
        reassembly latency low without spinning.
    """

    def __init__(
        self,
        port: str,
        *,
        baud: int = 460_800,
        timeout: float = 1.0,
        inter_byte_timeout: float = 0.01,
    ) -> None:
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            inter_byte_timeout=inter_byte_timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

    # ------------------------------------------------------------------
    # Transport implementation
    # ------------------------------------------------------------------

    def write(self, data: bytes) -> None:
        """Write *data* to the serial port."""
        self._serial.write(data)

    def read(self, n: int) -> bytes:
        """Read up to *n* bytes; returns fewer on timeout."""
        return self._serial.read(n)  # type: ignore[return-value]

    def reset_input(self) -> None:
        """Flush the serial receive buffer."""
        self._serial.reset_input_buffer()

    def close(self) -> None:
        """Close the serial port."""
        self._serial.close()


# ---------------------------------------------------------------------------
# LoopbackTransport
# ---------------------------------------------------------------------------


class LoopbackTransport:
    """In-memory transport for hardware-free testing.

    Two byte buffers model the two directions of the wire:

    - **write buffer** — bytes that the session (host) has sent; a
      ``FakeHub`` calls :meth:`drain_writes` to inspect them.
    - **read buffer** — bytes that will be returned to the session when it
      calls :meth:`read`; a ``FakeHub`` calls :meth:`inject` to enqueue
      response bytes.

    :meth:`read` returns immediately with whatever is in the read buffer (up
    to *n* bytes), or ``b""`` when the buffer is empty.  There is no blocking
    — the in-process test drives both sides synchronously.
    """

    def __init__(self) -> None:
        self._write_buf: bytearray = bytearray()
        self._read_buf: bytearray = bytearray()

    # ------------------------------------------------------------------
    # Transport implementation
    # ------------------------------------------------------------------

    def write(self, data: bytes) -> None:
        """Append *data* to the write buffer."""
        self._write_buf.extend(data)

    def read(self, n: int) -> bytes:
        """Pop up to *n* bytes from the read buffer.

        Returns ``b""`` when the buffer is empty (non-blocking).
        """
        chunk = bytes(self._read_buf[:n])
        del self._read_buf[:n]
        return chunk

    def reset_input(self) -> None:
        """Clear the read buffer."""
        self._read_buf.clear()

    def close(self) -> None:
        """No-op — nothing to release."""

    # ------------------------------------------------------------------
    # Test helpers (not part of the Transport Protocol)
    # ------------------------------------------------------------------

    def inject(self, data: bytes) -> None:
        """Append *data* to the read buffer.

        Use this from a ``FakeHub`` or test to simulate bytes arriving from
        the hub.
        """
        self._read_buf.extend(data)

    def drain_writes(self) -> bytes:
        """Return and clear all bytes that the session has written.

        Use this from a ``FakeHub`` or test to inspect outbound traffic.
        """
        result = bytes(self._write_buf)
        self._write_buf.clear()
        return result
