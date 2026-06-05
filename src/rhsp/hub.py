"""Hub — represents a single connected REV Hub module.

This module is a minimal stub created by ticket 009 to satisfy the return type
of ``discovery.connect()``.  Ticket 010 will flesh out ``Hub`` with device
objects (motors, servos, DIO, ADC, I2C, sensors) and ``init_peripherals()``.

Public API (ticket 009 seam)
----------------------------
- ``Hub(session, address)``:
  A thin container holding a ``Session`` and the module's address on the RS-485
  bus.  Child hubs (discovered via RS-485 expansion) are stored in ``children``.
- ``Hub.keep_alive()``:
  Send a KeepAlive to the hub to prevent fail-safe within 2500 ms.
- ``Hub.read_version_string()``:
  Convenience wrapper for ``session.read_version_string(address)``.

All other typed session methods are accessible via ``hub.session``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["Hub"]


class Hub:
    """Minimal representation of a connected REV Hub module.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared across all hubs on the
        same RS-485 bus.
    address:
        Module address on the RS-485 bus (typically 2 for the parent hub).

    Attributes
    ----------
    session:
        The shared :class:`~rhsp.session.Session`.
    address:
        Module address (``int``).
    children:
        List of child :class:`Hub` objects discovered during connection.
        Populated by :func:`~rhsp.discovery.connect`.
    deka_base:
        Shortcut to ``session.deka_base``; set after ``QueryInterface("DEKA")``
        is called during :func:`~rhsp.discovery.connect`.
    """

    def __init__(self, session: "Session", address: int) -> None:
        self.session = session
        self.address = address
        # Populated by connect() after discovery.
        self.children: list["Hub"] = []

    @property
    def deka_base(self) -> int:
        """DEKA base packet ID, mirrored from the shared session."""
        return self.session.deka_base

    def keep_alive(self) -> None:
        """Send a KeepAlive to prevent the hub entering fail-safe.

        Must be called within 2500 ms of the last packet or the hub will
        disable all outputs.
        """
        self.session.keep_alive(dest=self.address)

    def read_version_string(self) -> str:
        """Read the firmware version string from this hub.

        Returns
        -------
        str
            Version string, e.g. ``"HW: 20, Maj: 1, Min: 8, Eng: 2"``.
        """
        return self.session.read_version_string(dest=self.address)

    def __repr__(self) -> str:
        return (
            f"Hub(address={self.address!r}, "
            f"deka_base=0x{self.deka_base:04X}, "
            f"children={self.children!r})"
        )
