"""DIOPin — placeholder for ticket 011.

This module provides a minimal ``DIOPin`` class that ``Hub`` uses to build
``hub.dio[0..7]``.  Ticket 011 will flesh out the typed DIO commands.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["DIOPin"]


class DIOPin:
    """Placeholder representation of a single digital I/O pin.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    pin:
        DIO pin index (0–7).
    address:
        Hub module address on the RS-485 bus.
    """

    def __init__(self, session: "Session", pin: int, address: int) -> None:
        self.session = session
        self.pin = pin
        self.address = address

    def __repr__(self) -> str:
        return f"DIOPin(address={self.address!r}, pin={self.pin!r})"
