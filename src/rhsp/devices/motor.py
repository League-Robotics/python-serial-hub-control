"""Motor — placeholder for ticket 011.

This module provides a minimal ``Motor`` class that ``Hub`` uses to build
``hub.motors[0..3]``.  Ticket 011 will flesh out the typed motor commands.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["Motor"]


class Motor:
    """Placeholder representation of a single motor channel.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    channel:
        Motor channel index (0–3).
    address:
        Hub module address on the RS-485 bus.
    """

    def __init__(self, session: "Session", channel: int, address: int) -> None:
        self.session = session
        self.channel = channel
        self.address = address

    def __repr__(self) -> str:
        return f"Motor(address={self.address!r}, channel={self.channel!r})"
