"""Servo — placeholder for ticket 011.

This module provides a minimal ``Servo`` class that ``Hub`` uses to build
``hub.servos[0..5]``.  Ticket 011 will flesh out the typed servo commands.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["Servo"]


class Servo:
    """Placeholder representation of a single servo channel.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    channel:
        Servo channel index (0–5).
    address:
        Hub module address on the RS-485 bus.
    """

    def __init__(self, session: "Session", channel: int, address: int) -> None:
        self.session = session
        self.channel = channel
        self.address = address

    def __repr__(self) -> str:
        return f"Servo(address={self.address!r}, channel={self.channel!r})"
