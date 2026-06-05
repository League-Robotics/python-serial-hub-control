"""ADCPin — placeholder for ticket 011.

This module provides a minimal ``ADCPin`` class that ``Hub`` uses to build
``hub.adc[0..3]``.  Ticket 011 will flesh out the typed ADC commands.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["ADCPin"]


class ADCPin:
    """Placeholder representation of a single ADC channel.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    channel:
        ADC channel index (0–3).
    address:
        Hub module address on the RS-485 bus.
    """

    def __init__(self, session: "Session", channel: int, address: int) -> None:
        self.session = session
        self.channel = channel
        self.address = address

    def __repr__(self) -> str:
        return f"ADCPin(address={self.address!r}, channel={self.channel!r})"
