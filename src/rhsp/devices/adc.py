"""ADCPin — typed wrapper for RHSP ADC commands."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["ADCPin"]


class ADCPin:
    """Typed representation of a single ADC channel.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared by all devices on the hub.
    channel:
        ADC channel index (0–3 for external inputs; higher indices for internal
        monitoring rails — see :class:`~rhsp.enums.ADCChannel`).
    address:
        Hub module address on the RS-485 bus.
    """

    def __init__(self, session: "Session", channel: int, address: int) -> None:
        self.session = session
        self.channel = channel
        self.address = address

    def read(self, raw: bool = False) -> int:
        """Read the ADC channel.

        Parameters:
            raw: If ``True``, return raw counts; otherwise engineering units
                 (mV or mA depending on channel).

        Returns:
            ADC reading as a plain integer.
        """
        return self.session.get_adc(self.address, self.channel, raw=raw)

    def __repr__(self) -> str:
        return f"ADCPin(address={self.address!r}, channel={self.channel!r})"
