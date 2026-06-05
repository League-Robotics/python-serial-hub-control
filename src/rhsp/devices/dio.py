"""DIOPin — typed wrapper for RHSP digital I/O commands.

P7-b fix: ``get_direction()`` correctly returns the decoded ``bool`` value
from ``session.get_dio_direction()`` (the vendor implementation never
returned the value).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["DIOPin"]


class DIOPin:
    """Typed representation of a single digital I/O pin.

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

    def set_direction(self, output: bool) -> None:
        """Set the direction of this DIO pin.

        Parameters:
            output: ``True`` → output; ``False`` → input.
        """
        self.session.set_dio_direction(self.address, self.pin, output)

    def get_direction(self) -> bool:
        """Read the direction of this DIO pin.

        Returns:
            ``True`` if the pin is configured as output.

        Note:
            P7-b fix: the vendor implementation dropped the return value;
            this implementation correctly returns it.
        """
        return self.session.get_dio_direction(self.address, self.pin)

    def write(self, value: bool) -> None:
        """Set the output value of this DIO pin.

        Parameters:
            value: ``True`` for logic high; ``False`` for logic low.
        """
        self.session.set_single_dio_output(self.address, self.pin, value)

    def read(self) -> bool:
        """Read the input value of this DIO pin.

        Returns:
            ``True`` for logic high.
        """
        return self.session.get_single_dio_input(self.address, self.pin)

    def __repr__(self) -> str:
        return f"DIOPin(address={self.address!r}, pin={self.pin!r})"
