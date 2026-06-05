"""Servo — typed wrapper for RHSP servo commands."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["Servo"]

# Pulse-width limits (microseconds) per the servo specification.
_PW_MIN: int = 500
_PW_MAX: int = 2500


class Servo:
    """Typed representation of a single servo channel.

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

    # ------------------------------------------------------------------
    # Enable / disable
    # ------------------------------------------------------------------

    def enable(self) -> None:
        """Enable this servo channel."""
        self.session.set_servo_enable(self.address, self.channel, True)

    def disable(self) -> None:
        """Disable this servo channel."""
        self.session.set_servo_enable(self.address, self.channel, False)

    # ------------------------------------------------------------------
    # Pulse width
    # ------------------------------------------------------------------

    def set_pulse_width(self, pw: int) -> None:
        """Set the servo pulse width.

        Parameters:
            pw: Pulse width in microseconds; must be in [500, 2500].

        Raises:
            ValueError: If *pw* is outside the valid range.
        """
        if not (_PW_MIN <= pw <= _PW_MAX):
            raise ValueError(
                f"Pulse width {pw} µs is out of range [{_PW_MIN}, {_PW_MAX}]"
            )
        self.session.set_servo_pulse_width(self.address, self.channel, pw)

    def set_angle(self, degrees: float) -> None:
        """Set the servo angle.

        Converts *degrees* to a pulse width using the formula:
        ``pulse_width = 500 + degrees * 2000 / 180``

        The resulting value is clamped to [500, 2500].

        Parameters:
            degrees: Angle in degrees (nominally 0–180).
        """
        pw = int(round(500.0 + degrees * 2000.0 / 180.0))
        pw = max(_PW_MIN, min(_PW_MAX, pw))
        self.session.set_servo_pulse_width(self.address, self.channel, pw)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def set_configuration(self, frame_period: int) -> None:
        """Set the servo frame period.

        Parameters:
            frame_period: Frame period in microseconds (e.g. 20000 for 50 Hz).
        """
        self.session.set_servo_configuration(self.address, self.channel, frame_period)

    # ------------------------------------------------------------------
    # Dunder
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return f"Servo(address={self.address!r}, channel={self.channel!r})"
