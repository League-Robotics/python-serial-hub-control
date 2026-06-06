"""Motor — typed wrapper for RHSP motor commands.

Each ``Motor`` is a thin wrapper over ``session`` typed methods.
Velocity is read from a pre-fetched :class:`~rhsp.devices.bulk.BulkInputData`
(``GetBulkInputData``, DEKA 0x00), never via the legacy
``GetBulkMotorData`` (DEKA 0x37).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from rhsp.enums import ClosedLoopMode, MotorMode

if TYPE_CHECKING:
    from rhsp.devices.bulk import BulkInputData
    from rhsp.session import Session

__all__ = ["Motor"]


class Motor:
    """Typed representation of a single motor channel.

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

    # ------------------------------------------------------------------
    # Enable / disable
    # ------------------------------------------------------------------

    def enable(self) -> None:
        """Enable this motor channel."""
        self.session.set_motor_channel_enable(self.address, self.channel, True)

    def disable(self) -> None:
        """Disable this motor channel."""
        self.session.set_motor_channel_enable(self.address, self.channel, False)

    # ------------------------------------------------------------------
    # Mode and power
    # ------------------------------------------------------------------

    def set_mode(self, mode: MotorMode, float_at_zero: bool = True) -> None:
        """Set the run mode for this motor channel.

        Parameters:
            mode:          Control mode (see :class:`~rhsp.enums.MotorMode`).
            float_at_zero: ``True`` → float (coast) at zero power; ``False`` → brake.
        """
        self.session.set_motor_channel_mode(
            self.address, self.channel, mode, float_at_zero
        )

    def set_power(self, power: int) -> None:
        """Set the open-loop constant power for this motor channel.

        Parameters:
            power: Power level in range [−32767, 32767].
        """
        self.session.set_motor_constant_power(self.address, self.channel, power)

    # ------------------------------------------------------------------
    # Closed-loop velocity / position
    # ------------------------------------------------------------------

    def set_target_velocity(self, velocity: int) -> None:
        """Set the closed-loop target velocity.

        Parameters:
            velocity: Target velocity in encoder counts per second (signed 16-bit).
        """
        self.session.set_motor_target_velocity(self.address, self.channel, velocity)

    def set_target_position(self, position: int, tolerance: int) -> None:
        """Set the closed-loop target position.

        Parameters:
            position:  Target encoder count (signed 32-bit).
            tolerance: Acceptable position error in encoder counts.
        """
        self.session.set_motor_target_position(
            self.address, self.channel, position, tolerance
        )

    def at_target(self) -> bool:
        """Return ``True`` if the motor is within tolerance of its target."""
        return self.session.get_motor_at_target(self.address, self.channel)

    # ------------------------------------------------------------------
    # Encoder
    # ------------------------------------------------------------------

    def get_encoder_position(self) -> int:
        """Read the current encoder position (signed 32-bit)."""
        return self.session.get_motor_encoder_position(self.address, self.channel)

    def reset_encoder(self) -> None:
        """Reset the encoder count to zero."""
        self.session.reset_motor_encoder(self.address, self.channel)

    # ------------------------------------------------------------------
    # Velocity from BulkInputData
    # ------------------------------------------------------------------

    def get_velocity(self, hub_bulk: "BulkInputData") -> int:
        """Return the signed velocity for this channel from a pre-fetched bulk snapshot.

        This method does **not** issue any new RHSP transaction; it reads
        ``motorNVelocity`` (signed 16-bit) from the caller-supplied
        :class:`~rhsp.devices.bulk.BulkInputData` object, which is decoded
        from a ``GetBulkInputData`` (DEKA 0x00) response.  The legacy
        ``GetBulkMotorData`` (DEKA 0x37) is deliberately not used.

        Parameters:
            hub_bulk: A :class:`~rhsp.devices.bulk.BulkInputData` instance
                obtained from a recent ``hub.bulk_input()`` or
                ``session.get_bulk_input_data()`` call.

        Returns:
            Signed encoder-counts-per-second velocity for this channel.
        """
        attr = f"motor{self.channel}_velocity"
        return getattr(hub_bulk, attr)

    # ------------------------------------------------------------------
    # PID coefficients
    # ------------------------------------------------------------------

    def set_velocity_pid(self, p: float, i: float, d: float) -> None:
        """Set the velocity closed-loop PID coefficients for this motor channel.

        Delegates directly to
        :meth:`~rhsp.session.Session.set_motor_pid_coefficients` with
        :attr:`~rhsp.enums.ClosedLoopMode.VELOCITY`.  Q16 encoding is handled
        by the session layer — pass plain floating-point coefficients.

        Parameters:
            p: Proportional gain.
            i: Integral gain.
            d: Derivative gain.
        """
        self.session.set_motor_pid_coefficients(
            self.address, self.channel, ClosedLoopMode.VELOCITY, p, i, d
        )

    def get_velocity_pid(self) -> tuple[float, float, float]:
        """Read the velocity closed-loop PID coefficients for this motor channel.

        Delegates directly to
        :meth:`~rhsp.session.Session.get_motor_pid_coefficients` with
        :attr:`~rhsp.enums.ClosedLoopMode.VELOCITY`.  The session layer handles
        Q16 decoding — the returned values are plain floating-point coefficients.

        Returns:
            A ``(p, i, d)`` tuple of floating-point PID coefficients.
        """
        return self.session.get_motor_pid_coefficients(
            self.address, self.channel, ClosedLoopMode.VELOCITY
        )

    # ------------------------------------------------------------------
    # Dunder
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return f"Motor(address={self.address!r}, channel={self.channel!r})"
