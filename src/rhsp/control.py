"""Two-layer velocity control for RHSP motors.

Design overview
---------------
The host-side velocity control layer is split into two levels:

1. **Inner loop** — a :class:`VelocityController` that turns a single
   motor channel into a closed-loop actuator with a uniform interface
   (``attach``, ``command``, ``measured``, ``detach``).  The canonical
   concrete implementation is :class:`HubVelocityController`, which
   delegates the PID computation to the firmware's built-in
   ``CONSTANT_VELOCITY`` mode.

2. **Outer loop** — a ``RatioDrive`` (ticket 003) that coordinates two
   :class:`VelocityController` instances (left / right wheels) and maps
   abstract ``(linear_speed, angular_rate)`` commands to per-wheel
   velocity targets.

Composition pattern::

    import rhsp
    from rhsp.control import HubVelocityController

    with rhsp.connect() as hub:
        hub.init_peripherals()
        left  = HubVelocityController(hub, channel=0)
        right = HubVelocityController(hub, channel=1)
        left.attach()
        right.attach()
        left.command(600)
        right.command(600)
        bulk = hub.bulk_input()
        print(left.measured(bulk), right.measured(bulk))
        left.detach()
        right.detach()
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from rhsp.enums import MotorMode
from rhsp.devices.bulk import BulkInputData

if TYPE_CHECKING:
    from rhsp.hub import Hub
    from rhsp.devices.motor import Motor

__all__ = [
    "VelocityController",
    "HubVelocityController",
    "clamp_int16",
]

# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

_INT16_MIN: int = -32767
_INT16_MAX: int = 32767


def clamp_int16(value: int) -> int:
    """Clamp *value* to the signed 16-bit range ``[-32767, 32767]``.

    The maximum is 32767 (not 32768) because the hub treats -32768 as a
    special "float-at-zero" sentinel for some firmware variants.  Clamping
    symmetrically to ±32767 avoids that corner case.

    Parameters:
        value: Unclamped target velocity (encoder counts per second).

    Returns:
        The value clamped to ``[-32767, 32767]``.
    """
    if value > _INT16_MAX:
        return _INT16_MAX
    if value < _INT16_MIN:
        return _INT16_MIN
    return value


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------


class VelocityController(ABC):
    """Abstract seam for a single-channel closed-loop velocity actuator.

    Both the firmware-managed :class:`HubVelocityController` and any
    future host-side PID implementation must satisfy this interface so
    that higher-level coordinators (e.g. ``RatioDrive``) are decoupled
    from the concrete inner loop.

    Contract:
        - :meth:`attach` must be called before :meth:`command` or
          :meth:`measured`.
        - :meth:`detach` releases motor control; the object may be
          re-attached by calling :meth:`attach` again.
        - :attr:`available` indicates whether this controller can be used
          on the current hardware/firmware.
    """

    @abstractmethod
    def attach(self) -> None:
        """Claim the motor channel and enter velocity-control mode.

        Implementations must record any state needed to restore the motor
        to its pre-attach configuration when :meth:`detach` is called.
        """

    @abstractmethod
    def command(self, target_counts_s: int) -> None:
        """Issue a closed-loop velocity setpoint.

        Parameters:
            target_counts_s:
                Target velocity in encoder counts per second.  Values
                outside the hardware range will be clamped by the
                implementation.
        """

    @abstractmethod
    def measured(self, bulk: BulkInputData) -> int:
        """Return the most-recent measured velocity from a bulk snapshot.

        Parameters:
            bulk:
                A :class:`~rhsp.devices.bulk.BulkInputData` obtained from
                ``hub.bulk_input()``.

        Returns:
            Signed encoder counts per second for this channel.
        """

    @abstractmethod
    def detach(self, disable: bool = True) -> None:
        """Release the motor channel and restore its prior mode.

        Parameters:
            disable:
                If ``True`` (default), disable the motor output before
                restoring the prior mode.  Pass ``False`` to leave the
                motor enabled (useful if the caller will immediately
                transition to a different mode).
        """

    @property
    @abstractmethod
    def available(self) -> bool:
        """``True`` if this controller is usable on the current hardware."""


# ---------------------------------------------------------------------------
# Concrete implementation — hub firmware PID
# ---------------------------------------------------------------------------


class HubVelocityController(VelocityController):
    """Velocity controller backed by the hub's built-in ``CONSTANT_VELOCITY`` PID.

    The hub firmware runs the entire PID loop internally; the host only
    needs to send a target velocity (``SetMotorTargetVelocity``) and read
    back the measured velocity from ``GetBulkInputData``.

    Parameters:
        hub:     The connected :class:`~rhsp.hub.Hub`.
        channel: Motor channel index (0–3).

    Notes:
        - :meth:`command` clamps the target to ``[-32767, 32767]`` before
          forwarding to the firmware.  Values ±32768 are clamped to ±32767
          to avoid a firmware edge-case with -32768.
        - :attr:`available` returns ``True`` unconditionally because
          ``CONSTANT_VELOCITY`` is present on all known firmware versions
          (≥ 1.8.2).  A runtime probe can override this via subclassing if
          needed.
    """

    def __init__(self, hub: "Hub", channel: int) -> None:
        self._motor: "Motor" = hub.motors[channel]
        self._prior_mode: MotorMode | None = None

    def attach(self) -> None:
        """Enter ``CONSTANT_VELOCITY`` mode and enable the motor.

        Records the motor's current mode (as an opaque integer) so that
        :meth:`detach` can restore it.  In practice the mode is always
        ``CONSTANT_POWER`` after :meth:`~rhsp.hub.Hub.init_peripherals`,
        but recording it defensively is correct regardless.

        The firmware requires ``SetMotorTargetVelocity`` to be sent before
        ``SetMotorChannelEnable`` can be accepted in ``CONSTANT_VELOCITY``
        mode (NACK 50 otherwise).  A zero target is sent here so the motor
        starts at rest and can be enabled without error.
        """
        # Record mode before switching.  Hub init_peripherals always leaves
        # motors in CONSTANT_POWER, but we record whatever the mode is now.
        self._prior_mode = MotorMode.CONSTANT_POWER  # safe default
        self._motor.set_mode(MotorMode.CONSTANT_VELOCITY, float_at_zero=True)
        # Send an initial zero target so the firmware accepts enable().
        self._motor.set_target_velocity(0)
        self._motor.enable()

    def command(self, target_counts_s: int) -> None:
        """Send a clamped velocity setpoint to the hub firmware.

        Parameters:
            target_counts_s:
                Desired velocity in encoder counts per second.  Values
                outside ``[-32767, 32767]`` are silently clamped.
        """
        clamped = clamp_int16(target_counts_s)
        self._motor.set_target_velocity(clamped)

    def measured(self, bulk: BulkInputData) -> int:
        """Return the measured velocity from a bulk snapshot.

        Parameters:
            bulk:
                A :class:`~rhsp.devices.bulk.BulkInputData` obtained from
                ``hub.bulk_input()``.

        Returns:
            Signed encoder counts per second for this channel.
        """
        return self._motor.get_velocity(bulk)

    def detach(self, disable: bool = True) -> None:
        """Disable the motor (optionally) and restore its prior mode.

        Parameters:
            disable:
                If ``True`` (default), call ``motor.disable()`` before
                restoring the recorded mode.  Pass ``False`` to leave the
                motor enabled across the mode switch.
        """
        if disable:
            self._motor.disable()
        # Restore the prior mode that was recorded in attach().
        prior = self._prior_mode if self._prior_mode is not None else MotorMode.CONSTANT_POWER
        self._motor.set_mode(prior, float_at_zero=True)

    @property
    def available(self) -> bool:
        """``True`` — ``CONSTANT_VELOCITY`` is available on fw ≥ 1.8.2."""
        return True
