"""Hub — represents a single connected REV Hub module.

A ``Hub`` object owns all peripheral device objects (motors, servos, DIO,
ADC, I2C), runs the §5.2 peripheral initialisation recipe with ``fail_safe()``
rollback on error, and exposes an explicit ``keep_alive()``.

``Hub`` carries exactly one ``address`` field — there is no separate
``module`` or ``destinationModule`` attribute.  Device objects are
constructed as ``DeviceClass(session, channel_or_pin, address)``.

Public API
----------
- ``Hub(session, address, parent=False)``:
  Construct the hub and build all device lists.  No hub transactions in
  ``__init__``.
- ``hub.init_peripherals() -> None``:
  Run the bring-up recipe:
  - motor channels 0–3: ``SetMotorChannelMode(ch, CONSTANT_POWER, float_at_zero=True)``
    then ``SetMotorConstantPower(ch, 0)``
  - servo channels 0–5: ``SetServoConfiguration(ch, 20000)``
  If any command raises, ``fail_safe()`` is called and the exception re-raised.
- ``hub.keep_alive() -> None``:
  Send a KeepAlive; must be called within 2500 ms to prevent fail-safe.
- ``hub.fail_safe() -> None``:
  Immediately put the hub into fail-safe (all outputs disabled).
- ``hub.get_module_status(clear=False) -> ModuleStatus``:
  Read (and optionally clear) the module status register.
- ``hub.set_led_color(r, g, b) -> None``:
  Set the module LED colour.
- ``hub.read_version_string() -> str``:
  Read the firmware version string.
- ``hub.bulk_input() -> BulkInputData``:
  Read all digital/analog/motor/servo/I2C state in one transaction.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from rhsp.devices.adc import ADCPin
from rhsp.devices.bulk import BulkInputData, ModuleStatus
from rhsp.devices.dio import DIOPin
from rhsp.devices.i2c import I2CChannel
from rhsp.devices.motor import Motor
from rhsp.devices.servo import Servo
from rhsp.enums import MotorMode

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["Hub"]


class Hub:
    """Representation of a single connected REV Hub module.

    Parameters
    ----------
    session:
        Open :class:`~rhsp.session.Session` shared across all hubs on the
        same RS-485 bus.
    address:
        Module address on the RS-485 bus (typically 2 for the parent hub).
    parent:
        ``True`` if this hub replied with the parent flag during Discovery.

    Attributes
    ----------
    session:
        The shared :class:`~rhsp.session.Session`.
    address:
        Module address (``int``).
    parent:
        ``True`` if this hub is the parent (directly USB-connected) hub.
    children:
        List of child :class:`Hub` objects discovered during connection.
        Populated by :func:`~rhsp.discovery.connect`.
    motors:
        List of 4 :class:`~rhsp.devices.motor.Motor` objects (channels 0–3).
    servos:
        List of 6 :class:`~rhsp.devices.servo.Servo` objects (channels 0–5).
    dio:
        List of 8 :class:`~rhsp.devices.dio.DIOPin` objects (pins 0–7).
    adc:
        List of 4 :class:`~rhsp.devices.adc.ADCPin` objects (channels 0–3).
    i2c:
        List of 4 :class:`~rhsp.devices.i2c.I2CChannel` objects (channels 0–3).
    """

    def __init__(
        self,
        session: "Session",
        address: int,
        parent: bool = False,
    ) -> None:
        self.session = session
        self.address = address
        self.parent = parent

        # Populated by connect() after discovery.
        self.children: list["Hub"] = []

        # Device lists — no hub transactions here.
        self.motors: list[Motor] = [
            Motor(session, ch, address) for ch in range(4)
        ]
        self.servos: list[Servo] = [
            Servo(session, ch, address) for ch in range(6)
        ]
        self.dio: list[DIOPin] = [
            DIOPin(session, pin, address) for pin in range(8)
        ]
        self.adc: list[ADCPin] = [
            ADCPin(session, ch, address) for ch in range(4)
        ]
        self.i2c: list[I2CChannel] = [
            I2CChannel(session, ch, address) for ch in range(4)
        ]

    # ------------------------------------------------------------------
    # Peripheral initialisation
    # ------------------------------------------------------------------

    def init_peripherals(self) -> None:
        """Run the §5.2 peripheral bring-up recipe.

        For each motor channel 0–3:
        - ``SetMotorChannelMode(ch, CONSTANT_POWER, float_at_zero=True)``
        - ``SetMotorConstantPower(ch, 0)``

        For each servo channel 0–5:
        - ``SetServoConfiguration(ch, 20000)``

        If any command raises (including :exc:`~rhsp.errors.NackError`),
        ``fail_safe()`` is called on this hub and the original exception is
        re-raised.
        """
        try:
            for ch in range(4):
                self.session.set_motor_channel_mode(
                    self.address, ch, MotorMode.CONSTANT_POWER, True
                )
                self.session.set_motor_constant_power(self.address, ch, 0)
            for ch in range(6):
                self.session.set_servo_configuration(self.address, ch, 20000)
        except Exception:
            self.session.fail_safe(self.address)
            raise

    # ------------------------------------------------------------------
    # System operations
    # ------------------------------------------------------------------

    def keep_alive(self) -> None:
        """Send a KeepAlive to prevent the hub entering fail-safe.

        The hub enters fail-safe (all outputs disabled) if no valid packet
        is received within **2500 ms**.  Call this method at least that
        frequently to keep the hub active.
        """
        self.session.keep_alive(dest=self.address)

    def fail_safe(self) -> None:
        """Immediately put this hub into fail-safe (all outputs disabled)."""
        self.session.fail_safe(self.address)

    def get_module_status(self, clear: bool = False) -> ModuleStatus:
        """Read (and optionally clear) the module status register.

        Parameters
        ----------
        clear:
            If ``True``, the hub clears the latched status after reading.

        Returns
        -------
        ModuleStatus
            Decoded module status flags.
        """
        return self.session.get_module_status(self.address, clear)

    def set_led_color(self, r: int, g: int, b: int) -> None:
        """Set the module LED colour.

        Parameters
        ----------
        r:
            Red component (0–255).
        g:
            Green component (0–255).
        b:
            Blue component (0–255).
        """
        self.session.set_module_led_color(self.address, r, g, b)

    def read_version_string(self) -> str:
        """Read the firmware version string from this hub.

        Returns
        -------
        str
            Version string, e.g. ``"HW: 20, Maj: 1, Min: 8, Eng: 2"``.
        """
        return self.session.read_version_string(dest=self.address)

    def bulk_input(self) -> BulkInputData:
        """Read all digital/analog/motor/servo/I2C state in one transaction.

        Returns
        -------
        BulkInputData
            All input fields decoded into a single dataclass.
        """
        return self.session.get_bulk_input_data(dest=self.address)

    # ------------------------------------------------------------------
    # Dunder
    # ------------------------------------------------------------------

    @property
    def deka_base(self) -> int:
        """DEKA base packet ID, mirrored from the shared session."""
        return self.session.deka_base

    def __repr__(self) -> str:
        return (
            f"Hub(address={self.address!r}, "
            f"parent={self.parent!r}, "
            f"deka_base=0x{self.deka_base:04X}, "
            f"children={self.children!r})"
        )
