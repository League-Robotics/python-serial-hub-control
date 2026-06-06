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
  Run the bring-up recipe.  Clears any latched ``KeepAliveTimeout|FailSafe``
  status first, then:
  - motor channels 0–3: ``SetMotorChannelMode(ch, CONSTANT_POWER, float_at_zero=True)``
    then ``SetMotorConstantPower(ch, 0)``
  - servo channels 0–5: ``SetServoConfiguration(ch, 20000)``
  If any command raises, ``fail_safe()`` is called and the exception re-raised.
- ``with hub:`` / ``hub.start_keepalive(interval=2.0, on_error=None)`` / ``hub.stop_keepalive()``:
  Hub-managed heartbeat.  Use ``with hub:`` so the hub stays alive
  automatically and fail-safes on exit.  ``keep_alive()`` is still
  available for explicit one-shot pings.
- ``hub.keep_alive() -> None``:
  Send a single KeepAlive packet (explicit / one-shot).
- ``hub.fail_safe() -> None``:
  Immediately put the hub into fail-safe (all outputs disabled).
- ``hub.get_module_status(clear=False) -> ModuleStatus``:
  Read (and optionally clear) the module status register.
- ``hub.set_led_color(r, g, b) -> None``:
  Set the module LED colour (fire-and-forget; heartbeat re-asserts it).
- ``hub.clear_led_color() -> None``:
  Cancel the stored LED pattern; LED returns to firmware default on next heartbeat.
- ``hub.read_version_string() -> str``:
  Read the firmware version string.
- ``hub.bulk_input() -> BulkInputData``:
  Read all digital/analog/motor/servo/I2C state in one transaction.
"""

from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING, Callable

from rhsp.devices.adc import ADCPin
from rhsp.devices.bulk import BulkInputData, ModuleStatus
from rhsp.devices.dio import DIOPin
from rhsp.devices.i2c import I2CChannel
from rhsp.devices.motor import Motor
from rhsp.devices.servo import Servo
from rhsp.enums import ADCChannel, ModuleStatusBits, MotorMode

if TYPE_CHECKING:
    from rhsp.session import Session

__all__ = ["Hub"]

_logger = logging.getLogger(__name__)

# Bits that represent a routine re-latch every heartbeat cycle (always cleared).
_ROUTINE_LATCH_MASK: ModuleStatusBits = (
    ModuleStatusBits.KeepAliveTimeout | ModuleStatusBits.FailSafe
)

# Bits that represent genuine hardware faults (never silently cleared).
_NON_ROUTINE_FAULT_MASK: ModuleStatusBits = (
    ModuleStatusBits.ControllerOverTemp
    | ModuleStatusBits.BatteryLow
    | ModuleStatusBits.HIBFault
)


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

        # Keep-alive heartbeat state.
        self._ka_thread: threading.Thread | None = None
        self._ka_stop: threading.Event = threading.Event()

        # Desired LED pattern (fire-and-forget): stored by set_led_color(),
        # cleared by clear_led_color().  The heartbeat re-asserts it each tick.
        # Each step is an (r, g, b, t) tuple (r=red, g=green, b=blue, t=duration).
        self._desired_led_pattern: list[tuple[int, int, int, int]] | None = None

        # Optional callback invoked when a non-routine fault is detected.
        self._on_error: Callable[[int], None] | None = None

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

        Clears the ``KeepAliveTimeout | FailSafe`` latch before any motor or
        servo command so the LED is immediately usable without a blinking-blue
        window.

        For each motor channel 0–3:
        - ``SetMotorChannelMode(ch, CONSTANT_POWER, float_at_zero=True)``
        - ``SetMotorConstantPower(ch, 0)``

        For each servo channel 0–5:
        - ``SetServoConfiguration(ch, 20000)``
        - ``SetServoPulseWidth(ch, 1500)``  (center; required before Enable)

        If any command raises (including :exc:`~rhsp.errors.NackError`),
        ``fail_safe()`` is called on this hub and the original exception is
        re-raised.
        """
        try:
            # Clear the connect-time KeepAliveTimeout|FailSafe latch so the LED
            # is usable immediately after init_peripherals() returns.
            self.session.get_module_status(self.address, clear=True)

            for ch in range(4):
                self.session.set_motor_channel_mode(
                    self.address, ch, MotorMode.CONSTANT_POWER, True
                )
                self.session.set_motor_constant_power(self.address, ch, 0)
            for ch in range(6):
                self.session.set_servo_configuration(self.address, ch, 20000)
                # Hub requires a pulse width before SetServoEnable will be accepted.
                self.session.set_servo_pulse_width(self.address, ch, 1500)
        except Exception:
            self.session.fail_safe(self.address)
            raise

    # ------------------------------------------------------------------
    # System operations
    # ------------------------------------------------------------------

    def keep_alive(self) -> None:
        """Send a single KeepAlive packet.

        The hub disables all outputs if no valid packet arrives within
        **2500 ms**.  Prefer :meth:`start_keepalive` (or ``with hub:``) so
        the heartbeat runs automatically; call this only when you need an
        explicit one-shot ping.
        """
        self.session.keep_alive(dest=self.address)

    def start_keepalive(
        self,
        interval: float = 2.0,
        *,
        on_error: Callable[[int], None] | None = None,
    ) -> None:
        """Start a background heartbeat thread that sends KeepAlive packets.

        The hub disables all outputs if no valid packet arrives within
        2500 ms.  This method launches a daemon thread that calls
        :meth:`keep_alive` every *interval* seconds, so user code never
        needs to call it manually.

        After each heartbeat the thread inspects the module status register:

        - If only routine ``KeepAliveTimeout | FailSafe`` bits are set, the
          latch is cleared and the stored LED pattern (if any) is re-asserted.
        - If any non-routine fault bit (over-temp, battery-low, HIB fault) is
          present, a ``WARNING`` is logged, ``on_error`` is invoked (if
          provided), and the pattern is **not** cleared or re-asserted.

        The preferred idiom is to use ``Hub`` as a context manager::

            with rhsp.connect(port) as hub:
                hub.init_peripherals()
                hub.set_led_color(0, 255, 0)  # green — held automatically
                hub.motors[0].set_power(16000)
                time.sleep(10)  # heartbeat fires; LED stays green

        Parameters
        ----------
        interval:
            Seconds between heartbeats.  Must be less than 2.5 s.
            Defaults to 2.0 s.
        on_error:
            Optional callable invoked with the raw fault flags (int) when a
            non-routine fault is detected.  If ``None``, only a WARNING is
            logged.
        """
        if interval >= 2.5:
            raise ValueError("interval must be < 2.5 s (hub fail-safe timeout)")
        if self._ka_thread is not None and self._ka_thread.is_alive():
            return  # already running

        if on_error is not None:
            self._on_error = on_error

        self._ka_stop.clear()

        def _loop() -> None:
            while not self._ka_stop.wait(timeout=interval):
                try:
                    self.keep_alive()
                except Exception:
                    pass  # transient error — keep trying
                self._heartbeat_post_keepalive()

        self._ka_thread = threading.Thread(target=_loop, daemon=True, name="rhsp-keepalive")
        self._ka_thread.start()

    def _heartbeat_post_keepalive(self) -> None:
        """Check module status after a heartbeat and re-assert the LED pattern.

        Called by the keep-alive loop after each successful ``keep_alive()``.
        Reads the module status without clearing first, then:

        - If only ``KeepAliveTimeout | FailSafe`` bits are set (routine
          re-latch), clear the latch and re-send the stored LED pattern.
        - If any non-routine fault bit is present, log a WARNING and invoke
          ``_on_error`` (if set).  The pattern is NOT cleared or re-asserted.
        - If no fault bits are set, skip silently.
        """
        if self._desired_led_pattern is None:
            return  # Nothing to re-assert; skip the status read.

        try:
            status = self.get_module_status(clear=False)
        except Exception:
            return  # transient error — skip this tick

        flags = int(status.status_bits)
        if not flags:
            return  # No bits set; LED is under user control — nothing to do.

        if flags & int(_NON_ROUTINE_FAULT_MASK):
            # Non-routine fault: log + callback, do NOT re-assert or clear.
            _logger.warning("non-routine hub fault 0x%04x", flags)
            if self._on_error is not None:
                try:
                    self._on_error(flags)
                except Exception:
                    pass
            return

        # Only routine KeepAliveTimeout|FailSafe bits set — clear and re-assert.
        try:
            self.get_module_status(clear=True)
            pattern = self._desired_led_pattern
            if pattern is not None:
                self.session.set_module_led_pattern(self.address, pattern)
        except Exception:
            pass  # transient error — will retry next tick

    def stop_keepalive(self) -> None:
        """Stop the background heartbeat thread (if running)."""
        self._ka_stop.set()
        if self._ka_thread is not None:
            self._ka_thread.join(timeout=1.0)
            self._ka_thread = None

    def fail_safe(self) -> None:
        """Immediately put this hub into fail-safe (all outputs disabled)."""
        self.session.fail_safe(self.address)

    # ------------------------------------------------------------------
    # Context manager — starts heartbeat on enter, stops + fail-safes on exit
    # ------------------------------------------------------------------

    def __enter__(self) -> "Hub":
        """Start the keep-alive heartbeat."""
        self.start_keepalive()
        return self

    def __exit__(self, *_: object) -> None:
        """Stop the heartbeat and fail-safe the hub."""
        self.stop_keepalive()
        try:
            self.fail_safe()
        except Exception:
            pass

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
        """Set the module LED to a solid colour (fire-and-forget).

        Drives the LED via ``SetModuleLEDPattern`` (all 16 steps the same
        colour) and stores the pattern so the background heartbeat can
        automatically re-assert it after each ``KeepAliveTimeout | FailSafe``
        re-latch.  The caller never needs to re-send the pattern.

        Two firmware quirks (verified on fw 1.8.2) make the naive approach
        fail silently:

        * ``SetModuleLEDColor`` (0x7F0A) is a no-op — it ACKs but never
          changes the LED, so it is not used here.
        * The LED is held in its blinking-blue *status* animation while the
          ``KeepAliveTimeout``/``FailSafe`` flags are latched (they latch
          during connect, before the keep-alive heartbeat is up). The hub
          ignores LED commands until those flags are cleared, so this method
          clears them first via ``GetModuleStatus(clear=True)``.

        Parameters
        ----------
        r:
            Red component (0–255).
        g:
            Green component (0–255).
        b:
            Blue component (0–255).
        """
        # Build the 16-step solid pattern: each step is (r, g, b, t=1).
        solid: list[tuple[int, int, int, int]] = [(r, g, b, 1)] * 16

        # Store the pattern so the heartbeat can re-assert it.
        self._desired_led_pattern = solid

        # Drop the latched KeepAliveTimeout|FailSafe so the firmware releases
        # the LED from its status animation to user control.
        self.session.get_module_status(self.address, clear=True)
        self.session.set_module_led_pattern(self.address, solid)

    def clear_led_color(self) -> None:
        """Cancel the stored LED pattern.

        Sets ``_desired_led_pattern`` to ``None``.  The LED will return to
        the firmware default (blinking blue) on the next heartbeat tick when
        the ``KeepAliveTimeout | FailSafe`` latch fires.  No immediate packet
        is sent.
        """
        self._desired_led_pattern = None

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

        .. note::
            On fw 1.8.2 the hub response is only ~34 bytes.  The
            :class:`~rhsp.devices.bulk.BulkInputData` fields that are within
            that window (encoders, velocities, modes, analog_input0/1) are
            correct.  Fields beyond offset 34 are zero-padded and must not be
            used for current or voltage readings — use
            :meth:`battery_voltage_mv`, :meth:`battery_current_ma`, or
            :meth:`~rhsp.devices.motor.Motor.get_current_ma` instead.
        """
        return self.session.get_bulk_input_data(dest=self.address)

    def battery_voltage_mv(self) -> int:
        """Read the battery voltage in millivolts via ``GetADC``.

        Uses ADC channel ``ADC_BatteryMonitor`` (channel 13), which is the
        correct and reliable method on fw 1.8.2.  The bulk-input path does not
        carry battery voltage on this firmware.

        Returns
        -------
        int
            Battery voltage in millivolts.  Typically 11000–12500 mV when
            fully charged.
        """
        return self.session.get_adc(self.address, ADCChannel.ADC_BatteryMonitor)

    def battery_current_ma(self) -> int:
        """Read the battery current in milliamps via ``GetADC``.

        Uses ADC channel ``ADC_Battery`` (channel 7).  Reads 0 mA at idle
        (no load).

        Returns
        -------
        int
            Battery current draw in milliamps.
        """
        return self.session.get_adc(self.address, ADCChannel.ADC_Battery)

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
