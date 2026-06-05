"""Session — RHSP transaction engine.

``Session`` is the serialisation point between the application and the
hub.  Exactly one transaction is outstanding at any time.

Usage::

    from rhsp.transport import SerialTransport
    from rhsp.session import Session

    transport = SerialTransport("/dev/tty.usbmodem…")
    session = Session(transport)
    session.transaction("KeepAlive", dest=1)

Protocol state maintained here:
- ``_msg_num`` — a counter 1..255 that increments on every outbound
  frame and wraps from 255 back to 1 (never 0, as required by the spec).
- ``deka_base`` — the DEKA interface base address returned by
  ``QueryInterface``; defaults to 0x1000, overwritten by
  :meth:`query_interface` (added in ticket 008).

Threading note
--------------
``Session`` is **thread-safe for one background heartbeat thread plus one
caller thread** via an internal :class:`threading.RLock`.  Each public
I/O entry point (``transaction``, ``discover``, ``get_bulk_input_data``)
holds the lock for the full duration of the operation (write + read) so
transactions are atomic and cannot interleave on the wire.  The lock is
a *reentrant* lock so typed convenience methods (which call
``self.transaction`` internally) remain safe.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID, response_for, runtime_packet_id
from rhsp.codec import decode_payload, encode_payload
from rhsp.errors import ChecksumError, NackError, RhspTimeoutError
from rhsp.enums import (
    ClosedLoopMode,
    DIODirection,
    I2CSpeedCode,
    MotorMode,
    ModuleStatusBits,
    MotorStatusBits,
    NackCode,
    ZeroPowerBehavior,
)
from rhsp.framing import FrameParser, RawPacket, build_frame

if TYPE_CHECKING:
    from rhsp.transport import Transport
    from rhsp.devices.bulk import BulkInputData, ModuleStatus

__all__ = ["Session"]

# Absolute packet-type ids for system frames (not DEKA-relative).
_NACK_TYPE: int = 0x7F02
_DISCOVERY_TYPE: int = 0x7F0F
_DISCOVERY_RSP_TYPE: int = 0xFF0F

# Quiet-window duration for discover() (seconds).
_DISCOVER_QUIET_MS: float = 0.050

# Number of bytes to read per attempt from the transport.
_READ_CHUNK: int = 512


class Session:
    """Transaction engine for the RHSP protocol.

    Parameters
    ----------
    transport:
        The byte-level I/O object the session communicates through.
    retries:
        Number of *additional* attempts after the first timeout or
        checksum error.  With the default of 3 the session will try
        up to 4 times before raising :exc:`~rhsp.errors.RhspTimeoutError`.
    timeout:
        Per-attempt read deadline in seconds.  Each call to
        :meth:`transaction` reads from the transport until a valid
        response arrives or ``timeout`` seconds elapse.
    """

    def __init__(
        self,
        transport: "Transport",
        *,
        retries: int = 3,
        timeout: float = 1.0,
    ) -> None:
        self._transport = transport
        self._retries = retries
        self._timeout = timeout
        # msg_num: 1..255, never 0.  Starts at 1.
        self._msg_num: int = 1
        # DEKA base address; overwritten later by QueryInterface.
        self.deka_base: int = 0x1000
        # Shared frame parser (reused across transactions).
        self._parser: FrameParser = FrameParser()
        # Reentrant lock: serialises the public I/O methods so a background
        # heartbeat thread and the caller can share the session safely.
        self._lock: threading.RLock = threading.RLock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def transaction(
        self,
        command_name: str,
        dest: int,
        **fields: object,
    ) -> dict | None:
        """Execute a single RHSP request/response transaction.

        Looks up *command_name* in the catalogue, encodes ``**fields`` as
        the request payload, frames and sends it, then waits for the
        matching response.

        Thread-safe: acquires the internal RLock for the full duration so
        concurrent callers (e.g. a background heartbeat) are serialised.

        Parameters
        ----------
        command_name:
            Catalogue command name (e.g. ``"KeepAlive"``).
        dest:
            Destination module address (0..255).
        **fields:
            Keyword arguments matching the command's payload fields.

        Returns
        -------
        dict | None
            Decoded response dict for ``reply_kind == "rsp"`` commands;
            ``None`` for ``reply_kind == "ack"`` commands (ACK carries no
            semantic payload).

        Raises
        ------
        KeyError
            If *command_name* is not in the catalogue.
        NackError
            If the hub replies with a NACK frame.  NACKs are never retried.
        RhspTimeoutError
            If no valid response arrives within ``retries + 1`` attempts.
        """
        with self._lock:
            return self._transaction(command_name, dest, **fields)

    def _transaction(
        self,
        command_name: str,
        dest: int,
        **fields: object,
    ) -> dict | None:
        """Internal transaction implementation (lock must already be held)."""
        cmd = COMMANDS[command_name]
        ptype = runtime_packet_id(cmd, self.deka_base)
        payload = encode_payload(cmd.fields, fields) if fields else (
            encode_payload(cmd.fields, {}) if cmd.fields else b""
        )

        # Capture the msg_num we are about to send, then advance.
        sent_msg_num = self._msg_num
        self._advance_msg_num()

        frame = build_frame(dest, 0, sent_msg_num, 0, ptype, payload)

        # Expected response packet type for this command.
        expected_reply_id: int = cmd.reply_id

        # Discovery is exempt from ref_num correlation (multi-reply, broadcast).
        is_discovery = (ptype == _DISCOVERY_TYPE)

        last_exc: Exception = RhspTimeoutError(
            f"No response for {command_name!r} after {self._retries + 1} attempt(s)"
        )

        for attempt in range(self._retries + 1):
            # Write the frame on the first attempt; retransmit on subsequent.
            self._transport.write(frame)

            try:
                pkt = self._read_response(
                    expected_reply_id=expected_reply_id,
                    sent_msg_num=sent_msg_num,
                    is_discovery=is_discovery,
                )
            except (RhspTimeoutError, ChecksumError) as exc:
                last_exc = exc
                continue  # Retry on timeout or checksum error.

            # NACK: raise immediately, no retry.
            if pkt.packet_type == _NACK_TYPE:
                nack_code = _parse_nack_code(pkt)
                raise NackError(nack_code, NackCode.describe(nack_code))

            # ACK response.
            if cmd.reply_kind == "ack":
                return None

            # RSP response: decode and return.
            rsp_desc = RESPONSES_BY_ID.get(pkt.packet_type)
            if rsp_desc is None or not rsp_desc.fields:
                return {}
            return decode_payload(rsp_desc.fields, pkt.payload)

        raise last_exc

    def discover(self, dest: int = 0xFF) -> list[RawPacket]:
        """Send a Discovery broadcast and collect all replies.

        Sends one Discovery frame to *dest* (typically the broadcast
        address ``0xFF``) then accumulates incoming ``Discovery_RSP``
        packets until no bytes arrive for approximately 50 ms.

        Thread-safe: acquires the internal RLock for the full duration.

        Parameters
        ----------
        dest:
            Destination address for the Discovery frame (default ``0xFF``).

        Returns
        -------
        list[RawPacket]
            All ``Discovery_RSP`` packets received within the quiet window.
            Not subject to ``ref_num`` correlation (the spec permits any
            number of hubs to reply).

        Notes
        -----
        This method is exempt from the single-outstanding-transaction
        constraint: multiple replies are collected from a broadcast.
        """
        with self._lock:
            return self._discover(dest)

    def _discover(self, dest: int = 0xFF) -> list[RawPacket]:
        """Internal discover implementation (lock must already be held)."""
        cmd = COMMANDS["Discovery"]
        ptype = runtime_packet_id(cmd, self.deka_base)
        payload = b""

        sent_msg_num = self._msg_num
        self._advance_msg_num()

        frame = build_frame(dest, 0, sent_msg_num, 0, ptype, payload)
        self._transport.write(frame)

        # Collect replies until the quiet window expires.
        packets: list[RawPacket] = []
        deadline = time.monotonic() + self._timeout
        quiet_deadline = time.monotonic() + _DISCOVER_QUIET_MS

        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            chunk = self._transport.read(_READ_CHUNK)
            if chunk:
                for pkt in self._parser.feed(chunk):
                    if pkt.packet_type == _DISCOVERY_RSP_TYPE:
                        packets.append(pkt)
                # Reset the quiet window whenever bytes arrive.
                quiet_deadline = time.monotonic() + _DISCOVER_QUIET_MS
            else:
                if time.monotonic() >= quiet_deadline:
                    break
                # Yield the CPU briefly so the OS or a test thread can
                # inject data before we poll again.
                time.sleep(0.001)

        return packets

    # ==================================================================
    # Typed session methods — DEKA 0x00–0x30 + system commands
    # ==================================================================
    #
    # Each method wraps one ``transaction()`` call with typed parameters
    # and a typed return value.  Enum coercion, Q16 encode/decode, and
    # signed-integer reinterpretation are all handled here so that callers
    # receive clean Python types instead of raw bytes or dicts.
    #
    # The helper ``_int_field`` coerces a response field that the codec
    # may have returned as bytes (last-field variable-length rule) into a
    # proper Python int of the expected width and signedness.
    # ==================================================================

    # ------------------------------------------------------------------
    # System commands
    # ------------------------------------------------------------------

    def keep_alive(self, dest: int) -> None:
        """Send a KeepAlive command to reset the hub watchdog.

        The hub enters fail-safe (all outputs disabled) if no valid packet
        is received within ~2500 ms.

        Parameters:
            dest: Destination module address.
        """
        self.transaction("KeepAlive", dest=dest)

    def fail_safe(self, dest: int) -> None:
        """Immediately put the hub into fail-safe (all outputs disabled).

        Parameters:
            dest: Destination module address.
        """
        self.transaction("FailSafe", dest=dest)

    def query_interface(self, dest: int, name: str) -> tuple[int, int]:
        """Query the runtime base address for a named DEKA interface.

        When *name* is ``"DEKA"``, the hub returns the base packet-type ID
        for all DEKA commands (typically 0x1000).  This method also stores
        the result in ``self.deka_base`` so subsequent typed methods use the
        correct DEKA base.

        Parameters:
            dest: Destination module address.
            name: Interface name string (e.g. ``"DEKA"``).

        Returns:
            A ``(packet_id, num_values)`` tuple.
        """
        rsp = self.transaction("QueryInterface", dest=dest, interfaceName=name.encode("ascii"))
        assert rsp is not None
        packet_id = _int_field(rsp["packetID"], nbytes=2, signed=False)
        num_values = _int_field(rsp["numValues"], nbytes=2, signed=False)
        if name.upper() == "DEKA":
            self.deka_base = packet_id
        return packet_id, num_values

    def get_module_status(self, dest: int, clear: bool = False) -> "ModuleStatus":
        """Read (and optionally clear) the module status register.

        Parameters:
            dest:  Destination module address.
            clear: If ``True``, clear the latched status after reading.

        Returns:
            A :class:`~rhsp.devices.bulk.ModuleStatus` instance.
        """
        from rhsp.devices.bulk import ModuleStatus
        rsp = self.transaction("GetModuleStatus", dest=dest, clearStatus=1 if clear else 0)
        assert rsp is not None
        return ModuleStatus.from_response(rsp)

    def set_module_led_color(self, dest: int, r: int, g: int, b: int) -> None:
        """Set the module LED colour (``SetModuleLEDColor``, 0x7F0A).

        .. note::
            On hub firmware 1.8.2 this command is a no-op: the hub ACKs it but
            the LED never changes and ``GetModuleLEDColor`` keeps reading
            ``0,0,0``. Drive the LED with :meth:`set_module_led_pattern`
            instead (a solid colour is 16 identical steps). Kept for protocol
            completeness.

        Parameters:
            dest: Destination module address.
            r:    Red component (0–255).
            g:    Green component (0–255).
            b:    Blue component (0–255).
        """
        self.transaction(
            "SetModuleLEDColor", dest=dest, redPower=r, greenPower=g, bluePower=b
        )

    def set_module_led_pattern(
        self, dest: int, steps: list[tuple[int, int, int, int]]
    ) -> None:
        """Set the 16-step LED animation pattern.

        Each step is an ``(r, g, b, t)`` tuple where *t* is the step duration
        (0–255, firmware-defined units).  To show a solid non-blinking colour,
        pass all 16 steps with the same RGB and any non-zero *t*.

        Parameters:
            dest:  Destination module address.
            steps: 16-element list of ``(r, g, b, t)`` tuples.
        """
        if len(steps) != 16:
            raise ValueError(f"steps must have exactly 16 entries, got {len(steps)}")
        # Each rgbtStep is a 4-byte field whose wire order is [T, B, G, R]
        # (duration, blue, green, red) — matching REV's firmware and the FTC SDK
        # LynxSetModuleLEDPatternCommand. The codec serialises each field with
        # to_bytes(4, "little"), so the int's least-significant byte goes out
        # first: pack T in bits 0-7, B in 8-15, G in 16-23, R in 24-31.
        # Verified on hardware (fw 1.8.2): the [R,G,B,T] order is silently
        # ignored; [T,B,G,R] drives the LED correctly.
        fields: dict[str, int] = {}
        for i, (r, g, b, t) in enumerate(steps):
            fields[f"rgbtStep{i}"] = (
                (t & 0xFF) | ((b & 0xFF) << 8) | ((g & 0xFF) << 16) | ((r & 0xFF) << 24)
            )
        self.transaction("SetModuleLEDPattern", dest=dest, **fields)

    def get_module_led_color(self, dest: int) -> tuple[int, int, int]:
        """Read the current module LED colour.

        Parameters:
            dest: Destination module address.

        Returns:
            A ``(r, g, b)`` tuple with each component in 0–255.
        """
        rsp = self.transaction("GetModuleLEDColor", dest=dest)
        assert rsp is not None
        r = _int_field(rsp["redPower"], nbytes=1, signed=False)
        g = _int_field(rsp["greenPower"], nbytes=1, signed=False)
        b = _int_field(rsp["bluePower"], nbytes=1, signed=False)
        return r, g, b

    def read_version_string(self, dest: int) -> str:
        """Read the firmware version string from the hub.

        Parameters:
            dest: Destination module address.

        Returns:
            The ASCII version string (e.g. ``"HW: 20, Maj: 1, Min: 8, Eng: 2"``).
        """
        rsp = self.transaction("ReadVersionString", dest=dest)
        assert rsp is not None
        length = _int_field(rsp.get("length", 0), nbytes=1, signed=False)
        raw = rsp.get("versionString", b"")
        if isinstance(raw, (bytes, bytearray)):
            return raw[:length].decode("ascii", errors="replace")
        return str(raw)[:length]

    # ------------------------------------------------------------------
    # Motor commands (DEKA 0x08–0x18)
    # ------------------------------------------------------------------

    def set_motor_channel_mode(
        self,
        dest: int,
        channel: int,
        mode: MotorMode,
        float_at_zero: bool,
    ) -> None:
        """Set the run mode for a motor channel.

        The motor must be configured before it is enabled.

        Parameters:
            dest:          Destination module address.
            channel:       Motor channel (0–3).
            mode:          Control mode (see :class:`~rhsp.enums.MotorMode`).
            float_at_zero: ``True`` → float (coast) at zero power; ``False`` → brake.
        """
        self.transaction(
            "SetMotorChannelMode",
            dest=dest,
            motorChannel=channel,
            motorMode=int(mode),
            floatAtZero=1 if float_at_zero else 0,
        )

    def get_motor_channel_mode(
        self, dest: int, channel: int
    ) -> tuple[MotorMode, bool]:
        """Read the current mode for a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            A ``(mode, float_at_zero)`` tuple.
        """
        rsp = self.transaction("GetMotorChannelMode", dest=dest, motorChannel=channel)
        assert rsp is not None
        mode = MotorMode(_int_field(rsp["motorChannelMode"], nbytes=1, signed=False))
        float_at_zero = bool(_int_field(rsp["floatAtZero"], nbytes=1, signed=False))
        return mode, float_at_zero

    def set_motor_channel_enable(
        self, dest: int, channel: int, enabled: bool
    ) -> None:
        """Enable or disable a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).
            enabled: ``True`` to enable; ``False`` to disable.
        """
        self.transaction(
            "SetMotorChannelEnable",
            dest=dest,
            motorChannel=channel,
            enabled=1 if enabled else 0,
        )

    def get_motor_channel_enable(self, dest: int, channel: int) -> bool:
        """Read the enable state of a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            ``True`` if the motor is enabled.
        """
        rsp = self.transaction("GetMotorChannelEnable", dest=dest, motorChannel=channel)
        assert rsp is not None
        return bool(_int_field(rsp["enabled"], nbytes=1, signed=False))

    def set_motor_channel_current_alert_level(
        self, dest: int, channel: int, limit_ma: int
    ) -> None:
        """Set the current alert threshold for a motor channel.

        Parameters:
            dest:     Destination module address.
            channel:  Motor channel (0–3).
            limit_ma: Current limit in milliamps.
        """
        self.transaction(
            "SetMotorChannelCurrentAlertLevel",
            dest=dest,
            motorChannel=channel,
            currentLimit=limit_ma,
        )

    def get_motor_channel_current_alert_level(
        self, dest: int, channel: int
    ) -> int:
        """Read the current alert threshold for a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            Current limit in milliamps.
        """
        rsp = self.transaction(
            "GetMotorChannelCurrentAlertLevel", dest=dest, motorChannel=channel
        )
        assert rsp is not None
        return _int_field(rsp["currentLimit"], nbytes=2, signed=False)

    def reset_motor_encoder(self, dest: int, channel: int) -> None:
        """Reset the encoder count for a motor channel to zero.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).
        """
        self.transaction("ResetMotorEncoder", dest=dest, motorChannel=channel)

    def set_motor_constant_power(
        self, dest: int, channel: int, power: int
    ) -> None:
        """Set the open-loop power level for a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).
            power:   Power level in range [−32767, 32767].
        """
        self.transaction(
            "SetMotorConstantPower",
            dest=dest,
            motorChannel=channel,
            powerLevel=power,
        )

    def get_motor_constant_power(self, dest: int, channel: int) -> int:
        """Read the open-loop power level for a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            Power level in range [−32767, 32767].
        """
        rsp = self.transaction("GetMotorConstantPower", dest=dest, motorChannel=channel)
        assert rsp is not None
        return _int_field(rsp["powerLevel"], nbytes=2, signed=True)

    def set_motor_target_velocity(
        self, dest: int, channel: int, velocity: int
    ) -> None:
        """Set the closed-loop target velocity for a motor channel.

        Parameters:
            dest:     Destination module address.
            channel:  Motor channel (0–3).
            velocity: Target velocity in encoder counts per second (signed).
        """
        self.transaction(
            "SetMotorTargetVelocity",
            dest=dest,
            motorChannel=channel,
            velocity=velocity,
        )

    def get_motor_target_velocity(self, dest: int, channel: int) -> int:
        """Read the target velocity for a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            Velocity in encoder counts per second (signed 16-bit).
        """
        rsp = self.transaction(
            "GetMotorTargetVelocity", dest=dest, motorChannel=channel
        )
        assert rsp is not None
        return _int_field(rsp["velocity"], nbytes=2, signed=True)

    def set_motor_target_position(
        self, dest: int, channel: int, position: int, tolerance: int
    ) -> None:
        """Set the closed-loop target position for a motor channel.

        Parameters:
            dest:      Destination module address.
            channel:   Motor channel (0–3).
            position:  Target encoder count (signed 32-bit).
            tolerance: Acceptable position error in encoder counts.
        """
        self.transaction(
            "SetMotorTargetPosition",
            dest=dest,
            motorChannel=channel,
            position=position,
            atTargetTolerance=tolerance,
        )

    def get_motor_target_position(
        self, dest: int, channel: int
    ) -> tuple[int, int]:
        """Read the target position for a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            A ``(target_position, at_target_tolerance)`` tuple.
        """
        rsp = self.transaction(
            "GetMotorTargetPosition", dest=dest, motorChannel=channel
        )
        assert rsp is not None
        pos = _int_field(rsp["targetPosition"], nbytes=4, signed=True)
        tol = _int_field(rsp["atTargetTolerance"], nbytes=2, signed=False)
        return pos, tol

    def get_motor_at_target(self, dest: int, channel: int) -> bool:
        """Check whether a motor channel is at its target position.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            ``True`` if the motor is within tolerance of its target.
        """
        rsp = self.transaction("GetMotorAtTarget", dest=dest, motorChannel=channel)
        assert rsp is not None
        return bool(_int_field(rsp["atTarget"], nbytes=1, signed=False))

    def get_motor_encoder_position(self, dest: int, channel: int) -> int:
        """Read the encoder position for a motor channel.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).

        Returns:
            Signed 32-bit encoder count.
        """
        rsp = self.transaction(
            "GetMotorEncoderPosition", dest=dest, motorChannel=channel
        )
        assert rsp is not None
        return _int_field(rsp["currentPosition"], nbytes=4, signed=True)

    def set_motor_pid_coefficients(
        self,
        dest: int,
        channel: int,
        mode: int,
        p: float,
        i: float,
        d: float,
    ) -> None:
        """Set the PID coefficients for a motor channel and closed-loop mode.

        Values are transmitted as Q16 fixed-point (multiplied by 65536).

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).
            mode:    Closed-loop mode (see :class:`~rhsp.enums.ClosedLoopMode`).
            p:       Proportional coefficient.
            i:       Integral coefficient.
            d:       Derivative coefficient.
        """
        self.transaction(
            "SetMotorPIDCoefficients",
            dest=dest,
            motorChannel=channel,
            mode=mode,
            p=p,
            i=i,
            d=d,
        )

    def get_motor_pid_coefficients(
        self, dest: int, channel: int, mode: int
    ) -> tuple[float, float, float]:
        """Read the PID coefficients for a motor channel and closed-loop mode.

        Parameters:
            dest:    Destination module address.
            channel: Motor channel (0–3).
            mode:    Closed-loop mode (see :class:`~rhsp.enums.ClosedLoopMode`).

        Returns:
            A ``(p, i, d)`` tuple of floating-point Q16 coefficients.
        """
        rsp = self.transaction(
            "GetMotorPIDCoefficients", dest=dest, motorChannel=channel, mode=mode
        )
        assert rsp is not None
        p = _q16_field(rsp["p"])
        i = _q16_field(rsp["i"])
        d = _q16_field(rsp["d"])
        return p, i, d

    # ------------------------------------------------------------------
    # PWM commands (DEKA 0x19–0x1E)
    # ------------------------------------------------------------------

    def set_pwm_configuration(
        self, dest: int, channel: int, frame_period: int
    ) -> None:
        """Set the PWM frame period for a channel.

        Parameters:
            dest:         Destination module address.
            channel:      PWM channel.
            frame_period: Frame period in microseconds (e.g. 20000 for 50 Hz).
        """
        self.transaction(
            "SetPWMConfiguration",
            dest=dest,
            pwmChannel=channel,
            framePeriod=frame_period,
        )

    def get_pwm_configuration(self, dest: int, channel: int) -> int:
        """Read the PWM frame period for a channel.

        Parameters:
            dest:    Destination module address.
            channel: PWM channel.

        Returns:
            Frame period in microseconds.
        """
        rsp = self.transaction("GetPWMConfiguration", dest=dest, pwmChannel=channel)
        assert rsp is not None
        return _int_field(rsp["framePeriod"], nbytes=2, signed=False)

    def set_pwm_pulse_width(
        self, dest: int, channel: int, pulse_width: int
    ) -> None:
        """Set the PWM pulse width for a channel.

        Parameters:
            dest:        Destination module address.
            channel:     PWM channel.
            pulse_width: Pulse width in microseconds.
        """
        self.transaction(
            "SetPWMPulseWidth",
            dest=dest,
            pwmChannel=channel,
            pulseWidth=pulse_width,
        )

    def get_pwm_pulse_width(self, dest: int, channel: int) -> int:
        """Read the PWM pulse width for a channel.

        Parameters:
            dest:    Destination module address.
            channel: PWM channel.

        Returns:
            Pulse width in microseconds (2-byte unsigned integer).
        """
        rsp = self.transaction("GetPWNPulseWidth", dest=dest, pwmChannel=channel)
        assert rsp is not None
        # P7-g fix: response is 2 bytes (protocol.json updated accordingly).
        return _int_field(rsp["pulseWidth"], nbytes=2, signed=False)

    def set_pwm_enable(self, dest: int, channel: int, enabled: bool) -> None:
        """Enable or disable a PWM channel.

        Parameters:
            dest:    Destination module address.
            channel: PWM channel.
            enabled: ``True`` to enable; ``False`` to disable.
        """
        self.transaction(
            "SetPWMEnable",
            dest=dest,
            pwmChannel=channel,
            enable=1 if enabled else 0,
        )

    def get_pwm_enable(self, dest: int, channel: int) -> bool:
        """Read the enable state of a PWM channel.

        Parameters:
            dest:    Destination module address.
            channel: PWM channel.

        Returns:
            ``True`` if the PWM channel is enabled.
        """
        rsp = self.transaction("GetPWMEnable", dest=dest, pwmChannel=channel)
        assert rsp is not None
        return bool(_int_field(rsp["enabled"], nbytes=1, signed=False))

    # ------------------------------------------------------------------
    # Servo commands (DEKA 0x1F–0x24)
    # ------------------------------------------------------------------

    def set_servo_configuration(
        self, dest: int, channel: int, frame_period: int
    ) -> None:
        """Set the servo frame period for a channel.

        Parameters:
            dest:         Destination module address.
            channel:      Servo channel (0–5).
            frame_period: Frame period in microseconds (typically 20000 for 50 Hz).
        """
        self.transaction(
            "SetServoConfiguration",
            dest=dest,
            servoChannel=channel,
            framePeriod=frame_period,
        )

    def get_servo_configuration(self, dest: int, channel: int) -> int:
        """Read the servo frame period for a channel.

        Parameters:
            dest:    Destination module address.
            channel: Servo channel (0–5).

        Returns:
            Frame period in microseconds.
        """
        rsp = self.transaction("GetServoConfiguration", dest=dest, servoChannel=channel)
        assert rsp is not None
        return _int_field(rsp["framePeriod"], nbytes=2, signed=False)

    def set_servo_pulse_width(
        self, dest: int, channel: int, pulse_width: int
    ) -> None:
        """Set the servo pulse width for a channel.

        Parameters:
            dest:        Destination module address.
            channel:     Servo channel (0–5).
            pulse_width: Pulse width in microseconds (500–2500).
        """
        self.transaction(
            "SetServoPulseWidth",
            dest=dest,
            servoChannel=channel,
            pulseWidth=pulse_width,
        )

    def get_servo_pulse_width(self, dest: int, channel: int) -> int:
        """Read the servo pulse width for a channel.

        Parameters:
            dest:    Destination module address.
            channel: Servo channel (0–5).

        Returns:
            Pulse width in microseconds.
        """
        rsp = self.transaction("GetServoPulseWidth", dest=dest, servoChannel=channel)
        assert rsp is not None
        return _int_field(rsp["pulseWidth"], nbytes=2, signed=False)

    def set_servo_enable(self, dest: int, channel: int, enabled: bool) -> None:
        """Enable or disable a servo channel.

        Parameters:
            dest:    Destination module address.
            channel: Servo channel (0–5).
            enabled: ``True`` to enable; ``False`` to disable.
        """
        self.transaction(
            "SetServoEnable",
            dest=dest,
            servoChannel=channel,
            enable=1 if enabled else 0,
        )

    def get_servo_enable(self, dest: int, channel: int) -> bool:
        """Read the enable state of a servo channel.

        Parameters:
            dest:    Destination module address.
            channel: Servo channel (0–5).

        Returns:
            ``True`` if the servo channel is enabled.
        """
        rsp = self.transaction("GetServoEnable", dest=dest, servoChannel=channel)
        assert rsp is not None
        return bool(_int_field(rsp["enabled"], nbytes=1, signed=False))

    # ------------------------------------------------------------------
    # DIO commands (DEKA 0x01–0x06)
    # ------------------------------------------------------------------

    def set_dio_direction(
        self, dest: int, pin: int, output: bool
    ) -> None:
        """Set the direction of a digital I/O pin.

        Parameters:
            dest:   Destination module address.
            pin:    DIO pin (0–7).
            output: ``True`` → output; ``False`` → input.
        """
        self.transaction(
            "SetDIODirection",
            dest=dest,
            dioPin=pin,
            directionOutput=1 if output else 0,
        )

    def get_dio_direction(self, dest: int, pin: int) -> bool:
        """Read the direction of a digital I/O pin.

        Parameters:
            dest: Destination module address.
            pin:  DIO pin (0–7).

        Returns:
            ``True`` if the pin is configured as output.
        """
        rsp = self.transaction("GetDIODirection", dest=dest, dioPin=pin)
        assert rsp is not None
        # P7-b fix: return the decoded value (vendor bug was that it never returned).
        return bool(_int_field(rsp["directionOutput"], nbytes=1, signed=False))

    def set_single_dio_output(
        self, dest: int, pin: int, value: bool
    ) -> None:
        """Set the output value of a single digital I/O pin.

        Parameters:
            dest:  Destination module address.
            pin:   DIO pin (0–7).
            value: ``True`` for logic high; ``False`` for logic low.
        """
        self.transaction(
            "SetSingleDIOOutput",
            dest=dest,
            dioPin=pin,
            value=1 if value else 0,
        )

    def get_single_dio_input(self, dest: int, pin: int) -> bool:
        """Read the input value of a single digital I/O pin.

        Parameters:
            dest: Destination module address.
            pin:  DIO pin (0–7).

        Returns:
            ``True`` for logic high.
        """
        rsp = self.transaction("GetSingleDIOInput", dest=dest, dioPin=pin)
        assert rsp is not None
        return bool(_int_field(rsp["inputValue"], nbytes=1, signed=False))

    def set_all_dio_outputs(self, dest: int, mask: int) -> None:
        """Set all digital I/O output pins at once via a bitmask.

        Parameters:
            dest: Destination module address.
            mask: 8-bit bitmask; bit N sets pin N (pins must already be outputs).
        """
        self.transaction("SetAllDIOOutputs", dest=dest, values=mask)

    def get_all_dio_inputs(self, dest: int) -> int:
        """Read all digital I/O input pins as a bitmask.

        Parameters:
            dest: Destination module address.

        Returns:
            8-bit bitmask; bit N reflects the state of pin N.
        """
        rsp = self.transaction("GetAllDIOInputs", dest=dest)
        assert rsp is not None
        return _int_field(rsp["inputValues"], nbytes=1, signed=False)

    # ------------------------------------------------------------------
    # ADC (DEKA 0x07)
    # ------------------------------------------------------------------

    def get_adc(self, dest: int, channel: int, raw: bool = False) -> int:
        """Read an ADC channel.

        Parameters:
            dest:    Destination module address.
            channel: ADC channel (see :class:`~rhsp.enums.ADCChannel`).
            raw:     If ``True``, return raw counts; otherwise engineering units.

        Returns:
            ADC reading as a plain integer (mV or mA depending on channel,
            or raw counts when *raw* is ``True``).
        """
        rsp = self.transaction(
            "GetADC", dest=dest, adcChannel=channel, rawMode=1 if raw else 0
        )
        assert rsp is not None
        return _int_field(rsp["adcValue"], nbytes=2, signed=False)

    # ------------------------------------------------------------------
    # I2C commands (DEKA 0x25–0x2F)
    # ------------------------------------------------------------------

    def i2c_write_single_byte(
        self, dest: int, i2c_ch: int, address: int, byte: int
    ) -> None:
        """Write a single byte to an I2C device.

        Parameters:
            dest:    Destination module address.
            i2c_ch:  I2C channel (0–3).
            address: 7-bit I2C device address.
            byte:    Byte value to write.
        """
        self.transaction(
            "I2CWriteSingleByte",
            dest=dest,
            i2cChannel=i2c_ch,
            slaveAddress=address,
            byteToWrite=byte,
        )

    def i2c_write_multiple_bytes(
        self, dest: int, i2c_ch: int, address: int, data: bytes
    ) -> None:
        """Write multiple bytes to an I2C device.

        Parameters:
            dest:    Destination module address.
            i2c_ch:  I2C channel (0–3).
            address: 7-bit I2C device address.
            data:    Bytes to write (up to 121 bytes).
        """
        self.transaction(
            "I2CWriteMultipleBytes",
            dest=dest,
            i2cChannel=i2c_ch,
            slaveAddress=address,
            numBytes=len(data),
            bytesToWrite=bytes(data),
        )

    def i2c_read_single_byte(
        self, dest: int, i2c_ch: int, address: int
    ) -> None:
        """Initiate a single-byte I2C read.  The result is fetched via a status query.

        Parameters:
            dest:    Destination module address.
            i2c_ch:  I2C channel (0–3).
            address: 7-bit I2C device address.
        """
        self.transaction(
            "I2CReadSingleByte",
            dest=dest,
            i2cChannel=i2c_ch,
            slaveAddress=address,
        )

    def i2c_read_multiple_bytes(
        self, dest: int, i2c_ch: int, address: int, num_bytes: int
    ) -> None:
        """Initiate a multi-byte I2C read.  The result is fetched via a status query.

        Parameters:
            dest:      Destination module address.
            i2c_ch:    I2C channel (0–3).
            address:   7-bit I2C device address.
            num_bytes: Number of bytes to read.
        """
        self.transaction(
            "I2CReadMultipleBytes",
            dest=dest,
            i2cChannel=i2c_ch,
            slaveAddress=address,
            numBytes=num_bytes,
        )

    def i2c_read_status_query(
        self, dest: int, i2c_ch: int
    ) -> tuple[int, bytes]:
        """Query the result of a previous I2C read operation.

        Parameters:
            dest:   Destination module address.
            i2c_ch: I2C channel (0–3).

        Returns:
            A ``(status, data)`` tuple where *status* is the I2C status byte
            and *data* is the read bytes.
        """
        rsp = self.transaction("I2CReadStatusQuery", dest=dest, i2cChannel=i2c_ch)
        assert rsp is not None
        status = _int_field(rsp["i2cStatus"], nbytes=1, signed=False)
        num_bytes = _int_field(rsp.get("byteRead", 0), nbytes=1, signed=False)
        raw = rsp.get("payloadBytes", b"")
        if isinstance(raw, (bytes, bytearray)):
            data = bytes(raw[:num_bytes])
        else:
            data = b""
        return status, data

    def i2c_write_status_query(
        self, dest: int, i2c_ch: int
    ) -> tuple[int, int]:
        """Query the status of a previous I2C write operation.

        Parameters:
            dest:   Destination module address.
            i2c_ch: I2C channel (0–3).

        Returns:
            A ``(status, num_bytes)`` tuple.
        """
        rsp = self.transaction("I2CWriteStatusQuery", dest=dest, i2cChannel=i2c_ch)
        assert rsp is not None
        status = _int_field(rsp["i2cStatus"], nbytes=1, signed=False)
        num_bytes = _int_field(rsp["numBytes"], nbytes=1, signed=False)
        return status, num_bytes

    def i2c_configure_channel(
        self, dest: int, i2c_ch: int, speed_code: int
    ) -> None:
        """Configure the speed of an I2C channel.

        Parameters:
            dest:       Destination module address.
            i2c_ch:     I2C channel (0–3).
            speed_code: Speed (see :class:`~rhsp.enums.I2CSpeedCode`).
        """
        self.transaction(
            "I2CConfigureChannel",
            dest=dest,
            i2cChannel=i2c_ch,
            speedCode=speed_code,
        )

    def i2c_configure_query(self, dest: int, i2c_ch: int) -> int:
        """Query the current speed configuration of an I2C channel.

        This method uses the response id from ``RESPONSES_BY_ID`` for
        ``I2CConfigureQuery_RSP`` (P7-e fix).

        Parameters:
            dest:   Destination module address.
            i2c_ch: I2C channel (0–3).

        Returns:
            The configured speed code (see :class:`~rhsp.enums.I2CSpeedCode`).
        """
        rsp = self.transaction("I2CConfigureQuery", dest=dest, i2cChannel=i2c_ch)
        assert rsp is not None
        return _int_field(rsp["speedCode"], nbytes=1, signed=False)

    # ------------------------------------------------------------------
    # Bulk data (DEKA 0x00)
    # ------------------------------------------------------------------

    def get_bulk_input_data(self, dest: int) -> "BulkInputData":
        """Read all digital/analog/motor/servo/I2C state in one transaction.

        Some hub firmware versions return a shortened payload (e.g. 34 bytes
        instead of the full 141 bytes in the current spec).  This method
        zero-pads the raw payload to the expected length before decoding, so
        older firmware hubs return partial data (with zeros for missing fields)
        rather than raising a ``ValueError``.

        Thread-safe: acquires the internal RLock for the full duration.

        Parameters:
            dest: Destination module address.

        Returns:
            A :class:`~rhsp.devices.bulk.BulkInputData` instance with all
            fields decoded to proper Python types.  Fields not present in the
            hub response are zero.
        """
        with self._lock:
            return self._get_bulk_input_data(dest)

    def _get_bulk_input_data(self, dest: int) -> "BulkInputData":
        """Internal bulk-input implementation (lock must already be held)."""
        from rhsp.devices.bulk import BulkInputData

        # Compute the full expected payload length from the catalogue fields.
        rsp_desc = RESPONSES_BY_ID.get(COMMANDS["GetBulkInputData"].reply_id)
        if rsp_desc and rsp_desc.fields:
            last_field = rsp_desc.fields[-1]
            expected_len = last_field.offset + last_field.nbytes
        else:
            expected_len = 0

        # Send the frame and collect the raw packet directly so we can inspect
        # the payload length before passing it to decode_payload.
        cmd = COMMANDS["GetBulkInputData"]
        ptype = runtime_packet_id(cmd, self.deka_base)
        sent_msg_num = self._msg_num
        self._advance_msg_num()
        frame = build_frame(dest, 0, sent_msg_num, 0, ptype, b"")

        last_exc: Exception = RhspTimeoutError(
            f"No response for 'GetBulkInputData' after {self._retries + 1} attempt(s)"
        )
        for attempt in range(self._retries + 1):
            self._transport.write(frame)
            try:
                pkt = self._read_response(
                    expected_reply_id=cmd.reply_id,
                    sent_msg_num=sent_msg_num,
                    is_discovery=False,
                )
            except (RhspTimeoutError, ChecksumError) as exc:
                last_exc = exc
                continue

            if pkt.packet_type == _NACK_TYPE:
                nack_code = _parse_nack_code(pkt)
                raise NackError(nack_code, NackCode.describe(nack_code))

            # Pad truncated payload so decode_payload does not raise.
            raw_payload = pkt.payload
            if expected_len > 0 and len(raw_payload) < expected_len:
                raw_payload = raw_payload + b"\x00" * (expected_len - len(raw_payload))

            if rsp_desc is None or not rsp_desc.fields:
                rsp: dict = {}
            else:
                rsp = decode_payload(rsp_desc.fields, raw_payload)
            return BulkInputData.from_response(rsp)

        raise last_exc

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _advance_msg_num(self) -> None:
        """Increment ``_msg_num`` with wrap-around; never produces 0."""
        self._msg_num = (self._msg_num % 255) + 1

    def _read_response(
        self,
        *,
        expected_reply_id: int,
        sent_msg_num: int,
        is_discovery: bool,
    ) -> RawPacket:
        """Read from the transport until an acceptable packet is found.

        An acceptable packet is one where:
        - ``packet_type == expected_reply_id``  (or ``== _NACK_TYPE``)
        - ``ref_num == sent_msg_num``           (unless *is_discovery*)

        Packets that do not match are silently discarded; the loop
        continues reading until the timeout expires.

        Raises
        ------
        RhspTimeoutError
            When ``self._timeout`` seconds elapse without a matching packet.
        ChecksumError
            When a corrupt frame is detected (propagated from
            :class:`~rhsp.framing.FrameParser`); the caller retries.
        """
        deadline = time.monotonic() + self._timeout

        while time.monotonic() < deadline:
            chunk = self._transport.read(_READ_CHUNK)
            if not chunk:
                # Yield the CPU briefly so a FakeHub background thread (in tests)
                # or the OS serial buffer can produce data before we poll again.
                time.sleep(0.001)
                continue  # Transport returned nothing; keep polling.

            for pkt in self._parser.feed(chunk):
                # Check if this is a NACK intended for us.
                if pkt.packet_type == _NACK_TYPE:
                    if is_discovery or pkt.ref_num == sent_msg_num:
                        return pkt
                    # NACK for a different msg_num — discard.
                    continue

                # Check if this is the expected response type.
                if pkt.packet_type != expected_reply_id:
                    continue  # Unexpected packet type — discard.

                # Validate ref_num correlation (Discovery is exempt).
                if is_discovery or pkt.ref_num == sent_msg_num:
                    return pkt
                # ref_num mismatch — discard and keep waiting.

        raise RhspTimeoutError(
            f"Timed out waiting for packet type 0x{expected_reply_id:04X}"
        )


def _parse_nack_code(pkt: RawPacket) -> int:
    """Extract the NACK code integer from a raw NACK packet payload.

    The NACK response payload is a single byte ``nackCode``.  We parse
    it directly from the raw bytes to avoid the codec's variable-length
    trailing-field rule (which returns bytes rather than an int).
    """
    if pkt.payload:
        return pkt.payload[0]
    return 0


# ---------------------------------------------------------------------------
# Typed-method codec helpers
# ---------------------------------------------------------------------------


def _int_field(value: object, *, nbytes: int, signed: bool) -> int:
    """Coerce a response field value to a Python int.

    The codec's "last field is variable-length" rule returns raw ``bytes``
    for the trailing field in any single-field response (e.g. ``GetADC``,
    ``GetMotorEncoderPosition``).  Typed session methods call this helper
    to normalise both the ``bytes`` case and the already-decoded ``int``
    case into a clean Python integer of the expected width and signedness.

    Parameters:
        value:  Value from the decoded response dict.
        nbytes: Expected field width in bytes.
        signed: Whether to interpret as two's-complement signed.
    """
    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)[:nbytes].ljust(nbytes, b"\x00")
        return int.from_bytes(raw, "little", signed=signed)
    return int(value)


def _q16_field(value: object) -> float:
    """Coerce a Q16 fixed-point response field to a Python float.

    When the codec decodes a ``fixed_point=65536`` field that is *not* the
    last field, it already returns a float.  When it is the last field, the
    codec returns raw bytes; this helper handles both cases.

    Parameters:
        value: Value from the decoded response dict (float or bytes).
    """
    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)[:4].ljust(4, b"\x00")
        int_val = int.from_bytes(raw, "little", signed=True)
        return int_val / 65536.0
    return float(value)
