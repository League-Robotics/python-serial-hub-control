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

import math
import threading
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Callable, Mapping

from rhsp.enums import MotorMode
from rhsp.devices.bulk import BulkInputData

if TYPE_CHECKING:
    from rhsp.hub import Hub
    from rhsp.devices.motor import Motor

__all__ = [
    "VelocityController",
    "HubVelocityController",
    "RatioDrive",
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


# ---------------------------------------------------------------------------
# Outer-loop coordinator — RatioDrive
# ---------------------------------------------------------------------------

_EPSILON: float = 1e-9  # guard against division by zero in shortfall calc


class RatioDrive:
    """Ratio-preserving multi-wheel velocity coordinator with a cap-to-slowest governor.

    ``RatioDrive`` manages a set of :class:`HubVelocityController` instances
    (one per wheel channel) and applies a deterministic governor that keeps the
    commanded ratio between wheels exact at all times.  A single float scale
    ``g`` (counts/s equivalent) is maintained; each wheel is commanded
    ``clamp_int16(round(g * w_i))`` where ``w_i`` is that wheel's weight.

    Governor algorithm (one step):
        1. Read measured velocities ``v_i`` from ``BulkInputData``.
        2. Normalise: ``n_i = sign(w_i) * v_i / |w_i|`` (deadband applied first).
        3. Compute shortfall per saturated wheel; manage saturation state with
           hysteresis (``sat_margin`` to enter, ``sat_release_margin`` to leave).
        4. Determine ceiling:

           - If any wheel is saturated: ``ceiling = min(n_i for saturated wheels)``
             (cap-to-slowest).  Record the min as ``_learned_bottleneck``.
           - No prior saturation (``_learned_bottleneck is None``): ``ceiling = S``
             — spin-up path, full recovery_accel ramp.
           - Post-saturation, settle window active: ``ceiling = g`` (hold steady).
           - Post-saturation, settled: ``ceiling = min(g + probe_step, probe_target)``
             where ``probe_target`` is capped by any wheel with normalised speed
             significantly below ``g`` (raw-cap, bypasses sat_settle_s hysteresis
             to prevent g from silently climbing past the physical limit).

        5. Slew-limit ``g`` toward ceiling: down by ``max_accel * dt``, up by
           ``recovery_accel * dt``; clamp to ``[min_scale, S]``.
        6. Emit targets to each controller.

    Threading model:
        A daemon thread (``rhsp-ratiodrive``) calls ``step()`` at ``rate_hz``.
        All mutable state is guarded by an internal ``RLock``.  The thread
        coexists safely with the hub's keep-alive heartbeat.

    Parameters:
        hub:               The connected :class:`~rhsp.hub.Hub`.
        weights:           Mapping of channel index → signed weight (float).
                           Channels with weight 0 are excluded from the governor.
        rate_hz:           Daemon-thread tick rate (Hz).  Default 50.
        max_accel:         Maximum downward slew rate (counts/s per second).
        recovery_accel:    Maximum upward slew rate; defaults to ``max_accel``.
        sat_margin:        Shortfall fraction to enter saturation state.
        sat_release_margin: Shortfall fraction below which saturation clears.
        sat_settle_s:      Seconds the shortfall must continuously exceed
                           ``sat_margin`` before a wheel is declared saturated.
                           Prevents transient acceleration lag during spin-up
                           from triggering saturation.  Default 0.3 s.
        min_scale:         Floor for ``g`` (clamp after slew).
        deadband:          Measured velocity magnitude below which the reading
                           is treated as zero for normalisation.
        cpr:               Encoder counts per revolution.  Required for
                           :meth:`set_speed_rpm`.
        velocity_pid:      Optional ``(p, i, d)`` tuple pushed to each wheel's
                           motor PID during :meth:`start`.
        on_error:          Callable invoked with an ``Exception`` on transient
                           per-iteration errors.  If ``None``, errors are
                           suppressed silently.
        probe_step:        Maximum upward step applied to ``g`` per tick when no
                           wheel is saturated.  Acts as a slew-rate limit on
                           recovery so that ``g`` probes upward gradually rather
                           than jumping directly to ``S``.  This prevents the
                           limit-cycle where ``g`` bounces between the bottleneck
                           speed and ``S`` indefinitely.  Default 10 counts/s per
                           tick (≈ 500 counts/s over 1 s at 50 Hz).
        probe_settle_ticks: Number of governor ticks to hold ``g`` steady after a
                           cap-down event (velocity saturation triggered) before
                           the upward probe resumes.  This deadband window prevents
                           the probe from immediately re-triggering saturation at
                           the bottleneck equilibrium.  Default 5 ticks (≈ 100 ms
                           at 50 Hz).
        current_limit_ma:  Per-motor current limit in milliamps.  When set,
                           the governor reads ``Motor.get_current_ma()`` each Nth
                           tick and treats any motor exceeding the limit as the
                           bottleneck, capping the common scale ``g`` using the
                           same cap-to-slowest machinery so the commanded ratio
                           stays exact.  ``None`` (the default) disables current
                           sensing entirely — the governor is unchanged.

                           **Transaction-budget note**: ``GetADC`` costs ~16 ms
                           per call on fw 1.8.2.  Reading 2 motors every tick at
                           50 Hz (20 ms) would overflow the period.  Instead,
                           current is sampled every ``_cur_sample_every`` ticks
                           (default 3, giving ~16 Hz effective current sampling
                           at 50 Hz governor rate).  The last sampled values are
                           reused between reads.  The keep-alive heartbeat (100 ms
                           period) is never starved because the maximum burst is
                           2 × 16 ms ≈ 32 ms, which is the sample cost only once
                           every 3 × 20 ms = 60 ms.
    """

    def __init__(
        self,
        hub: "Hub",
        weights: Mapping[int, float],
        *,
        rate_hz: float = 50.0,
        max_accel: float = 6000.0,
        recovery_accel: float | None = None,
        sat_margin: float = 0.15,
        sat_release_margin: float = 0.07,
        sat_settle_s: float = 0.3,
        min_scale: float = 0.0,
        deadband: int = 20,
        cpr: float | None = None,
        velocity_pid: tuple[float, float, float] | None = None,
        on_error: Callable[[Exception], None] | None = None,
        current_limit_ma: int | None = None,
        probe_step: float = 10.0,
        probe_settle_ticks: int = 5,
    ) -> None:
        self._hub = hub
        self._rate_hz = rate_hz
        self._max_accel = max_accel
        self._recovery_accel = recovery_accel if recovery_accel is not None else max_accel
        self._sat_margin = sat_margin
        self._sat_release_margin = sat_release_margin
        self._sat_settle_s = sat_settle_s
        self._min_scale = min_scale
        self._deadband = deadband
        self._cpr = cpr
        self._velocity_pid = velocity_pid
        self._on_error = on_error
        # Damped upward probe parameters (fix for 003-009 limit-cycle).
        # probe_step: maximum upward step in g per tick when not saturated.
        # probe_settle_ticks: ticks to hold g steady after a cap-down event.
        self._probe_step: float = probe_step
        self._probe_settle_ticks: int = probe_settle_ticks
        # Countdown counter; decremented each tick when not saturated and settled.
        # Set to probe_settle_ticks whenever velocity saturation fires.
        self._probe_settle_counter: int = 0
        # Learned bottleneck normalised speed (counts/s): the minimum n_i observed
        # while velocity saturation was active.  Set to None initially (spin-up /
        # no prior saturation) so that ceiling = S and g ramps normally.
        # Once set, used to gate the probe path: raw_cap limits the ceiling to the
        # observed physical speed, preventing silent climb above the bottleneck.
        # Can only decrease (tighter bottleneck recorded on each saturation event).
        # NOT actively cleared — the raw_cap mechanism drives gradual recovery when
        # the bottleneck physically clears (n_i meets target → raw_cap = probe_target).
        self._learned_bottleneck: float | None = None

        # Current-limit governor (opt-in via current_limit_ma).
        self._current_limit_ma: int | None = current_limit_ma
        # Hysteresis: engage limit ceiling when current > limit * (1 + hys),
        # release when current < limit * (1 - hys).
        self._cur_hys: float = 0.05
        # Sample current every N ticks to stay within the 20 ms tick budget.
        # GetADC costs ~16 ms each; 2 motors × 16 ms = 32 ms per sample tick.
        # At 50 Hz (20 ms period) we sample every 3rd tick → effective ~16 Hz
        # current reads, keeping the average per-tick overhead to 32/3 ≈ 11 ms.
        self._cur_sample_every: int = 3
        self._cur_tick_counter: int = 0
        # Last sampled current per channel (mA); reused between sample ticks.
        self._last_current_ma: dict[int, int] = {}
        # Per-channel current-saturation flags (hysteresis state).
        self._cur_sat_flags: dict[int, bool] = {}

        # Internal RLock guards all mutable governor state.
        self._lock = threading.RLock()

        # Governor state.
        self._scale: float = 0.0
        self._target_scale: float = 0.0
        self._weights: dict[int, float] = dict(weights)
        self._commanded_targets: dict[int, int] = {}
        self._measured: dict[int, int] = {}
        self._normalized_actual: dict[int, float] = {}
        self._saturated: bool = False
        # Per-wheel saturation flags (keyed by channel).
        self._sat_flags: dict[int, bool] = {}
        # Per-wheel accumulated time-in-shortfall (seconds). Counts up while
        # shortfall > sat_margin and resets to 0 when shortfall < sat_release_margin.
        # A wheel enters the saturated state only after this accumulator exceeds
        # sat_settle_s, preventing transient acceleration lag from triggering saturation.
        self._sat_time: dict[int, float] = {}

        # Timestamp of the previous tick (ms); None on the first tick.
        self._prev_time_ms: int | None = None

        # Thread control.
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # Controllers: built lazily in start().
        self._controllers: dict[int, HubVelocityController] = {}

    # ------------------------------------------------------------------
    # Setpoint API (thread-safe)
    # ------------------------------------------------------------------

    def set_speed(self, speed: float) -> None:
        """Set the target scale ``S`` (counts/s for the reference wheel weight 1.0).

        Thread-safe; may be called from any thread while the governor is running.

        Parameters:
            speed: Target velocity scale in encoder counts per second.
        """
        with self._lock:
            self._target_scale = speed

    def set_speed_rpm(self, rpm: float) -> None:
        """Set the target scale in revolutions per minute.

        Converts ``rpm`` to counts/s via ``counts_per_s = rpm * cpr / 60``.

        Parameters:
            rpm: Target speed in RPM.

        Raises:
            ValueError: If ``cpr`` was not provided to the constructor.
        """
        if self._cpr is None:
            raise ValueError(
                "set_speed_rpm requires 'cpr' (counts per revolution) to be set in the constructor"
            )
        counts_per_s = rpm * self._cpr / 60.0
        with self._lock:
            self._target_scale = counts_per_s

    def set_weights(self, weights: Mapping[int, float]) -> None:
        """Replace the weight mapping atomically.

        Parameters:
            weights: New channel → weight mapping.
        """
        with self._lock:
            self._weights = dict(weights)
            # Reset saturation flags and accumulators for channels that changed.
            self._sat_flags = {}
            self._sat_time = {}

    def set_ratio(self, ratio: float, pair: tuple[int, int] = (0, 1)) -> None:
        """Convenience shorthand: set ``{pair[0]: 1.0, pair[1]: ratio}``.

        Only the two channels in *pair* are updated; other channel weights
        are left unchanged.

        Parameters:
            ratio: Weight for the second channel; the first channel is 1.0.
            pair:  Channel indices (first, second).
        """
        with self._lock:
            self._weights[pair[0]] = 1.0
            self._weights[pair[1]] = ratio
            # Reset saturation flags and accumulators for the affected channels.
            self._sat_flags.pop(pair[0], None)
            self._sat_flags.pop(pair[1], None)
            self._sat_time.pop(pair[0], None)
            self._sat_time.pop(pair[1], None)

    def stop(self, brake: bool = False) -> None:
        """Command all wheels to zero and set ``target_scale = 0``.

        Parameters:
            brake: Reserved for future use (not currently implemented at the
                   firmware level by this method).
        """
        with self._lock:
            self._target_scale = 0.0
            for ch, ctrl in self._controllers.items():
                ctrl.command(0)
            # Update commanded_targets to reflect the zero command.
            self._commanded_targets = {ch: 0 for ch in self._controllers}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Attach all wheel controllers and start the governor daemon thread.

        Idempotent: a second call while the thread is running is a no-op.

        If ``velocity_pid`` was specified in the constructor, it is pushed to
        each wheel's motor via ``motor.set_velocity_pid(p, i, d)`` here.
        """
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return  # already running

            # Build one HubVelocityController per non-zero-weight channel.
            with self._lock:
                weights_snapshot = dict(self._weights)

            for ch, w in weights_snapshot.items():
                if ch not in self._controllers:
                    self._controllers[ch] = HubVelocityController(self._hub, channel=ch)

            # Push PID coefficients if configured.
            if self._velocity_pid is not None:
                p, i, d = self._velocity_pid
                for ch in self._controllers:
                    self._hub.motors[ch].set_velocity_pid(p, i, d)

            # Attach each controller (sends CONSTANT_VELOCITY + enable).
            for ctrl in self._controllers.values():
                ctrl.attach()

            # Reset thread stop event and timestamp.
            self._stop_event.clear()
            self._prev_time_ms = None

            # Start the daemon thread.
            self._thread = threading.Thread(
                target=self._run,
                daemon=True,
                name="rhsp-ratiodrive",
            )
            self._thread.start()

    def stop_loop(self) -> None:
        """Signal the daemon thread to exit and join it.

        Does NOT detach controllers or zero wheels.  Call :meth:`close` for
        a full teardown.
        """
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def close(self) -> None:
        """Zero all wheels, detach controllers, restore motor modes, stop thread.

        Idempotent: safe to call multiple times.
        """
        # Signal and join the thread first so no more governor steps occur.
        self.stop_loop()

        with self._lock:
            controllers = dict(self._controllers)

        # Zero and detach each controller.
        for ch, ctrl in controllers.items():
            try:
                ctrl.command(0)
            except Exception:
                pass
            try:
                ctrl.detach(disable=True)
            except Exception:
                pass

        with self._lock:
            self._commanded_targets = {ch: 0 for ch in controllers}
            # Clear controllers so start() can rebuild cleanly.
            self._controllers.clear()
            self._scale = 0.0

    def __enter__(self) -> "RatioDrive":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Governor step
    # ------------------------------------------------------------------

    def update(self, bulk: BulkInputData) -> None:
        """Apply one synchronous governor step using a pre-fetched bulk snapshot.

        This method may be called directly from tests without running the
        daemon thread.  All state mutations are protected by the internal
        ``RLock``.

        Parameters:
            bulk: A :class:`~rhsp.devices.bulk.BulkInputData` snapshot obtained
                  from ``hub.bulk_input()`` (or built synthetically in tests).
        """
        with self._lock:
            # -- dt computation ------------------------------------------
            t_ms = bulk.monotonic_time_ms
            if self._prev_time_ms is None or t_ms == self._prev_time_ms:
                # First tick or no time advance: fall back to nominal period.
                dt = 1.0 / self._rate_hz
            else:
                raw_dt_ms = t_ms - self._prev_time_ms
                if raw_dt_ms < 0:
                    # Wraparound of 32-bit ms counter (rolls ~every 49 days).
                    raw_dt_ms += 2**32
                dt = raw_dt_ms / 1000.0
                # Sanity-cap: ignore implausibly large gaps (> 5 s) by falling back.
                if dt > 5.0:
                    dt = 1.0 / self._rate_hz
            self._prev_time_ms = t_ms

            # -- Read measured velocities ---------------------------------
            _VELOCITY_FIELDS = [
                bulk.motor0_velocity,
                bulk.motor1_velocity,
                bulk.motor2_velocity,
                bulk.motor3_velocity,
            ]
            weights = self._weights
            S = self._target_scale
            g = self._scale

            new_measured: dict[int, int] = {}
            new_normalized: dict[int, float] = {}
            for ch, w in weights.items():
                raw_v = _VELOCITY_FIELDS[ch] if 0 <= ch <= 3 else 0
                new_measured[ch] = raw_v
                if w != 0.0:
                    # Apply deadband.
                    v = 0 if abs(raw_v) < self._deadband else raw_v
                    new_normalized[ch] = math.copysign(1.0, w) * v / abs(w)

            self._measured = new_measured
            self._normalized_actual = new_normalized

            # -- Saturation state update ----------------------------------
            for ch, n_i in new_normalized.items():
                denom = max(abs(g), _EPSILON)
                shortfall = (g - n_i) / denom
                was_sat = self._sat_flags.get(ch, False)
                if was_sat:
                    # Leave saturation only when shortfall drops below release margin.
                    if shortfall < self._sat_release_margin:
                        self._sat_flags[ch] = False
                        self._sat_time[ch] = 0.0
                    # else: remain saturated (hysteresis — no accumulator change needed)
                else:
                    # Accumulate time-in-shortfall; enter saturation only after settle window.
                    if shortfall > self._sat_margin:
                        self._sat_time[ch] = self._sat_time.get(ch, 0.0) + dt
                        if self._sat_time[ch] >= self._sat_settle_s:
                            self._sat_flags[ch] = True
                    else:
                        # Shortfall below entry threshold — reset accumulator.
                        self._sat_time[ch] = 0.0

            saturated_channels = [ch for ch, f in self._sat_flags.items() if f]
            self._saturated = len(saturated_channels) > 0

            # -- Ceiling computation (velocity path) ---------------------
            #
            # The governor uses two complementary mechanisms to eliminate the
            # steady-state limit-cycle:
            #
            # 1. Damped upward probe: when not saturated, ceiling = g + probe_step
            #    per tick (not a jump to S).  After a cap-down event, a settle
            #    counter holds g steady for probe_settle_ticks ticks.
            #
            # 2. Learned bottleneck: when velocity saturation fires, the observed
            #    bottleneck speed (min n_i of saturated channels) is recorded in
            #    _learned_bottleneck.  In the probe path, a raw-cap limits the
            #    ceiling to any lagging wheel's observed speed, preventing g from
            #    silently climbing past the physical maximum even when the shortfall
            #    is below sat_margin.  When the bottleneck clears (all wheels keeping
            #    up), raw_cap = probe_target and g recovers at probe_step/tick.
            #
            if saturated_channels:
                # Cap-to-slowest via hysteresis state (official saturation).
                ceiling = min(new_normalized[ch] for ch in saturated_channels)
                # Record the observed bottleneck normalised speed.  Take the
                # minimum so that tightening bottlenecks are tracked downward;
                # loosening is detected via the recovery path below.
                if self._learned_bottleneck is None or ceiling < self._learned_bottleneck:
                    self._learned_bottleneck = ceiling
                # Reset the settle counter whenever saturation fires so that g
                # is held steady at the bottleneck equilibrium after the cap.
                self._probe_settle_counter = self._probe_settle_ticks
            else:
                # No hysteresis-flagged saturation.
                if self._learned_bottleneck is None:
                    # No prior saturation event (initial spin-up): full ceiling = S,
                    # original behaviour — g ramps at recovery_accel.
                    ceiling = S
                elif self._probe_settle_counter > 0:
                    # Post-saturation settle window: hold g steady.  This
                    # deadband gives the governor time to read a stable velocity
                    # measurement at the bottleneck equilibrium before probing.
                    self._probe_settle_counter -= 1
                    ceiling = g
                else:
                    # Post-settle probe: g is near the bottleneck equilibrium.
                    # Allow g to climb by at most probe_step per tick toward S,
                    # but immediately cap at any wheel's observed normalised speed
                    # that is BELOW the probe target.
                    #
                    # "Immediately cap" bypasses the sat_settle_s hysteresis window
                    # that would otherwise allow g to silently climb sat_margin above
                    # the physical limit before saturation fires.  Because this path
                    # is only active after a confirmed saturation event
                    # (_learned_bottleneck is not None), spin-up lag (which never
                    # enters this path) is unaffected.
                    probe_target = min(g + self._probe_step, S)
                    # Find the lowest measured normalised speed that is significantly
                    # below the current g.  "Significantly" means the shortfall
                    # exceeds sat_release_margin — the same threshold used to release
                    # the official saturation state.  This avoids false caps from
                    # normal measurement noise (which is typically < sat_release_margin
                    # of g).  A wheel that is genuinely unable to keep up (physically
                    # limited) will show n_i < g*(1-sat_release_margin) persistently.
                    noise_floor = g * self._sat_release_margin
                    raw_cap: float = probe_target
                    for n_i in new_normalized.values():
                        if n_i < g - noise_floor:
                            raw_cap = min(raw_cap, n_i)
                    ceiling = raw_cap
                    # _learned_bottleneck is NOT cleared here.  The raw_cap mechanism
                    # provides correct behaviour in both steady-state and recovery:
                    # - Steady-state bottleneck (n_i < g): raw_cap holds g near the
                    #   physical limit (ceiling ≈ n_i), achieving the fixed point.
                    # - After bottleneck clears (n_i ≥ g): raw_cap = probe_target =
                    #   g + probe_step, so g climbs at probe_step per tick toward S
                    #   — gradual recovery taking ~1 s at default probe_step=10, 50 Hz.
                    # The learned value can only decrease (updated when saturation
                    # fires with a tighter ceiling); it is never raised.

            # -- Current-aware ceiling (opt-in) --------------------------
            # When current_limit_ma is set, read per-motor current every
            # _cur_sample_every ticks (to stay within the 20 ms tick budget;
            # see constructor docstring for the transaction-budget rationale).
            # The last sampled values are reused on non-sample ticks.
            if self._current_limit_ma is not None:
                self._cur_tick_counter += 1
                if self._cur_tick_counter >= self._cur_sample_every:
                    self._cur_tick_counter = 0
                    # Sample current for all active channels.
                    for ch in weights:
                        try:
                            self._last_current_ma[ch] = self._hub.motors[ch].get_current_ma()
                        except Exception:
                            # On transient read failure keep the last known value.
                            pass

                # Update current hysteresis flags and compute current ceiling.
                limit = self._current_limit_ma
                hys = self._cur_hys
                cur_ceiling = S  # default: no current constraint
                for ch in weights:
                    cur_ma = self._last_current_ma.get(ch, 0)
                    was_cur_sat = self._cur_sat_flags.get(ch, False)
                    if was_cur_sat:
                        # Release when current drops below limit * (1 - hys).
                        if cur_ma < limit * (1.0 - hys):
                            self._cur_sat_flags[ch] = False
                    else:
                        # Engage when current exceeds limit * (1 + hys).
                        if cur_ma > limit * (1.0 + hys):
                            self._cur_sat_flags[ch] = True

                    if self._cur_sat_flags.get(ch, False) and g > _EPSILON:
                        # Project what g should be so this motor's current
                        # would be at (or just below) the limit.
                        # Simple proportional cap: ceiling_ch = g * limit / cur_ma.
                        # This is the same cap-to-slowest approach used for velocity.
                        ch_cur_ceiling = g * (limit / max(cur_ma, 1))
                        cur_ceiling = min(cur_ceiling, ch_cur_ceiling)

                # The more restrictive ceiling wins (velocity vs current).
                ceiling = min(ceiling, cur_ceiling)

            # -- Slew-limit g toward ceiling -----------------------------
            if g > ceiling:
                # Moving down: limited by max_accel.
                g = max(ceiling, g - self._max_accel * dt)
            else:
                # Moving up (or holding): limited by recovery_accel.
                g = min(ceiling, g + self._recovery_accel * dt)

            # Clamp to [min_scale, S].
            g = max(self._min_scale, min(S, g))
            self._scale = g

            # -- Emit targets --------------------------------------------
            new_targets: dict[int, int] = {}
            for ch, w in weights.items():
                if w == 0.0:
                    target = 0
                else:
                    target = clamp_int16(round(g * w))
                new_targets[ch] = target
                if ch in self._controllers:
                    self._controllers[ch].command(target)
            self._commanded_targets = new_targets

    def step(self) -> None:
        """Fetch a bulk snapshot from the hub and apply one governor step.

        Called by the daemon thread body; may also be called manually for
        testing with a real hub.
        """
        bulk = self._hub.bulk_input()
        self.update(bulk)

    # ------------------------------------------------------------------
    # Daemon thread
    # ------------------------------------------------------------------

    def _run(self) -> None:
        """Daemon thread body: tick at ``rate_hz`` until ``_stop_event`` is set."""
        period = 1.0 / self._rate_hz
        while not self._stop_event.wait(period):
            try:
                self.step()
            except Exception as exc:
                if self._on_error is not None:
                    try:
                        self._on_error(exc)
                    except Exception:
                        pass
                # Continue — never kill the thread on transient errors.

    # ------------------------------------------------------------------
    # Read-only properties
    # ------------------------------------------------------------------

    @property
    def scale(self) -> float:
        """Current governor scale ``g`` (counts/s for a unit-weight wheel)."""
        with self._lock:
            return self._scale

    @property
    def target_scale(self) -> float:
        """Target setpoint ``S``."""
        with self._lock:
            return self._target_scale

    @target_scale.setter
    def target_scale(self, value: float) -> None:
        with self._lock:
            self._target_scale = value

    @property
    def commanded_targets(self) -> dict[int, int]:
        """Most-recent commanded target per channel (copy)."""
        with self._lock:
            return dict(self._commanded_targets)

    @property
    def measured(self) -> dict[int, int]:
        """Most-recent measured velocity per channel from bulk (copy)."""
        with self._lock:
            return dict(self._measured)

    @property
    def normalized_actual(self) -> dict[int, float]:
        """Normalised actual speed ``n_i = sign(w_i) * v_i / |w_i|`` per active channel (copy)."""
        with self._lock:
            return dict(self._normalized_actual)

    @property
    def saturated(self) -> bool:
        """``True`` if any wheel is currently in the saturated state."""
        with self._lock:
            return self._saturated

    @property
    def weights(self) -> dict[int, float]:
        """Current weight mapping (copy)."""
        with self._lock:
            return dict(self._weights)

    @property
    def last_current_ma(self) -> dict[int, int]:
        """Most-recently sampled current per channel in milliamps (copy).

        Only populated when ``current_limit_ma`` is set.  Returns an empty
        dict when current sensing is disabled.  Values are updated every
        ``_cur_sample_every`` governor ticks.
        """
        with self._lock:
            return dict(self._last_current_ma)
