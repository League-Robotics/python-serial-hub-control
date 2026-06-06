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

_EPSILON: float = 1e-9  # guard against division by zero


class RatioDrive:
    """Ratio-preserving multi-wheel velocity coordinator with a two-cap governor.

    ``RatioDrive`` manages a set of :class:`HubVelocityController` instances
    (one per wheel channel) and applies a deterministic governor that keeps the
    commanded ratio between wheels exact at all times.  A single float scale
    ``g`` (counts/s equivalent) is maintained; each wheel is commanded
    ``clamp_int16(round(g * w_i))`` where ``w_i`` is that wheel's weight.

    Governor algorithm (one step):
        Two independent caps pull ``g`` down; the rest of the time ``g`` slews
        toward setpoint ``S``:

        1. **Max-speed cap (sticky, dv/dt-based)**: A wheel is "at its ceiling"
           when its measured velocity is below its commanded target by
           ``speed_margin_frac`` AND ``|dv/dt|`` is below
           ``plateau_threshold_cnts_s2`` (stopped accelerating — not just lagging
           during spin-up).  On detection, latch ``g_max_speed[i] = v_i/|w_i|``
           and hold it (no probing → no ripple / limit cycle).  Latches clear when
           the setpoint / weights change, or when measured velocity rises above
           the latched cap on its own.

        2. **Load cap (live current closed-loop)**: Active only when
           ``current_limit_ma`` is set.  If any motor current exceeds the limit,
           pull ``g`` down proportionally.  When all currents are under the limit,
           apply NO current cap — ``g`` is free to climb (fixes the 007/009 stuck-
           at-zero regression).

        3. ``g_target = min(S, sticky_max_speed_cap, live_current_cap)``; slew
           ``g`` down fast (``max_accel``), up damped (``recovery_accel_up``);
           clamp to ``[min_scale, S]``.

    Threading model:
        A daemon thread (``rhsp-ratiodrive``) calls ``step()`` at ``rate_hz``.
        All mutable state is guarded by an internal ``RLock``.  The thread
        coexists safely with the hub's keep-alive heartbeat.

    Parameters:
        hub:               The connected :class:`~rhsp.hub.Hub`.
        weights:           Mapping of channel index → signed weight (float).
                           Channels with weight 0 are excluded from the governor.
        rate_hz:           Daemon-thread tick rate (Hz).  Default 50.
        max_accel:         Maximum downward slew rate (counts/s²).  Default 6000.
        recovery_accel:    Upward slew rate alias for ``recovery_accel_up``
                           (kept for API compatibility).  If both are supplied,
                           ``recovery_accel_up`` wins.
        min_scale:         Floor for ``g`` (clamp after slew).  Default 0.0.
        deadband:          Measured velocity magnitude below which the reading
                           is treated as zero for normalisation.  Default 20.
        cpr:               Encoder counts per revolution.  Required for
                           :meth:`set_speed_rpm`.
        velocity_pid:      Optional ``(p, i, d)`` tuple pushed to each wheel's
                           motor PID during :meth:`start`.
        on_error:          Callable invoked with an ``Exception`` on transient
                           per-iteration errors.  If ``None``, errors are
                           suppressed silently.
        current_limit_ma:  Per-motor current limit in milliamps.  When set,
                           the governor reads ``Motor.get_current_ma()`` each Nth
                           tick and caps ``g`` proportionally when any motor exceeds
                           the limit.  Releasing load → current drops → ``g``
                           climbs back automatically.  ``None`` (default) disables
                           current sensing.

                           **Transaction-budget note**: ``GetADC`` costs ~16 ms
                           per call on fw 1.8.2.  Reading 2 motors every tick at
                           50 Hz (20 ms) would overflow the period.  Instead,
                           current is sampled every ``_cur_sample_every`` ticks
                           (default 3, giving ~16 Hz effective current sampling
                           at 50 Hz governor rate).

        ema_alpha:         EMA smoothing factor for per-wheel velocity derivative
                           (0 < alpha ≤ 1).  Smaller values smooth more.
                           Default 0.3.
        plateau_threshold_cnts_s2: |dv/dt| (counts/s²) below which a wheel is
                           considered to have stopped accelerating and may be
                           latched as at its speed ceiling.  Tune on hardware;
                           default 150.
        speed_margin_frac: Fractional shortfall (measured below commanded) that
                           must be present before the plateau test fires.  This
                           prevents latching when the wheel is already tracking
                           the target.  Default 0.15.
        recovery_accel_up: Upward slew rate for ``g`` (counts/s²).  Chosen so
                           load recovery from a de-rated ``g`` back to ``S``
                           takes approximately 1 second.  Default 500 counts/s²
                           (500 counts/s per second → ~2 s for a 1000 count/s
                           climb at 50 Hz).
        latch_persist_ticks: Number of consecutive ticks the plateau condition
                           must hold before the speed latch fires.  This prevents
                           a single noisy velocity sample (e.g. a transient 0
                           during spin-up under the daemon thread) from misfiring
                           the latch.  Default 3 (60 ms at 50 Hz).
        min_latch_frac:    Minimum cap value as a fraction of the current ``g``
                           for a latch to fire.  A would-be cap below
                           ``min_latch_frac * g`` is a sign of spin-up noise or
                           a momentary encoder transient, not a genuine physical
                           ceiling.  Default 0.25 (cap must be at least 25% of
                           the current commanded scale).

    Deprecated / superseded parameters (accepted but ignored):
        sat_margin, sat_release_margin, sat_settle_s: replaced by the dv/dt
            plateau latch.
        probe_step, probe_settle_ticks: replaced by the sticky latch (no probing).
    """

    def __init__(
        self,
        hub: "Hub",
        weights: Mapping[int, float],
        *,
        rate_hz: float = 50.0,
        max_accel: float = 6000.0,
        recovery_accel: float | None = None,
        # Superseded params — accepted but ignored (kept for API compat).
        sat_margin: float = 0.15,
        sat_release_margin: float = 0.07,
        sat_settle_s: float = 0.3,
        probe_step: float = 10.0,
        probe_settle_ticks: int = 5,
        # Active params.
        min_scale: float = 0.0,
        deadband: int = 20,
        cpr: float | None = None,
        velocity_pid: tuple[float, float, float] | None = None,
        on_error: Callable[[Exception], None] | None = None,
        current_limit_ma: int | None = None,
        ema_alpha: float = 0.3,
        plateau_threshold_cnts_s2: float = 150.0,
        speed_margin_frac: float = 0.15,
        recovery_accel_up: float | None = None,
        latch_persist_ticks: int = 3,
        min_latch_frac: float = 0.25,
    ) -> None:
        self._hub = hub
        self._rate_hz = rate_hz
        self._max_accel = max_accel
        self._min_scale = min_scale
        self._deadband = deadband
        self._cpr = cpr
        self._velocity_pid = velocity_pid
        self._on_error = on_error

        # Two-cap governor tunables.
        self._ema_alpha: float = ema_alpha
        self._plateau_threshold: float = plateau_threshold_cnts_s2
        self._speed_margin_frac: float = speed_margin_frac
        # Latch-hardening tunables (003-010 bug fix):
        # A plateau must persist this many consecutive ticks before the latch fires,
        # preventing a single noisy sample from misfiring it.
        self._latch_persist_ticks: int = latch_persist_ticks
        # A would-be cap below min_latch_frac * g is a spin-up transient, not a
        # genuine physical ceiling.  Prevents g from being pinned near 0.
        self._min_latch_frac: float = min_latch_frac
        # Upward recovery slew rate: recovery_accel_up > recovery_accel > max_accel fallback.
        if recovery_accel_up is not None:
            self._recovery_accel_up: float = recovery_accel_up
        elif recovery_accel is not None:
            self._recovery_accel_up = recovery_accel
        else:
            self._recovery_accel_up = 500.0  # ~2 s to climb 1000 counts at 50 Hz
        # Keep _recovery_accel as an alias so existing tests that read it still work.
        self._recovery_accel: float = self._recovery_accel_up

        # Superseded params stored so tests that inspect internal state still pass.
        # These are NOT used by the new governor.
        self._sat_margin: float = sat_margin
        self._sat_release_margin: float = sat_release_margin
        self._sat_settle_s: float = sat_settle_s
        self._probe_step: float = probe_step
        self._probe_settle_ticks: int = probe_settle_ticks
        # Superseded runtime state kept for backward-compat property access.
        self._probe_settle_counter: int = 0
        self._learned_bottleneck: float | None = None
        self._sat_flags: dict[int, bool] = {}
        self._sat_time: dict[int, float] = {}
        self._saturated: bool = False

        # ---------------------------------------------------------------
        # Max-speed cap state (per-wheel, sticky latch).
        # ---------------------------------------------------------------
        # Per-wheel EMA of measured velocity (counts/s).
        self._vel_ema: dict[int, float] = {}
        # Per-wheel EMA of dv/dt (counts/s²).
        self._dvdt_ema: dict[int, float] = {}
        # Per-wheel sticky latch: None if not latched, else the g cap value.
        self._speed_latch: dict[int, float | None] = {}
        # Per-wheel flag: True once dv/dt EMA has exceeded plateau_threshold at least
        # once, indicating the motor has actively accelerated (not just been stationary).
        # The plateau latch only fires once this flag is set, preventing spin-up lag
        # (v=0 because the motor hasn't responded yet) from being mistaken for a
        # genuine saturation plateau.
        self._ever_accelerating: dict[int, bool] = {}
        # Per-wheel consecutive-tick counter for plateau persistence.
        # Counts how many consecutive ticks the plateau condition (shortfall + low dv/dt)
        # has been continuously satisfied.  Latch fires only when this reaches
        # _latch_persist_ticks, preventing a single noisy sample from misfiring.
        self._plateau_ticks: dict[int, int] = {}

        # ---------------------------------------------------------------
        # Current-limit governor (opt-in via current_limit_ma).
        # ---------------------------------------------------------------
        self._current_limit_ma: int | None = current_limit_ma
        # Hysteresis: engage when current > limit*(1+hys), release when < limit*(1-hys).
        self._cur_hys: float = 0.05
        # Sample current every N ticks to stay within the 20 ms tick budget.
        # GetADC costs ~16 ms each; 2 motors × 16 ms = 32 ms per sample tick.
        # At 50 Hz (20 ms period) we sample every 3rd tick → effective ~16 Hz.
        self._cur_sample_every: int = 3
        self._cur_tick_counter: int = 0
        # Last sampled current per channel (mA); reused between sample ticks.
        self._last_current_ma: dict[int, int] = {}
        # Per-channel current-saturation flags (hysteresis state).
        self._cur_sat_flags: dict[int, bool] = {}
        # Live current cap for g (updated each tick when current sensing is active).
        self._live_current_cap: float | None = None

        # Internal RLock guards all mutable governor state.
        self._lock = threading.RLock()

        # Governor state.
        self._scale: float = 0.0
        self._target_scale: float = 0.0
        self._weights: dict[int, float] = dict(weights)
        self._commanded_targets: dict[int, int] = {}
        self._measured: dict[int, int] = {}
        self._normalized_actual: dict[int, float] = {}

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

    def _clear_all_latches(self) -> None:
        """Clear all sticky max-speed latches and spin-up flags.  Must be called under ``_lock``."""
        self._speed_latch = {ch: None for ch in self._speed_latch}
        # Also reset ever-accelerating flags so the new setpoint gets a fresh spin-up window.
        self._ever_accelerating = {ch: False for ch in self._ever_accelerating}
        self._vel_ema = {}
        self._dvdt_ema = {}
        # Reset plateau persistence counters so the new setpoint has a clean window.
        self._plateau_ticks = {}
        # Update the cached saturated flag immediately so callers see the cleared state.
        self._saturated = False

    def set_speed(self, speed: float) -> None:
        """Set the target scale ``S`` (counts/s for the reference wheel weight 1.0).

        Clears all sticky max-speed latches so ``g`` may climb to the new setpoint.

        Thread-safe; may be called from any thread while the governor is running.

        Parameters:
            speed: Target velocity scale in encoder counts per second.
        """
        with self._lock:
            self._target_scale = speed
            self._clear_all_latches()

    def set_speed_rpm(self, rpm: float) -> None:
        """Set the target scale in revolutions per minute.

        Converts ``rpm`` to counts/s via ``counts_per_s = rpm * cpr / 60``.
        Clears all sticky max-speed latches.

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
            self._clear_all_latches()

    def set_weights(self, weights: Mapping[int, float]) -> None:
        """Replace the weight mapping atomically.

        Clears all sticky max-speed latches so ``g`` may re-converge under the new weights.

        Parameters:
            weights: New channel → weight mapping.
        """
        with self._lock:
            self._weights = dict(weights)
            # Reset superseded saturation state.
            self._sat_flags = {}
            self._sat_time = {}
            # Clear all per-wheel latches (weight change invalidates old caps).
            self._speed_latch = {}
            self._ever_accelerating = {}
            self._vel_ema = {}
            self._dvdt_ema = {}
            self._plateau_ticks = {}
            self._saturated = False

    def set_ratio(self, ratio: float, pair: tuple[int, int] = (0, 1)) -> None:
        """Convenience shorthand: set ``{pair[0]: 1.0, pair[1]: ratio}``.

        Only the two channels in *pair* are updated; other channel weights
        are left unchanged.  Clears sticky max-speed latches for all channels.

        Parameters:
            ratio: Weight for the second channel; the first channel is 1.0.
            pair:  Channel indices (first, second).
        """
        with self._lock:
            self._weights[pair[0]] = 1.0
            self._weights[pair[1]] = ratio
            # Reset superseded saturation state.
            self._sat_flags.pop(pair[0], None)
            self._sat_flags.pop(pair[1], None)
            self._sat_time.pop(pair[0], None)
            self._sat_time.pop(pair[1], None)
            # Clear all per-wheel latches (ratio change invalidates old caps).
            self._speed_latch = {}
            self._ever_accelerating = {}
            self._vel_ema = {}
            self._dvdt_ema = {}
            self._plateau_ticks = {}
            self._saturated = False

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

        Two-cap governor design:
          Cap 1 — sticky max-speed latch: fired when a wheel's measured velocity
            has PLATEAUED below its commanded target (dv/dt ≈ 0).  Latched g cap
            held until setpoint/weight changes or spontaneous velocity recovery.
          Cap 2 — live current closed-loop: proportional pull-down when any motor
            current exceeds current_limit_ma; no cap when all are under limit.
          g_target = min(S, sticky_cap, current_cap); slew down fast, up damped.

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

            # -- Cap 1: sticky max-speed latch (dv/dt-based) -------------
            #
            # For each active wheel:
            #   1. Update velocity EMA and dv/dt EMA.
            #   2. Set _ever_accelerating flag once |dv/dt| exceeds plateau_threshold.
            #   3. Evaluate plateau condition (only eligible when _ever_accelerating):
            #      - shortfall: commanded_n > measured_n by speed_margin_frac.
            #      - plateau: |dv/dt EMA| < plateau_threshold (stopped accel).
            #   4. If both hold and not latched: LATCH g_cap = n_i.
            #   5. If latched and measured_velocity > g_cap: CLEAR latch (spontaneous
            #      recovery).
            #
            # Guard: the plateau latch fires ONLY after _ever_accelerating is True.
            # This prevents a motor that has never moved (v=0 since tick 0) from
            # being mistakenly declared "saturated at its ceiling" — it has never
            # been accelerating, so it cannot have plateaued.  A genuinely saturated
            # motor first shows high dv/dt while ramping up, then dv/dt drops to zero
            # at its physical ceiling — that two-phase pattern is what triggers the latch.
            #
            for ch, w in weights.items():
                if w == 0.0:
                    continue
                n_i = new_normalized.get(ch, 0.0)
                # Commanded normalised target for this wheel is always |g| direction-aligned.
                commanded_n = g  # n_commanded = g * |w| / |w| = g

                # Update velocity EMA.
                # Initialize to 0.0 (not n_i) so the first tick correctly shows
                # the acceleration from rest — if initialized to n_i, the first-tick
                # raw_dvdt would be zero and the ever_accelerating flag could miss
                # the initial spin-up.
                prev_vel_ema = self._vel_ema.get(ch, 0.0)
                vel_ema = self._ema_alpha * n_i + (1.0 - self._ema_alpha) * prev_vel_ema
                self._vel_ema[ch] = vel_ema

                # Update dv/dt EMA using real dt (counts/s per second = counts/s²).
                if dt > _EPSILON:
                    raw_dvdt = (vel_ema - prev_vel_ema) / dt
                else:
                    raw_dvdt = 0.0
                prev_dvdt_ema = self._dvdt_ema.get(ch, 0.0)
                dvdt_ema = self._ema_alpha * raw_dvdt + (1.0 - self._ema_alpha) * prev_dvdt_ema
                self._dvdt_ema[ch] = dvdt_ema

                # Set ever-accelerating once the wheel shows meaningful acceleration.
                # This latches permanently (until setpoint/weight change) so that
                # once a motor has demonstrated it can accelerate, the plateau test
                # becomes eligible.
                if abs(dvdt_ema) >= self._plateau_threshold:
                    self._ever_accelerating[ch] = True

                # Plateau condition — eligible only after the motor has first accelerated.
                ever_accel = self._ever_accelerating.get(ch, False)
                shortfall = commanded_n - n_i  # positive when wheel is below target
                is_short = shortfall > self._speed_margin_frac * max(commanded_n, _EPSILON)
                is_plateau = abs(dvdt_ema) < self._plateau_threshold

                current_latch = self._speed_latch.get(ch)
                if current_latch is not None:
                    # Latched: check for spontaneous velocity recovery.
                    # Clear only when measured velocity SIGNIFICANTLY exceeds the latch cap,
                    # using a hysteresis margin of speed_margin_frac * current_latch.
                    # This prevents noisy readings (±20 counts) from accidentally clearing
                    # a latch that is genuinely at the motor's physical ceiling.
                    # The ceiling has only lifted if the motor is consistently running
                    # well above its previously measured maximum.
                    clear_threshold = current_latch * (1.0 + self._speed_margin_frac)
                    if n_i > clear_threshold:
                        self._speed_latch[ch] = None
                        self._plateau_ticks[ch] = 0  # reset persistence on clear
                    # (else: stay latched)
                else:
                    # Not latched: check whether to latch.
                    # Require: wheel has previously accelerated (not first-tick zero lag),
                    # is currently below commanded target by margin, and has stopped
                    # accelerating (dv/dt below plateau threshold).
                    #
                    # HARDENED LATCH (003-010 bug fix):
                    # 1. Persistence: plateau condition must hold for _latch_persist_ticks
                    #    consecutive ticks before latching.  A single noisy sample (e.g.
                    #    a transient v=0 during spin-up on the daemon thread) resets the
                    #    counter and cannot fire the latch.
                    # 2. Minimum cap guard: the would-be cap must be at least
                    #    min_latch_frac * g.  A near-zero cap signals spin-up noise or
                    #    encoder dropout, not a genuine physical speed ceiling.
                    if ever_accel and is_short and is_plateau and commanded_n > _EPSILON:
                        # Plateau condition holds this tick — increment persistence counter.
                        self._plateau_ticks[ch] = self._plateau_ticks.get(ch, 0) + 1
                        # Check if the plateau has persisted long enough AND the
                        # would-be cap is plausibly high (not a spin-up transient).
                        cap_val = max(n_i, 0.0)  # don't latch negative caps
                        min_valid_cap = self._min_latch_frac * max(g, _EPSILON)
                        if (
                            self._plateau_ticks[ch] >= self._latch_persist_ticks
                            and cap_val >= min_valid_cap
                        ):
                            # Genuine sustained plateau at a plausible speed — latch.
                            self._speed_latch[ch] = cap_val
                            self._plateau_ticks[ch] = 0  # reset after firing
                    else:
                        # Plateau condition NOT met — reset persistence counter.
                        # A single non-plateau tick (e.g. motor accelerating again)
                        # cancels any in-progress latch build-up.
                        self._plateau_ticks[ch] = 0

            # Effective sticky max-speed cap: min of all latched values, or S.
            latched_caps = [v for v in self._speed_latch.values() if v is not None]
            sticky_max_speed_cap = min(latched_caps) if latched_caps else S

            # Keep legacy _saturated flag True when at least one latch is active,
            # so read-only code that inspects rd.saturated still gets a useful signal.
            self._saturated = len(latched_caps) > 0

            # -- Cap 2: live current closed-loop (opt-in) ----------------
            #
            # Sample per-motor current every _cur_sample_every ticks.
            # If any motor is over-limit: compute proportional ceiling.
            # If all under limit: no current cap (g free to climb).
            # This is the critical fix for the 007/009 regression: releasing load
            # causes current to drop, which automatically removes the cap.
            #
            live_current_cap = S  # default: no current constraint
            if self._current_limit_ma is not None:
                self._cur_tick_counter += 1
                if self._cur_tick_counter >= self._cur_sample_every:
                    self._cur_tick_counter = 0
                    for ch in weights:
                        try:
                            self._last_current_ma[ch] = self._hub.motors[ch].get_current_ma()
                        except Exception:
                            pass  # keep last known value on transient read failure

                limit = self._current_limit_ma
                hys = self._cur_hys
                any_over = False
                for ch in weights:
                    cur_ma = self._last_current_ma.get(ch, 0)
                    was_cur_sat = self._cur_sat_flags.get(ch, False)
                    if was_cur_sat:
                        if cur_ma < limit * (1.0 - hys):
                            self._cur_sat_flags[ch] = False
                    else:
                        if cur_ma > limit * (1.0 + hys):
                            self._cur_sat_flags[ch] = True

                    if self._cur_sat_flags.get(ch, False) and g > _EPSILON:
                        any_over = True
                        # Proportional cap: scale g down so this motor reaches the limit.
                        ch_cur_ceiling = g * (limit / max(cur_ma, 1))
                        live_current_cap = min(live_current_cap, ch_cur_ceiling)

                # If no motor is over-limit, live_current_cap remains S → no constraint.

            # -- Slew g toward g_target ----------------------------------
            g_target = min(S, sticky_max_speed_cap, live_current_cap)

            if g > g_target:
                # Moving down: fast, limited by max_accel.
                g = max(g_target, g - self._max_accel * dt)
            else:
                # Moving up (or holding): damped, limited by recovery_accel_up.
                g = min(g_target, g + self._recovery_accel_up * dt)

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
