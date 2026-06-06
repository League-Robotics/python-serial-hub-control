---
id: '003'
title: RatioDrive cap-to-slowest governor, threading, unit tests
status: done
use-cases:
- SUC-005
depends-on:
- '002'
github-issue: ''
issue: plan-two-layer-motor-velocity-control-on-hub-pid-host-side-ratio-coordinator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# RatioDrive cap-to-slowest governor, threading, unit tests

## Description

Adds `RatioDrive` to `src/rhsp/control.py` (the file created in ticket 002).
`RatioDrive` is the public outer-loop coordinator: it runs a daemon thread at a
configurable rate, calls `hub.bulk_input()` each tick, reads measured velocities
from the returned `BulkInputData`, and applies the deterministic cap-to-slowest
governor with slew limiting. A single common float scale `g` is moved each step;
all wheel targets are emitted as `round(g * w_i)` (clamped to int16), so the
commanded ratio is exact by construction.

The governor design is specified in the approved issue
`plan-two-layer-motor-velocity-control-on-hub-pid-host-side-ratio-coordinator.md`
and must be implemented faithfully — do not redesign.

## Acceptance Criteria

### Constructor

- [x] `RatioDrive` constructor signature:
      `RatioDrive(hub, weights: Mapping[int, float], *, rate_hz=50.0,
      max_accel=6000.0, recovery_accel=None, sat_margin=0.15,
      sat_release_margin=0.07, min_scale=0.0, deadband=20, cpr=None,
      velocity_pid=None, on_error=None)`.
- [x] `recovery_accel` defaults to `max_accel` when `None`.
- [x] `velocity_pid`, if provided, is pushed to each wheel's
      `motor.set_velocity_pid(p, i, d)` during `start()` / `attach()`.

### Setpoint API (thread-safe)

- [x] `set_speed(speed)`: sets `target_scale` (counts/s, or RPM if `cpr` set);
      guarded by internal `RLock`.
- [x] `set_speed_rpm(rpm)`: only callable when `cpr` is set; raises `ValueError`
      otherwise; converts via `counts_per_s = rpm * cpr / 60`.
- [x] `set_weights(weights: Mapping[int, float])`: replaces ratio weights
      atomically (internal `RLock`).
- [x] `set_ratio(ratio: float, pair=(0, 1))`: convenience shorthand setting
      `{pair[0]: 1.0, pair[1]: ratio}` (does not disturb other wheel weights).
- [x] `stop(brake=False)`: commands zero to all wheels via their
      `VelocityController.command(0)`; sets `target_scale = 0`.

### Lifecycle

- [x] `start()`: creates and starts the daemon thread `rhsp-ratiodrive`; calls
      `controller.attach()` on each wheel's `HubVelocityController`; is
      idempotent (second call is a no-op if already running).
- [x] `stop_loop()`: signals the daemon thread to exit and joins it; does NOT
      detach controllers or zero wheels.
- [x] `close()`: commands zero to all wheels, calls `controller.detach(disable=True)`
      on each, restores prior motor modes, stops the loop thread; is idempotent.
- [x] `__enter__` calls `start()`; `__exit__` calls `close()`.
- [x] `update(bulk: BulkInputData)`: one synchronous governor step — can be
      called directly from tests without running the thread.
- [x] `step()`: calls `hub.bulk_input()` then `update(bulk)`; for use in the
      daemon thread body.

### Read-only Properties

- [x] `scale: float` — current governor scale `g`.
- [x] `target_scale: float` — the commanded setpoint S.
- [x] `commanded_targets: dict[int, int]` — most recent `t_i` emitted per
      channel.
- [x] `measured: dict[int, int]` — most recent `v_i` read from bulk per channel.
- [x] `normalized_actual: dict[int, float]` — `n_i = sign(w_i) * v_i / |w_i|`
      per active channel.
- [x] `saturated: bool` — True if any wheel is currently in the saturated state.
- [x] `weights: dict[int, float]` — current weight mapping (copy).

### Governor Algorithm

- [x] Normalized actual speed per active channel: `n_i = sign(w_i) * v_i / |w_i|`
      (guard `|w_i| > 0`; apply `deadband`: treat `|v_i| < deadband` as `v_i = 0`).
- [x] Saturation with hysteresis judged against the *current* `g` (not `S`):
      `shortfall_i = (g - n_i) / max(g, epsilon)`. Enter `saturated` when
      `shortfall_i > sat_margin`; leave when `shortfall_i < sat_release_margin`.
- [x] Ceiling: if any wheel saturated, `g_ceiling = min(n_i for saturated i)`;
      else `g_ceiling = S` (climb toward setpoint).
- [x] Slew-limit `g` toward `g_ceiling`: move down by at most `max_accel * dt`,
      up by at most `recovery_accel * dt`; clamp result to `[min_scale, S]`.
      Use real `dt` from `bulk.monotonic_time_ms` (difference from previous tick).
- [x] Emit `t_i = clamp_int16(round(g * w_i))` to each `VelocityController.command`.
- [x] Zero-weight wheels (`w_i == 0`): hold at 0, excluded from governor normalization
      and saturation checks (no division by zero).

### Threading

- [x] Daemon thread named `rhsp-ratiodrive`.
- [x] Loop body: `update(hub.bulk_input())` per tick at approximately `rate_hz`.
- [x] Per-iteration `try/except`: transient `Exception` calls `on_error(exc)` if
      set and continues; does not kill the thread.
- [x] `RatioDrive` internal `RLock` guards: `target_scale`, `_weights`,
      `_scale`, `_commanded_targets`, `_measured`, `_saturated`.
- [x] Coexists with the keep-alive heartbeat (`with hub:` keeps running); does not
      replace it.

### Unit Tests

- [x] **Ratio invariance**: feed a sweep of `update(bulk)` calls with varying
      measured velocities; assert `commanded_targets[0] / commanded_targets[1] ==
      weights[0] / weights[1]` (within float tolerance) at every step.
- [x] **Cap-to-slowest convergence**: wheel 1 stuck at 50% of its target for
      N steps; assert `scale` converges to `v1 / |w1|`; assert wheel 0 is never
      commanded above its proportional target (lagging wheel never boosted).
- [x] **Slew-limit down**: sudden bottleneck (one wheel drops to 0); assert
      `scale` decreases by no more than `max_accel * dt` per step.
- [x] **Slew-limit up**: bottleneck clears; assert `scale` increases by no more
      than `recovery_accel * dt` per step.
- [x] **Transient-accel robustness**: both wheels lag symmetrically during a
      ramp (shortfall < `sat_margin`); assert `scale` does NOT collapse.
- [x] **Recovery + hysteresis**: after bottleneck clears, `saturated` flips to
      `False`; assert no chatter (no rapid on/off cycles) over subsequent steps.
- [x] **Zero-weight**: channel with `w_i = 0` is always commanded 0, never
      participates in normalization; no `ZeroDivisionError`.
- [x] **Stall**: one wheel reads 0 persistently; assert `scale` converges to 0
      (or `min_scale`); no crash.
- [x] **Negative weight**: `{0: 1.0, 1: -0.5}` (one wheel reversed); assert
      `commanded_targets[1]` is negative when `g > 0`; ratio sign is correct.
- [x] **int16 clamp**: target that would exceed 32767 is clamped; shows up as
      a real bottleneck in the next step (measured velocity stays at the clamp);
      assert ratio is preserved despite clamp.
- [x] **start/close idempotency**: calling `start()` twice does not create two
      threads; `close()` twice does not crash or double-detach.
- [x] **Zero-on-close**: after `close()`, `commanded_targets` are all 0.

## Implementation Plan

### Approach

Append `RatioDrive` to the `src/rhsp/control.py` file created in ticket 002.
Use `threading.Thread(target=self._run, daemon=True, name='rhsp-ratiodrive')`.
The `update(bulk)` method is the pure governor step — all state mutations go
through the internal `RLock`. The daemon thread body is just a thin `while not
self._stop.wait(period)` loop calling `step()`.

### Files to Modify

- `src/rhsp/control.py` — append `class RatioDrive`:
  - `__init__`: store all constructor args; create `_lock = threading.RLock()`;
    create `_stop = threading.Event()`; `_thread: threading.Thread | None = None`;
    initialise `_scale = 0.0`, `_target_scale = 0.0`, `_commanded_targets = {}`,
    `_measured = {}`, `_saturated = False`, `_controllers: dict[int, HubVelocityController]`.
  - `start()`: build one `HubVelocityController` per non-zero-weight channel if
    not already built; call `attach()` on each (push `velocity_pid` if set); start
    daemon thread.
  - `update(bulk)`: implement the 5-step governor algorithm exactly as specified.
    Read `dt` from `bulk.monotonic_time_ms` (keep previous timestamp in
    `self._prev_time_ms`).
  - All setpoint methods, lifecycle methods, and properties as specified.
  - Import `threading` at the top of the file.

- `tests/test_control.py` — add `RatioDrive` test class/functions:
  - Helper: `make_bulk(v0, v1, time_ms)` that returns a synthetic `BulkInputData`
    with the given motor velocities and monotonic timestamp.
  - All governor unit tests as listed in the Acceptance Criteria section.
  - Lifecycle tests using `FakeHub`.

### Testing Plan

- `uv run pytest` must pass with >= 434 existing tests plus all new
  `RatioDrive` tests.
- No live-hub step — live validation for RatioDrive is in ticket 004.

### Documentation Updates

- Docstring on `RatioDrive` class describing the governor algorithm, the
  `update(bulk)` synchronous entry point, and the threading model.
- Document the `cpr` parameter for RPM convenience.
- Note the int16 velocity clamp constraint from the REV hub hardware.
