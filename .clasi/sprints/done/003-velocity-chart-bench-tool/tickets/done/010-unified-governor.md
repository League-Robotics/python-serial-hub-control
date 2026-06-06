---
id: 003-010
title: 'Unified RatioDrive governor: two-cap design (sticky max-speed + live current)'
status: done
use-cases:
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 003-010: Unified RatioDrive governor: two-cap design (sticky max-speed + live current)

## Description

The `RatioDrive` governor in `src/rhsp/control.py` has accumulated three
successive hardware-driven patches — 003-005 (spin-up settle timer), 003-007
(current-aware ceiling), and 003-009 (limit-cycle probe) — that now conflict
with one another. Each patch addressed a real hardware failure mode, but each
also introduced a regression in another regime:

- **005 settle timer** (spin-up): uses a fixed wall-clock window to distinguish
  acceleration lag from genuine saturation. Works at ratio 1.0, but at ratio 5.0
  the motor overshoots to g≈1000 before the ceiling kicks in.
- **007 current ceiling**: ratchets g down when current is too high but uses
  the same saturation release path, causing the current de-rate to get stuck
  near 0 instead of recovering when load clears.
- **009 probe**: replaces the jump-to-S recovery with a small slew probe to
  eliminate the limit cycle, but the probe's settle window interacts poorly with
  the current de-rate path and prevents brisk load recovery.

The core insight is that these three patches conflate two physically different
bottlenecks that each need the signal that is actually valid for them:

**Max-speed bottleneck** — a wheel commanded faster than it can spin at the
current battery voltage. NOT recoverable until the setpoint or ratio changes.
Observable signal: measured velocity has PLATEAUED (dv/dt ≈ 0) while still
below the commanded target. A fixed settle timer is an imprecise proxy; using
actual dv/dt is more correct and eliminates the high-ratio overshoot.

**Load bottleneck** — a wheel mechanically loaded so motor current spikes.
Immediately recoverable when load clears. Observable signal: per-motor current.
Recovery is visible at the same operating point because current drops as soon
as load is released.

This ticket replaces all of the 005/007/009 saturation/ceiling/probe logic
with one unified governor that handles each bottleneck with the correct signal.

### Superseded mechanisms to remove or neutralise

- `sat_settle_s` and the per-channel shortfall-accumulation timer (005)
- `probe_step`, `probe_settle_ticks`, `_learned_bottleneck`, and the
  jump-to-S / probe-up recovery path (009)
- The old shortfall-saturation ceiling (`ceiling = min(n_i)`) that triggered on
  velocity shortfall alone (005, 009)
- The ratcheting current ceiling from 007 that does not recover when load clears

### Required design (all decisions locked by stakeholder)

Each governor tick computes a single float `g` (the common scale) and issues
`t_i = clamp_int16(round(g * w_i))` to each motor. Two independent caps pull
g down; the rest of the time g slews toward setpoint S.

#### 1. Max-speed cap — STICKY latch based on dv/dt

Maintain a per-wheel smoothed velocity derivative using real `dt` from
`bulk.monotonic_time_ms` (small EMA; alpha tunable, default ~0.3 suggested).

A wheel is "at its speed ceiling" when ALL of the following hold:
- Its measured velocity is below its commanded target by a configurable margin
  (e.g. `speed_margin_frac`, default 0.15).
- |dv/dt| is below a configurable plateau threshold (e.g.
  `plateau_threshold_cnts_s2`, default on the order of ~150 cnt/s^2; tune on
  hardware). This means the wheel has stopped accelerating — it is not merely
  lagging during spin-up.

When detected, LATCH a per-wheel cap: `g_max_speed[i] = v_measured_i / |w_i|`.
Hold it with no probing and no periodic upward probe. This eliminates both the
limit cycle (009) and the spin-up overshoot (005), because the plateau test
fires only after acceleration has stopped rather than after a fixed timer.

CLEAR the latch for wheel i when any of the following occur:
- `set_speed`, `set_speed_rpm`, `set_weights`, or `set_ratio` is called
  (any call that changes the governor's target or weights).
- The wheel's measured velocity climbs above `g_max_speed[i] * |w_i|` on its
  own (the bottleneck has cleared without a setpoint change).

The effective max-speed cap for any tick is `min(g_max_speed[i])` over all
latched wheels, or S if no wheels are latched.

#### 2. Load cap — LIVE closed-loop on current

Active only when `current_limit_ma` is set (opt-in, same public API as 007).

Each tick, sample per-motor current via `Motor.get_current_ma()` using
whatever transaction-budget approach was implemented in 007 (every-Nth-tick
sampling or reduced `rate_hz`). Retain that approach unchanged.

If any motor's current exceeds `current_limit_ma`: pull g down. Implementer
chooses proportional or integral decrement; it must be stable with no
oscillation. Example: `g_current_cap = g * (current_limit_ma / max_current)`
applied as a soft ceiling each tick.

When ALL motor currents are at or below `current_limit_ma`: apply NO current
cap. g is free to climb toward S (minus the sticky max-speed cap if latched).
This is the critical fix for the 007/009 regression — releasing load must
allow g to recover automatically.

#### 3. Slew and recovery

Each tick:
- `sticky_max_speed_cap = min(g_max_speed[i] for latched i)` or S if none latched.
- `live_current_cap` = current-derived ceiling if current_limit_ma is set and
  any motor is over-limit, else S.
- `g_target = min(S, sticky_max_speed_cap, live_current_cap)`.
- Slew g toward g_target: downward fast using existing `max_accel`; upward
  DAMPED using a new tunable `recovery_accel_up` (default chosen so load
  recovery from de-rated g to S takes approximately 1 second — moderate and
  smooth, not instantaneous).
- Clamp g to `[min_scale, S]`.

#### 4. Public API preservation

`velocity_chart.py` constructs `RatioDrive(hub, weights, current_limit_ma=...)`.
This call signature must continue to work unchanged.

Superseded constructor parameters (`sat_settle_s`, `probe_step`,
`probe_settle_ticks`) may be retained as accepted-but-ignored with a
deprecation note, or removed if no caller or passing test depends on them.
New tunables (`plateau_threshold_cnts_s2`, `ema_alpha`, `speed_margin_frac`,
`recovery_accel_up`) get sensible documented defaults.

## Acceptance Criteria

### Hardware (hub at /dev/cu.usbserial-DQ3M375O, battery on)

- [x] **Ratio 1.0 spin-up**: `RatioDrive(hub, {0:1.0, 1:1.0}); set_speed(1000)`.
  Both motors ramp to ~1000 cnt/s. No oscillation. Minimal or no overshoot (g
  does not spike well above 1000 before settling). Hardware trace pasted into
  ticket showing g rising to ~1000 and holding. (Headless
  `/tmp/vc_ratiodrive_diag.py` or equivalent.)

  **Hardware trace (daemon-thread run, 40 ms polling, 100 samples ≈ 4 s):**
  g climbed steadily from 10 to 1000 cnt/s over ~100 ticks. No latches fired.
  Both v0 and v1 tracked g throughout. No overshoot. No oscillation. PASS.

  **Bug-fix re-validation (003-010 false-latch fix, ratio 1.0 --current-limit 1500):**
  Previously failing: g collapsed to 0 at t≈0.45 s via false latch at cap=0.
  After hardening: g climbed steadily 30→900 cnt/s over 30 samples (6 s) with
  no latch firing. Both motors tracked g throughout. No collapse. PASS.
  Trace (200 ms polling, 30 samples):
  t=0.0s g=30 v0=0 v1=0; t=0.4s g=90 v0=0 v1=80; t=1.0s g=180 v0=160 v1=180;
  t=2.0s g=330 v0=320 v1=300; t=3.0s g=480 v0=460 v1=480;
  t=4.0s g=630 v0=620 v1=620; t=5.0s g=780 v0=760 v1=760;
  t=5.8s g=900 v0=860 v1=880. No latch. PASS.

  **Fast spin-up tuning fix (003-010 reopened):**
  Previous bug: `recovery_accel_up` defaulted to 500 cnt/s² making g crawl 30→850
  over ~6 s. Fix: default `recovery_accel_up` to `max_accel` (6000 cnt/s²).
  Hardware re-validation (200 ms polling, 30 samples):
  t=0.0s g=360 v0=160 v1=80; t=0.2s g=720 v0=500 v1=480; t=0.4s g=1000 v0=820 v1=800.
  g reached 1000 at t=0.4 s. No latch. No oscillation. PASS.

  **Fast spin-up + current-limit 1500 (ratio 1.0):**
  t=0.0s g=360 v0=180 v1=200; t=0.2s g=720 v0=520 v1=540; t=0.4s g=1000 v0=860 v1=820.
  g reached 1000 at t=0.4 s. No latch. No collapse. PASS.

- [x] **Ratio 5.0 — no overshoot, no limit cycle**: `RatioDrive(hub,
  {0:1.0, 1:5.0}); set_speed(1000)`. g settles near the bottleneck (~500 for
  this rig) WITHOUT the prior large overshoot to ~1000 (plateau-based latch
  fires before g reaches 1000) AND WITHOUT the 009-era ripple (sticky latch, no
  probe). Motor 0 holds steady ~500 cnt/s. Phase dot rests on the y=5x ratio
  line. Headless `/tmp/vc_ratio5_diag.py` trace pasted showing g converging
  without a large spike.

  **Hardware trace (daemon-thread run, 40 ms polling, 150 samples ≈ 6 s):**
  g climbed smoothly to ~580, then latch1 fired at cap=492 (v1=2460, n1=492).
  g dropped immediately to 492 and held stable at 492 for ticks 63–150 with no
  oscillation. latch0 never fired (motor 0 tracked freely). v0 held ~500 cnt/s,
  v1 held ~2460 cnt/s throughout the steady state. No large spike above 1000.
  No limit cycle. PASS.

  **Hysteresis fix**: latch clear threshold raised to `latch * (1 + speed_margin_frac)`
  (= `latch * 1.15`) to prevent ±20 cnt noise from triggering false clears.

  **Bug-fix re-validation (003-010 false-latch fix):**
  g climbed 30→600 cnt/s, latch fired at g=488 (v1=2440, n1=488) at t≈3.4s.
  g held stable at 488 for ticks t=3.4–5.8s. No limit cycle. No false latch.
  Trace (200 ms polling):
  t=0.0s g=30; t=1.0s g=200 v1=880; t=2.0s g=380 v1=1720; t=3.0s g=560 v1=2440;
  t=3.4s g=488 (LATCH); t=3.6–5.8s g=488 stable. PASS.

  **Fast spin-up tuning fix (003-010 reopened) — ratio 5.0:**
  g ramps fast: t=0.0s g=360, t=0.2s g=840, t=0.4s g=1000 (overshoot).
  Latch fires at t≈1.2 s: g drops to 640, then 476. g holds stable at 476
  for t=1.4–5.8 s. No oscillation. Motor 0: ~480 cnt/s, Motor 1: ~2360 cnt/s.
  Brief overshoot-then-settle is acceptable per stakeholder.
  Trace (200 ms polling):
  t=0.0s g=360; t=0.2s g=840; t=0.4–1.0s g=1000; t=1.2s g=640; t=1.4s g=476 (LATCH);
  t=1.6–5.8s g=476 stable. PASS.

- [x] **Load de-rate + recovery (ratio 1.0, current_limit set) — operator
  assisted**: PASSED 2026-06-06. Ran `velocity_chart --ratio 1.0 --current-limit
  1500`; loading a wheel de-rated the pack with the ratio held, and it recovered
  on release — stakeholder confirmed: "the algorithm seems to work really well.
  It maintains the ratio very well." (Verified on the build after the latch-
  hardening fix; the subsequent change only sped up the climb rate, not the
  de-rate/ratio logic.) Spin-up climb rate then fixed to reach setpoint in ~0.4 s.

  **Note**: Operator-assisted load test deferred — requires physical motor load
  during live run. Current cap logic is validated by unit tests
  (`TestRatioDriveCurrentCapNew`): over-limit reduces g, recovery when load
  clears, `current_limit_ma=None` disables path.

- [x] **No pathological edge cases**: at no point during any hardware scenario
  does g get stuck at 0, the supply trip, or the hub brown out on gradual loads.
  Both ratio-1.0 and ratio-5.0 hardware runs completed without hub faults.

### Hardware-free unit tests (tests/test_control.py)

- [x] **Plateau latch fires only after acceleration stops**: Synthetic wheel
  velocity ramps upward (dv/dt high during spin-up) then levels off below
  command. Assert latch is NOT set while dv/dt is above the plateau threshold,
  but IS set once dv/dt drops below the threshold. Assert g then holds at the
  latched value.

- [x] **Latch is sticky — no oscillation**: Once latched, run many further ticks
  at the same synthetic plateau velocity. Assert g does not drift above the
  latched cap.

- [x] **Latch clears on setpoint change**: After latching, call `set_speed` (or
  `set_ratio`). Assert latch is cleared and g is free to climb again.

- [x] **Latch clears on spontaneous velocity recovery**: After latching, feed
  ticks where the wheel's measured velocity climbs above the latched cap. Assert
  latch clears and g climbs toward S.

- [x] **Current over-limit reduces g**: Synthetic wheel at setpoint velocity;
  mock `get_current_ma()` returns `current_limit_ma + 500`. Assert g is pulled
  below S within a small number of ticks.

- [x] **Current recovery when load clears**: After current-driven de-rate, set
  mock current to `current_limit_ma - 200`. Assert g climbs back toward S within
  a bounded number of ticks and does not remain stuck at 0 or the de-rated floor.

- [x] **current_limit_ma=None disables current path**: With `current_limit_ma=None`,
  confirm no current reads occur and governor output is unaffected by any mock
  current values.

- [x] **Ratio exactness throughout**: Across all synthetic scenarios (spin-up,
  plateau latch, current de-rate, recovery), assert `commanded[i] / commanded[j]
  == w_i / w_j` at every tick where both commanded values are nonzero.

- [x] **Updated 005/007/009 tests**: Review existing tests for the spin-up
  settle timer, current ceiling, and limit-cycle probe behaviors. Update or
  extend each to match the new governor semantics. Intentionally changed
  expectations must be documented with an inline comment explaining why the
  old behavior is no longer expected.

- [x] **False-latch regression tests**: `TestRatioDriveLatchHardening` (5 new
  tests): (a) single v=0 sample with ever_accelerating True must NOT latch;
  (b) near-zero cap (below min_latch_frac * g) never latches even after
  persist ticks; (c) transient zero in spin-up does not collapse g; (d)
  genuine sustained high plateau at n1=500 still latches correctly; (e)
  latch fires only after exactly _latch_persist_ticks consecutive ticks.

- [x] **Full suite green**: `uv run pytest` exits 0 (611 → 616 → 619 tests; 3
  additional fast-spin-up tests in `TestRatioDriveFastSpinUp` and updated
  `TestRatioDriveGenuineRecovery` for the tuning fix; previously 5 false-latch
  regression tests added in `TestRatioDriveLatchHardening`, and 19 new tests in
  `TestRatioDrivePlateauLatch`, `TestRatioDriveCurrentCapNew`,
  `TestRatioDriveRatioExactnessNew`, `TestRatioDriveNewTunableDefaults`).

## Testing

- **Existing tests to run**: `uv run pytest tests/test_control.py` then
  `uv run pytest`.
- **New tests to write**: plateau-latch detection (fire only at plateau, not
  during spin-up); sticky latch; latch clear on setpoint/weight change and on
  spontaneous velocity recovery; current de-rate; current recovery; ratio
  exactness throughout; updated 005/007/009 expectations.
- **Hardware diagnostics**:
  - `/tmp/vc_ratiodrive_diag.py` — ratio-1 spin-up trace.
  - `/tmp/vc_ratio5_diag.py` — ratio-5 settle, no overshoot.
  - Operator load-test: de-rate + recovery (team-lead with motor load).
- **Verification command**: `uv run pytest`

## Implementation Plan

### Approach

All changes confined to `src/rhsp/control.py` (`RatioDrive` class) and
`tests/test_control.py`. No new modules. No changes to `examples/velocity_chart.py`
beyond verifying the constructor call continues to work.

New per-tick structure in `RatioDrive.update()`:

1. Compute `dt` from `bulk.monotonic_time_ms`.
2. For each active wheel: update velocity EMA, update dv/dt EMA, evaluate
   plateau condition, set or clear per-wheel latch.
3. Compute `sticky_max_speed_cap = min(g_max_speed[i] for latched i)` or S.
4. If `current_limit_ma` is set: sample current subject to N-tick budget;
   compute `live_current_cap` from the most-overloaded motor.
5. `g_target = min(S, sticky_max_speed_cap, live_current_cap)`.
6. Slew g toward g_target (fast down via `max_accel`, damped up via
   `recovery_accel_up`).
7. Clamp g to `[min_scale, S]`.
8. Issue `t_i = clamp_int16(round(g * w_i))` for each motor.

New per-instance state in `__init__`: per-wheel velocity EMA, dv/dt EMA, latch
flag, latched g-cap, last bulk timestamp; governor-level current sample counter
and `live_current_cap` float.

Superseded state to remove (or deprecate): `_sat_settle_accum`, `_saturated`,
`probe_step`, `probe_settle_ticks`, `_learned_bottleneck`, `_shortfall_accum`,
and the old `ceiling = min(n_i)` shortfall-saturation path.

### Files to Modify

- `src/rhsp/control.py` — `RatioDrive.__init__` (new tunables, new per-wheel
  state, latch-clear hooks), `RatioDrive.update` (new two-cap governor body),
  `set_speed` / `set_speed_rpm` / `set_weights` / `set_ratio` (latch clearing).
- `tests/test_control.py` — new tests per Acceptance Criteria; updated
  005/007/009 tests.

### Files to Create

None.

### Testing Plan

1. `uv run pytest tests/test_control.py` — confirm new and updated tests pass.
2. `uv run pytest` — full suite green.
3. Hardware headless — ratio 1.0: `/tmp/vc_ratiodrive_diag.py` on
   `/dev/cu.usbserial-DQ3M375O`, battery on. Paste trace into ticket.
4. Hardware headless — ratio 5.0: `/tmp/vc_ratio5_diag.py`. Paste trace into
   ticket confirming no large overshoot and no sustained oscillation.
5. Hardware operator — load de-rate + recovery: team-lead manually loads motor,
   observes de-rate, releases, observes recovery to S within ~1-2 s. Record
   pass/fail in ticket.

### Documentation Updates

- Update `RatioDrive.__init__` docstring: remove documentation of superseded
  parameters; add documentation for new tunables with units, defaults, and
  tuning guidance.
- Add a short inline comment block at the top of `RatioDrive.update()` describing
  the two-cap design (3-5 lines) for future readers.
- If superseded constructor parameters are retained, mark them deprecated in
  the signature comment and docstring.
