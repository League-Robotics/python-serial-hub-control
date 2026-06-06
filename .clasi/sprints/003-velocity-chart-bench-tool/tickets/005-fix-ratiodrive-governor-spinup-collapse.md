---
id: 003-005
title: Fix RatioDrive governor spin-up collapse (saturation on accel lag)
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

# 003-005: Fix RatioDrive governor spin-up collapse (saturation on accel lag)

## Description

Discovered during sprint 003 hardware validation: `RatioDrive` cannot spin the
motors up from rest. With `set_speed(1000)` on the real hub the governor
oscillates between `scale≈120` and `scale=0` forever; the motors barely twitch
and measured velocity stays ~0. Direct hub velocity control works fine (target
500 → measured 500), so the inner loop and hardware are healthy — the defect is
in the **`RatioDrive` outer-loop governor** in `src/rhsp/control.py`.

Instrumented trace (target=1000, channels 0&1 weight 1.0):
```
scale=0    cmd={}            meas={}
scale=120  cmd={0:120,1:120} meas={0:60,1:0}  sat=False
scale=120  cmd={0:120,1:120} meas={0:20,1:0}  sat=False
scale=0    cmd={0:0,1:0}     meas={0:0,1:0}    sat=True   <- collapse
scale=120  ...                                            <- climb
scale=0    ...                                 sat=True   <- collapse  (repeats)
```

### Root cause

In `RatioDrive.update()` the saturation test fires on a **single tick** with no
settling allowance:
```python
shortfall = (g - n_i) / max(abs(g), _EPSILON)
# enters saturation as soon as shortfall > sat_margin (0.15)
```
During spin-up the measured normalized speed `n_i` necessarily lags the scale
`g` (the motor takes time to accelerate). So on the very tick after `g` steps up
by `recovery_accel*dt` (≈120), `shortfall = (120 - ~0)/120 ≈ 1.0 > 0.15` → the
wheel is flagged saturated → `ceiling = min(n_i) ≈ 0` → `g` is slewed back to 0.
Next tick `g` is 0 (not saturated) → climbs to 120 → collapses again. The motor
never receives a sustained command long enough to reach speed.

The design intended saturation to trigger only for a wheel that *persistently*
cannot reach `g` (load/slip/stall), but the implementation has **no persistence
timer** — transient acceleration lag is indistinguishable from a real
bottleneck.

### Fix

Make saturation require the shortfall to **persist for a settling window** before
it caps the scale. Suggested approach (programmer may refine):

- Add a constructor param, e.g. `sat_settle_s: float = 0.3` (seconds the
  shortfall must continuously exceed `sat_margin` before a wheel is declared
  saturated). Default chosen so a normal motor spins up to setpoint within the
  window without ever flagging saturation, while a genuinely loaded/stalled
  wheel (shortfall sustained past the window) still caps the pack.
- Track per-channel "time in shortfall" (accumulate `dt` while
  `shortfall > sat_margin`, reset to 0 when it drops below
  `sat_release_margin`). Enter the saturated state only once the accumulator
  exceeds `sat_settle_s`; clear it on release (existing hysteresis preserved).
- Keep ratio exactness intact: the scale `g` still drives all wheels via
  `t_i = clamp_int16(round(g * w_i))`; only the saturation *gate* changes.

Preserve all existing public behavior (cap-to-slowest under genuine sustained
load, slew limits, hysteresis, zero-weight handling). This is a change to the
saturation-detection logic in `RatioDrive.update()` plus one new constructor
parameter.

## Acceptance Criteria

- [x] On the real hub, `RatioDrive(hub, {0:1.0, 1:1.0}); set_speed(1000)` ramps
      the scale up to ~1000 and both motors converge to ~1000 cnt/s within a few
      seconds — **no oscillation/collapse to 0**. Record the measured scale &
      velocities in the ticket. (Use `/tmp/vc_ratiodrive_diag.py` or an equivalent;
      motor motion expected.)

      Hardware trace (200 ms polling interval, 5 s run):
      - Tick 0: scale=0
      - Tick 1: scale=360, meas={0:0,1:0}, hwv0=260, hwv1=220 — no sat
      - Tick 2: scale=840, meas={0:560,1:520}, hwv0=620, hwv1=480 — no sat
      - Tick 3: scale=1000, meas={0:860,1:900}, hwv0=880, hwv1=820 — no sat
      - Ticks 4–24: scale=1000, both motors steady at ~980–1020 cnt/s, sat=False throughout
      Fix confirmed: no collapse to 0 at any point.

- [x] Ratio still preserved under a genuine sustained bottleneck: when one wheel
      is held/loaded so it cannot reach `g`, the scale caps to that wheel's
      actual speed and the other wheel slows to keep the commanded ratio (verified
      by unit test with synthetic bulks; hardware spot-check optional).
- [x] New hardware-free unit test in `tests/test_control.py` modeling
      **acceleration lag**: feed a sequence of synthetic `BulkInputData` where
      measured velocity trails the commanded target for several ticks during
      spin-up, and assert the governor ramps `scale` toward `S` instead of
      collapsing to 0. (The current "transient-accel robustness" test passes only
      because it does not model real spin-up lag — strengthen or add to it.)
- [x] Existing `tests/test_control.py` cases (ratio invariance, collapse-to-
      slowest, slew limiting, hysteresis, edge cases) still pass, adjusted only
      where the new settling semantics legitimately change expected timing.
- [x] `uv run pytest` green (was 540 passing).

## Testing

- **Existing tests to run**: `uv run pytest tests/test_control.py` then full `uv run pytest`.
- **New tests to write**: acceleration-lag spin-up test (above); ensure a
  sustained-shortfall test still caps the pack (real bottleneck) so the fix
  doesn't simply disable saturation.
- **Verification command**: `uv run pytest`
- **Hardware validation** (hub at `/dev/cu.usbserial-DQ3M375O`, battery on):
  run the RatioDrive diagnostic and confirm spin-up to setpoint; record numbers.
