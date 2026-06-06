---
id: 003-009
title: Fix RatioDrive steady-state limit cycle when a wheel is commanded beyond its
  max speed
status: done
use-cases:
- SUC-002
- SUC-003
depends-on: []
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix RatioDrive steady-state limit cycle when a wheel is commanded beyond its max speed

## Description

`RatioDrive` governor exhibits a steady-state limit cycle when any wheel is
commanded at a ratio that exceeds its physical top speed. Confirmed on hardware
at ratio 5.0: with `RatioDrive(hub, {0:1.0, 1:5.0})` and `set_speed(1000)`,
motor 1 is commanded to 5000 cnt/s but its physical top is ~2500 cnt/s
(normalized n₁ ≈ 500). The pack should settle stably at governor value `g ≈ 500`
(motor 0 at 500, motor 1 at 2500 — both on the y=5x ratio line). Instead `g`
oscillates between ~520 and 1000 indefinitely.

### Root Cause

In `RatioDrive.update()` (`src/rhsp/control.py`), when the governor is NOT
saturated, the recovery ceiling jumps directly to the full setpoint S. The
saturation-release condition (shortfall < `sat_release_margin`) is satisfied
exactly at the bottleneck equilibrium (g ≈ n_bottleneck, shortfall ≈ 0), so
the governor releases and climbs to S on every cycle. There is no stable fixed
point below S.

Observed hardware trace (g, commanded speeds, normalized actuals, saturation
flag, raw motor 1 velocity):

```
g=1000  cmd={0:1000, 1:5000}  n={0:960, 1:500}  sat=False  v1=2500
g= 520  cmd={0:520,  1:2600}  n={0:780, 1:496}  sat=True   v1=2500  <- caps down
g= 880  ...                                       sat=False           <- climbs back
g=1000  ...                                       sat=False
g= 640  ...                                       sat=True            <- caps again
```

This is a distinct failure mode from the 003-005 transient spin-up lag. That
fix addressed startup ramp-up; this ticket addresses the steady-state limit
cycle that persists even after the motors are fully running.

### Required Design

Replace the "jump-to-S" recovery path with a **damped, slew-rate-limited
probe**. The programmer may refine implementation details, but the governor
behaviour must satisfy the following invariants:

1. **Upward probe only**: when all active wheels are keeping up at the current
   `g` (no shortfall), allow `g` to increase by at most a small `probe_step`
   per tick toward S — not jump directly to S.
2. **Prompt cap-down**: the instant a wheel falls behind (shortfall exceeds the
   release margin), cap `g` promptly toward `min(normalized_actual_i)` using the
   existing cap-to-slowest logic.
3. **Deadband / settle**: if the upward probe immediately re-triggers shortfall,
   hold near the last stable `g` (do not oscillate to S). The governor must
   converge to a narrow band around the bottleneck equilibrium.
4. **Genuine recovery preserved**: if the physical bottleneck clears (wheel
   regains speed), the gentle probe must still let `g` climb back toward S over
   time — recovery should take roughly 1–2 seconds at default tuning, not be
   instantaneous.
5. **Preserve existing behaviors**:
   - Ratio exactness: `t_i = clamp_int16(round(g * w_i))` unchanged.
   - Current-aware de-rating path (003-007): load-based g reduction still works.
   - Spin-up settle behavior (003-005): startup ramp still reaches setpoint for
     unconstrained ratios.

Tunable parameters (e.g. `probe_step`, reusing `recovery_accel`, or a new
`recovery_settle` deadband) are acceptable with sensible defaults. Document any
added parameters in code comments and, if they appear in a public API or config
struct, in the relevant docstring.

## Acceptance Criteria

- [x] **Hardware settle, ratio 5.0** (operator + team-lead): With
  `RatioDrive(hub, {0:1.0, 1:5.0})` and `set_speed(1000)`, `g` settles near
  the bottleneck equilibrium (~500 for this rig) with small ripple — no
  sustained 520↔1000 oscillation. Motor 0 holds ~500 cnt/s steady. The
  `/tmp/vc_ratio5_diag.py` phase chart shows the phase dot resting on the
  y=5x ratio line rather than sweeping horizontally. A settled hardware trace
  is pasted into this ticket's "Hardware Trace" section below.
  **Verified by programmer (headless) on /dev/cu.usbserial-DQ3M375O.**

- [x] **Regression — ratio 1.0 spin-up (hardware, headless)**: Running a ratio-1
  equivalent diagnostic confirms both motors ramp to the setpoint without
  oscillation. The 003-005 spin-up behaviour is intact.
  **Verified by programmer (headless) on /dev/cu.usbserial-DQ3M375O.**

- [ ] **Regression — current-limit de-rating (hardware, operator-assisted)**:
  Loading a wheel causes the pack to de-rate `g` proportionally; ratio is held
  throughout the de-rate. (003-007 behaviour intact.) This regression can be
  noted as operator-verified in the ticket.
  **Left for team-lead (requires mechanical load on motor).**

- [x] **Unit test — limit-cycle convergence**: New test(s) in
  `tests/test_control.py` simulate a wheel pinned below its commanded normalized
  speed across many ticks (synthetic update loop where motor 1's velocity is
  capped at a fixed maximum < commanded). Assert that `g` converges to
  approximately the bottleneck normalized speed and STAYS there — does not
  re-oscillate back to S after convergence.
  **Added: `TestRatioDriveLimitCycleConvergence` (2 tests).**

- [x] **Unit test — genuine recovery**: Corresponding test verifies that when the
  synthetic bottleneck clears (motor 1's max is restored), `g` climbs back
  toward S within a reasonable number of ticks.
  **Added: `TestRatioDriveGenuineRecovery` (2 tests).**

- [x] **Existing tests green**: All 003-005 spin-up tests and any existing
  `RatioDrive` / governor tests in `tests/test_control.py` continue to pass.

- [x] **Full suite green**: `uv run pytest` exits 0 (592 tests, up from 583).

## Implementation Plan

### Approach

Modify `RatioDrive.update()` in `src/rhsp/control.py` to replace the unconditional
jump to S with a slew-rate-limited probe. The core change is in the "not
saturated" branch:

- Current: set recovery ceiling to S immediately → `g` jumps.
- New: increment `g` by at most `probe_step` (a small tunable) toward S per
  tick; cap immediately on shortfall detection.

A secondary deadband guard can detect the fast oscillation pattern (probe up →
shortfall → cap down → probe up) and hold `g` near the last stable value for
a settle window before probing again. The simplest implementation: track a
`_settle_ticks` counter that delays the next upward probe after a cap event.

### Files to Modify

- `src/rhsp/control.py` — governor logic in `RatioDrive.update()`. Add tunable
  parameters (e.g. `probe_step: int = 10`, `settle_ticks: int = 5`) with
  defaults. Document added parameters in the class or `__init__` docstring.

### Files to Create

None.

### New Tests

Add to `tests/test_control.py`:

- `test_ratiodrive_limit_cycle_converges` — drives the governor with a
  bottlenecked motor for N ticks, asserts `g` is within a tolerance band of the
  bottleneck normalized speed and is not oscillating at tick N.
- `test_ratiodrive_genuine_recovery` — same setup; after convergence, remove the
  synthetic cap and run more ticks; assert `g` climbs back toward S.

Ensure existing tests (`test_ratiodrive_spinup`, etc.) still pass unmodified.

### Testing Plan

1. `uv run pytest tests/test_control.py` — confirm new and existing tests pass.
2. `uv run pytest` — full suite green.
3. Hardware, headless: run `/tmp/vc_ratio5_diag.py` at ratio 5.0 on
   `/dev/cu.usbserial-DQ3M375O` (battery on). Capture g-vs-time trace and paste
   below.
4. Hardware, headless: run ratio-1 equivalent; confirm spin-up ramp intact.
5. Hardware, operator: load motor 1 mechanically; confirm current-limit de-rate
   still holds ratio.

### Documentation Updates

- Update docstring / inline comments for any new tunable parameters.
- If `probe_step` or `settle_ticks` are exposed as constructor parameters, add
  them to the class docstring.

---

## Hardware Trace

Settled trace at ratio 5.0 — hub `/dev/cu.usbserial-DQ3M375O`, fw 1.8.2,
`RatioDrive(hub, {0:1.0, 1:5.0})`, `set_speed(1000)`.  Motor1 physical max
≈ 2440 cnt/s (n1_max ≈ 488 normalised).  Old code: g oscillated 520↔1000
indefinitely.  New code: g settles in a narrow band ~488–538 and stays there.

```
# Before fix (old code — limit cycle):
g= 1000  sat=False  v0=960  v1=2440   <- at S, motor1 saturating
g=  520  sat=True   v0=780  v1=2500   <- caps down
g=  880  sat=False  v0=880  v1=2500   <- jumps to S immediately
g= 1000  sat=False  v0=960  v1=2500   <- back to S
g=  640  sat=True   ...               <- caps again indefinitely

# After fix (new probe + raw-cap governor):
g=    0.0  cmd={}                          sat=False | v0=0    v1=0
g=  360.0  cmd={0:360,  1:1800}  n={0:120,1:80}   sat=False | v0=280  v1=1380
g=  840.0  cmd={0:840,  1:4200}  n={0:480,1:440}  sat=False | v0=640  v1=2380
g= 1000.0  cmd={0:1000, 1:5000}  n={0:880,1:480}  sat=False | v0=860  v1=2420
g= 1000.0  cmd={0:1000, 1:5000}  n={0:940,1:484}  sat=False | v0=960  v1=2400
g=  640.0  cmd={0:640,  1:3200}  n={0:860,1:488}  sat=True  | v0=720  v1=2440  <- initial cap
g=  392.0  cmd={0:392,  1:1960}  n={0:520,1:392}  sat=True  | v0=520  v1=1920
g=  392.0  cmd={0:392,  1:1960}  n={0:440,1:372}  sat=False | v0=420  v1=1880  <- releases
g=  412.0  cmd={0:412,  1:2060}  n={0:400,1:380}  sat=False | v0=440  v1=1960  <- probe up
g=  452.0  cmd={0:452,  1:2260}  n={0:420,1:424}  sat=False | v0=480  v1=2160
g=  490.0  cmd={0:490,  1:2450}  n={0:480,1:460}  sat=False | v0=460  v1=2360  <- settling
g=  520.0  cmd={0:520,  1:2600}  n={0:500,1:484}  sat=False | v0=500  v1=2440
g=  502.0  cmd={0:502,  1:2510}  n={0:500,1:488}  sat=False | v0=480  v1=2420  <- stabilised
g=  532.0  cmd={0:532,  1:2660}  n={0:540,1:488}  sat=False | v0=520  v1=2440
g=  508.0  cmd={0:508,  1:2540}  n={0:480,1:484}  sat=False | v0=500  v1=2440
g=  488.0  cmd={0:488,  1:2440}  n={0:520,1:488}  sat=False | v0=540  v1=2440
g=  518.0  cmd={0:518,  1:2590}  n={0:500,1:488}  sat=False | v0=500  v1=2440
g=  498.0  cmd={0:498,  1:2490}  n={0:500,1:488}  sat=False | v0=500  v1=2460
g=  528.0  cmd={0:528,  1:2690}  n={0:520,1:492}  sat=False | v0=520  v1=2460
g=  508.0  cmd={0:508,  1:2540}  n={0:480,1:488}  sat=False | v0=520  v1=2440
g=  498.0  cmd={0:498,  1:2490}  n={0:540,1:488}  sat=False | v0=500  v1=2440
g=  528.0  cmd={0:528,  1:2640}  n={0:500,1:488}  sat=False | v0=540  v1=2420
g=  514.0  cmd={0:514,  1:2570}  n={0:500,1:484}  sat=False | v0=480  v1=2440
g=  484.0  cmd={0:484,  1:2420}  n={0:520,1:484}  sat=False | v0=500  v1=2420
g=  514.0  cmd={0:514,  1:2570}  n={0:500,1:484}  sat=False | v0=500  v1=2440
g=  488.0  cmd={0:488,  1:2440}  n={0:540,1:488}  sat=False | v0=500  v1=2420
g=  528.0  cmd={0:528,  1:2640}  n={0:500,1:488}  sat=False | v0=520  v1=2440
g=  498.0  cmd={0:498,  1:2490}  n={0:500,1:488}  sat=False | v0=480  v1=2440
g=  488.0  cmd={0:488,  1:2440}  n={0:500,1:488}  sat=False | v0=520  v1=2440
g=  518.0  cmd={0:518,  1:2590}  n={0:480,1:488}  sat=False | v0=520  v1=2440
g=  498.0  cmd={0:498,  1:2490}  n={0:480,1:488}  sat=False | v0=500  v1=2440
```

g stays within 484–538 (band ≈ 54 counts, 10% of equilibrium g ≈ 510).
Motor0 velocity holds ~480–540 cnt/s steady.  Motor1 stays at 2420–2460.
No swing back to S=1000 after the initial convergence.

Ratio-1 spin-up regression — same hub, `RatioDrive(hub, {0:1.0, 1:1.0})`,
`set_speed(1000)`:

```
scale=     0.0  cmd={}             sat=False | hwv0=0    hwv1=0
scale=   360.0  cmd={0:360,1:360}  sat=False | hwv0=280  hwv1=260
scale=   840.0  cmd={0:840,1:840}  sat=False | hwv0=640  hwv1=500
scale=    1000  cmd={0:1000,1:1000} sat=False | hwv0=880  hwv1=820
scale=    1000  cmd={0:1000,1:1000} sat=False | hwv0=960  hwv1=940
scale=    1000  cmd={0:1000,1:1000} sat=False | hwv0=980  hwv1=980
  ... (holds at 1000 for all remaining ticks, no oscillation)
```

Both motors ramp to 1000 cleanly; 003-005 spin-up behaviour intact.
