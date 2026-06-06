---
id: 003-007
title: Current-aware de-rating in RatioDrive + chart current display
status: done
use-cases:
- SUC-002
- SUC-003
depends-on: []
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
completes_issue: false
github-issue: ''
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 003-007: Current-aware de-rating in RatioDrive + chart current display

## Description

`RatioDrive`'s velocity-based saturation governor (fixed in 003-005) cannot
protect the rig on this hub (fw 1.8.2). The onboard CONSTANT_VELOCITY PID is
stiff: under load it holds velocity by ramping motor current until the supply
current-limits, voltage collapses, and the hub browns out — everything stops.
Because velocity never droops, the cap-to-slowest governor never triggers, and
the rig trips instead.

The solution is current-aware de-rating: when a motor's current exceeds a
configurable limit, `RatioDrive` treats that motor as the bottleneck and reduces
the common scale `g` until the current falls back under the limit. The
commanded-velocity ratio between motors is preserved exactly (same cap-to-slowest
machinery, now driven by current rather than velocity shortfall).

Per-motor current is now available via `Motor.get_current_ma()` (added in
003-006 via `GetADC ADC_Motor0..3`). The feature is opt-in: when
`current_limit_ma=None` (the default), the governor is unchanged and all existing
behavior is preserved.

The feature is also surfaced in `examples/velocity_chart.py` via a
`--current-limit` CLI flag and a visible current readout so the operator can see
each motor's current approach the limit in real time.

## Acceptance Criteria

- [x] `RatioDrive(hub, weights, current_limit_ma=<int>)` de-rates the pack when
      any motor's current exceeds `current_limit_ma`, preserving the commanded
      ratio between all motors' commanded targets (the ratio between commanded
      targets stays exact at all times). De-rating uses hysteresis and the
      existing slew limiting so the scale settles rather than oscillates.
- [x] When current falls back below the limit, `g` recovers toward the setpoint
      `S` (same slew-rate recovery as the velocity path).
- [x] When `current_limit_ma=None` (default), all behavior is identical to the
      pre-007 governor: velocity-only saturation, all existing tests unmodified.
- [x] **Hardware-free unit tests (synthetic)**:
      - Feed a mock hub where one motor's `get_current_ma()` returns a value
        above `current_limit_ma` while bulk velocities are at setpoint. Assert
        that `g` is capped (de-rated) and the ratio between commanded targets
        stays exact.
      - Feed mock current below the limit; assert `g` recovers toward `S`.
      - Feed `current_limit_ma=None`; assert behavior is identical to existing
        velocity-only tests (no regression).
- [x] `examples/velocity_chart.py` gains a `--current-limit` CLI flag
      (milliamps, per-motor, default 2000). The value is plumbed into
      `RatioDrive(current_limit_ma=...)`. `--help` lists the flag.
- [x] Each motor's current is visible in the GUI (e.g., shown in the window
      title or as a readout annotation). A full extra subplot is optional; a
      title-line readout is sufficient.
- [x] `uv run pytest` exits 0 with 562+ tests passing (no regressions).
- [ ] **Manual hardware gate (operator-in-the-loop, supply ~5 A)**:
      With `--current-limit` set safely below the supply trip threshold, run
      `velocity_chart.py`, press SPACE to start, then hand-load one motor.
      Expected: the pack de-rates, the phase-plot dot slides DOWN ALONG the
      ratio reference line (not off it), current holds near the limit, and
      the supply does NOT trip / the hub does NOT brown out. Release the load:
      the pack recovers toward setpoint. This step is run by the team-lead
      with the stakeholder and is the final acceptance gate.

## Implementation Plan

### Approach

The change is confined to `src/rhsp/control.py` (the `RatioDrive` class) and
`examples/velocity_chart.py`. No new modules are required.

#### 1. Constructor change in `RatioDrive` (`src/rhsp/control.py`)

Add `current_limit_ma: int | None = None` to `RatioDrive.__init__`. Store it.
When `None`, no current reads occur and the governor path is unchanged.

#### 2. Per-tick current sampling

When `current_limit_ma` is not `None`, read current for all active motor channels
once per governor tick using `Motor.get_current_ma()` (available via
`self._hub.motors[channel]` or the controllers dict, whichever is accessible).

**Transaction-budget consideration**: `GetADC` costs ~16 ms per call on this
hub. Reading 2 motors adds ~32 ms per tick. At the default 50 Hz (20 ms period)
this is unsustainable. The programmer must choose one of:
  - Lower the default governor `rate_hz` to ~20-25 Hz when
    `current_limit_ma` is not `None`.
  - OR sample current every Nth tick (e.g., every 3rd tick at 50 Hz ≈ 16 Hz
    effective current sampling), using the last sampled value in between.

Either approach is acceptable. Document the chosen approach in a docstring or
inline comment. The chosen approach must keep the keep-alive heartbeat from
starving (heartbeat period is 100 ms; governor tick + current reads must
complete well within that budget with margin).

#### 3. Current-aware saturation in `RatioDrive.update()`

When `current_limit_ma` is not `None` and any motor's current exceeds the limit:
- Determine the "bottleneck current" (the motor with the highest current).
- Compute a scale ceiling analogous to the velocity saturation path:
  cap `g` so the bottleneck motor's current is projected to fall toward the
  limit. A simple and robust approach: if `current > current_limit_ma`, set
  `ceiling = g * (current_limit_ma / current)` and apply it as an upper bound
  on `g` before the slew step.
- Apply hysteresis: only engage the current ceiling when current exceeds
  `current_limit_ma * (1 + hysteresis_fraction)` and only release it when
  current falls below `current_limit_ma * (1 - hysteresis_fraction)`. Reuse
  the existing `_sat_release_margin` or introduce a parallel `_cur_hys`
  parameter (programmer's choice, but must not oscillate on hardware).
- Continue applying the existing velocity saturation in parallel — both guards
  can act simultaneously; the more restrictive one wins.

#### 4. Ratio exactness

The commanded targets are still computed as `t_i = clamp_int16(round(g * w_i))`
where `w_i` are the weight ratios. `g` is what the current guard caps. This
means the ratio between any two commanded targets is always `w_i / w_j`,
exactly as before. No change to target computation is required.

#### 5. `examples/velocity_chart.py` changes

- Add `--current-limit` argument to the `argparse` parser:
  `type=int, default=2000, metavar="MA"`. Help string: per-motor current limit
  in milliamps; de-rates the pack if exceeded (default 2000 mA).
- Plumb the value into `RatioDrive(current_limit_ma=args.current_limit)` in the
  worker thread setup.
- Display per-motor current in the GUI. Preferred minimal approach: include each
  motor's current in the matplotlib figure title alongside the existing speed/
  status readout, e.g., appending `I0=NNNmA I1=NNNmA` updated each render
  frame. The current values must come from `Motor.get_current_ma()` (not bulk).
  A full extra subplot is acceptable but not required.

### Files to Create

None.

### Files to Modify

- `src/rhsp/control.py` — add `current_limit_ma` param and current-aware
  saturation logic to `RatioDrive`.
- `examples/velocity_chart.py` — add `--current-limit` flag and current readout.
- `tests/test_control.py` — add current-limit unit tests (synthetic mock).

### Testing Plan

1. **Unit tests** (`tests/test_control.py`):
   - `test_current_limit_derates_pack`: mock `get_current_ma()` for channel 0
     returning `current_limit_ma + 500` while bulk velocities are at setpoint.
     Run several ticks; assert `g < S` (scale is capped) and that commanded
     targets maintain exact ratio.
   - `test_current_limit_recovery`: start with current above limit, then drop
     mock current below limit; assert `g` climbs back toward `S` within a
     bounded number of ticks.
   - `test_current_limit_none_regression`: with `current_limit_ma=None`, run the
     same scenario as above; assert output is identical to the current
     velocity-only governor (existing tests).
   - `test_current_limit_ratio_exact`: with two motors at different weights
     (e.g., `{0: 1.0, 1: 0.5}`), trigger current saturation on motor 0; assert
     `commanded[0] / commanded[1] == 2.0` at every tick.

2. **Regression**: `uv run pytest tests/test_control.py` then `uv run pytest`.

3. **Hardware validation** (manual gate — team-lead with stakeholder):
   - Hub at `/dev/cu.usbserial-DQ3M375O`, 12V battery, bench supply ~5 A.
   - Run: `uv run --extra bench python examples/velocity_chart.py --speed 1000
     --ratio 1.0 --current-limit 1800`
   - Press SPACE. Motors spin; velocity strip charts converge; phase dot tracks
     ratio line.
   - Hand-load motor A. Observe: phase dot slides DOWN ALONG the ratio line;
     current readout approaches 1800 mA; supply does NOT trip; hub stays alive.
   - Release load. Observe: pack recovers toward setpoint.
   - Press SPACE to stop, then Q to quit.

### Idle hardware smoke trace (programmer gate — no load)

Hub at `/dev/cu.usbserial-DQ3M375O`, fw 1.8.2, battery on, no motor load.
`RatioDrive(hub, {0:1.0, 1:1.0}, current_limit_ma=2000)`, `set_speed(1000)`, run 3 s.

```
Connecting...
Hub connected, starting RatioDrive(current_limit_ma=2000)...
t=0.2s  g=360  i0=455mA  i1=309mA  v0=60   v1=80
t=0.4s  g=720  i0=390mA  i1=233mA  v0=400  v1=420
t=0.7s  g=1000 i0=277mA  i1=255mA  v0=800  v1=740
t=0.9s  g=1000 i0=244mA  i1=174mA  v0=900  v1=860
t=1.1s  g=1000 i0=233mA  i1=204mA  v0=980  v1=920
t=1.3s  g=1000 i0=255mA  i1=200mA  v0=980  v1=1000
t=1.6s  g=1000 i0=258mA  i1=204mA  v0=980  v1=980
t=1.8s  g=1000 i0=247mA  i1=247mA  v0=1000 v1=900
t=2.0s  g=1000 i0=222mA  i1=182mA  v0=1000 v1=1000
...
t=3.1s  g=1000 i0=255mA  i1=178mA  v0=1000 v1=1000
Stopping...
Done — exited cleanly.
```

Idle current ~200–450 mA (well below 2000 mA limit) → no spurious de-rate, `g`
ramps cleanly to setpoint 1000. Current readings populated in `last_current_ma`.
Exits cleanly.

### Documentation Updates

- Docstring on `RatioDrive.__init__`: document `current_limit_ma` parameter,
  its default (None = disabled), its effect, and the transaction-budget
  implication (reduced effective rate when active).
- Inline comment in `velocity_chart.py` near `RatioDrive(...)` explaining the
  `current_limit_ma` argument and the conservative default of 2000 mA.
