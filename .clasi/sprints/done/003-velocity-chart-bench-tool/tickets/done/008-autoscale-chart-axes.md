---
id: 003-008
title: Auto-scale velocity_chart axes for arbitrary ratio/speed
status: done
use-cases:
- SUC-002
- SUC-003
depends-on: []
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Auto-scale velocity_chart axes for arbitrary ratio/speed

## Description

`examples/velocity_chart.py` currently sets a single fixed y-limit `±vmax`
(derived only from channel A's `--speed`) for both strip charts and the phase
plot. When `--ratio` is far from 1.0 — e.g. `--ratio 5.0 --speed 1000` puts
motor B's setpoint at ~5000 cnt/s — motor B's trace is off-scale the entire
run. The phase plot is equally broken because it uses `set_aspect("equal")`
with a shared `±vmax`, making the plot degenerate when the two axes have very
different natural ranges.

This ticket replaces all fixed-limit logic with per-axis auto-scaling that
derives limits from live data and commanded setpoints, so the tool is readable
for any `--ratio`/`--speed` combination. All changes are confined to
`examples/velocity_chart.py`; no library code is touched.

## Scope

Rendering and axis-limit logic in `examples/velocity_chart.py` only. No
changes to library modules, worker thread logic, queue protocol, or keyboard
handling introduced by earlier tickets (003-005 through 003-007).

## Design

### Strip chart auto-scaling (motor A and motor B independently)

Each strip chart's y-limits are recomputed inside the existing `_update()`
render callback on every frame. The formula for each chart:

```
half_span = max(max(|data_in_window|, |setpoint|), floor) * 1.15
ylim = (-half_span, +half_span)
```

- `data_in_window` is the full rolling deque visible in the chart at that
  moment (already maintained per channel).
- `setpoint` is the channel's current commanded velocity (motor A: `speed`;
  motor B: `speed * ratio`).
- `floor` prevents the axes from collapsing to zero when the motor is stopped;
  a sensible value is 50 cnt/s.
- The 1.15 factor gives ~15% headroom above the largest value seen.
- The two charts scale independently — motor A tracks ~`speed`, motor B tracks
  ~`speed * ratio`.

**Jitter suppression:** only widen limits, never shrink them faster than the
data warrants. Implement a simple hysteresis: recompute the desired
`half_span` each frame; only apply if the new value is more than 5% larger or
smaller than the current limit. This prevents the axes from breathing on every
frame when data is near-steady.

**`--vmax` override:** if the user supplied `--vmax` explicitly on the command
line, that value acts as a cap on `half_span` for both channels. Precedence:
explicit `--vmax` pins the maximum half-span; otherwise auto-scale is
unconstrained. Document this in `--help` (update the `--vmax` help string).

The dashed setpoint line and the zero line must remain visible at all times
(they are horizontal lines already drawn at known y-values, so this is
automatic once ylim contains those values).

### Phase plot auto-scaling

Remove `set_aspect("equal")`. Instead, derive x-limits and y-limits
independently:

```
x_half = max(max(|motor_A_in_window|), |speed|, floor) * 1.15
y_half = max(max(|motor_B_in_window|), |speed * ratio|, floor) * 1.15
phase_ax.set_xlim(-x_half, x_half)
phase_ax.set_ylim(-y_half, y_half)
```

This keeps the operating point visible and fills the phase plot regardless of
ratio. With `--ratio 1.0` the result is essentially a square; with
`--ratio 5.0` the y-axis is ~5x taller.

**Reference line `y = ratio * x`:** the line must span the full visible x
range. Re-draw (or update) it each frame using the current xlim:

```python
x0, x1 = phase_ax.get_xlim()
ref_line.set_xdata([x0, x1])
ref_line.set_ydata([ratio * x0, ratio * x1])
```

The same hysteresis rule applies to the phase plot limits.

### No structural changes

Worker thread, queue protocol, SPACE/Q handling, current-readout panel
(003-007), and CLI argument parsing structure are all preserved. The only
logical change is how `_update()` computes and applies axis limits.

## Acceptance Criteria

- [x] With `--ratio 5.0 --speed 1000`, the motor-B strip chart keeps its
  trace on-screen (auto-scaled to approximately 5000 + margin); the motor-A
  strip chart auto-scales independently to approximately 1000 + margin.
- [x] The phase plot shows the operating point and the `y = ratio * x`
  reference line on-screen for ratios from ~0.2 to ~5.0 (reference line
  spans the full visible x range).
- [x] `--ratio 1.0` (default) still looks correct — axes track data, layout
  is sensible.
- [x] Explicit `--vmax N` still caps the auto-scaled half-span at N for both
  strip charts; `--help` text for `--vmax` is updated to document this
  precedence.
- [x] Auto-scaling does not cause distracting jitter when data is steady
  (hysteresis suppresses sub-5% fluctuations).
- [x] The dashed setpoint line and the zero line remain visible in both strip
  charts at all times.
- [x] Syntax check passes:
  `uv run python -c "import ast,pathlib; ast.parse(pathlib.Path('examples/velocity_chart.py').read_text())"`
- [x] `uv run python examples/velocity_chart.py --help` exits 0 and shows
  updated `--vmax` help text.
- [x] `uv run pytest` stays green (583 tests pass — 9 new unit tests added); `velocity_chart.py` is
  not pytest-collected.
- [x] **Manual hardware gate (operator):** PASSED 2026-06-06 — ran `--ratio 5.0
  --speed 1000`; both strip charts and the phase plot stayed on-screen/readable
  while motors ran (stakeholder: "that part is working better"). (The high-ratio
  run also surfaced a separate governor limit-cycle, tracked in 003-009 — not an
  auto-scaling issue.)

## Implementation Plan

### Approach

All changes are in `examples/velocity_chart.py`. No library files are
modified. Work entirely within the existing `_update()` callback and the
`VelocityChart` (or equivalent) class that owns the matplotlib figure.

### Files to Modify

- `examples/velocity_chart.py` — axis-limit logic in `_update()`, phase plot
  setup (remove `set_aspect("equal")`), reference-line update logic, `--vmax`
  help string.

### Files to Create

None.

### Step-by-step

1. **Locate current limit-setting code** in `_update()`. Identify where
   `set_ylim(±vmax)` is called for each strip chart and for the phase axes.
2. **Add helper `_auto_half_span(deque, setpoint, floor, vmax_cap)`** that
   returns the desired half-span using the formula above. Keep it a plain
   function or private method — no new classes.
3. **Strip chart motor A:** replace fixed `set_ylim` with
   `_auto_half_span(deque_A, speed, floor=50, vmax_cap)` + hysteresis check.
4. **Strip chart motor B:** replace fixed `set_ylim` with
   `_auto_half_span(deque_B, speed*ratio, floor=50, vmax_cap)` + hysteresis.
5. **Phase plot:** remove `set_aspect("equal")`. Add per-axis limit
   computation using `_auto_half_span` for x and y independently. Update
   reference line x/y data from the new xlim each frame.
6. **`--vmax` help text:** update `add_argument("--vmax", ...)` help string to
   say it caps auto-scaling rather than setting a fixed limit.
7. **Hysteresis:** track current half-spans (e.g. instance attributes
   `_half_A`, `_half_B`, `_half_px`, `_half_py`) initialized to `floor`. Only
   update when the new desired value differs by >5%.

### Testing Plan

**Hardware-free (CI-safe):**
- `uv run python -c "import ast,pathlib; ast.parse(pathlib.Path('examples/velocity_chart.py').read_text())"` — must exit 0.
- `uv run python examples/velocity_chart.py --help` — must exit 0, `--vmax` help must mention auto-scaling precedence.
- `uv run pytest` — all 574 tests must pass; confirm `velocity_chart.py` does not appear in collected items.

**Manual hardware gate:**
- Operator connects live hub (see memory: hub on `/dev/cu.usbserial-DQ3M375O`).
- Run `uv run python examples/velocity_chart.py --ratio 5.0 --speed 1000`.
- Confirm motor-B strip chart trace stays on-screen; motor-A chart independently scaled.
- Confirm phase plot operating point and reference line visible.
- Repeat with `--ratio 0.5 --speed 1000`.
- Confirm `--ratio 1.0` (default) unchanged in feel.
- Gate signed off by team-lead with stakeholder before ticket is moved to done.

### Documentation Updates

Update `--vmax` help string in the argument parser (inline in the script).
No separate documentation files.
