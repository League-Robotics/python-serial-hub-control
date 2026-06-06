---
id: 003-003
title: Validate bench tool (hardware-free gate + hardware checklist)
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- 003-002
issue: plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
---

# 003-003: Validate bench tool (hardware-free gate + hardware checklist)

## Description

Run the full validation sequence for the bench tool: first the automated
hardware-free gate (syntax check, optional dep install, pytest regression), then
the manual hardware checklist with the real hub on `/dev/cu.usbserial-DQ3M375O`
and the 12V battery connected. This ticket produces no new code — it is the
acceptance gate for Sprint 003.

If any hardware-free check fails, stop and report the failure (do not proceed to
hardware validation). If the hardware checks reveal a bug in the script, open a
new issue and fix it in a follow-on ticket or a direct patch.

## Acceptance Criteria

**Hardware-free (automated)** — all verified by team-lead 2026-06-06:
- [x] `uv run python -c "import ast, pathlib; ast.parse(pathlib.Path('examples/velocity_chart.py').read_text())"` exits 0. → "syntax ok"
- [x] `uv run --extra bench python -c "import matplotlib, numpy; print('ok')"` exits 0. → "imports ok"
- [x] `uv run pytest` exits 0 with zero new failures. → 537 passed
- [x] Confirm `examples/velocity_chart.py` does not appear in pytest collection
      output (`uv run pytest --collect-only | grep velocity_chart` returns empty). → empty

**Hardware (manual — requires hub + 12V battery)**:
- [ ] `uv run --extra bench python examples/velocity_chart.py --speed 1000 --ratio 1.0`
      opens a matplotlib window with three panels.
- [ ] SPACE key: both motors spin; strip charts fill with live traces; both
      traces converge to the dashed setpoint line within ~5 s.
- [ ] Phase plot: operating point dot tracks along the y=x (ratio=1.0)
      reference line.
- [ ] Hand-load motor A: strip chart for A droops below setpoint; phase dot
      slides down along the ratio line (not diagonally off it); recovers on
      release.
- [ ] Second SPACE: motors stop cleanly; title updates to STOPPED state;
      streaming pauses.
- [ ] Q key: window closes, motors fail-safed, process exits with code 0.
- [ ] Test with `--ratio 0.5`: phase plot reference line has slope 0.5; motor B
      runs at half the commanded scale of motor A; ratio-preservation under load
      is visible in the phase plot.

## Implementation Plan

### Approach

Execute the hardware-free checks in order. If all pass, proceed to the hardware
checklist. Document results inline (check boxes above). If a hardware test
fails, create an issue in `.clasi/issues/` describing the observed vs. expected
behavior.

### Hardware Setup

1. Connect hub via USB: `/dev/cu.usbserial-DQ3M375O`.
2. Connect 12V battery to hub power input (required for motor movement).
3. Confirm hub is enumerated: `uv run python -c "import rhsp; print(rhsp.enumerate_hubs())"`.

### Validation Commands

```bash
# Hardware-free gate
uv run python -c "import ast, pathlib; ast.parse(pathlib.Path('examples/velocity_chart.py').read_text()) and print('syntax ok')"
uv run --extra bench python -c "import matplotlib, numpy; print('imports ok')"
uv run pytest
uv run pytest --collect-only | grep velocity_chart  # should return nothing

# Hardware validation (with hub + battery)
uv run --extra bench python examples/velocity_chart.py --speed 1000 --ratio 1.0
uv run --extra bench python examples/velocity_chart.py --speed 1000 --ratio 0.5
```

### Files to Modify

None.

### Documentation Updates

None. Validation results are recorded via ticket acceptance criteria check boxes.
