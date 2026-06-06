---
id: '003'
title: velocity_chart bench tool
status: done
branch: sprint/003-velocity-chart-bench-tool
use-cases:
- SUC-001
- SUC-002
- SUC-003
issues:
- plan-velocity-chart-py-bench-tool-for-serialhubcontrol-rhsp.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 003: velocity_chart bench tool

## Goals

Build `examples/velocity_chart.py` — a live telemetry and visualization bench
tool for the RHSP library. The script provides real-time plots of motor velocity
via three matplotlib panels: two strip charts (measured velocity vs. time per
motor with a dashed commanded-setpoint reference line) and one X/Y phase plot
(motor-A velocity vs. motor-B velocity with a ratio reference line). It drives
motors through `RatioDrive` and reads measured velocities directly from
`hub.bulk_input()`.

## Problem

After Sprint 002 shipped `RatioDrive` and the two-layer motor velocity control
stack, there is no interactive tool to validate end-to-end behavior on real
hardware. Engineers cannot observe how the governor responds to load, whether the
ratio invariant is maintained under saturation, or whether the PID converges to
setpoint — they can only inspect log output or write one-off test scripts.

## Solution

Adapt the proven structure from the reference
`radio-robot-c/tests/bench/velocity_chart.py` (worker thread + queues +
main-thread matplotlib render loop, fresh-connection-per-SPACE) retargeted to
the RHSP API. Add matplotlib and numpy as an optional `bench` dependency group so
the core library stays at just `pyserial`. The script is placed in `examples/`
without a `test_` prefix so pytest does not collect it.

## Success Criteria

- `uv run --extra bench python examples/velocity_chart.py --speed 1000 --ratio 1.0`
  launches with three matplotlib panels on macOS.
- SPACE key starts motors and begins chart streaming; a second SPACE stops motors
  cleanly and pauses streaming.
- Q key exits cleanly (motors fail-safed via `with` block teardown, window closed).
- `uv run pytest` stays green — no new test failures, GUI file not collected.
- On real hardware (hub + 12V battery): strip charts converge to dashed setpoint;
  phase dot tracks the slope-=ratio reference line; hand-loading one motor causes
  the phase point to slide down the ratio line (governor holds ratio while de-rating).

## Scope

### In Scope

- `examples/velocity_chart.py` — the interactive bench tool.
- `pyproject.toml` — add `[project.optional-dependencies] bench = [...]` group.
- Hardware-free validation gate: syntax/import check + pytest regression.
- Hardware validation procedure (manual, requires hub + 12V battery).

### Out of Scope

- Changes to any `src/rhsp/` library module (no API changes, no new modules).
- Automated hardware CI (remains manual).
- Support for more than two motor channels in a single run.
- Any GUI framework other than matplotlib.

## Test Strategy

**Automated (hardware-free)**:
1. AST syntax parse of `examples/velocity_chart.py` confirms no syntax errors.
2. `uv run --extra bench python -c "import matplotlib, numpy"` confirms the
   optional dependency group resolves and installs correctly.
3. `uv run pytest` stays green (existing suite unaffected; GUI file not collected).

**Manual hardware validation** (requires `/dev/cu.usbserial-DQ3M375O` + 12V battery):
- SPACE → both motors spin, strip charts converge, phase dot tracks ratio line.
- Hand-load one motor → strip droops, phase point slides down ratio line, recovers.
- Second SPACE → motors stop, streaming pauses cleanly.
- Q → clean exit, window closes, motors fail-safed.

## Architecture Notes

The bench tool is entirely in `examples/` — it consumes the public RHSP API and
does not modify any library module. The only structural addition is the `bench`
optional dependency group in `pyproject.toml`. The tool's threading model (worker
thread + queues + main-thread matplotlib loop) is the same pattern as the
reference script and is safe alongside the hub's keep-alive heartbeat because
`Session` is RLock-guarded.

## GitHub Issues

(None linked to GitHub yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Add bench optional dependency group to pyproject.toml | — |
| 002 | Implement examples/velocity_chart.py | 001 |
| 003 | Validate bench tool (hardware-free gate + hardware checklist) | 002 |

Tickets execute serially in the order listed.
