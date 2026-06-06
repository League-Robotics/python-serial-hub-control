---
id: '004'
title: Public API exports + live-hub validation example
status: done
use-cases:
- SUC-004
- SUC-005
depends-on:
- '003'
github-issue: ''
issue: plan-two-layer-motor-velocity-control-on-hub-pid-host-side-ratio-coordinator.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Public API exports + live-hub validation example

## Description

Closes out the sprint by:

1. Exporting `RatioDrive` and `VelocityController` from `src/rhsp/__init__.py`
   so they are reachable via `import rhsp; rhsp.RatioDrive(...)`.
2. Creating `examples/test_ratio_drive.py` — the live-hub validation script that
   exercises the full two-layer velocity control stack (capability probe, ratio
   runs, induced-bottleneck scenario, recovery). The script auto-skips when no hub
   is present, so it does not block the hardware-free CI gate.

This ticket is purely integrative: no new logic is written. All governor and
controller logic is complete in tickets 001-003.

## Acceptance Criteria

### Public API Exports

- [x] `src/rhsp/__init__.py`: `RatioDrive` and `VelocityController` are imported
      from `rhsp.control` and added to `__all__`.
- [x] `from rhsp import RatioDrive, VelocityController` works in a clean Python
      environment with the package installed editable.

### Live-Hub Validation Example

- [x] `examples/test_ratio_drive.py` auto-skips cleanly (prints a skip message
      and exits 0) when the hub at `/dev/cu.usbserial-DQ3M375O` is not available.
- [x] **Step 1 — Capability probe**: before running any ratio test, attempt to
      attach a single `HubVelocityController` on channel 0, command a non-zero
      target, read velocity from a bulk snapshot, and confirm the measurement is
      non-zero (within a short timeout). Print PASS/FAIL. Exit early if FAIL.
- [x] **Step 2 — 1:1 run**: `RatioDrive(hub, {0: 1.0, 1: 1.0}).set_speed(600)`;
      run for 3 s; log CSV rows of `target_scale, scale, v0, v1` at each
      `update()` tick; assert steady-state measured ratio `|v0/v1 - 1.0| < 0.10`.
- [x] **Step 3 — 1:0.5 run**: `RatioDrive(hub, {0: 1.0, 1: 0.5}).set_speed(1200)`;
      run for 3 s; log CSV; assert steady-state measured ratio
      `|v0/v1 - 2.0| < 0.10`.
- [x] **Step 4 — Induced-bottleneck scenario**: operator manually loads (grips)
      one wheel; the script observes `drive.saturated` becomes `True` and
      `drive.scale` drops below `target_scale`; then release — observe `scale`
      recovers toward `target_scale` within a configurable timeout. The script
      logs a CSV and prints PASS/FAIL for the de-rate and recovery assertions.
      (Because operator intervention is required, the script may prompt and wait
      rather than automate — that is acceptable.)
- [x] **Finally block**: `drive.close()` then `hub.fail_safe()` always called,
      even on early exit or exception.

### Regression Gate

- [x] `uv run pytest` passes with >= 434 tests and no regressions.
      (`test_ratio_drive.py` is in `examples/`, not `tests/`, so it is not
      collected by pytest.)

## Implementation Plan

### Approach

Two very small file changes plus one new example script.

### Files to Modify

- `src/rhsp/__init__.py`
  - Add `from rhsp.control import RatioDrive, VelocityController`.
  - Add `"RatioDrive"` and `"VelocityController"` to `__all__`.

### Files to Create

- `examples/test_ratio_drive.py`
  - Top-level: attempt `rhsp.connect(PORT)` in a `try`; if the port is absent,
    print a skip message and `sys.exit(0)`.
  - `PORT = "/dev/cu.usbserial-DQ3M375O"`.
  - Implement the 4-step validation sequence described in the acceptance
    criteria, wrapped in a `finally` block.
  - Log CSV to stdout (or a file `ratio_drive_log.csv`) with columns:
    `time_s, target_scale, scale, v0, v1, saturated`.
  - Use `hub.init_peripherals()` and `with hub:` context manager idiom as the
    outer scope, with `RatioDrive` in a nested `with drive:`.

### Testing Plan

- `uv run pytest` — must pass all existing tests (>= 434) plus any new import
  smoke tests (e.g., assert `rhsp.RatioDrive is not None`).
- Add one test in `tests/test_control.py` (or a new `tests/test_exports.py`)
  that simply does `from rhsp import RatioDrive, VelocityController` and asserts
  both are non-None, confirming the export wiring.
- Live-hub validation: run `python examples/test_ratio_drive.py` on the rig at
  `/dev/cu.usbserial-DQ3M375O` with 12V battery connected to both motor channels
  (0 and 1); observe CSV output and PASS lines; perform the induced-bottleneck
  step manually.

### Documentation Updates

- Add a brief note to `examples/README.md` (if it exists) or a comment block at
  the top of `test_ratio_drive.py` describing the hardware setup required
  (hub port, 12V battery, motors on channels 0 and 1).
- No changes to the main `README.md` are required unless the stakeholder
  specifically requests it.
