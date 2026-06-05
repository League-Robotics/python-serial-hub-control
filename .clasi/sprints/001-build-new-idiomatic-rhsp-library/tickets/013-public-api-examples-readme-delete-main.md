---
id: '001-013'
title: Public API, examples, README, delete __main__.py
status: open
use-cases:
  - SUC-009
  - UC-023
  - UC-024
depends-on:
  - '001-012'
---

# Ticket 013: Public API, examples, README, delete __main__.py

## Description

Finalize the public API surface in `src/rhsp/__init__.py`, rewrite the hardware
test scripts as `examples/` (skipped without a hub), delete `src/rhsp/__main__.py`
(the dead GUI), and update the README to remove the `python -m rhsp` claim and
document the new snake_case API.

After this ticket the library is feature-complete and testable end-to-end
(minus the packaging flip in ticket 014).

## Acceptance Criteria

- [ ] `src/rhsp/__init__.py` re-exports:
  - `connect`, `enumerate_hubs` from `discovery`
  - `Hub` from `hub`
  - `RhspError`, `ProtocolError`, `ChecksumError`, `RhspTimeoutError`, `NackError` from `errors`
  - `MotorMode`, `ZeroPowerBehavior`, `NackCode`, `ModuleStatusBits`, `MotorStatusBits` from `enums`
  - `BulkInputData`, `ModuleStatus` from `devices.bulk`
  - `Session` from `session` (for advanced use)
- [ ] `from rhsp import connect, Hub, MotorMode, NackError` succeeds
- [ ] `src/rhsp/__main__.py` is deleted (the dead tkinter GUI)
- [ ] `python -m rhsp` raises `ModuleNotFoundError` or a clean "no entry point" error — not a tkinter import error
- [ ] `examples/` directory created with at minimum:
  - `examples/test_motor.py` — motor power + velocity sweep; skips if no hub
  - `examples/test_servo.py` — servo sweep; skips if no hub
  - `examples/test_dio.py` — DIO direction + read/write; skips if no hub
  - `examples/test_sensors.py` — color + distance + IMU read; skips if no hub
- [ ] Each example script uses the new snake_case API (not the camelCase vendor API)
- [ ] Each example calls `enumerate_hubs()`; if empty, raises `pytest.skip("no hub found")`
- [ ] `uv run pytest examples/` exits 0 on a machine with no hub (all tests skipped)
- [ ] `README.md` updated:
  - Removes `python -m rhsp` claim
  - Shows the new `connect()` / `hub.motors[0].set_power(16000)` usage pattern
  - Notes the breaking API change (camelCase → snake_case)
  - Notes Python ≥ 3.13 requirement

## Implementation Plan

### Approach

`__init__.py` uses explicit `from .module import Name` re-exports. No wildcard
imports. All re-exported names are also listed in `__all__`.

Delete `src/rhsp/__main__.py` with `git rm`.

`examples/` scripts are standard pytest files or standalone scripts with:
```python
import pytest, rhsp
hubs = rhsp.enumerate_hubs()
if not hubs:
    pytest.skip("no hub found")
hub = rhsp.connect(hubs[0])
```

README update is a targeted rewrite of the Quick Start section and the
installation/usage example. The breaking API change is announced prominently.

### Files to Create

- `examples/__init__.py` — empty (makes examples a package for pytest discovery)
- `examples/test_motor.py`
- `examples/test_servo.py`
- `examples/test_dio.py`
- `examples/test_sensors.py`

### Files to Modify

- `src/rhsp/__init__.py` — finalize public re-exports
- `README.md` — update Quick Start and remove `python -m rhsp`

### Files to Delete

- `src/rhsp/__main__.py` — dead tkinter GUI

### Testing Plan

- `uv run pytest tests/` must still pass (no regression).
- `uv run pytest examples/` on a no-hub machine: assert all tests have status `SKIPPED`, exit code 0.
- Import test: `python -c "from rhsp import connect, Hub, MotorMode, NackError; print('ok')"` prints `ok`.
- `python -m rhsp` does not import tkinter or call `sys.exit()`.

### Documentation Updates

- `README.md` — Quick Start section rewritten.
