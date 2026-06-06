---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 003 Use Cases

## SUC-001: Install and run the bench tool with optional dependencies
Parent: (no parent use case — new tooling capability)

- **Actor**: Engineer / developer
- **Preconditions**: RHSP library is installed or available via `uv run`. The
  `bench` optional dependency group does not yet exist in `pyproject.toml`.
- **Main Flow**:
  1. Engineer adds `matplotlib>=3.8` and `numpy>=1.26` to the `bench`
     optional-dependencies group in `pyproject.toml`.
  2. Engineer runs `uv run --extra bench python examples/velocity_chart.py`
     (with no hub connected, to verify startup and import chain).
  3. A matplotlib window opens showing three empty panels with the title
     "REV Hub motor velocity  [SPACE = connect]".
  4. Engineer presses Q to quit; the window closes cleanly with exit code 0.
- **Postconditions**: The `bench` group is installable; the script starts without
  import errors; `uv run pytest` remains green (no new test failures, script not
  collected by pytest).
- **Acceptance Criteria**:
  - [ ] `uv run --extra bench python -c "import matplotlib, numpy"` exits 0.
  - [ ] `uv run pytest` exits 0 with the same number of tests as before this
        sprint (GUI script not collected).
  - [ ] AST parse of `examples/velocity_chart.py` succeeds (no syntax errors).
  - [ ] Running the script without a hub opens the window and accepts Q to quit.

---

## SUC-002: Stream live motor velocities and observe convergence to setpoint
Parent: (no parent use case — new telemetry capability)

- **Actor**: Engineer validating motor velocity control on real hardware
- **Preconditions**: Hub connected on `/dev/cu.usbserial-DQ3M375O`; 12V battery
  connected to power the motors; `RatioDrive` is implemented (Sprint 002,
  confirmed shipped).
- **Main Flow**:
  1. Engineer runs `uv run --extra bench python examples/velocity_chart.py
     --speed 1000 --ratio 1.0`.
  2. Matplotlib window opens with three empty panels.
  3. Engineer presses SPACE; the worker thread opens a fresh hub connection,
     calls `hub.init_peripherals()`, starts `RatioDrive`, and begins polling
     `hub.bulk_input()`.
  4. Status title transitions: "CONNECTING" → "RUNNING".
  5. Strip chart for motor A (left panel, top) and motor B (left panel, bottom)
     fill with velocity traces. Both traces converge toward their dashed yellow
     setpoint lines within a few seconds.
  6. The phase plot (right panel) shows the operating point tracking along the
     y=x reference line (ratio 1.0).
  7. Engineer presses SPACE again; motors stop (RatioDrive.close() + hub
     context-manager exit); streaming pauses; title reverts to "STOPPED".
- **Postconditions**: Real-time velocity data was captured and rendered without
  crashes; motors were safely stopped via the `with` block teardown.
- **Acceptance Criteria**:
  - [ ] SPACE starts motor streaming; strip charts display live velocity data.
  - [ ] Both strip chart traces converge to the dashed setpoint within ~5 s
        at --speed 1000.
  - [ ] Phase plot dot tracks along the slope-1.0 reference line.
  - [ ] Second SPACE stops motors and pauses streaming without error.
  - [ ] Window title reflects correct state transitions (CONNECTING → RUNNING →
        STOPPED).

---

## SUC-003: Observe ratio preservation under motor load
Parent: (no parent use case — extends SUC-002)

- **Actor**: Engineer stress-testing the RatioDrive governor
- **Preconditions**: Same as SUC-002. Motors running at SPACE-start.
- **Main Flow**:
  1. With motors running (from SUC-002 step 3–6), engineer manually applies
     mechanical load to motor A (e.g., grips the wheel).
  2. Motor A's strip chart droops below the setpoint dashed line.
  3. The phase plot operating point slides down along the ratio reference line
     (not off it) — confirming the governor de-rates the pack while holding the
     B:A ratio in measured velocities.
  4. Engineer releases the load; motor A recovers toward the setpoint and the
     phase dot returns toward the commanded operating point.
  5. Engineer presses Q; the window closes cleanly and motors are fail-safed.
- **Postconditions**: The ratio-preservation property of RatioDrive is
  empirically visible in the phase plot.
- **Acceptance Criteria**:
  - [ ] Under load on motor A, the phase dot stays near the ratio reference line
        (within visual noise), not off it diagonally.
  - [ ] Motor A strip chart droops during load and recovers on release.
  - [ ] Q key exits cleanly from any state (running or stopped): window closes,
        motors are zeroed/disabled via RatioDrive.close() + hub context exit.
