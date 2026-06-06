---
id: '002'
title: Motor velocity-PID convenience + VelocityController ABC + HubVelocityController
status: done
use-cases:
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: plan-two-layer-motor-velocity-control-on-hub-pid-host-side-ratio-coordinator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Motor velocity-PID convenience + VelocityController ABC + HubVelocityController

## Description

This ticket adds the foundation of the host-side velocity control layer:

1. **Motor PID convenience methods** on `Motor`: thin wrappers over the already-
   existing `session.set_motor_pid_coefficients` (line 694) and
   `session.get_motor_pid_coefficients` (line 725) — no new session methods needed
   (OQ2 resolved).
2. **`VelocityController` ABC** in a new `src/rhsp/control.py`: the thin seam that
   `RatioDrive` (ticket 003) will drive, and that advanced users can implement for
   alternative inner loops.
3. **`HubVelocityController`** in the same `control.py`: the one concrete
   implementation, wrapping the hub's onboard `CONSTANT_VELOCITY` PID for a single
   motor channel.

`RatioDrive` is explicitly out of scope for this ticket — it is built in ticket 003.

## Acceptance Criteria

- [x] `motor.set_velocity_pid(p: float, i: float, d: float) -> None` added to
      `Motor`: delegates to
      `session.set_motor_pid_coefficients(self.address, self.channel, ClosedLoopMode.VELOCITY, p, i, d)`.
- [x] `motor.get_velocity_pid() -> tuple[float, float, float]` added to `Motor`:
      delegates to
      `session.get_motor_pid_coefficients(self.address, self.channel, ClosedLoopMode.VELOCITY)`.
- [x] `VelocityController` ABC defined in `src/rhsp/control.py` with exactly these
      five members:
      - `attach(self) -> None`
      - `command(self, target_counts_s: int) -> None`
      - `measured(self, bulk) -> int`
      - `detach(self, disable: bool = True) -> None`
      - `available: bool` (abstract property)
- [x] `HubVelocityController(hub: Hub, channel: int)` in `src/rhsp/control.py`:
      - `attach()`: records the motor's current mode; calls
        `motor.set_mode(MotorMode.CONSTANT_VELOCITY, float_at_zero=True)`;
        calls `motor.set_target_velocity(0)` (required by fw before enable);
        calls `motor.enable()`.
      - `command(target_counts_s)`: clamps argument to int16 range
        `[-32767, 32767]`, calls `motor.set_target_velocity(clamped)`.
      - `measured(bulk) -> int`: returns `motor.get_velocity(bulk)`.
      - `detach(disable=True)`: if `disable`, calls `motor.disable()`;
        restores the motor mode that was recorded in `attach()`.
      - `available: bool`: property returning `True` (hub always supports
        `CONSTANT_VELOCITY` on fw 1.8.2; the live example does a runtime probe).
- [x] FakeHub unit tests in `tests/test_control.py`:
      - `set_velocity_pid` / `get_velocity_pid`: assert correct delegation to
        session methods with correct address, channel, and `ClosedLoopMode.VELOCITY`;
        assert Q16 passthrough is handled by the session layer (no double-conversion).
      - `HubVelocityController.attach()`: assert `set_mode(CONSTANT_VELOCITY,
        float_at_zero=True)` and `enable()` are called; assert prior mode is stored.
      - `HubVelocityController.command(t)`: assert `set_target_velocity` is called
        with the clamped value; test boundary values 32767, -32767, 32768 (clamped),
        -32768 (clamped).
      - `HubVelocityController.measured(bulk)`: assert return value matches
        `motor.get_velocity(bulk)` for a synthetic bulk snapshot.
      - `HubVelocityController.detach(disable=True)`: assert `motor.disable()`
        called and prior mode restored; test `detach(disable=False)` does not
        disable the motor.
      - `HubVelocityController.available`: assert `True`.

## Implementation Plan

### Approach

Two files touched. `motor.py` gets two thin methods. `control.py` is created
from scratch with zero protocol coupling — it imports only `Hub`, `Motor`,
`BulkInputData`, and `enums`.

### Files to Modify

- `src/rhsp/devices/motor.py`
  - Add `set_velocity_pid(self, p: float, i: float, d: float) -> None`.
    Body: one delegation line to `self.session.set_motor_pid_coefficients(...)`.
  - Add `get_velocity_pid(self) -> tuple[float, float, float]`.
    Body: one delegation line to `self.session.get_motor_pid_coefficients(...)`.
  - Import `ClosedLoopMode` from `rhsp.enums` if not already imported.

### Files to Create

- `src/rhsp/control.py`
  - Module docstring describing the two-layer design.
  - `from abc import ABC, abstractmethod`
  - `from rhsp.enums import MotorMode, ClosedLoopMode`
  - `from rhsp.devices.bulk import BulkInputData`
  - `from rhsp.devices.motor import Motor`
  - `from rhsp.hub import Hub` (use `TYPE_CHECKING` guard and string annotation if
    circular import arises; `Hub` is only needed for `hub.motors[channel]` access
    in `__init__`)
  - `class VelocityController(ABC)`: abstract base as specified in acceptance
    criteria.
  - `class HubVelocityController(VelocityController)`: concrete implementation
    as specified. Constructor takes `hub: Hub, channel: int`; resolves
    `self._motor = hub.motors[channel]`.

- `tests/test_control.py`
  - Import `FakeHub` from `tests/fakehub.py`.
  - Fixture: `fake_hub` with two motors (channels 0 and 1).
  - Test groups: motor PID convenience, HubVelocityController methods (per
    acceptance criteria above).

### Testing Plan

- Run `uv run pytest` — all existing tests plus new tests in `test_control.py`
  must pass.
- No live-hub step for this ticket — the live validation for velocity control
  is performed in ticket 004 via `examples/test_ratio_drive.py`.

### Documentation Updates

- Docstrings on `motor.set_velocity_pid` and `motor.get_velocity_pid` noting
  the Q16 encoding is handled by the session layer.
- Module docstring on `control.py` summarising the two-layer design and the
  intended composition pattern.
- Docstrings on `VelocityController` (contract) and `HubVelocityController`
  (usage, int16 clamp note).
