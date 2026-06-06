---
id: '002'
title: Hub-managed LED + two-layer motor velocity control
status: done
branch: sprint/002-hub-managed-led-two-layer-motor-velocity-control
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- hub-managed-led-reassert.md
- plan-two-layer-motor-velocity-control-on-hub-pid-host-side-ratio-coordinator.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 002: Hub-managed LED + two-layer motor velocity control

## Goals

1. Make `hub.set_led_color()` fire-and-forget by having the keep-alive
   heartbeat re-assert the stored LED pattern each tick and by clearing the
   latched `KeepAliveTimeout | FailSafe` status on connect.
2. Deliver a two-layer motor velocity control system: a thin
   `VelocityController` seam with a `HubVelocityController` implementation
   (inner loop = hub's onboard `CONSTANT_VELOCITY` PID), plus a `RatioDrive`
   coordinator (outer loop) that maintains a configurable speed ratio between
   wheels using a deterministic "cap-to-slowest" governor.

## Problem

**LED**: `hub.set_led_color()` is a one-shot. If the keep-alive heartbeat
lapses or the firmware re-latches a timeout the LED reverts to blinking blue.
The user must manually re-send the pattern every ~0.8 s. Also, the latched
`KeepAliveTimeout | FailSafe` status on fresh connect keeps the LED blinking
blue even during normal operation.

**Motor velocity**: The library exposes the hub's velocity PID but there is no
host-side layer that coordinates multiple wheels. If one wheel cannot maintain
its target speed (load, slip, stall), the caller has no mechanism to
automatically de-rate the other wheels to preserve a desired speed ratio.

## Solution

**LED**: Store the desired LED pattern on the `Hub` object. After clearing the
latched status on connect, the keep-alive heartbeat re-asserts the stored
pattern every tick. Real faults (over-temp, battery-low, persistent
KeepAliveTimeout) are inspected before each re-assert and surfaced rather than
swallowed.

**Motor velocity**: New `src/rhsp/control.py` module. `VelocityController` ABC
provides the seam; `HubVelocityController` wraps the hub's onboard PID.
`RatioDrive` runs a daemon thread at a configurable rate, calls
`hub.bulk_input()` each tick, reads measured velocities, applies the
cap-to-slowest governor with slew limiting, and emits new targets. The governor
moves a single common scale `g`, keeping the ratio exact by construction.

## Success Criteria

- [ ] `hub.set_led_color()` stays steady without caller re-sends during normal
      operation; verified on the live hub.
- [ ] LED clears immediately after connect (no blinking-blue window).
- [ ] Real fault flags are not masked by repeated status clears.
- [ ] `RatioDrive({0:1.0, 1:0.5}).set_speed(1200)` drives two wheels at
      measured ~1200 : ~600 counts/s ratio on the live rig.
- [ ] Under induced load on one wheel, the other slows proportionally (ratio
      preserved); recovery works when load releases.
- [ ] Hardware-free `uv run pytest` suite passes with >= 434 tests (no
      regressions); new unit tests cover all acceptance criteria.

## Scope

### In Scope

- LED hub-management: `_desired_led_pattern` on `Hub`; status-clear on connect
  in `hub.init_peripherals()`; keep-alive heartbeat re-assert; fault inspection
  before re-assert; `hub.clear_led_color()` reset path.
- `src/rhsp/control.py`: `VelocityController` ABC, `HubVelocityController`,
  `RatioDrive` (full governor, threading, lifecycle, `update(bulk)` sync entry
  point).
- `motor.set_velocity_pid(p, i, d)` / `motor.get_velocity_pid()` convenience
  methods.
- Public exports in `__init__.py`: `RatioDrive`, `VelocityController`.
- Hardware-free unit tests: extensions to existing test files +
  `tests/test_control.py`.
- Live-hub validation examples: `examples/test_ratio_drive.py` with capability
  probe; LED behaviour validated via `examples/test_motor.py` or a dedicated
  `examples/test_led.py`.

### Out of Scope

- Host-side PID fallback velocity controller (seam is built; fallback deferred).
- Multi-hub / child-hub LED management.
- Position-control coordinator.
- Any protocol or codec changes.
- Sensor modules (color, distance, IMU).

## Test Strategy

**Hardware-free (gate)**: `uv run pytest` must pass. New unit tests use
`FakeHub` + synthetic `BulkInputData` instances to exercise all governor
logic, LED heartbeat assertion, and lifecycle without a physical hub.

**Live-hub validation (required per standing directive)**: Each
hardware-touching ticket requires a live-hub validation step against the REV
hub at `/dev/cu.usbserial-DQ3M375O` (module @2, fw 1.8.2; 12V battery
connected for motor actuation).

## Architecture Notes

- `Session` already holds an `RLock`; the `RatioDrive` loop thread and the
  keep-alive heartbeat thread coexist safely without additional wire locking.
- LED pattern is stored as `_desired_led_pattern: list[tuple] | None` on `Hub`.
  The keep-alive loop reads this attribute; a single writer (the caller thread)
  and a single reader (the heartbeat thread) make GIL protection sufficient for
  the reference swap.
- `RatioDrive` guards its own mutable state with an internal `RLock` for
  `set_speed / set_weights` atomic writes from the caller thread.
- `control.py` depends only on `rhsp.enums`, `rhsp.devices.bulk`,
  `rhsp.devices.motor`, and `rhsp.hub`. No protocol or codec coupling.
- The governor scale `g` is a pure float magnitude; direction lives entirely in
  the weight signs. The int16 clamp happens only at the point of emission to
  `VelocityController.command()`.

## GitHub Issues

(None linked to GitHub yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Hub-managed LED: heartbeat re-assert + status-clear on connect | — |
| 002 | Motor PID convenience + VelocityController ABC + HubVelocityController | 001 |
| 003 | RatioDrive governor, threading, and unit tests | 002 |
| 004 | Public API exports + live-hub validation + examples | 003 |

Tickets execute serially in the order listed.
