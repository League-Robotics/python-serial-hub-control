---
id: '001'
title: 'Hub-managed LED: heartbeat re-assert + status-clear on connect'
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: hub-managed-led-reassert.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hub-managed LED: heartbeat re-assert + status-clear on connect

## Description

`hub.set_led_color()` is currently a one-shot command. If the firmware
re-latches a `KeepAliveTimeout | FailSafe` flag (which happens normally on
every heartbeat interval), the LED reverts to blinking blue. Users must
manually re-send the pattern every ~0.8 s to keep a steady colour.

This ticket makes `set_led_color()` fire-and-forget by storing the desired
LED pattern on the `Hub` object and having the keep-alive heartbeat re-assert
it each tick. It also clears the latched `KeepAliveTimeout | FailSafe` status
during `init_peripherals()` so the LED is usable immediately after connect
with no blinking-blue window.

Non-routine faults (over-temp, battery-low) are surfaced via a warning log and
an optional `on_error` callback rather than silently cleared. Only the routine
`KeepAliveTimeout | FailSafe` latch is cleared and triggers LED re-assert.

**Key implementation facts from Sprint 001 (verified on fw 1.8.2):**
- Wire order for pattern steps is `[T, B, G, R]` (duration, blue, green, red).
- `SetModuleLEDPattern` (0x7F0C) works; `SetModuleLEDColor` (0x7F0A) is a no-op.
- `session.get_module_status(clear=True)` drains the latch before LED commands.

## Acceptance Criteria

- [ ] `Hub` stores `_desired_led_pattern: list[tuple[int,int,int,int]] | None`,
      initialised to `None` in `__init__`.
- [ ] `hub.set_led_color(r, g, b)` stores the 16-step solid pattern as
      `_desired_led_pattern` AND sends it immediately (preserving the existing
      correct `[T, B, G, R]` byte order and current clear-status behaviour).
- [ ] `hub.clear_led_color()` sets `_desired_led_pattern = None`; the LED
      returns to firmware default on the next heartbeat (no immediate send
      required).
- [ ] `hub.init_peripherals()` calls `session.get_module_status(clear=True)`
      (or equivalent) before motor/servo init commands, so the connect-time
      `KeepAliveTimeout | FailSafe` latch is drained before any LED command.
- [ ] Keep-alive heartbeat: after each `keep_alive()` call, if
      `_desired_led_pattern is not None`, read module status; if ONLY
      `KeepAliveTimeout | FailSafe` bits are set (no non-routine bits), clear +
      re-assert the stored pattern.
- [ ] If any non-routine fault bit (over-temp, battery-low) is present in the
      status word, the heartbeat logs a WARNING, calls `on_error(fault_flags)`
      if an `on_error` callback was provided, and does NOT clear/re-assert the
      pattern (per OQ1 resolution).
- [ ] An optional `on_error` hook is accepted by `Hub` (or `start_keepalive`)
      and invoked with the raw fault flags when a non-routine fault is detected.
- [ ] FakeHub unit test (a): heartbeat re-asserts the stored pattern after a
      simulated `KeepAliveTimeout` latch; verify `set_module_led_pattern` is
      called again without caller intervention.
- [ ] FakeHub unit test (b): `init_peripherals()` triggers a
      `get_module_status(clear=True)` call before any motor/servo init; verify
      via call-order assertions on FakeHub.
- [ ] FakeHub unit test (c): simulated over-temp or battery-low fault flag is
      NOT cleared; `set_module_led_pattern` is NOT re-called; the warning is
      logged; `on_error` callback is invoked.
- [ ] Live-hub validation criterion: after a single `hub.set_led_color()` call,
      the LED holds the requested solid colour for >= 10 s with no caller
      re-sends. LED is non-blinking immediately after `init_peripherals()`.

## Implementation Plan

### Approach

Extend `hub.py` only. No new modules, no new threads. Thread safety is
provided by the existing `Session` `RLock` (which guards all session I/O) and
Python's GIL (which is sufficient for a single-writer / single-reader reference
swap on `_desired_led_pattern`).

### Files to Modify

- `src/rhsp/hub.py`
  - Add `self._desired_led_pattern: list[tuple[int,int,int,int]] | None = None`
    to `Hub.__init__`.
  - Modify `set_led_color(r, g, b)`: after building the 16-step pattern, store
    it as `self._desired_led_pattern = pattern`, then call the existing send
    path.
  - Add `clear_led_color(self) -> None`: sets `self._desired_led_pattern = None`.
  - Modify `init_peripherals()`: insert
    `self.session.get_module_status(clear=True)` (or the equivalent low-level
    call) as the first step, before any motor/servo setup.
  - Extend the keep-alive loop body (inside `_loop` / `start_keepalive`): after
    `keep_alive()`, if `self._desired_led_pattern is not None`, call
    `get_module_status(clear=False)` to read the current flags. Inspect:
    - If status has ONLY `KeepAliveTimeout | FailSafe` bits: call
      `get_module_status(clear=True)` to drain, then re-call the LED pattern
      send path with `self._desired_led_pattern`.
    - If status has any non-routine bit (define a `_NON_ROUTINE_FAULT_MASK`
      constant for over-temp and battery-low bits): log
      `WARNING: non-routine hub fault 0x{flags:04x}` and call
      `self._on_error(flags)` if `self._on_error` is not None. Do not clear or
      re-assert.
  - Accept `on_error` kwarg in `start_keepalive()` (or `__init__`) and store
    as `self._on_error`.

### Files to Create

- `tests/test_hub_led.py` (or extend `tests/test_hub.py` if it exists):
  - Import `FakeHub` from `tests/fakehub.py`.
  - Test (a): call `hub.set_led_color()`, simulate a tick where module status
    returns `KeepAliveTimeout | FailSafe` only, assert the heartbeat re-asserts
    the pattern (check `FakeHub` call log).
  - Test (b): call `init_peripherals()`, assert `get_module_status(clear=True)`
    appears before motor/servo init calls in the call log.
  - Test (c): simulate a tick where status has an over-temp bit; assert
    `set_module_led_pattern` is NOT called again; assert `on_error` receives
    the fault flags; assert a WARNING was emitted (use `pytest.warns` or log
    capture).

### Testing Plan

- Run `uv run pytest` — must pass all existing tests with no regressions.
- New tests in `tests/test_hub_led.py` cover all three FakeHub scenarios above.
- Live-hub validation: run `examples/test_led.py` (or extend
  `examples/test_motor.py`) against the hub at `/dev/cu.usbserial-DQ3M375O`;
  observe LED holds colour >= 10 s; observe no blinking blue on connect.

### Documentation Updates

- Update docstring on `Hub.set_led_color` to describe the fire-and-forget
  semantics.
- Add docstring to `Hub.clear_led_color`.
- Update docstring on `Hub.init_peripherals` to note the status-clear step.
