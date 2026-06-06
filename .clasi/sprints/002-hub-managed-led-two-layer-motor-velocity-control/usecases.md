---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 002 Use Cases

---

## SUC-001: Fire-and-forget LED colour during normal operation

**Parent**: UC-004 (motor/peripheral control)

- **Actor**: Robot developer
- **Preconditions**: Hub is connected; keep-alive heartbeat is running (`with hub:`).
- **Main Flow**:
  1. Developer calls `hub.set_led_color(r, g, b)` once after connecting.
  2. The hub LED immediately shows the requested colour (no blinking blue).
  3. The keep-alive heartbeat fires at each interval and re-asserts the stored
     pattern without any further caller action.
  4. The LED stays steady for the duration of the session.
- **Postconditions**: LED remains at the requested colour without caller polling.
- **Acceptance Criteria**:
  - [ ] `hub.set_led_color()` stores the pattern on the `Hub` object.
  - [ ] The keep-alive heartbeat reads and re-asserts the stored pattern each tick.
  - [ ] The LED stays solid (no blinking) for at least 10 s on the live hub
        without caller re-sends.

---

## SUC-002: Clear latched status on connect so LED is usable immediately

**Parent**: UC-001 (connect to hub)

- **Actor**: Robot developer / library
- **Preconditions**: Hub is freshly connected; latched `KeepAliveTimeout | FailSafe`
  flags are set (normal after power-on or USB reconnect).
- **Main Flow**:
  1. `hub.init_peripherals()` is called during connection setup.
  2. The library clears the latched `KeepAliveTimeout | FailSafe` status.
  3. A subsequent `hub.set_led_color()` call takes effect immediately.
- **Postconditions**: The LED is not blinking blue after `init_peripherals()`
  completes; the hub obeys LED commands.
- **Acceptance Criteria**:
  - [ ] `init_peripherals()` calls `GetModuleStatus(clear=True)` to drain
        latched timeout/fail-safe flags.
  - [ ] `set_led_color()` called immediately after `init_peripherals()` shows
        the requested colour on the live hub (no blinking-blue window).

---

## SUC-003: Real fault flags are surfaced, not masked

**Parent**: UC-016 (error handling)

- **Actor**: Robot developer / monitoring code
- **Preconditions**: Hub is running with the heartbeat; a real fault occurs
  (over-temp, battery-low, genuine keep-alive lapse from a blocked thread).
- **Main Flow**:
  1. The heartbeat reads module status before re-asserting the LED pattern.
  2. If a real fault flag (over-temp, battery-low, persistent
     `KeepAliveTimeout` that the heartbeat itself did not cause) is detected,
     the heartbeat does NOT clear-and-re-assert silently; it surfaces the fault.
  3. The developer's `on_error` callback or the exception propagation makes the
     fault observable.
- **Postconditions**: Real faults are not hidden behind repeated status clears.
- **Acceptance Criteria**:
  - [ ] FakeHub unit test: inject an over-temp flag; verify the heartbeat does
        not silently swallow it.
  - [ ] The LED re-assert logic inspects the status word and raises / calls
        `on_error` for non-routine fault flags.

---

## SUC-004: Single-wheel velocity control via hub onboard PID

**Parent**: UC-005 (motor control)

- **Actor**: Robot developer
- **Preconditions**: Hub connected; motor channel configured; `CONSTANT_VELOCITY`
  mode available on fw 1.8.2.
- **Main Flow**:
  1. Developer creates a `HubVelocityController(hub, channel)`.
  2. Calls `attach()` — sets `CONSTANT_VELOCITY` mode, enables the motor.
  3. Calls `command(target_counts_s)` to set a velocity setpoint.
  4. Reads `measured(bulk)` from a `BulkInputData` snapshot to observe actual
     velocity.
  5. Calls `detach(disable=True)` to stop and disable.
- **Postconditions**: Motor tracks the setpoint using the hub's onboard PID.
- **Acceptance Criteria**:
  - [ ] `HubVelocityController.attach()` sets `CONSTANT_VELOCITY` mode with
        `float_at_zero=True` and enables the motor.
  - [ ] `command(t)` calls `motor.set_target_velocity(t)`.
  - [ ] `measured(bulk)` returns `motor.get_velocity(bulk)`.
  - [ ] `detach(disable=True)` disables the motor and restores prior mode.
  - [ ] `available` probes that the hub supports `CONSTANT_VELOCITY` (runtime
        capability check in the live example).
  - [ ] Unit tests with FakeHub pass for all methods.

---

## SUC-005: Coordinated multi-wheel ratio drive with automatic de-rating

**Parent**: UC-005 (motor control)

- **Actor**: Robot developer
- **Preconditions**: Hub connected; two motor channels (0, 1) with encoders;
  `HubVelocityController` operational; 12V battery connected.
- **Main Flow**:
  1. Developer creates `RatioDrive(hub, {0: 1.0, 1: 0.5})` and calls `start()`.
  2. Calls `set_speed(1200)` — wheels target 1200 and 600 counts/s respectively.
  3. Wheels spin; `RatioDrive` reads bulk input each tick and emits corrected
     targets using the cap-to-slowest governor.
  4. If one wheel slips or stalls (can't reach its target), the governor lowers
     the common scale `g` so the other wheel also slows — ratio is preserved.
  5. When the loaded wheel recovers, `g` climbs back toward the setpoint under
     slew limiting.
  6. `close()` zeros both wheels, detaches controllers, and stops the loop thread.
- **Postconditions**: Both wheels ran at the commanded ratio; the session is
  clean for reuse.
- **Acceptance Criteria**:
  - [ ] Ratio invariance: `commanded_targets[0] / commanded_targets[1] == w0 / w1`
        at every governor step (unit test).
  - [ ] Cap-to-slowest: wheel 1 stuck at 50% of target -> `scale` converges to
        `v1 / |w1|`; lagging wheel never boosted above its commanded target
        (unit test).
  - [ ] Slew limiting: `scale` drops <= `max_accel * dt` per step on sudden
        bottleneck; climbs <= `recovery_accel * dt` per step on clear (unit test).
  - [ ] Transient accel lag does not collapse `scale` (symmetric lag during
        ramp stays below `sat_margin`; unit test).
  - [ ] Recovery and hysteresis: `saturated` flag does not chatter (unit test).
  - [ ] Edge cases: zero-weight, stall, negative weight, int16 clamp (unit tests).
  - [ ] Lifecycle: `start()/close()` idempotency; zero-on-close; mode restore
        (unit test with FakeHub).
  - [ ] `update(bulk)` is a synchronous entry point usable without threads
        (unit tests drive it directly).
  - [ ] Live validation on the 2-wheel rig: measured ratio within 10% of target
        in steady state; observable de-rating under induced load.
