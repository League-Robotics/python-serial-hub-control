---
status: pending
---

# Plan: Two-layer motor velocity control — on-hub PID + host-side ratio coordinator

## Context

The stakeholder wants coordinated wheel control in two layers:

1. **Inner loop** — hold each wheel at a target velocity using the REV
   Expansion Hub's *onboard* closed-loop velocity PID.
2. **Outer loop** — a **host-side coordinator** that maintains a fixed
   **speed ratio** between wheels. In the stakeholder's words: *"I want to set
   a ratio of speeds between the wheels. If one wheel can't maintain its speed,
   the other wheel is reduced in speed so that the ratio stays constant."* The
   coordinator must **slow the wheel(s) that can keep up down to the bottleneck
   wheel**, preserving the ratio in *actual* velocities — never boost a lagging
   wheel.

The intended outcome: `RatioDrive(hub, {0: 1.0, 1: 0.5})` + `set_speed(...)`
drives the two rig wheels (channels 0 and 1) at a held ratio, automatically
de-rating the pace when a wheel is loaded/slipping, and recovering when it
clears.

### Key findings from research (already true — reuse, don't reinvent)

- The library **already wraps the hub's velocity PID**: `motor.set_mode(MotorMode.CONSTANT_VELOCITY)`,
  `motor.set_target_velocity(counts_per_s)` (signed 16-bit), `motor.get_velocity(bulk)`
  (measured, from a bulk snapshot), `session.set_motor_pid_coefficients(ch, ClosedLoopMode.VELOCITY, p, i, d)`
  (Q16 handled internally). See [motor.py](src/rhsp/devices/motor.py), [session.py](src/rhsp/session.py).
- `hub.bulk_input() -> BulkInputData` is **one atomic transaction** returning all
  4 motors' `motorN_velocity` (s16 counts/s), `motorN_encoder` (s32), `motorN_mode`,
  `motor_status`, and `monotonic_time_ms` (real-`dt` source). See [bulk.py](src/rhsp/devices/bulk.py).
- `Session` holds a reentrant `RLock`; **a host control-loop thread is safe**
  concurrently with the keep-alive heartbeat thread. No extra wire locking needed.
- **Firmware:** 1.8.2 is the **latest** Expansion Hub firmware (no upgrade path);
  `CONSTANT_VELOCITY`/`RUN_USING_ENCODER` is the standard, functional mode on this
  hardware. The earlier "might be broken" concern came only from a defensive
  `pytest.skip` in [examples/test_motor.py](examples/test_motor.py), not a real failure.
- **Real caveat:** hub velocity is a **signed 16-bit counter (±32767 counts/s)** —
  can overflow with high-CPR encoders at speed. Clamp targets to int16 and document.

### Decisions (locked with stakeholder)

- **Inner loop:** hub's onboard velocity PID is the **primary and only built**
  implementation. Keep a thin `VelocityController` seam so a host-side PID
  fallback can be slotted in later *iff* hardware testing shows the firmware
  loop misbehaving — but do **not** build that fallback now.
- **Governor:** **deterministic "cap-to-slowest"** with slew limiting (no PID
  tuning knobs). Ratio is exact by construction.
- **Units:** native **counts/s**, with an **optional `cpr`** for RPM convenience.

---

## Design

### New module: `src/rhsp/control.py`

Houses the whole host-side control layer (no protocol/codec coupling; depends
only on `rhsp.enums`, `rhsp.devices.bulk`, `rhsp.devices.motor`, `rhsp.hub`).
Composed *onto* a `Hub` — deliberately **not** embedded in `Hub`, which stays a
thin device container.

**Classes / names:**
- `VelocityController` — tiny ABC: `attach()`, `command(target_counts_s)`,
  `measured(bulk) -> int`, `detach(disable=True)`, `available: bool`.
- `HubVelocityController` — the one impl: `attach()` →
  `motor.set_mode(CONSTANT_VELOCITY, float_at_zero=True)`, optional
  `set_velocity_pid()`, `motor.enable()`; `command(t)` → `motor.set_target_velocity(t)`;
  `measured(bulk)` → `motor.get_velocity(bulk)`.
- `RatioDrive` — the public coordinator (primary user surface).

### `RatioDrive` public API

```
RatioDrive(
    hub, weights: Mapping[int, float], *,
    rate_hz=50.0,
    max_accel=6000.0,        # counts/s^2 ceiling on how fast `scale` drops
    recovery_accel=None,     # default = max_accel; how fast `scale` climbs back
    sat_margin=0.15,         # >15% normalized shortfall => wheel is bottlenecked
    sat_release_margin=0.07, # hysteresis to leave the saturated state
    min_scale=0.0,           # speed floor (counts/s)
    deadband=20,             # counts/s; ignore measurement noise below this
    cpr=None,                # optional counts-per-rev for RPM convenience
    velocity_pid=None,       # optional (p,i,d) pushed to the hub velocity loop
    on_error=None,           # callback(exc) for transient loop errors
)

# setpoints (thread-safe; may be called while the loop runs)
set_speed(speed)                 # base scale, counts/s (or RPM if cpr set)
set_speed_rpm(rpm)               # only when cpr provided
set_weights(weights)             # replace ratio weights atomically
set_ratio(ratio, pair=(0,1))     # convenience: {a:1.0, b:ratio}
stop(brake=False)                # command zero to all wheels

# lifecycle
start(); stop_loop(); close()    # thread control; close() zeros + disables + restores modes
__enter__/__exit__               # start() / close(); nests inside `with hub:`
update(bulk)                     # one synchronous control step (unit-test entry point)
step()                           # hub.bulk_input() then update(bulk)

# read-only properties
scale, target_scale, commanded_targets, measured,
normalized_actual, saturated, weights
```

### Governor algorithm (deterministic cap-to-slowest)

For each managed channel *i* with weight `w_i` (sign encodes direction;
`w_i = 0` → hold that wheel at 0, exclude from governor):

- Commanded target: `t_i = round(g · w_i)`, clamped to int16. **The ratio of
  commanded targets is always exactly the weight ratio** — the governor only
  ever moves the single common scale `g ∈ [min_scale, S]`, where `S = target_scale`.
- Normalized actual speed: `n_i = sign(w_i) · v_i / |w_i|` (from `bulk.motorN_velocity`).

Per step (real `dt` from `monotonic_time_ms`):

1. Compute `n_i` for all active wheels (guard `|w_i|>0`; apply `deadband`).
2. **Saturation w/ hysteresis**, judged vs the *current* `g` (not `S`, so we
   don't re-trigger on our own throttling): `shortfall_i = (g − n_i)/max(g,ε)`.
   Enter `saturated` when `shortfall_i > sat_margin`; leave when `< sat_release_margin`.
3. **Ceiling:** if any wheel saturated, `g_ceiling = min over saturated i of n_i`;
   else `g_ceiling = S` (climb back toward setpoint).
4. **Slew-limit** `g` toward `g_ceiling`: down by `max_accel·dt`, up by
   `recovery_accel·dt`; clamp to `[min_scale, S]`.
5. Emit `t_i = clamp_int16(round(g · w_i))` to each `VelocityController.command`.

This resists transient acceleration lag two ways: wheels lag `g` *together*
during accel (symmetric, doesn't flip them `saturated` past the margin), and the
slew limit prevents `g` collapsing onto a momentary dip. Only a wheel that
*persistently* can't reach `g` (load/slip/stall/int16-clamp) pulls the ceiling
down — and since the ratio lives entirely in `t_i = g·w_i`, it is preserved exactly.

### Threading / lifecycle

- Daemon loop thread `rhsp-ratiodrive`, structured like `Hub._loop` keep-alive
  (`while not self._stop.wait(period)`), each tick: `update(hub.bulk_input())`.
- Wire-safe via the existing Session `RLock`; coordinator's own state guarded by
  an internal `RLock`. Coexists with — **does not replace** — the keep-alive
  heartbeat (`with hub:` / `hub.start_keepalive()` keeps running).
- Per-iteration `try/except`: transient timeout/NACK → `on_error`, keep looping
  (mirrors keep-alive). Persistent fault → hub's 2.5 s fail-safe disables outputs.
- `close()`: stop thread, zero all wheels, `detach(disable=True)`, restore prior
  motor modes. Idiom:
  ```python
  with rhsp.connect(port) as hub:            # heartbeat on
      hub.init_peripherals()
      with RatioDrive(hub, {0:1.0, 1:0.5}) as drive:
          drive.set_speed(1200)
          time.sleep(5)
  ```

### Units & PID coefficients

- Native counts/s everywhere; `cpr` (user-supplied calibration) enables
  `set_speed_rpm()` / an `.rpm` view via `counts_per_s = rpm·cpr/60`.
- Do **not** ship magic PID gains; leave the firmware velocity loop at its
  defaults. Add convenience `motor.set_velocity_pid(p,i,d)` / `get_velocity_pid()`
  (thin wrappers over `session.set_motor_pid_coefficients(..., ClosedLoopMode.VELOCITY, ...)`)
  and a `RatioDrive(velocity_pid=...)` passthrough so users can tune and read back.

### Edge cases handled

Zero-weight wheel (hold 0, no div-by-zero); stalled wheel (`n_i→0` → pack slows,
optional `stall_floor`); encoder sign mismatch (fix via weight sign); reversal
through zero (`g` is a magnitude; sign rides on `w_i`; no governing inside a
small zero-band); **int16 velocity/power saturation** (clamp → shows up as a real
bottleneck → ratio preserved); disabled motor (detected via `motorN_mode`/repeated
zero → treated as stall, `on_error`); bulk staleness / dt jitter (use
`monotonic_time_ms`, not assumed `1/rate_hz`).

---

## Files to touch

**New**
- `src/rhsp/control.py` — `VelocityController` (ABC) + `HubVelocityController`,
  `RatioDrive`. Primary deliverable.
- `tests/test_control.py` — hardware-free unit tests driving `update(bulk)` with
  synthetic `BulkInputData`.
- `examples/test_ratio_drive.py` — live-rig validation, auto-skips without a hub.

**Extend**
- `src/rhsp/__init__.py` — export `RatioDrive`, `VelocityController` in `__all__`.
- `src/rhsp/devices/motor.py` — add `set_velocity_pid(p,i,d)` / `get_velocity_pid()`
  convenience (reused by the hub controller and tuning).

**Reference only (no change needed)**
- `src/rhsp/hub.py` (`bulk_input`, `motors[]`, keep-alive/context-manager model),
  `src/rhsp/session.py` (RLock contract, PID coeffs), `tests/fakehub.py` (reused).

---

## Verification

**Hardware-free unit tests** (`tests/test_control.py`, FakeHub + synthetic bulks):
- Ratio invariance: `commanded_targets[0]/[1] == w0/w1` across a feed sweep.
- Collapse-to-slowest: wheel 1 stuck at 50% → `scale` converges to `v1/|w1|`,
  ratio preserved, lagging wheel never boosted above its target.
- Slew limiting: sudden bottleneck → `scale` drops ≤ `max_accel·dt` per step;
  sudden clear → rises ≤ `recovery_accel·dt`.
- Transient-accel robustness: symmetric lag during a ramp does **not** collapse `scale`.
- Recovery after the bottleneck clears; hysteresis (no `saturated` chatter).
- Edge cases: zero-weight, stall, negative weight (reversed sign), int16 clamp.
- Lifecycle: `start()/close()` idempotency, zero-on-close, mode restore (FakeHub
  `run_in_thread()` at high `rate_hz`).

**Live-hardware validation** (`examples/test_ratio_drive.py`, on the 2-wheel rig,
motors 0 & 1; per the `test-on-real-hardware` directive):
1. **Capability probe FIRST** — confirm `CONSTANT_VELOCITY` enable + tracking on
   one wheel; this is the go/no-go for the hub-PID inner loop.
2. Straight (1:1) and curve (1:0.5): log measured velocities + `scale`; assert
   steady-state measured ratio ≈ weight ratio.
3. Induced bottleneck: operator loads one wheel → assert `scale` drops and the
   other wheel slows so the measured ratio holds; release → recovery. Log a CSV
   of `target_scale, scale, v0, v1, ratio`.
4. `finally: drive.close(); hub.fail_safe()`.

Gate stays the hardware-free `uv run pytest` suite (currently 434 tests).

## Process note

This is net-new feature work on a project whose Sprint 001 is closed. On exit,
the plan-to-issue hook captures this as a CLASI issue; it can then be planned
into a new sprint (sprint-planner → tickets → execute). It joins the two pending
issues already in the pool ([hub-managed-led-reassert](.clasi/issues/hub-managed-led-reassert.md)
plus the two rewrite issues).
