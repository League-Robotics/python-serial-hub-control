---
id: 001-010
title: "Hub \u2014 single address, init_peripherals with rollback, keep_alive"
status: done
use-cases:
- SUC-006
- UC-001
- UC-017
- UC-021
depends-on:
- 001-009
---

# Ticket 010: Hub — single address, init_peripherals with rollback, keep_alive

## Description

Implement `src/rhsp/hub.py`: the `Hub` class representing a single hub module.
`Hub` owns all device objects (motors, servos, DIO, ADC, I2C), runs the §5.2
peripheral initialization recipe with `fail_safe()` rollback on error, and
exposes explicit `keep_alive()`.

`Hub` must not duplicate module identity: one `address` field, not both `module`
and `destinationModule`. Device objects are constructed in `__init__` with just
`(session, channel/pin, address)`.

## Acceptance Criteria

- [x] `Hub.__init__(session: Session, address: int, parent: bool = False)`:
  - `self.session = session`
  - `self.address = address`
  - `self.parent = parent`
  - `self.motors: list[Motor]` — 4 entries, channels 0–3
  - `self.servos: list[Servo]` — 6 entries, channels 0–5
  - `self.dio: list[DIOPin]` — 8 entries, pins 0–7
  - `self.adc: list[ADCPin]` — 4 entries, channels 0–3
  - `self.i2c: list[I2CChannel]` — 4 entries, channels 0–3
  - Device construction is logic-free (no hub transactions in `__init__`)
- [x] `hub.init_peripherals() -> None`:
  - For each motor channel 0–3: `session.set_motor_channel_mode(addr, ch, CONSTANT_POWER, float_at_zero=True)` then `session.set_motor_constant_power(addr, ch, 0)`
  - For each servo channel 0–5: `session.set_servo_configuration(addr, ch, 20000)`
  - If any command raises, calls `session.fail_safe(addr)` then re-raises the original exception
- [x] `hub.keep_alive() -> None`:
  - Calls `session.keep_alive(self.address)`
  - Docstring states the 2500 ms fail-safe deadline
- [x] `hub.fail_safe() -> None`:
  - Calls `session.fail_safe(self.address)`
- [x] `hub.get_module_status(clear: bool = False) -> ModuleStatus`:
  - Delegates to `session.get_module_status(self.address, clear)`
- [x] `hub.set_led_color(r, g, b) -> None` and `hub.read_version_string() -> str`
- [x] FakeHub test: `init_peripherals()` emits correct byte sequence for 4 motors (SetMotorChannelMode + SetMotorConstantPower × 4) and 6 servos (SetServoConfiguration × 6)
- [x] FakeHub test: mid-sequence `NackError` in `init_peripherals()` causes `fail_safe()` to be sent and exception re-raised
- [x] `hub.address` is a single int; no `module` or `destinationModule` attribute exists
- [x] `keep_alive()` sends `KeepAlive` (0x7F04) and receives ACK — verified via FakeHub

## Implementation Plan

### Approach

`Hub.__init__` constructs device lists using list comprehensions:
```python
self.motors = [Motor(session, ch, address) for ch in range(4)]
self.servos = [Servo(session, ch, address) for ch in range(6)]
...
```

`init_peripherals()` wraps the bring-up recipe in a try/except:
```python
try:
    for ch in range(4):
        self.session.set_motor_channel_mode(self.address, ch, MotorMode.CONSTANT_POWER, True)
        self.session.set_motor_constant_power(self.address, ch, 0)
    for ch in range(6):
        self.session.set_servo_configuration(self.address, ch, 20000)
except Exception:
    self.session.fail_safe(self.address)
    raise
```

Device classes (Motor, Servo, etc.) are imported from `rhsp.devices.*`; those
modules are created in ticket 011 but the import structure is established here.

### Files to Create

- `src/rhsp/hub.py` — `Hub` class

### Files to Modify

- `src/rhsp/discovery.py` — import Hub and use it in `connect()`
- `tests/test_hub.py` (new) — init_peripherals byte sequence test; rollback test

### Testing Plan

`tests/test_hub.py`:
- FakeHub: call `init_peripherals()`, assert FakeHub received SetMotorChannelMode × 4,
  SetMotorConstantPower × 4, SetServoConfiguration × 6 in order.
- FakeHub: configure NACK on SetMotorConstantPower channel 2; call `init_peripherals()`;
  assert `NackError` raised and FakeHub received a FailSafe frame.
- Assert `hub.address == 1` (no `destinationModule` attribute).
- Assert `keep_alive()` sends KeepAlive; FakeHub records the frame.

### Documentation Updates

None required for this ticket.
