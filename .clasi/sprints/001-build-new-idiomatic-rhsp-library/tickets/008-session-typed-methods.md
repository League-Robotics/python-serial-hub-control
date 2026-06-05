---
id: '001-008'
title: Session typed methods (~40 snake_case wrappers)
status: open
use-cases:
  - SUC-004
  - UC-004
  - UC-005
  - UC-006
  - UC-007
  - UC-008
  - UC-009
  - UC-010
  - UC-011
  - UC-015
  - UC-017
  - UC-018
  - UC-019
depends-on:
  - '001-007'
---

# Ticket 008: Session typed methods (~40 snake_case wrappers)

## Description

Add all ~40 typed snake_case methods to `Session`. Each method wraps one call
to `transaction()` with typed parameters, enum coercion, and a typed return
value. No typed method calls a legacy command (index ≥ 0x31). All DEKA bulk
reads use `GetBulkInputData` (index 0x00).

This ticket also adds `get_bulk_input_data()` which returns a `BulkInputData`
dataclass (implemented here inline; moved to `devices/bulk.py` in ticket 011 if
preferred, but the session method itself lives here).

## Acceptance Criteria

All methods listed below must exist on `Session` with the correct signatures. Byte-exact payload
assertions using FakeHub are the primary acceptance test for each group.

**System commands:**
- [ ] `keep_alive(dest: int) -> None` — `KeepAlive` (0x7F04), ACK
- [ ] `fail_safe(dest: int) -> None` — `FailSafe` (0x7F05), ACK
- [ ] `query_interface(dest: int, name: str) -> tuple[int, int]` — returns `(packet_id, num_values)`; stores `self.deka_base = packet_id`
- [ ] `get_module_status(dest: int, clear: bool = False) -> ModuleStatus` — decodes status bytes
- [ ] `set_module_led_color(dest: int, r: int, g: int, b: int) -> None`
- [ ] `read_version_string(dest: int) -> str` — decodes `length` + bytes

**Motor commands (DEKA 0x08–0x18):**
- [ ] `set_motor_channel_mode(dest, channel, mode: MotorMode, float_at_zero: bool) -> None`
- [ ] `get_motor_channel_mode(dest, channel) -> tuple[MotorMode, bool]`
- [ ] `set_motor_channel_enable(dest, channel, enabled: bool) -> None`
- [ ] `get_motor_channel_enable(dest, channel) -> bool`
- [ ] `set_motor_channel_current_alert_level(dest, channel, limit_ma: int) -> None`
- [ ] `get_motor_channel_current_alert_level(dest, channel) -> int`
- [ ] `reset_motor_encoder(dest, channel) -> None`
- [ ] `set_motor_constant_power(dest, channel, power: int) -> None` — power ∈ [−32767, 32767]
- [ ] `get_motor_constant_power(dest, channel) -> int`
- [ ] `set_motor_target_velocity(dest, channel, velocity: int) -> None`
- [ ] `get_motor_target_velocity(dest, channel) -> int`
- [ ] `set_motor_target_position(dest, channel, position: int, tolerance: int) -> None`
- [ ] `get_motor_target_position(dest, channel) -> tuple[int, int]`
- [ ] `get_motor_at_target(dest, channel) -> bool`
- [ ] `get_motor_encoder_position(dest, channel) -> int` — signed-32
- [ ] `set_motor_pid_coefficients(dest, channel, mode: int, p: float, i: float, d: float) -> None` — Q16 encode
- [ ] `get_motor_pid_coefficients(dest, channel, mode: int) -> tuple[float, float, float]` — Q16 decode

**Servo commands (DEKA 0x1F–0x24):**
- [ ] `set_servo_configuration(dest, channel, frame_period: int) -> None`
- [ ] `get_servo_configuration(dest, channel) -> int`
- [ ] `set_servo_pulse_width(dest, channel, pulse_width: int) -> None`
- [ ] `get_servo_pulse_width(dest, channel) -> int`
- [ ] `set_servo_enable(dest, channel, enabled: bool) -> None`
- [ ] `get_servo_enable(dest, channel) -> bool`

**PWM commands (DEKA 0x19–0x1E):**
- [ ] `set_pwm_configuration(dest, channel, frame_period: int) -> None`
- [ ] `get_pwm_configuration(dest, channel) -> int`
- [ ] `set_pwm_pulse_width(dest, channel, pulse_width: int) -> None`
- [ ] `get_pwm_pulse_width(dest, channel) -> int` — 2-byte response (P7-g fix)
- [ ] `set_pwm_enable(dest, channel, enabled: bool) -> None`
- [ ] `get_pwm_enable(dest, channel) -> bool`

**DIO commands (DEKA 0x01–0x06):**
- [ ] `set_dio_direction(dest, pin, output: bool) -> None`
- [ ] `get_dio_direction(dest, pin) -> bool` — returns value (P7-b fix)
- [ ] `set_single_dio_output(dest, pin, value: bool) -> None`
- [ ] `get_single_dio_input(dest, pin) -> bool`
- [ ] `set_all_dio_outputs(dest, mask: int) -> None`
- [ ] `get_all_dio_inputs(dest) -> int`

**ADC (DEKA 0x07):**
- [ ] `get_adc(dest, channel: int, raw: bool = False) -> int`

**I2C commands (DEKA 0x25–0x2F):**
- [ ] `i2c_write_single_byte(dest, i2c_ch, address, byte) -> None`
- [ ] `i2c_write_multiple_bytes(dest, i2c_ch, address, data: bytes) -> None`
- [ ] `i2c_read_single_byte(dest, i2c_ch, address) -> None` — ACK only
- [ ] `i2c_read_multiple_bytes(dest, i2c_ch, address, num_bytes) -> None` — ACK only
- [ ] `i2c_read_status_query(dest, i2c_ch) -> tuple[int, bytes]` — (status, data)
- [ ] `i2c_write_status_query(dest, i2c_ch) -> tuple[int, int]` — (status, num_bytes)
- [ ] `i2c_configure_channel(dest, i2c_ch, speed_code: int) -> None`
- [ ] `i2c_configure_query(dest, i2c_ch) -> int` — I2CConfigureQuery_RSP registered (P7-e fix)

**Bulk data:**
- [ ] `get_bulk_input_data(dest) -> BulkInputData` — calls `GetBulkInputData` (DEKA 0x00); decodes via `BulkInputData.from_response()`

**Payload byte-exact assertions (FakeHub):**
- [ ] `set_servo_pulse_width(dest=1, channel=0, pulse_width=1500)` → payload bytes `00 DC 05`
- [ ] `set_motor_constant_power(dest=1, channel=0, power=16000)` → payload bytes `00 80 3E`
- [ ] `set_motor_channel_mode(dest=1, channel=0, mode=CONSTANT_POWER, float_at_zero=True)` → payload `00 00 01`
- [ ] `get_dio_direction(dest=1, pin=3)` returns the decoded value from FakeHub response (P7-b)
- [ ] `get_pwm_pulse_width` decodes 2-byte response correctly (P7-g)
- [ ] `i2c_configure_query` uses the correct response id from `RESPONSES_BY_ID` (P7-e)

## Implementation Plan

### Approach

Each typed method:
1. Calls `self.transaction(command_name, dest, **kwargs)` where `kwargs` are the
   raw field values (enums passed as int, booleans as 0/1, Q16 floats as floats
   — the codec layer handles conversion).
2. Extracts and returns the relevant fields from the returned dict (or `None` for ACK).
3. Applies enum wrapping if needed (e.g. `MotorMode(result["motorChannelMode"])`).

`BulkInputData` is a dataclass defined in this ticket (or in `devices/bulk.py` if
that module is started first; either way, `session.get_bulk_input_data()` returns
one). It has named fields for all 47+ fields in the `GetBulkInputData` response,
with signed-32 reinterpretation for encoder fields and signed-16 for velocity fields
(JSON overlay omits the `signed` flag; `BulkInputData.from_response()` handles this
explicitly; also handles the `mototonicTime` typo from vendor code).

The `i2c_configure_query` method must look up response id for `I2CConfigureQuery_RSP`
in `RESPONSES_BY_ID` — this ensures P7-e is fixed by construction.

### Files to Create

None (adds to `src/rhsp/session.py`).

### Files to Modify

- `src/rhsp/session.py` — add all ~40 typed methods
- `src/rhsp/devices/bulk.py` — create `BulkInputData` and `ModuleStatus` (or inline in session, to be extracted in ticket 011)
- `tests/test_session.py` — add byte-exact payload assertions for the bullet-point methods

### Testing Plan

`tests/test_devices.py` (pre-created here; extended in ticket 011):
- `set_servo_pulse_width(0, 1500)` byte assertion.
- `set_motor_constant_power(0, 16000)` byte assertion.
- `get_dio_direction` returns a value.
- `get_pwm_pulse_width` decodes 2 bytes.
- `i2c_configure_query` uses correct response id.
- `get_bulk_input_data()` → `BulkInputData` with negative encoder value (signed-32 test).

### Documentation Updates

None required for this ticket.
