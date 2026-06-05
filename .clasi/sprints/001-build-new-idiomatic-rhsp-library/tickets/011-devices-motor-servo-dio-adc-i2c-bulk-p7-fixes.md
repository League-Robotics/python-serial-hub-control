---
id: 001-011
title: "Devices \u2014 Motor, Servo, DIO, ADC, I2C, BulkInputData, P7 bug fixes"
status: done
use-cases:
- SUC-007
- UC-004
- UC-005
- UC-006
- UC-007
- UC-008
- UC-009
- UC-010
- UC-011
depends-on:
- 001-010
---

# Ticket 011: Devices — Motor, Servo, DIO, ADC, I2C, BulkInputData, P7 bug fixes

## Description

Implement all device classes in `src/rhsp/devices/`: `Motor`, `Servo`, `DIOPin`,
`ADCPin`, `I2CChannel`, `I2CDevice`, and the `BulkInputData`/`ModuleStatus`
dataclasses in `devices/bulk.py`. All classes are thin wrappers over
`session.<method>` calls. The `internal/` passthrough layer does not exist.

This ticket is the primary location for fixing and verifying all P7-series bugs
by construction (correct implementation) and by test assertion (FakeHub byte checks).

## Acceptance Criteria

**P7 bug fixes — all must be verified by FakeHub tests, not just inspection:**

- [x] P7-a: `I2CChannel.configure(speed)` calls `session.i2c_configure_channel(self.address, self.channel, speed)` — session reference is not dropped
- [x] P7-b: `DIOPin.get_direction() -> bool` returns the decoded value from `get_dio_direction()`
- [x] P7-c: `session.i2c_block_read_config()` and `session.imu_block_read_config()` are implemented using `transaction()` with correct field kwargs — payload fields are set, not message attributes
- [x] P7-d: `session.set_motor_pid_coefficients()` calls `SetMotorPIDCoefficients` (the setter), not `GetMotorPIDCoefficients` (the getter) — verified by FakeHub observing the correct packet type id
- [x] P7-e: `I2CConfigureQuery_RSP` is registered in `RESPONSES_BY_ID` (handled in catalogue; verify here that `session.i2c_configure_query()` decodes the response correctly)
- [x] P7-f: `OSC_CALIBRATE_VAL = 0xF8` in `sensors/registers.py` — verified by import assertion in `test_devices.py`
- [x] P7-g: `get_pwm_pulse_width()` decodes a 2-byte response field (same width as the setter's `pulseWidth` field)

**Device class contracts:**

- [x] `Motor(session: Session, channel: int, dest: int)`:
  - `enable()` → `set_motor_channel_enable(dest, channel, True)`
  - `disable()` → `set_motor_channel_enable(dest, channel, False)`
  - `set_power(power: int) -> None` — `set_motor_constant_power`
  - `set_mode(mode: MotorMode, float_at_zero: bool = True) -> None`
  - `set_target_velocity(velocity: int) -> None`
  - `set_target_position(position: int, tolerance: int) -> None`
  - `at_target() -> bool`
  - `get_encoder_position() -> int` (signed-32)
  - `reset_encoder() -> None`
  - `get_velocity(hub_bulk: BulkInputData) -> int` — reads from passed-in BulkInputData, NOT a separate transaction; specifically does NOT call GetBulkMotorData (0x37)

- [x] `Servo(session, channel, dest)`:
  - `enable()`, `disable()`
  - `set_pulse_width(pw: int) -> None` — raises `ValueError` if pw not in [500, 2500]
  - `set_angle(degrees: float) -> None` — converts: `500 + degrees * 2000 / 180`
  - `set_configuration(frame_period: int) -> None`

- [x] `DIOPin(session, pin, dest)`:
  - `set_direction(output: bool) -> None`
  - `get_direction() -> bool` — returns value (P7-b)
  - `write(value: bool) -> None`
  - `read() -> bool`

- [x] `ADCPin(session, channel, dest)`:
  - `read(raw: bool = False) -> int`

- [x] `I2CChannel(session, channel, dest)`:
  - `configure(speed_code: int) -> None` — passes session correctly (P7-a)
  - `device(address: int) -> I2CDevice`

- [x] `I2CDevice(session, i2c_channel, dest, address)`:
  - `write_register(register: int, data: bytes) -> None`
  - `read_register(register: int, num_bytes: int) -> bytes` — implements the §5.8 two-phase poll; polls `i2c_read_status_query` until status ≠ NACK-41 (up to 5 polls × 1 ms)

- [x] `BulkInputData` (frozen dataclass in `devices/bulk.py`):
  - `motor_0_encoder` through `motor_3_encoder`: `int` (signed-32)
  - `motor_0_velocity` through `motor_3_velocity`: `int` (signed-16)
  - `analog_input_0` through `analog_input_3`: `int`
  - `battery_voltage_mv`, `servo_0_cmd` through `servo_5_cmd`, etc.
  - `from_response(d: dict) -> BulkInputData` — reinterprets encoder/velocity as signed; handles `mototonicTime` typo

- [x] `ModuleStatus` (frozen dataclass in `devices/bulk.py`):
  - `keep_alive_timeout: bool`, `device_reset: bool`, `fail_safe: bool`,
    `controller_over_temp: bool`, `battery_low: bool`, `hib_fault: bool`
  - `motor_0_lost_encoder: bool` through `motor_3_lost_encoder: bool`
  - `motor_0_overheat: bool` through `motor_3_overheat: bool`
  - `from_response(d: dict) -> ModuleStatus`

**FakeHub payload byte assertions (extends test_devices.py):**

- [x] `motor.set_power(16000)` → FakeHub payload bytes `00 80 3E`
- [x] `servo.set_pulse_width(1500)` → FakeHub payload bytes `00 DC 05`
- [x] `dio.get_direction()` returns `True` when FakeHub encodes `directionOutput=1`
- [x] `session.set_motor_pid_coefficients(dest, 0, 0, p=1.5, i=0.0, d=0.0)` → FakeHub receives SetMotorPIDCoefficients packet type (not GetMotorPIDCoefficients)
- [x] `BulkInputData.from_response()` correctly decodes negative encoder value (e.g. −1 as 0xFFFFFFFF)
- [x] `I2CDevice.read_register()` issues write-then-poll sequence; FakeHub records write + status query

## Implementation Plan

### Approach

Each device class is a simple dataclass-like object holding `(session, channel_or_pin, dest)`.
No `internal/` package. Methods delegate directly to `session.<typed_method>()`.

`BulkInputData.from_response(d)` uses `struct.unpack_from` or Python's signed
reinterpretation: `encoder = d["motor0Encoder"]` reinterpreted as signed-32 via
`int.from_bytes(int.to_bytes(raw, 4, "little"), "little", signed=True)` if the
catalogue does not mark the field signed (which it doesn't per open question 3).

`I2CDevice.read_register()` poll loop:
```python
for _ in range(5):
    status, data = session.i2c_read_status_query(dest, channel)
    if status != NACK_41:
        return data
    time.sleep(0.001)
raise RhspTimeoutError("I2C read poll timeout")
```

### Files to Create

- `src/rhsp/devices/__init__.py` — empty or minimal re-exports
- `src/rhsp/devices/motor.py` — `Motor`
- `src/rhsp/devices/servo.py` — `Servo`
- `src/rhsp/devices/dio.py` — `DIOPin`
- `src/rhsp/devices/adc.py` — `ADCPin`
- `src/rhsp/devices/i2c.py` — `I2CChannel`, `I2CDevice`
- `src/rhsp/devices/bulk.py` — `BulkInputData`, `ModuleStatus`

### Files to Modify

- `tests/test_devices.py` — extend with all P7 regression assertions + BulkInputData decode test

### Testing Plan

`tests/test_devices.py` (created in ticket 008; extended here):
- All P7-a through P7-g FakeHub assertions listed above.
- `Motor.get_velocity(bulk)` does not trigger a new transaction (assert `len(fake.requests)` unchanged).
- `I2CDevice.read_register()` poll: FakeHub returns NACK-41 twice then data; assert 3 total I2C status query calls.
- `Servo.set_angle(90)` → pulse_width = `500 + 90 * 2000/180 ≈ 1500` → FakeHub payload `00 DC 05`.
- `Servo.set_pulse_width(499)` → raises `ValueError` before any transaction.

### Documentation Updates

None required for this ticket.
