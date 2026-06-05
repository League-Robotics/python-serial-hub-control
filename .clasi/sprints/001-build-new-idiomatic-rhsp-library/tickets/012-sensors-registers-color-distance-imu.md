---
id: '001-012'
title: Sensors — registers.py, ColorSensor, Distance2m, IMU
status: open
use-cases:
  - SUC-008
  - UC-012
  - UC-013
  - UC-014
depends-on:
  - '001-011'
---

# Ticket 012: Sensors — registers.py, ColorSensor, Distance2m, IMU

## Description

Implement `src/rhsp/sensors/registers.py` and the three sensor driver modules
`color.py`, `distance.py`, and `imu.py`. All sensors are layered strictly on
`I2CDevice.write_register()` / `read_register()` from ticket 011. Register maps
are in `registers.py`; sensor logic is in the sensor modules.

The VL53L0X `OSC_CALIBRATE_VAL = 0xF8` must be defined in `registers.py` — this
is P7-f.

## Acceptance Criteria

**`sensors/registers.py`:**

- [ ] APDS-9960 (color sensor) register constants:
  - `COMMAND_BIT = 0x80`
  - `MULTI_BYTE_BIT = 0x20`
  - `APDS9960_ENABLE`, `APDS9960_ATIME`, `APDS9960_PPULSE`, `APDS9960_ID`
  - Color data registers: `APDS9960_CDATAL`, `APDS9960_RDATAL`, `APDS9960_GDATAL`, `APDS9960_BDATAL`
  - `APDS9960_DEVICE_ID = 0x60`
- [ ] VL53L0X (distance sensor) register constants:
  - `OSC_CALIBRATE_VAL = 0xF8` — must be present (P7-f fix)
  - SPAD config registers, signal rate limit, GPIO interrupt registers, range result register,
    interrupt clear register, stop variable register
  - All constants needed for the ST initialization sequence from §2.14
- [ ] BNO055 (IMU) register constants as needed by `imu.py`

**`sensors/color.py` — `ColorSensor` / `ColorSensorV3`:**

- [ ] `ColorSensor.__init__(device: I2CDevice)`:
  - Writes ENABLE register (enable ALS + proximity)
  - Writes ATIME register
  - Writes PPULSE register
  - Reads device id register; raises `ProtocolError("unexpected color sensor id")` if not 0x60
- [ ] `read_color() -> tuple[int, int, int, int]`:
  - Issues `COMMAND_BIT | MULTI_BYTE_BIT | APDS9960_CDATAL` read (8 bytes for RGBC)
  - Returns `(red, green, blue, clear)` decoded from LE 16-bit pairs
- [ ] FakeHub test: `ColorSensor.__init__()` emits ENABLE write, ATIME write, PPULSE write, ID read — in that order

**`sensors/distance.py` — `Distance2m`:**

- [ ] `Distance2m.__init__(device: I2CDevice)`:
  - Runs the full VL53L0X initialization sequence from §2.14:
    read `stop_variable`, set signal-rate limit, load SPAD config, write default tuning
    register map, configure GPIO interrupt, run `performSingleRefCalibration(0x40)` then `(0x00)`,
    `setTimeout(200)`, `startContinuous()`
  - Uses `OSC_CALIBRATE_VAL = 0xF8` from `registers.py` where specified in the sequence
- [ ] `read_mm() -> int`:
  - Polls interrupt status register until asserted (up to timeout)
  - Reads range result register
  - Clears interrupt
  - Returns distance in millimeters
- [ ] FakeHub test: `Distance2m.__init__()` emits the full VL53L0X init sequence; `OSC_CALIBRATE_VAL` appears in the sequence
- [ ] FakeHub test: `read_mm()` issues interrupt poll + range read + interrupt clear in order

**`sensors/imu.py` — `IMU`:**

- [ ] `IMU.__init__(session: Session, dest: int)` (IMU is on the internal bus, not an external I2C channel):
  - Calls `session.transaction("IMUBlockReadConfig", dest, startRegister=..., numberOfBytes=10, readInterval_ms=10)` to configure hub-side autonomous polling
- [ ] `read_imu_block() -> bytes`:
  - Calls `session.get_bulk_input_data(dest)` and returns `bulk.imu_block` (10 bytes)
- [ ] FakeHub test: `IMU.__init__()` sends `IMUBlockReadConfig` with correct parameters

## Implementation Plan

### Approach

`registers.py` is a flat module of named constants — no classes, no functions.
Copy constants from `vendor/rhsp/distance.py` (VL53L0X) and `vendor/rhsp/color.py`
(APDS-9960) as the reference, applying the missing `OSC_CALIBRATE_VAL = 0xF8` fix.

Each sensor class accepts an `I2CDevice` (or `Session` for IMU) and runs its
init sequence in `__init__`. The FakeHub-based tests record the I2C write/read
calls in order and compare against the §2.14 bring-up recipes.

For `ColorSensor`, the init sequence is 3 writes + 1 read. The read must return
a mocked device-id byte of `0x60` from FakeHub for the test to proceed.

For `Distance2m`, the init sequence is longer (~20+ register writes and reads).
The FakeHub must be pre-loaded with mock response bytes for each read in the
sequence. The test focuses on verifying the sequence includes a write containing
`OSC_CALIBRATE_VAL` (0xF8).

### Files to Create

- `src/rhsp/sensors/__init__.py` — empty or minimal
- `src/rhsp/sensors/registers.py` — all register constants
- `src/rhsp/sensors/color.py` — `ColorSensor`, `ColorSensorV3`
- `src/rhsp/sensors/distance.py` — `Distance2m`
- `src/rhsp/sensors/imu.py` — `IMU`
- `tests/test_sensors.py` — FakeHub-backed register sequence tests

### Files to Modify

None.

### Testing Plan

`tests/test_sensors.py`:
- Import assertion: `from rhsp.sensors.registers import OSC_CALIBRATE_VAL; assert OSC_CALIBRATE_VAL == 0xF8`
- Color sensor: FakeHub records writes; assert sequence matches ENABLE/ATIME/PPULSE/ID-read order.
- Color sensor: FakeHub returns wrong device id; assert `ProtocolError` raised.
- Distance sensor: FakeHub records writes; assert `OSC_CALIBRATE_VAL` (0xF8) appears in byte stream.
- IMU: FakeHub records `IMUBlockReadConfig` with `numberOfBytes=10`.

### Documentation Updates

None required for this ticket.
