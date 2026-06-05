# rhsp — Use Cases

> **Actors**
> - **Robot developer**: FTC/robotics engineer using the `rhsp` library to
>   drive a REV hub from a host computer.
> - **Library developer**: engineer implementing or testing the `rhsp` codebase
>   itself.
>
> **Common preconditions** (apply to all consumer use cases unless stated otherwise):
> - Python ≥ 3.13 environment with `rhsp` installed.
> - A REV Expansion Hub or Control Hub connected via USB.
> - `connect()` has been called and returned a `Hub` object.
> - `hub.init_peripherals()` has completed without error.
> - `hub.keep_alive()` is called at least every 2 s while outputs are active.

---

## UC-001 — Enumerate and Connect to a Hub

**Actor**: Robot developer

**Preconditions**: Hub connected via USB. Python environment with `rhsp`.

**Main flow**:
1. Developer calls `enumerate_hubs()`.
2. Library scans USB serial ports; returns those whose USB serial number field
   starts with `D`.
3. Developer selects a port (or passes `None` to `connect()` to auto-select).
4. Developer calls `connect(port)`.
5. Library opens `SerialTransport` at 460800 8N1.
6. Library broadcasts `Discovery` (0x7F0F) to destination 0xFF.
7. Library collects all `Discovery_RSP` replies until a quiet window; creates
   one `Hub` object per responding module.
8. Library calls `QueryInterface("DEKA")` on the parent hub; stores the runtime
   `deka_base` in the session.
9. Library returns the parent `Hub`.

**Postconditions**: `Hub` object is ready. `session.deka_base` is set.
All subsequent commands use `deka_base + index` for DEKA commands.

**Error flows**:
- No port with `D`-prefix serial number found: `enumerate_hubs()` returns empty
  list; `connect(None)` raises `RhspError("No hub found")`.
- Serial port open fails: `RhspError` wraps the OS error.
- Discovery receives no replies within timeout: `RhspTimeoutError`.
- `QueryInterface` NACK (code 255 — unknown packet type): `NackError(255, ...)`.

---

## UC-002 — Discover Parent and Child Hubs (Multi-Hub Topology)

**Actor**: Robot developer

**Preconditions**: Parent hub connected via USB. One or more child hubs connected
to the parent over RS485 (daisy-chained).

**Main flow**:
1. Developer calls `connect(port)`.
2. Library sends `Discovery` to 0xFF (broadcast).
3. Parent hub replies with its own `Discovery_RSP` (`parent=True`).
4. Parent hub relays the broadcast over RS485; each child replies with
   `Discovery_RSP` (`parent=False`).
5. Library creates one `Hub` object per reply, each with its own `address`
   (the hub's `source` field in the response).
6. Developer inspects `hub.parent` to identify the parent; child hubs are
   addressable at their discovered addresses.

**Postconditions**: Developer has a list of `Hub` objects. Each child hub is
independently addressable.

**Error flows**:
- Child hub does not respond: its `Discovery_RSP` is absent; no `Hub` created
  for that child. Parent still available.
- Checksum error on a child response: `ChecksumError` logged; parser resyncs
  to next `DK` frame; discovery continues.

---

## UC-003 — Query Interface / Resolve DEKA Base

**Actor**: Robot developer (advanced) / Library internals

**Preconditions**: Serial port open. Discovery complete.

**Main flow**:
1. Library calls `QueryInterface("DEKA")` (system command 0x7F07) on the hub address.
2. Hub replies with `packetID` (first command id of the DEKA interface) and
   `numValues` (count of DEKA commands).
3. Library stores `deka_base = packetID` in the session.
4. All subsequent DEKA commands use `runtime_packet_id(cmd, deka_base)`.

**Postconditions**: `session.deka_base` set correctly. No command hard-codes 0x1000.

**Error flows**:
- Hub returns NACK (code 255 — interface name unknown): `NackError(255, ...)`.
  Indicates firmware incompatibility.

---

## UC-004 — Run a Motor (Open-Loop Power)

**Actor**: Robot developer

**Main flow**:
1. Developer accesses `hub.motors[channel]` (channels 0–3).
2. Motor is initialized: `set_motor_channel_mode(channel, CONSTANT_POWER, float_at_zero=True)`,
   then `set_motor_constant_power(channel, 0)` (done by `init_peripherals()`).
3. Developer calls `motor.enable()` → `set_motor_channel_enable(channel, True)`.
4. Developer calls `motor.set_power(power)` → `set_motor_constant_power(channel, power)`.
   `power` is in [-32767, 32767].
5. Motor runs. Developer calls `hub.keep_alive()` periodically.
6. Developer calls `motor.set_power(0)` to stop, then `motor.disable()`.

**Postconditions**: Motor runs at requested power level.

**Error flows**:
- NACK 50 — motor not configured for the mode before enable: `NackError(50, ...)`.
  Indicates `init_peripherals()` was not called or mode was changed improperly.
- NACK 52 — battery too low: `NackError(52, ...)`.
- Keep-alive timeout (> 2500 ms without any valid packet): hub enters fail-safe;
  motor disables. Developer must re-call `motor.enable()` after restoring keep-alive.

---

## UC-005 — Run a Motor (Closed-Loop Velocity)

**Actor**: Robot developer

**Main flow**:
1. Motor is initialized as in UC-004 (constant power, power=0, then enabled).
2. Developer calls `motor.set_mode(CONSTANT_VELOCITY)`.
3. Developer calls `motor.set_target_velocity(velocity)` (signed encoder counts/s).
4. Library sends `SetMotorTargetVelocity`. Hub runs its velocity PID loop.
5. Developer reads actual velocity via `hub.get_bulk_input_data().motor_N_velocity`
   (signed-16 from `GetBulkInputData`, DEKA index 0x00 — firmware-agnostic).

**Postconditions**: Motor tracks the velocity setpoint.

**Error flows**:
- NACK 51 — command not valid for selected motor mode: `NackError(51, ...)`.
  Sequence error (e.g. sent `SetMotorTargetVelocity` while in CONSTANT_POWER mode).

---

## UC-006 — Run a Motor (Position Target)

**Actor**: Robot developer

**Main flow**:
1. Motor initialized; enabled.
2. Developer optionally calls `motor.reset_encoder()` → `ResetMotorEncoder`.
3. Developer calls `motor.set_mode(POSITION_TARGET)`.
4. Developer calls `motor.set_target_position(position, tolerance)`.
5. Developer polls `motor.at_target()` → `GetMotorAtTarget` until `True`.
6. Developer reads encoder via `motor.get_encoder_position()` → `GetMotorEncoderPosition` (signed-32).

**Postconditions**: Motor reaches and holds the target position.

**Error flows**:
- NACK 50/51/52: as per UC-004.
- Motor never reaches target: `at_target()` remains `False`; developer
  implements timeout logic at the application layer.

---

## UC-007 — Drive a Servo

**Actor**: Robot developer

**Main flow**:
1. Developer accesses `hub.servos[channel]` (channels 0–5).
2. `init_peripherals()` has called `SetServoConfiguration(channel, 20000)` (50 Hz).
3. Developer calls `servo.enable()` → `SetServoEnable(channel, True)`.
4. Developer calls `servo.set_pulse_width(pw)` (µs, 500–2500).
   Or `servo.set_angle(degrees)` → `500 + degrees * 2000 / 180`.
5. Servo moves to the commanded position.

**Postconditions**: Servo at commanded position.

**Error flows**:
- NACK 30 — servo not configured before enable: `NackError(30, ...)`.
- NACK 31 — battery too low: `NackError(31, ...)`.
- Pulse width out of range (500–2500 µs): `ValueError` raised before any
  transaction is sent.

---

## UC-008 — Read and Write Digital I/O

**Actor**: Robot developer

**Main flow (output)**:
1. Developer accesses `hub.dio[pin]` (pins 0–7).
2. Developer calls `dio.set_direction(output=True)` → `SetDIODirection(pin, 1)`.
3. Developer calls `dio.write(value)` → `SetSingleDIOOutput(pin, value)`.
4. Or `hub.set_all_dio_outputs(mask)` → `SetAllDIOOutputs(mask)`.

**Main flow (input)**:
1. Developer calls `dio.set_direction(output=False)` → `SetDIODirection(pin, 0)`.
2. Developer calls `dio.read()` → `GetSingleDIOInput(pin)` → returns `bool`.
3. Or `hub.get_all_dio_inputs()` → `GetAllDIOInputs` → returns 8-bit bitmask.

**Postconditions**: Output pin holds the requested level. Input read returns current level.

**Error flows**:
- NACK 10–17 — GPIO not configured for output: `NackError(10+N, ...)`.
- NACK 18 — no GPIO configured for output: `NackError(18, ...)`.
- NACK 20–27 — GPIO not configured for input: `NackError(20+N, ...)`.

---

## UC-009 — Read an Analog Input

**Actor**: Robot developer

**Main flow**:
1. Developer accesses `hub.adc[channel]` (channels 0–3 for external analog inputs).
2. Developer calls `adc.read()` → `GetADC(adcChannel, rawMode=0)`.
3. Returns voltage or current in mV or mA (depending on channel type).
4. Developer may also read internal channels (motor current, battery voltage, etc.)
   via `session.get_adc(dest, channel)` directly.

**Postconditions**: Returns current ADC reading.

**Error flows**:
- Invalid channel id: `ValueError` before transaction.

---

## UC-010 — I2C Write

**Actor**: Robot developer

**Preconditions**: I2C device connected to hub I2C port 0–3.

**Main flow**:
1. Developer accesses `hub.i2c[channel].device(address)`.
2. Developer calls `device.write_register(register, data_bytes)`.
3. Library assembles address with `COMMAND_BIT` (0x80) if needed.
4. Library calls `I2CWriteMultipleBytes(channel, address, len, data)`.
5. Hub sends ACK.

**Postconditions**: I2C device register updated.

**Error flows**:
- NACK 40 — I2C master busy: `NackError(40, ...)`. Retry after a short delay.

---

## UC-011 — I2C Read

**Actor**: Robot developer

**Main flow**:
1. Developer calls `device.read_register(register, num_bytes)`.
2. Library sends `I2CReadMultipleBytes(channel, address, num_bytes)` → ACK.
3. Library polls `I2CReadStatusQuery(channel)`:
   - If `i2cStatus` == NACK-41 (poll again): wait and retry.
   - If success: returns data bytes.

**Postconditions**: Returns bytes read from I2C device register.

**Error flows**:
- NACK 41 — operation in progress: library polls again (up to budget, default 5 × 1 ms).
- NACK 42 — no results pending: `NackError(42, ...)`. Read was not initiated.
- NACK 43 — query mismatch: `NackError(43, ...)`.
- NACK 44–46 — I2C timeout (SDA stuck / SCK stuck / general): `NackError(code, ...)`.

---

## UC-012 — Read the Color Sensor

**Actor**: Robot developer

**Preconditions**: REV Color Sensor (APDS-9960, I2C address 0x39) connected to
a hub I2C port.

**Main flow**:
1. Developer calls `hub.i2c[channel].color_sensor()` or instantiates `ColorSensor`.
2. Library runs initialization sequence: write `ENABLE` (enable ALS + proximity),
   write `ATIME`, write `PPULSE`, verify device id 0x60.
3. Developer calls `sensor.read_color()`.
4. Library issues `I2CReadMultipleBytes` with `COMMAND_BIT | MULTI_BYTE_BIT | register`
   for the RGBC color data registers; polls `I2CReadStatusQuery`.
5. Returns `(red, green, blue, clear)` tuple.

**Postconditions**: Returns current RGBC readings.

**Error flows**:
- Device id read does not return 0x60: `ProtocolError("unexpected color sensor id")`.
- I2C timeout or busy: `NackError` as per UC-011.

---

## UC-013 — Read the Distance Sensor

**Actor**: Robot developer

**Preconditions**: REV 2 m Distance Sensor (VL53L0X, I2C address 0x29) connected.

**Main flow**:
1. Developer calls `hub.i2c[channel].distance_sensor()` or instantiates `Distance2m`.
2. Library runs the full VL53L0X initialization sequence (read `stop_variable`,
   set signal-rate limit, load SPAD config, write tuning register map, configure
   GPIO interrupt, run two `performSingleRefCalibration` passes, `setTimeout(200)`,
   `startContinuous()`).
3. Developer calls `sensor.read_mm()`.
4. Library polls interrupt status register, reads range result register,
   clears interrupt.
5. Returns distance in millimeters.

**Postconditions**: Returns measured distance.

**Error flows**:
- Sensor does not ACK on initialization I2C writes: `NackError`.
- Range measurement timeout (interrupt never asserts within timeout): `RhspTimeoutError`.

---

## UC-014 — Read the IMU

**Actor**: Robot developer

**Preconditions**: Hub's onboard BNO055 IMU (I2C address 0x28) is accessible.

**Main flow**:
1. Developer instantiates `IMU` for the hub's internal IMU bus.
2. Library sends `IMUBlockReadConfig(startRegister, numberOfBytes, readInterval_ms)`
   to configure autonomous hub-side polling of BNO055 registers.
3. Developer calls `imu.read_euler()` (or equivalent).
4. Library calls `get_bulk_input_data()` and reads `imuBlock` (10 bytes).
5. Returns orientation data (heading, pitch, roll) decoded from BNO055 register format.

**Postconditions**: Returns current IMU orientation reading.

**Error flows**:
- BNO055 I2C not responding: `NackError` from the hub's I2C subsystem.

---

## UC-015 — Get Module Status and Handle Module Alerts

**Actor**: Robot developer

**Main flow**:
1. Developer calls `hub.get_module_status(clear=False)` → `GetModuleStatus(clearStatus=0)`.
2. Library decodes `statusWord` and `motorAlerts` bytes into `ModuleStatus` dataclass.
3. Developer inspects `status.keep_alive_timeout`, `status.fail_safe`,
   `status.battery_low`, `status.motor_alerts`, etc.
4. Developer calls `hub.get_module_status(clear=True)` to clear latched flags.

**Postconditions**: Returns current module and motor alert states.

**Error flows**:
- Hub in fail-safe state (`status.fail_safe == True`): developer must re-enable
  outputs after resolving the cause (e.g. battery level, keep-alive restored).

---

## UC-016 — Handle NACK Errors

**Actor**: Robot developer

**Main flow**:
1. Developer issues a command.
2. Hub replies with NACK (`0x7F02`) carrying a `nackCode` byte.
3. Library raises `NackError(code=N, description="<human description>")`.
4. Developer catches `NackError`; inspects `e.code` and `e.description`;
   takes corrective action (re-configure, retry, abort).

**Postconditions**: Error is surfaced; developer can recover.

**Error flows** (NackError is itself the error flow):
- The NACK resets the hub's keep-alive timer (it is a valid packet).
- The library does NOT retry on NACK (NACK is definitive rejection, not transient failure).

---

## UC-017 — Keep-Alive / Fail-Safe Lifecycle Management

**Actor**: Robot developer

**Preconditions**: Motor or servo outputs are active.

**Main flow**:
1. Developer's control loop calls `hub.keep_alive()` at least every 2 s.
2. Library sends `KeepAlive` (0x7F04); hub replies with ACK; watchdog timer resets.
3. If developer needs an emergency stop: call `hub.fail_safe()` → `FailSafe` (0x7F05);
   hub enters fail-safe immediately (motor/servo outputs disabled).
4. After fail-safe: developer re-enables desired outputs before resuming.

**Postconditions**: Hub watchdog never fires during normal operation.

**Error flows**:
- Application stalls for > 2500 ms without any valid packet: hub enters fail-safe
  autonomously. `hub.get_module_status()` will report `keep_alive_timeout=True`
  and `fail_safe=True`.

---

## UC-018 — Set Module LED Color

**Actor**: Robot developer

**Main flow**:
1. Developer calls `hub.set_led_color(r, g, b)` → `SetModuleLEDColor(r, g, b)`.
2. Hub acknowledges; LED changes color.

**Postconditions**: Hub LED displays the requested color.

**Error flows**: None expected for valid (0–255) channel values.

---

## UC-019 — Read Hub Firmware Version

**Actor**: Robot developer

**Main flow**:
1. Developer calls `hub.read_version_string()` → `ReadVersionString` (DEKA 0x30).
2. Hub replies with `length` + up to 40 ASCII bytes.
3. Library decodes and returns version string, e.g. `"HW: 20, Maj: 1, Min: 8, Eng: 2"`.

**Postconditions**: Returns firmware/hardware version string.

---

## UC-020 — Set New Module Address

**Actor**: Robot developer (advanced)

**Preconditions**: Multiple hubs present; need to reassign an address to avoid
conflicts.

**Main flow**:
1. Developer calls `session.transaction("SetNewModuleAddress", dest=current_address, moduleAddress=new_address)`.
2. Hub ACKs; future transactions must use `new_address` as destination.

**Postconditions**: Hub responds to new address.

**Error flows**:
- Address conflict (another hub already at that address): behavior is
  firmware-defined; use `enumerate_hubs()` + `Discovery` to re-map.

---

## UC-021 — Trigger E-Stop (FailSafe)

**Actor**: Robot developer

**Main flow**:
1. Developer calls `hub.fail_safe()` → `FailSafe` (0x7F05).
2. Hub disables all motor and servo outputs immediately.
3. Hub ACKs.

**Postconditions**: All outputs disabled. Hub remains communicable.

---

## UC-022 — Use the Generic Transaction Escape Hatch

**Actor**: Robot developer (advanced) / Library developer

**Main flow**:
1. Developer needs a command not covered by a typed method (e.g. a legacy
   high-id command, or a stock-firmware PIDF command).
2. Developer calls `session.transaction("GetBulkMotorData", dest=addr)`.
3. Library looks up the command in the catalogue; if `legacy=True`, marks the
   call as experimental.
4. Library encodes the payload, sends the packet, decodes the response, returns `dict`.

**Postconditions**: Raw command/response exchange completed.

**Error flows**:
- Unknown command name: `KeyError` from catalogue lookup.
- Hub NACK (e.g. code 255 — unknown packet type): `NackError(255, ...)`. Indicates
  the high-id command is not available on this firmware version.

---

## UC-023 — Build and Run the Hardware-Free Test Suite

**Actor**: Library developer

**Preconditions**: Development environment. No physical hub required.

**Main flow**:
1. Developer runs `pytest tests/`.
2. Test suite runs (all tests in `tests/` are hardware-free):
   - `test_framing.py`: golden vector assertions; `FrameParser` round-trips;
     `ChecksumError` on corrupted bytes.
   - `test_codec.py`: `decode(encode(x)) == x` for every command/response in
     `protocol.json`; signed/Q16 extremes; 512-byte variable-length fields.
   - `test_session.py`: `_msg_num` sequencing; `ref_num` correlation; NACK →
     `NackError`; timeout → `RhspTimeoutError`; multi-reply discovery.
   - `test_devices.py`: byte-exact payload assertions via `FakeHub`.
   - `test_sensors.py`: recorded I2C register sequences vs. §9 recipes.

**Postconditions**: All tests pass. No hub connected.

**Error flows**:
- Any test failure indicates a regression in the codec, framing, session, or
  device/sensor layer.

---

## UC-024 — Validate with a Physical Hub (Hardware Smoke Test)

**Actor**: Library developer

**Preconditions**: Hub connected via USB with `D`-prefix serial number.

**Main flow**:
1. Developer runs `pytest examples/ -v` (or `python examples/test_motor.py`).
2. Examples auto-detect the hub via `enumerate_hubs()` and run smoke tests:
   motor power, servo sweep, DIO read/write, ADC read, color/distance/IMU read.
3. Developer optionally captures USB traffic with the REV Saleae analyzer to
   confirm wire-level compliance.

**Postconditions**: Library communicates correctly with real hardware.

**Error flows**:
- `enumerate_hubs()` returns empty: no hub detected; examples skip with
  `pytest.skip("no hub found")`.
- Any NACK or `RhspTimeoutError`: logged with full context for diagnosis.

---

## UC-025 — Regenerate and Validate the Protocol Catalogue

**Actor**: Library developer

**Preconditions**: Final implementation phase (Phase 7 of the build sequence).
`catalogue.py` is complete and `protocol.json` is the source of truth.

**Main flow**:
1. Developer runs `python docs/generate_protocol_json.py`.
2. Generator introspects `catalogue.py` (the new source) and emits JSON.
3. Developer diffs the output against the committed `protocol.json` /
   `src/rhsp/protocol.json`.
4. Any diff indicates a drift between the runtime catalogue and the committed spec.

**Postconditions**: Regenerated JSON matches committed copy byte-for-byte.

**Error flows**:
- Diff detected: developer resolves the discrepancy (update JSON or fix catalogue).
