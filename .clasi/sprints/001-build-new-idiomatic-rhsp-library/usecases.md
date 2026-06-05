---
status: approved
---

# Sprint 001 Use Cases

## SUC-001: Wire-layer encode and decode
Parent: UC-023

- **Actor**: Library developer
- **Preconditions**: `src/rhsp/protocol.json` is present; `catalogue.py` loads it at import.
- **Main Flow**:
  1. Developer runs `pytest tests/test_codec.py`.
  2. For every `Command` and `Response` in the catalogue, the test encodes a dict
     of field values to bytes then decodes back to a dict.
  3. Assertions cover signed min/max, Q16 extremes, and a 512-byte variable-length
     trailing field.
  4. Field `offset` values are verified to be contiguous.
- **Postconditions**: All codec round-trips pass; no hand-written message class is needed.
- **Acceptance Criteria**:
  - [ ] `decode_payload(fields, encode_payload(fields, values)) == values` for every entry in `protocol.json`.
  - [ ] Signed 16-bit min (−32768) and max (32767) survive a round-trip.
  - [ ] Q16 PID coefficient `1.5` encodes to `round(1.5 * 65536)` signed-4 bytes and decodes back.
  - [ ] 512-byte variable-length payload survives encode/decode.
  - [ ] Field offsets are contiguous (no gaps, no overlaps).

---

## SUC-002: Frame construction and parsing with golden vectors
Parent: UC-023

- **Actor**: Library developer
- **Preconditions**: `framing.py` is implemented.
- **Main Flow**:
  1. Developer runs `pytest tests/test_framing.py`.
  2. `build_frame` is called for four known command/payload combinations.
  3. Output bytes are compared against the §2.2 golden vectors (msg_num=1).
  4. The same bytes are fed to `FrameParser.feed()` and the resulting `RawPacket`
     is compared field-by-field to the inputs.
  5. A one-bit-flipped checksum byte is fed to `FrameParser`; `ChecksumError` is raised.
- **Postconditions**: Frame construction and parsing are byte-exact; checksum errors are detected.
- **Acceptance Criteria**:
  - [ ] `KeepAlive` dest=1 msg=1 → `44 4B 0B 00 01 00 01 00 04 7F 1F`.
  - [ ] `Discovery` dest=255 msg=1 → `44 4B 0B 00 FF 00 01 00 0F 7F 28`.
  - [ ] `SetServoPulseWidth` ch=0 pw=1500 dest=1 msg=1 → `44 4B 0E 00 01 00 01 00 21 10 00 DC 05 B1`.
  - [ ] `SetMotorConstantPower` ch=0 pwr=16000 dest=1 msg=1 → `44 4B 0E 00 01 00 01 00 0F 10 00 80 3E 7C`.
  - [ ] `FrameParser.feed()` reconstructs `RawPacket` with correct fields from all four vectors.
  - [ ] Bit-flipped checksum raises `ChecksumError`.

---

## SUC-003: Catalogue loads protocol.json; legacy commands tagged
Parent: UC-022, UC-023

- **Actor**: Library developer
- **Preconditions**: `catalogue.py` imports `src/rhsp/protocol.json` via `importlib.resources`.
- **Main Flow**:
  1. `import rhsp.catalogue` succeeds without a physical hub.
  2. Developer asserts `COMMANDS["GetBulkMotorData"].legacy is True`.
  3. Developer asserts `COMMANDS["GetBulkInputData"].legacy is False`.
  4. `runtime_packet_id` returns `deka_base + index` for DEKA group commands and
     `cmd.id` for system commands.
- **Postconditions**: Catalogue is populated at import time; high-id commands are tagged legacy.
- **Acceptance Criteria**:
  - [ ] `COMMANDS` dict has 72 entries (one per JSON command).
  - [ ] `RESPONSES_BY_ID` dict has 38 entries.
  - [ ] `COMMANDS["GetBulkMotorData"].legacy is True` (index 0x37 ≥ 0x31).
  - [ ] `COMMANDS["SetMotorConstantPower"].legacy is False` (index 0x0F < 0x31).
  - [ ] `runtime_packet_id(cmd, 0x1000)` returns `0x100F` for `SetMotorConstantPower`.
  - [ ] `runtime_packet_id(cmd, 0x1000)` returns `0x7F04` for `KeepAlive` (system group).

---

## SUC-004: Session transaction sequencing and error promotion
Parent: UC-016, UC-023

- **Actor**: Library developer
- **Preconditions**: `FakeHub` is implemented over `LoopbackTransport`.
- **Main Flow**:
  1. Developer runs `pytest tests/test_session.py`.
  2. Tests verify `_msg_num` starts at 1, increments each transaction, wraps 255→1, never reaches 0.
  3. FakeHub sends a mismatched `ref_num`; session discards and retries.
  4. FakeHub sends NACK code 50; session raises `NackError(50, ...)`.
  5. FakeHub sends no response for three retries; session raises `RhspTimeoutError`.
  6. FakeHub answers `QueryInterface("DEKA")` with `packetID=0x1000`; session stores `deka_base=0x1000`.
- **Postconditions**: Session protocol correctness is fully covered without hardware.
- **Acceptance Criteria**:
  - [ ] `_msg_num` is never 0 across 300 consecutive transactions.
  - [ ] `_msg_num` wraps from 255 to 1 (not 0).
  - [ ] Mismatched `ref_num` response is discarded; correct response accepted on retry.
  - [ ] NACK → `NackError`; not retried.
  - [ ] Timeout after `retries=3` → `RhspTimeoutError`.
  - [ ] `session.deka_base` is `0x1000` after `QueryInterface("DEKA")` exchange.
  - [ ] Multi-reply discovery returns the correct number of `RawPacket` objects.

---

## SUC-005: Discovery, hub construction, and dynamic DEKA base
Parent: UC-001, UC-002, UC-003

- **Actor**: Robot developer / Library internals
- **Preconditions**: FakeHub answers `Discovery_RSP` and `QueryInterface`.
- **Main Flow**:
  1. `connect()` opens a `LoopbackTransport` backed by `FakeHub`.
  2. `discover(0xFF)` broadcasts and collects all replies until a quiet window.
  3. `QueryInterface("DEKA")` obtains `deka_base`; stores in session.
  4. One `Hub` object is constructed per discovery reply.
  5. `hub.address` matches the `src` field in `Discovery_RSP`.
- **Postconditions**: Hub object ready; `session.deka_base` set; no hard-coded 0x1000.
- **Acceptance Criteria**:
  - [ ] FakeHub multi-reply discovery → N Hub objects with correct addresses.
  - [ ] `session.deka_base` is set from `QueryInterface` reply, not hard-coded.
  - [ ] `discover()` uses a quiet-window drain, not a hard `time.sleep(2)`.
  - [ ] `enumerate_hubs()` returns only ports whose USB serial number starts with `D`.

---

## SUC-006: Hub peripheral initialization with fail_safe rollback
Parent: UC-001, UC-017

- **Actor**: Robot developer
- **Preconditions**: `Hub` object constructed.
- **Main Flow**:
  1. `hub.init_peripherals()` runs the §5.2 bring-up recipe.
  2. For each motor channel: `SetMotorChannelMode(CONSTANT_POWER, FLOAT_AT_ZERO)` + `SetMotorConstantPower(0)`.
  3. For each servo channel: `SetServoConfiguration(ch, 20000)`.
  4. If any command raises mid-sequence, `fail_safe()` is called and the error re-raised.
- **Postconditions**: All peripherals initialized; partial init rolled back on failure.
- **Acceptance Criteria**:
  - [ ] `init_peripherals()` emits the correct bring-up byte sequence for motors and servos via FakeHub.
  - [ ] Mid-sequence exception causes `fail_safe()` to be sent before re-raising.
  - [ ] `hub.keep_alive()` sends `KeepAlive` (0x7F04) and receives ACK.

---

## SUC-007: Device typed API — byte-exact payload assertions
Parent: UC-004, UC-005, UC-006, UC-007, UC-008, UC-009, UC-010, UC-011

- **Actor**: Library developer
- **Preconditions**: Device classes (`Motor`, `Servo`, `DIOPin`, `ADCPin`, `I2CChannel`) implemented.
- **Main Flow**:
  1. Tests use `FakeHub` to capture bytes sent by each device method.
  2. Each device method's payload is compared byte-for-byte against the expected encoding.
  3. P7 bugs (DIO direction return, I2C configure, PID setter, PWM width, etc.) are tested by
     asserting the correct behavior on the new code.
- **Postconditions**: All device methods produce correct wire bytes; P7 bugs are absent.
- **Acceptance Criteria**:
  - [ ] `set_servo_pulse_width(0, 1500)` → payload bytes `00 DC 05`.
  - [ ] `set_motor_constant_power(0, 16000)` → payload bytes `00 80 3E`.
  - [ ] `get_bulk_input_data()` → `BulkInputData` with motor encoder as signed-32 and velocity as signed-16.
  - [ ] `DIOPin.get_direction()` returns a value (P7-b fixed).
  - [ ] `I2CChannel.configure(speed)` passes session correctly (P7-a fixed).
  - [ ] `set_motor_pid_coefficients` calls the setter command, not the getter (P7-d fixed).
  - [ ] `I2CConfigureQuery_RSP` is in `RESPONSES_BY_ID` (P7-e fixed).
  - [ ] `GetPWMPulseWidth` response decodes 2 bytes (P7-g fixed).
  - [ ] `i2c_block_read_config` / `imu_block_read_config` set payload fields, not message attrs (P7-c fixed).

---

## SUC-008: Sensor register-sequence verification
Parent: UC-012, UC-013, UC-014

- **Actor**: Library developer
- **Preconditions**: Sensor classes and `registers.py` implemented.
- **Main Flow**:
  1. `FakeHub` records all I2C write and read calls made during sensor initialization.
  2. The recorded sequence is compared against the §2.14 bring-up recipes.
  3. `OSC_CALIBRATE_VAL = 0xF8` is verified present in `sensors/registers.py`.
- **Postconditions**: Sensor init emits the correct hardware sequences; distance sensor bug fixed.
- **Acceptance Criteria**:
  - [ ] Color sensor init: APDS-9960 ENABLE/ATIME/PPULSE writes + device-id read (0x60) in order.
  - [ ] Distance sensor init: full VL53L0X sequence including `OSC_CALIBRATE_VAL = 0xF8`.
  - [ ] IMU init: `IMUBlockReadConfig` with correct `startRegister`, `numberOfBytes`, `readInterval_ms`.
  - [ ] `OSC_CALIBRATE_VAL` is defined as `0xF8` in `sensors/registers.py`.

---

## SUC-009: Public API and hardware-free CI
Parent: UC-023, UC-024, UC-025

- **Actor**: Library developer / Robot developer
- **Preconditions**: All layers implemented.
- **Main Flow**:
  1. `from rhsp import connect, Hub, enumerate_hubs` succeeds after packaging flip.
  2. `uv run pytest tests/` passes on a machine with no hub.
  3. `uv run pytest examples/` skips all tests when no `D`-prefix hub is found.
  4. `python docs/generate_protocol_json.py` emits JSON that matches `src/rhsp/protocol.json`.
- **Postconditions**: Library is installable, importable, and CI-green without hardware.
- **Acceptance Criteria**:
  - [ ] `import rhsp` resolves to `src/rhsp/` after pyproject flip.
  - [ ] `uv sync` succeeds with no errors.
  - [ ] `uv run pytest tests/` exits 0.
  - [ ] `uv run pytest examples/` exits 0 (all skipped when no hub).
  - [ ] Regenerated JSON matches committed `protocol.json` byte-for-byte.
  - [ ] `__main__.py` is absent from `src/rhsp/`.
