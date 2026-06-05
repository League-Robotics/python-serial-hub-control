# rhsp — Full Specification

> **Sources.** This specification consolidates:
> - `docs/RHSP-Protocol.md` — human protocol spec (wire format, §2–§9, §12 bugs)
> - `docs/rhsp-protocol.json` — machine-readable command/response catalogue
> - `.clasi/issues/rhsp-idiomatic-rewrite.md` — code review, target architecture, remediation plan
> - `.clasi/issues/rhsp-build-a-new-idiomatic-python-library.md` — locked decisions, concrete signatures, build sequence
>
> Where those sources disagree, the locked decisions in this document take precedence.

---

## 1. Locked Decisions (Immutable Constraints)

These decisions have been made by the stakeholder. They are not open for
reconsideration during sprint planning or implementation.

| Decision | Detail |
|----------|--------|
| **Codec strategy** | Runtime data-table seeded from `protocol.json`. No 224 hand-written classes. |
| **API style** | Breaking snake_case API. camelCase removed. |
| **Protocol fixes** | Dynamic DEKA base via `QueryInterface`; `msg_num` ≥ 1; 512-byte buffers; response validation; NACK → typed exception. |
| **High-id command map** | Stock-firmware map is canonical for ids ≥ 0x31. Legacy Python `GetBulk*`/`BlockRead*` commands are carried only behind the generic `transaction()` escape hatch. |
| **Typed API firmware scope** | Typed device methods use only universally-agreed ids 0x00–0x30. `Motor.get_velocity` reads `GetBulkInputData` (deka index 0x00), not the legacy `GetBulkMotorData` (0x37). |
| **Threading** | Synchronous library. Explicit `keep_alive()`. No background threads. |
| **Sensors** | Color, distance, and IMU are all in scope. |
| **GUI** | `__main__.py` is deleted. |
| **Python version** | ≥ 3.13. |
| **Vendor preservation** | Original code stays in `vendor/rhsp/` as a runnable reference. New code built fresh in `src/rhsp/`. |

---

## 2. Protocol Facts

### 2.1 Physical / Link Layer

- **Transport**: USB CDC virtual serial port (pyserial)
- **Baud rate**: 460800; 8N1; no flow control
- **Port discovery**: scan USB serial ports; select those whose USB `SER=` field starts with `D`
- **Topology**: controller ↔ parent hub over USB (UART0); child hubs connected to parent over RS485 (UART1). Same protocol on both legs; parent forwards packets to children. Discovery (§2.7) walks the chain.

### 2.2 Packet Framing

All traffic is discrete packets. Every packet must be built and parsed using
`framing.py`. Ref: `docs/RHSP-Protocol.md` §2.

```
Offset  Size  Field           Notes
──────  ────  ─────────────── ────────────────────────────────────────────────
0       2     frame bytes     Constant 0x44 0x4B (ASCII "DK"). Start of packet.
2       2     length (LE)     Total packet size in bytes (frame + header + payload + checksum).
4       1     destination     Target module address. 0xFF = broadcast.
5       1     source          Always 0x00 from controller. Hub's address on reply.
6       1     msg_num         Sequence id. MUST be ≥ 1. Starts at 1, wraps to 1 (never 0).
7       1     ref_num         On response: echoes the request's msg_num.
8       2     packet_type (LE) Command id. Bit 15 = response flag.
10      N     payload         Command-specific. May be empty.
10+N    1     checksum        8-bit additive: sum(all preceding bytes) % 256.
```

- Minimum packet (empty payload): **11 bytes**.
- `length = 10 (header incl. frame) + len(payload) + 1 (checksum)`.
- Receive buffer must accommodate **512 bytes** of payload (not 128 as in vendor code).
- On **bad checksum**: hub sends no reply and does not reset the keep-alive timer.
- All multi-byte integers are **little-endian**.

#### Framing implementation (`framing.py`)

```python
FRAME = b"DK"
HEADER = struct.Struct("<2sHBBBBH")  # frame(2) length(2) dest src msg ref ptype
MAX_PACKET = 512 + 10 + 1

def checksum(buf: bytes) -> int: return sum(buf) & 0xFF

def build_frame(dest, src, msg, ref, ptype, payload) -> bytes: ...

class FrameParser:
    def feed(self, data: bytes) -> Iterator[RawPacket]: ...
    # resync on DK; bound payload to 512; raise ChecksumError on mismatch

@dataclass(frozen=True)
class RawPacket:
    dest: int; src: int; msg_num: int; ref_num: int; packet_type: int; payload: bytes
```

**Golden test vectors** (msg_num re-stamped for spec compliance, msg=1):

| Command | Expected bytes (hex) |
|---------|----------------------|
| `KeepAlive` dest=1 msg=1 | `44 4B 0B 00 01 00 01 00 04 7F 1F` |
| `Discovery` dest=255 msg=1 | `44 4B 0B 00 FF 00 01 00 0F 7F 28` |
| `SetServoPulseWidth` ch=0 pw=1500 dest=1 msg=1 | `44 4B 0E 00 01 00 01 00 21 10 00 DC 05 B1` |
| `SetMotorConstantPower` ch=0 pwr=16000 dest=1 msg=1 | `44 4B 0E 00 01 00 01 00 0F 10 00 80 3E 7C` |

> Note: vendor/reference vectors used msg_num=0 (protocol violation). The above
> vectors are corrected for msg_num=1. Confirm exact checksums in
> `tests/test_framing.py`.

### 2.3 Transaction Model

- Strictly **half-duplex, request/response, one outstanding transaction at a time**.
- Session sends, waits up to `timeout` (default 1 s), retries up to `retries` times on timeout or checksum error.
- Accept a response only when `packet_type == expected_response_id` AND `ref_num == sent_msg_num` (Discovery exempt: multi-reply, no ref_num correlation).
- On **NACK** (`0x7F02`): raise `NackError(code, description)` — do not retry.
- On **timeout** after all retries: raise `RhspTimeoutError`.
- On **bad checksum** (received): raise `ChecksumError`, resync, retry.

### 2.4 Keep-Alive / Fail-Safe

- Hub runs a **2500 ms watchdog timer**.
- Any valid packet (correct `DK` + checksum) resets the timer, even one that gets NACKed.
- Packets with bad checksum or missing frame bytes do NOT reset the timer.
- If the timer expires: hub enters **fail-safe** (motors/servos disabled; LED signals timeout).
- The controller must call `hub.keep_alive()` — which sends `KeepAlive` (0x7F04) — at least every ~2 s while any output is active.
- `FailSafe` (0x7F05) lets the controller trigger fail-safe deliberately (e-stop).
- After a fail-safe, outputs must be explicitly re-enabled.

### 2.5 Command Id Structure

- **System commands**: 0x7F01–0x7F0F (see §2.6)
- **DEKA I/O commands**: `deka_base + index`, where `deka_base` is obtained at runtime via `QueryInterface("DEKA")` (typically 0x1000, but must not be hard-coded).
- **Response flag**: bit 15 (`RESPONSE_BIT = 0x8000`). Typed response id = `0x8000 | request_id`. ACK/NACK use their own ids.

### 2.6 System Command Catalogue

| Id (hex) | Name | Request payload | Reply |
|----------|------|-----------------|-------|
| 0x7F01 | ACK | `attnReq`:1 | — |
| 0x7F02 | NACK | `nackCode`:1 | — |
| 0x7F03 | GetModuleStatus | `clearStatus`:1 | RSP: `statusWord`:1, `motorAlerts`:1 |
| 0x7F04 | KeepAlive | — | ACK |
| 0x7F05 | FailSafe | — | ACK |
| 0x7F06 | SetNewModuleAddress | `moduleAddress`:1 | ACK |
| 0x7F07 | QueryInterface | `interfaceName`: null-term string | RSP: `packetID`:2, `numValues`:2 |
| 0x7F08 | StartProgramDownload | — | ACK |
| 0x7F09 | ProgramDownloadChunk | — | ACK |
| 0x7F0A | SetModuleLEDColor | `redPower`:1, `greenPower`:1, `bluePower`:1 | ACK |
| 0x7F0B | GetModuleLEDColor | — | RSP: `redPower`:1, `greenPower`:1, `bluePower`:1 |
| 0x7F0C | SetModuleLEDPattern | `rgbtStep0..15`: 16 × 4 B | ACK |
| 0x7F0D | GetModuleLEDPattern | — | RSP: 16 × 4 B |
| 0x7F0E | DebugLogLevel | `groupNumber`:1, `verbosityLevel`:1 | ACK |
| 0x7F0F | Discovery | — | Discovery_RSP: `parent`:1 (broadcast, multiple replies) |

LED pattern step layout: each 4-byte step is `[tenths_of_seconds, blue, green, red]`
little-endian. A step of all zeros terminates the pattern early.

### 2.7 DEKA Command Catalogue (Indices 0x00–0x30)

Commands at DEKA indices 0x00–0x30 are universally agreed across firmware
versions. The typed device API uses only these. Full payload layouts in
`docs/rhsp-protocol.json`; abbreviated here.

| Idx | Id (relative) | Name | Key fields |
|-----|---------------|------|------------|
| 0x00 | +0x00 | GetBulkInputData | — → large aggregate snapshot (§2.9) |
| 0x01 | +0x01 | SetSingleDIOOutput | `dioPin`:1, `value`:1 |
| 0x02 | +0x02 | SetAllDIOOutputs | `values`:1 (bitmask) |
| 0x03 | +0x03 | SetDIODirection | `dioPin`:1, `directionOutput`:1 |
| 0x04 | +0x04 | GetDIODirection | `dioPin`:1 → `directionOutput`:1 |
| 0x05 | +0x05 | GetSingleDIOInput | `dioPin`:1 → `inputValue`:1 |
| 0x06 | +0x06 | GetAllDIOInputs | — → `inputValues`:1 (bitmask) |
| 0x07 | +0x07 | GetADC | `adcChannel`:1, `rawMode`:1 → `adcValue`:2 |
| 0x08 | +0x08 | SetMotorChannelMode | `motorChannel`:1, `motorMode`:1, `floatAtZero`:1 |
| 0x09 | +0x09 | GetMotorChannelMode | `motorChannel`:1 → `motorChannelMode`:1, `floatAtZero`:1 |
| 0x0A | +0x0A | SetMotorChannelEnable | `motorChannel`:1, `enabled`:1 |
| 0x0B | +0x0B | GetMotorChannelEnable | `motorChannel`:1 → `enabled`:1 |
| 0x0C | +0x0C | SetMotorChannelCurrentAlertLevel | `motorChannel`:1, `currentLimit`:2 |
| 0x0D | +0x0D | GetMotorChannelCurrentAlertLevel | `motorChannel`:1 → `currentLimit`:2 |
| 0x0E | +0x0E | ResetMotorEncoder | `motorChannel`:1 |
| 0x0F | +0x0F | SetMotorConstantPower | `motorChannel`:1, `powerLevel`:2 signed |
| 0x10 | +0x10 | GetMotorConstantPower | `motorChannel`:1 → `powerLevel`:2 |
| 0x11 | +0x11 | SetMotorTargetVelocity | `motorChannel`:1, `velocity`:2 signed |
| 0x12 | +0x12 | GetMotorTargetVelocity | `motorChannel`:1 → `velocity`:2 |
| 0x13 | +0x13 | SetMotorTargetPosition | `motorChannel`:1, `position`:4, `atTargetTolerance`:2 |
| 0x14 | +0x14 | GetMotorTargetPosition | `motorChannel`:1 → `targetPosition`:4, `atTargetTolerance`:2 |
| 0x15 | +0x15 | GetMotorAtTarget | `motorChannel`:1 → `atTarget`:1 |
| 0x16 | +0x16 | GetMotorEncoderPosition | `motorChannel`:1 → `currentPosition`:4 signed |
| 0x17 | +0x17 | SetMotorPIDCoefficients | `motorChannel`:1, `mode`:1, `p`:4, `i`:4, `d`:4 (Q16) |
| 0x18 | +0x18 | GetMotorPIDCoefficients | `motorChannel`:1, `mode`:1 → `p`:4, `i`:4, `d`:4 (Q16) |
| 0x19 | +0x19 | SetPWMConfiguration | `pwmChannel`:1, `framePeriod`:2 |
| 0x1A | +0x1A | GetPWMConfiguration | `pwmChannel`:1 → `framePeriod`:2 |
| 0x1B | +0x1B | SetPWMPulseWidth | `pwmChannel`:1, `pulseWidth`:2 |
| 0x1C | +0x1C | GetPWMPulseWidth | `pwmChannel`:1 → `pulseWidth`:2 |
| 0x1D | +0x1D | SetPWMEnable | `pwmChannel`:1, `enable`:1 |
| 0x1E | +0x1E | GetPWMEnable | `pwmChannel`:1 → `enabled`:1 |
| 0x1F | +0x1F | SetServoConfiguration | `servoChannel`:1, `framePeriod`:2 |
| 0x20 | +0x20 | GetServoConfiguration | `servoChannel`:1 → `framePeriod`:2 |
| 0x21 | +0x21 | SetServoPulseWidth | `servoChannel`:1, `pulseWidth`:2 |
| 0x22 | +0x22 | GetServoPulseWidth | `servoChannel`:1 → `pulseWidth`:2 |
| 0x23 | +0x23 | SetServoEnable | `servoChannel`:1, `enable`:1 |
| 0x24 | +0x24 | GetServoEnable | `servoChannel`:1 → `enabled`:1 |
| 0x25 | +0x25 | I2CWriteSingleByte | `i2cChannel`:1, `slaveAddress`:1, `byteToWrite`:1 |
| 0x26 | +0x26 | I2CWriteMultipleBytes | `i2cChannel`:1, `slaveAddress`:1, `numBytes`:1, `bytesToWrite`:≤121 |
| 0x27 | +0x27 | I2CReadSingleByte | `i2cChannel`:1, `slaveAddress`:1 → ACK; data via status query |
| 0x28 | +0x28 | I2CReadMultipleBytes | `i2cChannel`:1, `slaveAddress`:1, `numBytes`:1 → ACK |
| 0x29 | +0x29 | I2CReadStatusQuery | `i2cChannel`:1 → `i2cStatus`:1, `byteRead`:1, `payloadBytes`:≤121 |
| 0x2A | +0x2A | I2CWriteStatusQuery | `i2cChannel`:1 → `i2cStatus`:1, `numBytes`:1 |
| 0x2B | +0x2B | I2CConfigureChannel | `i2cChannel`:1, `speedCode`:1 |
| 0x2C | +0x2C | PhoneChargeControl | `enable`:1 |
| 0x2D | +0x2D | PhoneChargeQuery | — → `enable`:1 |
| 0x2E | +0x2E | InjectDataLogHint | `length`:1, `hintText`:≤121 |
| 0x2F | +0x2F | I2CConfigureQuery | `i2cChannel`:1 → `speedCode`:1 |
| 0x30 | +0x30 | ReadVersionString | — → `length`:1, `versionString`:40 |

### 2.8 High-Id Command Map Divergence (≥ 0x31)

From DEKA index 0x31 onward, the vendor Python package and REV's stock
firmware/`librhsp` assign different commands to the same ids. The new library
treats the **stock-firmware map as canonical**.

| Idx | Stock firmware / librhsp | Legacy Python rhsp |
|-----|--------------------------|---------------------|
| 0x31 | FTDI_RESET_CONTROL | GetBulkPIDData |
| 0x32 | FTDI_RESET_QUERY | I2CBlockReadConfig |
| 0x33 | SET_MOTOR_PIDF_COEFFICIENTS | I2CBlockReadQuery |
| 0x34 | I2C_WRITE_READ_MULTIPLE_BYTES (agree) | I2CWriteReadMultipleBytes |
| 0x35 | GET_MOTOR_PIDF_COEFFICIENTS | IMUBlockReadConfig |
| 0x36 | I2C_TRANSACTION | IMUBlockReadQuery |
| 0x37 | I2C_QUERY_TRANSACTION | GetBulkMotorData |
| 0x38 | SET_BULK_OUTPUT_DATA | GetBulkADCData |
| 0x39 | READ_VERSION (binary) | GetBulkI2CData |

**Implementation rule**: `catalogue.py` marks any command with `group="deka"` and
`index >= 0x31` as `legacy=True`. Legacy commands are available only through
`session.transaction()`. No typed device method calls a legacy command. Typed
methods that need bulk motor/velocity/ADC/I2C data use `GetBulkInputData`
(index 0x00), which is firmware-agnostic.

### 2.9 Bulk Response Payloads

`GetBulkInputData` (0x1000) returns a full I/O snapshot in one transaction:

```
digitalInputs(1); motor0..3Encoder(4 each, signed-32); motorStatus(1);
motor0..3Velocity(2 each, signed-16); motor0..3mode(1 each);
analogInput0..3(2 each); gpioCurrent_mA, i2cCurrent_mA, servoCurrent_mA,
batteryCurrent_mA (2 each); motor0..3current_mA(2 each); mon5v_mV(2);
batteryVoltage_mV(2); servo0..5cmd(2 each); servo0..5framePeriod_us(2 each);
i2c0..3data(10 each); imuBlock(10); i2c0..3Status(1 each); imuStatus(1);
monotonicTime(4)
```

Note: the vendor source misspells the timestamp field as `mototonicTime`.
`BulkInputData.from_response()` handles both spellings.

### 2.10 Data Encoding Conventions

| Type | Encoding |
|------|----------|
| All integers | Little-endian |
| Signed integers | Two's complement. Encoder position: 32-bit signed. Target velocity, power: 16-bit signed. Power range: ±32767. |
| PID coefficients | **Q16 fixed-point**: `int_value = round(float_value * 65536)`, stored as signed 4-byte. Decode: `int_value / 65536.0`. |
| Strings | `ReadVersionString`: `length` byte + up to 40 ASCII bytes (not null-terminated). Format: `"HW: 20, Maj: 1, Min: 8, Eng: 2"`. |
| Servo pulse widths | Microseconds. Range: 500–2500 µs. Default frame period: 20000 µs (50 Hz). Angle (0–180°): `500 + angle * 2000 / 180`. |
| Variable-length payload fields | Last field in a command absorbs remaining bytes (`bytesToWrite`, `payloadBytes`, `interfaceName`, `hintText`, `versionString`). |

### 2.11 Motor Modes and ADC Channels

**MotorMode**: `0 = CONSTANT_POWER`, `1 = CONSTANT_VELOCITY`, `2 = POSITION_TARGET`, `3 = CONSTANT_CURRENT`

**ZeroPowerBehavior**: `0 = BRAKE_AT_ZERO`, `1 = FLOAT_AT_ZERO`

**ADC channels**:
```
0–3   Analog input 0–3          8–11  Motor 0–3 current (mA)
4     GPIO current               12    5V monitor (mV)
5     I2C bus current            13    Battery monitor (mV)
6     Servo current              14    CPU temperature
7     Battery current
```

`rawMode=0` returns scaled (mA or mV); `rawMode=1` returns raw ADC counts.

### 2.12 NACK Codes

When a command is rejected, the hub sends NACK (0x7F02) with a single `nackCode` byte.
The new library raises `NackError(code, description)`. Codes (from DuckLynx `RHSP.md`):

| Range | Meaning |
|-------|---------|
| 0–9 | Parameter #N out of range |
| 10–17 | GPIO #(code−10) not configured for output |
| 18 | No GPIO pins configured for output |
| 20–27 | GPIO #(code−20) not configured for input |
| 28 | No GPIO pins configured for input |
| 30 | Servo not fully configured before enable |
| 31 | Battery too low for servo |
| 40 | I2C master busy |
| 41 | I2C operation in progress (poll again) |
| 42 | I2C no results pending |
| 43 | I2C query mismatch |
| 44 | I2C timeout — SDA stuck |
| 45 | I2C timeout — SCK stuck |
| 46 | I2C timeout |
| 50 | Motor not fully configured before enable |
| 51 | Command not valid for selected motor mode |
| 52 | Battery too low for motor |
| 253 | Command implementation pending |
| 254 | Command routing error |
| 255 | Packet type id unknown |

### 2.13 Module Status Bitfields

`GetModuleStatus` returns two bytes decoded by `ModuleStatus`:

**Byte 0 — Module status**:
bits 0–5: keep-alive timeout, device reset, fail-safe, controller over-temp, battery low, HIB fault.

**Byte 1 — Motor status (alerts)**:
bits 0–3: motor 0–3 lost encoder counts; bits 4–7: motor 0–3 driver overheat.

### 2.14 I2C Sub-Bus and Sensor Bring-Up

The hub bridges up to **4 external I2C channels** plus an internal IMU bus.
I2C reads are two-phase: issue `I2CRead…`, then poll `I2CReadStatusQuery` until
`i2cStatus` is not NACK-41.

| Sensor | I2C address | Chip |
|--------|-------------|------|
| REV Color Sensor | 0x39 (device id 0x60) | APDS-9960 |
| IMU | 0x28 | BNO055 |
| 2 m Distance Sensor | 0x29 | VL53L0X |

**Color sensor bring-up** (APDS-9960): write `ENABLE`/`ATIME`/`PPULSE` registers
using `COMMAND_BIT | register`, verify device id 0x60, read color channels with
`COMMAND_BIT | MULTI_BYTE_BIT | register` + 2-byte read.

**Distance sensor bring-up** (VL53L0X): full ST initialization sequence — read
`stop_variable`, set signal-rate limit, load SPAD config, write default tuning
register map, configure GPIO interrupt, run `performSingleRefCalibration(0x40)`
then `(0x00)`, `setTimeout(200)`, `startContinuous()`. Then
`readRangeContinuousMillimeters()` polls interrupt status, reads range, clears
interrupt. **Note**: `OSC_CALIBRATE_VAL = 0xF8` was missing in vendor code; must
be included in `sensors/registers.py`.

**IMU bring-up** (BNO055): `IMUBlockReadConfig(startRegister, numberOfBytes,
readInterval_ms)` to start autonomous hub polling; cached results retrieved via
`GetBulkInputData` (`imuBlock` field) or `IMUBlockReadQuery`.

---

## 3. Target Module Layout

```
src/rhsp/
  __init__.py       Public API: connect(), Hub, enumerate_hubs(), errors, enums, bulk dataclasses
  protocol.json     Packaged copy of docs/rhsp-protocol.json (shipped in wheel; runtime data)
  catalogue.py      Loads JSON → Command/Response tables; legacy tagging; runtime_packet_id()
  codec.py          Field/Command/Response dataclasses + generic encode/decode
  framing.py        FRAME=b"DK", HEADER struct, checksum, build_frame(), FrameParser, RawPacket
  transport.py      Transport Protocol; SerialTransport; LoopbackTransport
  session.py        Session: transaction() + ~40 typed methods + msg_num sequencing
  discovery.py      enumerate_hubs(), connect(), drained discover(), QueryInterface base
  hub.py            Hub: single address, init_peripherals()+rollback, explicit keep_alive()
  errors.py         RhspError, ProtocolError, ChecksumError, RhspTimeoutError, NackError
  enums.py          IntEnum/IntFlag from JSON enums (MotorMode, NackCode, ModuleStatusBits, …)
  devices/
    motor.py        Motor (channels 0–3)
    servo.py        Servo (channels 0–5)
    dio.py          DIOPin (pins 0–7)
    adc.py          ADCPin (channels 0–3)
    i2c.py          I2CDevice, I2CChannel
    bulk.py         BulkInputData, ModuleStatus dataclasses
  sensors/
    color.py        ColorSensor / ColorSensorV3 (APDS-9960)
    distance.py     Distance2m (VL53L0X)
    imu.py          IMU (BNO055)
    registers.py    Register maps for all three sensor chips (incl. OSC_CALIBRATE_VAL=0xF8)
  py.typed
```

**Deleted from `src/rhsp/`**: `__main__.py`, `internal/` package, `messages.py`,
`rshp_serial.py`, `printDict`, all `REV*` hex-string classes.

---

## 4. Core Interface Signatures

These are the binding contracts for each module. Implementation must match.

### 4.1 `framing.py`

```python
FRAME: bytes = b"DK"
HEADER: struct.Struct  # "<2sHBBBBH"
MAX_PACKET: int = 523  # 512 payload + 10 header + 1 checksum

def checksum(buf: bytes) -> int: ...
def build_frame(dest: int, src: int, msg: int, ref: int,
                ptype: int, payload: bytes) -> bytes: ...

@dataclass(frozen=True)
class RawPacket:
    dest: int; src: int; msg_num: int; ref_num: int
    packet_type: int; payload: bytes

class FrameParser:
    def feed(self, data: bytes) -> Iterator[RawPacket]: ...
```

### 4.2 `codec.py`

```python
@dataclass(frozen=True, slots=True)
class Field:
    name: str; nbytes: int; offset: int
    signed: bool = False
    fixed_point: int | None = None  # denominator, e.g. 65536 for Q16
    unit: str | None = None
    enum: str | None = None
    range: tuple[int,int] | None = None

@dataclass(frozen=True, slots=True)
class Command:
    name: str; id: int; group: str; index: int
    fields: tuple[Field, ...]
    reply_kind: str; reply_id: int; reply_name: str
    legacy: bool = False; notes: str = ""

@dataclass(frozen=True, slots=True)
class Response:
    name: str; id: int; fields: tuple[Field, ...]

def encode_payload(fields: Sequence[Field], values: dict[str, Any]) -> bytes: ...
def decode_payload(fields: Sequence[Field], data: bytes) -> dict[str, Any]: ...
```

Encoding rules: `int.to_bytes(n, "little", signed=field.signed)`;
Q16: `round(v * 65536)` → signed-4 bytes; last field is variable-length.

### 4.3 `catalogue.py`

```python
COMMANDS: dict[str, Command]         # keyed by name
RESPONSES_BY_ID: dict[int, Response] # keyed by numeric id

def runtime_packet_id(cmd: Command, deka_base: int) -> int: ...
# Returns deka_base + cmd.index for deka group, else cmd.id
```

### 4.4 `transport.py`

```python
class Transport(Protocol):
    def write(self, data: bytes) -> None: ...
    def read(self, n: int) -> bytes: ...
    def reset_input(self) -> None: ...
    def close(self) -> None: ...

class SerialTransport:
    def __init__(self, port: str, baud: int = 460800,
                 timeout: float = 1.0, inter_byte_timeout: float = 0.01): ...

class LoopbackTransport: ...  # in-memory; for tests
```

### 4.5 `session.py`

```python
class Session:
    def __init__(self, transport: Transport, retries: int = 3, timeout: float = 1.0): ...

    def transaction(self, command_name: str, dest: int, **fields) -> dict | None: ...

    # ~40 typed methods (subset shown):
    def keep_alive(self, dest: int) -> None: ...
    def fail_safe(self, dest: int) -> None: ...
    def query_interface(self, dest: int, name: str) -> tuple[int, int]: ...
    def get_module_status(self, dest: int, clear: bool = False) -> ModuleStatus: ...
    def discover(self, dest: int = 0xFF) -> list[RawPacket]: ...
    def get_bulk_input_data(self, dest: int) -> BulkInputData: ...
    def set_motor_channel_mode(self, dest: int, channel: int,
                                mode: MotorMode, float_at_zero: bool) -> None: ...
    def set_motor_channel_enable(self, dest: int, channel: int, enabled: bool) -> None: ...
    def set_motor_constant_power(self, dest: int, channel: int, power: int) -> None: ...
    def set_motor_target_velocity(self, dest: int, channel: int, velocity: int) -> None: ...
    def set_motor_target_position(self, dest: int, channel: int,
                                   position: int, tolerance: int) -> None: ...
    def get_motor_encoder_position(self, dest: int, channel: int) -> int: ...
    def get_motor_at_target(self, dest: int, channel: int) -> bool: ...
    def reset_motor_encoder(self, dest: int, channel: int) -> None: ...
    def set_servo_configuration(self, dest: int, channel: int, frame_period: int) -> None: ...
    def set_servo_pulse_width(self, dest: int, channel: int, pulse_width: int) -> None: ...
    def set_servo_enable(self, dest: int, channel: int, enabled: bool) -> None: ...
    def set_dio_direction(self, dest: int, pin: int, output: bool) -> None: ...
    def set_single_dio_output(self, dest: int, pin: int, value: bool) -> None: ...
    def get_single_dio_input(self, dest: int, pin: int) -> bool: ...
    def get_adc(self, dest: int, channel: int, raw: bool = False) -> int: ...
    def i2c_write_multiple_bytes(self, dest: int, i2c_channel: int,
                                  address: int, data: bytes) -> None: ...
    def i2c_read_multiple_bytes(self, dest: int, i2c_channel: int,
                                 address: int, num_bytes: int) -> None: ...
    def i2c_read_status_query(self, dest: int, i2c_channel: int) -> tuple[int, bytes]: ...
    def set_module_led_color(self, dest: int, r: int, g: int, b: int) -> None: ...
    def read_version_string(self, dest: int) -> str: ...
```

`_msg_num` is a persistent counter (1..255, never 0). Wraps from 255 back to 1.
Discovery sends to 0xFF; collects replies until a quiet window; exempt from
`ref_num` correlation.

### 4.6 `discovery.py` + `hub.py`

```python
def enumerate_hubs() -> list[str]: ...
# Returns serial port names whose USB serial number starts with "D"

def connect(port: str | None = None) -> Hub: ...
# Opens SerialTransport → Session → drained discover() → QueryInterface("DEKA")
# → constructs Hub(s); returns the parent Hub

class Hub:
    session: Session
    address: int
    parent: bool
    motors: list[Motor]    # 4
    servos: list[Servo]    # 6
    dio: list[DIOPin]      # 8
    adc: list[ADCPin]      # 4
    i2c: list[I2CChannel]  # 4

    def init_peripherals(self) -> None: ...
    # Runs §2.15 bring-up recipe; calls fail_safe() rollback on error

    def keep_alive(self) -> None: ...
    # Sends KeepAlive; caller responsible for ≥ 1/2.5 s frequency
```

Hub stores a single `address` (no `module`+`destinationModule` duplication).

### 4.7 `errors.py`

```python
class RhspError(Exception): ...
class ProtocolError(RhspError): ...
class ChecksumError(ProtocolError): ...
class RhspTimeoutError(RhspError): ...
class NackError(RhspError):
    code: int
    description: str
```

---

## 5. Sequencing Recipes

### 5.1 Mandatory Session Bring-Up (in order)

1. `enumerate_hubs()` — find port(s) with USB serial number starting with `D`.
2. Open `SerialTransport(port, baud=460800)`.
3. `discover(dest=0xFF)` — broadcast; collect all `Discovery_RSP` replies;
   create one logical module per reply; extract `parent` and `address`.
4. `QueryInterface("DEKA")` — obtain runtime `deka_base`; store in session.
5. `init_peripherals()` — run the initialization recipe (§5.2).
6. Begin keep-alive loop: call `hub.keep_alive()` at least every ~2 s.

### 5.2 Peripheral Initialization (per channel)

| Block | Count | Init commands |
|-------|------:|---------------|
| Motors | 4 | `SetMotorChannelMode(ch, CONSTANT_POWER, FLOAT_AT_ZERO)` then `SetMotorConstantPower(ch, 0)` |
| I2C channels | 4 | No bus traffic until a device is attached |
| DIO pins | 8 | No init; configure direction before use |
| Servos | 6 | `SetServoConfiguration(ch, framePeriod=20000)` |
| ADC pins | 4 | No init |

`init_peripherals()` calls `fail_safe()` and re-raises on any error mid-sequence.

### 5.3 Motor — Open-Loop Power

```
set_motor_channel_mode(ch, CONSTANT_POWER, float_at_zero=True)
set_motor_constant_power(ch, 0)
set_motor_channel_enable(ch, True)
set_motor_constant_power(ch, power)   # power in [-32767, 32767]
# keep_alive() at least every 2 s
```

### 5.4 Motor — Closed-Loop Velocity

```
set_motor_channel_mode(ch, CONSTANT_POWER, float_at_zero=True)
set_motor_constant_power(ch, 0)
set_motor_channel_enable(ch, True)
set_motor_channel_mode(ch, CONSTANT_VELOCITY, float_at_zero=True)
set_motor_target_velocity(ch, velocity)   # signed counts/s
# read velocity: get_bulk_input_data().motor_N_velocity
```

### 5.5 Motor — Position Target

```
# (same init as velocity)
set_motor_channel_mode(ch, POSITION_TARGET, float_at_zero=True)
set_motor_target_position(ch, position, tolerance)
# poll: get_motor_at_target(ch) until True
```

### 5.6 Servo

```
set_servo_configuration(ch, frame_period=20000)   # 50 Hz
set_servo_pulse_width(ch, 1500)                   # center
set_servo_enable(ch, True)
set_servo_pulse_width(ch, pw)   # 500..2500 µs; angle: 500 + angle * 2000/180
```

### 5.7 Digital I/O

Output: `set_dio_direction(pin, output=True)` then `set_single_dio_output(pin, value)`.
Input: `set_dio_direction(pin, output=False)` then `get_single_dio_input(pin)`.

### 5.8 I2C Read Sequence

```
i2c_read_multiple_bytes(dest, channel, address, num_bytes)  # returns ACK
# poll until not NACK 41:
while True:
    status, data = i2c_read_status_query(dest, channel)
    if status != NACK_41: break
```

---

## 6. Protocol-Alignment Fixes (vs. Vendor Code)

These are the bugs in `vendor/rhsp/` that must not exist in `src/rhsp/`:

| Ref | Fix |
|-----|-----|
| P1 | `QueryInterface("DEKA")` obtains `deka_base` at runtime. Never hard-code 0x1000. |
| P2 | Ids ≥ 0x31 use stock-firmware map. Legacy commands behind `transaction()`. |
| P3 | `_msg_num` is a persistent counter (1..255, never 0, wraps 255→1). |
| P4 | Response validated: `packet_type == expected_id` AND `ref_num == sent_msg_num`. |
| P5 | Receive buffer: 512 bytes. `MAX_PACKET = 523`. |
| P6 | NACK raises `NackError(code, description)`. Never swallowed or printed. |
| P7-a | `I2CChannel.configure` passes `session` correctly. |
| P7-b | `DIOPin.get_direction` returns the value. |
| P7-c | `i2c_block_read_config` / `imu_block_read_config` set payload fields, not message attrs. |
| P7-d | `set_current_pid_coefficients` calls the setter, not the getter. |
| P7-e | `I2CConfigureQuery_RSP` is registered in the response table. |
| P7-f | `OSC_CALIBRATE_VAL = 0xF8` defined in `sensors/registers.py`. |
| P7-g | `GetPWMPulseWidth` response payload is 2 bytes (matching the setter). |

---

## 7. Hardware-Free Verification Strategy

All of these pass with no physical hub present. Hardware tests live in `examples/`
and are skipped when no hub is enumerated.

### 7.1 Golden Frame Vectors (`tests/test_framing.py`)

Assert `build_frame(...)` produces the exact byte sequences in §2.2 (msg_num=1
variants). Assert `FrameParser` reconstructs identical `RawPacket` from the same
bytes. Assert `ChecksumError` is raised on a 1-bit-flipped checksum.

### 7.2 Codec Round-Trips (`tests/test_codec.py`)

For every `Command` and `Response` in the catalogue:
- `decode_payload(fields, encode_payload(fields, values)) == values`
- Signed min/max values for signed fields; Q16 extremes for PID fields.
- 512-byte trailing payload for variable-length fields.
- Assert field `offset` values are contiguous.

### 7.3 Session + FakeHub (`tests/fakehub.py`, `tests/test_session.py`)

`FakeHub` is a `LoopbackTransport`-backed hub simulator:
- Answers `KeepAlive` with ACK.
- Answers `Discovery` with a `Discovery_RSP` (sets `parent=True`).
- Answers `QueryInterface("DEKA")` with `packetID=0x1000, numValues=49`.
- Returns configurable NACK for negative-path tests.

Session tests assert:
- `_msg_num` is never 0; increments each transaction; wraps 255→1.
- `ref_num` correlation enforced; mismatched response discarded.
- NACK → `NackError` (not retry).
- Timeout → retry → `RhspTimeoutError`.
- Multi-reply discovery returns N `Hub` objects.
- `QueryInterface` correctly sets `session.deka_base`.

### 7.4 Device Payload Assertions (`tests/test_devices.py`)

Using `FakeHub`:
- `set_servo_pulse_width(0, 1500)` → exact bytes `00 DC 05` in payload.
- `set_motor_constant_power(0, 16000)` → `00 80 3E` in payload.
- `get_bulk_input_data()` → `BulkInputData` with correct signed interpretation.
- All P7-series bugs absent by construction.

### 7.5 Sensor Register Sequence (`tests/test_sensors.py`)

`FakeHub` records the I2C write/read calls:
- Color sensor init emits the APDS-9960 register sequence from §2.14.
- Distance sensor init emits the VL53L0X sequence from §2.14.
- IMU init issues `IMUBlockReadConfig` with correct parameters.

---

## 8. Open Implementation Questions

These are non-blocking; carry into implementation sprints.

1. **`QueryInterface` string padding**: send raw `b"DEKA\x00"` vs zero-pad to
   121-byte field. Expose a per-field `pad` switch. Confirm on hardware.
2. **`I2CReadStatusQuery` poll budget** (NACK 41 "poll again"): default 5 polls
   × 1 ms. Verify timing on real sensor.
3. **Stock-firmware high-id commands** (PIDF, I2C_TRANSACTION, binary READ_VERSION,
   SET_BULK_OUTPUT_DATA): carry as `firmware="stock"` catalogue tags, no typed
   methods until validated on a real hub.
4. **`BulkInputData` signed fields**: JSON RSP overlay omits `signed` flag for
   encoder/velocity. `bulk.py` reinterprets them as signed; flag for an upstream
   JSON overlay correction so the inverted generator stays consistent.
