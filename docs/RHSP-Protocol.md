# REV Hub Serial Protocol (RHSP)

This document describes the wire protocol and software interface used by the
`rhsp` package ([src/rhsp/](../src/rhsp/)) to communicate with a **REV Robotics
Expansion / Control Hub** over a USB serial connection. It was reconstructed by
reviewing the Python source, principally
[src/rhsp/internal/messages.py](../src/rhsp/internal/messages.py) (the message
definitions) and [src/rhsp/client.py](../src/rhsp/client.py) (the framing,
send/receive, and checksum logic).

It is intended to be **complete enough for an agent to reimplement the protocol
in another language/environment**. It ships with two companion artifacts in this
directory:

- **[rhsp-protocol.json](rhsp-protocol.json)** — a machine-readable catalogue of
  every command and response, with field layouts, byte offsets, enums, and
  sequencing recipes (§10).
- **[generate_protocol_json.py](generate_protocol_json.py)** — the generator that
  introspects the live message classes to (re)produce that JSON, so it can never
  drift from the implementation.

§9 covers the **ordering/sequencing** constraints (what must be set up before
what), and §13 lists external **validation references** (REV's official C library,
the FTC SDK source, and REV's Saleae protocol analyzer).

> **Provenance.** This code descends from the community
> `unofficial-rev-port` projects —
> [SerialHubControl](https://github.com/unofficial-rev-port/SerialHubControl)
> (the most likely direct upstream) and the GUI tool
> [REVHubInterface](https://github.com/unofficial-rev-port/REVHubInterface),
> which share the same message layer. The `messages.py` message definitions were
> originally decompiled from a `.pyc` (see the trailing
> `# okay decompiling REVmessages.pyc` marker), then reorganized into this
> `rhsp` package (this repo is `League-Robotics/python-serial-hub-control`). The
> protocol itself is REV's firmware-level "DEKA" interface; field names and
> semantics below come directly from that source.

---

## 1. Physical / link layer

| Parameter | Value | Source |
|-----------|-------|--------|
| Transport | USB CDC serial (virtual COM port) | [client.py](../src/rhsp/client.py) |
| Baud rate | **460800** | `Client._serial_config` |
| Data bits | 8 | `Client._serial_config` |
| Parity | None | `Client._serial_config` |
| Stop bits | 1 | `Client._serial_config` |
| Flow control | None | — |

Note: the `RHSPSerial` wrapper ([rshp_serial.py](../src/rhsp/rshp_serial.py))
defaults to 9600 baud, but `Client` always overrides it with the 460800
configuration above when it opens a port.

### Port discovery

`comPort.enumerate()` ([rshp_serial.py:183](../src/rhsp/rshp_serial.py#L183))
scans system serial ports (`serial.tools.list_ports`) and selects a hub by USB
descriptor:

- The port's `hwid` must contain a `SER=` (serial-number) field, **and**
- that serial number must start with the letter `D`.

This `D…` serial-number prefix is how a genuine REV hub is distinguished from
other USB serial devices.

---

## 2. Packet framing

All traffic is exchanged as discrete packets. Internally the code manipulates
packets as **hex-ASCII strings** (two characters per byte) and only converts to
raw bytes at the moment of `write()`/`read()`; this document describes the
**bytes on the wire**.

### 2.1 Packet structure

```
+--------+--------+---------+--------+--------+--------+--------+----------+---------+--------+
| 0x44   | 0x4B   | length  |  dest  | source | msgNum | refNum | cmd/type | payload | chksum |
| (1B)   | (1B)   | (2B LE) |  (1B)  | (1B)   | (1B)   | (1B)   | (2B LE)  |  (N B)  | (1B)   |
+--------+--------+---------+--------+--------+--------+--------+----------+---------+--------+
| <----------------------- frame + 8-byte header ----------------------> |
| <----------------------------- covered by checksum -----------------------------> |
```

Byte offsets (from
[`REVPacket`](../src/rhsp/internal/messages.py#L280) index constants):

| Offset | Size | Field | Description |
|-------:|:----:|-------|-------------|
| 0 | 2 | **Frame bytes** | Constant `0x44 0x4B` (ASCII `"DK"`). Marks the start of a packet. |
| 2 | 2 | **Length** | Total packet length in bytes, little-endian (includes frame, header, payload, and checksum). |
| 4 | 1 | **Destination** | Target module address. `255` = broadcast (used for Discovery). |
| 5 | 1 | **Source** | Originating address. Set by the responder; on responses this is the module's address. |
| 6 | 1 | **Message number** | Sender's sequence id for this request. |
| 7 | 1 | **Reference number** | On a response, echoes the request's message number (correlates response↔request). |
| 8 | 2 | **Packet type / command** | Command id, little-endian. See §4. |
| 10 | N | **Payload** | Command-specific fields. May be empty. |
| 10+N | 1 | **Checksum** | 8-bit additive checksum (see §2.3). |

The fixed header is 8 bytes; with the 2 frame bytes and 1 checksum byte, the
minimum packet (empty payload) is 11 bytes.

### 2.2 Length field

`REVPacket.calcLength()`
([messages.py:309](../src/rhsp/internal/messages.py#L309)) computes:

```
length = len(header) + len(payload) + 3
       = 8           + N            + 3   (2 frame bytes + 1 checksum byte)
       = total packet size in bytes
```

The value is then byte-swapped to little-endian for transmission. On receive,
the parser rejects any frame whose declared payload length exceeds
`PAYLOAD_MAX_SIZE = 128`.

### 2.3 Checksum

8-bit additive checksum over **every byte from the frame start through the end
of the payload** (i.e. all bytes except the checksum byte itself), taken
mod 256:

```
chksum = (sum of all preceding bytes) % 256
```

Computed in `REVPacket.getPacketData()`
([messages.py:313](../src/rhsp/internal/messages.py#L313)) and verified on
receive in `Client.checkPacket()`
([client.py:218](../src/rhsp/client.py#L218)).

### 2.4 Byte order

Multi-byte integer fields are **little-endian** on the wire. The code performs
this conversion explicitly:

- For the 2-byte length and command fields via the idiom
  `x >> 8 | x % 256 << 8`.
- For general payload fields via `Client.swapEndianess()`
  ([client.py:253](../src/rhsp/client.py#L253)), which reverses the byte order
  of a hex string.

Signed fields use two's-complement (see §5).

---

## 3. Transaction model

The protocol is strictly **request/response, half-duplex, one outstanding
transaction at a time**. `Client.sendAndReceive(packet, destination)`
([client.py:65](../src/rhsp/client.py#L65)) drives every exchange:

1. Set `header.destination = destination` and assign a message number.
2. Write the encoded packet to the serial port.
3. Wait (up to ~1 s) for bytes to arrive; on timeout, retry up to
   `MaxRetries = 3` times.
4. Feed incoming bytes through a receive state machine (§3.2).
5. On a complete, checksum-valid packet, decode it (`processPacket`) and return
   the response object. On checksum failure, log and resync.

### 3.1 Responses: ACK vs. typed response

Every command has an expected reply, captured in the `printDict` table
([messages.py:2003](../src/rhsp/internal/messages.py#L2003)):

- **Setters / actions** (e.g. `SetMotorConstantPower`, `KeepAlive`,
  `ResetMotorEncoder`) reply with a bare **`ACK`** (payload: `attnReq`, 1 byte).
- **Getters / queries** (e.g. `GetADC`, `GetMotorEncoderPosition`) reply with a
  dedicated **`*_RSP`** packet whose command id is the request id OR-ed with the
  response bit (§4.3).
- On error, the hub replies with **`NACK`** (payload: `nackCode`, 1 byte).

`Client.checkResponse()` ([client.py:195](../src/rhsp/client.py#L195)) validates
that the received `packetType` equals the expected response and that the
response's `refNum` matches the request's `msgNum` (Discovery is exempted, since
it is a broadcast with multiple replies).

### 3.2 Receive state machine

The receiver parses byte-by-byte
([client.py:134](../src/rhsp/client.py#L134)) through these states:

```
WaitForFrameByte1   -- discard until 0x44 seen
WaitForFrameByte2   -- expect 0x4B (0x44 repeats stay here; anything else resets)
WaitForPacketLengthByte1
WaitForPacketLengthByte2  -- assemble little-endian length; bounds-check vs 128
WaitForPayloadBytes -- accumulate until accumulated length == declared length,
                       then verify checksum and decode
```

The destination/source/msgNum/refNum/command fields are not parsed as separate
states; they are accumulated with the payload and then sliced out by fixed
offset in `Client.processPacket()`
([client.py:226](../src/rhsp/client.py#L226)).

### 3.3 Discovery (broadcast)

`Client.discovery()` ([client.py:317](../src/rhsp/client.py#L317)) sends a
`Discovery` command to destination **255**. Unlike normal transactions, the hub
(and any daisy-chained modules) may emit **multiple** `Discovery_RSP` packets.
The client collects responses, pausing ~2 s and re-checking the input buffer,
until no more arrive. Each `Discovery_RSP` carries a `parent` byte and its
`source` becomes that module's address. One `Module` object is created per
discovered module.

### 3.4 Addressing & message numbering

- **Destination** is supplied per call; it is the module address (the address a
  `Module` was discovered at, reassignable via `SetNewModuleAddress`).
- **Message number** is managed locally inside `sendAndReceive` (it starts at 0
  for each call and increments only across retries; it is **not** a persistent
  monotonic counter across transactions — see §7).

---

## 4. Command catalogue

Command ids fall into two ranges, both defined in `MsgNum`
([messages.py:1217](../src/rhsp/internal/messages.py#L1217)).

### 4.1 System / module-management commands (0x7F01–0x7F0F)

| Cmd id | Name | Request payload | Reply |
|-------:|------|-----------------|-------|
| 0x7F01 | ACK | `attnReq`:1 | — |
| 0x7F02 | NACK | `nackCode`:1 | — |
| 0x7F03 | GetModuleStatus | `clearStatus`:1 | RSP: `statusWord`:1, `motorAlerts`:1 |
| 0x7F04 | KeepAlive | — | ACK |
| 0x7F05 | FailSafe | — | ACK |
| 0x7F06 | SetNewModuleAddress | `moduleAddress`:1 | ACK |
| 0x7F07 | QueryInterface | `interfaceName`:string | RSP: `packetID`:2, `numValues`:2 |
| 0x7F08 | StartProgramDownload | — | ACK |
| 0x7F09 | ProgramDownloadChunk | — | ACK |
| 0x7F0A | SetModuleLEDColor | `redPower`:1, `greenPower`:1, `bluePower`:1 | ACK |
| 0x7F0B | GetModuleLEDColor | — | RSP: `redPower`:1, `greenPower`:1, `bluePower`:1 |
| 0x7F0C | SetModuleLEDPattern | `rgbtStep0..15`:4 each (16×4 = 64 B) | ACK |
| 0x7F0D | GetModuleLEDPattern | — | RSP: `rgbtStep0..15`:4 each |
| 0x7F0E | DebugLogLevel | `groupNumber`:1, `verbosityLevel`:1 | ACK |
| 0x7F0F | Discovery | — | Discovery_RSP: `parent`:1 (broadcast; multiple replies) |

`QueryInterface` is the mechanism by which a host asks the hub for the base
command id of a named interface ("DEKA"), returning the `packetID` offset and
the number of commands the interface exposes.

### 4.2 DEKA I/O interface commands (base 0x1000 + index)

`DekaInterfacePrefix = 0x1000` (4096). Each command id is `0x1000 + index`. The
notation below uses `name:bytes`.

| Idx | Cmd id | Name | Request payload | Reply payload |
|----:|-------:|------|-----------------|---------------|
| 0 | 0x1000 | GetBulkInputData | — | Large aggregate snapshot (§6.1) |
| 1 | 0x1001 | SetSingleDIOOutput | `dioPin`:1, `value`:1 | ACK |
| 2 | 0x1002 | SetAllDIOOutputs | `values`:1 (bitmask) | ACK |
| 3 | 0x1003 | SetDIODirection | `dioPin`:1, `directionOutput`:1 | ACK |
| 4 | 0x1004 | GetDIODirection | `dioPin`:1 | `directionOutput`:1 |
| 5 | 0x1005 | GetSingleDIOInput | `dioPin`:1 | `inputValue`:1 |
| 6 | 0x1006 | GetAllDIOInputs | — | `inputValues`:1 (bitmask) |
| 7 | 0x1007 | GetADC | `adcChannel`:1, `rawMode`:1 | `adcValue`:2 |
| 8 | 0x1008 | SetMotorChannelMode | `motorChannel`:1, `motorMode`:1, `floatAtZero`:1 | ACK |
| 9 | 0x1009 | GetMotorChannelMode | `motorChannel`:1 | `motorChannelMode`:1, `floatAtZero`:1 |
| 10 | 0x100A | SetMotorChannelEnable | `motorChannel`:1, `enabled`:1 | ACK |
| 11 | 0x100B | GetMotorChannelEnable | `motorChannel`:1 | `enabled`:1 |
| 12 | 0x100C | SetMotorChannelCurrentAlertLevel | `motorChannel`:1, `currentLimit`:2 | ACK |
| 13 | 0x100D | GetMotorChannelCurrentAlertLevel | `motorChannel`:1 | `currentLimit`:2 |
| 14 | 0x100E | ResetMotorEncoder | `motorChannel`:1 | ACK |
| 15 | 0x100F | SetMotorConstantPower | `motorChannel`:1, `powerLevel`:2 (signed) | ACK |
| 16 | 0x1010 | GetMotorConstantPower | `motorChannel`:1 | `powerLevel`:2 |
| 17 | 0x1011 | SetMotorTargetVelocity | `motorChannel`:1, `velocity`:2 (signed) | ACK |
| 18 | 0x1012 | GetMotorTargetVelocity | `motorChannel`:1 | `velocity`:2 |
| 19 | 0x1013 | SetMotorTargetPosition | `motorChannel`:1, `position`:4, `atTargetTolerance`:2 | ACK |
| 20 | 0x1014 | GetMotorTargetPosition | `motorChannel`:1 | `targetPosition`:4, `atTargetTolerance`:2 |
| 21 | 0x1015 | GetMotorAtTarget | `motorChannel`:1 | `atTarget`:1 |
| 22 | 0x1016 | GetMotorEncoderPosition | `motorChannel`:1 | `currentPosition`:4 (signed) |
| 23 | 0x1017 | SetMotorPIDCoefficients | `motorChannel`:1, `mode`:1, `p`:4, `i`:4, `d`:4 (Q16) | ACK |
| 24 | 0x1018 | GetMotorPIDCoefficients | `motorChannel`:1, `mode`:1 | `p`:4, `i`:4, `d`:4 (Q16) |
| 25 | 0x1019 | SetPWMConfiguration | `pwmChannel`:1, `framePeriod`:2 | ACK |
| 26 | 0x101A | GetPWMConfiguration | `pwmChannel`:1 | `framePeriod`:2 |
| 27 | 0x101B | SetPWMPulseWidth | `pwmChannel`:1, `pulseWidth`:2 | ACK |
| 28 | 0x101C | GetPWNPulseWidth* | `pwmChannel`:1 | `pulseWidth`:1 |
| 29 | 0x101D | SetPWMEnable | `pwmChannel`:1, `enable`:1 | ACK |
| 30 | 0x101E | GetPWMEnable | `pwmChannel`:1 | `enabled`:1 |
| 31 | 0x101F | SetServoConfiguration | `servoChannel`:1, `framePeriod`:2 | ACK |
| 32 | 0x1020 | GetServoConfiguration | `servoChannel`:1 | `framePeriod`:2 |
| 33 | 0x1021 | SetServoPulseWidth | `servoChannel`:1, `pulseWidth`:2 | ACK |
| 34 | 0x1022 | GetServoPulseWidth | `servoChannel`:1 | `pulseWidth`:2 |
| 35 | 0x1023 | SetServoEnable | `servoChannel`:1, `enable`:1 | ACK |
| 36 | 0x1024 | GetServoEnable | `servoChannel`:1 | `enabled`:1 |
| 37 | 0x1025 | I2CWriteSingleByte | `i2cChannel`:1, `slaveAddress`:1, `byteToWrite`:1 | ACK |
| 38 | 0x1026 | I2CWriteMultipleBytes | `i2cChannel`:1, `slaveAddress`:1, `numBytes`:1, `bytesToWrite`:≤121 | ACK |
| 39 | 0x1027 | I2CReadSingleByte | `i2cChannel`:1, `slaveAddress`:1 | ACK (data via status query) |
| 40 | 0x1028 | I2CReadMultipleBytes | `i2cChannel`:1, `slaveAddress`:1, `numBytes`:1 | ACK (data via status query) |
| 41 | 0x1029 | I2CReadStatusQuery | `i2cChannel`:1 | `i2cStatus`:1, `byteRead`:1, `payloadBytes`:≤121 |
| 42 | 0x102A | I2CWriteStatusQuery | `i2cChannel`:1 | `i2cStatus`:1, `numBytes`:1 |
| 43 | 0x102B | I2CConfigureChannel | `i2cChannel`:1, `speedCode`:1 | ACK |
| 44 | 0x102C | PhoneChargeControl | `enable`:1 | ACK |
| 45 | 0x102D | PhoneChargeQuery | — | `enable`:1 |
| 46 | 0x102E | InjectDataLogHint | `length`:1, `hintText`:≤121 | ACK |
| 47 | 0x102F | I2CConfigureQuery | `i2cChannel`:1 | `speedCode`:1 |
| 48 | 0x1030 | ReadVersionString | — | `length`:1, `versionString`:40 |
| 49 | 0x1031 | GetBulkPIDData | `motorChannel`:1 | Large PID telemetry dump (§6.2) |
| 50 | 0x1032 | I2CBlockReadConfig | `channel`:1, `address`:1, `startRegister`:1, `numberOfBytes`:1, `readInterval_ms`:1 | ACK |
| 51 | 0x1033 | I2CBlockReadQuery | `channel`:1 | `address`:1, `startRegister`:1, `numberOfBytes`:1, `readInterval_ms`:1 |
| 52 | 0x1034 | I2CWriteReadMultipleBytes | `channel`:1, `address`:1, `startRegister`:1, `numberOfBytes`:1 | ACK |
| 53 | 0x1035 | IMUBlockReadConfig | `startRegister`:1, `numberOfBytes`:1, `readInterval_ms`:1 | ACK |
| 54 | 0x1036 | IMUBlockReadQuery | `channel`:1 | `startRegister`:1, `numberOfBytes`:1, `readInterval_ms`:1 |
| 55 | 0x1037 | GetBulkMotorData | — | Motor encoders/velocities/modes/time (§6.3) |
| 56 | 0x1038 | GetBulkADCData | — | All analog inputs/currents/voltages/time (§6.4) |
| 57 | 0x1039 | GetBulkI2CData | — | Cached I2C block data + IMU block (§6.5) |
| 64 | 0x1040 | GetBulkServoData | — | All servo commands & frame periods (§6.6) |

\* `GetPWNPulseWidth` is spelled with an `N` in the source and its response
declares `pulseWidth` as 1 byte while the setter uses 2 bytes — see §7.

> Note the gap: `GetBulkServoData` is index **64** (`0x1040`), not 58.

### 4.3 Response command ids

`RESPONSE_BIT = 0x8000` ([messages.py:1295](../src/rhsp/internal/messages.py#L1295)).
A typed response's command id is `RESPONSE_BIT | requestCmd`. For example
`GetADC` = `0x1007` → `GetADC_RSP` = `0x9007`. `RespNum`
([messages.py:1297](../src/rhsp/internal/messages.py#L1297)) enumerates all of
these. (ACK/NACK replies, by contrast, use their own fixed ids `0x7F01`/`0x7F02`
rather than a response-bit-encoded id.)

---

## 5. Data encoding conventions

| Concept | Encoding |
|---------|----------|
| Integers | Little-endian on the wire. |
| Signed integers | Two's-complement. Encoder position is **32-bit signed**; target velocity and power are **16-bit signed**. Sign extension is applied on decode (e.g. [internal/motors.py:124](../src/rhsp/internal/motors.py#L124)). |
| PID coefficients | **Q16 fixed-point**: transmitted as `value × 65536` in a 4-byte field; decoded by dividing by 65536 (`Q16` in [internal/motors.py:6](../src/rhsp/internal/motors.py#L6)). |
| Strings | ASCII, packed two hex chars per byte; `ReadVersionString` returns `length` + up to 40 bytes, decoded char-by-char in [module.py:136](../src/rhsp/module.py#L136). |

### 5.1 Motor modes (`motorMode`)

From [internal/motors.py](../src/rhsp/internal/motors.py#L7):

| Value | Mode |
|------:|------|
| 0 | `MODE_CONSTANT_POWER` |
| 1 | `MODE_CONSTANT_VELOCITY` |
| 2 | `MODE_POSITION_TARGET` |
| 3 | `MODE_CONSTANT_CURRENT` |

`floatAtZero`: `0 = BRAKE_AT_ZERO`, `1 = FLOAT_AT_ZERO`. Power level range
observed in tests is `±(2^15 − 1)`.

### 5.2 Servo encoding

From [servo.py](../src/rhsp/servo.py): pulse widths are in **microseconds**,
clamped to **500–2500 µs**; the default frame period is **20000 µs** (50 Hz).
`setAngle(0..180°)` maps linearly to `500 + angle × (2000/180)` µs.

### 5.3 ADC channels (`adcChannel`)

From [adc.py](../src/rhsp/adc.py#L7) / `ADCChannel` in
[messages.py:9](../src/rhsp/internal/messages.py#L9):

```
0–3  Analog input channels 0–3        8–11 Motor 0–3 current
4    GPIO current                      12   5V monitor
5    I2C bus current                   13   Battery monitor
6    Servo current                     14   CPU temperature
7    Battery current
```

`rawMode` selects raw vs. scaled readings; `adcValue` is returned as 2 bytes.

---

## 6. Bulk / aggregate response payloads

To minimize round-trips at 460800 baud, the hub offers "bulk" reads that return
a large, fixed-layout snapshot in one transaction. All fields are little-endian;
sizes in bytes shown in parentheses. (Exact field orders are in the `*_RSP_Payload`
classes in [messages.py](../src/rhsp/internal/messages.py).)

### 6.1 GetBulkInputData_RSP ([messages.py:895](../src/rhsp/internal/messages.py#L895))
A full I/O snapshot: `digitalInputs`(1); `motor0..3Encoder`(4 each);
`motorStatus`(1); `motor0..3Velocity`(2 each); `motor0..3mode`(1 each);
`analogInput0..3`(2 each); `gpioCurrent_mA`,`i2cCurrent_mA`,`servoCurrent_mA`,
`batteryCurrent_mA`(2 each); `motor0..3current_mA`(2 each); `mon5v_mV`(2);
`batteryVoltage_mV`(2); `servo0..5cmd`(2 each); `servo0..5framePeriod_us`(2 each);
`i2c0..3data`(10 each); `imuBlock`(10); `i2c0..3Status`(1 each); `imuStatus`(1);
`mototonicTime`(4) *(monotonic timestamp; field name is misspelled in the source)*.

### 6.2 GetBulkPIDData_RSP ([messages.py:1103](../src/rhsp/internal/messages.py#L1103))
Per-motor PID telemetry: for each of the **current**, **velocity**, and
**position** loops — `Pterm`, `Iterm`, `Dterm`, `Output`, `Cmd`, `Error`
(4 bytes each) — plus `monotonicTime`(4).

### 6.3 GetBulkMotorData_RSP ([messages.py:1144](../src/rhsp/internal/messages.py#L1144))
`motor0..3Encoder`(4 each); `motorStatus`(1); `motor0..3Velocity`(2 each);
`motor0..3mode`(1 each); `monotonicTime`(4). Used by `Motor.getVelocity()`.

### 6.4 GetBulkADCData_RSP ([messages.py:1163](../src/rhsp/internal/messages.py#L1163))
`analogInput0..3`(2 each); the four bus currents (2 each); `motor0..3current_mA`
(2 each); `mon5v_mV`(2); `batteryVoltage_mV`(2); `monotonicTime`(4).

### 6.5 GetBulkI2CData_RSP ([messages.py:1183](../src/rhsp/internal/messages.py#L1183))
`i2c0..3data`(10 each); `imuBlock`(10); `i2c0..3Status`(1 each); `imuStatus`(1);
`monotonicTime`(4). Returns the most recent **block-read** results cached by the
hub (see §6.7).

### 6.6 GetBulkServoData_RSP ([messages.py:1199](../src/rhsp/internal/messages.py#L1199))
`servo0..5cmd`(2 each); `servo0..5framePeriod_us`(2 each); `monotonicTime`(4).

### 6.7 Block reads (autonomous I2C/IMU polling)

`I2CBlockReadConfig` / `IMUBlockReadConfig` instruct the hub to **autonomously
poll** a register block on an I2C device (or the on-board IMU) every
`readInterval_ms`. The cached results are then retrievable cheaply via
`GetBulkI2CData` (the `i2cNdata` / `imuBlock` fields) without a per-read I2C
transaction. `*BlockReadQuery` returns the current block-read configuration.

---

## 7. I2C sub-bus and attached sensors

The hub bridges to up to **4 external I2C channels** plus an internal IMU bus.
The host drives I2C through the commands in §4.2 (single/multi byte write & read,
status query, channel speed config, block read). Reads are two-phase: issue
`I2CRead…`, then poll `I2CReadStatusQuery` to retrieve `i2cStatus` and the data
bytes ([internal/i2c.py:101](../src/rhsp/internal/i2c.py#L101)).

I2C command/address bit helpers (`I2CConstants`,
[internal/i2c.py:9](../src/rhsp/internal/i2c.py#L9)):
`COMMAND_REGISTER_BIT = 0x80`, `MULTI_BYTE_BIT = 0x20`,
`SINGLE_BYTE_BIT = 0x00`.

The higher-level device layer ([i2c.py](../src/rhsp/i2c.py),
[color.py](../src/rhsp/color.py), [distance.py](../src/rhsp/distance.py),
[imu.py](../src/rhsp/imu.py)) implements specific sensors on top of these
primitives:

| Sensor | I2C address | Notes |
|--------|------------:|-------|
| REV color sensor (APDS-9960 family) | 57 (0x39), device id 0x60 | Register map in `I2CConstants` ([internal/i2c.py](../src/rhsp/internal/i2c.py#L17)). |
| BNO055 IMU | 40 (0x28) | Full register map in `IMUConstants` ([internal/imu.py](../src/rhsp/internal/imu.py)). |
| 2 m distance sensor (VL53L0X) | configured in `Distance2m` | [distance.py](../src/rhsp/distance.py). |

These sensor protocols are layered **on top of** the RHSP I2C commands; they are
device-specific and not part of the core hub serial protocol.

---

## 8. Software object model

The package layers a clean API over the wire protocol:

```
comPort.enumerate()           → discovers hub USB ports
Client.open() / .discovery()  → opens serial, broadcasts Discovery → [Module]
  Module                      → one REV hub/module (holds its address)
    .motors[0..3]   Motor     → SetMotor* / GetMotor* commands
    .servos[0..5]   Servo     → Servo* commands (µs pulse widths)
    .adcPins[0..3]  ADCPin    → GetADC
    .dioPins[0..7]  DIOPin    → DIO commands
    .i2cChannels[0..3] I2CChannel → I2CDevice / ColorSensor / IMU / Distance2m
```

`Module.init_periphs()` ([module.py:24](../src/rhsp/module.py#L24)) constructs 4
motors, 4 I2C channels, 8 DIO pins, 6 servos, and 4 ADC pins, and initializes
motors to constant-power mode at zero. `Module.keep_alive()` must be sent
periodically or the hub's fail-safe disables outputs.

---

## 9. Sequencing & usage recipes

The protocol is mostly stateless per command, but the **hub** is stateful: some
commands must precede others, and a heartbeat must run continuously while
outputs are active. The recipes below are distilled from `Module.init_periphs()`
and the example scripts in [test/](../test/). They are also encoded
machine-readably under `"sequencing"` in
[rhsp-protocol.json](rhsp-protocol.json) (§10).

### 9.1 Mandatory session bring-up (in order)

1. **Enumerate** serial ports; pick the one whose USB serial number starts with
   `D` (§1).
2. **Open** the port at 460800 8N1, no flow control.
3. **Discovery** — send `Discovery` to destination **255**; create one logical
   module per `Discovery_RSP`, using each reply's `source` as that module's
   address. *Nothing else works until discovery has run.*
4. *(Optional but recommended)* **QueryInterface("DEKA")** to obtain the runtime
   base command id for the I/O block. This firmware uses `0x1000`, but a
   replication should not hard-code it — read `firstPacketID`/`packetID` and add
   the per-command index from §4.2.
5. **Initialize peripherals** (§9.2).
6. **Start the keep-alive heartbeat** — send `KeepAlive` at least every ~2.5 s
   for as long as any output (motor/servo/LED) is driven. If the watchdog
   expires the hub enters fail-safe and disables outputs; you must then
   re-enable them.

### 9.2 Peripheral initialization (`Module.init_periphs()`)

| Block | Count | Init action |
|-------|------:|-------------|
| Motors | 4 | `SetMotorChannelMode(ch, mode=0, floatAtZero=1)`, then `SetMotorConstantPower(ch, 0)` |
| I2C channels | 4 | handle only; no bus traffic until a device is added |
| DIO pins | 8 | handle only |
| Servos | 6 | `SetServoConfiguration(ch, framePeriod=20000)` (50 Hz) |
| ADC pins | 4 | handle only |

### 9.3 Subsystem recipes

**Motor — open-loop power** (`test_motor.py`):
```
SetMotorChannelMode(ch, mode=0 CONSTANT_POWER, floatAtZero=1)
SetMotorConstantPower(ch, 0)
SetMotorChannelEnable(ch, 1)
SetMotorConstantPower(ch, power)      # power ∈ [-32767, 32767]
# …repeat KeepAlive() ≥ every 2.5 s…
```

**Motor — closed-loop velocity** (`test_motor_velocity.py`):
```
SetMotorChannelMode(ch, 0, 1); SetMotorConstantPower(ch, 0)   # init
SetMotorChannelEnable(ch, 1)
SetMotorChannelMode(ch, mode=1 CONSTANT_VELOCITY, floatAtZero=1)
SetMotorTargetVelocity(ch, velocity)                          # signed counts/s
# read back via GetBulkMotorData → motorNVelocity (16-bit signed)
```
Note the ordering: the channel is enabled *then* switched to velocity mode in
the example. Position mode is analogous using `SetMotorTargetPosition(ch,
position, tolerance)` and polling `GetMotorAtTarget`.

**Servo** (`test_servo.py`):
```
SetServoConfiguration(ch, framePeriod=20000)   # 50 Hz
SetServoPulseWidth(ch, 1500)                   # center, µs
SetServoEnable(ch, 1)
SetServoPulseWidth(ch, pw)                     # 500..2500 µs; angle→pw = 500 + angle*2000/180
```

**Digital I/O**: `SetDIODirection(pin, 1)` to make a pin an output, then
`SetSingleDIOOutput(pin, value)`. For inputs, `SetDIODirection(pin, 0)` then
`GetSingleDIOInput(pin)` / `GetAllDIOInputs`.

**Analog**: `GetADC(channel, rawMode)` — channel per the `ADCChannel` enum;
current channels read mA, voltage channels read mV when `rawMode=0`.

**I2C color sensor (APDS-9960, addr 0x39)** (`color.py`): write
`ENABLE`/`ATIME`/`PPULSE` config registers (each as `COMMAND_BIT|register` then
value), verify device id `0x60`, then read color channels with
`COMMAND_BIT|MULTI_BYTE_BIT|register` + a 2-byte read.

**I2C distance sensor (VL53L0X, addr 0x29)** (`distance.py`): `initialize()`
performs the standard ST data-init sequence — read `stop_variable`, set the
signal-rate limit, load SPAD config, write the large default tuning register
map, configure the GPIO interrupt, run `performSingleRefCalibration(0x40)` then
`(0x00)`, `setTimeout(200)`, `startContinuous()` — after which
`readRangeContinuousMillimeters()` polls the interrupt-status register, reads the
range, and clears the interrupt. This is a long fixed sequence; replicate it
register-for-register.

**IMU (BNO055)**: `IMUBlockReadConfig(startRegister, numberOfBytes,
readInterval_ms)` to start autonomous polling, then read the cached block via
`GetBulkI2CData` (`imuBlock`) or `IMUBlockReadQuery`.

### 9.4 Block reads (latency optimization)

For sensors polled continuously, configure the hub to auto-poll a register block
(`I2CBlockReadConfig` / `IMUBlockReadConfig`) and then retrieve cached data with
the bulk reads (§6.5) instead of issuing a full I2C transaction per sample.

### 9.5 Worked wire example

These are exact bytes produced by `REVPacket.getPacketData()` (frame +
little-endian fields + additive checksum). Use them as encoder test vectors:

| Command | Bytes (hex) |
|---------|-------------|
| `KeepAlive` (dest=1, msg=0) | `44 4B 0B 00 01 00 00 00 04 7F 1E` |
| `Discovery` (dest=255, msg=0) | `44 4B 0B 00 FF 00 00 00 0F 7F 27` |
| `SetServoPulseWidth` ch=0, pw=1500µs (dest=1) | `44 4B 0E 00 01 00 00 00 21 10 00 DC 05 B0` |
| `SetMotorConstantPower` ch=0, power=16000 (dest=1) | `44 4B 0E 00 01 00 00 00 0F 10 00 80 3E 7B` |

Decoding the servo example: `44 4B` frame · `0E 00` length=14 · `01` dest · `00`
src · `00` msgNum · `00` refNum · `21 10` packetType=`0x1021`
(SetServoPulseWidth) · `00` servoChannel · `DC 05` pulseWidth=`0x05DC`=1500 ·
`B0` checksum. The checksum is `(0x44+0x4B+0x0E+…+0x05) mod 256 = 0xB0`.

---

## 10. Machine-readable command catalogue (`rhsp-protocol.json`)

For programmatic replication, the complete command set is emitted as a single
structured file: **[rhsp-protocol.json](rhsp-protocol.json)**. It is generated by
**[generate_protocol_json.py](generate_protocol_json.py)**, which *introspects*
the live message classes in
[messages.py](../src/rhsp/internal/messages.py) so the structural data cannot
drift from the implementation. Regenerate with:

```
.venv/bin/python docs/generate_protocol_json.py
```

Top-level keys:

| Key | Contents |
|-----|----------|
| `link_layer` | Baud/format and port-discovery rule (§1). |
| `framing` | Frame bytes, header field table with offsets, checksum algorithm, payload offset (§2). |
| `constants` | `response_bit` (0x8000), `deka_interface_prefix` (0x1000), broadcast address, response-id rule (§4.3). |
| `enums` | `MotorMode`, `ZeroPowerBehavior`, `ClosedLoopMode`, `DIODirection`, `ADCChannel`, `LEDColor`, `I2CSpeedCode`. |
| `counts` | Channel counts (4 motors, 6 servos, 8 DIO, 4 ADC, 4 I2C). |
| `commands` | Every host→hub command: `id`/`id_hex`, `group`, ordered `payload` (each field with `name`, `bytes`, `offset`, and optional `signed`/`fixed_point`/`unit`/`enum`/`range`/`description`), and the expected `reply` (`ack` or a typed response). |
| `responses` | Every hub→host response packet with its ordered payload layout (including the bulk snapshots of §6). |
| `sequencing` | The ordered recipes of §9 as step arrays. |

Each command/response field carries a byte `offset` within the payload, so an
agent can build an encoder/decoder directly from the JSON without re-reading the
Python. Field structure (names, sizes, order, ids, reply mapping) is
authoritative-by-introspection; signedness, units, fixed-point scaling, and enum
references are a curated overlay (sourced from the device modules) and are the
parts most worth double-checking against `librhsp` or the FTC SDK (§11).

---

## 11. Cross-reference: REV's official C/Node implementation

The protocol above was extracted from the Python `rhsp` package. It was
cross-checked against REV Robotics' **official** implementation — the C library
`librhsp` wrapped as a Node.js native addon (local copy at
`/Users/eric/proj/RobotProjects/rhsplib-old`, upstream
[`@rev-robotics/rev-hub-core`](https://github.com/REVrobotics) /
`REVrobotics/RHSPlib`). In that copy the `librhsp` C submodule was not checked
out, so the exact framing/checksum C code could not be read directly, but the
addon's public API ([`lib/binding.ts`](file:///Users/eric/proj/RobotProjects/rhsplib-old/lib/binding.ts),
[`src/RevHubWrapper.cc`](file:///Users/eric/proj/RobotProjects/rhsplib-old/src/RevHubWrapper.cc))
maps one-to-one onto the commands documented here, confirming the command set
and semantics. Notable points of agreement and a few additions:

- **Same command surface.** `getBulkInputData`, `queryInterface`,
  module status (`statusWord`, `motorAlerts`, `attentionRequired`), LED
  color/pattern, the full motor / servo / DIO / ADC / I2C command groups, phone
  charge control, `injectDataLogHint`, and version reads all match §4.
- **QueryInterface confirmed.** The official API returns `firstPacketID` and
  `numberIDValues` — exactly the `packetID` / `numValues` of
  `QueryInterface_RSP` (§4.1). This confirms that the DEKA interface's base
  command id (`0x1000` here) is meant to be obtained **dynamically** at runtime
  via QueryInterface rather than hard-coded.
- **Discovery confirmed.** `discoverRevHubs()` is a **port-level** operation
  returning `parentAddress`, `childAddresses`, and `numberOfChildModules` —
  matching the broadcast-to-255 / multi-reply behavior in §3.3.
- **Generic command interface.** The official lib exposes
  `sendWriteCommand(packetTypeID, payload) → ACK` and
  `sendReadCommand(packetTypeID, payload) → response`, making explicit the
  setter→ACK / getter→typed-response split described in §3.1, with commands
  addressed by numeric `packetTypeID`.
- **Configurable response timeout.** `setResponseTimeoutMs` /
  `getResponseTimeoutMs` parameterize the receive timeout that appears as a
  fixed ~1 s value in the Python `sendAndReceive` (§3).
- **Serial parameters confirmed.** The `Serial.open(name, baudrate, databits,
  parity, stopbits, flowControl)` signature matches the 460800-8N1, no-flow
  configuration in §1.
- **Additions present in the C lib but not in the Python package:**
  - `readVersion()` returns a **structured** version (`hwType`, `majorVersion`,
    `minorVersion`, `engineeringRevision`, `majorHwRevision`, `minorHwRevision`)
    alongside the raw `readVersionString()`.
  - `setMotorClosedLoopControlCoefficients` with a `ClosedLoopControlAlgorithm`
    selector and **PIDF** coefficients (`PidfCoefficients`) — a superset of the
    older `SetMotorPIDCoefficients` (with its `mode` byte) documented in §4.2.
  - `setFTDIResetControl` / `getFTDIResetControl` — FTDI USB reset control, with
    no equivalent command in the Python package.

In short, the Python `rhsp` package implements an older/partial subset of the
same REV Hub Serial Protocol that the official `librhsp` speaks; nothing in the
official API contradicts the framing, addressing, or command encoding documented
above.

## 12. Review observations / discrepancies

These were noted while extracting the protocol. They are software issues in the
**host code**, not in the wire protocol, but they affect correctness and are
worth tracking:

1. **`msgNum` is not a persistent counter.** In
   `Client.sendAndReceive` ([client.py:84](../src/rhsp/client.py#L84)) `msgNum`
   is a local that resets to 0 on every call (incrementing only across retries).
   The instance field `self.msgNum` is never used for transmission. Every first
   transmission therefore carries `msgNum = 0`, so the response-correlation check
   in `checkResponse` is effectively trivial.

2. **`checkResponse` is unused on the receive path.** `sendAndReceive` returns
   the first checksum-valid decoded packet without calling `checkResponse`, so
   response-type / refNum validation is not actually enforced during normal
   operation.

3. **Block-read config functions write to the wrong object.**
   `i2cBlockReadConfig` and `imuBlockReadConfig`
   ([internal/i2c.py:126](../src/rhsp/internal/i2c.py#L126),
   [internal/i2c.py:144](../src/rhsp/internal/i2c.py#L144)) set attributes on the
   *message* object (`msg.channel = …`) instead of its `payload`
   (`msg.payload.channel = …`). The payload fields stay 0, so the configured
   register/address/interval are not transmitted.

4. **`setCurrentPIDCoefficients` is broken.**
   [internal/motors.py:158](../src/rhsp/internal/motors.py#L158) calls
   `getMotorPIDCoefficients(...)` (a getter) with setter arguments and the wrong
   arity. It should call `setMotorPIDCoefficients(... mode=3 ...)`.

5. **`I2CChannel.setSpeed` drops the client argument.**
   [i2c.py:125](../src/rhsp/i2c.py#L125) calls
   `i2cConfigureChannel(self.destinationModule, self.channel, speedCode)` without
   the leading `commObj`/client argument the function expects.

6. **`GetPWNPulseWidth` inconsistency.** The command name is misspelled (`PWN`),
   and its response payload declares `pulseWidth` as **1 byte**
   ([messages.py:1039](../src/rhsp/internal/messages.py#L1039)) while the setter
   `SetPWMPulseWidth` uses **2 bytes**. This is likely a defect carried over from
   the decompiled source.

7. **`I2CConfigureQuery_RSP` is not registered in `printDict`.** The response
   class and its `RespNum` id (`0x902F`) exist, and `I2CConfigureQuery` declares
   it as the expected reply, but there is no `printDict` entry for it. Because
   `Client.processPacket` looks the incoming command id up in `printDict`
   ([client.py:236](../src/rhsp/client.py#L236)), an actual
   `I2CConfigureQuery_RSP` from the hub would raise `KeyError`. It is the only
   `RespNum` value missing from `printDict`.

8. **Cosmetic:** several `*_RSP` entries in `printDict` use a key `'Response '`
   with a trailing space ([messages.py:2212](../src/rhsp/internal/messages.py#L2212)+);
   harmless because response packets have no further response. The
   `mototonicTime` field in `GetBulkInputData_RSP` is also a misspelling of
   "monotonic".

---

## 13. References

### This project
- **`League-Robotics/python-serial-hub-control`** — the repository this document
  lives in; the `rhsp` Python package under [src/rhsp/](../src/rhsp/) is the
  subject of this protocol extraction.
- **[rhsp-protocol.json](rhsp-protocol.json)** — machine-readable command
  catalogue (§10), generated by
  **[generate_protocol_json.py](generate_protocol_json.py)**.

### Upstream / sibling Python implementations (`unofficial-rev-port`)
- **SerialHubControl** — <https://github.com/unofficial-rev-port/SerialHubControl>
  — the most likely direct upstream of the `rhsp` package.
- **REVHubInterface** — <https://github.com/unofficial-rev-port/REVHubInterface>
  — a GUI tool sharing the same decompiled message layer (`REVmessages`); useful
  for confirming command/payload definitions.

### REV's official implementation (authoritative)
- **`REVrobotics/RHSPlib`** — <https://github.com/REVrobotics/RHSPlib> — REV's
  official C library (`librhsp`) implementing the REV Hub Serial Protocol. The
  canonical reference for framing, checksum, and command encoding.
- **`@rev-robotics/rev-hub-core`** — <https://github.com/REVrobotics> — the
  TypeScript/Node core types and the native addon that wraps `librhsp`. A local
  copy was reviewed at `/Users/eric/proj/RobotProjects/rhsplib-old` (see §11); its
  `lib/binding.ts` and `src/RevHubWrapper.cc` were used to cross-validate the
  command set, QueryInterface/Discovery semantics, and serial parameters.
- **`REVrobotics/REV-Hub-Serial-Protocol-Analyzer-For-Saleae`** —
  <https://github.com/REVrobotics/REV-Hub-Serial-Protocol-Analyzer-For-Saleae>
  — REV's official **Saleae Logic high-level analyzer** for this protocol. It
  decodes RHSP frames captured on the wire (UART), labelling each packet's
  command, header fields, and payload. This is the single most useful reference
  for *validating* a reimplementation: capture the traffic from a known-good
  client, run it through this analyzer, and compare the decoded frames against
  the bytes your implementation produces. It also serves as an independent,
  authoritative description of the on-wire framing and command numbering.

### Canonical protocol source (FTC SDK / "Lynx")
- **`OpenFTC/Extracted-RC`** — <https://github.com/OpenFTC/Extracted-RC> — the
  extracted FTC Robot Controller SDK. It contains the **original Java
  implementation** of this protocol in the `com.qualcomm.hardware.lynx` package
  ("Lynx" is REV's internal name for the hub). The `LynxModule`, `LynxCommand`,
  `LynxMessage`, and per-command `Lynx*Command` / `Lynx*Response` classes are the
  most complete and authoritative description of the wire protocol, packet
  framing, command numbering (including the dynamically-assigned interface base
  ids returned by QueryInterface), and the full DEKA command set. Recommended as
  the ground-truth reference when this document or the Python/C implementations
  are ambiguous.

### Hardware / sensor datasheets (for the I2C device layer, §7)
- **Bosch BNO055** — absolute-orientation IMU (I2C address 0x28); register map
  mirrored in [internal/imu.py](../src/rhsp/internal/imu.py).
- **Broadcom/Avago APDS-9960** — RGB/gesture/proximity sensor family used by the
  REV color sensor (I2C address 0x39); register map in
  [internal/i2c.py](../src/rhsp/internal/i2c.py).
- **STMicroelectronics VL53L0X** — time-of-flight distance sensor used by the
  REV 2 m distance sensor; driver in [distance.py](../src/rhsp/distance.py).
