---
title: REV Hub Serial Protocol (RHSP)
blurb: Complete wire-protocol reference for the REV Robotics Expansion/Control Hub, with a machine-readable command catalogue.
order: 10
slug: rhsp-protocol
tags: [rhsp, rev-hub, protocol, serial]
---

> **Source repository:** [League-Robotics/python-serial-hub-control](https://github.com/League-Robotics/python-serial-hub-control) — pure-Python driver plus the machine-readable command catalogue **[`protocol.json`](https://github.com/League-Robotics/python-serial-hub-control/blob/main/src/rhsp/protocol.json)** (also mirrored at [`docs/rhsp-protocol.json`](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/rhsp-protocol.json)).

# REV Hub Serial Protocol (RHSP)

This document describes the wire protocol and software interface used by the
`rhsp` package (src/rhsp/) to communicate with a **REV Robotics
Expansion / Control Hub** over a USB serial connection. It was reconstructed by
reviewing the Python source, principally
src/rhsp/internal/messages.py (the message
definitions) and src/rhsp/client.py (the framing,
send/receive, and checksum logic).

It is intended to be **complete enough for an agent to reimplement the protocol
in another language/environment**. It ships with two companion artifacts in this
directory:

- **[rhsp-protocol.json](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/rhsp-protocol.json)** — a machine-readable catalogue of
  every command and response, with field layouts, byte offsets, enums, and
  sequencing recipes (§10).
- **[generate_protocol_json.py](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/generate_protocol_json.py)** — the generator that
  introspects the live message classes to (re)produce that JSON, so it can never
  drift from the implementation.

§9 covers the **ordering/sequencing** constraints (what must be set up before
what).

---

## Sources & references

The REV Hub Serial Protocol has no single published specification; the
authoritative description **is** the set of implementations below. They are
listed here, grouped by language, with the specific documents/files that define
the wire format. This document was written against the Python source and
cross-checked against the C and Java sources.

### Python implementations
- **`League-Robotics/python-serial-hub-control`** (this repo) — the `rhsp`
  package. Wire-format definitions: src/rhsp/internal/messages.py;
  framing / send-receive / checksum: src/rhsp/client.py.
- **[unofficial-rev-port/SerialHubControl](https://github.com/unofficial-rev-port/SerialHubControl)**
  — most likely direct upstream of the `rhsp` package.
- **[unofficial-rev-port/REVHubInterface](https://github.com/unofficial-rev-port/REVHubInterface)**
  — GUI tool sharing the same message layer. The definitions originate from a
  decompiled `REVmessages.pyc` (note the `# okay decompiling REVmessages.pyc`
  marker still present in `messages.py`).
- **This document's machine-readable companion:**
  [rhsp-protocol.json](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/rhsp-protocol.json) and its generator
  [generate_protocol_json.py](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/generate_protocol_json.py).

### C / C++ / Node implementation (REV official — `librhsp`)
- **[REVrobotics/node-rhsplib](https://github.com/REVrobotics/node-rhsplib)** —
  REV's official C library (`librhsp`) plus a Node.js binding. The C library is
  the **canonical reference for framing, checksum, and command encoding**, at
  `packages/rhsplib/librhsp/`. Key documents:
  - `src/packet.c` + `include/internal/packet.h` — packet framing, length, checksum.
  - `include/internal/RhspRxStates.h` — the receive (parser) state machine.
  - `src/command.c` + `include/internal/command.h` — request/response dispatch and retries.
  - `src/rhsp.c` + `include/rhsp/rhsp.h` — top-level API; `include/rhsp/errors.h` — NACK/error codes.
  - Per-subsystem: `motor.c`, `servo.c`, `dio.c`, `i2c.c`, `deviceControl.c`, `module.c`, `revhub.c` (with matching headers under `include/rhsp/`).
  - Transport: `src/arch/{linux,mac,win}/serial.c` + `include/rhsp/serial.h`.
  - Node/C++ binding: `packages/rhsplib/src/RevHubWrapper.cc` and `packages/rhsplib/lib/binding.ts` (the API surface cross-checked in §11; reviewed locally as `rhsplib-old`).

### Java implementation (FTC SDK / "Lynx" — ground truth)
- **[OpenFTC/Extracted-RC](https://github.com/OpenFTC/Extracted-RC)** — the
  extracted FTC Robot Controller SDK. The original, most complete implementation,
  under `Hardware/src/main/java/com/qualcomm/hardware/lynx/` ("Lynx" is REV's
  internal name for the hub). Key documents:
  - `commands/LynxDatagram.java` — on-wire packet framing, length, and checksum.
  - `commands/LynxMessage.java`, `LynxCommand.java`, `LynxResponse.java`, `LynxRespondable.java` — message base classes and command/response numbering.
  - `commands/LynxInterface.java`, `LynxInterfaceCommand.java`, `LynxInterfaceResponse.java` — the QueryInterface mechanism and dynamically-assigned interface command base ids.
  - `commands/standard/` — the **system commands** (e.g. `LynxDiscoveryCommand`, `LynxKeepAliveCommand`, `LynxQueryInterfaceCommand`, `LynxGetModuleStatusCommand`, `LynxSetModuleLEDColorCommand`, `LynxFailSafeCommand`, `LynxSetNewModuleAddressCommand`, `LynxAck`/`LynxNack`).
  - `commands/core/` — the **DEKA I/O commands** (base class `LynxDekaInterfaceCommand`; e.g. `LynxGetADCCommand`, `LynxGetBulkInputDataCommand`, and the motor/servo/DIO/I2C command classes). ~85 classes, one per command, each giving the exact payload layout.
  - `LynxModule.java`, `LynxUsbDevice.java` — transport, discovery, dispatch, keep-alive; `LynxNackException.java` — error handling.

### Reverse-engineered firmware & independent protocol write-up
- **[DuckTapeAndAPrayer/DuckLynx](https://github.com/DuckTapeAndAPrayer/DuckLynx)**
  — a clean-room **replacement firmware** (C) for the Lynx Hardware Interface
  Board, plus reverse-engineering notes. Two documents here are especially
  valuable because they describe the **hub (device) side** of the protocol:
  - `info/RHSP.md` — an independent prose specification of RHSP. It is the only
    source consulted that enumerates the **NACK codes** (§4.4), the **module/motor
    status bitfields** (§4.5), the inter-hub **RS485 topology** (§1), the
    **timeout semantics** (§3), and the firmware command map for the high command
    IDs (§4.6). Cross-references the FTC SDK, the Saleae analyzer, and librhsp.
  - `info/Stock Firmware.md` and `firmware/src/rhsp/` — notes on, and a partial
    re-implementation of, the stock firmware's command handling.

### Protocol analyzer & on-wire validation
- **[REVrobotics/REV-Hub-Serial-Protocol-Analyzer-For-Saleae](https://github.com/REVrobotics/REV-Hub-Serial-Protocol-Analyzer-For-Saleae)**
  — REV's official **Saleae Logic high-level analyzer** for this protocol
  (`HighLevelAnalyzer.py`). It decodes captured RHSP/UART frames (command, header
  fields, payload) and is the best tool for validating a reimplementation against
  real traffic from a known-good client.

### Hardware / sensor datasheets (for the I2C device layer, §7)
- **Bosch BNO055** — absolute-orientation IMU (I2C address 0x28); register map
  mirrored in internal/imu.py.
- **Broadcom/Avago APDS-9960** — RGB/proximity sensor family used by the REV
  color sensor (I2C address 0x39); register map in
  internal/i2c.py.
- **STMicroelectronics VL53L0X** — time-of-flight distance sensor used by the
  REV 2 m distance sensor; driver in distance.py.

> **Provenance.** The `rhsp` message layer was originally decompiled from
> `REVmessages.pyc` and reorganized into this package (this repo is
> `League-Robotics/python-serial-hub-control`). The protocol is REV's
> firmware-level **"DEKA"** interface — a name confirmed by both the C source
> (`deviceControl.c`) and the Java source (`LynxDekaInterfaceCommand`). Field
> names and semantics in this document come from the Python source, cross-checked
> against the C and Java references above.

---

## 1. Physical / link layer

| Parameter | Value | Source |
|-----------|-------|--------|
| Transport | USB CDC serial (virtual COM port) | client.py |
| Baud rate | **460800** | `Client._serial_config` |
| Data bits | 8 | `Client._serial_config` |
| Parity | None | `Client._serial_config` |
| Stop bits | 1 | `Client._serial_config` |
| Flow control | None | — |

Note: the `RHSPSerial` wrapper (rshp_serial.py)
defaults to 9600 baud, but `Client` always overrides it with the 460800
configuration above when it opens a port.

### Port discovery

`comPort.enumerate()` (rshp_serial.py:183)
scans system serial ports (`serial.tools.list_ports`) and selects a hub by USB
descriptor:

- The port's `hwid` must contain a `SER=` (serial-number) field, **and**
- that serial number must start with the letter `D`.

This `D…` serial-number prefix is how a genuine REV hub is distinguished from
other USB serial devices.

### Topology (per DuckLynx `RHSP.md`)

The link this package speaks to — controller ↔ **parent** hub — is the USB serial
port above. Beyond that, hubs form a small network:

- The **controller** (this library, or the FTC Robot Controller app) connects to
  the **parent** hub over `UART0` (USB).
- **Child** hubs connect to the parent over **RS485** on `UART1`, daisy-chained
  and addressed individually.
- The protocol is the same on both legs; the parent hub forwards/repeats packets
  between USB and RS485. Discovery (§3.3) is what walks the RS485 chain.
- The whole protocol is little-endian to match the ARM MCU in the hub.

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
`REVPacket` index constants):

| Offset | Size | Field | Description |
|-------:|:----:|-------|-------------|
| 0 | 2 | **Frame bytes** | Constant `0x44 0x4B` (ASCII `"DK"`). Marks the start of a packet. |
| 2 | 2 | **Length** | Total packet length in bytes, little-endian (includes frame, header, payload, and checksum). |
| 4 | 1 | **Destination** | Target module address. `0xFF` (255) = broadcast to all hubs (used for Discovery). |
| 5 | 1 | **Source** | Originating address. **Always `0x00` for the controller**; on a reply it is the responding hub's address. |
| 6 | 1 | **Message number** | Sender's sequence id for this request. Per spec it **must never be 0** — it starts at 1 and, on overflow, wraps back to 1. (The Python client does not honor this; see §12.) |
| 7 | 1 | **Reference number** | On a response, echoes the request's message number (correlates response↔request). |
| 8 | 2 | **Packet type / command** | Command id, little-endian. Bit 15 = response flag (§4.3). See §4. |
| 10 | N | **Payload** | Command-specific fields. May be empty. |
| 10+N | 1 | **Checksum** | 8-bit additive checksum (see §2.3). |

The fixed header is 8 bytes; with the 2 frame bytes and 1 checksum byte, the
minimum packet (empty payload) is 11 bytes.

### 2.2 Length field

`REVPacket.calcLength()`
(messages.py:309) computes:

```
length = len(header) + len(payload) + 3
       = 8           + N            + 3   (2 frame bytes + 1 checksum byte)
       = total packet size in bytes
```

The value is then byte-swapped to little-endian for transmission. On receive,
the parser rejects any frame whose declared payload length exceeds
`PAYLOAD_MAX_SIZE = 128`.

> **Max payload discrepancy.** This Python package caps the payload at **128**
> bytes (`PAYLOAD_MAX_SIZE`). REV's `librhsp` allows up to **512** bytes
> (`RHSP_PACKET_PAYLOAD_BUFFER_SIZE`, per DuckLynx `RHSP.md`). A reimplementation
> that needs the large I2C/version payloads should size its receive buffer for
> 512, not 128.

### 2.3 Checksum

8-bit additive checksum over **every byte from the frame start through the end
of the payload** (i.e. all bytes except the checksum byte itself), taken
mod 256:

```
chksum = (sum of all preceding bytes) % 256
```

Computed in `REVPacket.getPacketData()`
(messages.py:313) and verified on
receive in `Client.checkPacket()`
(client.py:218). Per the firmware spec, **if the
checksum is wrong the hub sends no reply at all** (the packet is dropped, not
NACK'd), and a bad-checksum packet does **not** reset the keep-alive timeout
(§3).

### 2.4 Byte order

Multi-byte integer fields are **little-endian** on the wire. The code performs
this conversion explicitly:

- For the 2-byte length and command fields via the idiom
  `x >> 8 | x % 256 << 8`.
- For general payload fields via `Client.swapEndianess()`
  (client.py:253), which reverses the byte order
  of a hex string.

Signed fields use two's-complement (see §5).

---

## 3. Transaction model

The protocol is strictly **request/response, half-duplex, one outstanding
transaction at a time**. `Client.sendAndReceive(packet, destination)`
(client.py:65) drives every exchange:

1. Set `header.destination = destination` and assign a message number.
2. Write the encoded packet to the serial port.
3. Wait (up to ~1 s) for bytes to arrive; on timeout, retry up to
   `MaxRetries = 3` times.
4. Feed incoming bytes through a receive state machine (§3.2).
5. On a complete, checksum-valid packet, decode it (`processPacket`) and return
   the response object. On checksum failure, log and resync.

### 3.0 Keep-alive timeout / fail-safe (hub side)

The hub runs a watchdog. Per DuckLynx `RHSP.md`:

- If a hub receives **no packet for 2500 ms**, it enters **timeout mode**: it
  signals the timeout on the LED and stops all motion by entering **fail-safe**
  (motors/servos disabled).
- **Any valid packet resets the timeout — even one that gets NACK'd.** A packet
  is "valid" if it has the magic frame bytes and a correct checksum, regardless
  of whether the command or payload is accepted.
- Packets with a **bad checksum or missing magic number do *not* reset the
  timeout** (and draw no reply).

Therefore a controller must send *something* (typically `KeepAlive`, §4.1) at
least every ~2.5 s while any output is active. After a fail-safe, outputs must be
re-enabled. The `FailSafe` command lets the controller trigger this state
deliberately (e-stop).

### 3.1 Responses: ACK vs. typed response

Every command has an expected reply, captured in the `printDict` table
(messages.py:2003):

- **Setters / actions** (e.g. `SetMotorConstantPower`, `KeepAlive`,
  `ResetMotorEncoder`) reply with a bare **`ACK`** (payload: `attnReq`, 1 byte).
- **Getters / queries** (e.g. `GetADC`, `GetMotorEncoderPosition`) reply with a
  dedicated **`*_RSP`** packet whose command id is the request id OR-ed with the
  response bit (§4.3).
- On error, the hub replies with **`NACK`** (payload: `nackCode`, 1 byte).

`Client.checkResponse()` (client.py:195) validates
that the received `packetType` equals the expected response and that the
response's `refNum` matches the request's `msgNum` (Discovery is exempted, since
it is a broadcast with multiple replies).

### 3.2 Receive state machine

The receiver parses byte-by-byte
(client.py:134) through these states:

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
(client.py:226).

### 3.3 Discovery (broadcast)

`Client.discovery()` (client.py:317) sends a
`Discovery` command to destination **255** (broadcast). Unlike normal
transactions, the hub (and any daisy-chained modules) may emit **multiple**
`Discovery_RSP` packets. The client collects responses, pausing ~2 s and
re-checking the input buffer, until no more arrive. Each `Discovery_RSP` carries
a `parent` byte and its `source` becomes that module's address. One `Module`
object is created per discovered module.

How the hub side responds (DuckLynx `RHSP.md`): the **parent** hub first replies
with its own address (changing the packet's `source` from the broadcast `0xFF`
to its own). It then sends a discovery to **every possible child address (0–254)**
over RS485 and forwards each child's reply back to the controller. The
`Discovery_RSP` `parent` field (a bool) indicates whether that reply originated
at the parent itself or was retransmitted from an RS485 child — this is how the
controller learns the parent/child layout.

### 3.4 Addressing & message numbering

- **Destination** is supplied per call; it is the module address (the address a
  `Module` was discovered at, reassignable via `SetNewModuleAddress`).
- **Message number** is managed locally inside `sendAndReceive` (it starts at 0
  for each call and increments only across retries; it is **not** a persistent
  monotonic counter across transactions — see §7).

---

## 4. Command catalogue

Command ids fall into two ranges, both defined in `MsgNum`
(messages.py:1217).

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
command id of a named interface. The `interfaceName` is a **null-terminated
string** (e.g. `"DEKA"`); the response gives the first command id and the number
of commands in that interface.

**LED pattern step layout.** Each of the 16 `rgbtStep` entries is 4 bytes. On the
wire (little-endian) the byte order is `[tenths-of-seconds, blue, green, red]`
(DuckLynx `RHSP.md`). A step of all zeros terminates the pattern early. This
matches the Python `LEDPattern.set_step`, which packs `r<<24 | g<<16 | b<<8 | t`
(serialized little-endian → `t, b, g, r`).

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

> **⚠ Command numbering above index 0x30 diverges from current REV firmware.**
> Indices `0x00`–`0x30` (0–48, through `ReadVersionString`) agree across this
> Python package, `librhsp`, and the FTC SDK. From index **0x31 (49) onward the
> Python package's map does not match REV's stock firmware / `librhsp`** — see
> §4.6 for the side-by-side table. If you are reimplementing against a current
> hub, use the firmware numbering in §4.6, not the Python rows above. (The Python
> rows are accurate to *this package*, which appears to target an older or
> non-stock command set for the high IDs.)

### 4.3 Response command ids

`RESPONSE_BIT = 0x8000` (messages.py:1295).
A typed response's command id is `RESPONSE_BIT | requestCmd`. For example
`GetADC` = `0x1007` → `GetADC_RSP` = `0x9007`. `RespNum`
(messages.py:1297) enumerates all of
these. (ACK/NACK replies, by contrast, use their own fixed ids `0x7F01`/`0x7F02`
rather than a response-bit-encoded id.)

### 4.4 NACK codes

When a command is rejected, the hub replies with `NACK` (`0x7F02`) carrying a
single `nackCode` byte. The Python package only prints this code; the meanings
below are from DuckLynx `RHSP.md` (stock-firmware semantics):

| Code | Meaning |
|-----:|---------|
| 0–9 | Parameter #N out of range (the index is the offending parameter). |
| 10–17 | GPIO #(code−10) not configured for output. |
| 18 | No GPIO pins configured for output. |
| 20–27 | GPIO #(code−20) not configured for input. |
| 28 | No GPIO pins configured for input. |
| 30 | Servo not fully configured before being enabled. |
| 31 | Battery voltage too low to run servo. |
| 40 | I2C master busy (command rejected). |
| 41 | I2C operation in progress (poll again for completion). |
| 42 | I2C no results pending. |
| 43 | I2C query mismatch (query doesn't match last operation). |
| 44 | I2C timeout — SDA stuck. |
| 45 | I2C timeout — SCK stuck. |
| 46 | I2C timeout. |
| 50 | Motor not fully configured for the selected mode before being enabled. |
| 51 | Command not valid for the selected motor mode. |
| 52 | Battery voltage too low to run motor. |
| 253 | Command implementation pending (known/delivered but not implemented). |
| 254 | Command routing error (known but not handled by the receiving subsystem). |
| 255 | Packet Type ID unknown (should not happen if discovery/QueryInterface was done). |

Codes 19, 29, 32–39, 47–49, 53–59 are reserved.

### 4.5 Status bitfields (`GetModuleStatus` response)

`GetModuleStatus` (`0x7F03`) returns two bytes. The Python layer exposes them as
opaque `statusWord` / `motorAlerts`; the bit meanings (DuckLynx `RHSP.md`) are:

**Byte 0 — Module status**

| Bit | Meaning |
|----:|---------|
| 0 | Keep-alive timeout |
| 1 | Device reset (set once after the device comes up from reset) |
| 2 | Fail-safe (battery too low to run, or a keep-alive timeout) |
| 3 | Controller over-temperature |
| 4 | Battery low (set below ~7 V; also triggers fail-safe + LED) |
| 5 | HIB fault |
| 6–7 | Reserved |

**Byte 1 — Motor status (alerts)**

| Bit | Meaning |
|----:|---------|
| 0–3 | Motor 0–3 lost encoder counts |
| 4–7 | Motor 0–3 driver overheat |

### 4.6 Firmware command map for IDs ≥ 0x31 (divergence)

DEKA function numbers `0x00`–`0x30` agree across implementations. From `0x31`
onward, **REV's stock firmware / `librhsp`** (left, per DuckLynx `RHSP.md`,
confirmed against `librhsp` `deviceControl.c` / `motor.c`) and **this Python
package** (right, from `messages.py`) assign *different* commands to the same
ids:

| Idx | id | Stock firmware / librhsp | Python `rhsp` package |
|----:|----|--------------------------|------------------------|
| 0x31 | 0x1031 | `FTDI_RESET_CONTROL` | `GetBulkPIDData` |
| 0x32 | 0x1032 | `FTDI_RESET_QUERY` | `I2CBlockReadConfig` |
| 0x33 | 0x1033 | `SET_MOTOR_PIDF_COEFFICIENTS` | `I2CBlockReadQuery` |
| 0x34 | 0x1034 | `I2C_WRITE_READ_MULTIPLE_BYTES` | `I2CWriteReadMultipleBytes` *(agree)* |
| 0x35 | 0x1035 | `GET_MOTOR_PIDF_COEFFICIENTS` | `IMUBlockReadConfig` |
| 0x36 | 0x1036 | `I2C_TRANSACTION` | `IMUBlockReadQuery` |
| 0x37 | 0x1037 | `I2C_QUERY_TRANSACTION` | `GetBulkMotorData` |
| 0x38 | 0x1038 | `SET_BULK_OUTPUT_DATA` | `GetBulkADCData` |
| 0x39 | 0x1039 | `READ_VERSION` (binary firmware version) | `GetBulkI2CData` |

Notes:
- The firmware adds **PIDF** closed-loop coefficients (`0x33`/`0x35`) as a
  superset of the older PID command (`0x17`/`0x18`), plus **FTDI reset control**
  (`0x31`/`0x32`), a generic **I2C transaction** API (`0x36`/`0x37`), a
  **`SET_BULK_OUTPUT_DATA`** write-everything command (`0x38`), and a binary
  **`READ_VERSION`** (`0x39`).
- `librhsp` resolves these at runtime via
  `rhsp_getInterfacePacketID(hub, "DEKA", functionNumber, …)` — i.e. it adds the
  function number to the base id returned by QueryInterface, rather than
  hard-coding `0x1000 + n`. **A portable reimplementation should do the same.**
- The Python package's `GetBulk*Data` / `*BlockRead*` commands at these ids are
  not present in current stock firmware; treat them as belonging to a different
  firmware generation and **verify against your target hub** (capture with the
  Saleae analyzer) before relying on them.

---

## 5. Data encoding conventions

| Concept | Encoding |
|---------|----------|
| Integers | Little-endian on the wire. |
| Signed integers | Two's-complement. Encoder position is **32-bit signed**; target velocity and power are **16-bit signed**. Sign extension is applied on decode (e.g. internal/motors.py:124). |
| PID coefficients | **Q16 fixed-point**: transmitted as `value × 65536` in a 4-byte field; decoded by dividing by 65536 (`Q16` in internal/motors.py:6). |
| Strings | ASCII, packed two hex chars per byte; `ReadVersionString` returns `length` + up to 40 bytes (not null-terminated), decoded char-by-char in module.py:136. The version string format is `"HW: 20, Maj: 1, Min: 8, Eng: 2"` — hardware revision (`20` = 2.0), then semantic major/minor/patch (DuckLynx `RHSP.md`). |

### 5.1 Motor modes (`motorMode`)

From internal/motors.py:

| Value | Mode |
|------:|------|
| 0 | `MODE_CONSTANT_POWER` |
| 1 | `MODE_CONSTANT_VELOCITY` |
| 2 | `MODE_POSITION_TARGET` |
| 3 | `MODE_CONSTANT_CURRENT` |

`floatAtZero`: `0 = BRAKE_AT_ZERO`, `1 = FLOAT_AT_ZERO`. Power level range
observed in tests is `±(2^15 − 1)`.

### 5.2 Servo encoding

From servo.py: pulse widths are in **microseconds**,
clamped to **500–2500 µs**; the default frame period is **20000 µs** (50 Hz).
`setAngle(0..180°)` maps linearly to `500 + angle × (2000/180)` µs.

### 5.3 ADC channels (`adcChannel`)

From adc.py / `ADCChannel` in
messages.py:9:

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
classes in messages.py.)

### 6.1 GetBulkInputData_RSP (messages.py:895)
A full I/O snapshot: `digitalInputs`(1); `motor0..3Encoder`(4 each);
`motorStatus`(1); `motor0..3Velocity`(2 each); `motor0..3mode`(1 each);
`analogInput0..3`(2 each); `gpioCurrent_mA`,`i2cCurrent_mA`,`servoCurrent_mA`,
`batteryCurrent_mA`(2 each); `motor0..3current_mA`(2 each); `mon5v_mV`(2);
`batteryVoltage_mV`(2); `servo0..5cmd`(2 each); `servo0..5framePeriod_us`(2 each);
`i2c0..3data`(10 each); `imuBlock`(10); `i2c0..3Status`(1 each); `imuStatus`(1);
`mototonicTime`(4) *(monotonic timestamp; field name is misspelled in the source)*.

### 6.2 GetBulkPIDData_RSP (messages.py:1103)
Per-motor PID telemetry: for each of the **current**, **velocity**, and
**position** loops — `Pterm`, `Iterm`, `Dterm`, `Output`, `Cmd`, `Error`
(4 bytes each) — plus `monotonicTime`(4).

### 6.3 GetBulkMotorData_RSP (messages.py:1144)
`motor0..3Encoder`(4 each); `motorStatus`(1); `motor0..3Velocity`(2 each);
`motor0..3mode`(1 each); `monotonicTime`(4). Used by `Motor.getVelocity()`.

### 6.4 GetBulkADCData_RSP (messages.py:1163)
`analogInput0..3`(2 each); the four bus currents (2 each); `motor0..3current_mA`
(2 each); `mon5v_mV`(2); `batteryVoltage_mV`(2); `monotonicTime`(4).

### 6.5 GetBulkI2CData_RSP (messages.py:1183)
`i2c0..3data`(10 each); `imuBlock`(10); `i2c0..3Status`(1 each); `imuStatus`(1);
`monotonicTime`(4). Returns the most recent **block-read** results cached by the
hub (see §6.7).

### 6.6 GetBulkServoData_RSP (messages.py:1199)
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
bytes (internal/i2c.py:101).

I2C command/address bit helpers (`I2CConstants`,
internal/i2c.py:9):
`COMMAND_REGISTER_BIT = 0x80`, `MULTI_BYTE_BIT = 0x20`,
`SINGLE_BYTE_BIT = 0x00`.

The higher-level device layer (i2c.py,
color.py, distance.py,
imu.py) implements specific sensors on top of these
primitives:

| Sensor | I2C address | Notes |
|--------|------------:|-------|
| REV color sensor (APDS-9960 family) | 57 (0x39), device id 0x60 | Register map in `I2CConstants` (internal/i2c.py). |
| BNO055 IMU | 40 (0x28) | Full register map in `IMUConstants` (internal/imu.py). |
| 2 m distance sensor (VL53L0X) | configured in `Distance2m` | distance.py. |

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

`Module.init_periphs()` (module.py:24) constructs 4
motors, 4 I2C channels, 8 DIO pins, 6 servos, and 4 ADC pins, and initializes
motors to constant-power mode at zero. `Module.keep_alive()` must be sent
periodically or the hub's fail-safe disables outputs.

---

## 9. Sequencing & usage recipes

The protocol is mostly stateless per command, but the **hub** is stateful: some
commands must precede others, and a heartbeat must run continuously while
outputs are active. The recipes below are distilled from `Module.init_periphs()`
and the example scripts in test/. They are also encoded
machine-readably under `"sequencing"` in
[rhsp-protocol.json](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/rhsp-protocol.json) (§10).

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
structured file: **[rhsp-protocol.json](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/rhsp-protocol.json)**. It is generated by
**[generate_protocol_json.py](https://github.com/League-Robotics/python-serial-hub-control/blob/main/docs/generate_protocol_json.py)**, which *introspects*
the live message classes in
messages.py so the structural data cannot
drift from the implementation. Regenerate with:

```
.venv/bin/python docs/generate_protocol_json.py
```

Top-level keys:

| Key | Contents |
|-----|----------|
| `link_layer` | Baud/format, port-discovery rule, and RS485 `topology` (§1). |
| `framing` | Frame bytes, header field table with offsets, checksum algorithm + `on_bad_checksum`, `max_payload_size` (+ librhsp note), payload offset (§2). |
| `constants` | `response_bit` (0x8000), `deka_interface_prefix` (0x1000), broadcast address, response-id rule (§4.3). |
| `enums` | `MotorMode`, `ZeroPowerBehavior`, `ClosedLoopMode`, `DIODirection`, `ADCChannel`, `LEDColor`, `I2CSpeedCode`, `NackCode` (§4.4), `ModuleStatusBits` / `MotorStatusBits` (§4.5). |
| `counts` | Channel counts (4 motors, 6 servos, 8 DIO, 4 ADC, 4 I2C). |
| `keep_alive_interval_ms` / `keep_alive_note` | 2500 ms watchdog and reset semantics (§3.0). |
| `firmware_command_map_divergence` | Stock-firmware vs Python id map for indices ≥ 0x31 (§4.6). |
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
addon's public API (`lib/binding.ts`,
`src/RevHubWrapper.cc`)
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

These were noted while extracting the protocol. Most are software issues in the
**host code**; item 0 is a wire-protocol divergence. All affect correctness and
are worth tracking:

0. **Command numbering ≥ 0x31 diverges from current REV firmware (§4.6).** This
   is the most important caveat for a reimplementation: the Python package's
   `GetBulkPIDData` / `*BlockRead*` / `GetBulk*Data` commands (ids `0x1031`+) do
   **not** match REV's stock firmware / `librhsp`, which place FTDI reset, PIDF
   coefficients, I2C transaction, `SET_BULK_OUTPUT_DATA`, and binary
   `READ_VERSION` at those ids. Resolve DEKA ids dynamically via QueryInterface
   and validate the high-id commands against your target hub.

1. **`msgNum` is not a persistent counter — and violates the spec.** In
   `Client.sendAndReceive` (client.py:84) `msgNum`
   is a local that resets to 0 on every call (incrementing only across retries).
   The instance field `self.msgNum` is never used for transmission. Every first
   transmission therefore carries `msgNum = 0`, which the protocol spec forbids
   (message number must be ≥ 1; §2.1), and the response-correlation check in
   `checkResponse` is effectively trivial.

2. **`checkResponse` is unused on the receive path.** `sendAndReceive` returns
   the first checksum-valid decoded packet without calling `checkResponse`, so
   response-type / refNum validation is not actually enforced during normal
   operation.

3. **Block-read config functions write to the wrong object.**
   `i2cBlockReadConfig` and `imuBlockReadConfig`
   (internal/i2c.py:126,
   internal/i2c.py:144) set attributes on the
   *message* object (`msg.channel = …`) instead of its `payload`
   (`msg.payload.channel = …`). The payload fields stay 0, so the configured
   register/address/interval are not transmitted.

4. **`setCurrentPIDCoefficients` is broken.**
   internal/motors.py:158 calls
   `getMotorPIDCoefficients(...)` (a getter) with setter arguments and the wrong
   arity. It should call `setMotorPIDCoefficients(... mode=3 ...)`.

5. **`I2CChannel.setSpeed` drops the client argument.**
   i2c.py:125 calls
   `i2cConfigureChannel(self.destinationModule, self.channel, speedCode)` without
   the leading `commObj`/client argument the function expects.

6. **`GetPWNPulseWidth` inconsistency.** The command name is misspelled (`PWN`),
   and its response payload declares `pulseWidth` as **1 byte**
   (messages.py:1039) while the setter
   `SetPWMPulseWidth` uses **2 bytes**. This is likely a defect carried over from
   the decompiled source.

7. **`I2CConfigureQuery_RSP` is not registered in `printDict`.** The response
   class and its `RespNum` id (`0x902F`) exist, and `I2CConfigureQuery` declares
   it as the expected reply, but there is no `printDict` entry for it. Because
   `Client.processPacket` looks the incoming command id up in `printDict`
   (client.py:236), an actual
   `I2CConfigureQuery_RSP` from the hub would raise `KeyError`. It is the only
   `RespNum` value missing from `printDict`.

8. **Cosmetic:** several `*_RSP` entries in `printDict` use a key `'Response '`
   with a trailing space (messages.py:2212+);
   harmless because response packets have no further response. The
   `mototonicTime` field in `GetBulkInputData_RSP` is also a misspelling of
   "monotonic".
