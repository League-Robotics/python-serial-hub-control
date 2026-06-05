# rhsp — Project Overview

## What It Is

`rhsp` is a pure-Python library for driving a **REV Robotics Expansion Hub or
Control Hub** over its USB serial interface. It implements the hub's firmware
protocol — known internally as **RHSP** or the **DEKA interface** — giving a
host computer direct, low-level control over motors, servos, digital I/O,
analog inputs, I2C buses, and onboard/attached sensors.

The library is under a ground-up rewrite from a decompiled reference
implementation into a clean, spec-compliant, well-tested codebase. The
authoritative protocol spec is `docs/RHSP-Protocol.md`; the machine-readable
command catalogue is `docs/rhsp-protocol.json` (72 commands, 38 responses, full
field layouts with offsets, enums, and sequencing recipes).

## Who Uses It

**FTC robotics developers** (FIRST Tech Challenge) and experimenters who want to
drive a REV hub directly from a Linux/macOS/Windows host over USB — without the
Android-based FTC SDK. Typical users: teams building autonomous test rigs,
simulation harnesses, or custom control dashboards.

## Core Value

| Property | What it means |
|----------|---------------|
| **Spec-driven** | Codec generated at runtime from `protocol.json`; no hand-maintained 224-class explosion. One data-table update propagates everywhere. |
| **Protocol-correct** | Fixes known deviations: dynamic DEKA base via `QueryInterface`, `msg_num` ≥ 1, 512-byte buffers, full response validation, NACK → typed exception. |
| **Idiomatic Python** | `struct`/`dataclasses`, snake_case, typed exceptions instead of `exit()`, no busy-wait I/O, no decompilation artifacts. Python ≥ 3.13. |
| **Hardware-free testable** | `LoopbackTransport` + `FakeHub` enable full session, device, and sensor testing without a physical hub. |
| **All sensors in scope** | Color (APDS-9960), distance (VL53L0X), and IMU (BNO055) are first-class, layered cleanly on the I2C primitive. |

## High-Level Architecture

```
┌─────────────────────────────────────────┐
│  Public API  connect() / Hub / devices  │  hub.py · discovery.py · __init__.py
├─────────────────────────────────────────┤
│  Session  transaction() / typed methods │  session.py  (msg_num, validation, retry, NACK)
├─────────────────────────────────────────┤
│  Framing   build_frame / FrameParser    │  framing.py  (DK header, checksum, 512-byte buf)
├────────────────┬────────────────────────┤
│  Codec         │  Catalogue             │  codec.py  encode_payload / decode_payload
│  (struct)      │  (protocol.json)       │  catalogue.py  Command / Response tables
├────────────────┴────────────────────────┤
│  Transport  SerialTransport / Loopback  │  transport.py  (pyserial, blocking + timeout)
└─────────────────────────────────────────┘
         │ wraps
┌────────────────────────────────────────┐
│  Devices   Motor / Servo / DIO / ADC   │  devices/  (motor.py servo.py dio.py adc.py i2c.py)
│  Sensors   Color / Distance / IMU      │  sensors/  (color.py distance.py imu.py registers.py)
└────────────────────────────────────────┘
```

Data flows upward: the catalogue loads `protocol.json` at import time; the
codec uses those field descriptors to encode/decode raw bytes; framing wraps
bytes into RHSP packets; the transport moves bytes over the serial port; the
session handles sequencing, retry, and error promotion; device and sensor
classes sit at the top and speak only to the session.

## Brownfield Context

The original package (`vendor/rhsp/`) was decompiled from `REVmessages.pyc`
and is kept as a runnable reference. The new library is built fresh in
`src/rhsp/`. The final phase of the rewrite flips `pyproject.toml` packaging
from `vendor/rhsp` to `src/rhsp` and inverts the JSON generator so that
`protocol.json` is the source and the generator becomes a validator.

Key protocol references:

- `docs/RHSP-Protocol.md` — human-readable protocol spec (framing, command
  catalogue, NACK codes, sequencing recipes, worked wire examples).
- `docs/rhsp-protocol.json` — machine-readable command/response catalogue
  (seeds `catalogue.py` at runtime).
- `vendor/rhsp/` — original decompiled implementation (reference only).
