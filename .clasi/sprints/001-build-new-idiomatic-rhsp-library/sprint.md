---
id: '001'
title: Build new idiomatic rhsp library
status: planning-docs
branch: sprint/001-build-new-idiomatic-rhsp-library
use-cases:
  - UC-001
  - UC-002
  - UC-003
  - UC-004
  - UC-005
  - UC-006
  - UC-007
  - UC-008
  - UC-009
  - UC-010
  - UC-011
  - UC-012
  - UC-013
  - UC-014
  - UC-015
  - UC-016
  - UC-017
  - UC-018
  - UC-019
  - UC-022
  - UC-023
  - UC-024
  - UC-025
issues:
  - rhsp-build-a-new-idiomatic-python-library.md
  - rhsp-idiomatic-rewrite.md
---

# Sprint 001: Build new idiomatic rhsp library

## Goals

Replace the decompiled, non-idiomatic `vendor/rhsp/` driver with a ground-up,
spec-compliant, well-tested library in `src/rhsp/`. The new library is
data-table-driven (codec seeded from `protocol.json`), fully snake_case,
protocol-correct (dynamic DEKA base, msg_num ≥ 1, 512-byte buffers, validated
responses, NACK → typed exception), hardware-free testable via
`LoopbackTransport` + `FakeHub`, and covers all sensors (color, distance, IMU).

## Problem

The current `vendor/rhsp/` package was decompiled from a compiled artifact
and carries deep structural problems: hex-ASCII wire format through custom
metaclass metaprogramming, 224 near-identical hand-maintained message classes,
protocol violations (hard-coded DEKA base 0x1000, msg_num=0, 128-byte buffers,
no response validation, NACK swallowed), `exit()` error handling, a dead tkinter
GUI, camelCase throughout, and a set of concrete command-specific bugs (P7-a
through P7-g) that break DIO direction reads, I2C configuration, PID writes,
pulse-width responses, and the distance sensor OSC calibration constant.

## Solution

Build `src/rhsp/` from scratch in 15 steps following the ordered build sequence
in the build-ready issue. Foundation first (errors, enums, codec, framing,
catalogue), then transport and test infrastructure (LoopbackTransport, FakeHub,
Session), then discovery and Hub, then devices and sensors, then public API and
packaging flip. All steps are hardware-free verifiable. The vendor package stays
in `vendor/rhsp/` as a runnable reference throughout; the final ticket flips
`pyproject.toml` packaging from `vendor/rhsp` to `src/rhsp`.

## Success Criteria

- `uv run pytest tests/` passes with no hub connected.
- Golden frame vectors match §2.2 byte-for-byte (msg_num=1).
- Codec round-trips every command and response in `protocol.json`.
- `FakeHub` session tests: msg_num never 0, wraps 255→1, ref_num enforced,
  NACK → `NackError`, timeout → `RhspTimeoutError`.
- Device payload assertions: `set_servo_pulse_width(0,1500)` → `00 DC 05`,
  `set_motor_constant_power(0,16000)` → `00 80 3E`.
- All P7-series bugs verified absent by FakeHub byte assertions.
- `OSC_CALIBRATE_VAL = 0xF8` present in `sensors/registers.py`.
- `rhsp` imports from `src/rhsp/` after `pyproject` flip.
- Regenerated JSON matches committed `protocol.json` byte-for-byte.

## Scope

### In Scope

- All modules in `src/rhsp/`: errors, enums, codec, framing, catalogue,
  transport, session (~40 typed methods), discovery, hub, devices (motor, servo,
  dio, adc, i2c, bulk), sensors (color, distance, imu, registers), `__init__`
  public API, `py.typed`.
- `tests/` directory: `fakehub.py`, `test_framing.py`, `test_codec.py`,
  `test_session.py`, `test_devices.py`, `test_sensors.py`.
- `examples/` directory: rewritten `test/` scripts, skipped without a hub.
- `pyproject.toml` packaging flip from `vendor/rhsp` to `src/rhsp` + `uv sync`.
- Inversion of `docs/generate_protocol_json.py` to introspect `catalogue.py`.
- Deletion of `src/rhsp/__main__.py` (GUI removal).
- Copying `docs/rhsp-protocol.json` → `src/rhsp/protocol.json`.

### Out of Scope

- Typed session methods for high-id (≥ 0x31) stock-firmware commands (PIDF,
  I2C_TRANSACTION, binary READ_VERSION, SET_BULK_OUTPUT_DATA). These are tagged
  `legacy=True` in the catalogue and accessible only via `transaction()`.
- Async or multi-threaded transport.
- Deletion of `vendor/rhsp/` — it stays as a reference.
- Any on-hardware validation (hardware tests live in `examples/`, skipped in CI).

## Test Strategy

All sprint acceptance criteria are verified by `uv run pytest tests/` with no
hub present. Test layers:

1. `test_framing.py` — golden vector assertions + `FrameParser` round-trips +
   `ChecksumError` on bit-flipped checksum.
2. `test_codec.py` — `decode(encode(x)) == x` for every command and response in
   `protocol.json`; signed min/max, Q16 extremes, 512-byte variable-length fields;
   contiguous offset assertions.
3. `test_session.py` — `FakeHub`-backed: msg_num sequencing, ref_num correlation,
   NACK → `NackError`, timeout + retry → `RhspTimeoutError`, multi-reply
   Discovery, `QueryInterface` sets `session.deka_base`.
4. `test_devices.py` — byte-exact payload assertions for all device methods;
   P7 bug regression suite.
5. `test_sensors.py` — recorded I2C write/read sequence vs. §2.14 register recipes
   for APDS-9960, VL53L0X, BNO055.

Hardware smoke tests live in `examples/` and are skipped via `pytest.skip()`
when `enumerate_hubs()` returns an empty list.

## Architecture Notes

See `architecture-update.md` for the full module architecture. Key design
decisions:

- Runtime data-table codec: `Field`/`Command`/`Response` frozen dataclasses;
  `encode_payload`/`decode_payload` driven by field descriptors from `catalogue.py`.
- Catalogue loads `src/rhsp/protocol.json` via `importlib.resources`; no JSON
  reads in `codec.py`.
- `legacy = group == "deka" and index >= 0x31`; no typed method calls a legacy
  command.
- Typed device methods use only DEKA indices 0x00–0x30, all via `GetBulkInputData`
  (index 0x00) for bulk reads.
- `_msg_num` persistent counter, 1..255, never 0, wraps 255→1.
- Synchronous, single-outstanding-transaction, no background threads.
- `Hub` stores a single `address`; no `module`+`destinationModule` duplication.

## GitHub Issues

None linked externally at sprint creation.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Foundation — errors, enums, and packaged protocol.json | — |
| 002 | Codec — encode/decode with signed, Q16, and variable-length fields | 001 |
| 003 | Framing — build_frame, FrameParser, golden vectors | 001 |
| 004 | Catalogue — JSON-to-tables loader, legacy tagging, runtime_packet_id | 002, 003 |
| 005 | Transport — SerialTransport and LoopbackTransport | 001 |
| 006 | FakeHub test fixture | 004, 005 |
| 007 | Session — transaction engine, msg_num sequencing, NACK, retry | 004, 006 |
| 008 | Session typed methods (~40 snake_case wrappers) | 007 |
| 009 | Discovery — enumerate_hubs, connect, drained discover, QueryInterface | 008 |
| 010 | Hub — single address, init_peripherals with rollback, keep_alive | 009 |
| 011 | Devices — Motor, Servo, DIO, ADC, I2C, BulkInputData, P7 bug fixes | 010 |
| 012 | Sensors — registers.py, ColorSensor, Distance2m, IMU | 011 |
| 013 | Public API, examples, README, delete __main__.py | 012 |
| 014 | Flip pyproject packaging to src/rhsp and uv sync | 013 |
| 015 | Invert generate_protocol_json.py to introspect catalogue.py | 014 |

Tickets execute serially in the order listed.
