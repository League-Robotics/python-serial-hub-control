---
status: pending
---

# RHSP — build a new idiomatic Python library

## Context

The current `rhsp` package (`src/rhsp/`) is a decompiled, non-idiomatic driver
for the REV Hub Serial Protocol; the full review is captured in
[.clasi/issues/rhsp-idiomatic-rewrite.md](../../../Volumes/Proj/proj/RobotProjects/SerialHubControl/.clasi/issues/rhsp-idiomatic-rewrite.md).
This plan is the **concrete, build-ready design** for the from-scratch library,
driven by the machine-readable spec
[docs/rhsp-protocol.json](../../../Volumes/Proj/proj/RobotProjects/SerialHubControl/docs/rhsp-protocol.json)
and the human spec
[docs/RHSP-Protocol.md](../../../Volumes/Proj/proj/RobotProjects/SerialHubControl/docs/RHSP-Protocol.md).

**Repo layout (set up before this sprint):** the original implementation has
been moved to **`vendor/rhsp/`** and kept as a runnable reference ("mostly
works"); `pyproject` temporarily installs `vendor/rhsp` so the repo stays green.
The new library is built fresh in **`src/rhsp/`**; the final phase flips
`pyproject` packaging from `vendor/rhsp` back to `src/rhsp`. Nothing is deleted —
the old code lives on in `vendor/` for reference and on-hardware comparison. Path
references to old files below are now under `vendor/rhsp/...`.

**Locked decisions:** runtime data-table codec seeded from `protocol.json` (no
224 classes); **breaking snake_case API**; fix protocol alignment (dynamic DEKA
base, `msg_num`≥1, 512-byte buffers, response validation, NACK→exception);
**stock-firmware command map canonical, legacy `GetBulk*`/`BlockRead*` ids ≥ 0x31
behind the generic `transaction()` escape hatch**; **synchronous, explicit
keep-alive, NO background threads**; **all sensors in scope**; remove the GUI;
Python ≥ 3.13.

**Key firmware-agnostic corollary:** no typed device method depends on a
divergent id ≥ 0x31. `Motor.get_velocity` reads `GetBulkInputData` (deka index
0x00, identical across firmwares — its response carries `motorNVelocity`), *not*
the legacy `GetBulkMotorData` (0x37). Verified against `messages.py`/§6.1.

---

## Target layout (`src/rhsp/`)

```
__init__.py     public: connect, Hub, Session, enumerate_hubs, errors, enums, bulk dataclasses
protocol.json   packaged copy of docs/rhsp-protocol.json (runtime data, shipped in wheel)
catalogue.py    loads JSON → Command/Response tables; legacy tagging; runtime_packet_id()
codec.py        Field/Command/Response dataclasses + generic encode/decode (struct/int.to_bytes)
framing.py      FRAME=b"DK", HEADER struct, checksum, build_frame(), FrameParser
transport.py    Transport Protocol; SerialTransport (pyserial, blocking+timeout); LoopbackTransport
session.py      Session.transaction() + msg_num≥1 + validation + retry + NackError + ~40 typed methods
discovery.py    enumerate_hubs() (D-prefix), connect(), drained discover(), QueryInterface base
hub.py          Hub (was Module): single address, init_peripherals(rollback), explicit keep_alive()
errors.py       RhspError/ProtocolError/ChecksumError/RhspTimeoutError/NackError/...
enums.py        IntEnum/IntFlag from JSON enums (incl. NackCode.describe())
devices/        motor.py servo.py dio.py adc.py i2c.py + bulk.py (BulkInputData, ModuleStatus)
sensors/        color.py distance.py imu.py registers.py
py.typed
```
**Old code:** not deleted — it lives in `vendor/rhsp/` (`__main__.py`,
`internal/`, `messages.py`, `client.py`, `rshp_serial.py`, the old device
modules, `printDict`, all `REV*` classes) as reference. `src/rhsp/` is built
all-new. The final phase only flips `pyproject` packaging from `vendor/rhsp` to
`src/rhsp`.

---

## Core design (real signatures)

**Framing** — `framing.py`
- `FRAME = b"DK"`; `HEADER = struct.Struct("<2sHBBBBH")` (10 B incl. frame:
  frame,length,dest,src,msg,ref,ptype); `MAX_PACKET = 512 + 10 + 1`.
- `checksum(buf) = sum(buf) & 0xFF`; `build_frame(dest,src,msg,ref,ptype,payload)->bytes`
  with `length = 10 + len(payload) + 1`.
- `class FrameParser: feed(data) -> Iterator[RawPacket]` (resync on `DK`, bound
  to 512, raise `ChecksumError` + resync on mismatch). `RawPacket(dest,src,msg_num,ref_num,packet_type,payload)`.
- Golden vector (msg≥1): `build_frame` KeepAlive dest=1 msg=1 → `444B0B0101000000047F1F`.

**Codec** — `codec.py` (no per-command classes)
- `Field(name,nbytes,offset,signed=False,fixed_point=None,unit=None,enum=None,range=None)`,
  `Command(name,id,group,index,fields,reply_kind,reply_id,reply_name,legacy,notes)`,
  `Response(name,id,fields)` — all frozen/slots.
- `encode_payload(fields,values)->bytes` / `decode_payload(fields,data)->dict`:
  `int.to_bytes(n,"little",signed=…)`; Q16 = `round(v*65536)` signed-4 on encode,
  `/65536.0` on decode; **last field is variable-length** (absorbs remaining
  bytes) for `versionString`/`payloadBytes`/`interfaceName`/`hintText`/`bytesToWrite`.
  decode applies signed+Q16, leaves enums as raw int (wrapped in the typed layer).
- JSON overlay (`signed/fixed_point/enum/range`) is applied **only** in
  `catalogue.py` when building `Field` rows; `codec.py` never reads JSON.

**Catalogue** — `catalogue.py`
- Loads packaged JSON via `importlib.resources`. `COMMANDS:dict[str,Command]`,
  `RESPONSES_BY_ID:dict[int,Response]`, `FRAMING/CONSTANTS/COUNTS/ENUMS`.
- `legacy = group=="deka" and index >= 0x31` (cross-checked vs
  `firmware_command_map_divergence`). `runtime_packet_id(cmd, deka_base) = deka_base+index`
  for deka else `cmd.id`. Resolves names → ids using the QueryInterface base.

**Transport** — `transport.py`: `Transport` Protocol (`write`/`read`/`reset_input`/`close`);
`SerialTransport(port, baud=460800, timeout=1.0, inter_byte_timeout=0.01)` (8N1,
blocking `read(512)` — no busy-wait); `LoopbackTransport` (in-memory, for tests).

**Session** — `session.py` (single-threaded, one-outstanding; documented)
- `Session(transport, retries=3, timeout=1.0)`, `_msg_num=1` persistent (1..255,
  never 0); `transaction(command_name, dest, **fields) -> dict|None`.
- Accept reply iff `packet_type == expected_response_id` **and**
  `ref_num == sent_msg_num` (Discovery exempt). NACK(`0x7F02`)→`raise NackError(code,name)`
  (NACK is a valid packet, not retried). timeout/checksum → retry → `RhspTimeoutError`.
- **~40 typed snake_case methods** (the device-API + system commands) wrap
  `transaction()` and apply enum/Q16: motor set/get (mode, enable, power,
  velocity, position, at_target, encoder, pid, current_alert, reset),
  servo/pwm, dio (single/all/direction), adc, i2c (write/read single/multiple,
  status_query, configure), system (keep_alive, fail_safe, query_interface,
  get_module_status, led color/pattern, read_version_string, discover,
  get_bulk_input_data). **No typed methods for legacy ≥0x31 commands** — only via
  `transaction("GetBulkMotorData", …)`.

**Discovery/Hub** — `discovery.py` + `hub.py`
- `enumerate_hubs()` (USB serial number starts with `D`); `connect(port=None)`
  opens transport → Session → `discover()` → `QueryInterface("DEKA")` (sets
  `session.deka_base`) → returns `Hub`.
- `discover()` broadcasts to `0xFF` and **drains** replies until a quiet window
  (replaces `time.sleep(2)`).
- `Hub(session, address, parent)` owns `motors[4]/servos[6]/dio[8]/adc[4]/i2c[4]`,
  all constructed `(session, channel/pin, address)` — **single address, no
  `module`+`destinationModule` duplication**. `init_peripherals()` runs the §9
  bring-up recipe with `fail_safe()` rollback on mid-bringup error.
  `keep_alive()` is explicit/caller-driven (docstring states the 2500 ms fail-safe).

**Bulk dataclasses** — `devices/bulk.py`: `BulkInputData.from_response(dict)`
with named fields (encoders signed-32, velocities signed-16 — reinterpreted in
the builder since the JSON RSP overlay omits the `signed` flag; also handles the
misspelled `mototonicTime`). `ModuleStatus` maps the two bytes to
`ModuleStatusBits`/`MotorStatusBits`.

**Devices** — thin typed wrappers calling `session.<method>`; `internal/` gone.
Fixes the P7 bugs in new code: `DIOPin.get_direction` **returns**; `I2CChannel.configure`
passes `session`; PID setter uses setter fields. `I2CDevice` implements the
read-then-`I2CReadStatusQuery` poll (honoring NACK 41 "poll again").

**Sensors** — `sensors/` on `I2CDevice`; register maps → `sensors/registers.py`
(APDS-9960 / color-V3 / **VL53L0X incl. the missing `OSC_CALIBRATE_VAL=0xF8`** /
BNO055). `ColorSensor`/`ColorSensorV3`, `Distance2m`, `IMU` port the existing
sequences verbatim as ordered `write_register`/`read_register` calls.

---

## Build sequence (CLASI tickets; mapped to the issue's phases)

Build new modules alongside the old package; delete old code only at the end.

| # | Ph | Work item | Verify |
|--|--|-----------|--------|
| 1 | 0 | Copy `protocol.json` into `rhsp/`; `errors.py`; `enums.py` | `test_enums` == JSON enums |
| 2 | 1 | `codec.py` (encode/decode incl. signed/Q16/variable-tail) | round-trips every JSON entry |
| 3 | 1 | `framing.py` (HEADER, checksum, build_frame, FrameParser) | golden vectors + ChecksumError |
| 4 | 1 | `catalogue.py` (JSON→tables, legacy tag, runtime_packet_id) | `command("GetBulkMotorData").legacy is True` |
| 5 | 2 | `transport.py` (Protocol, Serial, Loopback) | Loopback echo unit test |
| 6 | 2 | `tests/fakehub.py` (FakeHub over Loopback) | answers KeepAlive with ACK |
| 7 | 2 | `session.py` (msg_num≥1, validation, NACK, retry, transaction) | `test_session` passes |
| 8 | 2 | ~40 typed Session methods | byte-exact payload assertions |
| 9 | 3 | `discovery.py`+`connect()`+drained discover+QueryInterface base | FakeHub multi-reply → N hubs; deka_base set |
| 10 | 3 | `hub.py` (single address, init_peripherals+rollback, keep_alive) | init emits §9 recipe bytes |
| 11 | 4 | `devices/*` + `BulkInputData`; firmware-agnostic get_velocity/get_adc | get_velocity reads GetBulkInputData (0x1000) |
| 12 | 5 | `sensors/registers.py` + color/distance/imu | recorded register seq == §9 recipes |
| 13 | 6 | `__init__` public API; `examples/`; README fix; **delete `__main__.py`** | examples import; CI green |
| 14 | 7 | Flip `pyproject` packaging `vendor/rhsp` → `src/rhsp`; `uv sync` | `rhsp` imports from `src/rhsp`; suite green |
| 15 | 7 | Invert `docs/generate_protocol_json.py` to introspect `catalogue.py` | regenerated JSON == committed copy |

Items 1–4 consume `docs/rhsp-protocol.json` verbatim. Item 15 makes the JSON the
*source* and the generator a *validator*.

---

## Verification (hardware-free)

- **Golden vectors** (`tests/test_framing.py`): assert exact bytes for the §9.5
  examples (re-stamped for `msg≥1`), and `FrameParser` round-trip + `ChecksumError`.
- **Codec round-trip** (`test_codec.py`): `decode(encode(x))==x` for **every**
  command/response in `protocol.json` over signed min/max, Q16 extremes, and a
  512-byte trailing payload; assert field `offset`s are contiguous.
- **Session/FakeHub** (`test_session.py`): msg_num never 0; ref_num correlation;
  NACK→`NackError`; timeout→retry→`RhspTimeoutError`; multi-reply discovery;
  `QueryInterface` sets `deka_base`.
- **Devices/sensors** (`test_devices.py`, `test_sensors.py`): byte-exact payloads
  (e.g. `set_servo_pulse_width(0,1500)`→`00 DC 05`) and recorded I2C register
  sequences vs the §9 recipes. Proves all P7 bugs are absent.
- **`examples/`**: rewritten `test/` scripts, skipped when no `D…` hub is
  present. CI runs only `tests/` (no hardware). Optional on-wire confirmation via
  the REV Saleae analyzer for the one hardware-dependent path.

---

## Open implementation questions (carry into execution, non-blocking)

1. **`interfaceName` padding** for `QueryInterface` — send raw `b"DEKA"` (default)
   vs zero-pad to the 121-byte field; expose a per-field `pad` switch, confirm on
   hardware.
2. **`I2CReadStatusQuery` poll budget** (NACK 41 "poll again") — pick a default
   (e.g. 5 polls × 1 ms) without a thread; verify timing on a real sensor.
3. **Stock-firmware high-id commands** (PIDF, I2C_TRANSACTION, binary
   `READ_VERSION`, `SET_BULK_OUTPUT_DATA`) — carry as `firmware="stock"` catalogue
   tags now, but no typed methods until a hub is available to validate.
4. **Unsigned-but-physically-signed bulk fields** — `BulkInputData` reinterprets
   encoder/velocity as signed though the JSON RSP overlay omits the flag; flag for
   an upstream overlay correction so the inverted generator stays consistent.

---

## Critical files

- `docs/rhsp-protocol.json` → copied to `src/rhsp/protocol.json` (seeds codec/catalogue).
- `vendor/rhsp/internal/messages.py` — field layouts + §12 bugs being replaced (reference).
- `vendor/rhsp/client.py` — transport/session being redesigned (reference).
- `vendor/rhsp/distance.py` — VL53L0X register map → `src/rhsp/sensors/registers.py` (+ `OSC_CALIBRATE_VAL` fix).
- `docs/generate_protocol_json.py` — inverted in the final phase to introspect the new catalogue.
