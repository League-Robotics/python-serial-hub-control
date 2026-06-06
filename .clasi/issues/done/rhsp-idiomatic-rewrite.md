---
status: done
tickets:
- 001-001
- 001-002
- 001-003
- 001-004
- 001-005
- 001-006
- 001-007
- 001-008
---

# RHSP module: idiomatic-Python rewrite & protocol-alignment fixes

## Context

The `rhsp` package (`src/rhsp/`) is a pure-Python driver for the REV Hub Serial
Protocol (RHSP / "DEKA" interface). It was reverse-engineered/decompiled from
another language and is non-idiomatic: the wire layer manipulates **hex-ASCII
strings** through custom dunder metaprogramming, error handling calls `exit()`,
naming is camelCase, and a dead tkinter GUI ships inside the library. The
protocol itself is now fully documented in this repo
([docs/RHSP-Protocol.md](../../docs/RHSP-Protocol.md)) and, crucially, as a
machine-readable catalogue ([docs/rhsp-protocol.json](../../docs/rhsp-protocol.json):
72 commands + 38 responses with field layouts, offsets, enums, sequencing).

This issue is **(1) a detailed code review** assessed along three independent
axes — architecture, protocol alignment, idiomatic Python — and **(2) a
remediation plan** for a ground-up rewrite.

**Decisions already taken (by the stakeholder):**
- **Full idiomatic rewrite** of the wire layer (struct/dataclasses, not hex strings).
- **Breaking API redesign allowed** — snake_case only; the `test/` scripts get updated.
- **Fix protocol alignment** too (not just code quality).
- **Remove the GUI** (`__main__.py`) from the package.

---

## Part 1 — Code review

> Three separate assessments, per the review brief. File refs are
> `src/rhsp/...`. Severity: **H** = correctness/architecture-critical, **M** =
> quality, **L** = minor.

### 1A. Architecture — *is it well-architected for what it does?*

**Verdict: weak.** The structure mirrors the source language, not Python; its
layers do not earn their keep.

| # | Finding | Sev |
|---|---------|-----|
| A1 | **Pointless two-layer split.** `internal/{motors,servo,dio,i2c}.py` are free-function passthroughs; the top-level `{motors,servo,dio,adc,i2c}.py` device classes wrap them 1:1. The "internal" layer is the same abstraction level minus `self` — no encapsulation benefit. | H |
| A2 | **224-class explosion + hand-maintained dispatch.** ~110 `*_Payload` + ~110 `*_Packet` near-identical classes, plus a 220-entry `printDict` id→{Name,Packet,Response} table maintained by hand (`src/rhsp/internal/messages.py`). | H |
| A3 | **Redundant identity on every device.** `Motor` stores both `module` and `destinationModule`; the code comment admits the redundancy. Mirrored in Servo/DIO/ADC/I2C. This duplication directly causes the arg-order bugs in 1B. | H |
| A4 | **Monolithic transport.** `Client.sendAndReceive` (`src/rhsp/client.py`) is one ~130-line method doing framing + retry + parse + discovery special-casing, with state-machine constants re-defined on every call. | H |
| A5 | **Vestigial concurrency + no thread safety.** `mp.Queue` (`rxQueue`/`txQueue`), `serialReceive_Thread`, `queue.Queue`, and `tkinter` are imported/created but never used; serial access is unguarded (no lock), so concurrent callers would corrupt the parser. | H |
| A6 | **No lifecycle / partial-init risk.** `Module.init_periphs()` constructs and configures all peripherals with no error handling/rollback; `killSwitch()` has empty DIO/ADC loops. | M |
| A7 | **Sensor drivers conflated with protocol.** `color.py`/`distance.py`/`imu.py` (device register maps for APDS-9960/VL53L0X/BNO055) sit alongside the core protocol primitives with no layering boundary. | M |
| A8 | **Dead GUI in the library.** `__main__.py` (~1427 lines) imports a non-existent `REVHubInterface.REVcomm` and cannot run; it's ~95% tkinter boilerplate. | H |

### 1B. Protocol alignment — *how well does it match the actual protocol?* (separate)

**Verdict: correct on the common path, but it hard-codes assumptions, skips
validation, and mis-handles the high-id command range and several specific
commands.** (Cross-referenced to the spec sections in docs/RHSP-Protocol.md.)

| # | Finding | Sev |
|---|---------|-----|
| P1 | **DEKA base id hard-coded to `0x1000`.** Spec/`librhsp` resolve it dynamically via `QueryInterface("DEKA")` (`base + index`). §4.2/§9.1. | H |
| P2 | **Command map diverges ≥ id 0x31.** Python has `GetBulk*`/`*BlockRead*` where stock firmware/`librhsp` have FTDI reset / PIDF / I2C-transaction / `SET_BULK_OUTPUT_DATA` / binary `READ_VERSION`. §4.6. | H |
| P3 | **`msgNum` violates spec.** Sent as `0` on first transmission; spec requires ≥ 1 (`sendAndReceive` resets a local each call). §2.1, §12.1. | M |
| P4 | **Response not validated.** `checkResponse` is never called on the receive path; packet-type and `refNum`↔`msgNum` correlation are not enforced. §12.2. | M |
| P5 | **Payload cap 128 vs 512.** `PAYLOAD_MAX_SIZE = 128`; `librhsp` allows 512. Large I2C/version reads can exceed 128. §2.2. | M |
| P6 | **NACK swallowed.** A NACK is `print`ed and flow continues; the §4.4 nack codes are never surfaced as an error. | M |
| P7 | **Command-specific bugs** (each breaks one operation): `I2CChannel.set_speed` drops the client arg (`i2c.py:125`); `Module.setAllDIO` drops the client arg (`module.py:131`); `DIOPin.getDirection` never returns (`dio.py:36`); `i2cBlockReadConfig`/`imuBlockReadConfig` set attrs on the message, not its payload (`internal/i2c.py:126`); `setCurrentPIDCoefficients` calls the *getter* with setter args (`internal/motors.py:158`); `I2CConfigureQuery_RSP` is missing from `printDict` (decode `KeyError`); `distance.py:458` references undefined `OSC_CALIBRATE_VAL`; `GetPWNPulseWidth` response declares 1 byte vs 2 in the setter. | H |
| P8 | **Good alignments to preserve:** frame `44 4B`, little-endian, additive mod-256 checksum, Q16 PID, signed encoder/velocity, and LED-pattern byte order `[t,b,g,r]` all match the spec. | — |

### 1C. Idiomatic Python — *it was reverse-engineered; make it idiomatic* (separate)

**Verdict: not idiomatic; clear decompilation artifacts.**

| # | Finding | Sev |
|---|---------|-----|
| I1 | **Hex-ASCII string wire format.** `REVBytes`/`REVPayload`/`REVHeader`/`REVPacket` store bytes as hex strings; custom `__setattr__`/`__add__`/`__radd__`/`__getitem__`/`__len__`; `creationCounter`/`memberOrder` to recover field order; `swapEndianess` by string-slicing; manual two's-complement; `binascii.hexlify/unhexlify` round-trips. Should be `struct` / `int.to_bytes` / dataclasses. | H |
| I2 | **Duplicate `REVBytes.__add__`** (defined twice; second shadows first) — latent bug. (`messages.py:89,98`) | H |
| I3 | **`exit()` for error handling** in `REVHeader.__setattr__`, `REVPayload.__setattr__`, and `Client.sendAndReceive` — a library must raise, not kill the process. | H |
| I4 | **camelCase pervasive** (methods, locals, internal funcs); file typo `rshp_serial.py`; mixed `commObj` vs `client` naming. | M |
| I5 | **CPU busy-wait I/O.** `while inWaiting()==0` with no sleep (up to ~3 s spin across retries); no inter-byte timeout (hangs on a partial packet); hardcoded `time.sleep(2)` in discovery and `0.2 s` per port in enumerate. | H |
| I6 | **Inconsistent/sparse type hints**, bare `Any` for `commObj`/`module`; **no docstrings** on core classes; magic numbers (bit masks, register addresses); dynamic format-string building (`'%0'+str(n)+'X'`). | M |
| I7 | **Dead imports/vars/code:** `tkinter`/`queue`/unused `mp` in client.py; unused parse-state constants 4–9; `RetryTimeout_s`; commented stubs; `distance.py` `__main__` references undefined `REVComm`. | L |

---

## Part 2 — Target architecture (recommended)

Rewrite the wire layer as a **data-table-driven, struct-based codec seeded from
`docs/rhsp-protocol.json`** — 72 data rows replace 224 classes. Ship the JSON in
the wheel as the single source of truth.

### Proposed layout
```
src/rhsp/
  __init__.py      public exports: connect(), Hub, exceptions, enums
  protocol.json    packaged copy of docs/rhsp-protocol.json (runtime data)
  catalogue.py     loads JSON → Command/Response tables
  codec.py         Field/Command/Response dataclasses + generic encode/decode
  framing.py       FRAME bytes, length, checksum, FrameParser state machine
  transport.py     SerialTransport (pyserial, blocking+timeout) + LoopbackTransport
  session.py       Session: transaction(), msg_num≥1 sequencing, retry, NACK→raise
  discovery.py     port enumeration (D-prefix rule) + Discovery broadcast
  hub.py           Hub (was Module): peripherals, keep-alive, dynamic DEKA base id
  errors.py        RhspError, NackError(code), ChecksumError, RhspTimeoutError, ProtocolError
  enums.py         MotorMode, ZeroPowerBehavior, ClosedLoopMode, DIODirection,
                   ADCChannel, I2CSpeedCode, NackCode, ModuleStatusBits, MotorStatusBits
  devices/         motor.py servo.py dio.py adc.py i2c.py
  sensors/         color.py distance.py imu.py registers.py
  py.typed
```
**Deleted:** `__main__.py`, the `internal/` package, `messages.py`,
`rshp_serial.py`, `printDict`, and all `REV*` hex-string classes.

### Key design points
- **Codec (recommended: runtime data table, not codegen, not 110 dataclasses).**
  One `encode_payload(fields, values)` / `decode_payload(fields, data)` pair
  driven by `Field(name, nbytes, signed, fixed_point, variable)` rows.
  Endianness/signedness/Q16/checksum centralized:
  `int.to_bytes(n,"little",signed=…)`, `round(v*65536)` for Q16, fixed 10-byte
  header via `struct.Struct("<2sHBBBBH")`, `checksum = sum(buf) & 0xFF`.
- **Typed convenience layer:** hand-written snake_case `Session` methods for the
  ~20 commands the device API uses (docstrings + type hints live here), plus a
  generic `session.transaction(name, dest, **fields)` escape hatch for the rest —
  avoids re-introducing 72 stubs.
- **Collapse the two-layer split:** device classes call `self._session.<op>`
  directly; `internal/` disappears.
- **Transport:** blocking pyserial `read` with `timeout`/`inter_byte_timeout`
  (kills the busy-wait); `FrameParser` yields `RawPacket`s or raises
  `ChecksumError`; 512-byte buffers. A `Transport` Protocol enables a
  `LoopbackTransport`/`FakeHub` for hardware-free tests.
- **Session:** persistent `msg_num` 1..255 (never 0); response-type + `ref_num`
  validation re-enabled; `NackError(code)` with the §4.4 meaning; retry on
  timeout/checksum.
- **Dynamic DEKA base** via `QueryInterface("DEKA")` (fixes P1); command id =
  `base + deka_index`.
- **Device API:** uniform `(session, channel/pin, address)` constructors; drop
  the `module`+`destinationModule` duplication; bulk responses decode to named
  dataclasses (no positional `bulkData[ch+OFFSET]` arithmetic).
- **Errors:** `errors.py` replaces every `exit()`/`print+return False`.
- **Sensors:** keep in-package under `sensors/` (they're the headline value),
  layered strictly on `I2CDevice`; register maps → `sensors/registers.py`.
- **Keep-alive (2500 ms):** explicit `Hub.keep_alive()` plus an **opt-in**
  daemon-thread `start_keepalive()` guarded by the same lock that serializes all
  serial I/O.

---

## Part 3 — Phased migration

Each phase is independently verifiable; build new alongside old, then delete — no
big-bang.

0. **Test scaffolding** — `tests/` + the §9.5 worked byte vectors as fixtures.
1. **Wire layer** — `codec.py` + `framing.py` + `catalogue.py`, seeded from
   `protocol.json`. Verifies offline (round-trip every JSON entry + golden
   vectors). Fixes 512/signed/Q16 by construction.
2. **Transport + Session + errors + enums** — blocking I/O, msg_num≥1, response
   validation, `NackError`, retry. Verifies via `LoopbackTransport`/`FakeHub`.
3. **Discovery + Hub + dynamic DEKA base** — `QueryInterface` resolution (P1);
   timeout-drained discovery (drops `sleep(2)`).
4. **Devices** — Motor/Servo/DIOPin/ADCPin/I2C on the new Session; **fix all P7
   bugs in new code** (verified by FakeHub byte assertions); bulk → dataclasses.
5. **Sensors** — port color/distance/imu; register maps → `registers.py`; verify
   the emitted I2C register sequence against a recording FakeHub.
6. **Public API + examples + README + delete GUI** — `connect()`/`Hub`; rewrite
   `test/` scripts to snake_case (move to `examples/`, skip without a hub);
   delete `__main__.py`; fix the README `python -m rhsp` claim.
7. **Delete old layer + invert the generator** — remove `internal/`,
   `messages.py`, `rshp_serial.py`; repoint `docs/generate_protocol_json.py` to
   introspect the new `catalogue.py` (JSON is now source, not output).

Phases 1–5 add new modules without touching old ones, so the package keeps
working throughout; Phase 6 flips the public surface; Phase 7 deletes.

---

## Part 4 — Verification (hardware-free)

- **Golden vectors:** assert `encode(KeepAlive, dest=1) == bytes.fromhex("444B0B0001000000047F1E")` etc. (docs §9.5), plus `decode(encode(x))==x` over min/max signed, Q16 extremes, and 512-byte payloads for every JSON command/response.
- **`LoopbackTransport` + `FakeHub`:** exercise the full Session path (msg_num sequencing, ref_num correlation, NACK→exception, timeout→retry) and assert the **exact bytes the hub receives** for device/sensor operations.
- **`test/` scripts → `examples/`:** hardware smoke tests, skipped when no `D…` hub is enumerated. CI runs only the hardware-free `tests/`.
- **One genuine hardware dependency:** `Motor.get_velocity` uses `GetBulkMotorData` (0x1037), which sits in the divergent high-id range (P2) — must be confirmed against the target hub (optionally via the Saleae analyzer).

---

## Part 5 — Open decisions (recommended defaults; confirm at execution)

1. **High-id divergence (P2/§4.6):** *recommend* tagging the legacy Python
   `GetBulk*`/`BlockRead*` commands `firmware_variant: legacy` behind the generic
   `transaction()` escape hatch, and adding the stock-firmware commands (PIDF,
   I2C-transaction, binary `READ_VERSION`, `SET_BULK_OUTPUT_DATA`) as a second
   tagged group — carry both, default to neither in the typed API until validated.
2. **Sync vs async:** *recommend sync* (protocol is strictly half-duplex,
   one-outstanding; matches current usage).
3. **Keep-alive:** *recommend opt-in* background thread (off by default; explicit
   `keep_alive()` always available) — safety-relevant (outputs fail-safe at 2.5 s).
4. **Typed methods vs generic only:** *recommend hybrid* (typed methods for the
   ~20 device-API commands + generic `transaction()`).
5. **Min Python:** pyproject pins `>=3.13`; the design works on 3.10+. *Recommend
   keep 3.13* unless wider support is wanted.

---

## Critical files

- `docs/rhsp-protocol.json` — seeds `catalogue.py`/`codec.py` (the key asset).
- `src/rhsp/internal/messages.py` — wire layer being replaced (field layouts, bugs).
- `src/rhsp/client.py` — transport/session to redesign (msg_num, validation, busy-wait).
- `src/rhsp/module.py` — `Module`→`Hub` model + `init_periphs` bring-up sequence.
- `src/rhsp/{motors,servo,dio,adc,i2c,color,distance,imu}.py` — device/sensor API to redesign.
- `docs/generate_protocol_json.py` — inverted in Phase 7 to introspect the new tables.
