---
id: '001-004'
title: Catalogue — JSON-to-tables loader, legacy tagging, runtime_packet_id
status: open
use-cases:
  - SUC-003
depends-on:
  - '001-002'
  - '001-003'
---

# Ticket 004: Catalogue — JSON-to-tables loader, legacy tagging, runtime_packet_id

## Description

Implement `src/rhsp/catalogue.py`: load `src/rhsp/protocol.json` via
`importlib.resources` at module import time, build the `COMMANDS` and
`RESPONSES_BY_ID` lookup tables using the `Field`, `Command`, and `Response`
dataclasses from `codec.py`, apply the JSON overlay (signed, fixed_point, enum,
range) when constructing `Field` instances, tag legacy commands, and expose
`runtime_packet_id()`.

After this ticket, every other module can look up any command by name and get a
fully typed descriptor without reading JSON itself.

## Acceptance Criteria

- [ ] `catalogue.COMMANDS: dict[str, Command]` has exactly 72 entries (one per JSON command)
- [ ] `catalogue.RESPONSES_BY_ID: dict[int, Response]` has exactly 38 entries
- [ ] `COMMANDS["GetBulkMotorData"].legacy is True` (DEKA group, index 0x37 ≥ 0x31)
- [ ] `COMMANDS["GetBulkInputData"].legacy is False` (DEKA group, index 0x00 < 0x31)
- [ ] `COMMANDS["KeepAlive"].legacy is False` (system group — legacy rule only applies to DEKA group)
- [ ] `COMMANDS["SetMotorPIDCoefficients"].fields` contains a `Field` with `fixed_point=65536` (Q16)
- [ ] `COMMANDS["SetMotorConstantPower"].fields` contains a `Field` with `signed=True` for `powerLevel`
- [ ] `runtime_packet_id(COMMANDS["SetMotorConstantPower"], deka_base=0x1000)` returns `0x100F`
- [ ] `runtime_packet_id(COMMANDS["KeepAlive"], deka_base=0x1000)` returns `0x7F04` (system command, not DEKA)
- [ ] `catalogue.py` imports only `importlib.resources`, `json`, and `rhsp.codec`; never reads files elsewhere
- [ ] Module import is idempotent (tables built once at import time, not per call)
- [ ] Full codec round-trip test against all 72 commands and 38 responses passes
  (Extend `tests/test_codec.py`: for each entry in `COMMANDS` and `RESPONSES_BY_ID`,
  build a sample values dict with field-appropriate min/max/zero values, encode, decode,
  assert equality)

## Implementation Plan

### Approach

At module level, use `importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()`
to load the JSON. Parse into a dict. Iterate the `commands` array and `responses` array.

For each command:
- Build `Field` tuples from the command's `fields` list, applying `signed`, `fixed_point`,
  `unit`, `enum`, `range` from the JSON overlay where present.
- Set `legacy = (entry["group"] == "deka" and entry["index"] >= 0x31)`.
- Construct `Command(name, id, group, index, fields, reply_kind, reply_id, reply_name, legacy, notes)`.
- Insert into `COMMANDS[name]`.

For each response:
- Build `Field` tuples from the response's `fields` list.
- Construct `Response(name, id, fields)`.
- Insert into `RESPONSES_BY_ID[id]`.

`runtime_packet_id(cmd, deka_base)`:
- If `cmd.group == "deka"`: return `deka_base + cmd.index`
- Else: return `cmd.id`

Also expose `FRAMING`, `CONSTANTS`, `COUNTS`, `ENUMS` dicts from the JSON top-level
sections for reference (needed by `enums.py` to stay in sync; accessible but not
required by other runtime modules).

### Files to Create

None (catalogue.py is a new module, but is listed in "Files to Modify" for clarity).

### Files to Modify

- `src/rhsp/catalogue.py` — full implementation (currently does not exist)
- `tests/test_codec.py` — add a parametrize loop over `COMMANDS` and `RESPONSES_BY_ID`
  for the full round-trip test

### Testing Plan

`tests/test_catalogue.py` (new file):
- Assert `len(COMMANDS) == 72`.
- Assert `len(RESPONSES_BY_ID) == 38`.
- Assert `COMMANDS["GetBulkMotorData"].legacy is True`.
- Assert `COMMANDS["GetBulkInputData"].legacy is False`.
- Assert `runtime_packet_id(COMMANDS["SetMotorConstantPower"], 0x1000) == 0x100F`.
- Assert `runtime_packet_id(COMMANDS["KeepAlive"], 0x1000) == 0x7F04`.
- Assert PID command has Q16 field.
- Assert power command has signed field.

### Documentation Updates

None required for this ticket.
