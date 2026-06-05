---
id: '001-002'
title: Codec — encode/decode with signed, Q16, and variable-length fields
status: open
use-cases:
  - SUC-001
depends-on:
  - '001-001'
---

# Ticket 002: Codec — encode/decode with signed, Q16, and variable-length fields

## Description

Implement `src/rhsp/codec.py`: the `Field`, `Command`, and `Response` frozen
dataclasses, and the two generic functions `encode_payload` / `decode_payload`
that convert between Python dicts and raw bytes. This is the only place in the
library where binary serialization rules live.

No command-specific classes. No JSON reads. `codec.py` is purely data-driven by
the `Field` descriptors passed in.

## Acceptance Criteria

- [ ] `Field` is a frozen dataclass with slots: `name: str`, `nbytes: int`,
  `offset: int`, `signed: bool = False`, `fixed_point: int | None = None`,
  `unit: str | None = None`, `enum: str | None = None`, `range: tuple[int,int] | None = None`
- [ ] `Command` is a frozen dataclass with slots: `name`, `id`, `group`, `index`,
  `fields: tuple[Field, ...]`, `reply_kind`, `reply_id`, `reply_name`,
  `legacy: bool = False`, `notes: str = ""`
- [ ] `Response` is a frozen dataclass with slots: `name`, `id`, `fields: tuple[Field, ...]`
- [ ] `encode_payload(fields, values) -> bytes`:
  - Encodes each field using `int.to_bytes(n, "little", signed=field.signed)`
  - For `fixed_point` fields: value is `round(float_value * field.fixed_point)`, stored as signed-`nbytes`
  - Last field absorbs remaining bytes for variable-length fields (bytes/str values passed through)
  - Concatenates all encoded fields in order
- [ ] `decode_payload(fields, data) -> dict`:
  - Decodes each field using `int.from_bytes(slice, "little", signed=field.signed)`
  - For `fixed_point` fields: returns `raw_int / field.fixed_point` as float
  - Last field absorbs remaining data bytes
  - Returns dict keyed by field name
- [ ] `decode(encode(values)) == values` for:
  - `signed=True` 16-bit field at min (−32768) and max (32767)
  - `signed=True` 32-bit field at min (−2147483648) and max (2147483647)
  - Q16 field (`fixed_point=65536`) with value `1.5` (encode → `round(1.5*65536)=98304`)
  - Q16 field with value `−1.5`
  - Variable-length last field with 512 bytes of content
  - Variable-length last field with 0 bytes (empty)
- [ ] `tests/test_codec.py` round-trip test covers all 72 commands and 38 responses (using field descriptors built by hand or loaded via catalogue once ticket 004 exists; for this ticket, at least a representative set of 10+ entries)

## Implementation Plan

### Approach

Write the dataclasses first (they require only `dataclasses` and `typing`). Then
implement `encode_payload` as a straightforward loop over fields. Implement
`decode_payload` symmetrically. Handle the variable-length tail case last (it
only applies when `fields[-1].nbytes == 0` or when the data exceeds the sum of
fixed field sizes — use the last field to absorb remaining bytes).

The fixed_point denominator (e.g. 65536 for Q16) is stored on `Field.fixed_point`.
Encode: `round(float_val * denom)` then `int.to_bytes(nbytes, "little", signed=True)`.
Decode: `int.from_bytes(bytes, "little", signed=True) / denom`.

### Files to Create

- `src/rhsp/codec.py` — `Field`, `Command`, `Response`, `encode_payload`, `decode_payload`
- `tests/test_codec.py` — round-trip tests; signed extremes; Q16; variable-length

### Files to Modify

None.

### Testing Plan

`tests/test_codec.py`:
- Build a set of `Field` descriptors by hand covering: unsigned 1-byte, unsigned 2-byte LE,
  signed 2-byte LE (16-bit), signed 4-byte LE (32-bit), Q16 signed-4, variable-length tail.
- Assert `decode_payload(fields, encode_payload(fields, values)) == values` for each.
- Assert signed min/max round-trips exactly.
- Assert Q16 `1.5` encodes to 4 bytes `00 80 01 00` (little-endian 98304) and decodes back to `1.5`.
- Assert 512-byte variable-length tail round-trips.
- Assert empty variable-length tail round-trips.
- Assert `Field` is hashable (frozen dataclass).

### Documentation Updates

None required for this ticket.
