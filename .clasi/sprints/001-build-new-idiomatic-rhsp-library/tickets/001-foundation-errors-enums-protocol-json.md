---
id: '001-001'
title: Foundation — errors, enums, and packaged protocol.json
status: open
use-cases:
  - SUC-003
  - SUC-009
depends-on: []
issue:
  - rhsp-build-a-new-idiomatic-python-library.md
  - rhsp-idiomatic-rewrite.md
---

# Ticket 001: Foundation — errors, enums, and packaged protocol.json

## Description

Create the foundation layer for `src/rhsp/`: the exception hierarchy, the
integer enum types, and the packaged runtime data file. This ticket has no
dependencies and unblocks every other ticket.

No codec, framing, or session logic belongs here. The goal is a clean,
importable `rhsp` package root with errors and enums available from the
start.

## Acceptance Criteria

- [ ] `src/rhsp/errors.py` defines the full exception hierarchy:
  - `RhspError(Exception)` — base for all library errors
  - `ProtocolError(RhspError)` — framing/protocol violations
  - `ChecksumError(ProtocolError)` — bad packet checksum
  - `RhspTimeoutError(RhspError)` — transaction timed out
  - `NackError(RhspError)` — hub rejected a command; has `code: int` and `description: str` attributes
- [ ] `src/rhsp/enums.py` defines typed enums (Python `IntEnum`/`IntFlag`) for at minimum:
  - `MotorMode` (CONSTANT_POWER=0, CONSTANT_VELOCITY=1, POSITION_TARGET=2, CONSTANT_CURRENT=3)
  - `ZeroPowerBehavior` (BRAKE_AT_ZERO=0, FLOAT_AT_ZERO=1)
  - `NackCode` (all codes from §2.12 of the spec) with a `describe()` classmethod
  - `ModuleStatusBits` (`IntFlag`, bits 0–5 per §2.13)
  - `MotorStatusBits` (`IntFlag`, bits 0–7 per §2.13)
  - `ADCChannel` (channels 0–14 per §2.11)
  - `I2CSpeedCode` (standard/fast speed codes)
- [ ] `src/rhsp/protocol.json` is a byte-for-byte copy of `docs/rhsp-protocol.json`
- [ ] `pyproject.toml` (or equivalent packaging config) includes `src/rhsp/protocol.json`
  as package data so it is included in the installed package and accessible via
  `importlib.resources`
- [ ] `src/rhsp/__init__.py` exists (may be minimal at this stage; at least `from .errors import *`)
- [ ] `src/rhsp/py.typed` exists (empty PEP 561 marker)
- [ ] `from rhsp.errors import NackError, ChecksumError, RhspTimeoutError` succeeds
- [ ] `from rhsp.enums import MotorMode, NackCode` succeeds
- [ ] `NackError(50, "Motor not configured")` has `.code == 50` and `.description == "Motor not configured"`
- [ ] `import importlib.resources; importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()` returns valid JSON

## Implementation Plan

### Approach

Pure Python; no external dependencies beyond the standard library. Write
`errors.py` first (simplest, no imports from within `rhsp`), then `enums.py`
(imports only `errors.py` for `NackCode.describe()` optionally), then copy
`protocol.json`, then the minimal `__init__.py`.

### Files to Create

- `src/rhsp/__init__.py` — minimal; exports error and enum names
- `src/rhsp/errors.py` — full exception hierarchy
- `src/rhsp/enums.py` — all IntEnum/IntFlag types
- `src/rhsp/protocol.json` — copy of `docs/rhsp-protocol.json`
- `src/rhsp/py.typed` — empty marker
- `tests/__init__.py` — empty (if not present)
- `tests/test_foundation.py` — import smoke tests + enum value assertions

### Files to Modify

- `pyproject.toml` — add `package-data` or equivalent for `protocol.json`

### Testing Plan

`tests/test_foundation.py`:
- Import all error classes; assert MRO is correct.
- Assert `NackError(50, "x").code == 50`.
- Assert `MotorMode.CONSTANT_POWER == 0`.
- Assert `NackCode` has entries for all 20+ codes from §2.12.
- Assert `importlib.resources` can load `protocol.json` and parse it as JSON.
- Assert `ModuleStatusBits` is an `IntFlag`.

### Documentation Updates

None required for this ticket.
