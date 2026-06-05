---
id: 001-015
title: Invert generate_protocol_json.py to introspect catalogue.py
status: done
use-cases:
- SUC-009
- UC-025
depends-on:
- 001-014
---

# Ticket 015: Invert generate_protocol_json.py to introspect catalogue.py

## Description

Rewrite `docs/generate_protocol_json.py` so that it introspects `catalogue.py`
(the new `COMMANDS`/`RESPONSES_BY_ID` tables) and emits JSON. The emitted JSON
must match the committed `src/rhsp/protocol.json` byte-for-byte (modulo
insignificant whitespace). This inverts the historical relationship: `protocol.json`
is now the source of truth, and the generator is a validator.

This is the final ticket of the sprint. It completes the UC-025 use case and
surfaces any drift between the runtime catalogue and the committed spec.

## Acceptance Criteria

- [x] `docs/generate_protocol_json.py`:
  - Imports `rhsp.catalogue.COMMANDS` and `rhsp.catalogue.RESPONSES_BY_ID`
  - Iterates commands and responses to produce a JSON structure matching the committed `protocol.json` schema
  - Does NOT read the original `docs/rhsp-protocol.json` as input — it reads the live catalogue tables
- [x] `python docs/generate_protocol_json.py` runs without error (requires `uv run`)
- [x] The generated JSON, when diffed against `src/rhsp/protocol.json`, shows no meaningful differences:
  - Same commands with same names, ids, groups, indices, field layouts, reply kinds
  - Same responses with same names, ids, field layouts
  - Enum values match
  - The `signed` flag for encoder/velocity fields in `BulkInputData` response may differ
    (see open question 3 — document this explicitly if it appears in the diff)
- [x] If a diff is detected, the ticket is NOT complete until either:
  - The JSON is updated to match the catalogue (if the catalogue is correct), OR
  - The catalogue is corrected (if the JSON is correct), AND
  - The reason for the drift is documented in a comment in `generate_protocol_json.py`
- [x] The script exits with code 0 if the generated JSON matches committed JSON; code 1 if there is a diff
  (making it usable as a CI gate)
- [x] `uv run python docs/generate_protocol_json.py` exits 0 at sprint close

## Implementation Plan

### Approach

`generate_protocol_json.py` is restructured as:

```python
from rhsp.catalogue import COMMANDS, RESPONSES_BY_ID
import json

def command_to_dict(cmd) -> dict:
    return {
        "name": cmd.name,
        "id": cmd.id,
        "group": cmd.group,
        "index": cmd.index,
        "legacy": cmd.legacy,
        "fields": [field_to_dict(f) for f in cmd.fields],
        "reply_kind": cmd.reply_kind,
        "reply_id": cmd.reply_id,
        "reply_name": cmd.reply_name,
        "notes": cmd.notes,
    }

def field_to_dict(f) -> dict: ...
def response_to_dict(r) -> dict: ...

generated = {
    "commands": [command_to_dict(c) for c in COMMANDS.values()],
    "responses": [response_to_dict(r) for r in RESPONSES_BY_ID.values()],
}
```

Then compare against `importlib.resources.files("rhsp").joinpath("protocol.json").read_text()`.
Output the diff if any; exit 1 if diff found.

The `signed` field drift (open question 3): if encoder/velocity fields are not
marked `signed` in the JSON but `BulkInputData.from_response()` reinterprets them
as signed, document this in a comment and treat it as a known drift item. Do not
silently suppress it; the diff should be visible.

### Files to Modify

- `docs/generate_protocol_json.py` — full rewrite

### Files That May Be Modified (if drift is found)

- `src/rhsp/protocol.json` — updated to match catalogue (or vice versa)

### Testing Plan

- `uv run python docs/generate_protocol_json.py` exits 0 after all drift is resolved.
- CI step (optional): add to `Makefile` or `pyproject.toml` scripts section:
  `"validate-json": "python docs/generate_protocol_json.py"`

### Documentation Updates

- Add a comment block at the top of `docs/generate_protocol_json.py` explaining the
  inverted role: "This script is now a validator. It reads `rhsp.catalogue` and
  asserts the output matches `src/rhsp/protocol.json`. Any diff indicates drift
  between the runtime catalogue and the committed spec."
