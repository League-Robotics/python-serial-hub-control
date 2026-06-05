#!/usr/bin/env python3
"""
Validator / regenerator for docs/rhsp-protocol.json.

ROLE (inverted from the original):
  This script is now a VALIDATOR.  It reads ``rhsp.catalogue`` (the live
  ``COMMANDS`` / ``RESPONSES_BY_ID`` tables, plus ``FRAMING``, ``CONSTANTS``,
  ``COUNTS``, ``ENUMS``) and reconstructs the protocol JSON from first
  principles.  The output is then diffed against the committed
  ``src/rhsp/protocol.json``.

  - Exit 0: generated JSON matches the committed file — no drift.
  - Exit 1: diff detected — the catalogue and the committed JSON have drifted
    and one of them must be corrected.

  The script also writes ``docs/rhsp-protocol.json`` so that the docs copy
  stays in sync with ``src/rhsp/protocol.json``.

WHAT IS INTROSPECTED vs. CURATED:
  Introspected from the live catalogue (cannot drift silently):
    - All command names, ids, groups, indices, payload fields, reply kinds.
    - All response names, ids, payload fields.
    - Enum definitions (from ENUMS).
    - Framing constants (from FRAMING).
    - Protocol constants (from CONSTANTS).
    - Channel counts (from COUNTS).

  Preserved verbatim from the committed ``src/rhsp/protocol.json``
  (curated narrative that has no catalogue counterpart):
    - ``_about``
    - ``protocol``
    - ``link_layer``
    - ``keep_alive_interval_ms`` / ``keep_alive_note``
    - ``firmware_command_map_divergence``
    - ``sequencing``

SIGNED-FIELD DRIFT NOTE (GetBulkInputData_RSP / GetBulkMotorData_RSP):
  The motorNEncoder and motorNVelocity fields in the bulk-input and
  bulk-motor responses are physically signed on the wire (two's-complement),
  but the current protocol.json does NOT carry ``"signed": true`` for those
  fields.  The ``BulkInputData`` helper in ``session.py`` compensates by
  reinterpreting them via ``struct.unpack`` after the generic decoder returns
  unsigned values.
  This is a KNOWN intentional drift: the catalogue omits the signed flag to
  preserve the existing wire-decode contract; the application layer performs
  the reinterpretation.  No correction is needed here; the drift is documented
  for future implementors.

Run with:  uv run python docs/generate_protocol_json.py
"""

import difflib
import importlib.resources
import json
import os
import sys

# Make src/rhsp importable when the script is run directly from the repo root.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.join(_repo_root, "src") not in sys.path:
    sys.path.insert(0, os.path.join(_repo_root, "src"))

from rhsp.catalogue import (  # noqa: E402  (must come after sys.path manipulation)
    COMMANDS,
    CONSTANTS,
    COUNTS,
    ENUMS,
    FRAMING,
    RESPONSES_BY_ID,
)


# ---------------------------------------------------------------------------
# Curated field-level descriptions that are not modeled in the Field dataclass.
#
# ``description`` entries in protocol.json are editorial annotations; they have
# no representation in ``codec.Field``.  Rather than silently dropping them (which
# would cause a spurious diff on every run), we preserve them here verbatim.
# Keyed by "<PacketName>.<fieldName>".
# ---------------------------------------------------------------------------

_FIELD_DESCRIPTIONS: dict[str, str] = {}  # populated from committed JSON at build time


# ---------------------------------------------------------------------------
# Helpers: convert Field dataclass → JSON dict
# ---------------------------------------------------------------------------

_FIXED_POINT_REVERSE: dict[int, str] = {65536: "Q16"}


def _field_to_dict(f, packet_name: str) -> dict:
    """Serialise one ``Field`` to its JSON representation."""
    d: dict = {
        "name": f.name,
        "bytes": f.nbytes,
        "offset": f.offset,
    }
    if f.signed:
        d["signed"] = True
    if f.fixed_point is not None:
        d["fixed_point"] = _FIXED_POINT_REVERSE.get(f.fixed_point, f.fixed_point)
    if f.unit is not None:
        d["unit"] = f.unit
    if f.enum is not None:
        d["enum"] = f.enum
    if f.range is not None:
        d["range"] = list(f.range)
    # Re-attach curated description if present.
    key = f"{packet_name}.{f.name}"
    if key in _FIELD_DESCRIPTIONS:
        d["description"] = _FIELD_DESCRIPTIONS[key]
    return d


def _payload_to_list(fields, packet_name: str) -> list:
    return [_field_to_dict(f, packet_name) for f in fields]


# ---------------------------------------------------------------------------
# Build commands list
# ---------------------------------------------------------------------------

def _build_commands() -> list:
    commands = []
    for name, cmd in COMMANDS.items():
        entry: dict = {
            "name": cmd.name,
            "id": cmd.id,
            "id_hex": "0x%04X" % cmd.id,
            "group": cmd.group,
            "payload": _payload_to_list(cmd.fields, cmd.name),
            "payload_bytes": sum(f.nbytes for f in cmd.fields),
            "reply": {
                "kind": cmd.reply_kind,
                "command": cmd.reply_name,
                "id": cmd.reply_id,
                "id_hex": "0x%04X" % cmd.reply_id,
            },
        }
        if cmd.notes:
            entry["notes"] = cmd.notes
        commands.append(entry)
    return commands


# ---------------------------------------------------------------------------
# Build responses list
# ---------------------------------------------------------------------------

def _build_responses() -> list:
    responses = []
    for rid, rsp in RESPONSES_BY_ID.items():
        entry: dict = {
            "name": rsp.name,
            "id": rsp.id,
            "id_hex": "0x%04X" % rsp.id,
            "group": "response",
            "payload": _payload_to_list(rsp.fields, rsp.name),
            "payload_bytes": sum(f.nbytes for f in rsp.fields),
        }
        responses.append(entry)
    return responses


# ---------------------------------------------------------------------------
# Load the committed protocol.json to pull curated blocks
# ---------------------------------------------------------------------------

def _load_committed() -> dict:
    """Return the parsed committed ``src/rhsp/protocol.json``."""
    raw = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    return json.loads(raw)


def _index_field_descriptions(committed: dict) -> None:
    """Populate ``_FIELD_DESCRIPTIONS`` from the committed JSON payload entries.

    ``description`` is a curated editorial annotation not modeled in
    ``codec.Field``; we carry it forward verbatim so the generated output
    matches the committed file.
    """
    for section in ("commands", "responses"):
        for entry in committed.get(section, []):
            pname = entry["name"]
            for fd in entry.get("payload", []):
                if "description" in fd:
                    _FIELD_DESCRIPTIONS[f"{pname}.{fd['name']}"] = fd["description"]


# ---------------------------------------------------------------------------
# Top-level build
# ---------------------------------------------------------------------------

def build() -> dict:
    """Reconstruct the full protocol document from the live catalogue."""
    committed = _load_committed()
    _index_field_descriptions(committed)

    doc = {
        # Curated narrative blocks — preserved verbatim from committed JSON.
        "_about": committed["_about"],
        "protocol": committed["protocol"],
        "link_layer": committed["link_layer"],
        # Introspected structural blocks.
        "framing": dict(FRAMING),
        "constants": dict(CONSTANTS),
        "enums": dict(ENUMS),
        "counts": dict(COUNTS),
        # Curated timing / divergence notes.
        "keep_alive_interval_ms": committed["keep_alive_interval_ms"],
        "keep_alive_note": committed["keep_alive_note"],
        "firmware_command_map_divergence": committed["firmware_command_map_divergence"],
        # Introspected command/response tables.
        "commands": _build_commands(),
        "responses": _build_responses(),
        # Curated sequencing recipes.
        "sequencing": committed["sequencing"],
    }
    return doc


# ---------------------------------------------------------------------------
# Diff / validation
# ---------------------------------------------------------------------------

def _normalise(doc: dict) -> str:
    """Return a canonical, stable JSON string for diffing."""
    return json.dumps(doc, indent=2, sort_keys=False) + "\n"


def _diff(generated: str, committed: str) -> list[str]:
    return list(
        difflib.unified_diff(
            committed.splitlines(keepends=True),
            generated.splitlines(keepends=True),
            fromfile="src/rhsp/protocol.json (committed)",
            tofile="generated (from catalogue)",
        )
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    doc = build()
    generated_text = _normalise(doc)

    # Write docs/rhsp-protocol.json (keeps the docs copy in sync).
    docs_out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rhsp-protocol.json")
    with open(docs_out, "w") as fh:
        fh.write(generated_text)
    print(f"Wrote {docs_out}: {len(doc['commands'])} commands, {len(doc['responses'])} responses.")

    # Validate against the packaged src/rhsp/protocol.json.
    committed_raw = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    committed_doc = json.loads(committed_raw)
    committed_text = _normalise(committed_doc)

    diff_lines = _diff(generated_text, committed_text)
    if diff_lines:
        print("\nDRIFT DETECTED — generated JSON differs from src/rhsp/protocol.json:")
        for line in diff_lines[:120]:
            print(line, end="")
        if len(diff_lines) > 120:
            print(f"\n... ({len(diff_lines) - 120} more lines)")
        sys.exit(1)
    else:
        print("OK — generated JSON matches src/rhsp/protocol.json (no drift).")
        sys.exit(0)
