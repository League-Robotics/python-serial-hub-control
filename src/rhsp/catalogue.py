"""Catalogue — protocol.json loader, command/response tables, legacy tagging.

Loads ``src/rhsp/protocol.json`` at module import time via
``importlib.resources`` and builds two lookup tables:

- ``COMMANDS: dict[str, Command]``  — keyed by command name, 72 entries.
- ``RESPONSES_BY_ID: dict[int, Response]`` — keyed by numeric packet id,
  38 entries.

Also exposes raw top-level JSON sections for reference:

- ``FRAMING``   — framing/header constants.
- ``CONSTANTS`` — numeric protocol constants.
- ``COUNTS``    — per-peripheral channel counts.
- ``ENUMS``     — raw enum string→int mappings (for ``enums.py``).

Legacy tagging
--------------
A command is marked ``legacy=True`` when its group is ``"deka"`` *and* its
DEKA function index is ≥ 0x31.  These are the high-id commands where the
stock firmware and the Python vendor package disagree on the function map.
Legacy commands are still present in ``COMMANDS`` but must not be called via
typed session methods; use ``session.transaction()`` instead.

``runtime_packet_id``
---------------------
Resolves the wire packet-type id for a command given the runtime DEKA base
obtained from ``QueryInterface``:

- DEKA commands: ``deka_base + cmd.index``.
- All other groups (system, etc.): ``cmd.id`` (absolute, no base needed).

Field overlay
-------------
The ``signed``, ``fixed_point``, ``unit``, ``enum``, and ``range`` columns
in the JSON payload entries are applied here when constructing ``Field``
instances.  The ``"Q16"`` token is resolved to the integer ``65536``.
``codec.py`` itself stays JSON-free.
"""

from __future__ import annotations

__all__ = [
    "COMMANDS",
    "RESPONSES_BY_ID",
    "FRAMING",
    "CONSTANTS",
    "COUNTS",
    "ENUMS",
    "command",
    "deka_index",
    "response_for",
    "runtime_packet_id",
]

import importlib.resources
import json

from rhsp.codec import Command, Field, Response

# ---------------------------------------------------------------------------
# Internal constants
# ---------------------------------------------------------------------------

_DEKA_BASE: int = 0x1000
_SYSTEM_BASE: int = 0x7F00

_FIXED_POINT_ALIASES: dict[str, int] = {
    "Q16": 65536,
}

# Legacy threshold: DEKA indices >= 0x31 diverge from stock firmware.
_LEGACY_INDEX_THRESHOLD: int = 0x31


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _resolve_fixed_point(value: object) -> int | None:
    """Convert a JSON ``fixed_point`` value to an integer denominator.

    Accepts ``None``, an integer, or the string token ``"Q16"``
    (→ 65536).  Raises ``ValueError`` for unknown tokens.
    """
    if value is None:
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        resolved = _FIXED_POINT_ALIASES.get(value)
        if resolved is None:
            raise ValueError(f"Unknown fixed_point token: {value!r}")
        return resolved
    raise TypeError(f"Unexpected fixed_point type: {type(value)!r}")


def _build_field(fd: dict) -> Field:
    """Construct a ``Field`` from one JSON payload-entry dict."""
    raw_range = fd.get("range")
    range_tuple: tuple[int, int] | None = (
        (int(raw_range[0]), int(raw_range[1])) if raw_range is not None else None
    )
    return Field(
        name=fd["name"],
        nbytes=fd["bytes"],
        offset=fd.get("offset", 0),
        signed=bool(fd.get("signed", False)),
        fixed_point=_resolve_fixed_point(fd.get("fixed_point")),
        unit=fd.get("unit"),
        enum=fd.get("enum"),
        range=range_tuple,
    )


def _fields_from_payload(payload: list) -> tuple[Field, ...]:
    """Build an ordered ``Field`` tuple from a JSON ``payload`` list."""
    return tuple(_build_field(fd) for fd in payload)


def _command_index(entry: dict) -> int:
    """Derive the function index from the command id and group."""
    group: str = entry["group"]
    cid: int = entry["id"]
    if group == "deka":
        return cid - _DEKA_BASE
    if group == "system":
        return cid - _SYSTEM_BASE
    # Fallback: use the raw id as the index.
    return cid


# ---------------------------------------------------------------------------
# Module-level tables (built once at import time)
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Command] = {}
RESPONSES_BY_ID: dict[int, Response] = {}
FRAMING: dict = {}
CONSTANTS: dict = {}
COUNTS: dict = {}
ENUMS: dict = {}


def _load() -> None:
    """Parse ``protocol.json`` and populate all module-level tables."""
    raw = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    data: dict = json.loads(raw)

    # Top-level reference sections.
    FRAMING.update(data.get("framing", {}))
    CONSTANTS.update(data.get("constants", {}))
    COUNTS.update(data.get("counts", {}))
    ENUMS.update(data.get("enums", {}))

    # Build COMMANDS.
    for entry in data.get("commands", []):
        name: str = entry["name"]
        group: str = entry["group"]
        cid: int = entry["id"]
        idx: int = _command_index(entry)
        legacy: bool = group == "deka" and idx >= _LEGACY_INDEX_THRESHOLD
        reply = entry["reply"]

        fields = _fields_from_payload(entry.get("payload", []))

        cmd = Command(
            name=name,
            id=cid,
            group=group,
            index=idx,
            fields=fields,
            reply_kind=reply["kind"],
            reply_id=reply["id"],
            reply_name=reply["command"],
            legacy=legacy,
            notes=entry.get("notes", ""),
        )
        COMMANDS[name] = cmd

    # Build RESPONSES_BY_ID.
    for entry in data.get("responses", []):
        rname: str = entry["name"]
        rid: int = entry["id"]
        fields = _fields_from_payload(entry.get("payload", []))

        rsp = Response(
            name=rname,
            id=rid,
            fields=fields,
        )
        RESPONSES_BY_ID[rid] = rsp


# Execute the load exactly once at import time.
_load()


# ---------------------------------------------------------------------------
# Public accessors
# ---------------------------------------------------------------------------


def command(name: str) -> Command:
    """Return the ``Command`` descriptor for *name*.

    Raises:
        KeyError: if *name* is not in the catalogue.
    """
    return COMMANDS[name]


def deka_index(name: str) -> int:
    """Return the DEKA function index for the named command.

    Raises:
        KeyError:    if *name* is not in the catalogue.
        ValueError:  if the command is not in the ``"deka"`` group.
    """
    cmd = COMMANDS[name]
    if cmd.group != "deka":
        raise ValueError(
            f"Command {name!r} is in group {cmd.group!r}, not 'deka'."
        )
    return cmd.index


def response_for(req: Command) -> Response:
    """Return the ``Response`` descriptor that corresponds to *req*.

    For commands with ``reply_kind == "response"`` the response is looked
    up by the pre-computed ``req.reply_id``.  For ACK/NACK replies the
    corresponding ACK or NACK ``Response`` entry is returned.

    Raises:
        KeyError: if the reply id is not found in ``RESPONSES_BY_ID``.
    """
    return RESPONSES_BY_ID[req.reply_id]


def runtime_packet_id(cmd: Command, deka_base: int) -> int:
    """Compute the wire packet-type id for *cmd* given the runtime DEKA base.

    For DEKA commands the hub assigns the actual base address at runtime via
    ``QueryInterface``; it is not always 0x1000.  For system commands the id
    is absolute and fixed.

    Parameters:
        cmd:        Command descriptor from ``COMMANDS``.
        deka_base:  The DEKA interface base address returned by
                    ``QueryInterface`` (e.g. 0x1000).

    Returns:
        The wire packet-type id to use in the frame header.
    """
    if cmd.group == "deka":
        return deka_base + cmd.index
    return cmd.id
