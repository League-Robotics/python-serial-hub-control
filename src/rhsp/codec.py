"""Binary codec for the RHSP protocol.

Defines the ``Field``, ``Command``, and ``Response`` frozen dataclasses that
describe payload layouts, and the two generic functions ``encode_payload`` /
``decode_payload`` that convert between Python dicts and raw bytes.

No JSON reads occur here.  This module is data-driven: callers pass ``Field``
descriptors (typically built by ``catalogue.py`` from ``protocol.json``) and
this module handles all binary serialisation rules:

- Little-endian unsigned integers (default).
- Signed two's-complement integers (``field.signed = True``).
- Fixed-point values (``field.fixed_point = N``): encode stores
  ``round(value * N)`` as a signed integer; decode returns ``raw / N`` as a
  float.  Q16 uses ``N = 65536`` in a 4-byte signed field.
- Variable-length trailing field: the *last* field in a descriptor list is
  treated as variable-length.  On encode, a ``bytes`` or ``str`` value is
  emitted as-is (str → ASCII).  On decode it absorbs all remaining bytes.

Enumeration interpretation (mapping raw ints to ``IntEnum`` members) is
intentionally *not* performed here — that layer belongs to ``session.py`` or
the typed device API.
"""

from __future__ import annotations

__all__ = [
    "Field",
    "Command",
    "Response",
    "encode_payload",
    "decode_payload",
]

from dataclasses import dataclass, field
from typing import Sequence


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Field:
    """Descriptor for one field in a command or response payload.

    Attributes:
        name:        Snake-case field name (used as dict key in encode/decode).
        nbytes:      Number of bytes on the wire.  0 is reserved for variable-
                     length trailing fields (absorbs all remaining bytes).
        offset:      Byte offset from the start of the payload (informational;
                     encode/decode use sequential layout, not random-access).
        signed:      True if the field uses signed two's-complement encoding.
        fixed_point: Denominator for fixed-point encoding (e.g. 65536 for Q16).
                     When set, the field is stored as a signed integer and the
                     Python value is a float.
        unit:        Optional human-readable unit string (e.g. "mA", "mV").
        enum:        Optional name of the enum type for this field's values.
        range:       Optional (min, max) tuple for value validation (unused by
                     the codec itself; present for catalogue consumers).
    """

    name: str
    nbytes: int
    offset: int
    signed: bool = False
    fixed_point: int | None = None
    unit: str | None = None
    enum: str | None = None
    range: tuple[int, int] | None = None


@dataclass(frozen=True, slots=True)
class Command:
    """Descriptor for a command (request) entry from the protocol catalogue.

    Attributes:
        name:       Human-readable command name (e.g. ``"SetMotorConstantPower"``).
        id:         Numeric packet-type id.
        group:      Catalogue group (``"deka"``, ``"system"``, etc.).
        index:      DEKA function index (relative to the interface base).
        fields:     Ordered tuple of payload field descriptors.
        reply_kind: ``"ack"``, ``"nack"``, or ``"response"``.
        reply_id:   Numeric id of the expected reply packet.
        reply_name: Human-readable name of the reply (e.g. ``"ACK"``).
        legacy:     True if this command is at a firmware-divergent high index.
        notes:      Free-text annotation from the protocol catalogue.
    """

    name: str
    id: int
    group: str
    index: int
    fields: tuple[Field, ...]
    reply_kind: str
    reply_id: int
    reply_name: str
    legacy: bool = False
    notes: str = ""


@dataclass(frozen=True, slots=True)
class Response:
    """Descriptor for a response entry from the protocol catalogue.

    Attributes:
        name:   Human-readable response name (e.g. ``"GetADC_RSP"``).
        id:     Numeric packet-type id.
        fields: Ordered tuple of payload field descriptors.
    """

    name: str
    id: int
    fields: tuple[Field, ...]


# ---------------------------------------------------------------------------
# Encode
# ---------------------------------------------------------------------------


def encode_payload(fields: Sequence[Field], values: dict) -> bytes:
    """Serialise *values* into raw bytes according to *fields*.

    Parameters:
        fields: Ordered field descriptors.  The last descriptor is treated as
                variable-length if the corresponding value is ``bytes`` or
                ``str``.
        values: Dict mapping field names to Python values.  For fixed-point
                fields supply a ``float``; for variable-length trailing fields
                supply ``bytes`` or ``str``; for all other fields supply an
                ``int``-compatible value.

    Returns:
        Concatenated little-endian payload bytes.

    Raises:
        KeyError:    A field name is missing from *values*.
        ValueError:  A value cannot be encoded (out-of-range for the field
                     width, or an unexpected type for a variable-length field).
        OverflowError: An integer value exceeds the signed/unsigned range of
                     the field.
    """
    if not fields:
        return b""

    parts: list[bytes] = []
    last_idx = len(fields) - 1

    for i, f in enumerate(fields):
        raw = values[f.name]

        if i == last_idx and isinstance(raw, (bytes, bytearray, str)):
            # Variable-length trailing field: pass through as-is.
            if isinstance(raw, str):
                parts.append(raw.encode("ascii"))
            else:
                parts.append(bytes(raw))
            continue

        if f.fixed_point is not None:
            # Fixed-point: scale and store as signed integer.
            int_val = round(float(raw) * f.fixed_point)
            parts.append(int_val.to_bytes(f.nbytes, "little", signed=True))
        else:
            # Plain integer field.
            parts.append(int(raw).to_bytes(f.nbytes, "little", signed=f.signed))

    return b"".join(parts)


# ---------------------------------------------------------------------------
# Decode
# ---------------------------------------------------------------------------


def decode_payload(fields: Sequence[Field], data: bytes | bytearray) -> dict:
    """Deserialise raw *data* bytes into a Python dict according to *fields*.

    Parameters:
        fields: Ordered field descriptors.  The last descriptor absorbs all
                bytes that remain after the preceding fixed-width fields have
                been consumed.
        data:   Raw payload bytes received from the hub.

    Returns:
        Dict mapping each field name to its decoded Python value.  Signed and
        fixed-point conversions are applied; enum interpretation is left to the
        caller.

    Raises:
        ValueError: *data* is too short to satisfy the fixed-width fields.
    """
    if not fields:
        return {}

    result: dict = {}
    offset = 0
    last_idx = len(fields) - 1

    for i, f in enumerate(fields):
        if i == last_idx:
            # Last field: absorb remaining bytes regardless of nbytes.
            chunk = bytes(data[offset:])
            result[f.name] = chunk
        else:
            chunk = bytes(data[offset : offset + f.nbytes])
            if len(chunk) < f.nbytes:
                raise ValueError(
                    f"Payload too short: field '{f.name}' at offset {offset} "
                    f"requires {f.nbytes} bytes but only {len(chunk)} available."
                )

            if f.fixed_point is not None:
                raw_int = int.from_bytes(chunk, "little", signed=True)
                result[f.name] = raw_int / f.fixed_point
            else:
                result[f.name] = int.from_bytes(chunk, "little", signed=f.signed)

            offset += f.nbytes

    return result
