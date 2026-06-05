"""Tests for rhsp.codec — encode_payload / decode_payload round-trips.

All tests are purely in-process; no hub hardware required.

Round-trip invariant: for fields/values where the last field is variable-length
(bytes), decode_payload(fields, encode_payload(fields, values)) == values.
For non-terminal fields, value types (int, float) survive encode→decode exactly.
"""

import importlib.resources
import json
import struct

import pytest

from rhsp.codec import Command, Field, Response, decode_payload, encode_payload


# ---------------------------------------------------------------------------
# Helpers — commonly reused field descriptors
# ---------------------------------------------------------------------------

UINT8 = Field(name="u8", nbytes=1, offset=0)
UINT16 = Field(name="u16", nbytes=2, offset=1)
INT16 = Field(name="s16", nbytes=2, offset=3, signed=True)
INT32 = Field(name="s32", nbytes=4, offset=5, signed=True)
Q16 = Field(name="q16", nbytes=4, offset=9, signed=True, fixed_point=65536)
VARTAIL = Field(name="tail", nbytes=0, offset=13)  # variable-length tail


# ---------------------------------------------------------------------------
# Dataclass smoke tests
# ---------------------------------------------------------------------------


def test_field_frozen() -> None:
    f = Field(name="x", nbytes=1, offset=0)
    with pytest.raises((AttributeError, TypeError)):
        f.name = "y"  # type: ignore[misc]


def test_field_hashable() -> None:
    f = Field(name="x", nbytes=1, offset=0, signed=True)
    assert hash(f) is not None
    s = {f}
    assert f in s


def test_field_defaults() -> None:
    f = Field(name="y", nbytes=2, offset=0)
    assert f.signed is False
    assert f.fixed_point is None
    assert f.unit is None
    assert f.enum is None
    assert f.range is None


def test_command_frozen() -> None:
    cmd = Command(
        name="Foo", id=1, group="deka", index=0,
        fields=(UINT8,), reply_kind="ack", reply_id=0x7F01, reply_name="ACK",
    )
    with pytest.raises((AttributeError, TypeError)):
        cmd.name = "Bar"  # type: ignore[misc]


def test_command_defaults() -> None:
    cmd = Command(
        name="Foo", id=1, group="deka", index=0,
        fields=(), reply_kind="ack", reply_id=0x7F01, reply_name="ACK",
    )
    assert cmd.legacy is False
    assert cmd.notes == ""


def test_response_frozen() -> None:
    rsp = Response(name="Foo_RSP", id=0x8001, fields=(UINT8,))
    with pytest.raises((AttributeError, TypeError)):
        rsp.name = "Bar"  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Unsigned integer round-trips
# ---------------------------------------------------------------------------


def test_encode_uint8_zero() -> None:
    fields = (Field("x", 1, 0),)
    assert encode_payload(fields, {"x": 0}) == b"\x00"


def test_encode_uint8_max() -> None:
    fields = (Field("x", 1, 0),)
    assert encode_payload(fields, {"x": 255}) == b"\xff"


def test_decode_uint8() -> None:
    fields = (Field("x", 1, 0),)
    # Only field → last field → returns bytes
    result = decode_payload(fields, b"\x2a")
    assert result["x"] == b"\x2a"


def test_encode_uint16_little_endian() -> None:
    # 0x0102 in little-endian = bytes [0x02, 0x01]
    fields = (Field("x", 1, 0), Field("val", 2, 1))
    data = encode_payload(fields, {"x": 0, "val": 0x0102})
    assert data == b"\x00\x02\x01"


def test_roundtrip_uint8_then_vartail() -> None:
    fields = (UINT8, VARTAIL)
    values = {"u8": 42, "tail": b"hello"}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["u8"] == 42
    assert result["tail"] == b"hello"


def test_roundtrip_uint16_then_vartail() -> None:
    fields = (UINT16, VARTAIL)
    values = {"u16": 0xBEEF, "tail": b"\xca\xfe"}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["u16"] == 0xBEEF
    assert result["tail"] == b"\xca\xfe"


# ---------------------------------------------------------------------------
# Signed 16-bit integer — min/max round-trips
# ---------------------------------------------------------------------------


def test_roundtrip_signed16_min() -> None:
    """signed=True 16-bit field at minimum value −32768."""
    fields = (INT16, VARTAIL)
    values = {"s16": -32768, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["s16"] == -32768


def test_roundtrip_signed16_max() -> None:
    """signed=True 16-bit field at maximum value 32767."""
    fields = (INT16, VARTAIL)
    values = {"s16": 32767, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["s16"] == 32767


def test_roundtrip_signed16_zero() -> None:
    fields = (INT16, VARTAIL)
    values = {"s16": 0, "tail": b"x"}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["s16"] == 0


def test_roundtrip_signed16_negative_one() -> None:
    fields = (INT16, VARTAIL)
    values = {"s16": -1, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["s16"] == -1


# ---------------------------------------------------------------------------
# Signed 32-bit integer — min/max round-trips
# ---------------------------------------------------------------------------


def test_roundtrip_signed32_min() -> None:
    """signed=True 32-bit field at minimum value −2147483648."""
    fields = (INT32, VARTAIL)
    values = {"s32": -2_147_483_648, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["s32"] == -2_147_483_648


def test_roundtrip_signed32_max() -> None:
    """signed=True 32-bit field at maximum value 2147483647."""
    fields = (INT32, VARTAIL)
    values = {"s32": 2_147_483_647, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["s32"] == 2_147_483_647


# ---------------------------------------------------------------------------
# Q16 fixed-point round-trips
# ---------------------------------------------------------------------------


def test_q16_encode_1_5() -> None:
    """1.5 × 65536 = 98304 → 4 bytes little-endian signed."""
    fields = (Field("x", 1, 0), Q16, VARTAIL)
    encoded = encode_payload(fields, {"x": 0, "q16": 1.5, "tail": b""})
    # bytes [1:5] are the Q16 field; 98304 = 0x00018000
    q16_bytes = encoded[1:5]
    assert int.from_bytes(q16_bytes, "little", signed=True) == 98304
    assert q16_bytes == bytes([0x00, 0x80, 0x01, 0x00])


def test_q16_roundtrip_1_5() -> None:
    """Q16 field with value 1.5 encodes to 98304 and decodes back to 1.5."""
    fields = (Q16, VARTAIL)
    values = {"q16": 1.5, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["q16"] == pytest.approx(1.5)


def test_q16_roundtrip_negative_1_5() -> None:
    """Q16 field with value −1.5."""
    fields = (Q16, VARTAIL)
    values = {"q16": -1.5, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["q16"] == pytest.approx(-1.5)


def test_q16_roundtrip_zero() -> None:
    fields = (Q16, VARTAIL)
    values = {"q16": 0.0, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["q16"] == pytest.approx(0.0)


def test_q16_roundtrip_small_fraction() -> None:
    """Q16 value that is a multiple of 1/65536 round-trips exactly."""
    value = 3.0 / 65536.0  # smallest nonzero positive Q16 step
    fields = (Q16, VARTAIL)
    values = {"q16": value, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["q16"] == pytest.approx(value)


# ---------------------------------------------------------------------------
# Variable-length trailing field
# ---------------------------------------------------------------------------


def test_vartail_512_bytes() -> None:
    """Variable-length last field with 512 bytes round-trips exactly."""
    payload = bytes(range(256)) * 2  # 512 bytes
    fields = (UINT8, VARTAIL)
    values = {"u8": 7, "tail": payload}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["u8"] == 7
    assert result["tail"] == payload


def test_vartail_empty() -> None:
    """Variable-length last field with 0 bytes round-trips to b''."""
    fields = (UINT8, VARTAIL)
    values = {"u8": 255, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["u8"] == 255
    assert result["tail"] == b""


def test_vartail_str_encodes_as_ascii() -> None:
    """str value for trailing field is ASCII-encoded on encode."""
    fields = (UINT8, VARTAIL)
    encoded = encode_payload(fields, {"u8": 0, "tail": "DEKA"})
    assert encoded == b"\x00DEKA"


def test_vartail_str_roundtrip_via_bytes() -> None:
    """str encoding → decode returns bytes (not str)."""
    fields = (UINT8, VARTAIL)
    values_enc = {"u8": 1, "tail": "hello"}
    encoded = encode_payload(fields, values_enc)
    result = decode_payload(fields, encoded)
    assert result["tail"] == b"hello"


# ---------------------------------------------------------------------------
# Multi-field round-trips covering several types at once
# ---------------------------------------------------------------------------


def test_multi_field_roundtrip_all_types() -> None:
    """Round-trip: u8 + u16 + s16 + s32 + q16 + var-tail."""
    fields = (UINT8, UINT16, INT16, INT32, Q16, VARTAIL)
    values = {
        "u8": 0xAB,
        "u16": 0x1234,
        "s16": -1000,
        "s32": -999_999,
        "q16": 2.5,
        "tail": b"\xDE\xAD\xBE\xEF",
    }
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["u8"] == 0xAB
    assert result["u16"] == 0x1234
    assert result["s16"] == -1000
    assert result["s32"] == -999_999
    assert result["q16"] == pytest.approx(2.5)
    assert result["tail"] == b"\xDE\xAD\xBE\xEF"


def test_encode_empty_fields() -> None:
    assert encode_payload([], {}) == b""


def test_decode_empty_fields() -> None:
    assert decode_payload([], b"") == {}


# ---------------------------------------------------------------------------
# Error cases
# ---------------------------------------------------------------------------


def test_decode_too_short_raises() -> None:
    """decode_payload raises ValueError when data is too short for a fixed field."""
    fields = (INT32, VARTAIL)
    with pytest.raises(ValueError, match="too short"):
        decode_payload(fields, b"\x00\x00")  # only 2 bytes, need 4


def test_encode_overflow_unsigned_raises() -> None:
    """Encoding a value that overflows the field width raises OverflowError."""
    fields = (Field("x", 1, 0), VARTAIL)
    with pytest.raises(OverflowError):
        encode_payload(fields, {"x": 256, "tail": b""})


def test_encode_signed_underflow_raises() -> None:
    """Encoding a signed value below the minimum raises OverflowError."""
    fields = (INT16, VARTAIL)
    with pytest.raises(OverflowError):
        encode_payload(fields, {"s16": -32769, "tail": b""})


# ---------------------------------------------------------------------------
# Representative entries from protocol.json
# ---------------------------------------------------------------------------


def _load_protocol() -> dict:
    data = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    return json.loads(data)


def _make_field(fd: dict, offset_running: int) -> Field:
    """Build a Field from a protocol.json payload entry."""
    return Field(
        name=fd["name"],
        nbytes=fd["bytes"],
        offset=fd.get("offset", offset_running),
        signed=fd.get("signed", False),
        fixed_point=fd.get("fixed_point", None),
        unit=fd.get("unit", None),
        enum=fd.get("enum", None),
    )


def _fields_from_payload(payload: list) -> tuple[Field, ...]:
    offset = 0
    result = []
    for fd in payload:
        f = _make_field(fd, offset)
        result.append(f)
        offset += fd["bytes"]
    return tuple(result)


def _default_value(f: Field, idx: int, is_last: bool) -> object:
    """Return a reasonable default test value for a field."""
    if is_last:
        # Variable-length tail: use a short byte string.
        return b"\xAB\xCD"
    if f.fixed_point is not None:
        return 1.0
    if f.signed:
        return -1
    return idx  # small unsigned int


@pytest.mark.parametrize("cmd_name,payload", [
    (c["name"], c["payload"])
    for c in _load_protocol().get("commands", [])
    if c.get("payload")
][:15])  # first 15 commands with non-empty payloads
def test_command_payload_roundtrip(cmd_name: str, payload: list) -> None:
    """Round-trip encode/decode for representative command payloads."""
    fields = _fields_from_payload(payload)
    last_idx = len(fields) - 1
    values = {
        f.name: _default_value(f, i, i == last_idx)
        for i, f in enumerate(fields)
    }

    encoded = encode_payload(fields, values)
    decoded = decode_payload(fields, encoded)

    for i, f in enumerate(fields):
        if i == last_idx:
            # Last field always decoded as bytes.
            expected = values[f.name]
            if isinstance(expected, (bytes, bytearray)):
                assert decoded[f.name] == expected, (
                    f"{cmd_name}.{f.name}: tail mismatch"
                )
        elif f.fixed_point is not None:
            assert decoded[f.name] == pytest.approx(values[f.name]), (
                f"{cmd_name}.{f.name}: Q16 mismatch"
            )
        else:
            assert decoded[f.name] == values[f.name], (
                f"{cmd_name}.{f.name}: int mismatch"
            )


@pytest.mark.parametrize("rsp_name,payload", [
    (r["name"], r["payload"])
    for r in _load_protocol().get("responses", [])
    if r.get("payload")
][:15])  # first 15 responses with non-empty payloads
def test_response_payload_roundtrip(rsp_name: str, payload: list) -> None:
    """Round-trip encode/decode for representative response payloads."""
    fields = _fields_from_payload(payload)
    last_idx = len(fields) - 1
    values = {
        f.name: _default_value(f, i, i == last_idx)
        for i, f in enumerate(fields)
    }

    encoded = encode_payload(fields, values)
    decoded = decode_payload(fields, encoded)

    for i, f in enumerate(fields):
        if i == last_idx:
            expected = values[f.name]
            if isinstance(expected, (bytes, bytearray)):
                assert decoded[f.name] == expected, (
                    f"{rsp_name}.{f.name}: tail mismatch"
                )
        elif f.fixed_point is not None:
            assert decoded[f.name] == pytest.approx(values[f.name]), (
                f"{rsp_name}.{f.name}: Q16 mismatch"
            )
        else:
            assert decoded[f.name] == values[f.name], (
                f"{rsp_name}.{f.name}: int mismatch"
            )


# ---------------------------------------------------------------------------
# Specific RHSP command round-trips — hand-built, representative set
# ---------------------------------------------------------------------------


def test_set_single_dio_output() -> None:
    """SetSingleDIOOutput: dioPin(u8) + value(u8-last)."""
    fields = (
        Field("dioPin", 1, 0),
        Field("value", 1, 1),
    )
    encoded = encode_payload(fields, {"dioPin": 3, "value": 1})
    assert encoded == b"\x03\x01"
    decoded = decode_payload(fields, encoded)
    assert decoded["dioPin"] == 3
    assert decoded["value"] == b"\x01"  # last field → bytes


def test_set_motor_constant_power_signed() -> None:
    """SetMotorConstantPower: motorChannel(u8) + powerLevel(s16) — last is s16."""
    fields = (
        Field("motorChannel", 1, 0),
        Field("powerLevel", 2, 1, signed=True, range=(-32767, 32767)),
    )
    # Negative power level
    encoded = encode_payload(fields, {"motorChannel": 2, "powerLevel": -500})
    assert len(encoded) == 3
    decoded = decode_payload(fields, encoded)
    assert decoded["motorChannel"] == 2
    # Last field: raw bytes; verify the int interpretation
    raw = int.from_bytes(decoded["powerLevel"], "little", signed=True)
    assert raw == -500


def test_set_motor_constant_power_roundtrip_via_fields() -> None:
    """Round-trip: put powerLevel before a tail so it is not the last field."""
    fields = (
        Field("motorChannel", 1, 0),
        Field("powerLevel", 2, 1, signed=True),
        VARTAIL,
    )
    values = {"motorChannel": 0, "powerLevel": -32767, "tail": b""}
    result = decode_payload(fields, encode_payload(fields, values))
    assert result["motorChannel"] == 0
    assert result["powerLevel"] == -32767


def test_get_bulk_input_data_empty_payload() -> None:
    """GetBulkInputData has no payload fields."""
    fields: tuple[Field, ...] = ()
    assert encode_payload(fields, {}) == b""
    assert decode_payload(fields, b"") == {}


def test_query_interface_str_name() -> None:
    """QueryInterface: interfaceName is a str trailing field."""
    fields = (Field("interfaceName", 121, 0),)
    encoded = encode_payload(fields, {"interfaceName": "DEKA"})
    assert encoded == b"DEKA"
    decoded = decode_payload(fields, encoded)
    assert decoded["interfaceName"] == b"DEKA"


def test_set_motor_channel_mode() -> None:
    """SetMotorChannelMode: motorChannel + motorMode + floatAtZero."""
    fields = (
        Field("motorChannel", 1, 0),
        Field("motorMode", 1, 1, enum="MotorMode"),
        Field("floatAtZero", 1, 2, enum="ZeroPowerBehavior"),
    )
    values = {"motorChannel": 1, "motorMode": 1, "floatAtZero": 0}
    encoded = encode_payload(fields, values)
    assert encoded == b"\x01\x01\x00"
    decoded = decode_payload(fields, encoded)
    assert decoded["motorChannel"] == 1
    assert decoded["motorMode"] == 1
    # Last field → bytes
    assert decoded["floatAtZero"] == b"\x00"


def test_get_adc_with_channel() -> None:
    """GetADC: adcChannel(u8) + rawMode(u8-last)."""
    fields = (
        Field("adcChannel", 1, 0, enum="ADCChannel"),
        Field("rawMode", 1, 1),
    )
    encoded = encode_payload(fields, {"adcChannel": 7, "rawMode": 0})
    assert encoded == b"\x07\x00"


# ---------------------------------------------------------------------------
# Signed 16-bit and 32-bit extremes — explicit
# ---------------------------------------------------------------------------


def test_signed16_min_explicit() -> None:
    f = Field("v", 2, 0, signed=True)
    fields = (f, VARTAIL)
    values = {"v": -32768, "tail": b""}
    r = decode_payload(fields, encode_payload(fields, values))
    assert r["v"] == -32768


def test_signed16_max_explicit() -> None:
    f = Field("v", 2, 0, signed=True)
    fields = (f, VARTAIL)
    values = {"v": 32767, "tail": b""}
    r = decode_payload(fields, encode_payload(fields, values))
    assert r["v"] == 32767


def test_signed32_min_explicit() -> None:
    f = Field("v", 4, 0, signed=True)
    fields = (f, VARTAIL)
    values = {"v": -2_147_483_648, "tail": b""}
    r = decode_payload(fields, encode_payload(fields, values))
    assert r["v"] == -2_147_483_648


def test_signed32_max_explicit() -> None:
    f = Field("v", 4, 0, signed=True)
    fields = (f, VARTAIL)
    values = {"v": 2_147_483_647, "tail": b""}
    r = decode_payload(fields, encode_payload(fields, values))
    assert r["v"] == 2_147_483_647
