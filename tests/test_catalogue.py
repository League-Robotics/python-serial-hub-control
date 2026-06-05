"""Tests for rhsp.catalogue — JSON loader, legacy tagging, runtime_packet_id.

All tests are purely in-process; no hub hardware required.
"""

import pytest

from rhsp.catalogue import (
    COMMANDS,
    COUNTS,
    ENUMS,
    FRAMING,
    RESPONSES_BY_ID,
    command,
    deka_index,
    response_for,
    runtime_packet_id,
)
from rhsp.codec import Command, Field, Response, decode_payload, encode_payload


# ---------------------------------------------------------------------------
# Table size assertions
# ---------------------------------------------------------------------------


def test_commands_count() -> None:
    """Catalogue must have exactly 72 command entries."""
    assert len(COMMANDS) == 72


def test_responses_by_id_count() -> None:
    """Catalogue must have exactly 38 response entries."""
    assert len(RESPONSES_BY_ID) == 38


# ---------------------------------------------------------------------------
# Legacy tagging
# ---------------------------------------------------------------------------


def test_get_bulk_motor_data_is_legacy() -> None:
    """GetBulkMotorData: deka group, index 0x37 >= 0x31 → legacy=True."""
    assert COMMANDS["GetBulkMotorData"].legacy is True


def test_get_bulk_input_data_not_legacy() -> None:
    """GetBulkInputData: deka group, index 0x00 < 0x31 → legacy=False."""
    assert COMMANDS["GetBulkInputData"].legacy is False


def test_keep_alive_not_legacy() -> None:
    """KeepAlive: system group — legacy rule only applies to deka group."""
    assert COMMANDS["KeepAlive"].legacy is False


def test_get_adc_not_legacy() -> None:
    """GetADC: deka group, index 0x07 < 0x31 → legacy=False."""
    assert COMMANDS["GetADC"].legacy is False


def test_read_version_string_boundary() -> None:
    """ReadVersionString: deka group, index 0x30 — just below threshold, not legacy."""
    cmd = COMMANDS["ReadVersionString"]
    assert cmd.group == "deka"
    assert cmd.index == 0x30
    assert cmd.legacy is False


def test_get_bulk_pid_data_at_threshold() -> None:
    """GetBulkPIDData: deka group, index 0x31 == threshold → legacy=True."""
    cmd = COMMANDS["GetBulkPIDData"]
    assert cmd.group == "deka"
    assert cmd.index == 0x31
    assert cmd.legacy is True


# ---------------------------------------------------------------------------
# Field overlay: fixed_point (Q16)
# ---------------------------------------------------------------------------


def test_pid_coefficients_has_q16_field() -> None:
    """SetMotorPIDCoefficients must have a Field with fixed_point=65536."""
    cmd = COMMANDS["SetMotorPIDCoefficients"]
    q16_fields = [f for f in cmd.fields if f.fixed_point == 65536]
    assert len(q16_fields) >= 1, (
        "SetMotorPIDCoefficients must have at least one Q16 field"
    )


def test_pid_coefficients_q16_fields_are_signed() -> None:
    """Q16 fields in SetMotorPIDCoefficients are signed (per protocol spec)."""
    cmd = COMMANDS["SetMotorPIDCoefficients"]
    for f in cmd.fields:
        if f.fixed_point == 65536:
            assert f.signed is True, f"Q16 field {f.name!r} should be signed"


# ---------------------------------------------------------------------------
# Field overlay: signed
# ---------------------------------------------------------------------------


def test_set_motor_constant_power_has_signed_field() -> None:
    """SetMotorConstantPower must have a signed Field named 'powerLevel'."""
    cmd = COMMANDS["SetMotorConstantPower"]
    power_fields = [f for f in cmd.fields if f.name == "powerLevel"]
    assert power_fields, "SetMotorConstantPower must have a 'powerLevel' field"
    assert power_fields[0].signed is True


def test_set_motor_constant_power_range() -> None:
    """SetMotorConstantPower.powerLevel must carry range (-32767, 32767)."""
    cmd = COMMANDS["SetMotorConstantPower"]
    pf = next(f for f in cmd.fields if f.name == "powerLevel")
    assert pf.range == (-32767, 32767)


# ---------------------------------------------------------------------------
# runtime_packet_id
# ---------------------------------------------------------------------------


def test_runtime_packet_id_deka_command() -> None:
    """SetMotorConstantPower: deka, index 0x0F → deka_base + 0x0F."""
    cmd = COMMANDS["SetMotorConstantPower"]
    assert cmd.index == 0x0F
    assert runtime_packet_id(cmd, deka_base=0x1000) == 0x100F


def test_runtime_packet_id_system_command() -> None:
    """KeepAlive: system command → absolute id 0x7F04 regardless of base."""
    cmd = COMMANDS["KeepAlive"]
    assert runtime_packet_id(cmd, deka_base=0x1000) == 0x7F04


def test_runtime_packet_id_different_base() -> None:
    """DEKA id shifts with the base address."""
    cmd = COMMANDS["GetBulkInputData"]  # index 0x00
    assert runtime_packet_id(cmd, deka_base=0x2000) == 0x2000


def test_runtime_packet_id_legacy_command() -> None:
    """Legacy commands still get a correct runtime id."""
    cmd = COMMANDS["GetBulkMotorData"]  # index 0x37
    assert runtime_packet_id(cmd, deka_base=0x1000) == 0x1037


# ---------------------------------------------------------------------------
# Command accessor helpers
# ---------------------------------------------------------------------------


def test_command_accessor_returns_correct_entry() -> None:
    cmd = command("KeepAlive")
    assert cmd is COMMANDS["KeepAlive"]


def test_command_accessor_missing_raises_key_error() -> None:
    with pytest.raises(KeyError):
        command("NoSuchCommand")


def test_deka_index_returns_index() -> None:
    assert deka_index("GetBulkInputData") == 0x00
    assert deka_index("GetADC") == 0x07
    assert deka_index("GetBulkMotorData") == 0x37


def test_deka_index_system_command_raises() -> None:
    with pytest.raises(ValueError, match="not 'deka'"):
        deka_index("KeepAlive")


def test_response_for_typed_response() -> None:
    """response_for returns the Response descriptor for a response-kind command."""
    cmd = COMMANDS["GetADC"]
    rsp = response_for(cmd)
    assert isinstance(rsp, Response)
    assert rsp.id == cmd.reply_id


def test_response_for_ack() -> None:
    """response_for returns the ACK Response for an ack-kind command."""
    cmd = COMMANDS["SetSingleDIOOutput"]
    assert cmd.reply_kind == "ack"
    rsp = response_for(cmd)
    assert rsp.name == "ACK"


# ---------------------------------------------------------------------------
# Module-level reference sections present
# ---------------------------------------------------------------------------


def test_framing_populated() -> None:
    assert FRAMING, "FRAMING dict must be non-empty"


def test_enums_populated() -> None:
    assert ENUMS, "ENUMS dict must be non-empty"
    assert "MotorMode" in ENUMS


def test_counts_populated() -> None:
    assert COUNTS, "COUNTS dict must be non-empty"
    assert "motor_channels" in COUNTS


# ---------------------------------------------------------------------------
# Import idempotency
# ---------------------------------------------------------------------------


def test_import_idempotent() -> None:
    """Re-importing catalogue must return the same objects (not re-parse JSON)."""
    import importlib

    import rhsp.catalogue as cat1
    import rhsp.catalogue as cat2

    assert cat1.COMMANDS is cat2.COMMANDS
    assert cat1.RESPONSES_BY_ID is cat2.RESPONSES_BY_ID


# ---------------------------------------------------------------------------
# Codec round-trip via catalogue Field rows
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Full catalogue round-trip: every COMMAND's fields through encode/decode.
# ---------------------------------------------------------------------------
#
# Rules:
#   - Commands with no fields: encode({}) == b"" and decode(b"") == {}.
#   - For commands with fields: build a sample values dict using field-
#     appropriate min/max/zero values, encode, decode, assert equality.
#   - The last field is always decoded as bytes by the codec; non-last fields
#     round-trip as int or float.
#
# Note: encode_payload/decode_payload treat the *last* field as variable-length
# when it comes to decoding (it absorbs remaining bytes).  For encoding, the
# last field is only treated as variable-length if the value is bytes/str.
# Because we supply int/float for fixed-width fields, the encoder still writes
# the exact number of bytes, and the decoder reads them back as bytes.  We
# therefore re-interpret the last field's decoded value as an integer/float
# before comparing.

def _sample_value(f: Field, is_last: bool) -> object:
    """Return a test value for a field that round-trips cleanly."""
    if is_last:
        # Provide an int/float value — encoder writes fixed bytes; decoder
        # returns bytes for the last field.  We'll handle the comparison
        # in the test body.
        if f.fixed_point is not None:
            return 1.0
        if f.signed:
            return -1
        return 0
    if f.fixed_point is not None:
        return 1.0
    if f.signed:
        return -1
    return 0


@pytest.mark.parametrize("name,cmd", list(COMMANDS.items()))
def test_command_catalogue_roundtrip(name: str, cmd: Command) -> None:
    """Every command's Field rows must produce a valid encode/decode cycle."""
    fields = cmd.fields
    if not fields:
        assert encode_payload(fields, {}) == b""
        assert decode_payload(fields, b"") == {}
        return

    last_idx = len(fields) - 1
    values = {
        f.name: _sample_value(f, i == last_idx)
        for i, f in enumerate(fields)
    }

    encoded = encode_payload(fields, values)
    decoded = decode_payload(fields, encoded)

    for i, f in enumerate(fields):
        expected = values[f.name]
        actual = decoded[f.name]

        if i == last_idx:
            # Decoder returns bytes for the last field; re-interpret.
            assert isinstance(actual, bytes)
            if f.fixed_point is not None:
                actual_val = int.from_bytes(actual, "little", signed=True) / f.fixed_point
                assert actual_val == pytest.approx(float(expected)), (
                    f"{name}.{f.name}: Q16 last-field mismatch"
                )
            else:
                actual_int = int.from_bytes(actual, "little", signed=f.signed)
                assert actual_int == int(expected), (
                    f"{name}.{f.name}: int last-field mismatch"
                )
        elif f.fixed_point is not None:
            assert actual == pytest.approx(float(expected)), (
                f"{name}.{f.name}: Q16 mismatch"
            )
        else:
            assert actual == int(expected), f"{name}.{f.name}: int mismatch"


@pytest.mark.parametrize("rid,rsp", list(RESPONSES_BY_ID.items()))
def test_response_catalogue_roundtrip(rid: int, rsp: Response) -> None:
    """Every response's Field rows must produce a valid encode/decode cycle."""
    fields = rsp.fields
    if not fields:
        assert encode_payload(fields, {}) == b""
        assert decode_payload(fields, b"") == {}
        return

    last_idx = len(fields) - 1
    values = {
        f.name: _sample_value(f, i == last_idx)
        for i, f in enumerate(fields)
    }

    encoded = encode_payload(fields, values)
    decoded = decode_payload(fields, encoded)

    for i, f in enumerate(fields):
        expected = values[f.name]
        actual = decoded[f.name]

        if i == last_idx:
            assert isinstance(actual, bytes)
            if f.fixed_point is not None:
                actual_val = int.from_bytes(actual, "little", signed=True) / f.fixed_point
                assert actual_val == pytest.approx(float(expected)), (
                    f"{rsp.name}.{f.name}: Q16 last-field mismatch"
                )
            else:
                actual_int = int.from_bytes(actual, "little", signed=f.signed)
                assert actual_int == int(expected), (
                    f"{rsp.name}.{f.name}: int last-field mismatch"
                )
        elif f.fixed_point is not None:
            assert actual == pytest.approx(float(expected)), (
                f"{rsp.name}.{f.name}: Q16 mismatch"
            )
        else:
            assert actual == int(expected), f"{rsp.name}.{f.name}: int mismatch"
