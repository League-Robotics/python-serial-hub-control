"""Tests for docs/generate_protocol_json.py — catalogue-to-JSON validator.

These tests assert that the inverted generator:
  1. Can be imported and builds the protocol document without error.
  2. The generated JSON matches the committed ``src/rhsp/protocol.json``
     byte-for-byte (i.e., no drift has been introduced).
  3. The generated output has the correct command and response counts.

No hardware access required.
"""

import importlib.resources
import importlib.util
import json
import os
import sys

import pytest


# ---------------------------------------------------------------------------
# Import the generator module dynamically (it lives in docs/, not a package).
# ---------------------------------------------------------------------------

_GENERATOR_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "docs",
    "generate_protocol_json.py",
)


def _load_generator():
    """Dynamically import docs/generate_protocol_json as a module."""
    spec = importlib.util.spec_from_file_location("generate_protocol_json", _GENERATOR_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def generator():
    """Return the loaded generator module (imported once per test session)."""
    return _load_generator()


@pytest.fixture(scope="module")
def generated_doc(generator):
    """Return the protocol document built by the generator."""
    return generator.build()


@pytest.fixture(scope="module")
def committed_doc():
    """Return the committed ``src/rhsp/protocol.json`` as a parsed dict."""
    raw = importlib.resources.files("rhsp").joinpath("protocol.json").read_bytes()
    return json.loads(raw)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_generator_imports_without_error(generator) -> None:
    """The generator module must be importable and expose a ``build`` callable."""
    assert callable(generator.build)


def test_generated_command_count(generated_doc) -> None:
    """Generated JSON must have 72 commands (same as the committed catalogue)."""
    assert len(generated_doc["commands"]) == 72


def test_generated_response_count(generated_doc) -> None:
    """Generated JSON must have 39 responses (including I2CConfigureQuery_RSP)."""
    assert len(generated_doc["responses"]) == 39


def test_no_drift_against_committed(generator, committed_doc) -> None:
    """Generated JSON must be semantically identical to the committed protocol.json.

    This is the CI gate: if it fails, the catalogue and committed JSON have
    drifted and one of them must be corrected.
    """
    generated_doc = generator.build()

    # Use the same normalisation as the generator itself.
    generated_text = generator._normalise(generated_doc)
    committed_text = generator._normalise(committed_doc)

    diff = generator._diff(generated_text, committed_text)
    assert not diff, (
        "generate_protocol_json output does not match src/rhsp/protocol.json:\n"
        + "".join(diff[:60])
        + (f"\n... ({len(diff) - 60} more lines)" if len(diff) > 60 else "")
    )


def test_commands_names_match(generated_doc, committed_doc) -> None:
    """Command names and ids must agree exactly."""
    gen_cmds = {c["name"]: c["id"] for c in generated_doc["commands"]}
    com_cmds = {c["name"]: c["id"] for c in committed_doc["commands"]}
    assert gen_cmds == com_cmds


def test_response_names_match(generated_doc, committed_doc) -> None:
    """Response names and ids must agree exactly."""
    gen_rsps = {r["name"]: r["id"] for r in generated_doc["responses"]}
    com_rsps = {r["name"]: r["id"] for r in committed_doc["responses"]}
    assert gen_rsps == com_rsps


def test_i2c_configure_query_rsp_present(generated_doc) -> None:
    """I2CConfigureQuery_RSP must appear in responses (ticket-008 fix)."""
    names = {r["name"] for r in generated_doc["responses"]}
    assert "I2CConfigureQuery_RSP" in names


def test_get_pwn_pulse_width_rsp_is_2_bytes(generated_doc) -> None:
    """GetPWNPulseWidth_RSP.pulseWidth must be 2 bytes (ticket-008 fix)."""
    rsp = next(
        r for r in generated_doc["responses"] if r["name"] == "GetPWNPulseWidth_RSP"
    )
    pulse_field = next(f for f in rsp["payload"] if f["name"] == "pulseWidth")
    assert pulse_field["bytes"] == 2
