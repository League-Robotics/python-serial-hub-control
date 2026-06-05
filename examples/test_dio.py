"""DIO example — set direction and read/write digital pins.

Requires a REV Hub attached via USB and powered on.  If no hub is found or
it is not responding, the test is automatically skipped so
``uv run pytest examples/`` exits 0 on hardware-free machines.

Usage (manual run)::

    uv run python examples/test_dio.py

Usage (pytest)::

    uv run pytest examples/test_dio.py -v
"""

from __future__ import annotations

import time

import pytest

import rhsp
from rhsp import RhspError


def _connect_or_skip() -> rhsp.Hub:
    """Return a connected Hub or skip if no hub is available/responding."""
    ports = rhsp.enumerate_hubs()
    if not ports:
        pytest.skip("no hub found")
    try:
        return rhsp.connect(ports[0])
    except RhspError as exc:
        pytest.skip(f"hub not responding: {exc}")


def test_dio_direction_and_readwrite() -> None:
    """Configure DIO pins as outputs/inputs; write outputs; read inputs.

    Pins 0–3 are configured as output (write); pins 4–7 remain as input (read).
    """
    hub = _connect_or_skip()

    output_pins = hub.dio[:4]
    input_pins = hub.dio[4:]

    for pin in output_pins:
        pin.set_direction(True)   # output
    for pin in input_pins:
        pin.set_direction(False)  # input

    # Allow the hub a moment to apply the direction changes.
    time.sleep(0.05)

    try:
        hub.keep_alive()
        for cycle in range(5):
            value = bool(cycle % 2)
            try:
                for pin in output_pins:
                    pin.write(value)
            except RhspError as exc:
                pytest.skip(f"DIO write not available: {exc}")
            try:
                readings = [pin.read() for pin in input_pins]
            except RhspError as exc:
                pytest.skip(f"DIO read not available: {exc}")
            print(f"  cycle={cycle} wrote={int(value)} to pins 0-3  read pins 4-7={readings}")
            time.sleep(0.2)
    finally:
        hub.fail_safe()


if __name__ == "__main__":
    _connect_or_skip()
    test_dio_direction_and_readwrite()
