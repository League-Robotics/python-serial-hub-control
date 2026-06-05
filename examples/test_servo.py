"""Servo example — angle sweep from 0° to 180° and back.

Requires a REV Hub attached via USB and powered on.  If no hub is found or
it is not responding, the test is automatically skipped so
``uv run pytest examples/`` exits 0 on hardware-free machines.

Usage (manual run)::

    uv run python examples/test_servo.py

Usage (pytest)::

    uv run pytest examples/test_servo.py -v
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


def test_servo_sweep() -> None:
    """Sweep servo 0 from 0° to 180° then back."""
    hub = _connect_or_skip()
    hub.init_peripherals()

    servo = hub.servos[0]
    servo.set_configuration(20_000)  # 50 Hz frame period
    servo.enable()

    angles = list(range(0, 181, 10)) + list(range(180, -1, -10))
    try:
        hub.keep_alive()
        for angle in angles:
            servo.set_angle(angle)
            print(f"  servo[0] angle={angle:3d}°")
            time.sleep(0.05)
    finally:
        servo.disable()
        hub.fail_safe()


if __name__ == "__main__":
    _connect_or_skip()
    test_servo_sweep()
