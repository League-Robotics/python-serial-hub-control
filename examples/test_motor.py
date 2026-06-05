"""Motor example — power sweep and velocity sweep.

Requires a REV Hub attached via USB and powered on.  If no hub is found or
it is not responding, the test is automatically skipped so
``uv run pytest examples/`` exits 0 on hardware-free machines.

Usage (manual run)::

    uv run python examples/test_motor.py

Usage (pytest)::

    uv run pytest examples/test_motor.py -v
"""

from __future__ import annotations

import time

import pytest

import rhsp
from rhsp import MotorMode, RhspError


def _connect_or_skip() -> rhsp.Hub:
    """Return a connected Hub or skip if no hub is available/responding."""
    ports = rhsp.enumerate_hubs()
    if not ports:
        pytest.skip("no hub found")
    try:
        return rhsp.connect(ports[0])
    except RhspError as exc:
        pytest.skip(f"hub not responding: {exc}")


def test_motor_power_sweep() -> None:
    """Sweep motor 0 through a range of power levels then back to zero."""
    hub = _connect_or_skip()
    hub.init_peripherals()

    motor = hub.motors[0]
    motor.set_mode(MotorMode.CONSTANT_POWER)
    motor.enable()

    max_power = 2**15 - 1
    steps = 5
    try:
        hub.keep_alive()
        for i in range(steps + 1):
            power = int((i / steps) * max_power)
            motor.set_power(power)
            print(f"  motor[0] power={power}")
            time.sleep(0.3)
        motor.set_power(0)
    finally:
        hub.fail_safe()


def test_motor_velocity_sweep() -> None:
    """Sweep motor 0 through closed-loop velocity targets."""
    hub = _connect_or_skip()
    hub.init_peripherals()

    motor = hub.motors[0]
    motor.set_mode(MotorMode.CONSTANT_VELOCITY)
    try:
        motor.enable()
    except RhspError as exc:
        pytest.skip(f"motor velocity mode not available: {exc}")

    try:
        hub.keep_alive()
        for target in range(0, 1500, 300):
            motor.set_target_velocity(target)
            bulk = hub.bulk_input()
            velocity = motor.get_velocity(bulk)
            print(f"  target={target:6d}  measured={velocity:6d} enc/s")
            time.sleep(0.5)
        motor.set_target_velocity(0)
    finally:
        hub.fail_safe()


if __name__ == "__main__":
    hub = _connect_or_skip()
    test_motor_power_sweep()
    test_motor_velocity_sweep()
