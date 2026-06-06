"""Motor example — power sweep, velocity sweep, and HubVelocityController probe.

Requires a REV Hub attached via USB and powered on.  If no hub is found or
it is not responding, the test is automatically skipped so
``uv run pytest examples/`` exits 0 on hardware-free machines.

Usage (manual run)::

    uv run python -u examples/test_motor.py

Usage (pytest)::

    uv run pytest examples/test_motor.py -v
"""

from __future__ import annotations

import sys
import time

import pytest

import rhsp
from rhsp import MotorMode, RhspError
from rhsp.control import HubVelocityController


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


def test_hub_velocity_controller_probe() -> None:
    """GO/NO-GO probe for the hub onboard velocity PID via HubVelocityController.

    Constructs a HubVelocityController on channel 0, attaches it, commands a
    series of velocity targets, and reads back the measured velocity from
    GetBulkInputData.  Prints target vs measured for each step.

    This is the ticket 002 hardware validation gate.
    """
    hub = _connect_or_skip()
    hub.init_peripherals()

    ctrl = HubVelocityController(hub, channel=0)
    try:
        ctrl.attach()
    except RhspError as exc:
        pytest.skip(f"HubVelocityController.attach() failed: {exc}")

    targets = [300, 600, 900]
    try:
        hub.keep_alive()
        print("\n--- HubVelocityController probe (ticket 002) ---", flush=True)
        for target in targets:
            ctrl.command(target)
            time.sleep(0.5)  # allow firmware PID to settle
            hub.keep_alive()
            bulk = hub.bulk_input()
            measured = ctrl.measured(bulk)
            print(f"  target={target:6d}  measured={measured:6d} enc/s", flush=True)
        # Command zero and read one more time.
        ctrl.command(0)
        time.sleep(0.3)
        hub.keep_alive()
        bulk = hub.bulk_input()
        measured = ctrl.measured(bulk)
        print(f"  target={0:6d}  measured={measured:6d} enc/s  (stop)", flush=True)
        print("--- probe complete ---", flush=True)
    finally:
        ctrl.detach(disable=True)
        hub.fail_safe()


if __name__ == "__main__":
    hub = _connect_or_skip()
    test_motor_power_sweep()
    test_motor_velocity_sweep()
    test_hub_velocity_controller_probe()
