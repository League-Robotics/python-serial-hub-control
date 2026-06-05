"""Sensor example — color, distance, and IMU reads via I2C.

Requires a REV Hub attached via USB with sensors wired to I2C channels 0 and 1.
If no hub is found or not responding, the test is automatically skipped so
``uv run pytest examples/`` exits 0 on hardware-free machines.

Wiring assumed:
  - I2C channel 0: REV 2m Distance Sensor (VL53L0X) at address 0x29
  - I2C channel 1: REV Color Sensor V3 (APDS-9151) at address 0x52
  - IMU: built-in (read via BulkInputData.imu_block)

Usage (manual run)::

    uv run python examples/test_sensors.py

Usage (pytest)::

    uv run pytest examples/test_sensors.py -v
"""

from __future__ import annotations

import time

import pytest

import rhsp
from rhsp import RhspError
from rhsp.sensors.color import ColorSensorV3
from rhsp.sensors.distance import Distance2m

# Standard I2C addresses for these sensors
_COLOR_ADDR = 0x52
_DISTANCE_ADDR = 0x29


def _connect_or_skip() -> rhsp.Hub:
    """Return a connected Hub or skip if no hub is available/responding."""
    ports = rhsp.enumerate_hubs()
    if not ports:
        pytest.skip("no hub found")
    try:
        return rhsp.connect(ports[0])
    except RhspError as exc:
        pytest.skip(f"hub not responding: {exc}")


def test_color_sensor() -> None:
    """Read RGBA values from a REV Color Sensor V3 on I2C channel 1."""
    hub = _connect_or_skip()
    i2c_dev = hub.i2c[1].device(_COLOR_ADDR)
    try:
        color = ColorSensorV3(i2c_dev)
    except RhspError as exc:
        pytest.skip(f"color sensor not available: {exc}")

    hub.keep_alive()
    for _ in range(5):
        r, g, b, clear = color.read_color()
        print(f"  color R={r:5d} G={g:5d} B={b:5d} clear={clear:5d}")
        time.sleep(0.25)


def test_distance_sensor() -> None:
    """Read range from a REV 2m Distance Sensor (VL53L0X) on I2C channel 0."""
    hub = _connect_or_skip()
    i2c_dev = hub.i2c[0].device(_DISTANCE_ADDR)
    try:
        dist = Distance2m(i2c_dev)
    except RhspError as exc:
        pytest.skip(f"distance sensor not available: {exc}")

    hub.keep_alive()
    for _ in range(5):
        mm = dist.read_mm()
        print(f"  distance {mm} mm")
        time.sleep(0.5)


def test_imu_bulk_read() -> None:
    """Read the IMU block from BulkInputData (built-in hub IMU)."""
    hub = _connect_or_skip()

    hub.keep_alive()
    for _ in range(5):
        bulk = hub.bulk_input()
        print(f"  imu_block={bulk.imu_block.hex()}  imu_status={bulk.imu_status}")
        time.sleep(0.25)


if __name__ == "__main__":
    _connect_or_skip()
    test_color_sensor()
    test_distance_sensor()
    test_imu_bulk_read()
