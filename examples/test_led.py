"""LED fire-and-forget validation example.

Verifies that after a single ``hub.set_led_color()`` call, the LED holds the
requested solid colour for >= 10 s with NO caller re-sends, and that the LED
is non-blinking immediately after ``init_peripherals()``.

Requires a REV Hub attached at /dev/cu.usbserial-DQ3M375O (module @2, fw 1.8.2)
with 12 V battery connected.  If the port is not found the test is skipped so
``uv run pytest examples/`` exits 0 on hardware-free machines.

Usage (manual run)::

    uv run python -u examples/test_led.py

Usage (pytest)::

    uv run pytest examples/test_led.py -v -s
"""

from __future__ import annotations

import time

import pytest

import rhsp

_PORT = "/dev/cu.usbserial-DQ3M375O"
_MODULE_ADDRESS = 2


def _connect_or_skip() -> rhsp.Hub:
    """Return a connected Hub or skip if the target port is not present."""
    import serial  # type: ignore[import-untyped]
    try:
        # Check the port is at least openable before we hand it to rhsp.
        s = serial.Serial(_PORT, timeout=0.1)
        s.close()
    except Exception:
        pytest.skip(f"Hub port {_PORT} not available")

    try:
        return rhsp.connect(_PORT)
    except Exception as exc:
        pytest.skip(f"Could not connect to hub: {exc}")


def test_led_holds_colour_for_10s() -> None:
    """LED must remain solid green for 10 s after a single set_led_color() call.

    Validation criteria (ticket 002-001):
    - After init_peripherals(), LED is NOT blinking blue (status latch cleared).
    - After set_led_color(0, 255, 0), LED stays green for >= 10 s without
      any manual re-sends.
    """
    hub = _connect_or_skip()

    with hub:
        hub.init_peripherals()

        # At this point the LED should already be solid (not blinking blue)
        # because init_peripherals() cleared the latch.
        print("\n[LED test] After init_peripherals(): LED should be non-blinking (check visually).", flush=True)
        time.sleep(1.0)

        # Set green — fire-and-forget.
        hub.set_led_color(0, 255, 0)
        print("[LED test] set_led_color(0, 255, 0) called ONCE. LED should be solid green.", flush=True)

        # Hold for 10+ seconds without any re-send; heartbeat fires every 2 s.
        for i in range(11):
            print(f"[LED test] t={i:2d}s — LED should still be solid green (no re-send)", flush=True)
            time.sleep(1.0)

        print("[LED test] 10 s elapsed. Clearing LED pattern.", flush=True)
        hub.clear_led_color()


if __name__ == "__main__":
    import logging
    logging.basicConfig(level=logging.WARNING)

    hub = rhsp.connect(_PORT)
    print(f"Connected to hub: {hub!r}", flush=True)

    with hub:
        hub.init_peripherals()
        print("init_peripherals() done — LED should be non-blinking now.", flush=True)
        time.sleep(1.0)

        hub.set_led_color(0, 255, 0)
        print("set_led_color(green) called ONCE — heartbeat will re-assert.", flush=True)

        for i in range(12):
            print(f"t={i:2d}s  LED should be solid green", flush=True)
            time.sleep(1.0)

        print("Clearing stored pattern.", flush=True)
        hub.clear_led_color()
        time.sleep(3.0)
        print("Done — LED should have reverted to blinking blue.", flush=True)
