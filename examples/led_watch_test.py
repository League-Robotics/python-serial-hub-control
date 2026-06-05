"""Definitive LED test using the VENDOR (old) code path.

Run it, then watch the hub LED. It holds each colour for 8 seconds with a
continuous keep-alive so the hub never drops to its idle status pattern.

    uv run --with pyserial --directory vendor python ../examples/led_watch_test.py

(or just `uv run python examples/led_watch_test.py` once vendor is importable)
"""
import sys
import time

sys.path.insert(0, "/Volumes/Proj/proj/RobotProjects/SerialHubControl/vendor")

from rhsp.client import Client
from rhsp.rshp_serial import comPort
from rhsp.internal.messages import LEDPattern


def solid(r, g, b, t=50):
    p = LEDPattern()
    for i in range(15):
        p.set_step(i, r, g, b, t)
    return p


def hold(c, dest, label, pattern, seconds=8):
    print(f"\n>>> {label} — for {seconds}s. WATCH THE HUB LED NOW.", flush=True)
    c.keepAlive(dest)            # SDK hack: KA immediately before pattern
    c.setModuleLEDPattern(dest, pattern)
    t_end = time.time() + seconds
    while time.time() < t_end:
        c.keepAlive(dest)
        time.sleep(0.4)


def main():
    c = Client()
    ports = comPort.enumerate()
    c.open(ports[0])
    c.discovery()
    dest = 2
    c.keepAlive(dest)

    print("=" * 60)
    print("LED TEST STARTING. Get your eyes on the hub.")
    print("Counting down...", flush=True)
    for n in (5, 4, 3, 2, 1):
        print(f"  {n}...", flush=True)
        c.keepAlive(dest)
        time.sleep(1)

    hold(c, dest, "FULL RED",    solid(255, 0, 0))
    hold(c, dest, "FULL GREEN",  solid(0, 255, 0))
    hold(c, dest, "FULL BLUE",   solid(0, 0, 255))
    hold(c, dest, "FULL WHITE",  solid(255, 255, 255))
    # Fast strobe: alternate white/off via a multi-step pattern
    strobe = LEDPattern()
    for i in range(15):
        strobe.set_step(i, 255, 255, 255, 3) if i % 2 == 0 else strobe.set_step(i, 0, 0, 0, 3)
    hold(c, dest, "WHITE STROBE", strobe, seconds=8)

    c.close()
    print("\nDONE. (Hub reverts to its idle pattern once keep-alive stops.)", flush=True)


if __name__ == "__main__":
    main()
