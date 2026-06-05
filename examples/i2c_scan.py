"""Quick I2C bus scanner — print every address that responds and its first register.

Run with:  uv run python examples/i2c_scan.py
"""

from __future__ import annotations

import rhsp

CHANNELS = (0, 1)          # I2C channels to scan
ADDR_RANGE = range(0x08, 0x78)   # 7-bit addresses

# NOTE: on channels with SDA stuck low, each probe causes a hub-side I2C timeout
# before returning NACK 44.  The scan will be slow on those channels (~1-5 s/address).
# Do not run concurrently with any other process holding the hub's serial port.


def scan_channel(hub: rhsp.Hub, channel: int) -> None:
    print(f"\n--- I2C channel {channel} ---")
    found = 0
    for addr in ADDR_RANGE:
        try:
            data = hub.i2c[channel].device(addr).read_register(0x00, 1)
            print(f"  0x{addr:02X}: responded — reg[0x00]=0x{data[0]:02X}")
            found += 1
        except Exception:
            pass
    if found == 0:
        print("  (no devices responded)")


def main() -> None:
    ports = rhsp.enumerate_hubs()
    if not ports:
        print("No REV hub found.")
        return
    with rhsp.connect(ports[0]) as hub:
        print(f"connected: {hub.read_version_string()}")
        for ch in CHANNELS:
            scan_channel(hub, ch)


if __name__ == "__main__":
    main()
