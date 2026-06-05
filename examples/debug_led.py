"""Debug LED color commands — prints raw wire bytes for each transaction."""
from __future__ import annotations

import time
import rhsp
from rhsp.framing import build_frame, FrameParser, checksum


def raw_led_test(port: str, hub_address: int) -> None:
    import serial
    ser = serial.Serial(port, baudrate=460800, bytesize=8, parity="N", stopbits=1, timeout=0.5)
    parser = FrameParser()

    msg_num = 1

    def send_recv(name: str, ptype: int, payload: bytes) -> bytes | None:
        nonlocal msg_num
        frame = build_frame(dest=hub_address, src=0, msg=msg_num, ref=0, ptype=ptype, payload=payload)
        msg_num = (msg_num % 255) + 1
        print(f"\n[{name}] SEND: {frame.hex()}")
        ser.write(frame)
        # Drain reply
        deadline = time.monotonic() + 1.0
        raw = b""
        while time.monotonic() < deadline:
            chunk = ser.read(64)
            if chunk:
                raw += chunk
                for pkt in parser.feed(chunk):
                    print(f"[{name}] RECV packet_type=0x{pkt.packet_type:04X} payload={pkt.payload.hex()} (raw_recv={raw.hex()})")
                    return pkt.payload
        print(f"[{name}] TIMEOUT — raw bytes so far: {raw.hex()}")
        return None

    # 1. KeepAlive to wake hub
    send_recv("KeepAlive", 0x7F04, b"")
    time.sleep(0.05)

    # 2. SetModuleLEDColor RED (R=255, G=0, B=0)
    send_recv("SetLEDColor RED", 0x7F0A, bytes([255, 0, 0]))
    time.sleep(0.2)
    print(">>> Look at the hub LED — is it RED?")
    time.sleep(3)

    # 3. GetModuleLEDColor
    payload = send_recv("GetLEDColor", 0x7F0B, b"")
    if payload is not None:
        print(f"  readback raw bytes: {payload.hex()}  (expected ff 00 00 for red)")

    # 4. SetModuleLEDColor GREEN (R=0, G=255, B=0)
    send_recv("SetLEDColor GREEN", 0x7F0A, bytes([0, 255, 0]))
    time.sleep(0.2)
    print(">>> Look at the hub LED — is it GREEN?")
    time.sleep(3)

    # 5. GetModuleLEDColor again
    payload = send_recv("GetLEDColor", 0x7F0B, b"")
    if payload is not None:
        print(f"  readback raw bytes: {payload.hex()}  (expected 00 ff 00 for green)")

    # 6. Try SetModuleLEDPattern with all-solid-green steps (corrected int encoding)
    #    Each step = (r, g, b, t) → encoded as little-endian uint32: [R, G, B, T]
    def make_solid_step(r: int, g: int, b: int, t: int = 50) -> bytes:
        val = (r & 0xFF) | ((g & 0xFF) << 8) | ((b & 0xFF) << 16) | ((t & 0xFF) << 24)
        return val.to_bytes(4, "little")

    # 16 steps all green
    pattern_payload = b"".join(make_solid_step(0, 255, 0) for _ in range(16))
    print(f"\n[SetLEDPattern solid-green] payload length: {len(pattern_payload)} bytes")
    send_recv("SetLEDPattern GREEN", 0x7F0C, pattern_payload)
    time.sleep(0.2)
    print(">>> Look at hub LED — is it SOLID GREEN (SetModuleLEDPattern working)?")
    time.sleep(5)

    # 7. All blue
    pattern_payload = b"".join(make_solid_step(0, 0, 255) for _ in range(16))
    send_recv("SetLEDPattern BLUE", 0x7F0C, pattern_payload)
    time.sleep(0.2)
    print(">>> Is it SOLID BLUE?")
    time.sleep(5)

    # 8. Try GetModuleLEDPattern — do we get a response?
    payload = send_recv("GetLEDPattern", 0x7F0D, b"")
    if payload is not None:
        print(f"  GetLEDPattern returned {len(payload)} bytes: {payload[:8].hex()}...")

    # 9. Off
    pattern_payload = b"".join(make_solid_step(0, 0, 0, 0) for _ in range(16))
    send_recv("SetLEDPattern OFF", 0x7F0C, pattern_payload)

    ser.close()


def main() -> None:
    ports = rhsp.enumerate_hubs()
    if not ports:
        print("No REV hub found.")
        return
    port = ports[0]
    print(f"Using port: {port}")

    # We need the hub address — discover it first using our normal library
    with rhsp.connect(port) as hub:
        addr = hub.address
        print(f"Hub address: {addr}")

    # Now run the raw test
    raw_led_test(port, addr)


if __name__ == "__main__":
    main()
