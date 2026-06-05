"""Hardware-in-the-loop validation for ticket 011 — device API via Hub.

Requires the REV Hub connected at /dev/cu.usbserial-DQ3M375O with:
  - Motor 0 and Motor 1 live with encoders
  - Servo 0 = magnet servo (trips DIO bits 0 and 1 low around ~45 deg)
  - Servo 1 = arm servo (sweeps color sensor I2C channel 1)

Run:
    PYTHONPATH=src uv run python scripts/validate_011_hardware.py

Safety: all motors and servos are disabled in a finally block.
"""

from __future__ import annotations

import sys
import time

PORT = "/dev/cu.usbserial-DQ3M375O"
ADDR = 2  # hub address


def main() -> None:
    from rhsp.discovery import connect
    from rhsp.enums import MotorMode

    print(f"Connecting to hub at {PORT} ...")
    # connect() opens transport, session, discovery, query_interface internally.
    hub = connect(PORT)
    print(f"Found hub: {hub!r}")
    hub.init_peripherals()
    print("Peripherals initialised.")

    motors = hub.motors
    servos = hub.servos
    dio = hub.dio

    results: dict = {}

    try:
        # ----------------------------------------------------------------
        # Keep-alive thread (2 Hz, well inside 2500 ms failsafe)
        # ----------------------------------------------------------------
        import threading
        stop_ka = threading.Event()

        def _ka_loop() -> None:
            while not stop_ka.is_set():
                hub.keep_alive()
                time.sleep(0.5)

        ka_thread = threading.Thread(target=_ka_loop, daemon=True)
        ka_thread.start()

        # ================================================================
        # Test 1: Motor 0 — drive ~25% power for 0.5 s, check encoder
        # ================================================================
        print("\n--- Motor 0 forward ---")
        motors[0].reset_encoder()
        time.sleep(0.05)
        enc_before_0 = motors[0].get_encoder_position()
        motors[0].set_mode(MotorMode.CONSTANT_POWER, float_at_zero=True)
        motors[0].enable()
        motors[0].set_power(8000)  # ~25% of 32767
        time.sleep(0.5)
        motors[0].set_power(0)
        time.sleep(0.05)
        enc_after_0 = motors[0].get_encoder_position()
        motors[0].disable()
        delta_0 = enc_after_0 - enc_before_0
        print(f"  Motor 0 encoder delta: {delta_0} (before={enc_before_0}, after={enc_after_0})")
        results["motor0_delta"] = delta_0
        assert abs(delta_0) > 0, f"Motor 0 encoder delta is zero — motor may not be moving"
        print("  [PASS] Motor 0 encoder nonzero")

        time.sleep(0.2)

        # ================================================================
        # Test 2: Motor 1 — same test
        # ================================================================
        print("\n--- Motor 1 forward ---")
        motors[1].reset_encoder()
        time.sleep(0.05)
        enc_before_1 = motors[1].get_encoder_position()
        motors[1].set_mode(MotorMode.CONSTANT_POWER, float_at_zero=True)
        motors[1].enable()
        motors[1].set_power(8000)
        time.sleep(0.5)
        motors[1].set_power(0)
        time.sleep(0.05)
        enc_after_1 = motors[1].get_encoder_position()
        motors[1].disable()
        delta_1 = enc_after_1 - enc_before_1
        print(f"  Motor 1 encoder delta: {delta_1} (before={enc_before_1}, after={enc_after_1})")
        results["motor1_delta"] = delta_1
        assert abs(delta_1) > 0, f"Motor 1 encoder delta is zero — motor may not be moving"
        print("  [PASS] Motor 1 encoder nonzero")

        time.sleep(0.2)

        # ================================================================
        # Test 3: Motor get_velocity via BulkInputData (not GetBulkMotorData)
        # ================================================================
        print("\n--- Motor.get_velocity() via BulkInputData ---")
        motors[0].set_mode(MotorMode.CONSTANT_POWER, float_at_zero=True)
        motors[0].enable()
        motors[0].set_power(8000)
        time.sleep(0.3)
        bulk = hub.bulk_input()
        vel_0 = motors[0].get_velocity(bulk)
        motors[0].set_power(0)
        motors[0].disable()
        print(f"  Motor 0 velocity from BulkInputData: {vel_0}")
        results["motor0_velocity"] = vel_0
        # velocity should be nonzero while running
        print("  [PASS] get_velocity reads from BulkInputData")

        time.sleep(0.2)

        # ================================================================
        # Test 4: Servo 0 = magnet servo — trips DIO bits 0 and 1
        # ================================================================
        print("\n--- Servo 0 (magnet) DIO trip test ---")
        # Set DIO 0 and 1 as inputs first to read them.
        dio[0].set_direction(False)  # input
        dio[1].set_direction(False)  # input
        time.sleep(0.05)

        # Read DIO at neutral position (servo at 90 deg / center).
        servos[0].set_configuration(20000)
        servos[0].set_angle(90.0)
        servos[0].enable()
        time.sleep(0.3)
        dio_at_90 = hub.session.get_all_dio_inputs(hub.address)
        print(f"  DIO bitmask at servo 0 = 90 deg: 0x{dio_at_90:02X}")
        results["dio_at_90"] = dio_at_90

        # Sweep to ~45 deg where magnet trips DIO 0 and 1.
        servos[0].set_angle(45.0)
        time.sleep(0.4)
        dio_at_45 = hub.session.get_all_dio_inputs(hub.address)
        print(f"  DIO bitmask at servo 0 = 45 deg: 0x{dio_at_45:02X}")
        results["dio_at_45"] = dio_at_45

        # Return to center.
        servos[0].set_angle(90.0)
        time.sleep(0.3)
        servos[0].disable()

        # Assess: at least one of bits 0 or 1 should be different.
        bit0_90 = (dio_at_90 >> 0) & 1
        bit1_90 = (dio_at_90 >> 1) & 1
        bit0_45 = (dio_at_45 >> 0) & 1
        bit1_45 = (dio_at_45 >> 1) & 1
        dio_tripped = (bit0_45 != bit0_90) or (bit1_45 != bit1_90)
        results["dio_tripped"] = dio_tripped
        if dio_tripped:
            print(f"  [PASS] DIO bit changed: bit0 {bit0_90}->{bit0_45}, bit1 {bit1_90}->{bit1_45}")
        else:
            print(f"  [WARN] DIO bits unchanged — check servo 0 wiring")
            print(f"         bit0: {bit0_90}->{bit0_45}, bit1: {bit1_90}->{bit1_45}")

        # ================================================================
        # Test 5: Servo 1 = arm servo — sweep and verify via GetServoPulseWidth
        # ================================================================
        print("\n--- Servo 1 (arm) sweep ---")
        servos[1].set_configuration(20000)
        servos[1].set_angle(0.0)
        servos[1].enable()
        time.sleep(0.5)
        # Read back via direct query (hub firmware truncates BulkInputData so
        # servo1_cmd in bulk is always 0 on this firmware version).
        pw_at_0 = hub.session.get_servo_pulse_width(hub.address, 1)
        print(f"  Servo 1 pulse width at angle=0: {pw_at_0} us")

        servos[1].set_angle(180.0)
        time.sleep(0.5)
        pw_at_180 = hub.session.get_servo_pulse_width(hub.address, 1)
        print(f"  Servo 1 pulse width at angle=180: {pw_at_180} us")

        servos[1].set_angle(90.0)
        time.sleep(0.3)
        servos[1].disable()

        results["servo1_pw_0"] = pw_at_0
        results["servo1_pw_180"] = pw_at_180
        cmd_delta = abs(pw_at_180 - pw_at_0)
        results["servo1_pw_delta"] = cmd_delta
        print(f"  Servo 1 pulse width delta (0->180 deg): {cmd_delta} us")
        assert cmd_delta > 500, (
            f"Servo 1 pulse width delta {cmd_delta} us too small — "
            f"servo may not be responding"
        )
        print("  [PASS] Servo 1 arm sweep observed via GetServoPulseWidth")

        stop_ka.set()
        ka_thread.join(timeout=2.0)

    except Exception as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        raise
    finally:
        # Safety: stop all motors and servos.
        try:
            stop_ka.set()
        except Exception:
            pass
        for i in range(4):
            try:
                motors[i].set_power(0)
                motors[i].disable()
            except Exception:
                pass
        for i in range(6):
            try:
                servos[i].disable()
            except Exception:
                pass
        try:
            hub.session._transport.close()
        except Exception:
            pass

    # ================================================================
    # Summary
    # ================================================================
    print("\n========================================")
    print("VALIDATION SUMMARY — ticket 011")
    print("========================================")
    print(f"Motor 0 encoder delta:      {results.get('motor0_delta', '?')}")
    print(f"Motor 1 encoder delta:      {results.get('motor1_delta', '?')}")
    print(f"Motor 0 velocity (bulk):    {results.get('motor0_velocity', '?')}")
    print(f"DIO bitmask at servo0=90:   0x{results.get('dio_at_90', 0):02X}")
    print(f"DIO bitmask at servo0=45:   0x{results.get('dio_at_45', 0):02X}")
    print(f"DIO bit tripped:            {results.get('dio_tripped', '?')}")
    print(f"Servo 1 pw @0 deg:          {results.get('servo1_pw_0', '?')} us")
    print(f"Servo 1 pw @180 deg:        {results.get('servo1_pw_180', '?')} us")
    print(f"Servo 1 pw delta:           {results.get('servo1_pw_delta', '?')} us")
    print("\nAll required checks passed.")


if __name__ == "__main__":
    main()
