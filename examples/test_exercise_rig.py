"""Exercise the League test rig — drive every part and report what it senses.

The rig:
  * two motors with encoders on ports 0 and 1
  * servo 1 swings a colour card in front of the colour + distance sensors
  * servo 0 swings a magnet past a magnetic switch (a digital input)
  * a colour sensor and a distance sensor on the I2C bus

Run it and you'll hear the motors and servos move while it verifies each part:

    uv run python examples/test_exercise_rig.py
"""

from __future__ import annotations

import time

import rhsp
from rhsp.sensors.color import ColorSensor
from rhsp.sensors.distance import Distance2m

# --- rig wiring -----------------------------------------------------------
MOTOR_PORTS = (0, 1)
ARM_SERVO = 1          # swings the colour card past the sensors
MAGNET_SERVO = 0       # swings the magnet past the magnetic switch
MAGNET_DIO = (0, 1)    # the magnetic switch reads low on these digital pins
COLOUR_CHANNEL = 3     # I2C channel for the colour sensor (hub port 4)
DISTANCE_CHANNEL = 2   # I2C channel for the distance sensor (hub port 3)
# TODO(api): hub.i2c[n].device(0x39) + ColorSensor(...) leaks the I2C address;
#            iterate toward hub.i2c[n].colour_sensor() / .distance_sensor().
COLOUR_I2C_ADDRESS = 0x39
DISTANCE_I2C_ADDRESS = 0x29

CYCLES = 3


def exercise_motors(hub: rhsp.Hub) -> None:
    print("motors:")
    for port in MOTOR_PORTS:
        motor = hub.motors[port]
        start = motor.get_encoder_position()
        motor.enable()
        for power in (9000, -9000):           # ~27 % forward then reverse
            motor.set_power(power)
            for _ in range(6):
                time.sleep(0.1)
        motor.set_power(0)
        motor.disable()
        end = motor.get_encoder_position()
        moved = "moved" if end != start else "NO MOVEMENT"
        print(f"  motor {port}: encoder {start} -> {end}  (delta {end - start}, {moved})")


def exercise_colour_card(hub: rhsp.Hub, colour, distance) -> None:
    print("arm servo (1) — colour card in front of the sensors:")
    arm = hub.servos[ARM_SERVO]
    arm.enable()
    for angle in (20, 160):
        arm.set_angle(angle)
        time.sleep(0.6)
        readings = []
        if colour is not None:
            try:
                r, g, b, c = colour.read_color()
                readings.append(f"colour rgb=({r},{g},{b}) clear={c}")
            except Exception as exc:
                readings.append(f"colour error: {exc}")
        if distance is not None:
            try:
                readings.append(f"distance={distance.read_mm()} mm")
            except Exception as exc:
                readings.append(f"distance error: {exc}")
        print(f"  angle {angle:3d}: " + "  ".join(readings) if readings else
              f"  angle {angle:3d}: (no sensors)")
    arm.disable()


def exercise_magnet(hub: rhsp.Hub) -> None:
    print("magnet servo (0) — magnetic switch:")
    for pin in MAGNET_DIO:
        hub.dio[pin].set_direction(output=False)
    magnet = hub.servos[MAGNET_SERVO]
    magnet.enable()
    tripped_at = None
    for angle in range(0, 181, 8):
        magnet.set_angle(angle)
        time.sleep(0.12)
        if any(not hub.dio[pin].read() for pin in MAGNET_DIO):
            tripped_at = angle
            break
    magnet.set_angle(90)
    magnet.disable()
    if tripped_at is not None:
        print(f"  switch tripped at ~{tripped_at} deg  (verified)")
    else:
        print("  switch did NOT trip across the sweep")


def setup_sensors(hub: rhsp.Hub):
    colour = distance = None
    try:
        colour = ColorSensor(hub.i2c[COLOUR_CHANNEL].device(COLOUR_I2C_ADDRESS))
        print("colour sensor: ready")
    except Exception as exc:
        print(f"colour sensor: unavailable ({exc})")
    try:
        distance = Distance2m(hub.i2c[DISTANCE_CHANNEL].device(DISTANCE_I2C_ADDRESS))
        print("distance sensor: ready" if distance.is_present() else
              "distance sensor: initialised but is_present() is False")
    except Exception as exc:
        print(f"distance sensor: unavailable ({exc})")
    return colour, distance


def main() -> None:
    ports = rhsp.enumerate_hubs()
    if not ports:
        print("No REV hub found — is it plugged in?")
        return
    with rhsp.connect(ports[0]) as hub:
        print(f"connected: {hub.read_version_string()}  (address {hub.address})")
        hub.init_peripherals()
        colour, distance = setup_sensors(hub)
        for cycle in range(1, CYCLES + 1):
            print(f"\n===== exercise cycle {cycle}/{CYCLES} =====")
            exercise_motors(hub)
            exercise_colour_card(hub, colour, distance)
            exercise_magnet(hub)
        for motor in hub.motors:
            motor.set_power(0)
            motor.disable()
        for servo in hub.servos:
            servo.disable()
        print("\nrig stopped + outputs disabled")


if __name__ == "__main__":
    main()
