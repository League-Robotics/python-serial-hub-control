---
title: Using the rhsp Python Driver
blurb: Install the rhsp package and drive REV Hub motors, servos, digital I/O, and sensors from Python.
order: 20
slug: rhsp-usage
tags: [rhsp, rev-hub, python, driver, usage]
updated: 2026-06-20
---

> **Source repository:** [League-Robotics/python-serial-hub-control](https://github.com/League-Robotics/python-serial-hub-control)
> This page is the task-oriented "how do I drive a hub" guide. For the wire
> protocol itself (framing, command catalogue, byte layouts) see the companion
> [REV Hub Serial Protocol](rhsp-protocol.md) reference.

# Using the `rhsp` Python Driver

`rhsp` is a pure-Python driver for the **REV Robotics Expansion Hub / Control
Hub**. It speaks the REV Hub Serial Protocol (RHSP / "DEKA") directly over the
hub's USB serial connection, so you can discover hubs and drive motors, servos,
digital I/O, analog inputs, the on-board IMU, and I2C sensors **without the FTC
SDK**. It is a library — there is no command-line tool; you drive the hub from
Python.

This guide is written so that an agent can read it and then run commands on a
real robot. If you only need one thing from this page, make it the
[heartbeat rule](#critical-keep-the-hub-alive).

## Requirements

- **Python ≥ 3.12** (3.12 is the ROS 2 system Python; 3.13 also works).
- A REV Expansion/Control Hub connected over **USB**.
- For anything that moves (motors, servos), the hub's **12 V battery must be
  connected and powered on**. USB alone powers the hub's logic and lets you
  read status, but actuators will not move on USB power.

## Where to get it / how to install

The package is published from the [source repository](https://github.com/League-Robotics/python-serial-hub-control),
not PyPI. Install the latest version directly from Git:

```bash
pip install "git+https://github.com/League-Robotics/python-serial-hub-control.git"
```

or, with [`uv`](https://docs.astral.sh/uv/):

```bash
uv pip install "git+https://github.com/League-Robotics/python-serial-hub-control.git"
```

The only runtime dependency is `pyserial`. Optional plotting extras (used by the
`velocity_chart` bench tool) come from the `bench` extra:

```bash
pip install "rhsp[bench] @ git+https://github.com/League-Robotics/python-serial-hub-control.git"
```

To work on the package itself, clone the repo and use `uv`:

```bash
git clone https://github.com/League-Robotics/python-serial-hub-control.git
cd python-serial-hub-control
uv sync --dev          # create .venv with runtime + dev (pytest) deps
uv run pytest tests/   # hardware-free suite (uses a software fake hub)
```

## How to connect

```python
import rhsp

# 1. Find attached hubs. Returns OS device paths whose USB serial number
#    starts with "D" (the REV Hub convention), e.g.
#    ["/dev/cu.usbserial-DQ3M375O"].  Empty list if none are attached.
ports = rhsp.enumerate_hubs()
if not ports:
    raise RuntimeError("No REV Hub found — is it plugged in?")

# 2. Connect.  Pass a port, or omit it to auto-pick the first hub found.
#    `with hub:` starts the keep-alive heartbeat and fail-safes on exit.
with rhsp.connect(ports[0]) as hub:
    hub.init_peripherals()          # bring up motor/servo channels
    print(hub.read_version_string())
    print("battery:", hub.battery_voltage_mv(), "mV")
```

`connect(port=None, *, baud=460_800, timeout=1.0, retries=3)` opens the serial
port, runs Discovery, and returns the parent [`Hub`](#the-hub-object). If the
hub is a daisy-chain parent, child hubs are available as `hub.children`.

### Critical: keep the hub alive

The hub **disables all outputs if it does not receive a valid packet within
2500 ms**. Always run the keep-alive heartbeat while you are driving anything.
The simplest and recommended way is to use the hub as a context manager:

```python
with rhsp.connect(ports[0]) as hub:   # starts a 2 s background heartbeat
    hub.init_peripherals()
    hub.motors[0].set_power(16000)
    time.sleep(10)                    # heartbeat keeps outputs live
# on exit: heartbeat stops and the hub is put into fail-safe automatically
```

If you cannot use `with`, call `hub.start_keepalive()` after connecting and
`hub.stop_keepalive()` / `hub.fail_safe()` when done. `hub.keep_alive()` sends a
single explicit ping if you are managing timing yourself.

## The `Hub` object

A connected `Hub` owns one device list per peripheral type. Indices match the
physical ports on the hub:

| Attribute      | Count | Index range | Device class |
|----------------|-------|-------------|--------------|
| `hub.motors`   | 4     | 0–3         | `Motor`      |
| `hub.servos`   | 6     | 0–5         | `Servo`      |
| `hub.dio`      | 8     | 0–7         | `DIOPin`     |
| `hub.adc`      | 4     | 0–3         | `ADCPin`     |
| `hub.i2c`      | 4     | 0–3         | `I2CChannel` |

Useful hub-level methods:

- `hub.init_peripherals()` — bring-up recipe: clears the connect-time
  `KeepAliveTimeout | FailSafe` latch, sets all motors to `CONSTANT_POWER` at 0,
  and configures all servos. Call this once after connecting, before driving
  anything.
- `hub.bulk_input()` — read all encoder/velocity/analog/mode state in one
  transaction; returns a `BulkInputData`.
- `hub.battery_voltage_mv()` / `hub.battery_current_ma()` — battery telemetry.
- `hub.read_version_string()` — firmware version, e.g. `"HW: 20, Maj: 1, Min: 8, Eng: 2"`.
- `hub.set_led_color(r, g, b)` / `hub.clear_led_color()` — module LED.
- `hub.fail_safe()` — immediately disable all outputs.

## Motors

Motor channels are `0–3`. Power is a signed level in `[-32767, 32767]`.

```python
motor = hub.motors[0]

# Open-loop power
motor.set_mode(rhsp.MotorMode.CONSTANT_POWER)
motor.enable()
motor.set_power(16000)            # ~half power forward; negative = reverse

# Closed-loop velocity (encoder counts / second)
motor.set_mode(rhsp.MotorMode.CONSTANT_VELOCITY)
motor.enable()
motor.set_target_velocity(800)
bulk = hub.bulk_input()           # one transaction snapshot
print("velocity:", motor.get_velocity(bulk))   # reads from the snapshot, no new I/O

# Closed-loop position
motor.set_mode(rhsp.MotorMode.POSITION_TARGET)
motor.set_target_position(2000, tolerance=20)
print("at target:", motor.at_target())

# Encoder + current
print("encoder:", motor.get_encoder_position())
motor.reset_encoder()
print("current:", motor.get_current_ma(), "mA")
```

`MotorMode` values: `CONSTANT_POWER`, `CONSTANT_VELOCITY`, `POSITION_TARGET`,
`CONSTANT_CURRENT`. Velocity PID can be tuned with
`motor.set_velocity_pid(p, i, d)` / `motor.get_velocity_pid()`.

> `get_velocity(bulk)` does **not** issue a new transaction — it reads the
> channel's velocity out of a `BulkInputData` snapshot you fetched with
> `hub.bulk_input()`. Fetch a fresh snapshot when you want current numbers.

## Servos

Servo channels are `0–5`. Configure the frame period once, then drive by angle
(0–180°) or by raw pulse width (500–2500 µs).

```python
servo = hub.servos[0]
servo.set_configuration(20_000)   # 50 Hz frame period (µs)
servo.enable()
servo.set_angle(90)               # degrees → pulse width internally
servo.set_pulse_width(1500)       # or drive the pulse width directly (µs)
servo.disable()
```

## Digital I/O

DIO pins are `0–7`. Set direction first (`True` = output), then write or read.

```python
pin = hub.dio[0]
pin.set_direction(True)           # output
pin.write(True)                   # drive high

sensor = hub.dio[1]
sensor.set_direction(False)       # input
print("reads:", sensor.read())    # True = logic high
```

## Sensors, ADC, and the LED

- **Bulk state:** `hub.bulk_input()` returns a `BulkInputData` with encoders,
  velocities, motor modes, and `analog_input0/1`.
- **I2C sensors:** drivers for the REV Color Sensor (V3) and the VL53L0X
  distance sensor live in `rhsp.sensors`; the on-board IMU has a driver there
  too. See [`examples/test_sensors.py`](https://github.com/League-Robotics/python-serial-hub-control/blob/main/examples/test_sensors.py).
- **LED:** `hub.set_led_color(r, g, b)` sets a solid colour and the background
  heartbeat re-asserts it automatically after each fail-safe re-latch — you do
  not need to re-send it. (Internally it uses `SetModuleLEDPattern`, because the
  simpler `SetModuleLEDColor` command is a no-op on firmware 1.8.2.)

## Complete runnable example

This connects, spins motor 0 up and back down, and always leaves the hub safe:

```python
import time
import rhsp

ports = rhsp.enumerate_hubs()
if not ports:
    raise SystemExit("No REV Hub found — is it plugged in and powered?")

with rhsp.connect(ports[0]) as hub:     # heartbeat on; fail-safe on exit
    hub.init_peripherals()
    hub.set_led_color(0, 255, 0)        # green = running

    motor = hub.motors[0]
    motor.set_mode(rhsp.MotorMode.CONSTANT_POWER)
    motor.enable()

    for step in range(6):               # ramp 0 → full power
        motor.set_power(int((step / 5) * 32767))
        time.sleep(0.3)
    motor.set_power(0)
# leaving the `with` block stops the heartbeat and fail-safes the hub
```

Run it with `python yourscript.py` (or `uv run python yourscript.py` in a repo
checkout). With no hub attached, `enumerate_hubs()` returns `[]` so the script
exits cleanly instead of hanging.

## Gotchas for agents

- **Heartbeat or nothing moves.** Without a keep-alive within 2500 ms the hub
  fail-safes. Use `with hub:` (or `start_keepalive()`); see
  [Critical: keep the hub alive](#critical-keep-the-hub-alive).
- **12 V battery for actuators.** Motors and servos need the battery connected
  and switched on. On USB power alone you can connect and read status, but
  nothing will move.
- **Always leave it safe.** The context manager fail-safes on exit. If you
  manage the connection manually, call `hub.fail_safe()` in a `finally` block.
- **Port identity.** `enumerate_hubs()` matches USB serial numbers starting
  with `"D"`. If your hub is not found, check the cable and that the OS sees the
  serial device (e.g. `/dev/cu.usbserial-…` on macOS, `/dev/ttyUSB…` on Linux).
- **snake_case API.** Methods are `set_power`, `init_peripherals`, etc. The old
  camelCase vendor code under `vendor/rhsp/` is reference only — do not use it.

## See also

- Runnable hardware examples: [`examples/`](https://github.com/League-Robotics/python-serial-hub-control/tree/main/examples)
  (`test_motor.py`, `test_servo.py`, `test_dio.py`, `test_sensors.py`). Each
  skips cleanly when no hub is attached.
- Wire-protocol reference: [REV Hub Serial Protocol](rhsp-protocol.md).
- Repository README: [League-Robotics/python-serial-hub-control](https://github.com/League-Robotics/python-serial-hub-control#readme).
