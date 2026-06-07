#!/usr/bin/env python3
"""rig_teleop_node — ROS 2 joystick teleop + sensor publisher for the REV hub rig.

Subscribes to a ``sensor_msgs/Joy`` topic and drives the rig via the ``rhsp``
driver:

* **Left stick** → wheels:  X axis → motor 0 power, Y axis → motor 1 power.
* **Right stick** → servos: X axis → servo 0 angle, Y axis → servo 1 angle.

and publishes the rig's sensor state as a JSON blob on a ``std_msgs/String``
topic (default ``/nepr/rig/sensors``):

    {"stamp": <float>, "magnet": bool, "touch": bool,
     "distance_mm": int|null, "color": {"r":..,"g":..,"b":..,"clear":..}|null}

Motors are open-loop power teleop (axis → power). A dead-man safety stops the
motors if no Joy message arrives within ``joy_timeout`` seconds.

Run on the robot (after the package is installed into a Python that also has
rclpy):

    source /opt/ros/<distro>/setup.bash
    python3 rig_teleop_node.py --ros-args \
        -p joy_topic:=/nepr/joy/joystick0 \
        -p axis_motor0:=0 -p axis_motor1:=1 \
        -p axis_servo0:=2 -p axis_servo1:=3

All axis indices, inversions, ports, scaling, and rates are ROS parameters so
the mapping can be calibrated without editing code.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import rhsp
from rhsp.enums import MotorMode
from rhsp.sensors.color import ColorSensor
from rhsp.sensors.distance import Distance2m


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class RigTeleop(Node):
    def __init__(self) -> None:
        super().__init__("rig_teleop")

        # ---- parameters -------------------------------------------------
        p = self.declare_parameter
        self.port = p("port", "").value or None
        self.joy_topic = p("joy_topic", "/nepr/joy/joystick0").value
        self.sensors_topic = p("sensors_topic", "/nepr/rig/sensors").value

        # axis index mapping (calibrate per controller) + inversion
        self.axis_motor0 = int(p("axis_motor0", 0).value)   # left stick X
        self.axis_motor1 = int(p("axis_motor1", 1).value)   # left stick Y
        self.axis_servo0 = int(p("axis_servo0", 2).value)   # right stick X
        self.axis_servo1 = int(p("axis_servo1", 3).value)   # right stick Y
        self.inv_motor0 = float(p("invert_motor0", 1.0).value)
        self.inv_motor1 = float(p("invert_motor1", 1.0).value)
        self.inv_servo0 = float(p("invert_servo0", 1.0).value)
        self.inv_servo1 = float(p("invert_servo1", 1.0).value)

        self.deadzone = float(p("deadzone", 0.08).value)
        self.max_power = int(p("max_power", 18000).value)    # of 32767
        self.servo_center = float(p("servo_center_deg", 90.0).value)
        self.servo_span = float(p("servo_span_deg", 90.0).value)  # +/- from center

        # hub wiring (matches the League rig)
        self.motor_channels = (int(p("motor0_ch", 0).value), int(p("motor1_ch", 1).value))
        self.servo_channels = (int(p("servo0_ch", 0).value), int(p("servo1_ch", 1).value))
        self.magnet_dios = tuple(int(x) for x in p("magnet_dios", [0, 1]).value)
        self.touch_dio = int(p("touch_dio", 2).value)
        self.color_ch = int(p("color_ch", 3).value)
        self.color_addr = int(p("color_addr", 0x39).value)
        self.distance_ch = int(p("distance_ch", 2).value)
        self.distance_addr = int(p("distance_addr", 0x29).value)

        self.control_rate = float(p("control_rate_hz", 20.0).value)
        self.sensor_rate = float(p("sensor_rate_hz", 5.0).value)
        self.joy_timeout = float(p("joy_timeout_s", 0.5).value)

        # ---- connect + bring up the hub ---------------------------------
        port = self.port or (rhsp.enumerate_hubs() or [None])[0]
        if port is None:
            raise RuntimeError("No REV hub found (set the 'port' parameter)")
        self.get_logger().info(f"connecting to hub on {port}")
        self.hub = rhsp.connect(port)
        self.hub.init_peripherals()
        self.hub.start_keepalive()

        for ch in self.motor_channels:
            self.hub.motors[ch].set_mode(MotorMode.CONSTANT_POWER)
            self.hub.motors[ch].enable()
        for ch in self.servo_channels:
            self.hub.servos[ch].enable()
        for pin in (*self.magnet_dios, self.touch_dio):
            try:
                self.hub.dio[pin].set_direction(output=False)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"dio[{pin}] setup failed: {exc}")

        # sensors (optional — keep running if absent)
        self.color = self._try(lambda: ColorSensor(self.hub.i2c[self.color_ch].device(self.color_addr)), "color")
        self.distance = self._try(lambda: Distance2m(self.hub.i2c[self.distance_ch].device(self.distance_addr)), "distance")

        # ---- ROS wiring -------------------------------------------------
        self._last_axes: list[float] = []
        self._last_joy_t = 0.0
        self._last_servo_cmd: dict[int, float] = {}
        # best-effort sensor QoS: a best-effort subscriber matches BOTH
        # best-effort and reliable Joy publishers (reliable-only subs miss
        # best-effort publishers — a common "subscribed but nothing arrives").
        self.create_subscription(Joy, self.joy_topic, self._on_joy, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, self.sensors_topic, 10)
        self.create_timer(1.0 / self.control_rate, self._control_tick)
        self.create_timer(1.0 / self.sensor_rate, self._sensor_tick)
        self.get_logger().info(
            f"teleop ready: joy={self.joy_topic} sensors={self.sensors_topic} "
            f"motors={self.motor_channels} servos={self.servo_channels}"
        )

    def _try(self, ctor, name):
        try:
            obj = ctor()
            self.get_logger().info(f"{name} sensor: ready")
            return obj
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"{name} sensor unavailable: {exc}")
            return None

    # ---- joystick ------------------------------------------------------
    def _on_joy(self, msg: Joy) -> None:
        self._last_axes = list(msg.axes)
        self._last_joy_t = self.get_clock().now().nanoseconds / 1e9

    def _axis(self, idx: int, inv: float) -> float:
        if idx < 0 or idx >= len(self._last_axes):
            return 0.0
        v = self._last_axes[idx] * inv
        return 0.0 if abs(v) < self.deadzone else _clamp(v, -1.0, 1.0)

    def _control_tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        stale = (now - self._last_joy_t) > self.joy_timeout or not self._last_axes
        # motors
        for ch, ai, inv in (
            (self.motor_channels[0], self.axis_motor0, self.inv_motor0),
            (self.motor_channels[1], self.axis_motor1, self.inv_motor1),
        ):
            power = 0 if stale else int(self._axis(ai, inv) * self.max_power)
            try:
                self.hub.motors[ch].set_power(power)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"motor[{ch}] set_power failed: {exc}")
        if stale:
            return
        # servos (write only on meaningful change to spare the bus)
        for ch, ai, inv in (
            (self.servo_channels[0], self.axis_servo0, self.inv_servo0),
            (self.servo_channels[1], self.axis_servo1, self.inv_servo1),
        ):
            angle = self.servo_center + self._axis(ai, inv) * self.servo_span
            angle = _clamp(angle, 0.0, 180.0)
            if abs(self._last_servo_cmd.get(ch, -999) - angle) >= 1.5:
                self._last_servo_cmd[ch] = angle
                try:
                    self.hub.servos[ch].set_angle(int(round(angle)))
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"servo[{ch}] set_angle failed: {exc}")

    # ---- sensors -------------------------------------------------------
    def _sensor_tick(self) -> None:
        state: dict = {"stamp": self.get_clock().now().nanoseconds / 1e9}
        # magnet: switch pulls a DIO low when tripped
        try:
            state["magnet"] = any(not self.hub.dio[pin].read() for pin in self.magnet_dios)
        except Exception:  # noqa: BLE001
            state["magnet"] = None
        # touch: active-low on its DIO (report raw level too for clarity)
        try:
            level = self.hub.dio[self.touch_dio].read()
            state["touch"] = (not level)
        except Exception:  # noqa: BLE001
            state["touch"] = None
        # distance
        if self.distance is not None:
            try:
                state["distance_mm"] = self.distance.read_mm()
            except Exception:  # noqa: BLE001
                state["distance_mm"] = None
        else:
            state["distance_mm"] = None
        # color
        if self.color is not None:
            try:
                r, g, b, c = self.color.read_color()
                state["color"] = {"r": r, "g": g, "b": b, "clear": c}
            except Exception:  # noqa: BLE001
                state["color"] = None
        else:
            state["color"] = None
        self.pub.publish(String(data=json.dumps(state)))

    # ---- shutdown ------------------------------------------------------
    def shutdown(self) -> None:
        try:
            self.hub.stop_keepalive()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.hub.fail_safe()
        except Exception:  # noqa: BLE001
            pass


def main() -> None:
    rclpy.init()
    node = RigTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
