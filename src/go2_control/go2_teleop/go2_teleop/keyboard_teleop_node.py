# Copyright 2025 go2_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Keyboard teleop: WASD + QE -> geometry_msgs/Twist on /go2/teleop/cmd_vel.

`ros2 launch` and many tools leave stdin non-interactive, so we read keys from
`/dev/tty` when possible (controlling terminal), with stdin as fallback.

Velocity components use SI units (m/s for linear.x/y, rad/s for angular.z).

command_timeout_sec must exceed the terminal's initial key-repeat delay (often
~400–500 ms on Linux); otherwise holding a key publishes zeros between the
first keypress and the first auto-repeat, which makes the robot stutter.
"""

from __future__ import annotations

import atexit
import os
import select
import sys
import termios
import tty
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("go2_keyboard_teleop")

        self.declare_parameter("cmd_vel_topic", "/go2/teleop/cmd_vel")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.65)
        self.declare_parameter("max_linear_velocity_x", 0.4)
        self.declare_parameter("max_linear_velocity_y", 0.35)
        self.declare_parameter("max_angular_velocity_z", 1.0)

        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        rate = max(1.0, self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self._timeout = self.get_parameter("command_timeout_sec").get_parameter_value().double_value
        self._max_x = self.get_parameter("max_linear_velocity_x").get_parameter_value().double_value
        self._max_y = self.get_parameter("max_linear_velocity_y").get_parameter_value().double_value
        self._max_wz = self.get_parameter("max_angular_velocity_z").get_parameter_value().double_value

        self._pub = self.create_publisher(Twist, topic, 10)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_key_mono = time.monotonic()

        self._tty_fd: int | None = None
        self._input_fd: int | None = None
        self._old_settings = None
        self._tty_restored = False
        self._setup_tty()

        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'Publishing Twist (SI: m/s, rad/s) on "{topic}" — '
            f"max vx={self._max_x}, vy={self._max_y}, wz={self._max_wz}"
        )
        print(
            "\nGo2 keyboard teleop (Twist is SI units, not -1..1)\n"
            "  W/S  forward / back   (linear.x)\n"
            "  A/D  turn left / right (angular.z, CCW +)\n"
            "  Q/E  strafe left / right (linear.y)\n"
            "  Space / C / K  stop all\n"
            f"  Timeout {self._timeout}s after last key → zero cmd\n",
            file=sys.stderr,
        )

    def _open_tty(self) -> None:
        try:
            self._tty_fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            self.get_logger().warning("Could not open /dev/tty (%s); using stdin if available", e)
            self._tty_fd = None

    def _setup_tty(self) -> None:
        self._open_tty()
        if self._tty_fd is not None and os.isatty(self._tty_fd):
            self._input_fd = self._tty_fd
            self.get_logger().info("Reading keys from /dev/tty (works with ros2 launch in a terminal).")
        elif sys.stdin.isatty():
            self._input_fd = sys.stdin.fileno()
            self.get_logger().info("Reading keys from stdin.")
        else:
            self.get_logger().error(
                "No interactive TTY: stdin is not a TTY and /dev/tty failed. "
                "Run in a real terminal (e.g. `ros2 run go2_teleop keyboard_teleop_node`), "
                "or use Foxglove / another publisher on /go2/teleop/cmd_vel."
            )
            self._input_fd = None
            return

        self._old_settings = termios.tcgetattr(self._input_fd)
        tty.setcbreak(self._input_fd)
        atexit.register(self._reset_tty)

    def _reset_tty(self) -> None:
        if self._tty_restored:
            return
        self._tty_restored = True
        if self._old_settings is not None and self._input_fd is not None:
            try:
                termios.tcsetattr(self._input_fd, termios.TCSADRAIN, self._old_settings)
            except OSError:
                pass
        if self._tty_fd is not None:
            try:
                os.close(self._tty_fd)
            except OSError:
                pass
            self._tty_fd = None

    def _tick(self) -> None:
        if self._input_fd is not None:
            while select.select([self._input_fd], [], [], 0)[0]:
                try:
                    data = os.read(self._input_fd, 32)
                except BlockingIOError:
                    break
                except OSError:
                    break
                for b in data:
                    self._on_key(chr(b & 0x7F))
                self._last_key_mono = time.monotonic()

        if time.monotonic() - self._last_key_mono > self._timeout:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0

        msg = Twist()
        msg.linear.x = self._vx
        msg.linear.y = self._vy
        msg.angular.z = self._wz
        self._pub.publish(msg)

    def _on_key(self, ch: str) -> None:
        c = ch.lower()
        if c == "w":
            self._vx = abs(self._max_x)
        elif c == "s":
            self._vx = -abs(self._max_x)
        elif c == "a":
            self._wz = abs(self._max_wz)
        elif c == "d":
            self._wz = -abs(self._max_wz)
        elif c == "q":
            self._vy = abs(self._max_y)
        elif c == "e":
            self._vy = -abs(self._max_y)
        elif c in (" ", "c", "k"):
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._reset_tty()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
