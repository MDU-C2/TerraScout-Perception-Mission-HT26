#!/usr/bin/env python3
"""Simple keyboard teleop for TerraScout (publishes Twist to cmd_vel_teleop)."""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP_MSG = """
TerraScout Keyboard Teleop -> /terrascout1/cmd_vel_teleop
--------------------------------------------------------
   w / Up    : Forward
   s / Down  : Backward
   a / Left  : Turn Left
   d / Right : Turn Right
   Space / k : Stop
   + / -     : Increase / Decrease speed
   q         : Quit
"""


def get_key(timeout: float = 0.1) -> str:
    """Read a single keypress (including arrow escape sequences) without blocking."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return ""
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        # Read remaining arrow-key sequence if available
        if select.select([sys.stdin], [], [], 0.02)[0]:
            ch += sys.stdin.read(2)
    return ch


class KeyboardTeleop(Node):
    """Publishes TwistStamped velocity commands from keyboard input."""

    def __init__(self) -> None:
        super().__init__("terrascout_keyboard_teleop")
        self.declare_parameter("topic", "/terrascout1/cmd_vel_teleop")
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, topic, 10)
        self.linear_speed = 0.6
        self.angular_speed = 1.0

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP_MSG)
    print(f"Speed: linear={node.linear_speed:.2f} m/s, angular={node.angular_speed:.2f} rad/s")

    linear_x = 0.0
    angular_z = 0.0
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = get_key(0.1)
            if key in ("w", "W", "\x1b[A"):
                linear_x, angular_z = node.linear_speed, 0.0
            elif key in ("s", "S", "\x1b[B"):
                linear_x, angular_z = -node.linear_speed, 0.0
            elif key in ("a", "A", "\x1b[D"):
                linear_x, angular_z = 0.0, node.angular_speed
            elif key in ("d", "D", "\x1b[C"):
                linear_x, angular_z = 0.0, -node.angular_speed
            elif key in (" ", "k", "K"):
                linear_x, angular_z = 0.0, 0.0
            elif key in ("+", "="):
                node.linear_speed = min(1.5, node.linear_speed + 0.1)
                node.angular_speed = min(2.0, node.angular_speed + 0.1)
                sys.stdout.write(
                    f"\rSpeed: linear={node.linear_speed:.2f} m/s, angular={node.angular_speed:.2f} rad/s   \r\n"
                )
            elif key in ("-", "_"):
                node.linear_speed = max(0.1, node.linear_speed - 0.1)
                node.angular_speed = max(0.2, node.angular_speed - 0.1)
                sys.stdout.write(
                    f"\rSpeed: linear={node.linear_speed:.2f} m/s, angular={node.angular_speed:.2f} rad/s   \r\n"
                )
            elif key in ("q", "Q", "\x03"):
                break
            elif key == "":
                # Release-to-stop safety: stop if no key is held
                linear_x, angular_z = 0.0, 0.0

            node.publish_twist(linear_x, angular_z)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.publish_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

