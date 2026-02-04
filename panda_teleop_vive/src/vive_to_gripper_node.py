#!/usr/bin/env python3
"""
Vive to Gripper Node
Subscribes to Vive controller buttons (sensor_msgs/Joy).
Maps trigger/grip (or configurable buttons) to gripper open/close.
Publishes std_msgs/Float64 on /gripper/position.

Topics:
  - Subscribe: /vive/controller/buttons (sensor_msgs/Joy)
  - Publish:   /gripper/position (std_msgs/Float64)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class ViveToGripper(Node):
    def __init__(self):
        super().__init__('vive_to_gripper_node')

        self.declare_parameter('button_open', 2)
        self.declare_parameter('button_close', 0)
        self.declare_parameter('step', 0.02)
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 0.8)
        self.declare_parameter('vive_buttons_topic', '/vive/controller/buttons')

        self.button_open = self.get_parameter('button_open').value
        self.button_close = self.get_parameter('button_close').value
        self.step = self.get_parameter('step').value
        self.min_position = self.get_parameter('min_position').value
        self.max_position = self.get_parameter('max_position').value
        self.vive_buttons_topic = self.get_parameter('vive_buttons_topic').value

        self.current_position = 0.0
        self._last_buttons = []

        self.buttons_sub = self.create_subscription(
            Joy,
            self.vive_buttons_topic,
            self.buttons_callback,
            10
        )
        self.gripper_pub = self.create_publisher(Float64, '/gripper/position', 10)

        self.get_logger().info(
            f'Vive to Gripper: topic={self.vive_buttons_topic}, open={self.button_open}, close={self.button_close}'
        )
        self._publish_position()

    def _publish_position(self):
        msg = Float64()
        msg.data = self.current_position
        self.gripper_pub.publish(msg)

    def buttons_callback(self, msg: Joy):
        buttons = msg.buttons
        if not self._last_buttons:
            self._last_buttons = [0] * max(len(buttons), 1)

        open_pressed = (
            self.button_open < len(buttons)
            and buttons[self.button_open] == 1
            and (self._last_buttons[self.button_open] if self.button_open < len(self._last_buttons) else 0) == 0
        )
        close_pressed = (
            self.button_close < len(buttons)
            and buttons[self.button_close] == 1
            and (self._last_buttons[self.button_close] if self.button_close < len(self._last_buttons) else 0) == 0
        )

        if open_pressed:
            self.current_position = min(self.max_position, self.current_position + self.step)
        if close_pressed:
            self.current_position = max(self.min_position, self.current_position - self.step)

        self._last_buttons = list(buttons) if buttons else [0]
        self._publish_position()


def main(args=None):
    rclpy.init(args=args)
    node = ViveToGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
