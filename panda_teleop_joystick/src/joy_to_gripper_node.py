#!/usr/bin/env python3

"""
Joy to Gripper Node
-------------------
Maps joystick buttons to a simple gripper open/close command.

This node:
- Subscribes to /joy
- Tracks a 1-DOF gripper opening value in [min_position, max_position]
- Publishes the current opening as std_msgs/Float64 on /gripper/position

The actual visualization is handled by trajectory_to_joint_states.py,
which uses /gripper/position to set finger_joint (and related joints) in /joint_states.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class JoyToGripper(Node):
    def __init__(self):
        super().__init__('joy_to_gripper_node')

        # Parameters (button indices follow servo_config.yaml joystick section by default)
        self.declare_parameter('button_open', 8)   # Triangle (DualShock3) / Y
        self.declare_parameter('button_close', 9)  # Cross     (DualShock3) / A
        self.declare_parameter('step', 0.02)       # Increment per press
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 0.8)  # From SRDF: finger_joint upper ≈ 0.8

        self.button_open = self.get_parameter('button_open').value
        self.button_close = self.get_parameter('button_close').value
        self.step = self.get_parameter('step').value
        self.min_position = self.get_parameter('min_position').value
        self.max_position = self.get_parameter('max_position').value

        # Internal state
        self.current_position = 0.0
        self._last_buttons = []

        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publisher
        self.gripper_pub = self.create_publisher(
            Float64,
            '/gripper/position',
            10
        )

        self.get_logger().info('Joy to Gripper node started')
        self.get_logger().info(f'button_open: {self.button_open}, button_close: {self.button_close}')
        self.get_logger().info(f'gripper range: [{self.min_position}, {self.max_position}], step: {self.step}')

        # Publish initial position
        self.publish_position()

    def joy_callback(self, msg: Joy):
        # Detect rising edges for buttons (press events)
        buttons = msg.buttons
        if not self._last_buttons:
            self._last_buttons = [0] * len(buttons)

        open_pressed = (
            self.button_open < len(buttons)
            and buttons[self.button_open] == 1
            and self._last_buttons[self.button_open] == 0
        )
        close_pressed = (
            self.button_close < len(buttons)
            and buttons[self.button_close] == 1
            and self._last_buttons[self.button_close] == 0
        )

        # NOTE: In SRDF/URDF, finger_joint = 0.0 is "open", 0.8 is "closed".
        # close_pressed → increase position toward max (닫기)
        # open_pressed  → decrease position toward min (열기)
        if close_pressed:
            self.current_position += self.step
        if open_pressed:
            self.current_position -= self.step

        # Clamp to [min_position, max_position]
        self.current_position = max(self.min_position, min(self.current_position, self.max_position))

        if open_pressed or close_pressed:
            self.get_logger().info(f'Gripper position command: {self.current_position:.3f}')
            self.publish_position()

        self._last_buttons = buttons

    def publish_position(self):
        msg = Float64()
        msg.data = float(self.current_position)
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToGripper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down joy_to_gripper_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

