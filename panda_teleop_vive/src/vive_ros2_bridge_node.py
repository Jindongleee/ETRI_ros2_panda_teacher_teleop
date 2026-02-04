#!/usr/bin/env python3
"""
Vive ROS2 Bridge Node
Subscribes to vive_ros2 controller_data (VRControllerData) and republishes
PoseStamped and Joy so panda_teleop_vive (vive_to_twist, vive_to_gripper) can use them.

- Subscribe: /controller_data (vive_ros2/msg/VRControllerData)
- Publish:   /vive/controller/pose (geometry_msgs/PoseStamped)
             /vive/controller/buttons (sensor_msgs/Joy)

Controller role: 0 = right, 1 = left (only the selected controller is forwarded).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy

# VRControllerData is from vive_ros2 (build dependency)
from vive_ros2.msg import VRControllerData


class ViveRos2Bridge(Node):
    def __init__(self):
        super().__init__('vive_ros2_bridge_node')

        self.declare_parameter('controller_data_topic', 'controller_data')
        self.declare_parameter('pose_out_topic', '/vive/controller/pose')
        self.declare_parameter('buttons_out_topic', '/vive/controller/buttons')
        self.declare_parameter('controller_role', 0)  # 0 = right, 1 = left

        controller_data_topic = self.get_parameter('controller_data_topic').value
        pose_out_topic = self.get_parameter('pose_out_topic').value
        buttons_out_topic = self.get_parameter('buttons_out_topic').value
        role = self.get_parameter('controller_role').value
        self.controller_role = int(role) if isinstance(role, str) else role

        self.controller_data_sub = self.create_subscription(
            VRControllerData,
            controller_data_topic,
            self.controller_data_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, pose_out_topic, 10)
        self.buttons_pub = self.create_publisher(Joy, buttons_out_topic, 10)

        self.get_logger().info(
            f'Bridge: {controller_data_topic} -> pose={pose_out_topic}, buttons={buttons_out_topic}, role={self.controller_role}'
        )

    def controller_data_callback(self, msg: VRControllerData):
        if msg.role != self.controller_role:
            return

        # TransformStamped -> PoseStamped (use abs_pose)
        t = msg.abs_pose
        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation
        self.pose_pub.publish(pose)

        # VRControllerData -> Joy
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = 'vive_controller'
        joy.axes = [float(msg.trigger), msg.trackpad_x, msg.trackpad_y]
        joy.buttons = [
            1 if msg.trigger_button else 0,
            1 if msg.grip_button else 0,
            1 if msg.menu_button else 0,
            1 if msg.trackpad_touch else 0,
            1 if msg.trackpad_button else 0,
        ]
        self.buttons_pub.publish(joy)


def main(args=None):
    rclpy.init(args=args)
    node = ViveRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
