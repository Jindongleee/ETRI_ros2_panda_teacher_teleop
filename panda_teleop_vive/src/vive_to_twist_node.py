#!/usr/bin/env python3
"""
Vive to Twist Node
Subscribes to Vive controller pose (PoseStamped) and optional buttons (Joy).
Publishes TwistStamped for MoveIt Servo. Pose delta over time is converted to velocity.

Topics:
  - Subscribe: /vive/controller/pose (geometry_msgs/PoseStamped)
               /vive/controller/buttons (sensor_msgs/Joy, optional deadman)
               /clutch/active (std_msgs/Bool, optional)
  - Publish:   /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import math


class ViveToTwist(Node):
    def __init__(self):
        super().__init__('vive_to_twist_node')

        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 0.3)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('use_clutch', True)
        self.declare_parameter('vive_pose_topic', '/vive/controller/pose')
        self.declare_parameter('vive_buttons_topic', '/vive/controller/buttons')
        self.declare_parameter('deadman_button', 1)
        # VR→Panda 축 매핑 (Vive와 Panda가 마주보는 배치: Vive 왼쪽→Panda X, Vive 앞→Panda Z, Vive 위→Panda Y)
        # VR: X=오른쪽, Y=위, Z=뒤 → invert_linear_x=-1, invert_linear_z=-1, invert_linear_y=1
        self.declare_parameter('invert_linear_x', -1.0)
        self.declare_parameter('invert_linear_y', 1.0)
        self.declare_parameter('invert_linear_z', 1.0)
        self.declare_parameter('invert_angular_x', -1.0)
        self.declare_parameter('invert_angular_y', -1.0)
        self.declare_parameter('invert_angular_z', 1.0)

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_linear_z = self.get_parameter('invert_linear_z').value
        self.invert_angular_x = self.get_parameter('invert_angular_x').value
        self.invert_angular_y = self.get_parameter('invert_angular_y').value
        self.invert_angular_z = self.get_parameter('invert_angular_z').value
        self.frame_id = self.get_parameter('frame_id').value
        self.use_clutch = self.get_parameter('use_clutch').value
        self.vive_pose_topic = self.get_parameter('vive_pose_topic').value
        self.vive_buttons_topic = self.get_parameter('vive_buttons_topic').value
        self.deadman_button = self.get_parameter('deadman_button').value

        self.clutch_active = False
        self.deadman_pressed = False
        self.prev_pose = None
        self.prev_time = None

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.vive_pose_topic,
            self.pose_callback,
            10
        )
        self.buttons_sub = self.create_subscription(
            Joy,
            self.vive_buttons_topic,
            self.buttons_callback,
            10
        )
        if self.use_clutch:
            self.clutch_sub = self.create_subscription(
                Bool,
                '/clutch/active',
                self.clutch_callback,
                10
            )

        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        self.get_logger().info(
            f'Vive to Twist: pose={self.vive_pose_topic}, buttons={self.vive_buttons_topic}, use_clutch={self.use_clutch}'
        )

    def clutch_callback(self, msg: Bool):
        self.clutch_active = msg.data

    def buttons_callback(self, msg: Joy):
        if self.deadman_button < len(msg.buttons):
            self.deadman_pressed = msg.buttons[self.deadman_button] == 1

    def pose_callback(self, msg: PoseStamped):
        control_enabled = self.clutch_active if self.use_clutch else self.deadman_pressed
        now = self.get_clock().now()
        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        if control_enabled and self.prev_pose is not None and self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt > 0.0 and dt < 1.0:
                p = msg.pose.position
                q = msg.pose.orientation
                p0 = self.prev_pose.position
                q0 = self.prev_pose.orientation
                vx = (p.x - p0.x) / dt * self.linear_scale
                vy = (p.y - p0.y) / dt * self.linear_scale
                vz = (p.z - p0.z) / dt * self.linear_scale
                twist.twist.linear.y = vx * self.invert_linear_x
                twist.twist.linear.z = vy * self.invert_linear_y
                twist.twist.linear.x = vz * self.invert_linear_z
                self._quat_to_angular(q0, q, dt, twist)

                max_lin = max(abs(twist.twist.linear.x), abs(twist.twist.linear.y), abs(twist.twist.linear.z))
                max_ang = max(abs(twist.twist.angular.x), abs(twist.twist.angular.y), abs(twist.twist.angular.z))
                if max_lin > 1.0 or max_ang > 1.0:
                    s = max(max_lin, max_ang)
                    twist.twist.linear.x /= s
                    twist.twist.linear.y /= s
                    twist.twist.linear.z /= s
                    twist.twist.angular.x /= s
                    twist.twist.angular.y /= s
                    twist.twist.angular.z /= s

        self.prev_pose = msg.pose
        self.prev_time = now
        self.twist_pub.publish(twist)

    def _quat_to_angular(self, q0, q1, dt, twist):
        """Approximate angular velocity from quaternion delta."""
        if dt <= 0:
            return
        q0x, q0y, q0z, q0w = q0.x, q0.y, q0.z, q0.w
        q1x, q1y, q1z, q1w = q1.x, q1.y, q1.z, q1.w
        qdx = q1x - q0x
        qdy = q1y - q0y
        qdz = q1z - q0z
        qdw = q1w - q0w
        wx = 2.0 * (-q0w * qdx + q0x * qdw + q0z * qdy - q0y * qdz) / dt
        wy = 2.0 * (-q0w * qdy + q0y * qdw + q0x * qdz - q0z * qdx) / dt
        wz = 2.0 * (-q0w * qdz + q0z * qdw + q0y * qdx - q0x * qdy) / dt
        twist.twist.angular.y = wx * self.angular_scale * self.invert_angular_x
        twist.twist.angular.z = wy * self.angular_scale * self.invert_angular_y
        twist.twist.angular.x = wz * self.angular_scale * self.invert_angular_z


def main(args=None):
    rclpy.init(args=args)
    node = ViveToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
