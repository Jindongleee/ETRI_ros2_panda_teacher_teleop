#!/usr/bin/env python3

"""
Omy L100 to Twist Node
-----------------------
Converts omy_l100 end-effector motion to TwistStamped commands for panda servo.

This node:
- Subscribes to /leader/joint_states (omy_l100 joint states)
- Uses TF to compute end-effector pose via forward kinematics
- Computes velocity from pose difference
- Publishes TwistStamped to /servo_node/delta_twist_cmds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import numpy as np
import math


class OmyL100ToTwist(Node):
    def __init__(self):
        super().__init__('omy_l100_to_twist_node')

        # Parameters
        self.declare_parameter('base_frame', 'leader_link0')  # omy_l100 base frame
        self.declare_parameter('ee_frame', 'leader_link7')  # omy_l100 end-effector frame
        self.declare_parameter('target_frame', 'panda_link0')  # panda base frame (for coordinate transform)
        self.declare_parameter('joint_states_topic', '/leader/joint_states')
        self.declare_parameter('linear_scale', 1.0)  # Scale factor for linear velocity
        self.declare_parameter('angular_scale', 1.0)  # Scale factor for angular velocity
        self.declare_parameter('publish_rate', 50.0)  # Hz

        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        publish_rate = self.get_parameter('publish_rate').value

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Previous pose for velocity calculation
        self.last_pose = None
        self.last_time = None

        # Subscriber
        self.joint_states_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10
        )

        # Publisher
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        # Timer for periodic publishing (if needed)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info('Omy L100 to Twist node started')
        self.get_logger().info(f'Subscribing to: {joint_states_topic}')
        self.get_logger().info(f'Base frame: {self.base_frame}, EE frame: {self.ee_frame}')
        self.get_logger().info(f'Publishing to: /servo_node/delta_twist_cmds')

    def joint_state_callback(self, msg: JointState):
        """Compute twist from joint states using TF"""
        try:
            # Wait for transform from base_frame to ee_frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Get current pose
            current_pose = PoseStamped()
            current_pose.header.frame_id = self.base_frame
            current_pose.header.stamp = transform.header.stamp
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation

            # Compute velocity from pose difference
            current_time = self.get_clock().now()

            if self.last_pose is not None and self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds * 1e-9
                if dt > 0.0 and dt < 1.0:  # Valid time difference
                    # Linear velocity
                    dx = current_pose.pose.position.x - self.last_pose.pose.position.x
                    dy = current_pose.pose.position.y - self.last_pose.pose.position.y
                    dz = current_pose.pose.position.z - self.last_pose.pose.position.z

                    # Angular velocity (from quaternion difference)
                    # Compute relative rotation: q_rel = q_curr * q_prev^-1
                    q_prev = np.array([
                        self.last_pose.pose.orientation.w,
                        self.last_pose.pose.orientation.x,
                        self.last_pose.pose.orientation.y,
                        self.last_pose.pose.orientation.z
                    ])
                    q_curr = np.array([
                        current_pose.pose.orientation.w,
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z
                    ])
                    
                    # Quaternion inverse: [w, x, y, z]^-1 = [w, -x, -y, -z]
                    q_prev_inv = np.array([q_prev[0], -q_prev[1], -q_prev[2], -q_prev[3]])
                    
                    # Quaternion multiplication: q_rel = q_curr * q_prev_inv
                    w1, x1, y1, z1 = q_curr
                    w2, x2, y2, z2 = q_prev_inv
                    q_rel = np.array([
                        w1*w2 - x1*x2 - y1*y2 - z1*z2,
                        w1*x2 + x1*w2 + y1*z2 - z1*y2,
                        w1*y2 - x1*z2 + y1*w2 + z1*x2,
                        w1*z2 + x1*y2 - y1*x2 + z1*w2
                    ])
                    
                    # Convert quaternion to axis-angle representation
                    # q = [cos(θ/2), sin(θ/2) * axis]
                    angle = 2 * math.acos(max(-1.0, min(1.0, q_rel[0])))  # Clamp to [-1, 1]
                    if abs(angle) < 1e-6:
                        axis_angle = np.array([0.0, 0.0, 0.0])
                    else:
                        sin_half = math.sqrt(1 - q_rel[0]*q_rel[0])
                        if sin_half < 1e-6:
                            axis_angle = np.array([0.0, 0.0, 0.0])
                        else:
                            axis = np.array([q_rel[1], q_rel[2], q_rel[3]]) / sin_half
                            axis_angle = angle * axis

                    # Create TwistStamped message
                    twist = TwistStamped()
                    twist.header.stamp = current_time.to_msg()
                    twist.header.frame_id = 'panda_link8'  # panda end-effector frame

                    # Apply scale and compute velocity
                    twist.twist.linear.x = (dx / dt) * self.linear_scale
                    twist.twist.linear.y = (dy / dt) * self.linear_scale
                    twist.twist.linear.z = (dz / dt) * self.linear_scale

                    twist.twist.angular.x = (axis_angle[0] / dt) * self.angular_scale
                    twist.twist.angular.y = (axis_angle[1] / dt) * self.angular_scale
                    twist.twist.angular.z = (axis_angle[2] / dt) * self.angular_scale

                    # Limit twist components to reasonable values (MoveIt Servo requirement)
                    max_component = max(
                        abs(twist.twist.linear.x), abs(twist.twist.linear.y), abs(twist.twist.linear.z),
                        abs(twist.twist.angular.x), abs(twist.twist.angular.y), abs(twist.twist.angular.z)
                    )
                    if max_component > 1.0:
                        scale = 1.0 / max_component
                        twist.twist.linear.x *= scale
                        twist.twist.linear.y *= scale
                        twist.twist.linear.z *= scale
                        twist.twist.angular.x *= scale
                        twist.twist.angular.y *= scale
                        twist.twist.angular.z *= scale

                    # Publish twist
                    self.twist_pub.publish(twist)

            # Update last pose and time
            self.last_pose = current_pose
            self.last_time = current_time

        except Exception as e:
            # TF lookup might fail initially, just log and continue
            self.get_logger().debug(f'TF lookup failed: {str(e)}')

    def timer_callback(self):
        """Periodic callback (can be used for continuous publishing if needed)"""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = OmyL100ToTwist()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down omy_l100_to_twist_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
