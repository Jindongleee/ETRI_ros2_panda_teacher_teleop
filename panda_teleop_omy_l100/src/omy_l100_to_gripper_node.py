#!/usr/bin/env python3

"""
Omy L100 to Gripper Node
-------------------------
Converts omy_l100 gripper joint (rh_r1_joint) to panda gripper position command.

This node:
- Subscribes to /leader/joint_states (omy_l100 joint states)
- Extracts rh_r1_joint value (range: -1.0 ~ 1.0)
- Maps it to panda gripper range (0.0 ~ 0.8)
- Publishes as std_msgs/Float64 on /gripper/position
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class OmyL100ToGripper(Node):
    def __init__(self):
        super().__init__('omy_l100_to_gripper_node')

        # Parameters for mapping omy_l100 gripper to panda gripper
        self.declare_parameter('omy_gripper_joint_name', 'rh_r1_joint')
        self.declare_parameter('omy_gripper_min', -1.0)  # omy_l100 gripper range
        self.declare_parameter('omy_gripper_max', 1.0)
        self.declare_parameter('panda_gripper_min', 0.0)  # panda gripper range
        self.declare_parameter('panda_gripper_max', 0.8)
        self.declare_parameter('joint_states_topic', '/leader/joint_states')

        self.omy_gripper_joint_name = self.get_parameter('omy_gripper_joint_name').value
        self.omy_gripper_min = self.get_parameter('omy_gripper_min').value
        self.omy_gripper_max = self.get_parameter('omy_gripper_max').value
        self.panda_gripper_min = self.get_parameter('panda_gripper_min').value
        self.panda_gripper_max = self.get_parameter('panda_gripper_max').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        # Current panda gripper position
        self.current_panda_gripper = 0.0
        self.last_omy_gripper = None

        # Subscriber
        self.joint_states_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10
        )

        # Publisher
        self.gripper_pub = self.create_publisher(
            Float64,
            '/gripper/position',
            10
        )

        self.get_logger().info('Omy L100 to Gripper node started')
        self.get_logger().info(f'Subscribing to: {joint_states_topic}')
        self.get_logger().info(f'Looking for joint: {self.omy_gripper_joint_name}')
        self.get_logger().info(
            f'Mapping: omy_l100 [{self.omy_gripper_min}, {self.omy_gripper_max}] '
            f'→ panda [{self.panda_gripper_min}, {self.panda_gripper_max}]'
        )

        # Publish initial position
        self.publish_position()

    def joint_state_callback(self, msg: JointState):
        """Extract rh_r1_joint from omy_l100 joint_states and map to panda gripper"""
        if self.omy_gripper_joint_name not in msg.name:
            return

        # Find the index of rh_r1_joint
        joint_index = msg.name.index(self.omy_gripper_joint_name)
        if joint_index >= len(msg.position):
            return

        omy_gripper_value = msg.position[joint_index]

        # Skip if value hasn't changed significantly (reduce unnecessary publishes)
        if self.last_omy_gripper is not None:
            if abs(omy_gripper_value - self.last_omy_gripper) < 0.01:
                return

        self.last_omy_gripper = omy_gripper_value

        # Linear mapping: omy_l100 [-1.0, 1.0] → panda [0.0, 0.8]
        # Formula: panda = (omy - omy_min) / (omy_max - omy_min) * (panda_max - panda_min) + panda_min
        normalized = (omy_gripper_value - self.omy_gripper_min) / (self.omy_gripper_max - self.omy_gripper_min)
        panda_gripper = (1.0 - normalized) * (self.panda_gripper_max - self.panda_gripper_min) + self.panda_gripper_min
        # Clamp to panda gripper range
        panda_gripper = max(self.panda_gripper_min, min(panda_gripper, self.panda_gripper_max))

        self.current_panda_gripper = panda_gripper
        self.publish_position()

        # Debug log (only when significant change)
        self.get_logger().debug(
            f'Omy gripper: {omy_gripper_value:.3f} → Panda gripper: {panda_gripper:.3f}'
        )

    def publish_position(self):
        """Publish current panda gripper position"""
        msg = Float64()
        msg.data = float(self.current_panda_gripper)
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OmyL100ToGripper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down omy_l100_to_gripper_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
