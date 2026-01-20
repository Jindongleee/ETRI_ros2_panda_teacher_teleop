#!/usr/bin/env python3

"""
Disable Collision Flag Node
----------------------------
Publishes false to /collision_flag to disable collision detection
for joint_trajectory_command_broadcaster.

This allows omy_l100 to move freely without being blocked by collision detection.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class DisableCollisionFlag(Node):
    def __init__(self):
        super().__init__('disable_collision_flag_node')
        
        # Publisher for collision_flag
        self.collision_flag_pub = self.create_publisher(
            Bool,
            '/collision_flag',
            10
        )
        
        # Timer to publish false at regular intervals (50 Hz for higher priority)
        self.timer = self.create_timer(0.02, self.publish_collision_flag)
        
        # Immediately publish false
        self.publish_collision_flag()
        
        self.get_logger().info('Disable Collision Flag node started')
        self.get_logger().info('Publishing false to /collision_flag to disable collision detection')
    
    def publish_collision_flag(self):
        """Publish false to disable collision detection"""
        msg = Bool()
        msg.data = False
        self.collision_flag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DisableCollisionFlag()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down disable_collision_flag_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
