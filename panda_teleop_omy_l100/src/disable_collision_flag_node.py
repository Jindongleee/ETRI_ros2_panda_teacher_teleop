#!/usr/bin/env python3
"""
충돌 감지 비활성화 플래그 노드
================================
목적: /collision_flag 토픽에 false를 주기적으로 발행하여
      joint_trajectory_command_broadcaster의 충돌 감지를 비활성화한다.

발행 토픽: /collision_flag (std_msgs/Bool)

사용 이유: OMY L100 리더 암이 텔레오퍼레이션 중 충돌 감지에 의해
           차단되지 않도록 하기 위함이다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class DisableCollisionFlag(Node):
    """충돌 감지 비활성화 플래그를 주기적으로 발행하는 ROS2 노드"""

    def __init__(self):
        super().__init__('disable_collision_flag_node')

        self.collision_flag_pub = self.create_publisher(Bool, '/collision_flag', 10)

        # 50Hz로 false 발행 (높은 우선순위 보장)
        self.timer = self.create_timer(0.02, self.publish_collision_flag)

        self.publish_collision_flag()
        self.get_logger().info('[충돌플래그] 노드 시작 - 충돌 감지 비활성화')

    def publish_collision_flag(self):
        """충돌 감지 비활성화(false) 발행"""
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
            node.get_logger().info('[충돌플래그] 노드 종료')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
