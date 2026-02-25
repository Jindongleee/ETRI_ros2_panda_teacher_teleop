#!/usr/bin/env python3
"""
OMY L100 → Panda 그리퍼 변환 노드
====================================
목적: OMY L100 리더 암의 그리퍼 조인트(rh_r1_joint)를
      Panda 그리퍼 위치 명령으로 선형 매핑하여 발행한다.

구독 토픽: /leader/joint_states (sensor_msgs/JointState)
발행 토픽: /gripper/position (std_msgs/Float64)

매핑:
  OMY L100 그리퍼 [-1.0 ~ 1.0] → Panda 그리퍼 [0.0 ~ 0.8]
  (반전 매핑: OMY -1.0 = Panda 0.8(닫힘), OMY 1.0 = Panda 0.0(열림))
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class OmyL100ToGripper(Node):
    """OMY L100 그리퍼 조인트를 Panda 그리퍼 위치로 매핑하는 ROS2 노드"""

    def __init__(self):
        super().__init__('omy_l100_to_gripper_node')

        # 파라미터: OMY 그리퍼 조인트명 및 양쪽 그리퍼 범위
        self.declare_parameter('omy_gripper_joint_name', 'rh_r1_joint')
        self.declare_parameter('omy_gripper_min', -1.0)
        self.declare_parameter('omy_gripper_max', 1.0)
        self.declare_parameter('panda_gripper_min', 0.0)
        self.declare_parameter('panda_gripper_max', 0.8)
        self.declare_parameter('joint_states_topic', '/leader/joint_states')

        self.omy_gripper_joint_name = self.get_parameter('omy_gripper_joint_name').value
        self.omy_gripper_min = self.get_parameter('omy_gripper_min').value
        self.omy_gripper_max = self.get_parameter('omy_gripper_max').value
        self.panda_gripper_min = self.get_parameter('panda_gripper_min').value
        self.panda_gripper_max = self.get_parameter('panda_gripper_max').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        # 내부 상태
        self.current_panda_gripper = 0.0
        self.last_omy_gripper = None

        # 구독: OMY L100 조인트 상태
        self.joint_states_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_state_callback, 10
        )

        # 발행: Panda 그리퍼 위치
        self.gripper_pub = self.create_publisher(
            Float64, '/gripper/position', 10
        )

        self.get_logger().info('[그리퍼] 노드 시작')
        self.publish_position()

    def joint_state_callback(self, msg: JointState):
        """OMY L100 조인트에서 그리퍼 값을 추출하여 Panda 범위로 매핑"""
        if self.omy_gripper_joint_name not in msg.name:
            return

        joint_index = msg.name.index(self.omy_gripper_joint_name)
        if joint_index >= len(msg.position):
            return

        omy_gripper_value = msg.position[joint_index]

        # 의미 있는 변화가 없으면 발행 생략
        if self.last_omy_gripper is not None:
            if abs(omy_gripper_value - self.last_omy_gripper) < 0.01:
                return

        self.last_omy_gripper = omy_gripper_value

        # 선형 매핑: OMY [-1.0, 1.0] → Panda [0.0, 0.8] (반전)
        normalized = (omy_gripper_value - self.omy_gripper_min) / (self.omy_gripper_max - self.omy_gripper_min)
        panda_gripper = (1.0 - normalized) * (self.panda_gripper_max - self.panda_gripper_min) + self.panda_gripper_min
        panda_gripper = max(self.panda_gripper_min, min(panda_gripper, self.panda_gripper_max))

        self.current_panda_gripper = panda_gripper
        self.publish_position()

    def publish_position(self):
        """현재 Panda 그리퍼 위치를 발행"""
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
            node.get_logger().info('[그리퍼] 노드 종료')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
