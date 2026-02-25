#!/usr/bin/env python3
"""
조이스틱 → 그리퍼 제어 노드
============================
목적: 조이스틱 버튼 입력을 Panda 그리퍼 열기/닫기 명령으로 변환한다.

구독 토픽: /joy (sensor_msgs/Joy)
발행 토픽: /gripper/position (std_msgs/Float64)

동작 방식:
  - button_open 버튼(라이징 엣지) → 그리퍼 위치를 step만큼 감소(열기)
  - button_close 버튼(라이징 엣지) → 그리퍼 위치를 step만큼 증가(닫기)
  - 위치는 [min_position, max_position] 범위로 클램핑
  - trajectory_to_joint_states 노드가 /gripper/position을 구독하여
    finger_joint 관련 조인트를 /joint_states에 반영한다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class JoyToGripper(Node):
    """조이스틱 버튼을 그리퍼 위치 명령으로 변환하는 ROS2 노드"""

    def __init__(self):
        super().__init__('joy_to_gripper_node')

        # 파라미터 선언: 버튼 인덱스 및 그리퍼 범위
        self.declare_parameter('button_open', 1)        # 열기 버튼 (기본: Circle)
        self.declare_parameter('button_close', 3)       # 닫기 버튼 (기본: Square)
        self.declare_parameter('step', 0.02)            # 1회 버튼 입력당 위치 변화량
        self.declare_parameter('min_position', 0.0)     # 최소 위치 (완전 열림)
        self.declare_parameter('max_position', 0.8)     # 최대 위치 (완전 닫힘, SRDF 기준)

        self.button_open = self.get_parameter('button_open').value
        self.button_close = self.get_parameter('button_close').value
        self.step = self.get_parameter('step').value
        self.min_position = self.get_parameter('min_position').value
        self.max_position = self.get_parameter('max_position').value

        # 내부 상태: 현재 그리퍼 위치 및 이전 버튼 상태 (라이징 엣지 감지용)
        self.current_position = 0.0
        self._last_buttons = []

        # /joy 구독
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        # /gripper/position 발행
        self.gripper_pub = self.create_publisher(
            Float64, '/gripper/position', 10
        )

        self.get_logger().info('[그리퍼] 노드 시작')

        # 초기 위치 발행
        self.publish_position()

    def joy_callback(self, msg: Joy):
        """조이스틱 메시지 수신 시 버튼 라이징 엣지를 감지하여 그리퍼 위치를 갱신"""
        buttons = msg.buttons
        if not self._last_buttons:
            self._last_buttons = [0] * len(buttons)

        # 라이징 엣지 감지 (이전 0 → 현재 1)
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

        # SRDF/URDF 기준: finger_joint 0.0=열림, 0.8=닫힘
        if close_pressed:
            self.current_position += self.step
        if open_pressed:
            self.current_position -= self.step

        self.current_position = max(self.min_position, min(self.current_position, self.max_position))

        if open_pressed or close_pressed:
            self.publish_position()

        self._last_buttons = buttons

    def publish_position(self):
        """현재 그리퍼 위치를 /gripper/position 토픽으로 발행"""
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
