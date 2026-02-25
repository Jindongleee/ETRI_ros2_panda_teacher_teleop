#!/usr/bin/env python3
"""
Vive → 그리퍼 제어 노드
=========================
목적: Vive 컨트롤러의 버튼/트리거 입력을 Panda 그리퍼 위치 명령으로 변환한다.

구독 토픽: /vive/controller/buttons (sensor_msgs/Joy)
발행 토픽: /gripper/position (std_msgs/Float64)

제어 모드:
  - analog: 트리거 축 값(0~1)을 그리퍼 위치에 직접 매핑
  - discrete: 버튼 누름 시 step만큼 증감 (라이징 엣지)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class ViveToGripper(Node):
    """Vive 컨트롤러 입력을 그리퍼 위치 명령으로 변환하는 ROS2 노드"""

    def __init__(self):
        super().__init__('vive_to_gripper_node')

        # 파라미터: 제어 모드 및 매핑 설정
        self.declare_parameter('mode', 'analog')        # 'analog' 또는 'discrete'
        self.declare_parameter('axis_trigger', 0)       # 트리거 축 인덱스
        self.declare_parameter('invert_trigger', False)  # 트리거 반전 여부
        self.declare_parameter('button_open', 2)        # 열기 버튼 (discrete 모드)
        self.declare_parameter('button_close', 0)       # 닫기 버튼 (discrete 모드)
        self.declare_parameter('step', 0.02)            # 1회 버튼당 위치 변화 (discrete)
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 0.8)
        self.declare_parameter('vive_buttons_topic', '/vive/controller/buttons')

        self.mode = self.get_parameter('mode').value
        self.axis_trigger = self.get_parameter('axis_trigger').value
        self.invert_trigger = self.get_parameter('invert_trigger').value
        self.button_open = self.get_parameter('button_open').value
        self.button_close = self.get_parameter('button_close').value
        self.step = self.get_parameter('step').value
        self.min_position = self.get_parameter('min_position').value
        self.max_position = self.get_parameter('max_position').value
        self.vive_buttons_topic = self.get_parameter('vive_buttons_topic').value

        # 내부 상태
        self.current_position = 0.0
        self._last_buttons = []

        # 구독: Vive 버튼 입력
        self.buttons_sub = self.create_subscription(
            Joy, self.vive_buttons_topic, self.buttons_callback, 10
        )

        # 발행: 그리퍼 위치
        self.gripper_pub = self.create_publisher(Float64, '/gripper/position', 10)

        self.get_logger().info(f'[그리퍼] 노드 시작 (모드: {self.mode})')
        self._publish_position()

    def _publish_position(self):
        """현재 그리퍼 위치를 발행"""
        msg = Float64()
        msg.data = self.current_position
        self.gripper_pub.publish(msg)

    def buttons_callback(self, msg: Joy):
        """모드에 따라 analog 또는 discrete 처리"""
        if self.mode == 'analog':
            self._handle_analog_mode(msg)
        else:
            self._handle_discrete_mode(msg)

    def _handle_analog_mode(self, msg: Joy):
        """트리거 축 값을 그리퍼 위치에 연속 매핑"""
        axes = msg.axes
        if self.axis_trigger < len(axes):
            trigger_val = max(0.0, min(1.0, axes[self.axis_trigger]))

            if self.invert_trigger:
                ratio = 1.0 - trigger_val
            else:
                ratio = trigger_val

            self.current_position = self.min_position + ratio * (self.max_position - self.min_position)
            self._publish_position()

    def _handle_discrete_mode(self, msg: Joy):
        """버튼 라이징 엣지로 그리퍼 위치를 step만큼 증감"""
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
        node.get_logger().info('[그리퍼] 노드 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
