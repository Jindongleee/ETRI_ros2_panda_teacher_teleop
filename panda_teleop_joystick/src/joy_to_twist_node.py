#!/usr/bin/env python3
"""
조이스틱 → Twist 변환 노드
============================
목적: 조이스틱 축/버튼 입력을 MoveIt Servo용 TwistStamped 속도 명령으로 변환한다.

구독 토픽:
  - /joy (sensor_msgs/Joy) : 조이스틱 입력
  - /clutch/active (std_msgs/Bool) : 클러치 페달 상태 (use_clutch=True 시)
발행 토픽:
  - /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped) : Servo 속도 명령

안전 제어:
  - use_clutch=True (기본값): 클러치 페달을 누르고 있어야만 로봇이 움직임
  - use_clutch=False: 조이스틱 데드맨 버튼 방식으로 대체 가능

축 매핑 (기본, 런치 파일에서 오버라이드 가능):
  - 왼쪽 스틱 X/Y → 선속도 X/Y
  - L1/L2 버튼    → 선속도 Z (상/하)
  - 오른쪽 스틱 X/Y → 각속도 X/Y (Roll/Pitch)
  - R1/R2 버튼    → 각속도 Z (Yaw)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


class JoyToTwist(Node):
    """조이스틱 입력을 TwistStamped 속도 명령으로 변환하는 ROS2 노드"""

    def __init__(self):
        super().__init__('joy_to_twist_node')

        # === 파라미터 선언 ===
        # 스케일: 조이스틱 축 값(-1~1)에 곱해져 Servo 입력이 됨
        self.declare_parameter('linear_scale', 3.0)
        self.declare_parameter('angular_scale', 3.5)
        self.declare_parameter('frame_id', 'gripper_tip_link')
        self.declare_parameter('use_clutch', True)

        # 버튼 매핑 (Z축은 버튼 기반)
        self.declare_parameter('button_linear_z_up', 6)
        self.declare_parameter('button_linear_z_down', 8)
        self.declare_parameter('button_angular_z_pos', 7)
        self.declare_parameter('button_angular_z_neg', 9)

        # 축 매핑 (어느 조이스틱 축이 어느 방향인지)
        self.declare_parameter('axis_linear_x', 0)
        self.declare_parameter('axis_linear_y', 1)
        self.declare_parameter('axis_angular_x', 3)
        self.declare_parameter('axis_angular_y', 2)

        # 축 반전 (1 = 정방향, -1 = 반전)
        self.declare_parameter('invert_linear_x', 1)
        self.declare_parameter('invert_linear_y', 1)
        self.declare_parameter('invert_angular_x', 1)
        self.declare_parameter('invert_angular_y', 1)

        # 데드존: 이 값 이하의 축 입력은 0으로 처리 (드리프트 방지)
        self.declare_parameter('axis_deadzone', 0.1)

        # === 파라미터 로드 ===
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.frame_id = self.get_parameter('frame_id').value
        self.axis_deadzone = self.get_parameter('axis_deadzone').value
        self.use_clutch = self.get_parameter('use_clutch').value

        self.button_linear_z_up = self.get_parameter('button_linear_z_up').value
        self.button_linear_z_down = self.get_parameter('button_linear_z_down').value
        self.button_angular_z_pos = self.get_parameter('button_angular_z_pos').value
        self.button_angular_z_neg = self.get_parameter('button_angular_z_neg').value

        self.clutch_active = False

        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_angular_x = self.get_parameter('axis_angular_x').value
        self.axis_angular_y = self.get_parameter('axis_angular_y').value

        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_angular_x = self.get_parameter('invert_angular_x').value
        self.invert_angular_y = self.get_parameter('invert_angular_y').value

        # === 구독자 ===
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        if self.use_clutch:
            self.clutch_sub = self.create_subscription(
                Bool, '/clutch/active', self.clutch_callback, 10
            )

        # === 발행자 ===
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )

        safety_mode = '클러치 페달' if self.use_clutch else '데드맨 버튼'
        self.get_logger().info(f'[Twist] 노드 시작 (안전 모드: {safety_mode})')

    def clutch_callback(self, msg: Bool):
        """클러치 페달 상태 갱신"""
        self.clutch_active = msg.data

    def joy_callback(self, msg: Joy):
        """조이스틱 입력을 TwistStamped로 변환하여 발행"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        control_enabled = self.clutch_active if self.use_clutch else True

        if control_enabled:
            def apply_deadzone(value, deadzone):
                return 0.0 if abs(value) < deadzone else value

            # 선속도 X (축 기반)
            if len(msg.axes) > abs(self.axis_linear_x):
                axis_val = apply_deadzone(msg.axes[abs(self.axis_linear_x)], self.axis_deadzone)
                twist.twist.linear.x = axis_val * self.invert_linear_x * self.linear_scale

            # 선속도 Y (축 기반)
            if len(msg.axes) > abs(self.axis_linear_y):
                axis_val = apply_deadzone(msg.axes[abs(self.axis_linear_y)], self.axis_deadzone)
                twist.twist.linear.y = axis_val * self.invert_linear_y * self.linear_scale

            # 선속도 Z (버튼 기반: L1=위, L2=아래)
            l1 = len(msg.buttons) > self.button_linear_z_up and msg.buttons[self.button_linear_z_up] == 1
            l2 = len(msg.buttons) > self.button_linear_z_down and msg.buttons[self.button_linear_z_down] == 1
            if l1 and not l2:
                twist.twist.linear.z = self.linear_scale
            elif l2 and not l1:
                twist.twist.linear.z = -self.linear_scale

            # 각속도 X (축 기반)
            if len(msg.axes) > abs(self.axis_angular_x):
                axis_val = apply_deadzone(msg.axes[abs(self.axis_angular_x)], self.axis_deadzone)
                twist.twist.angular.x = axis_val * self.invert_angular_x * self.angular_scale

            # 각속도 Y (축 기반)
            if len(msg.axes) > abs(self.axis_angular_y):
                axis_val = apply_deadzone(msg.axes[abs(self.axis_angular_y)], self.axis_deadzone)
                twist.twist.angular.y = axis_val * self.invert_angular_y * self.angular_scale

            # 각속도 Z (버튼 기반: R1=양, R2=음)
            r1 = len(msg.buttons) > self.button_angular_z_pos and msg.buttons[self.button_angular_z_pos] == 1
            r2 = len(msg.buttons) > self.button_angular_z_neg and msg.buttons[self.button_angular_z_neg] == 1
            if r1 and not r2:
                twist.twist.angular.z = self.angular_scale
            elif r2 and not r1:
                twist.twist.angular.z = -self.angular_scale

            # MoveIt Servo 입력 제한: 최대 성분이 1.0을 초과하면 전체 정규화
            max_component = max(
                abs(twist.twist.linear.x), abs(twist.twist.linear.y), abs(twist.twist.linear.z),
                abs(twist.twist.angular.x), abs(twist.twist.angular.y), abs(twist.twist.angular.z),
            )
            if max_component > 1.0:
                scale = 1.0 / max_component
                twist.twist.linear.x *= scale
                twist.twist.linear.y *= scale
                twist.twist.linear.z *= scale
                twist.twist.angular.x *= scale
                twist.twist.angular.y *= scale
                twist.twist.angular.z *= scale

        # 안전을 위해 항상 발행 (제어 비활성 시 0 속도)
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('[Twist] 노드 종료')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
