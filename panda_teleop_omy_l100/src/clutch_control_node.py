#!/usr/bin/env python3
"""
키보드 기반 클러치 제어 노드
==============================
목적: 키보드 'b' 키를 누르고 있는 동안만 텔레오퍼레이션을 활성화하는
      안전 제어(hold-to-activate) 노드이다.

발행 토픽: /clutch/active (std_msgs/Bool)

동작 방식:
  - 'b' 키를 누르고 있으면 clutch_active=True (텔레오퍼레이션 활성화)
  - 'b' 키를 놓으면 clutch_active=False (텔레오퍼레이션 일시정지)
  - 'q' 키로 노드 종료
  - 100ms 이내에 'b' 키 입력이 없으면 자동 비활성화

참고: evdev 기반 clutch_pedal_node (panda_common)와 역할이 동일하며,
      풋스위치가 없을 때 키보드로 대체하기 위한 노드이다.
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ClutchControlNode(Node):
    """키보드 'b' 키로 클러치를 제어하는 ROS2 노드 (hold-to-activate)"""

    def __init__(self):
        super().__init__('clutch_control_node')

        self.clutch_active = False
        self.last_key_state = False
        self.last_b_key_time = None
        self.key_timeout = 0.1  # 100ms: 이 시간 내에 'b' 입력이 없으면 비활성화

        # 클러치 상태 발행자
        self.clutch_pub = self.create_publisher(Bool, '/clutch/active', 10)

        # 키보드 폴링 타이머 (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # 터미널 raw 모드 설정 (키 입력 즉시 감지를 위해)
        self.old_settings = None
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception as e:
            self.get_logger().warn(f'[클러치] 터미널 설정 실패: {e}')

        self.publish_clutch_state()

    def timer_callback(self):
        """키보드 입력 폴링: 'b' 키가 유지되는 동안만 활성화"""
        try:
            current_time = self.get_clock().now()

            # 버퍼에 있는 모든 키 입력 읽기
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'b':
                    self.last_b_key_time = current_time
                elif key.lower() == 'q':
                    self.get_logger().info('[클러치] 종료 키(q) 입력')
                    rclpy.shutdown()
                    return

            # 'b' 키 타임아웃 기반 클러치 상태 결정
            prev_state = self.clutch_active
            if self.last_b_key_time is not None:
                time_since_last_b = (current_time - self.last_b_key_time).nanoseconds * 1e-9
                self.clutch_active = (time_since_last_b < self.key_timeout)
            else:
                self.clutch_active = False

            # 상태 변화 시에만 발행 및 로그 (사용자 확인용)
            if prev_state != self.clutch_active:
                self.publish_clutch_state()
                if self.clutch_active:
                    self.get_logger().info('[클러치] 활성화 - 텔레오퍼레이션 시작')
                else:
                    self.get_logger().info('[클러치] 해제 - 텔레오퍼레이션 일시정지')

        except Exception:
            pass

    def publish_clutch_state(self):
        """현재 클러치 상태를 발행"""
        msg = Bool()
        msg.data = self.clutch_active
        self.clutch_pub.publish(msg)

    def cleanup(self):
        """터미널 설정 복원"""
        if self.old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ClutchControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cleanup()
            node.get_logger().info('[클러치] 노드 종료')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
