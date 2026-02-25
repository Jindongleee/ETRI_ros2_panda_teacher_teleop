#!/usr/bin/env python3
"""
OMY L100 → Twist 변환 노드
============================
목적: OMY L100 리더 암의 엔드이펙터 포즈 변화를
      Panda 팔로워 암용 TwistStamped 속도 명령으로 변환한다.

구독 토픽:
  - /leader/joint_states (sensor_msgs/JointState) : OMY L100 조인트 상태
  - /clutch/active (std_msgs/Bool) : 클러치 안전 제어

발행 토픽:
  - /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped) : Servo 속도 명령

동작 방식:
  1. OMY L100 조인트 상태 → TF로 FK(순기구학) 계산 → 엔드이펙터 포즈 획득
  2. 이전 포즈와의 차이 / dt → 선속도 및 각속도 계산
  3. 스케일 적용 + 축 반전 + 속도 제한 후 TwistStamped 발행
  4. 클러치가 비활성 상태이면 0 속도만 발행 (안전)

오프셋 시스템:
  - 리더/팔로워 암의 초기 위치 차이를 오프셋으로 기록
  - 클러치 활성화 시마다 오프셋 재계산 → 급격한 점프 방지
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
import numpy as np
import math


class OmyL100ToTwist(Node):
    """OMY L100 엔드이펙터 모션을 Panda Servo 속도 명령으로 변환하는 ROS2 노드"""

    def __init__(self):
        super().__init__('omy_l100_to_twist_node')

        # === 파라미터 선언 ===
        self.declare_parameter('base_frame', 'leader_link0')       # OMY L100 베이스 프레임
        self.declare_parameter('ee_frame', 'leader_link7')         # OMY L100 엔드이펙터 프레임
        self.declare_parameter('target_frame', 'panda_link0')      # Panda 베이스 프레임
        self.declare_parameter('follower_ee_frame', 'gripper_tip_link')  # Panda 엔드이펙터 프레임
        self.declare_parameter('joint_states_topic', '/leader/joint_states')
        self.declare_parameter('linear_scale', 1.5)
        self.declare_parameter('angular_scale', 1.5)
        self.declare_parameter('publish_rate', 150.0)              # 제어 루프 주파수 (Hz)

        # 축 반전 (리더-팔로워 좌표계가 다를 때 사용, 1.0=정방향, -1.0=반전)
        self.declare_parameter('invert_linear_x', 1.0)
        self.declare_parameter('invert_linear_y', 1.0)
        self.declare_parameter('invert_linear_z', -1.0)
        self.declare_parameter('invert_angular_x', 1.0)
        self.declare_parameter('invert_angular_y', 1.0)
        self.declare_parameter('invert_angular_z', 1.0)

        # 오프셋 초기값 (자동 계산 전 사용)
        self.declare_parameter('initial_offset_x', 0.0)
        self.declare_parameter('initial_offset_y', 0.0)
        self.declare_parameter('initial_offset_z', 0.0)

        # === 파라미터 로드 ===
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.follower_ee_frame = self.get_parameter('follower_ee_frame').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        publish_rate = self.get_parameter('publish_rate').value

        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_linear_z = self.get_parameter('invert_linear_z').value
        self.invert_angular_x = self.get_parameter('invert_angular_x').value
        self.invert_angular_y = self.get_parameter('invert_angular_y').value
        self.invert_angular_z = self.get_parameter('invert_angular_z').value

        # 속도 제한 (안전 범위)
        self.max_linear_speed = 10.0    # [m/s]
        self.max_angular_speed = 10.0   # [rad/s]

        # 클러치 상태 (True=활성=텔레오퍼레이션 가능, False=일시정지)
        self.clutch_active = False
        self.last_clutch_active = False

        # 리더-팔로워 오프셋 (클러치 활성화 시 자동 계산)
        self.offset_position = np.array([
            self.get_parameter('initial_offset_x').value,
            self.get_parameter('initial_offset_y').value,
            self.get_parameter('initial_offset_z').value
        ])
        self.offset_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # 단위 쿼터니언 [w, x, y, z]
        self.offset_initialized = False

        # TF 버퍼 (FK 계산에 필요)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 속도 계산용 이전 포즈/시간
        self.last_pose = None
        self.last_time = None

        # === 구독자 ===
        self.joint_states_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_state_callback, 10
        )
        self.clutch_sub = self.create_subscription(
            Bool, '/clutch/active', self.clutch_callback, 10
        )

        # === 발행자 ===
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )

        self.get_logger().info('[Twist] 노드 시작')

    def clutch_callback(self, msg: Bool):
        """클러치 상태 변화 처리 (사용자 확인 필요 → 한글 로그 유지)"""
        prev_state = self.clutch_active
        self.clutch_active = msg.data

        if prev_state != self.clutch_active:
            if self.clutch_active:
                self.get_logger().info('[클러치] 활성화 - 텔레오퍼레이션 시작')
                if self.offset_initialized:
                    self.recalculate_offset()
                self.last_pose = None
                self.last_time = None
            else:
                self.get_logger().info('[클러치] 해제 - 텔레오퍼레이션 일시정지')
                self.publish_zero_twist()
                self.last_pose = None
                self.last_time = None

    def recalculate_offset(self):
        """리더-팔로워 간 위치/방향 오프셋 재계산"""
        try:
            leader_transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            leader_pos = np.array([
                leader_transform.transform.translation.x,
                leader_transform.transform.translation.y,
                leader_transform.transform.translation.z
            ])
            leader_quat = np.array([
                leader_transform.transform.rotation.w,
                leader_transform.transform.rotation.x,
                leader_transform.transform.rotation.y,
                leader_transform.transform.rotation.z
            ])

            follower_transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.follower_ee_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            follower_pos = np.array([
                follower_transform.transform.translation.x,
                follower_transform.transform.translation.y,
                follower_transform.transform.translation.z
            ])
            follower_quat = np.array([
                follower_transform.transform.rotation.w,
                follower_transform.transform.rotation.x,
                follower_transform.transform.rotation.y,
                follower_transform.transform.rotation.z
            ])

            # 오프셋 = 팔로워 위치 - 리더 위치
            self.offset_position = follower_pos - leader_pos
            leader_quat_inv = np.array([leader_quat[0], -leader_quat[1], -leader_quat[2], -leader_quat[3]])
            self.offset_orientation = self.quaternion_multiply(follower_quat, leader_quat_inv)
            self.offset_initialized = True

            self.get_logger().info('[오프셋] 재계산 성공')

        except Exception as e:
            self.get_logger().warn(f'[오프셋] 재계산 실패: {str(e)}')
            self.offset_initialized = False

    def quaternion_multiply(self, q1, q2):
        """쿼터니언 곱셈: q1 * q2 (형식: [w, x, y, z])"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def publish_zero_twist(self):
        """0 속도 Twist 발행 (안전 정지)"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'panda_link0'
        self.twist_pub.publish(twist)

    def joint_state_callback(self, msg: JointState):
        """OMY L100 조인트 상태 수신 → FK로 포즈 계산 → 속도 명령 발행"""

        # 클러치 비활성 시 안전 정지
        if not self.clutch_active:
            self.publish_zero_twist()
            return

        # 오프셋 미초기화 시 첫 유효 포즈에서 자동 계산
        if not self.offset_initialized and self.last_pose is not None:
            self.recalculate_offset()

        try:
            # TF에서 리더 엔드이펙터 포즈 조회 (FK)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )

            current_pose = PoseStamped()
            current_pose.header.frame_id = self.base_frame
            current_pose.header.stamp = transform.header.stamp
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation

            current_time = self.get_clock().now()

            if self.last_pose is not None and self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds * 1e-9
                if 0.0 < dt < 1.0:
                    # 선속도 = 위치 변화 / dt
                    dx = current_pose.pose.position.x - self.last_pose.pose.position.x
                    dy = current_pose.pose.position.y - self.last_pose.pose.position.y
                    dz = current_pose.pose.position.z - self.last_pose.pose.position.z

                    # 각속도 = 쿼터니언 차이 → 축-각 변환 / dt
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
                    q_prev_inv = np.array([q_prev[0], -q_prev[1], -q_prev[2], -q_prev[3]])

                    w1, x1, y1, z1 = q_curr
                    w2, x2, y2, z2 = q_prev_inv
                    q_rel = np.array([
                        w1*w2 - x1*x2 - y1*y2 - z1*z2,
                        w1*x2 + x1*w2 + y1*z2 - z1*y2,
                        w1*y2 - x1*z2 + y1*w2 + z1*x2,
                        w1*z2 + x1*y2 - y1*x2 + z1*w2
                    ])

                    angle = 2 * math.acos(max(-1.0, min(1.0, q_rel[0])))
                    if abs(angle) < 1e-6:
                        axis_angle = np.array([0.0, 0.0, 0.0])
                    else:
                        sin_half = math.sqrt(1 - q_rel[0]*q_rel[0])
                        if sin_half < 1e-6:
                            axis_angle = np.array([0.0, 0.0, 0.0])
                        else:
                            axis = np.array([q_rel[1], q_rel[2], q_rel[3]]) / sin_half
                            axis_angle = angle * axis

                    # 스케일 및 축 반전 적용
                    lin_vec = np.array([dx, dy, dz]) / dt * self.linear_scale
                    ang_vec = np.array(axis_angle) / dt * self.angular_scale

                    lin_vec[0] *= self.invert_linear_x
                    lin_vec[1] *= self.invert_linear_y
                    lin_vec[2] *= self.invert_linear_z
                    ang_vec[0] *= self.invert_angular_x
                    ang_vec[1] *= self.invert_angular_y
                    ang_vec[2] *= self.invert_angular_z

                    # 속도 제한 (벡터 크기 기준)
                    lin_norm = np.linalg.norm(lin_vec)
                    if lin_norm > self.max_linear_speed and lin_norm > 0.0:
                        lin_vec *= self.max_linear_speed / lin_norm

                    ang_norm = np.linalg.norm(ang_vec)
                    if ang_norm > self.max_angular_speed and ang_norm > 0.0:
                        ang_vec *= self.max_angular_speed / ang_norm

                    # TwistStamped 발행
                    twist = TwistStamped()
                    twist.header.stamp = current_time.to_msg()
                    twist.header.frame_id = 'panda_link0'
                    twist.twist.linear.x = lin_vec[0]
                    twist.twist.linear.y = lin_vec[1]
                    twist.twist.linear.z = lin_vec[2]
                    twist.twist.angular.x = ang_vec[0]
                    twist.twist.angular.y = ang_vec[1]
                    twist.twist.angular.z = ang_vec[2]
                    self.twist_pub.publish(twist)
            else:
                # 첫 프레임: 0 속도 발행 (초기화)
                self.publish_zero_twist()

            self.last_pose = current_pose
            self.last_time = current_time

        except Exception:
            # TF 조회 실패 시 0 속도 발행 (안전)
            self.publish_zero_twist()


def main(args=None):
    rclpy.init(args=args)
    node = OmyL100ToTwist()

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
