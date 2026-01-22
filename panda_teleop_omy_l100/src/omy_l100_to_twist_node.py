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
- Supports clutch control (pause/resume) via /clutch/active
- Applies position/orientation offset between leader and follower arms
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Bool
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
        self.declare_parameter('follower_ee_frame', 'panda_link8')  # panda end-effector frame
        self.declare_parameter('joint_states_topic', '/leader/joint_states')
        # 스케일: 기본값 (launch 파일에서 오버라이드됨)
        self.declare_parameter('linear_scale', 1.5)   # Scale factor for linear velocity
        self.declare_parameter('angular_scale', 1.5)  # Scale factor for angular velocity
        # 제어 주기: 100 Hz (Servo와 일치, publish_period: 0.01s)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        # 축 반전 파라미터 (리더암과 팔로워암 좌표계 방향이 반대일 때 사용)
        # 1.0 = 정방향, -1.0 = 반전
        self.declare_parameter('invert_linear_x', 1.0)
        self.declare_parameter('invert_linear_y', 1.0)
        self.declare_parameter('invert_linear_z', -1.0)  # 기본값: Z축 반전 (위로 올리면 위로 가도록)
        self.declare_parameter('invert_angular_x', 1.0)
        self.declare_parameter('invert_angular_y', 1.0)
        self.declare_parameter('invert_angular_z', 1.0)
        # Offset 초기값 (선택사항)
        self.declare_parameter('initial_offset_x', 0.0)
        self.declare_parameter('initial_offset_y', 0.0)
        self.declare_parameter('initial_offset_z', 0.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.follower_ee_frame = self.get_parameter('follower_ee_frame').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        publish_rate = self.get_parameter('publish_rate').value
        # 축 반전 파라미터
        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_linear_z = self.get_parameter('invert_linear_z').value
        self.invert_angular_x = self.get_parameter('invert_angular_x').value
        self.invert_angular_y = self.get_parameter('invert_angular_y').value
        self.invert_angular_z = self.get_parameter('invert_angular_z').value

        # 최대 속도 제한 (대폭 완화)
        self.max_linear_speed = 1.0   # [m/s] (이전: 0.2)
        self.max_angular_speed = 2.0  # [rad/s] (이전: 0.5)
        
        # Clutch control
        self.clutch_active = False  # False = teleoperation active, True = paused
        self.last_clutch_active = False
        
        # Offset: P(offset) = P(follower_start) - P(leader_start)
        # 초기값은 파라미터에서 가져오거나 자동 계산
        self.offset_position = np.array([
            self.get_parameter('initial_offset_x').value,
            self.get_parameter('initial_offset_y').value,
            self.get_parameter('initial_offset_z').value
        ])
        self.offset_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion [w, x, y, z]
        self.offset_initialized = False  # Offset이 자동 계산되었는지 여부

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Previous pose for velocity calculation
        self.last_pose = None
        self.last_time = None
        
        # Debug logging: 카운터와 최소 움직임 임계값
        self.debug_counter = 0
        self.debug_log_interval = 10  # 10번마다 로그 출력 (너무 자주 출력 방지)
        self.min_motion_threshold = 1e-4  # 최소 움직임 임계값 (m 또는 rad)
        
        # TF lookup 실패 추적
        self.tf_lookup_fail_count = 0
        self.tf_lookup_success_count = 0
        self.last_tf_error = None
        self._last_status_log_time = self.get_clock().now()

        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10
        )
        
        self.clutch_sub = self.create_subscription(
            Bool,
            '/clutch/active',
            self.clutch_callback,
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
        self.get_logger().info(f'Follower EE frame: {self.follower_ee_frame}')
        self.get_logger().info(f'Publishing to: /servo_node/delta_twist_cmds')
        self.get_logger().info(f'Subscribing to: /clutch/active')
        self.get_logger().info(f'[CONFIG] Linear scale: {self.linear_scale}, Angular scale: {self.angular_scale}')
        self.get_logger().info(f'[CONFIG] Max linear speed: {self.max_linear_speed} m/s, Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'[CONFIG] Publish rate: {publish_rate} Hz')
        self.get_logger().info(f'[CONFIG] Axis inversion - Linear: X={self.invert_linear_x}, Y={self.invert_linear_y}, Z={self.invert_linear_z}')
        self.get_logger().info(f'[CONFIG] Axis inversion - Angular: X={self.invert_angular_x}, Y={self.invert_angular_y}, Z={self.invert_angular_z}')
        self.get_logger().info(f'[CONFIG] Initial offset: [{self.offset_position[0]:.3f}, {self.offset_position[1]:.3f}, {self.offset_position[2]:.3f}]')
        self.get_logger().info('[INFO] Offset will be auto-calculated on first motion or clutch toggle')
        self.get_logger().info('[INFO] Hold "b" key in clutch_control_node to enable teleoperation')
    
    def clutch_callback(self, msg: Bool):
        """Handle clutch state changes
        
        Clutch logic (safety feature):
        - clutch_active = True: Teleoperation ENABLED (operator holding 'b' key)
        - clutch_active = False: Teleoperation PAUSED (operator released 'b' key)
        """
        prev_state = self.clutch_active
        self.clutch_active = msg.data
        
        # Clutch 상태 변화 감지
        if prev_state != self.clutch_active:
            if self.clutch_active:
                # Clutch 활성화 (텔레오퍼레이션 시작)
                self.get_logger().info('[CLUTCH] PRESSED - Teleoperation ENABLED')
                # Offset 재계산 (안전한 시작을 위해)
                if self.offset_initialized:
                    self.recalculate_offset()
                # 이전 pose 리셋하여 갑작스러운 움직임 방지
                self.last_pose = None
                self.last_time = None
            else:
                # Clutch 비활성화 (텔레오퍼레이션 일시정지)
                self.get_logger().info('[CLUTCH] RELEASED - Teleoperation PAUSED (safety)')
                # 일시정지 시 0 속도 발행
                self.publish_zero_twist()
                # 이전 pose 리셋
                self.last_pose = None
                self.last_time = None
    
    def recalculate_offset(self):
        """Recalculate offset when clutch is toggled off"""
        try:
            # Get current leader EE pose
            leader_transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
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
            
            # Get current follower EE pose (from /joint_states via TF)
            follower_transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.follower_ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
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
            
            # Calculate offset: P(offset) = P(follower) - P(leader)
            self.offset_position = follower_pos - leader_pos
            
            # Calculate orientation offset: q_offset = q_follower * q_leader^-1
            leader_quat_inv = np.array([leader_quat[0], -leader_quat[1], -leader_quat[2], -leader_quat[3]])
            self.offset_orientation = self.quaternion_multiply(follower_quat, leader_quat_inv)
            
            self.offset_initialized = True
            
            self.get_logger().info(
                f'[OFFSET] Recalculated - Position: [{self.offset_position[0]:.3f}, '
                f'{self.offset_position[1]:.3f}, {self.offset_position[2]:.3f}] m'
            )
            self.get_logger().info(
                f'[OFFSET] Orientation: [{self.offset_orientation[0]:.3f}, '
                f'{self.offset_orientation[1]:.3f}, {self.offset_orientation[2]:.3f}, '
                f'{self.offset_orientation[3]:.3f}]'
            )
            
        except Exception as e:
            self.get_logger().error(f'[OFFSET] Failed to recalculate offset: {str(e)}')
            self.offset_initialized = False
    
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions: q1 * q2 (format: [w, x, y, z])"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def publish_zero_twist(self):
        """Publish zero twist command"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'panda_link8'
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.twist_pub.publish(twist)

    def joint_state_callback(self, msg: JointState):
        """Compute twist from joint states using TF"""
        
        # Clutch 비활성화 시 명령 발행 중지 (safety: 'b' 키를 누르고 있을 때만 동작)
        if not self.clutch_active:
            self.publish_zero_twist()
            return
        
        # Offset 자동 초기화 (첫 번째 유효한 pose에서)
        if not self.offset_initialized and self.last_pose is not None:
            self.get_logger().info('[OFFSET] Auto-initializing offset on first motion...')
            self.recalculate_offset()
        
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
            
            # TF lookup 성공 카운트
            self.tf_lookup_success_count += 1

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
                    twist.header.frame_id = 'panda_link8'  # panda end-effector frame (servo_config와 일치)

                    # Apply scale and compute velocity
                    # 스케일 적용 전 속도 (원본)
                    lin_vec_raw = np.array([dx, dy, dz]) / dt
                    ang_vec_raw = np.array(axis_angle) / dt
                    
                    # 스케일 적용
                    lin_vec = lin_vec_raw * self.linear_scale
                    ang_vec = ang_vec_raw * self.angular_scale
                    
                    # 축 반전 적용 (리더암과 팔로워암 좌표계 방향이 반대일 때)
                    lin_vec[0] *= self.invert_linear_x
                    lin_vec[1] *= self.invert_linear_y
                    lin_vec[2] *= self.invert_linear_z
                    ang_vec[0] *= self.invert_angular_x
                    ang_vec[1] *= self.invert_angular_y
                    ang_vec[2] *= self.invert_angular_z

                    # 최대 선속도 제한 (벡터 크기 기준)
                    lin_norm_raw = np.linalg.norm(lin_vec_raw)
                    lin_norm = np.linalg.norm(lin_vec)
                    lin_limited = False
                    if lin_norm > self.max_linear_speed and lin_norm > 0.0:
                        lin_vec *= self.max_linear_speed / lin_norm
                        lin_limited = True

                    # 최대 각속도 제한 (벡터 크기 기준)
                    ang_norm_raw = np.linalg.norm(ang_vec_raw)
                    ang_norm = np.linalg.norm(ang_vec)
                    ang_limited = False
                    if ang_norm > self.max_angular_speed and ang_norm > 0.0:
                        ang_vec *= self.max_angular_speed / ang_norm
                        ang_limited = True

                    twist.twist.linear.x = lin_vec[0]
                    twist.twist.linear.y = lin_vec[1]
                    twist.twist.linear.z = lin_vec[2]

                    twist.twist.angular.x = ang_vec[0]
                    twist.twist.angular.y = ang_vec[1]
                    twist.twist.angular.z = ang_vec[2]

                    # 디버그 로깅: 움직임이 있을 때만 출력
                    motion_detected = (lin_norm_raw > self.min_motion_threshold or 
                                     ang_norm_raw > self.min_motion_threshold)
                    
                    if motion_detected:
                        self.debug_counter += 1
                        if self.debug_counter % self.debug_log_interval == 0:
                            self.get_logger().info(
                                f'[DEBUG] dt={dt*1000:.2f}ms | '
                                f'Pose change: dx={dx*1000:.3f}mm, dy={dy*1000:.3f}mm, dz={dz*1000:.3f}mm | '
                                f'Raw speed: lin={lin_norm_raw:.4f}m/s, ang={ang_norm_raw:.4f}rad/s | '
                                f'Scaled speed: lin={lin_norm:.4f}m/s (scale={self.linear_scale}), '
                                f'ang={ang_norm:.4f}rad/s (scale={self.angular_scale}) | '
                                f'Limited: lin={lin_limited}, ang={ang_limited} | '
                                f'Final twist: lin=[{lin_vec[0]:.4f}, {lin_vec[1]:.4f}, {lin_vec[2]:.4f}], '
                                f'ang=[{ang_vec[0]:.4f}, {ang_vec[1]:.4f}, {ang_vec[2]:.4f}]'
                            )

                    # Publish twist
                    self.twist_pub.publish(twist)
            else:
                # last_pose가 None이면 0.0 twist를 발행 (초기화 단계)
                twist = TwistStamped()
                twist.header.stamp = current_time.to_msg()
                twist.header.frame_id = 'panda_link8'  # panda end-effector frame
                twist.twist.linear.x = 0.0
                twist.twist.linear.y = 0.0
                twist.twist.linear.z = 0.0
                twist.twist.angular.x = 0.0
                twist.twist.angular.y = 0.0
                twist.twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                
                # 첫 번째 pose 획득 로그
                if self.tf_lookup_success_count == 1:
                    self.get_logger().info(
                        f'[INIT] First pose acquired: position=[{current_pose.pose.position.x:.3f}, '
                        f'{current_pose.pose.position.y:.3f}, {current_pose.pose.position.z:.3f}]'
                    )

            # Update last pose and time
            self.last_pose = current_pose
            self.last_time = current_time

        except Exception as e:
            # TF lookup 실패 처리 및 상세 로깅
            self.tf_lookup_fail_count += 1
            self.last_tf_error = str(e)
            
            # 100번마다 또는 처음 10번은 상세 로그
            if self.tf_lookup_fail_count <= 10 or self.tf_lookup_fail_count % 100 == 0:
                self.get_logger().warn(
                    f'[TF ERROR] Lookup failed (count: {self.tf_lookup_fail_count}): {str(e)} | '
                    f'Base frame: {self.base_frame}, EE frame: {self.ee_frame}'
                )
            
            # TF lookup 실패 시에도 0.0 twist를 발행 (Servo가 계속 명령을 받을 수 있도록)
            try:
                twist = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.header.frame_id = 'panda_link8'  # panda end-effector frame
                twist.twist.linear.x = 0.0
                twist.twist.linear.y = 0.0
                twist.twist.linear.z = 0.0
                twist.twist.angular.x = 0.0
                twist.twist.angular.y = 0.0
                twist.twist.angular.z = 0.0
                self.twist_pub.publish(twist)
            except Exception as pub_error:
                self.get_logger().error(f'Failed to publish zero twist: {str(pub_error)}')

    def timer_callback(self):
        """Periodic callback: 주기적으로 상태 로그 출력"""
        # 5초마다 상태 요약 출력
        if hasattr(self, '_last_status_log_time'):
            elapsed = (self.get_clock().now() - self._last_status_log_time).nanoseconds * 1e-9
            if elapsed < 5.0:
                return
        
        self._last_status_log_time = self.get_clock().now()
        
        total_attempts = self.tf_lookup_success_count + self.tf_lookup_fail_count
        if total_attempts > 0:
            success_rate = (self.tf_lookup_success_count / total_attempts) * 100.0
            self.get_logger().info(
                f'[STATUS] TF lookup: success={self.tf_lookup_success_count}, '
                f'fail={self.tf_lookup_fail_count}, rate={success_rate:.1f}% | '
                f'Last pose: {"OK" if self.last_pose is not None else "None"}'
            )
            if self.tf_lookup_fail_count > 0 and self.last_tf_error:
                self.get_logger().warn(f'[STATUS] Last TF error: {self.last_tf_error}')


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
