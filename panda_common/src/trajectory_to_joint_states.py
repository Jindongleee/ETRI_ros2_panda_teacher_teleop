#!/usr/bin/env python3

"""
Trajectory to Joint States Node
Converts servo_node's joint_trajectory messages to joint_states for visualization

This node subscribes to /panda_arm_controller/joint_trajectory and publishes /joint_states
so that robot_state_publisher can generate TF transforms for RViz visualization.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64


class TrajectoryToJointStates(Node):
    def __init__(self):
        super().__init__('trajectory_to_joint_states')
        
        # Parameters
        self.declare_parameter('joint_names', [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7',
            'finger_joint',
            'left_inner_knuckle_joint', 'left_inner_finger_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint',
            'right_outer_knuckle_joint'
        ])
        
        self.joint_names = self.get_parameter('joint_names').value
        
        # Current joint positions (initialize to "ready" pose to avoid joint limit issues)
        # "ready" pose from SRDF: [0, -0.785, 0, -2.356, 0, 1.571, 0.785] for panda_joint1-7
        initial_positions = {
            'panda_joint1': 0.0,
            'panda_joint2': -0.785,
            'panda_joint3': 0.0,
            'panda_joint4': -2.356,
            'panda_joint5': 0.0,
            'panda_joint6': 1.571,
            'panda_joint7': 0.785,
            'finger_joint': 0.0,
            'left_inner_knuckle_joint': 0.0,
            'left_inner_finger_joint': 0.0,
            'right_inner_knuckle_joint': 0.0,
            'right_inner_finger_joint': 0.0,
            'right_outer_knuckle_joint': 0.0
        }
        # Use initial positions if defined, otherwise 0.0
        self.current_positions = {name: initial_positions.get(name, 0.0) for name in self.joint_names}
        self.current_velocities = {name: 0.0 for name in self.joint_names}
        # Debug용: 관절 속도 추정 로그를 위한 이전 상태 저장
        self._last_debug_time = self.get_clock().now()
        self._last_debug_positions = dict(self.current_positions)
        
        # Internal gripper state (finger_joint opening)
        self.gripper_position = 0.0

        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        # Gripper position subscriber (from joy_to_gripper_node)
        self.gripper_sub = self.create_subscription(
            Float64,
            '/gripper/position',
            self.gripper_callback,
            10
        )
        
        # Publishers
        # Use BEST_EFFORT QoS to match robot_state_publisher and servo_node
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile
        )
        
        # Timer to publish joint_states at regular intervals (50 Hz)
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        # Immediately publish initial joint_states so servo_node can receive it
        # This breaks the circular dependency: servo_node needs joint_states to calculate trajectory
        self.publish_joint_states()
        
        self.get_logger().info('Trajectory to Joint States node started')
        self.get_logger().info(f'Subscribing to: /panda_arm_controller/joint_trajectory')
        self.get_logger().info(f'Publishing to: /joint_states')
        self.get_logger().info(f'Tracking {len(self.joint_names)} joints')
        self.get_logger().info('Initial joint_states published (ready pose to avoid joint limits)')
    
    def is_valid_number(self, value):
        """Check if value is a valid number (not NaN or Inf)"""
        return isinstance(value, (int, float)) and math.isfinite(value)
        
    def trajectory_callback(self, msg: JointTrajectory):
        """Extract joint positions from trajectory message"""
        if not msg.points:
            self.get_logger().warn('Received trajectory with no points')
            return
        
        # Get the latest point (last point in trajectory)
        latest_point = msg.points[-1]
        
        # Log first received trajectory for debugging
        if not hasattr(self, '_first_trajectory_received'):
            self._first_trajectory_received = True
            self.get_logger().info(f'First trajectory received! Joints: {msg.joint_names}')
            self.get_logger().info(f'Positions: {latest_point.positions}')
        
        # Update current positions and velocities (NaN 체크 포함)
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.current_positions:
                if i < len(latest_point.positions):
                    pos = latest_point.positions[i]
                    if self.is_valid_number(pos):
                        self.current_positions[joint_name] = pos
                if i < len(latest_point.velocities):
                    vel = latest_point.velocities[i]
                    if self.is_valid_number(vel):
                        self.current_velocities[joint_name] = vel
        
        # Also publish immediately when trajectory is received
        self.publish_joint_states()

    def gripper_callback(self, msg: Float64):
        """Update gripper joints (finger_joint and related) from /gripper/position"""
        # Clamp to [0, 0.8] as in URDF (finger_joint limit upper ≈ 0.725/0.8)
        pos = max(0.0, min(float(msg.data), 0.8))
        self.gripper_position = pos

        # In URDF:
        # - finger_joint               : main driving joint (0=open, ~0.8=closed)
        # - left_inner_knuckle_joint   : mimic finger_joint * 1
        # - left_inner_finger_joint    : mimic finger_joint * -1
        # - right_inner_knuckle_joint  : mimic finger_joint * -1
        # - right_inner_finger_joint   : mimic finger_joint * 1
        # - right_outer_knuckle_joint  : mimic finger_joint * -1
        #
        # For visualization, we manually apply these relationships.

        if 'finger_joint' in self.current_positions:
            self.current_positions['finger_joint'] = pos

        if 'left_inner_knuckle_joint' in self.current_positions:
            self.current_positions['left_inner_knuckle_joint'] = pos
        if 'left_inner_finger_joint' in self.current_positions:
            self.current_positions['left_inner_finger_joint'] = -pos

        if 'right_inner_knuckle_joint' in self.current_positions:
            self.current_positions['right_inner_knuckle_joint'] = -pos
        if 'right_inner_finger_joint' in self.current_positions:
            self.current_positions['right_inner_finger_joint'] = pos

        if 'right_outer_knuckle_joint' in self.current_positions:
            self.current_positions['right_outer_knuckle_joint'] = -pos

        # Immediately publish updated joint states so RViz sees gripper motion
        self.publish_joint_states()
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        joint_state.name = self.joint_names
        # NaN 체크 후 안전한 값만 발행
        positions = []
        velocities = []
        
        for name in self.joint_names:
            pos = self.current_positions[name]
            vel = self.current_velocities[name]
            
            # NaN이나 Inf가 있으면 0으로 대체
            if not self.is_valid_number(pos):
                pos = 0.0
                self.current_positions[name] = 0.0
            if not self.is_valid_number(vel):
                vel = 0.0
                self.current_velocities[name] = 0.0
            
            positions.append(pos)
            velocities.append(vel)
        
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = []  # Not available from trajectory
        
        # /joint_states 발행
        self.joint_states_pub.publish(joint_state)

        # ===== Debug: 추정 관절 속도(rad/s) 출력 =====
        now = self.get_clock().now()
        dt = (now - self._last_debug_time).nanoseconds * 1e-9
        if dt > 0.0:
            max_abs_vel = 0.0
            sum_sq = 0.0
            for name, pos in zip(self.joint_names, positions):
                last_pos = self._last_debug_positions.get(name, pos)
                vel = (pos - last_pos) / dt
                abs_vel = abs(vel)
                if abs_vel > max_abs_vel:
                    max_abs_vel = abs_vel
                sum_sq += vel * vel
            joint_speed_l2 = math.sqrt(sum_sq)

            # 움직임이 충분히 있을 때만 로그 출력 (노이즈 방지)
            if max_abs_vel > 0.01:
                self.get_logger().info(
                    f'Estimated joint speed  max: {max_abs_vel:.2f} rad/s, '
                    f'L2-norm: {joint_speed_l2:.2f} rad/s'
                )

            self._last_debug_time = now
            self._last_debug_positions = {name: pos for name, pos in zip(self.joint_names, positions)}


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToJointStates()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down trajectory_to_joint_states')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
