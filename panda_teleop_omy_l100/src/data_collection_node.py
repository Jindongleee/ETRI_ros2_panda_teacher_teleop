#!/usr/bin/env python3

"""
Data Collection Node for Imitation Learning
--------------------------------------------
Collects State-Action pairs at 15Hz for robot imitation learning.

Data structure per sample:
{
  "timestamp": ROS2 Time (nanoseconds),
  "seq": sequence number,
  "episode_id": "episode_001",
  
  "state": {
    "ee_pose": [x, y, z, qx, qy, qz, qw],  # Panda EE pose
    "joints": [j1, j2, j3, j4, j5, j6, j7]  # Panda joint angles
  },
  
  "action": {
    "delta_twist": [vx, vy, vz, wx, wy, wz]  # Velocity command
  }
}

Note: The next sample's state is implicitly the result of this action.
      state[i+1] is the result of executing action[i] from state[i].

Episode Management:
- Generates random target points in workspace
- Starts episode when clutch is activated
- Ends episode when EE reaches target and stays stationary for 3 seconds
- Collects exactly 3 episodes per session
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer
import numpy as np
import json
import os
from datetime import datetime
from enum import Enum
from pathlib import Path


class EpisodeState(Enum):
    """Episode state machine"""
    WAITING_CLUTCH = 0      # Waiting for clutch activation
    COLLECTING = 1          # Collecting data
    WAITING_STATIONARY = 2  # Target reached, waiting for 3s stationary
    EPISODE_END = 3         # Episode finished


class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        
        # ===== Parameters =====
        self.declare_parameter('max_episodes', 3)
        self.declare_parameter('controller_type', 'omy_l100')
        self.declare_parameter('collection_rate', 15.0)  # Hz
        
        # Workspace limits
        self.declare_parameter('workspace_x_min', 0.3)
        self.declare_parameter('workspace_x_max', 0.65)
        self.declare_parameter('workspace_y_min', -0.3)
        self.declare_parameter('workspace_y_max', 0.3)
        self.declare_parameter('workspace_z_min', 0.15)
        self.declare_parameter('workspace_z_max', 0.55)
        
        # Episode termination conditions
        self.declare_parameter('reach_threshold', 0.02)  # 2cm
        self.declare_parameter('velocity_threshold', 0.008)  # 8mm/s
        self.declare_parameter('angular_velocity_threshold', 0.015)  # rad/s
        self.declare_parameter('stationary_duration', 3.0)  # seconds
        self.declare_parameter('max_episode_duration', 60.0)  # seconds
        
        # Storage
        self.declare_parameter('output_dir', './data')
        
        # Visualization
        self.declare_parameter('enable_rviz_markers', True)
        
        # Get parameters
        self.max_episodes = self.get_parameter('max_episodes').value
        self.controller_type = self.get_parameter('controller_type').value
        self.collection_rate = self.get_parameter('collection_rate').value
        
        self.workspace_limits = {
            'x_min': self.get_parameter('workspace_x_min').value,
            'x_max': self.get_parameter('workspace_x_max').value,
            'y_min': self.get_parameter('workspace_y_min').value,
            'y_max': self.get_parameter('workspace_y_max').value,
            'z_min': self.get_parameter('workspace_z_min').value,
            'z_max': self.get_parameter('workspace_z_max').value,
        }
        
        self.reach_threshold = self.get_parameter('reach_threshold').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.angular_velocity_threshold = self.get_parameter('angular_velocity_threshold').value
        self.stationary_duration = self.get_parameter('stationary_duration').value
        self.max_episode_duration = self.get_parameter('max_episode_duration').value
        
        self.output_dir = self.get_parameter('output_dir').value
        self.enable_rviz_markers = self.get_parameter('enable_rviz_markers').value
        
        # ===== State Variables =====
        self.current_episode = 0
        self.episode_state = EpisodeState.WAITING_CLUTCH
        self.collection_complete = False
        self.clutch_active = False
        
        # Target point
        self.target_position = None
        
        # Buffers for latest messages
        self.latest_joint_state = None
        self.latest_twist_cmd = None
        self.latest_ee_pose = None
        
        # Episode data buffer
        self.episode_data = []
        self.episode_seq = 0
        self.episode_start_time = None
        self.stationary_timer = 0.0
        
        # Session info
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = None
        self.episode_metrics = []
        
        # ===== TF =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ===== Subscribers =====
        # Use BEST_EFFORT QoS to match trajectory_to_joint_states publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            qos_profile
        )
        
        self.twist_cmd_sub = self.create_subscription(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            self.twist_cmd_callback,
            10
        )
        
        self.clutch_sub = self.create_subscription(
            Bool,
            '/clutch/active',
            self.clutch_callback,
            10
        )
        
        # ===== Publishers =====
        if self.enable_rviz_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                '/data_collection/markers',
                10
            )
        
        # ===== Timer =====
        timer_period = 1.0 / self.collection_rate  # 15Hz = 0.0667s
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # ===== Initialization =====
        self.setup_session_directory()
        self.generate_new_target()
        
        # Log startup info
        self.get_logger().info('='*60)
        self.get_logger().info('🤖 Data Collection Node Started!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Controller Type: {self.controller_type}')
        self.get_logger().info(f'Max Episodes: {self.max_episodes}')
        self.get_logger().info(f'Collection Rate: {self.collection_rate} Hz')
        self.get_logger().info(f'Workspace: X[{self.workspace_limits["x_min"]:.2f}, {self.workspace_limits["x_max"]:.2f}], '
                             f'Y[{self.workspace_limits["y_min"]:.2f}, {self.workspace_limits["y_max"]:.2f}], '
                             f'Z[{self.workspace_limits["z_min"]:.2f}, {self.workspace_limits["z_max"]:.2f}]')
        self.get_logger().info(f'Reach Threshold: {self.reach_threshold*100:.1f} cm')
        self.get_logger().info(f'Velocity Threshold: {self.velocity_threshold*1000:.1f} mm/s')
        self.get_logger().info(f'Stationary Duration: {self.stationary_duration:.1f} s')
        self.get_logger().info(f'Output Directory: {self.session_dir}')
        self.get_logger().info('='*60)
        self.get_logger().info(f'🎯 Target Position: [{self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f}]')
        self.get_logger().info('⏳ Waiting for clutch activation...')
        self.get_logger().info('='*60)
    
    def setup_session_directory(self):
        """Create session directory for data storage"""
        session_name = f'session_{self.session_timestamp}'
        self.session_dir = os.path.join(self.output_dir, self.controller_type, session_name)
        Path(self.session_dir).mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Session directory created: {self.session_dir}')
    
    def generate_new_target(self):
        """Generate random target position in workspace"""
        x = np.random.uniform(self.workspace_limits['x_min'], self.workspace_limits['x_max'])
        y = np.random.uniform(self.workspace_limits['y_min'], self.workspace_limits['y_max'])
        z = np.random.uniform(self.workspace_limits['z_min'], self.workspace_limits['z_max'])
        self.target_position = np.array([x, y, z])
        
        # Publish marker
        if self.enable_rviz_markers:
            self.publish_markers()
    
    def clutch_callback(self, msg: Bool):
        """Handle clutch state changes"""
        prev_clutch = self.clutch_active
        self.clutch_active = msg.data
        
        # Clutch activated: start new episode
        if not prev_clutch and self.clutch_active:
            if self.episode_state == EpisodeState.WAITING_CLUTCH and not self.collection_complete:
                self.start_new_episode()
    
    def start_new_episode(self):
        """Start a new episode"""
        self.current_episode += 1
        self.episode_state = EpisodeState.COLLECTING
        self.episode_data = []
        self.episode_seq = 0
        self.episode_start_time = self.get_clock().now()
        self.stationary_timer = 0.0
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'📝 Episode {self.current_episode}/{self.max_episodes} STARTED')
        self.get_logger().info(f'🎯 Target: [{self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f}]')
        self.get_logger().info('='*60)
    
    def joint_states_callback(self, msg: JointState):
        """Store latest joint state"""
        self.latest_joint_state = msg
        # Debug: Log first few callbacks to verify it's working
        if not hasattr(self, '_joint_states_callback_count'):
            self._joint_states_callback_count = 0
        self._joint_states_callback_count += 1
        if self._joint_states_callback_count <= 3:
            self.get_logger().info(f'✅ joint_states_callback received! Count: {self._joint_states_callback_count}, Joints: {len(msg.position)}')
    
    def twist_cmd_callback(self, msg: TwistStamped):
        """Store latest twist command"""
        self.latest_twist_cmd = msg
    
    def get_ee_pose(self):
        """Get current end-effector pose via TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'panda_link0',
                'panda_link8',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            pose = {
                'position': np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]),
                'orientation': np.array([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])
            }
            return pose
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {str(e)}', throttle_duration_sec=5.0)
            return None
    
    def timer_callback(self):
        """Main data collection loop at 15Hz"""
        if self.collection_complete:
            return
        
        # Update EE pose
        self.latest_ee_pose = self.get_ee_pose()
        
        # Publish markers
        if self.enable_rviz_markers:
            self.publish_markers()
        
        # State machine
        if self.episode_state == EpisodeState.WAITING_CLUTCH:
            # Just waiting, do nothing
            pass
        
        elif self.episode_state == EpisodeState.COLLECTING:
            self.collect_data_sample()
            self.check_episode_progress()
        
        elif self.episode_state == EpisodeState.WAITING_STATIONARY:
            self.collect_data_sample()
            self.check_stationary_condition()
        
        elif self.episode_state == EpisodeState.EPISODE_END:
            self.finish_episode()
    
    def collect_data_sample(self):
        """Collect one data sample: {state, action}"""
        # Check if we have all required data (with debug logging)
        missing_data = []
        if self.latest_joint_state is None:
            missing_data.append('joint_state')
        if self.latest_twist_cmd is None:
            missing_data.append('twist_cmd')
        if self.latest_ee_pose is None:
            missing_data.append('ee_pose')
        
        if missing_data:
            self.get_logger().warn(
                f'⚠️ Missing data for sample collection: {", ".join(missing_data)} | '
                f'Episode: {self.current_episode}, State: {self.episode_state.name}',
                throttle_duration_sec=2.0
            )
            return
        
        # Check workspace bounds
        if not self.is_in_workspace(self.latest_ee_pose['position']):
            self.get_logger().warn(
                f'⚠️ EE out of workspace! Position: [{self.latest_ee_pose["position"][0]:.3f}, '
                f'{self.latest_ee_pose["position"][1]:.3f}, {self.latest_ee_pose["position"][2]:.3f}]',
                throttle_duration_sec=2.0
            )
            return
        
        # Get current state
        state = {
            'ee_pose': np.concatenate([
                self.latest_ee_pose['position'],
                self.latest_ee_pose['orientation']
            ]).tolist(),  # [x, y, z, qx, qy, qz, qw]
            'joints': list(self.latest_joint_state.position[:7])  # First 7 joints
        }
        
        # Get action
        action = {
            'delta_twist': [
                self.latest_twist_cmd.twist.linear.x,
                self.latest_twist_cmd.twist.linear.y,
                self.latest_twist_cmd.twist.linear.z,
                self.latest_twist_cmd.twist.angular.x,
                self.latest_twist_cmd.twist.angular.y,
                self.latest_twist_cmd.twist.angular.z
            ]
        }
        
        # Create sample with current state and action
        # Note: The next sample's state will be the result of this action
        sample = {
            'timestamp': self.get_clock().now().nanoseconds,
            'seq': self.episode_seq,
            'episode_id': f'episode_{self.current_episode:03d}',
            'state': state,
            'action': action
        }
        
        self.episode_data.append(sample)
        self.episode_seq += 1
        
        # 실시간 state-action 출력
        ee_pos = state['ee_pose'][:3]  # [x, y, z]
        linear_vel = action['delta_twist'][:3]  # [vx, vy, vz]
        angular_vel = action['delta_twist'][3:]  # [wx, wy, wz]
        linear_vel_norm = np.linalg.norm(linear_vel)
        angular_vel_norm = np.linalg.norm(angular_vel)
        
        self.get_logger().info(
            f'[Episode {self.current_episode:03d} | Seq {self.episode_seq:04d} | Total: {len(self.episode_data):04d}] '
            f'State: EE=[{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] | '
            f'Action: v=[{linear_vel[0]:.4f}, {linear_vel[1]:.4f}, {linear_vel[2]:.4f}] '
            f'|v|={linear_vel_norm:.4f}, w=[{angular_vel[0]:.4f}, {angular_vel[1]:.4f}, {angular_vel[2]:.4f}] '
            f'|w|={angular_vel_norm:.4f}'
        )
    
    def check_episode_progress(self):
        """Check if target is reached"""
        if self.latest_ee_pose is None:
            return
        
        # Calculate distance to target
        distance = np.linalg.norm(self.latest_ee_pose['position'] - self.target_position)
        
        # Log progress periodically
        if self.episode_seq % 15 == 0:  # Every 1 second at 15Hz
            self.get_logger().info(f'[Episode {self.current_episode}/{self.max_episodes}] Distance: {distance*100:.1f} cm | Samples: {len(self.episode_data)}')
        
        # Check if reached
        if distance < self.reach_threshold:
            self.get_logger().info(f'🎯 Target reached! Distance: {distance*100:.2f} cm')
            self.episode_state = EpisodeState.WAITING_STATIONARY
            self.stationary_timer = 0.0
        
        # Check timeout
        elapsed = (self.get_clock().now() - self.episode_start_time).nanoseconds * 1e-9
        if elapsed > self.max_episode_duration:
            self.get_logger().warn(f'⏰ Episode timeout ({self.max_episode_duration}s)! Ending episode...')
            self.episode_state = EpisodeState.EPISODE_END
    
    def check_stationary_condition(self):
        """Check if EE is stationary for required duration"""
        if self.latest_twist_cmd is None or self.latest_ee_pose is None:
            return
        
        # Calculate velocities
        linear_vel = np.linalg.norm([
            self.latest_twist_cmd.twist.linear.x,
            self.latest_twist_cmd.twist.linear.y,
            self.latest_twist_cmd.twist.linear.z
        ])
        
        angular_vel = np.linalg.norm([
            self.latest_twist_cmd.twist.angular.x,
            self.latest_twist_cmd.twist.angular.y,
            self.latest_twist_cmd.twist.angular.z
        ])
        
        # Check if stationary
        is_stationary = (linear_vel < self.velocity_threshold and 
                        angular_vel < self.angular_velocity_threshold)
        
        # Update timer
        dt = 1.0 / self.collection_rate
        if is_stationary:
            self.stationary_timer += dt
            
            # Log progress
            if int(self.stationary_timer * 10) % 5 == 0:  # Every 0.5s
                self.get_logger().info(f'⏸️  Stationary: {self.stationary_timer:.1f}s / {self.stationary_duration:.1f}s')
            
            # Check if duration met
            if self.stationary_timer >= self.stationary_duration:
                self.get_logger().info(f'✅ Stationary condition met! ({self.stationary_duration:.1f}s)')
                self.episode_state = EpisodeState.EPISODE_END
        else:
            # Reset timer if not stationary
            if self.stationary_timer > 0:
                self.get_logger().info('❌ Movement detected! Resetting stationary timer...')
            self.stationary_timer = 0.0
    
    def finish_episode(self):
        """Finish current episode and save data"""
        episode_duration = (self.get_clock().now() - self.episode_start_time).nanoseconds * 1e-9
        
        # Save episode data
        self.save_episode()
        
        # Save metadata
        metadata = {
            'episode_id': f'episode_{self.current_episode:03d}',
            'controller_type': self.controller_type,
            'target_position': self.target_position.tolist(),
            'start_time': datetime.fromtimestamp(self.episode_start_time.nanoseconds * 1e-9).isoformat(),
            'end_time': datetime.now().isoformat(),
            'duration': episode_duration,
            'num_samples': len(self.episode_data),
            'success': True
        }
        self.episode_metrics.append(metadata)
        self.save_episode_metadata(metadata)
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'✅ Episode {self.current_episode}/{self.max_episodes} COMPLETED!')
        self.get_logger().info(f'📊 Samples: {len(self.episode_data)} | Duration: {episode_duration:.1f}s')
        self.get_logger().info('='*60)
        
        # Check if all episodes complete
        if self.current_episode >= self.max_episodes:
            self.complete_collection()
        else:
            # Prepare for next episode
            self.generate_new_target()
            self.episode_state = EpisodeState.WAITING_CLUTCH
            self.get_logger().info(f'🎯 New Target: [{self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f}]')
            self.get_logger().info('⏳ Waiting for clutch activation...')
    
    def save_episode(self):
        """Save episode data to JSONL file"""
        filename = f'episode_{self.current_episode:03d}.jsonl'
        filepath = os.path.join(self.session_dir, filename)
        
        with open(filepath, 'w') as f:
            for sample in self.episode_data:
                f.write(json.dumps(sample) + '\n')
        
        self.get_logger().info(f'💾 Data saved: {filepath}')
    
    def save_episode_metadata(self, metadata):
        """Save episode metadata"""
        filename = f'episode_{self.current_episode:03d}_meta.json'
        filepath = os.path.join(self.session_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def complete_collection(self):
        """Complete data collection and save summary"""
        self.collection_complete = True
        
        # Save session summary
        summary = {
            'controller_type': self.controller_type,
            'session_timestamp': self.session_timestamp,
            'total_episodes': self.max_episodes,
            'total_samples': sum(len(self.episode_data) for _ in range(self.max_episodes)),
            'episodes': self.episode_metrics
        }
        
        summary_path = os.path.join(self.session_dir, 'session_summary.json')
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)
        
        self.get_logger().info('='*60)
        self.get_logger().info('🎉 DATA COLLECTION COMPLETED!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Controller: {self.controller_type}')
        self.get_logger().info(f'Total Episodes: {self.max_episodes}/{self.max_episodes}')
        self.get_logger().info(f'Success Rate: 100%')
        self.get_logger().info(f'Data saved in: {self.session_dir}')
        self.get_logger().info('='*60)
        self.get_logger().info('Node will remain active. Press Ctrl+C to shutdown.')
        self.get_logger().info('='*60)
    
    def is_in_workspace(self, position):
        """Check if position is within workspace"""
        return (self.workspace_limits['x_min'] <= position[0] <= self.workspace_limits['x_max'] and
                self.workspace_limits['y_min'] <= position[1] <= self.workspace_limits['y_max'] and
                self.workspace_limits['z_min'] <= position[2] <= self.workspace_limits['z_max'])
    
    def publish_markers(self):
        """Publish RViz markers for visualization"""
        # Check if target position is initialized
        if self.target_position is None:
            return
        
        marker_array = MarkerArray()
        
        # Target marker (red sphere)
        target_marker = Marker()
        target_marker.header.frame_id = 'panda_link0'
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = 'target'
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = float(self.target_position[0])
        target_marker.pose.position.y = float(self.target_position[1])
        target_marker.pose.position.z = float(self.target_position[2])
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.05  # 5cm diameter
        target_marker.scale.y = 0.05
        target_marker.scale.z = 0.05
        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 0.8
        marker_array.markers.append(target_marker)
        
        # Distance text
        if self.latest_ee_pose is not None:
            distance = np.linalg.norm(self.latest_ee_pose['position'] - self.target_position)
            
            text_marker = Marker()
            text_marker.header.frame_id = 'panda_link0'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'distance'
            text_marker.id = 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(self.target_position[0])
            text_marker.pose.position.y = float(self.target_position[1])
            text_marker.pose.position.z = float(self.target_position[2]) + 0.1
            text_marker.text = f'Episode {self.current_episode}/{self.max_episodes}\nDist: {distance*100:.1f}cm'
            text_marker.scale.z = 0.03
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down data_collection_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
