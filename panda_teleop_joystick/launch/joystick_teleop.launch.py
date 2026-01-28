#!/usr/bin/env python3
"""
Complete servo control launch file with visualization
Launches all necessary nodes for joystick-controlled robot with RViz.

This launch file includes:
- robot_state_publisher (robot description and TF)
- RViz (visualization)
- joy_node (joystick input)
- servo_node (MoveIt Servo)
- joy_to_twist_node (joystick to servo converter)
- clutch_pedal_node (safety control - robot only moves when pedal is held)

Usage:
    ros2 launch custom_panda_description servo_complete.launch.py
    
Optional arguments:
    use_sim_time:=true/false  (default: false)
    use_rviz:=true/false      (default: true)
    
Safety:
    Robot only moves when clutch pedal is held down (hold-to-activate safety feature)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package directories
    panda_desc_pkg = get_package_share_directory('custom_panda_description')
    panda_teleop_pkg = get_package_share_directory('panda_teleop_joystick')
    panda_common_pkg = get_package_share_directory('panda_common')
    
    # Paths
    urdf_file = os.path.join(panda_desc_pkg, 'urdf', 'panda_with_robotiq.urdf')
    srdf_file = os.path.join(panda_desc_pkg, 'config', 'panda_robotiq.srdf')
    rviz_config_file = os.path.join(panda_desc_pkg, 'config', 'view_robot.rviz')
    servo_config_file = os.path.join(panda_teleop_pkg, 'config', 'servo_config.yaml')
    kinematics_file = os.path.join(panda_desc_pkg, 'config', 'kinematics.yaml')
    data_collection_config_file = os.path.join(panda_common_pkg, 'config', 'data_collection_config_joystick.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    enable_data_collection = LaunchConfiguration('enable_data_collection', default='false')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # Read SRDF file
    with open(srdf_file, 'r') as infp:
        robot_description_semantic_content = infp.read()
    
    # Read kinematics.yaml file (for IK solver configuration)
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)
    
    # ========================================
    # Base Robot
    # ========================================
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # ========================================
    # Visualization
    # ========================================
    # Trajectory to Joint States Node (from panda_common)
    # servo_node가 발행하는 /panda_arm_controller/joint_trajectory를 구독하여
    # /joint_states로 변환하여 발행 (robot_state_publisher가 TF를 발행할 수 있도록)
    trajectory_to_joint_states_node = Node(
        package='panda_common',
        executable='trajectory_to_joint_states.py',
        name='trajectory_to_joint_states',
        output='screen'
    )
    
    # RViz (conditional - can be disabled)
    # Delayed start to ensure robot_state_publisher is ready
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=IfCondition(use_rviz)
    )
    
    # ========================================
    # Joystick Control
    # ========================================
    
    # Joy Node (for DualShock 3 controller)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',  # Change if your joystick is on a different device
            'deadzone': 0.05,   
            'autorepeat_rate': 20.0
        }]
    )
    
    # ========================================
    # MoveIt Servo
    # ========================================
    
    # Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_config_file,
            {
                'robot_description': robot_description_content,
                'robot_description_semantic': robot_description_semantic_content,
                'robot_description_kinematics': kinematics_config,  # IK solver 설정 추가!
                'use_sim_time': use_sim_time
            }
        ]
    )
    
    # Joystick to Twist Node (converts joy messages to twist commands)
    # Now uses clutch pedal for safety control instead of joystick deadman button
    joy_to_twist_node = Node(
        package='panda_teleop_joystick',
        executable='joy_to_twist_node.py',
        name='joy_to_twist_node',
        output='screen',
        parameters=[{
            # Scales are further normalized in the node to keep commands within [-1, 1]
            'linear_scale': 10.0,
            'angular_scale': 10.0,
            'frame_id': 'panda_link0',  # End-effector 기준
            'use_clutch': True,  # Use clutch pedal for safety control
            
            # Button mapping for button-based controls
            'button_linear_z_up': 6,      # L1 button (Linear Z up)
            'button_linear_z_down': 8,    # L2 button (Linear Z down)
            'button_angular_z_pos': 7,    # R1 button (Angular Z positive)
            'button_angular_z_neg': 9,    # R2 button (Angular Z negative)
            
            # Joystick axis mapping
            'axis_linear_x': 1,      # Left stick X (좌우)
            'axis_linear_y': 0,      # Left stick Y (전후)
            'axis_angular_x': 2,     # Right stick Y (Roll)
            'axis_angular_y': 3,     # Right stick X (Pitch)
            
            # Axis inversion (1 = normal, -1 = inverted)
            'invert_linear_x': 1,
            'invert_linear_y': -1,
            'invert_angular_x': 1,
            'invert_angular_y': 1,
            
            # Deadzone for joystick axes
            'axis_deadzone': 0.25
        }]
    )

    # Joystick to Gripper Node (converts joy buttons to gripper open/close)
    joy_to_gripper_node = Node(
        package='panda_teleop_joystick',
        executable='joy_to_gripper_node.py',
        name='joy_to_gripper_node',
        output='screen',
        parameters=[{
            'button_open': 1,   # Circle button
            'button_close': 3,  # Square button
            'step': 0.02,
            'min_position': 0.0,
            'max_position': 0.8
        }]
    )
    
    # Clutch Pedal Node (evdev-based - reads PCsensor FootSwitch directly)
    # This provides safety control: robot only moves when clutch pedal is held
    clutch_pedal_node = Node(
        package='panda_common',
        executable='clutch_pedal_node.py',
        name='clutch_pedal_node',
        output='screen',
        parameters=[{
            'device_name': 'PCsensor FootSwitch Keyboard',  # Auto-detect by name
            # 'device_path': '/dev/input/event18',  # Or specify path directly
            'key_code': 48  # KEY_B = 48
        }]
    )
    
    # ========================================
    # Delayed Starts (to avoid resource conflicts)
    # ========================================
    
    # Delay control nodes by 2 seconds to ensure trajectory_to_joint_states_node 
    # has time to publish initial joint_states before servo_node starts
    delayed_control_nodes = TimerAction(
        period=2.0,
        actions=[joy_node, servo_node, joy_to_twist_node, joy_to_gripper_node, clutch_pedal_node]
    )
    
    # Delay RViz startup by 3 seconds to ensure everything is ready
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )
    
    # Servo 시작 서비스 호출 (servo_node 시작 후 2초 뒤)
    # delayed_control_nodes가 1초 후에 시작되므로, 총 3초 후에 호출
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )
    
    # Servo 활성화를 더 늦춰서 servo_node가 완전히 초기화되고 
    # /joint_states를 받을 수 있도록 보장
    delayed_start_servo = TimerAction(
        period=5.0,  # servo_node 시작 후 3초 뒤 (delayed_control_nodes가 2초 후 시작하므로 총 5초)
        actions=[start_servo_service]
    )
    
    # Data Collection Node (optional, controlled by enable_data_collection parameter)
    data_collection_node = Node(
        package='panda_common',
        executable='data_collection_node.py',
        name='data_collection_node',
        output='screen',
        parameters=[data_collection_config_file],
        condition=IfCondition(enable_data_collection)
    )
    
    # Data collection starts delayed (4 seconds) to ensure all other nodes are ready
    delayed_data_collection = TimerAction(
        period=4.0,
        actions=[data_collection_node],
        condition=IfCondition(enable_data_collection)
    )
    
    # ========================================
    # Launch Description
    # ========================================
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'enable_data_collection',
            default_value='false',
            description='Enable data collection for imitation learning'
        ),
        
        # Base Robot (starts immediately at t=0s)
        robot_state_publisher_node,
        trajectory_to_joint_states_node,  # 초기 joint_states 발행을 위해 즉시 시작
        
        # Control nodes (delayed 1 second at t=1s)
        delayed_control_nodes,
        
        # Start servo service (delayed 3 seconds at t=3s, after servo_node starts)
        delayed_start_servo,
        
        # Visualization (delayed 3 seconds at t=3s)
        delayed_rviz,
        
        # Data Collection (delayed 4 seconds at t=4s, optional)
        delayed_data_collection
    ])
