#!/usr/bin/env python3
"""
Panda Teleop Omy L100 Launch File
----------------------------------
Launches omy_l100 leader arm and panda follower arm with servo control.

This launch file includes:
- omy_l100_leader_ai.launch.py (omy_l100 leader arm)
- robot_state_publisher (panda robot description and TF)
- trajectory_to_joint_states (converts trajectory to joint_states)
- servo_node (MoveIt Servo for IK)
- omy_l100_to_twist_node (FK: omy_l100 joint_states → TwistStamped)
- omy_l100_to_gripper_node (omy_l100 gripper → panda gripper)
- RViz (visualization)

Usage:
    ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py
    
Optional arguments:
    use_sim_time:=true/false           (default: false)
    use_rviz:=true/false               (default: true)
    enable_data_collection:=true/false (default: false)
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
    panda_teleop_pkg = get_package_share_directory('panda_teleop_omy_l100')
    custom_panda_pkg = get_package_share_directory('custom_panda_description')
    open_manipulator_bringup_pkg = get_package_share_directory('open_manipulator_bringup')
    
    # Paths
    urdf_file = os.path.join(custom_panda_pkg, 'urdf', 'panda_with_robotiq.urdf')
    srdf_file = os.path.join(custom_panda_pkg, 'config', 'panda_robotiq.srdf')
    rviz_config_file = os.path.join(custom_panda_pkg, 'config', 'view_robot.rviz')
    servo_config_file = os.path.join(panda_teleop_pkg, 'config', 'servo_config.yaml')
    kinematics_file = os.path.join(panda_teleop_pkg, 'config', 'kinematics.yaml')
    data_collection_config_file = os.path.join(panda_teleop_pkg, 'config', 'data_collection_config.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    enable_data_collection = LaunchConfiguration('enable_data_collection', default='false')
    # OMY L100 리더 하드웨어 포트 (기본: /dev/ttyUSB0)
    port_name = LaunchConfiguration('port_name', default='/dev/ttyUSB0')
    
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
    # Omy L100 Leader Arm
    # ========================================
    
    omy_l100_leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'omy_l100_leader_ai.launch.py'
            ])
        ]),
        launch_arguments={
            'port_name': port_name
        }.items()
    )
    
    # ========================================
    # Panda Robot
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
    # Trajectory to Joint States Node
    # servo_node가 발행하는 /panda_arm_controller/joint_trajectory를 구독하여
    # /joint_states로 변환하여 발행 (robot_state_publisher가 TF를 발행할 수 있도록)
    trajectory_to_joint_states_node = Node(
        package='panda_teleop_omy_l100',
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
                'robot_description_kinematics': kinematics_config,  # IK solver 설정
                'use_sim_time': use_sim_time
            }
        ]
    )
    
    # ========================================
    # Omy L100 to Panda Bridge Nodes
    # ========================================
    
    # Omy L100 to Twist Node (FK: omy_l100 joint_states → TwistStamped)
    omy_l100_to_twist_node = Node(
        package='panda_teleop_omy_l100',
        executable='omy_l100_to_twist_node.py',
        name='omy_l100_to_twist_node',
        output='screen',
        parameters=[{
            'base_frame': 'leader_link0',  # omy_l100 base frame
            'ee_frame': 'leader_link7',    # omy_l100 end-effector frame
            'joint_states_topic': '/leader/joint_states',
            'linear_scale': 10.0,   # 스케일 대폭 증가 (이전: 2.0)
            'angular_scale': 5.0,   # 스케일 대폭 증가 (이전: 1.0)
            'publish_rate': 100.0,   # Servo와 주파수 일치 (이전: 50.0)
            # 축 반전 파라미터 (리더암과 팔로워암 좌표계 방향이 반대일 때)
            # 1.0 = 정방향, -1.0 = 반전
            # 리더암 위로 올리면 팔로워암도 위로 가도록 Z축 반전
            'invert_linear_x': 1.0,
            'invert_linear_y': 1.0,
            'invert_linear_z': -1.0,  # Z축 반전 (위로 올리면 위로 가도록)
            'invert_angular_x': 1.0,
            'invert_angular_y': 1.0,
            'invert_angular_z': 1.0
        }]
    )
    
    # Omy L100 to Gripper Node (omy_l100 gripper → panda gripper)
    omy_l100_to_gripper_node = Node(
        package='panda_teleop_omy_l100',
        executable='omy_l100_to_gripper_node.py',
        name='omy_l100_to_gripper_node',
        output='screen',
        parameters=[{
            'omy_gripper_joint_name': 'rh_r1_joint',
            'omy_gripper_min': -1.0,  # omy_l100 gripper range
            'omy_gripper_max': 1.0,
            'panda_gripper_min': 0.0,  # panda gripper range
            'panda_gripper_max': 0.8,
            'joint_states_topic': '/leader/joint_states'
        }]
    )
    
    # Clutch Pedal Node (evdev-based - reads PCsensor FootSwitch directly)
    clutch_pedal_node = Node(
        package='panda_teleop_omy_l100',
        executable='clutch_pedal_node.py',
        name='clutch_pedal_node',
        output='screen',
        parameters=[{
            'device_name': 'PCsensor FootSwitch Keyboard',  # Auto-detect by name
            'key_code': 48  # KEY_B = 48
        }]
    )    
    # ========================================
    # Data Collection Node (for Imitation Learning)
    # ========================================
    
    # Data Collection Node (optional - enable with enable_data_collection:=true)
    data_collection_node = Node(
        package='panda_teleop_omy_l100',
        executable='data_collection_node.py',
        name='data_collection_node',
        output='screen',
        parameters=[data_collection_config_file],
        condition=IfCondition(enable_data_collection)
    )
    
    # ========================================
    # Delayed Starts (to avoid resource conflicts)
    # ========================================
    
    # Delay bridge nodes and servo_node by 3 seconds to ensure omy_l100 is ready
    # and trajectory_to_joint_states_node has time to publish initial joint_states
    delayed_control_nodes = TimerAction(
        period=3.0,
        actions=[servo_node, omy_l100_to_twist_node, omy_l100_to_gripper_node, clutch_pedal_node]
    )
    
    # Data collection node (delayed 7 seconds to ensure all systems ready)
    delayed_data_collection = TimerAction(
        period=7.0,
        actions=[data_collection_node]
    )
    
    # Delay RViz startup by 5 seconds to ensure everything is ready
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )
    
    # Servo 시작 서비스 호출 (servo_node 시작 후 추가로 2초 뒤)
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )
    
    # Servo 활성화를 더 늦춰서 servo_node가 완전히 초기화되고 
    # /joint_states를 받을 수 있도록 보장
    delayed_start_servo = TimerAction(
        period=6.0,  # servo_node 시작 후 3초 뒤 (delayed_control_nodes가 3초 후 시작하므로 총 6초)
        actions=[start_servo_service]
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
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for omy_l100 hardware connection (e.g., /dev/ttyUSB0, /dev/ttyUSB1)'
        ),
        DeclareLaunchArgument(
            'enable_data_collection',
            default_value='false',
            description='Enable data collection for imitation learning (default: false)'
        ),
        
        # Omy L100 Leader Arm (starts immediately)
        omy_l100_leader_launch,
        
        # Base Robot (starts immediately at t=0s)
        robot_state_publisher_node,
        trajectory_to_joint_states_node,  # 초기 joint_states 발행을 위해 즉시 시작
        
        # Control nodes (delayed 3 seconds at t=3s)
        delayed_control_nodes,
        
        # Start servo service (delayed 6 seconds at t=6s, after servo_node starts)
        delayed_start_servo,
        
        # Visualization (delayed 5 seconds at t=5s)
        delayed_rviz,
        
        # Data collection (delayed 7 seconds at t=7s, optional)
        delayed_data_collection
    ])
