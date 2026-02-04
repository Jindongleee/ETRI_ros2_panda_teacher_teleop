#!/usr/bin/env python3
"""
Panda Teleop Vive Launch File
------------------------------
Single launch: Vive driver (vive_input + vive_node) + Panda + Servo + RViz.
Target: RViz simulation on one machine with display and Vive.

This launch file includes:
- vive_input (OpenVR, socket server) + vive_node (ROS2 client, 100 Hz)
- robot_state_publisher, trajectory_to_joint_states
- vive_ros2_bridge_node (controller_data -> PoseStamped + Joy)
- servo_node, vive_to_twist_node, vive_to_gripper_node, clutch_pedal_node
- RViz, optional data collection

Usage:
    ros2 launch panda_teleop_vive panda_teleop_vive.launch.py

Optional: use_rviz:=false, enable_data_collection:=true, controller_role:=1 (left)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _get_vive_env():
    """Build OPENVR and LD_LIBRARY_PATH for vive_input (same as set_vr_env.sh)."""
    vive_share = get_package_share_directory('vive_ros2')
    # .../install/vive_ros2/share/vive_ros2 -> workspace root = .../install parent
    install_dir = os.path.dirname(os.path.dirname(vive_share))
    workspace_root = os.path.dirname(install_dir)
    openvr_path = os.path.join(workspace_root, 'libraries', 'openvr')
    steam = os.path.expanduser('~/.steam')
    steamvr = os.path.join(steam, 'steam', 'steamapps', 'common', 'SteamVR')
    ld_parts = [
        '/usr/lib', '/usr/lib32',
        os.path.join(openvr_path, 'lib', 'linux32'),
        os.path.join(openvr_path, 'lib', 'linux64'),
        os.path.join(steam, 'ubuntu12_32', 'steam-runtime', 'i386', 'lib', 'i386-linux-gnu'),
        os.path.join(steam, 'ubuntu12_32', 'steam-runtime', 'amd64', 'lib', 'x86_64-linux-gnu'),
        os.path.join(steamvr, 'bin', 'linux32'),
        os.path.join(steamvr, 'bin', 'linux64'),
        os.path.join(steamvr, 'drivers', 'lighthouse', 'bin', 'linux32'),
        os.path.join(steamvr, 'drivers', 'lighthouse', 'bin', 'linux64'),
    ]
    ld_path = os.environ.get('LD_LIBRARY_PATH', '') + ':' + ':'.join(ld_parts)
    return {'OPENVR': openvr_path, 'LD_LIBRARY_PATH': ld_path}


def generate_launch_description():
    panda_desc_pkg = get_package_share_directory('custom_panda_description')
    panda_teleop_pkg = get_package_share_directory('panda_teleop_vive')
    panda_common_pkg = get_package_share_directory('panda_common')

    urdf_file = os.path.join(panda_desc_pkg, 'urdf', 'panda_with_robotiq.urdf')
    srdf_file = os.path.join(panda_desc_pkg, 'config', 'panda_robotiq.srdf')
    rviz_config_file = os.path.join(panda_desc_pkg, 'config', 'view_robot.rviz')
    servo_config_file = os.path.join(panda_teleop_pkg, 'config', 'servo_config.yaml')
    kinematics_file = os.path.join(panda_desc_pkg, 'config', 'kinematics.yaml')
    data_collection_config_file = os.path.join(panda_common_pkg, 'config', 'data_collection_config_vive.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    enable_data_collection = LaunchConfiguration('enable_data_collection', default='false')
    controller_role = LaunchConfiguration('controller_role', default='0')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    vr_env = _get_vive_env()

    # ========================================
    # Vive Driver (vive_input = OpenVR/socket server, then vive_node = ROS2 client)
    # ========================================
    vive_input_process = ExecuteProcess(
        cmd=['ros2', 'run', 'vive_ros2', 'vive_input'],
        output='screen',
        additional_env=vr_env
    )

    vive_node_process = Node(
        package='vive_ros2',
        executable='vive_node',
        name='vive_node',
        arguments=['100'],
        output='screen'
    )
    delayed_vive_node = TimerAction(period=2.0, actions=[vive_node_process])

    # ========================================
    # Panda Robot
    # ========================================
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

    trajectory_to_joint_states_node = Node(
        package='panda_common',
        executable='trajectory_to_joint_states.py',
        name='trajectory_to_joint_states',
        output='screen'
    )

    # ========================================
    # Visualization
    # ========================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # RViz가 갑자기 꺼져도 전체 런치를 죽이지 않도록 respawn 활성화
        respawn=True,
        respawn_delay=2.0,
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=IfCondition(use_rviz)
    )

    # ========================================
    # MoveIt Servo
    # ========================================
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
                'robot_description_kinematics': kinematics_config,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # ========================================
    # Vive Bridge & Teleop Nodes
    # ========================================
    vive_ros2_bridge_node = Node(
        package='panda_teleop_vive',
        executable='vive_ros2_bridge_node.py',
        name='vive_ros2_bridge_node',
        output='screen',
        parameters=[{
            'controller_data_topic': 'controller_data',
            'pose_out_topic': '/vive/controller/pose',
            'buttons_out_topic': '/vive/controller/buttons',
            'controller_role': controller_role
        }]
    )

    vive_to_twist_node = Node(
        package='panda_teleop_vive',
        executable='vive_to_twist_node.py',
        name='vive_to_twist_node',
        output='screen',
        parameters=[{
            'linear_scale': 6.0,
            'angular_scale': 4.0,
            'frame_id': 'world',
            'use_clutch': True,
            'vive_pose_topic': '/vive/controller/pose',
            'vive_buttons_topic': '/vive/controller/buttons',
            'deadman_button': 1,
        }]
    )

    vive_to_gripper_node = Node(
        package='panda_teleop_vive',
        executable='vive_to_gripper_node.py',
        name='vive_to_gripper_node',
        output='screen',
        parameters=[{
            'button_open': 1,
            'button_close': 0,
            'step': 0.02,
            'min_position': 0.0,
            'max_position': 0.8,
            'vive_buttons_topic': '/vive/controller/buttons'
        }]
    )

    clutch_pedal_node = Node(
        package='panda_common',
        executable='clutch_pedal_node.py',
        name='clutch_pedal_node',
        output='screen',
        parameters=[{
            'device_name': 'PCsensor FootSwitch Keyboard',
            'key_code': 48
        }]
    )

    # ========================================
    # Data Collection (optional)
    # ========================================
    data_collection_node = Node(
        package='panda_common',
        executable='data_collection_node.py',
        name='data_collection_node',
        output='screen',
        parameters=[data_collection_config_file],
        condition=IfCondition(enable_data_collection)
    )

    # ========================================
    # Delayed Starts (align with omy_l100 style)
    # ========================================
    delayed_control_nodes = TimerAction(
        period=3.0,
        actions=[vive_ros2_bridge_node, servo_node, vive_to_twist_node, vive_to_gripper_node, clutch_pedal_node]
    )

    delayed_rviz = TimerAction(period=7.0, actions=[rviz_node])

    # Servo 활성화 (omy_l100과 동일: control 노드 3초 후 → 6초에 1회)
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )
    delayed_start_servo = TimerAction(period=6.0, actions=[start_servo_service])

    delayed_data_collection = TimerAction(
        period=1.0,
        actions=[data_collection_node],
        condition=IfCondition(enable_data_collection)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('enable_data_collection', default_value='false', description='Enable data collection'),
        DeclareLaunchArgument('controller_role', default_value='0', description='Vive controller: 0=right, 1=left'),
        # Vive driver: vive_input first (OpenVR/socket), vive_node 2s later (ROS2 client)
        vive_input_process,
        delayed_vive_node,
        robot_state_publisher_node,
        trajectory_to_joint_states_node,
        delayed_control_nodes,
        delayed_start_servo,
        delayed_rviz,
        delayed_data_collection
    ])
