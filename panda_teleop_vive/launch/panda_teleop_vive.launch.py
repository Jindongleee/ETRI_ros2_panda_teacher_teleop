#!/usr/bin/env python3
"""
Vive 텔레오퍼레이션 런치 파일
================================
목적: HTC Vive 컨트롤러로 Panda 로봇을 제어하기 위한 전체 시스템을 일괄 실행한다.

실행 노드 구성:
  [t=0s] vive_input               - OpenVR 소켓 서버 (Vive 드라이버)
         robot_state_publisher     - Panda URDF 기반 TF 트리 발행
         trajectory_to_joint_states - Servo 출력 궤적을 /joint_states로 변환
  [t=2s] vive_node                - ROS2 Vive 클라이언트 (100Hz)
  [t=3s] vive_ros2_bridge_node    - VRControllerData → PoseStamped + Joy 변환
         servo_node               - MoveIt Servo (IK 기반 속도 제어)
         vive_to_twist_node       - Vive 포즈 변화 → TwistStamped 변환
         vive_to_gripper_node     - Vive 버튼/트리거 → 그리퍼 명령 변환
         clutch_pedal_node        - 풋스위치 기반 클러치 안전 제어
  [t=6s] start_servo 서비스 호출  - Servo 활성화
  [t=7s] rviz2                    - 시각화 (use_rviz:=false로 비활성화 가능)

선택 인자:
  use_sim_time:=true/false           (기본: false)
  use_rviz:=true/false               (기본: true)
  enable_data_collection:=true/false  (기본: false, 모방학습 데이터 수집용)
  controller_role:=0/1               (기본: 0, 0=오른손 1=왼손)
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
    """Vive/OpenVR 실행에 필요한 환경 변수(OPENVR, LD_LIBRARY_PATH)를 구성"""
    vive_share = get_package_share_directory('vive_ros2')
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

    # === 패키지 경로 ===
    panda_desc_pkg = get_package_share_directory('custom_panda_description')
    panda_teleop_pkg = get_package_share_directory('panda_teleop_vive')
    panda_common_pkg = get_package_share_directory('panda_common')

    # === 설정 파일 경로 ===
    urdf_file = os.path.join(panda_desc_pkg, 'urdf', 'panda_with_robotiq.urdf')
    srdf_file = os.path.join(panda_desc_pkg, 'config', 'panda_robotiq.srdf')
    rviz_config_file = os.path.join(panda_desc_pkg, 'config', 'view_robot.rviz')
    servo_config_file = os.path.join(panda_teleop_pkg, 'config', 'servo_config.yaml')
    kinematics_file = os.path.join(panda_desc_pkg, 'config', 'kinematics.yaml')
    data_collection_config_file = os.path.join(panda_common_pkg, 'config', 'data_collection_config_vive.yaml')

    # === 런치 인자 ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    enable_data_collection = LaunchConfiguration('enable_data_collection', default='false')
    controller_role = LaunchConfiguration('controller_role', default='0')

    # === URDF / SRDF / Kinematics 로드 ===
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    vr_env = _get_vive_env()

    # ==================================================
    # Vive 드라이버
    # ==================================================

    # OpenVR 소켓 서버
    vive_input_process = ExecuteProcess(
        cmd=['ros2', 'run', 'vive_ros2', 'vive_input'],
        output='screen',
        additional_env=vr_env
    )

    # ROS2 Vive 클라이언트 (2초 후 시작, vive_input 준비 대기)
    vive_node_process = Node(
        package='vive_ros2',
        executable='vive_node',
        name='vive_node',
        arguments=['100'],
        output='screen'
    )
    delayed_vive_node = TimerAction(period=2.0, actions=[vive_node_process])

    # ==================================================
    # Panda 로봇 기반 노드
    # ==================================================

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

    # Servo 출력 JointTrajectory → /joint_states 변환
    trajectory_to_joint_states_node = Node(
        package='panda_common',
        executable='trajectory_to_joint_states.py',
        name='trajectory_to_joint_states',
        output='screen'
    )

    # ==================================================
    # 시각화
    # ==================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=IfCondition(use_rviz)
    )

    # ==================================================
    # MoveIt Servo (IK 기반 속도 제어)
    # ==================================================

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

    # ==================================================
    # Vive 텔레오퍼레이션 노드
    # ==================================================

    # VRControllerData → PoseStamped + Joy 브릿지
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

    # Vive 포즈 → Twist 속도 명령
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

    # Vive 버튼/트리거 → 그리퍼 명령
    vive_to_gripper_node = Node(
        package='panda_teleop_vive',
        executable='vive_to_gripper_node.py',
        name='vive_to_gripper_node',
        output='screen',
        parameters=[{
            'button_open': 1,
            'button_close': 0,
            'step': 0.005,
            'min_position': 0.0,
            'max_position': 0.8,
            'vive_buttons_topic': '/vive/controller/buttons'
        }]
    )

    # 풋스위치 기반 클러치 안전 제어
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

    # ==================================================
    # 데이터 수집 (선택, enable_data_collection:=true 시 활성화)
    # ==================================================

    data_collection_node = Node(
        package='panda_common',
        executable='data_collection_node.py',
        name='data_collection_node',
        output='screen',
        parameters=[data_collection_config_file],
        condition=IfCondition(enable_data_collection)
    )

    # ==================================================
    # 지연 시작 (노드 간 의존 순서 보장)
    # ==================================================

    # t=3s: 제어 노드
    delayed_control_nodes = TimerAction(
        period=3.0,
        actions=[vive_ros2_bridge_node, servo_node, vive_to_twist_node, vive_to_gripper_node, clutch_pedal_node]
    )

    # t=7s: RViz
    delayed_rviz = TimerAction(period=7.0, actions=[rviz_node])

    # t=6s: Servo 활성화 서비스 호출
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )
    delayed_start_servo = TimerAction(period=6.0, actions=[start_servo_service])

    # t=1s: 데이터 수집 (선택)
    delayed_data_collection = TimerAction(
        period=1.0,
        actions=[data_collection_node],
        condition=IfCondition(enable_data_collection)
    )

    # ==================================================
    # 런치 설명 반환
    # ==================================================

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='시뮬레이션(Gazebo) 시간 사용 여부'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='RViz 시각화 실행 여부'),
        DeclareLaunchArgument('enable_data_collection', default_value='false',
                              description='모방학습용 데이터 수집 활성화'),
        DeclareLaunchArgument('controller_role', default_value='0',
                              description='Vive 컨트롤러 선택 (0=오른손, 1=왼손)'),

        # 즉시 시작 (t=0s)
        vive_input_process,
        robot_state_publisher_node,
        trajectory_to_joint_states_node,

        # Vive 클라이언트 (t=2s)
        delayed_vive_node,

        # 제어 노드 (t=3s)
        delayed_control_nodes,

        # Servo 활성화 (t=6s)
        delayed_start_servo,

        # RViz (t=7s)
        delayed_rviz,

        # 데이터 수집 (t=1s, 선택)
        delayed_data_collection
    ])
