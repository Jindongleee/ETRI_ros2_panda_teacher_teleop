#!/usr/bin/env python3
"""
OMY L100 텔레오퍼레이션 런치 파일
====================================
목적: OMY L100 리더 암과 Panda 팔로워 암 간 텔레오퍼레이션 시스템을 일괄 실행한다.

실행 노드 구성:
  [t=0s] omy_l100_leader_ai         - OMY L100 리더 암 하드웨어 드라이버
         robot_state_publisher       - Panda URDF 기반 TF 트리 발행
         trajectory_to_joint_states  - Servo 출력 궤적을 /joint_states로 변환
  [t=3s] servo_node                 - MoveIt Servo (IK 기반 속도 제어)
         omy_l100_to_twist_node     - OMY FK → Panda TwistStamped 변환
         omy_l100_to_gripper_node   - OMY 그리퍼 → Panda 그리퍼 매핑
         clutch_pedal_node          - 풋스위치 기반 클러치 안전 제어
  [t=5s] rviz2                      - 시각화 (use_rviz:=false로 비활성화 가능)
  [t=6s] start_servo 서비스 호출    - Servo 활성화
  [t=7s] data_collection_node       - 데이터 수집 (enable_data_collection:=true 시)

선택 인자:
  use_sim_time:=true/false           (기본: false)
  use_rviz:=true/false               (기본: true)
  port_name:=/dev/ttyUSBx            (기본: /dev/ttyUSB0)
  enable_data_collection:=true/false  (기본: false, 모방학습 데이터 수집용)
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

    # === 패키지 경로 ===
    panda_teleop_pkg = get_package_share_directory('panda_teleop_omy_l100')
    custom_panda_pkg = get_package_share_directory('custom_panda_description')
    panda_common_pkg = get_package_share_directory('panda_common')

    # === 설정 파일 경로 ===
    urdf_file = os.path.join(custom_panda_pkg, 'urdf', 'panda_with_robotiq.urdf')
    srdf_file = os.path.join(custom_panda_pkg, 'config', 'panda_robotiq.srdf')
    rviz_config_file = os.path.join(custom_panda_pkg, 'config', 'view_robot.rviz')
    servo_config_file = os.path.join(panda_teleop_pkg, 'config', 'servo_config.yaml')
    kinematics_file = os.path.join(panda_teleop_pkg, 'config', 'kinematics.yaml')
    data_collection_config_file = os.path.join(panda_common_pkg, 'config', 'data_collection_config_omy_l100.yaml')

    # === 런치 인자 ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    enable_data_collection = LaunchConfiguration('enable_data_collection', default='false')
    port_name = LaunchConfiguration('port_name', default='/dev/ttyUSB0')

    # === URDF / SRDF / Kinematics 로드 ===
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    with open(srdf_file, 'r') as infp:
        robot_description_semantic_content = infp.read()
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # ==================================================
    # OMY L100 리더 암 (하드웨어 드라이버)
    # ==================================================

    omy_l100_leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch', 'omy_l100_leader_ai.launch.py'
            ])
        ]),
        launch_arguments={'port_name': port_name}.items()
    )

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
    # OMY L100 → Panda 브릿지 노드
    # ==================================================

    # FK → Twist 변환 (OMY 엔드이펙터 모션 → Panda 속도 명령)
    omy_l100_to_twist_node = Node(
        package='panda_teleop_omy_l100',
        executable='omy_l100_to_twist_node.py',
        name='omy_l100_to_twist_node',
        output='screen',
        parameters=[{
            'base_frame': 'leader_link0',
            'ee_frame': 'leader_link7',
            'joint_states_topic': '/leader/joint_states',
            'linear_scale': 5.0,
            'angular_scale': 5.0,
            'max_linear_speed': 20.0,
            'max_angular_speed': 20.0,
            'invert_linear_x': 1.0,
            'invert_linear_y': -1.0,
            'invert_linear_z': 1.0,
            'invert_angular_x': -1.0,
            'invert_angular_y': 1.0,   # 오리엔테이션 Y(Pitch): 1.0=정방향, -1.0=반전
            'invert_angular_z': 1.0
        }]
    )

    # 그리퍼 매핑 (OMY rh_r1_joint → Panda finger_joint)
    omy_l100_to_gripper_node = Node(
        package='panda_teleop_omy_l100',
        executable='omy_l100_to_gripper_node.py',
        name='omy_l100_to_gripper_node',
        output='screen',
        parameters=[{
            'omy_gripper_joint_name': 'rh_r1_joint',
            'omy_gripper_min': -1.0,
            'omy_gripper_max': 1.0,
            'panda_gripper_min': 0.0,
            'panda_gripper_max': 0.8,
            'joint_states_topic': '/leader/joint_states'
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

    # t=3s: 제어 노드 (OMY 드라이버 및 trajectory_to_joint_states 준비 후)
    delayed_control_nodes = TimerAction(
        period=3.0,
        actions=[servo_node, omy_l100_to_twist_node, omy_l100_to_gripper_node, clutch_pedal_node]
    )

    # t=7s: 데이터 수집
    delayed_data_collection = TimerAction(
        period=7.0,
        actions=[data_collection_node]
    )

    # t=5s: RViz
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    # t=6s: Servo 활성화 서비스 호출
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )
    delayed_start_servo = TimerAction(
        period=6.0,
        actions=[start_servo_service]
    )

    # ==================================================
    # 런치 설명 반환
    # ==================================================

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='시뮬레이션(Gazebo) 시간 사용 여부'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='RViz 시각화 실행 여부'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0',
                              description='OMY L100 하드웨어 포트 (예: /dev/ttyUSB0)'),
        DeclareLaunchArgument('enable_data_collection', default_value='false',
                              description='모방학습용 데이터 수집 활성화'),

        # 즉시 시작 (t=0s)
        omy_l100_leader_launch,
        robot_state_publisher_node,
        trajectory_to_joint_states_node,

        # 제어 노드 (t=3s)
        delayed_control_nodes,

        # Servo 활성화 (t=6s)
        delayed_start_servo,

        # RViz (t=5s)
        delayed_rviz,

        # 데이터 수집 (t=7s, 선택)
        delayed_data_collection
    ])
