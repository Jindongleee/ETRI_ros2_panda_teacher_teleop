#!/usr/bin/env python3
"""
조이스틱 텔레오퍼레이션 런치 파일
====================================
목적: 조이스틱으로 Panda 로봇을 제어하기 위한 전체 시스템을 일괄 실행한다.

실행 노드 구성:
  [t=0s] robot_state_publisher     - URDF 기반 로봇 모델 및 TF 트리 발행
         trajectory_to_joint_states - Servo 출력 궤적을 /joint_states로 변환
  [t=2s] joy_node                  - 조이스틱 디바이스 입력 수신
         servo_node                - MoveIt Servo (IK 기반 속도 제어)
         joy_to_twist_node         - 조이스틱 → TwistStamped 변환
         joy_to_gripper_node       - 조이스틱 버튼 → 그리퍼 명령 변환
         clutch_pedal_node         - 풋스위치 기반 클러치 안전 제어
  [t=3s] rviz2                     - 시각화 (use_rviz:=false로 비활성화 가능)
  [t=5s] start_servo 서비스 호출   - Servo 활성화

선택 인자:
  use_sim_time:=true/false           (기본: false)
  use_rviz:=true/false               (기본: true)
  enable_data_collection:=true/false  (기본: false, 모방학습 데이터 수집용)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # === 패키지 경로 ===
    panda_desc_pkg = get_package_share_directory('custom_panda_description')
    panda_teleop_pkg = get_package_share_directory('panda_teleop_joystick')
    panda_common_pkg = get_package_share_directory('panda_common')

    # === 설정 파일 경로 ===
    urdf_file = os.path.join(panda_desc_pkg, 'urdf', 'panda_with_robotiq.urdf')
    srdf_file = os.path.join(panda_desc_pkg, 'config', 'panda_robotiq.srdf')
    rviz_config_file = os.path.join(panda_desc_pkg, 'config', 'view_robot.rviz')
    servo_config_file = os.path.join(panda_teleop_pkg, 'config', 'servo_config.yaml')
    kinematics_file = os.path.join(panda_desc_pkg, 'config', 'kinematics.yaml')
    data_collection_config_file = os.path.join(panda_common_pkg, 'config', 'data_collection_config_joystick.yaml')

    # === 런치 인자 ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    enable_data_collection = LaunchConfiguration('enable_data_collection', default='false')

    # === URDF / SRDF / Kinematics 로드 ===
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    with open(srdf_file, 'r') as infp:
        robot_description_semantic_content = infp.read()
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # ==================================================
    # 로봇 기반 노드 (즉시 시작)
    # ==================================================

    # URDF 기반 TF 트리 발행
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
    # 조이스틱 입력
    # ==================================================

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
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
    # 텔레오퍼레이션 노드
    # ==================================================

    # 조이스틱 → Twist 변환 (축/버튼 매핑은 런치 파라미터로 설정)
    joy_to_twist_node = Node(
        package='panda_teleop_joystick',
        executable='joy_to_twist_node.py',
        name='joy_to_twist_node',
        output='screen',
        parameters=[{
            'linear_scale': 0.5,
            'angular_scale': 0.3,
            'frame_id': 'panda_link0',
            'use_clutch': True,
            'button_linear_z_up': 8,
            'button_linear_z_down': 6,
            'button_angular_z_pos': 7,
            'button_angular_z_neg': 9,
            'axis_linear_x': 1,
            'axis_linear_y': 0,
            'axis_angular_x': 2,
            'axis_angular_y': 3,
            'invert_linear_x': 1,
            'invert_linear_y': -1,
            'invert_angular_x': 1,
            'invert_angular_y': -1,
            'axis_deadzone': 0.00
        }]
    )

    # 조이스틱 버튼 → 그리퍼 열기/닫기
    joy_to_gripper_node = Node(
        package='panda_teleop_joystick',
        executable='joy_to_gripper_node.py',
        name='joy_to_gripper_node',
        output='screen',
        parameters=[{
            'button_open': 1,
            'button_close': 3,
            'step': 0.02,
            'min_position': 0.0,
            'max_position': 0.8
        }]
    )

    # 풋스위치 기반 클러치 안전 제어 (발판을 밟고 있어야 로봇 동작)
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
    # 지연 시작 (노드 간 의존 순서 보장)
    # ==================================================

    # t=2s: 제어 노드 (trajectory_to_joint_states가 초기 joint_states 발행 후)
    delayed_control_nodes = TimerAction(
        period=2.0,
        actions=[joy_node, servo_node, joy_to_twist_node, joy_to_gripper_node, clutch_pedal_node]
    )

    # t=3s: RViz
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    # t=5s: Servo 활성화 서비스 호출
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )
    delayed_start_servo = TimerAction(
        period=5.0,
        actions=[start_servo_service]
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

    delayed_data_collection = TimerAction(
        period=4.0,
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

        # 즉시 시작 (t=0s)
        robot_state_publisher_node,
        trajectory_to_joint_states_node,

        # 제어 노드 (t=2s)
        delayed_control_nodes,

        # Servo 활성화 (t=5s)
        delayed_start_servo,

        # RViz (t=3s)
        delayed_rviz,

        # 데이터 수집 (t=4s, 선택)
        delayed_data_collection
    ])
