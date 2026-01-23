# 시스템 토픽 정보 (Topic Information)

## 시스템 개요
이 시스템은 Panda 로봇 팔을 제어하는 ROS2 기반 텔레오퍼레이션 시스템입니다.
두 가지 제어 모드를 지원합니다:
1. **조이스틱 제어 모드** (servo_complete.launch.py)
2. **OMY L100 리더암 텔레오퍼레이션 모드** (panda_teleop_omy_l100.launch.py)

**공통 안전 기능**: 두 모드 모두 클러치 페달(PCsensor FootSwitch)을 사용하여 안전 제어를 수행합니다.
- 클러치 페달을 누르고 있을 때만 로봇이 움직입니다 (hold-to-activate)
- 클러치를 놓으면 즉시 정지합니다

---

## 1. 조이스틱 제어 모드 (Joystick Control Mode)

### 1.1 joy_node
- **Pub**: `/joy` (MT: `sensor_msgs/msg/Joy`)
  - 조이스틱 입력 데이터 (버튼, 축 값)

### 1.2 joy_to_twist_node
- **Sub**: 
  - `/joy` (MT: `sensor_msgs/msg/Joy`)
  - `/clutch/active` (MT: `std_msgs/msg/Bool`)
    - 클러치 페달 상태 (True: 활성, False: 비활성) - 안전 제어용
- **Pub**: 
  - `/servo_node/delta_twist_cmds` (MT: `geometry_msgs/msg/TwistStamped`)
    - 조이스틱 입력을 변환한 엔드이펙터 속도 명령 (클러치가 활성화된 경우에만)
  - `/joy_to_twist/status` (MT: `std_msgs/msg/Float64`)
    - 클러치/제어 상태 (1.0: 활성, 0.0: 비활성)

### 1.3 clutch_pedal_node
- **Pub**: `/clutch/active` (MT: `std_msgs/msg/Bool`)
  - 클러치 페달 상태 (True: 활성/로봇 제어 활성화, False: 비활성/일시정지)
  - PCsensor FootSwitch 하드웨어에서 직접 읽음 (evdev 기반)
  - 안전 기능: 클러치를 누르고 있을 때만 로봇이 움직임 (hold-to-activate)

### 1.4 joy_to_gripper_node
- **Sub**: `/joy` (MT: `sensor_msgs/msg/Joy`)
- **Pub**: `/gripper/position` (MT: `std_msgs/msg/Float64`)
  - 그리퍼 위치 명령 (0.0: 열림, 0.8: 닫힘)

### 1.5 trajectory_to_joint_states (custom_panda_description)
- **Sub**: 
  - `/panda_arm_controller/joint_trajectory` (MT: `trajectory_msgs/msg/JointTrajectory`)
  - `/gripper/position` (MT: `std_msgs/msg/Float64`)
- **Pub**: `/joint_states` (MT: `sensor_msgs/msg/JointState`)
  - 로봇의 모든 관절 상태 (위치, 속도)

### 1.6 servo_node (MoveIt Servo)
- **Sub**: 
  - `/servo_node/delta_twist_cmds` (MT: `geometry_msgs/msg/TwistStamped`)
  - `/joint_states` (MT: `sensor_msgs/msg/JointState`)
- **Pub**: `/panda_arm_controller/joint_trajectory` (MT: `trajectory_msgs/msg/JointTrajectory`)
  - 역기구학으로 계산된 관절 궤적 명령

### 1.7 robot_state_publisher
- **Sub**: `/joint_states` (MT: `sensor_msgs/msg/JointState`)
- **Pub**: 
  - `/tf` (MT: `tf2_msgs/msg/TFMessage`)
  - `/tf_static` (MT: `tf2_msgs/msg/TFMessage`)

---

## 2. OMY L100 리더암 텔레오퍼레이션 모드 (OMY L100 Leader-Follower Teleoperation Mode)

### 2.1 omy_l100_leader_ai (리더암 시스템)
- **Pub**: `/leader/joint_states` (MT: `sensor_msgs/msg/JointState`)
  - OMY L100 리더암의 관절 상태

### 2.2 omy_l100_to_twist_node
- **Sub**: 
  - `/leader/joint_states` (MT: `sensor_msgs/msg/JointState`)
  - `/clutch/active` (MT: `std_msgs/msg/Bool`)
    - 클러치 페달 상태 (True: 활성, False: 비활성)
- **Pub**: `/servo_node/delta_twist_cmds` (MT: `geometry_msgs/msg/TwistStamped`)
  - 리더암 엔드이펙터 움직임을 변환한 팔로워암 속도 명령

### 2.3 omy_l100_to_gripper_node
- **Sub**: `/leader/joint_states` (MT: `sensor_msgs/msg/JointState`)
- **Pub**: `/gripper/position` (MT: `std_msgs/msg/Float64`)
  - 리더암 그리퍼를 변환한 팔로워암 그리퍼 위치 명령

### 2.4 clutch_pedal_node
- **Pub**: `/clutch/active` (MT: `std_msgs/msg/Bool`)
  - 클러치 페달 상태 (True: 활성/텔레오퍼레이션 활성화, False: 비활성/일시정지)

### 2.5 trajectory_to_joint_states (panda_teleop_omy_l100)
- **Sub**: 
  - `/panda_arm_controller/joint_trajectory` (MT: `trajectory_msgs/msg/JointTrajectory`)
  - `/gripper/position` (MT: `std_msgs/msg/Float64`)
- **Pub**: `/joint_states` (MT: `sensor_msgs/msg/JointState`)
  - 로봇의 모든 관절 상태 (위치, 속도)

### 2.6 servo_node (MoveIt Servo)
- **Sub**: 
  - `/servo_node/delta_twist_cmds` (MT: `geometry_msgs/msg/TwistStamped`)
  - `/joint_states` (MT: `sensor_msgs/msg/JointState`)
- **Pub**: `/panda_arm_controller/joint_trajectory` (MT: `trajectory_msgs/msg/JointTrajectory`)
  - 역기구학으로 계산된 관절 궤적 명령

### 2.7 robot_state_publisher
- **Sub**: `/joint_states` (MT: `sensor_msgs/msg/JointState`)
- **Pub**: 
  - `/tf` (MT: `tf2_msgs/msg/TFMessage`)
  - `/tf_static` (MT: `tf2_msgs/msg/TFMessage`)

---

## 3. 공통 토픽 요약

### 주요 입력 토픽 (Input Topics)
- `/joy` - 조이스틱 입력 (조이스틱 모드)
- `/leader/joint_states` - 리더암 관절 상태 (텔레오퍼레이션 모드)
- `/clutch/active` - 클러치 페달 상태 (두 모드 모두에서 안전 제어용)
- `/panda_arm_controller/joint_trajectory` - 관절 궤적 명령
- `/gripper/position` - 그리퍼 위치 명령
- `/joint_states` - 로봇 관절 상태

### 주요 출력 토픽 (Output Topics)
- `/servo_node/delta_twist_cmds` - 엔드이펙터 속도 명령
- `/panda_arm_controller/joint_trajectory` - 관절 궤적 명령
- `/joint_states` - 로봇 관절 상태
- `/gripper/position` - 그리퍼 위치 명령
- `/tf`, `/tf_static` - 좌표계 변환 정보
- `/joy_to_twist/status` - 조이스틱 제어 상태
- `/clutch/active` - 클러치 페달 상태

---

## 4. 메시지 타입 상세 정보

### sensor_msgs/msg/Joy
- 조이스틱 버튼 및 축 입력 데이터

### geometry_msgs/msg/TwistStamped
- 타임스탬프가 포함된 속도 명령 (선속도, 각속도)

### trajectory_msgs/msg/JointTrajectory
- 관절 궤적 명령 (관절 이름, 위치, 속도, 가속도)

### sensor_msgs/msg/JointState
- 관절 상태 (이름, 위치, 속도, 토크)

### std_msgs/msg/Float64
- 단일 실수 값 (그리퍼 위치, 상태 값)

### std_msgs/msg/Bool
- 불리언 값 (클러치 상태)

### tf2_msgs/msg/TFMessage
- 좌표계 변환 정보

---

## 5. 시스템 아키텍처 흐름

### 조이스틱 모드:
```
clutch_pedal_node → /clutch/active
joy_node → joy_to_twist_node → servo_node → trajectory_to_joint_states → robot_state_publisher
         → joy_to_gripper_node → trajectory_to_joint_states
         (joy_to_twist_node는 /clutch/active를 구독하여 안전 제어)
```

### 텔레오퍼레이션 모드:
```
omy_l100_leader → omy_l100_to_twist_node → servo_node → trajectory_to_joint_states → robot_state_publisher
                → omy_l100_to_gripper_node → trajectory_to_joint_states
clutch_pedal_node → (모든 제어 노드에 클러치 신호 전달)
```

---

## 참고사항
- 모든 토픽은 ROS2 표준 메시지 타입을 사용합니다.
- QoS 설정은 노드별로 다를 수 있습니다 (일반적으로 BEST_EFFORT 또는 RELIABLE).
- TF는 robot_state_publisher를 통해 자동으로 발행됩니다.
- **안전 기능**: 두 모드 모두 클러치 페달을 사용하여 안전 제어를 수행합니다.
  - 클러치 페달을 누르고 있을 때만 로봇이 움직입니다 (hold-to-activate).
  - 클러치를 놓으면 즉시 정지합니다.
