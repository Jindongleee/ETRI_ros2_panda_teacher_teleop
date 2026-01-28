# 🏗️ Panda Robot Teleoperation System Architecture

**Workspace**: `/home/etri/ros2_ws`  
**Last Updated**: 2026-01-28  
**ROS2 Distribution**: Humble

---

## 📋 Table of Contents

1. [시스템 개요](#-시스템-개요)
2. [패키지 구조](#-패키지-구조)
3. [OMY L100 컨트롤러 아키텍처](#-omy-l100-컨트롤러-아키텍처)
4. [조이스틱 컨트롤러 아키텍처](#-조이스틱-컨트롤러-아키텍처)
5. [공통 시스템](#-공통-시스템)
6. [데이터 수집 시스템](#-데이터-수집-시스템)
7. [토픽 레퍼런스](#-토픽-레퍼런스)
8. [의존성 그래프](#-의존성-그래프)

---

## 🎯 시스템 개요

Panda 로봇과 Robotiq 그리퍼를 위한 **멀티 컨트롤러 텔레오퍼레이션 시스템**입니다.

### 주요 기능
- ✅ **2가지 컨트롤러 지원**: OMY L100 리더암, 조이스틱
- ✅ **실시간 제어**: MoveIt Servo 기반 IK/FK
- ✅ **안전 메커니즘**: 클러치 페달 안전 제어
- ✅ **데이터 수집**: Imitation Learning용 자동 데이터 수집
- ✅ **모듈식 설계**: 새 컨트롤러 쉽게 추가 가능

### 지원 로봇
- **Follower Arm**: Franka Emika Panda (7-DOF)
- **End-Effector**: Robotiq 2F-85 Gripper
- **Leader Arm**: ROBOTIS OMY L100 (7-DOF, 선택적)

---

## 📦 패키지 구조

### 패키지 분류

```
ros2_ws/
│
├── 🎯 Core Packages (우리가 만든 패키지)
│   ├── panda_common/              # 공통 유틸리티 노드
│   ├── panda_teleop_omy_l100/     # OMY L100 컨트롤러
│   ├── panda_teleop_joystick/     # 조이스틱 컨트롤러
│   └── custom_panda_description/  # Panda + Robotiq URDF
│
├── 🤖 Hardware Interface (OMY L100용)
│   ├── open_manipulator_bringup/
│   ├── open_manipulator_description/
│   ├── open_manipulator_collision/
│   ├── dynamixel_hardware_interface/
│   ├── dynamixel_interfaces/
│   └── DynamixelSDK/
│
├── 🎛️ ROS2 Controllers (OMY L100용)
│   └── ros2_controller/
│       ├── om_gravity_compensation_controller/
│       ├── om_joint_trajectory_command_broadcaster/
│       └── om_spring_actuator_controller/
│
├── 🛠️ Utilities
│   ├── realtime_tools/
│   └── scripts/                   # 데이터 분석 도구
│
└── 📊 Data
    └── data/
        ├── omy_l100/             # OMY L100 데이터
        └── joystick/             # 조이스틱 데이터
```

---

### 📦 Core Packages 상세

#### **panda_common** (공통 유틸리티)
> 모든 컨트롤러가 공유하는 노드와 기능

```
panda_common/
├── src/
│   ├── data_collection_node.py        # 데이터 수집 (15Hz)
│   ├── clutch_pedal_node.py           # 안전 클러치 인터페이스
│   └── trajectory_to_joint_states.py  # 궤적 → joint_states 변환
└── config/
    ├── data_collection_config_omy_l100.yaml
    └── data_collection_config_joystick.yaml
```

**역할**:
- 🔒 안전 제어 (클러치 페달)
- 📊 데이터 수집 (모든 컨트롤러 공통)
- 🔄 메시지 변환 (궤적 → 상태)

---

#### **panda_teleop_omy_l100** (OMY L100 전용)
> Leader-Follower 텔레오퍼레이션

```
panda_teleop_omy_l100/
├── src/
│   ├── omy_l100_to_twist_node.py      # FK: Joint → Twist
│   ├── omy_l100_to_gripper_node.py    # 그리퍼 매핑
│   ├── clutch_control_node.py         # 클러치 제어 로직
│   └── disable_collision_flag_node.py # 충돌 플래그 관리
├── config/
│   ├── servo_config.yaml              # MoveIt Servo 설정
│   └── kinematics.yaml                # IK 솔버 설정
└── launch/
    └── panda_teleop_omy_l100.launch.py
```

**역할**:
- 🤖 리더암(OMY L100) 모션 캡처
- 🔄 Forward Kinematics (관절 → 엔드이펙터 속도)
- 🎯 스케일링 및 매핑

---

#### **panda_teleop_joystick** (조이스틱 전용)
> 게임 컨트롤러 기반 제어

```
panda_teleop_joystick/
├── src/
│   ├── joy_to_twist_node.py           # 조이스틱 → Twist
│   └── joy_to_gripper_node.py         # 그리퍼 제어
├── config/
│   └── servo_config.yaml
└── launch/
    └── joystick_teleop.launch.py
```

**조이스틱 매핑**:
- **Left Stick**: Linear X/Y (전후/좌우)
- **Right Stick**: Linear Z (상하)
- **L1/L2**: Angular X (Roll 회전)
- **Triangle/Cross**: 그리퍼 열기/닫기

**역할**:
- 🎮 조이스틱 입력 → 속도 명령 변환
- ⚖️ 정규화 및 속도 제한
- 🔒 클러치 기반 안전 제어

---

#### **custom_panda_description** (로봇 정의)
> Panda + Robotiq 통합 URDF

```
custom_panda_description/
├── urdf/
│   └── panda_with_robotiq.urdf        # 통합 로봇 모델
├── meshes/                             # Robotiq STL 파일
├── config/
│   ├── panda_robotiq.srdf             # MoveIt 설정
│   ├── kinematics.yaml                # IK 솔버
│   └── view_robot.rviz                # RViz 설정
└── README.md
```

**특징**:
- `gripper_tip_link`: 그리퍼 손끝 중앙 (엔드이펙터)
- `robotiq_85_base_link`에서 +11cm 오프셋
- MoveIt 그룹: `panda_arm`, `gripper`

---

## 🤖 OMY L100 컨트롤러 아키텍처

### 시스템 다이어그램

```
┌─────────────────────────────────────────────────────────────────────┐
│                      OMY L100 Leader Arm                            │
│                    (ROBOTIS 7-DOF Arm)                              │
└────────────┬────────────────────────────────────────────────────────┘
             │
             ↓ /leader/joint_states (sensor_msgs/JointState)
             │   [joint1...joint6, rh_r1_joint]
             │
    ┌────────┴────────┐
    │                 │
    ↓                 ↓
┌───────────────┐  ┌──────────────────┐
│ omy_l100_to   │  │ omy_l100_to      │
│ _twist_node   │  │ _gripper_node    │
│               │  │                  │
│ • FK 계산     │  │ • Gripper 매핑   │
│ • Scaling     │  │ • 범위 변환      │
│ • Clutch 체크 │  │                  │
└───────┬───────┘  └────────┬─────────┘
        │                   │
        ↓                   ↓
    /servo_node/        /gripper/position
    delta_twist_cmds    (std_msgs/Float64)
    (TwistStamped)          │
        │                   │
        ↓                   │
┌───────────────────────────┼──────────────────────────────────┐
│            MoveIt Servo   │                                  │
│         (IK Solver)       │                                  │
└───────────┬───────────────┼──────────────────────────────────┘
            │               │
            ↓               │
    /panda_arm_controller/  │
    joint_trajectory        │
    (JointTrajectory)       │
            │               │
            ↓               ↓
┌────────────────────────────────────────────────────────────┐
│         trajectory_to_joint_states                         │
│         (panda_common)                                     │
│                                                            │
│  • Trajectory → JointState 변환                            │
│  • Gripper position 통합                                   │
└─────────────┬──────────────────────────────────────────────┘
              │
              ↓ /joint_states (sensor_msgs/JointState)
              │   [panda_joint1...7, finger_joint, ...]
              │
┌─────────────┴──────────────────────────────────────────────┐
│          robot_state_publisher                             │
│          (TF Publisher)                                    │
└─────────────┬──────────────────────────────────────────────┘
              │
              ↓ /tf, /tf_static
              │
┌─────────────┴──────────────────────────────────────────────┐
│                    RViz2                                    │
│              (Visualization)                                │
└─────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│              clutch_pedal_node                             │
│              (panda_common)                                │
│                                                            │
│  • PCsensor FootSwitch 읽기                                │
│  • /clutch/active 발행 (Bool)                             │
│                                                            │
│  → omy_l100_to_twist_node에서 구독                        │
│  → data_collection_node에서 구독                          │
└────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│           data_collection_node (Optional)                  │
│           (panda_common)                                   │
│                                                            │
│  • State: TF lookup (gripper_tip_link)                    │
│  • Action: /servo_node/delta_twist_cmds                   │
│  • 15Hz 수집, 자동 episode 관리                            │
│  • 저장: data/omy_l100/session_*/episode_*.jsonl          │
└────────────────────────────────────────────────────────────┘
```

### 노드 상세 설명

#### 1. **omy_l100_to_twist_node**
**입력**:
- `/leader/joint_states` (sensor_msgs/JointState)
- `/clutch/active` (std_msgs/Bool)

**출력**:
- `/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped)

**기능**:
1. **Forward Kinematics**: 리더암 관절 각도 → 엔드이펙터 포즈 계산
2. **속도 계산**: 이전 포즈와 비교하여 속도 추정
3. **스케일링**: 
   - Linear: 0.4x (안전한 속도)
   - Angular: 0.4x
4. **클러치 체크**: 클러치가 활성화되었을 때만 명령 발행
5. **프레임 변환**: `gripper_tip_link` 기준

**파라미터**:
- `leader_ee_frame`: "leader_link7"
- `follower_ee_frame`: "gripper_tip_link"
- `linear_scale`: 0.4
- `angular_scale`: 0.4

---

#### 2. **omy_l100_to_gripper_node**
**입력**:
- `/leader/joint_states` (rh_r1_joint)

**출력**:
- `/gripper/position` (std_msgs/Float64)

**기능**:
- 리더암 그리퍼 관절(`rh_r1_joint`) → Robotiq 그리퍼 위치
- 범위 변환: [-0.5, 0.5] → [0.0, 0.8]

---

#### 3. **MoveIt Servo**
**입력**:
- `/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped)

**출력**:
- `/panda_arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)

**기능**:
- **Inverse Kinematics**: Twist → Joint velocities
- **충돌 회피**: MoveIt planning scene 활용
- **Singularity 회피**
- **관절 한계 체크**

**설정**: `panda_teleop_omy_l100/config/servo_config.yaml`
- `publish_period`: 0.02 (50Hz)
- `incoming_command_timeout`: 0.1
- `ee_frame_name`: gripper_tip_link

---

## 🎮 조이스틱 컨트롤러 아키텍처

### 시스템 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                  Joystick (PS4/Xbox)                        │
│                                                             │
│  Left Stick:  Linear X/Y   (전후/좌우)                      │
│  Right Stick: Linear Z     (상하)                           │
│  L1/L2:       Angular X    (Roll 회전)                      │
│  △/✕:        Gripper       (열기/닫기)                      │
└────────────┬────────────────────────────────────────────────┘
             │
             ↓ /joy (sensor_msgs/Joy)
             │   axes[6], buttons[13]
             │
    ┌────────┴────────┐
    │                 │
    ↓                 ↓
┌───────────────┐  ┌──────────────────┐
│ joy_to_twist  │  │ joy_to_gripper   │
│ _node         │  │ _node            │
│               │  │                  │
│ • Axis 매핑   │  │ • Button 매핑    │
│ • 정규화      │  │ • 증분 제어      │
│ • Clutch 체크 │  │                  │
│ • Dead zone   │  │                  │
└───────┬───────┘  └────────┬─────────┘
        │                   │
        ↓                   ↓
    /servo_node/        /gripper/position
    delta_twist_cmds    (std_msgs/Float64)
    (TwistStamped)          │
        │                   │
        │   ┌───────────────┘
        │   │
        ↓   ↓
┌─────────────────────────────────────────────────────────────┐
│            MoveIt Servo + trajectory_to_joint_states        │
│            (OMY L100과 동일한 흐름)                         │
└─────────────────────────────────────────────────────────────┘
             │
             ↓
        RViz2 시각화

┌────────────────────────────────────────────────────────────┐
│              clutch_pedal_node                             │
│              (panda_common)                                │
│                                                            │
│  클러치를 밟고 있을 때만 로봇 움직임 허용                   │
│  → joy_to_twist_node가 구독                                │
└────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────┐
│           data_collection_node (Optional)                  │
│           (panda_common)                                   │
│                                                            │
│  저장: data/joystick/session_*/episode_*.jsonl            │
└────────────────────────────────────────────────────────────┘
```

### 노드 상세 설명

#### 1. **joy_to_twist_node**
**입력**:
- `/joy` (sensor_msgs/Joy)
- `/clutch/active` (std_msgs/Bool)

**출력**:
- `/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped)

**조이스틱 매핑**:
```yaml
Linear:
  X (전후):  axis 1 (Left Stick Y)   → max 0.5 m/s
  Y (좌우):  axis 0 (Left Stick X)   → max 0.5 m/s
  Z (상하):  axis 3 (Right Stick Y)  → max 0.5 m/s

Angular:
  X (Roll):  button 4 (L1) + / button 6 (L2) - → max 1.0 rad/s
  Y (Pitch): 0.0 (사용 안 함)
  Z (Yaw):   0.0 (사용 안 함)
```

**기능**:
1. **Dead Zone**: ±0.1 이하 무시
2. **정규화**: 최대 속도 제한
3. **벡터 정규화**: 모든 축의 최대값이 1.0 초과 시 전체 스케일링
4. **클러치 안전**: 클러치 비활성화 시 명령 중단

**파라미터**:
- `frame_id`: "gripper_tip_link"
- `max_linear`: 0.5 m/s
- `max_angular`: 1.0 rad/s
- `dead_zone`: 0.1

---

#### 2. **joy_to_gripper_node**
**입력**:
- `/joy` (sensor_msgs/Joy)

**출력**:
- `/gripper/position` (std_msgs/Float64)

**버튼 매핑**:
- **Triangle (button 2)**: 그리퍼 열기 (+0.05)
- **Cross (button 0)**: 그리퍼 닫기 (-0.05)

**기능**:
- 증분 제어 (버튼 누를 때마다 조금씩 변경)
- 범위 제한: [0.0, 0.8]

---

## 🔧 공통 시스템

### 1. **clutch_pedal_node** (panda_common)

#### 하드웨어
- **장치**: PCsensor FootSwitch
- **인터페이스**: evdev (USB HID Keyboard)
- **키코드**: KEY_B (48)

#### 기능
```python
while True:
    event = read_footswitch()
    if event.key_code == KEY_B:
        if event.value == 1:  # Pressed
            publish(/clutch/active, True)
        elif event.value == 0:  # Released
            publish(/clutch/active, False)
```

#### 출력
- `/clutch/active` (std_msgs/Bool)
  - `True`: 로봇 제어 활성화
  - `False`: 로봇 정지 (안전)

#### 사용처
- `omy_l100_to_twist_node`: 클러치 체크
- `joy_to_twist_node`: 클러치 체크
- `data_collection_node`: 데이터 수집 활성화 판단

---

### 2. **trajectory_to_joint_states** (panda_common)

#### 입력
- `/panda_arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)
- `/gripper/position` (std_msgs/Float64)

#### 출력
- `/joint_states` (sensor_msgs/JointState)
  - Panda 관절 7개: panda_joint1...7
  - Robotiq 관절 6개: finger_joint, knuckle_joints

#### 기능
1. **궤적 → 상태 변환**: MoveIt Servo의 출력을 ROS 표준 메시지로 변환
2. **그리퍼 통합**: 그리퍼 명령을 joint_states에 추가
3. **TF 발행 지원**: robot_state_publisher가 TF를 계산할 수 있도록

#### 중요성
- `robot_state_publisher`는 `/joint_states`를 구독하여 TF 발행
- TF 없이는 RViz 시각화 불가능
- 데이터 수집 시 TF lookup 필수

---

### 3. **robot_state_publisher** (Standard ROS2)

#### 입력
- `/joint_states` (sensor_msgs/JointState)
- `robot_description` (URDF)

#### 출력
- `/tf` (tf2_msgs/TFMessage) - 동적 변환
- `/tf_static` (tf2_msgs/TFMessage) - 정적 변환

#### 발행 프레임
```
panda_link0 (base)
  └─ panda_link1
      └─ panda_link2
          └─ panda_link3
              └─ panda_link4
                  └─ panda_link5
                      └─ panda_link6
                          └─ panda_link7
                              └─ panda_link8
                                  └─ robotiq_85_base_link
                                      └─ gripper_tip_link ✨
                                          └─ [fingers...]
```

**gripper_tip_link**: 그리퍼 손끝 중앙 (엔드이펙터)
- `robotiq_85_base_link`에서 +11cm (Z축)

---

## 📊 데이터 수집 시스템

### 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│              data_collection_node                           │
│              (panda_common)                                 │
└─────────────────────────────────────────────────────────────┘
          │                    │                    │
          ↓                    ↓                    ↓
    Subscribe:          Subscribe:          Subscribe:
    /clutch/active      /servo_node/        TF Lookup
    (Bool)              delta_twist_cmds    (gripper_tip_link)
                        (TwistStamped)
          │                    │                    │
          └────────────────────┴────────────────────┘
                               │
                               ↓
                    Timer Callback (15Hz)
                               │
                ┌──────────────┴──────────────┐
                │                             │
                ↓                             ↓
         State 수집:                    Action 수집:
         • EE Pose (x,y,z,qx,qy,qz,qw)  • Twist (vx,vy,vz,wx,wy,wz)
         • Joint Angles (7 joints)      
                │                             │
                └──────────────┬──────────────┘
                               ↓
                      Episode 관리
                               │
            ┌──────────────────┼──────────────────┐
            ↓                  ↓                  ↓
       Episode Start     Episode Recording   Episode End
       • Target 생성    • JSONL 저장         • Meta 저장
       • 파일 생성       • 15Hz rate          • 통계 계산
                               │
                               ↓
                data/[controller]/session_*/
                ├── episode_001.jsonl
                ├── episode_001_meta.json
                ├── episode_002.jsonl
                └── ...
```

### Episode 관리

#### Episode 시작 조건
1. 클러치 활성화 (`/clutch/active == True`)
2. 이전 episode 종료됨

#### Episode 종료 조건
1. **목표 도달**: 
   - 목표 위치 2cm 이내
   - 3초간 정지 상태 유지
2. **타임아웃**: 60초 경과
3. **클러치 해제**: 안전을 위해 즉시 종료

#### Target 생성
```python
# 현재 위치 기준 랜덤 오프셋
target_x = current_x + random.uniform(-0.2, 0.2)  # ±20cm
target_y = current_y + random.uniform(-0.2, 0.2)
target_z = current_z + random.uniform(-0.1, 0.1)  # ±10cm

# RViz에 초록색 구체로 표시
```

### 데이터 포맷

#### episode_XXX.jsonl
```jsonl
{"timestamp": 1234.56, "state": {"ee_pose": [x,y,z,qx,qy,qz,qw], "joint_angles": [j1...j7]}, "action": {"twist": [vx,vy,vz,wx,wy,wz]}}
{"timestamp": 1234.62, "state": {...}, "action": {...}}
...
```

#### episode_XXX_meta.json
```json
{
  "episode_id": 1,
  "start_time": "2026-01-28T10:03:26.123456",
  "end_time": "2026-01-28T10:04:11.654321",
  "duration_seconds": 45.53,
  "num_samples": 683,
  "controller_type": "omy_l100",
  "target_position": [0.5, 0.2, 0.3],
  "reached_target": true,
  "termination_reason": "target_reached",
  "collection_rate_hz": 15.0
}
```

### 파라미터 설정

#### data_collection_config_omy_l100.yaml
```yaml
controller_type: "omy_l100"
collection_rate: 15.0          # Hz
base_frame: "panda_link0"
ee_frame: "gripper_tip_link"
target_tolerance: 0.02         # 2cm
stationary_duration: 3.0       # 3초
stationary_threshold: 0.005    # 5mm
episode_timeout: 60.0          # 60초
data_dir: "data/omy_l100"
```

#### data_collection_config_joystick.yaml
```yaml
controller_type: "joystick"
# ... (동일한 파라미터, data_dir만 다름)
data_dir: "data/joystick"
```

---

## 📡 토픽 레퍼런스

### 공통 토픽

| Topic | Type | Publisher | Subscriber | 설명 |
|-------|------|-----------|------------|------|
| `/clutch/active` | `std_msgs/Bool` | clutch_pedal_node | 모든 제어 노드 | 클러치 페달 상태 |
| `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | 컨트롤러 노드 | servo_node | 엔드이펙터 속도 명령 |
| `/gripper/position` | `std_msgs/Float64` | 그리퍼 노드 | trajectory_to_joint_states | 그리퍼 위치 명령 |
| `/panda_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | servo_node | trajectory_to_joint_states | 관절 궤적 명령 |
| `/joint_states` | `sensor_msgs/JointState` | trajectory_to_joint_states | robot_state_publisher | 모든 관절 상태 |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher | 모든 노드 | 동적 변환 |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | 모든 노드 | 정적 변환 |

### OMY L100 전용 토픽

| Topic | Type | Publisher | Subscriber | 설명 |
|-------|------|-----------|------------|------|
| `/leader/joint_states` | `sensor_msgs/JointState` | omy_l100_leader | omy_l100_to_twist, omy_l100_to_gripper | 리더암 관절 상태 |

### 조이스틱 전용 토픽

| Topic | Type | Publisher | Subscriber | 설명 |
|-------|------|-----------|------------|------|
| `/joy` | `sensor_msgs/Joy` | joy_node | joy_to_twist, joy_to_gripper | 조이스틱 입력 |

### 데이터 수집 전용 토픽

| Topic | Type | Publisher | Subscriber | 설명 |
|-------|------|-----------|------------|------|
| `/data_collection/target_marker` | `visualization_msgs/Marker` | data_collection_node | RViz | 목표 위치 시각화 |
| `/data_collection/status` | (로그) | data_collection_node | - | 데이터 수집 상태 |

---

## 🔗 의존성 그래프

### 패키지 의존성

```
                    ┌─────────────────────┐
                    │  custom_panda       │
                    │  _description       │
                    │  (URDF/SRDF)        │
                    └──────────┬──────────┘
                               │
                ┌──────────────┴──────────────┐
                │                             │
                ↓                             ↓
    ┌───────────────────┐         ┌───────────────────┐
    │  panda_common     │         │  open_manipulator │
    │                   │         │  _* (OMY L100)    │
    │  • Data collect   │         └─────────┬─────────┘
    │  • Clutch         │                   │
    │  • Trajectory     │                   │
    └─────────┬─────────┘                   │
              │                             │
      ┌───────┴────────┐                    │
      │                │                    │
      ↓                ↓                    ↓
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ panda_teleop │  │ panda_teleop │  │ dynamixel_*  │
│ _joystick    │  │ _omy_l100    │◄─┤ (HW 인터페이스)│
└──────────────┘  └──────┬───────┘  └──────────────┘
                         │
                         ↓
                  ┌──────────────┐
                  │ ros2_        │
                  │ controller/* │
                  │ (Controllers)│
                  └──────────────┘
```

### 런타임 노드 의존성 (OMY L100)

```
omy_l100_leader_ai ──→ /leader/joint_states
                              │
                    ┌─────────┴─────────┐
                    ↓                   ↓
            omy_l100_to_twist    omy_l100_to_gripper
                    │                   │
clutch_pedal ──→ ┌──┴──┐                │
                 │     │                │
                 ↓     ↓                ↓
            servo_node ←──┐    trajectory_to_joint_states
                 │         │             ↑
                 └─────────┘             │
                           └─────────────┘
                                  │
                                  ↓
                        robot_state_publisher
                                  │
                        ┌─────────┴─────────┐
                        ↓                   ↓
                      RViz2          data_collection_node
```

### 런타임 노드 의존성 (조이스틱)

```
joy_node ──→ /joy
                │
          ┌─────┴──────┐
          ↓            ↓
    joy_to_twist  joy_to_gripper
          │            │
          │            │
clutch ──→└──┬──┐      │
             │  │      │
             ↓  ↓      ↓
        servo_node  trajectory_to_joint_states
             │              │
             └──────┬───────┘
                    ↓
          robot_state_publisher
                    │
          ┌─────────┴─────────┐
          ↓                   ↓
        RViz2          data_collection_node
```

---

## 🚀 실행 가이드

### OMY L100 모드

```bash
# 1. 워크스페이스 빌드 및 소싱
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. OMY L100 + Panda 실행 (데이터 수집 없음)
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py

# 3. OMY L100 + Panda 실행 (데이터 수집 활성화)
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py \
    enable_data_collection:=true

# 4. RViz 없이 실행
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py \
    use_rviz:=false
```

### 조이스틱 모드

```bash
# 1. 워크스페이스 소싱
source ~/ros2_ws/install/setup.bash

# 2. 조이스틱 + Panda 실행
ros2 launch panda_teleop_joystick joystick_teleop.launch.py

# 3. 데이터 수집 활성화
ros2 launch panda_teleop_joystick joystick_teleop.launch.py \
    enable_data_collection:=true
```

### 데이터 분석

```bash
# 데이터셋 검증
python3 ~/ros2_ws/scripts/validate_dataset.py \
    ~/ros2_ws/data/omy_l100/session_20260128_100326/

# 에피소드 시각화
python3 ~/ros2_ws/scripts/visualize_episode.py \
    ~/ros2_ws/data/omy_l100/session_20260128_100326/episode_001.jsonl
```

---

## 🛠️ 새 컨트롤러 추가 가이드

### 1단계: 패키지 생성
```bash
cd ~/ros2_ws
ros2 pkg create panda_teleop_<your_controller> --build-type ament_cmake
```

### 2단계: package.xml 의존성 추가
```xml
<depend>panda_common</depend>
<depend>custom_panda_description</depend>
<depend>moveit_servo</depend>
```

### 3단계: 변환 노드 구현
```python
# your_controller_to_twist_node.py
# 입력: 컨트롤러 특화 메시지
# 출력: /servo_node/delta_twist_cmds (TwistStamped)
```

### 4단계: Launch 파일 작성
```python
# Include:
# - custom_panda_description (URDF)
# - panda_common/trajectory_to_joint_states
# - panda_common/clutch_pedal_node
# - panda_common/data_collection_node (optional)
# - Your converter node
```

### 5단계: Config 파일
```bash
# panda_common/config/에 추가:
data_collection_config_<your_controller>.yaml
```

### 완료! 🎉
- 자동으로 데이터 수집 지원
- 클러치 안전 제어 통합
- MoveIt Servo IK 활용

---

## 📝 설정 파일 위치

### MoveIt Servo 설정
- **OMY L100**: `panda_teleop_omy_l100/config/servo_config.yaml`
- **조이스틱**: `panda_teleop_joystick/config/servo_config.yaml`

### IK 솔버 설정
- **OMY L100**: `panda_teleop_omy_l100/config/kinematics.yaml`
- **공통**: `custom_panda_description/config/kinematics.yaml`

### 데이터 수집 설정
- **OMY L100**: `panda_common/config/data_collection_config_omy_l100.yaml`
- **조이스틱**: `panda_common/config/data_collection_config_joystick.yaml`

### 로봇 정의
- **URDF**: `custom_panda_description/urdf/panda_with_robotiq.urdf`
- **SRDF**: `custom_panda_description/config/panda_robotiq.srdf`
- **RViz**: `custom_panda_description/config/view_robot.rviz`

---

## 🐛 문제 해결

### RViz에 로봇이 안 보임
```bash
# TF 확인
ros2 run tf2_ros tf2_echo panda_link0 gripper_tip_link

# joint_states 확인
ros2 topic echo /joint_states
```

### 로봇이 움직이지 않음
```bash
# 클러치 상태 확인
ros2 topic echo /clutch/active

# Twist 명령 확인
ros2 topic echo /servo_node/delta_twist_cmds

# 노드 목록 확인
ros2 node list
```

### 데이터 수집이 안 됨
```bash
# 데이터 수집 노드 확인
ros2 node list | grep data_collection

# TF 확인 (필수)
ros2 run tf2_ros tf2_echo panda_link0 gripper_tip_link

# 로그 확인
ros2 node info /data_collection_node
```

---

## 📊 성능 지표

### 제어 주파수
- **MoveIt Servo**: 50Hz (`publish_period: 0.02`)
- **데이터 수집**: 15Hz (`collection_rate: 15.0`)
- **조이스틱**: ~100Hz (joy_node 기본값)
- **OMY L100**: ~30Hz (Dynamixel SDK 기본값)

### 지연시간
- **OMY L100 → Panda**: ~40ms (평균)
- **조이스틱 → Panda**: ~20ms (평균)

### 데이터 수집
- **예상 샘플 수**: ~900 samples/episode (60초 @ 15Hz)
- **실제 관측**: ~530-840 samples/episode (병목 있음)
  - 원인: 로깅, RViz 마커 발행
  - 개선 방법: 로깅 throttle, 마커 주파수 감소

---

## 📚 참고 자료

### 내부 문서
- `README.md` - 프로젝트 개요 및 Quick Start
- `TOPIC_INFORMATION.md` - 토픽 상세 정보
- `custom_panda_description/ARCHITECTURE.md` - URDF 구조

### 외부 링크
- [MoveIt Servo Documentation](https://moveit.picknik.ai/humble/doc/realtime_servo/realtime_servo_tutorial.html)
- [Franka Panda Specs](https://frankaemika.github.io/docs/)
- [Robotiq 2F-85 Manual](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf)

---

## ✨ 주요 특징 요약

1. ✅ **모듈식 설계**: 컨트롤러마다 독립적인 패키지
2. ✅ **공통 유틸리티**: `panda_common`으로 코드 재사용
3. ✅ **안전 제어**: 클러치 페달 통합
4. ✅ **자동 데이터 수집**: Episode 관리 자동화
5. ✅ **확장 가능**: 새 컨트롤러 쉽게 추가
6. ✅ **실시간 시각화**: RViz 통합
7. ✅ **표준 준수**: ROS2 best practices

---

**End of Architecture Document**

*최종 업데이트: 2026-01-28*  
*문서 버전: 1.0*  
*작성자: etri*
