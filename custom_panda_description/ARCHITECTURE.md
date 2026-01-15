# Custom Panda Description - Node & Topic Architecture

이 문서는 `custom_panda_description` 패키지의 ROS2 노드와 토픽 구조를 설명합니다.

## 📋 목차

1. [시스템 개요](#시스템-개요)
2. [노드 목록](#노드-목록)
3. [토픽 흐름도](#토픽-흐름도)
4. [노드 상세 설명](#노드-상세-설명)
5. [서비스](#서비스)
6. [설정 파일](#설정-파일)

---

## 시스템 개요

이 패키지는 **조이스틱으로 Franka Panda 로봇을 제어**하는 시스템입니다.

- **입력**: SONY DualShock 3 조이스틱
- **제어**: MoveIt Servo (실시간 Cartesian 제어)
- **시각화**: RViz2
- **그리퍼**: Robotiq 85 (2-finger parallel gripper)

---

## 노드 목록

### 표준 ROS2 노드

| 노드명 | 패키지 | 실행 파일 | 역할 |
|--------|--------|-----------|------|
| `robot_state_publisher` | `robot_state_publisher` | `robot_state_publisher` | URDF 기반 TF 발행 |
| `joy_node` | `joy` | `joy_node` | 조이스틱 하드웨어 입력 → `/joy` 토픽 |
| `servo_node` | `moveit_servo` | `servo_node_main` | MoveIt Servo: Twist → Joint Trajectory 변환 |
| `rviz2` | `rviz2` | `rviz2` | 3D 시각화 |

### 커스텀 노드 (이 패키지)

| 노드명 | 파일 | 역할 |
|--------|------|------|
| `joy_to_twist_node` | `src/joy_to_twist_node.py` | 조이스틱 → Twist 명령 변환 |
| `joy_to_gripper_node` | `src/joy_to_gripper_node.py` | 조이스틱 버튼 → 그리퍼 위치 명령 |
| `trajectory_to_joint_states` | `src/trajectory_to_joint_states.py` | Joint Trajectory → Joint State 변환 |

---

## 토픽 흐름도

### 전체 시스템 흐름

```
┌─────────────┐
│  Joystick   │ (Hardware)
└──────┬──────┘
       │
       ▼
┌─────────────┐      /joy
│  joy_node   │ ──────────────────┐
└─────────────┘                    │
                                   │
        ┌──────────────────────────┴──────────────────────────┐
        │                                                        │
        ▼                                                        ▼
┌──────────────────┐                          ┌──────────────────────┐
│joy_to_twist_node │                          │joy_to_gripper_node   │
└────────┬─────────┘                          └──────────┬───────────┘
         │                                               │
         │ /servo_node/delta_twist_cmds                  │ /gripper/position
         │                                               │
         ▼                                               ▼
┌──────────────────┐                          ┌──────────────────────────┐
│   servo_node     │                          │trajectory_to_joint_states│
│  (MoveIt Servo)  │                          └──────────┬───────────────┘
└────────┬─────────┘                                     │
         │                                               │
         │ /panda_arm_controller/joint_trajectory        │ /joint_states
         │                                               │
         └───────────────────┬───────────────────────────┘
                             │
                             ▼
                    ┌────────────────────┐
                    │robot_state_publisher│
                    └──────────┬─────────┘
                               │
                               │ /tf, /tf_static
                               │
                               ▼
                          ┌─────────┐
                          │  rviz2  │
                          └─────────┘
```

### 제어 흐름 (Arm Movement)

```
/joy (sensor_msgs/Joy)
    │
    ├─> joy_to_twist_node
    │       │
    │       └─> /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped)
    │               │
    │               └─> servo_node (MoveIt Servo)
    │                       │
    │                       └─> /panda_arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
    │                               │
    │                               └─> trajectory_to_joint_states
    │                                       │
    │                                       └─> /joint_states (sensor_msgs/JointState)
    │                                               │
    │                                               └─> robot_state_publisher
    │                                                       │
    │                                                       └─> /tf, /tf_static
    │                                                               │
    │                                                               └─> rviz2 (시각화)
```

### 그리퍼 제어 흐름

```
/joy (sensor_msgs/Joy)
    │
    ├─> joy_to_gripper_node
    │       │
    │       └─> /gripper/position (std_msgs/Float64)
    │               │
    │               └─> trajectory_to_joint_states
    │                       │
    │                       └─> /joint_states (sensor_msgs/JointState)
    │                               │
    │                               └─> robot_state_publisher
    │                                       │
    │                                       └─> /tf, /tf_static
    │                                               │
    │                                               └─> rviz2 (그리퍼 시각화)
```

---

## 노드 상세 설명

### 1. `joy_node`

**패키지**: `joy`  
**역할**: 조이스틱 하드웨어 입력을 ROS2 메시지로 변환

**구독 토픽**: 없음 (하드웨어 직접 읽기)

**발행 토픽**:
- `/joy` (`sensor_msgs/Joy`)
  - `axes[]`: 조이스틱 축 값 (-1.0 ~ 1.0)
  - `buttons[]`: 버튼 상태 (0 또는 1)

**파라미터**:
- `dev`: `/dev/input/js1` (조이스틱 장치 경로)
- `deadzone`: `0.05` (데드존 크기) # 잘못된 조작 방지
- `autorepeat_rate`: `20.0` (Hz)

---

### 2. `joy_to_twist_node`

**파일**: `src/joy_to_twist_node.py`  
**역할**: 조이스틱 입력을 MoveIt Servo용 Twist 명령으로 변환

**구독 토픽**:
- `/joy` (`sensor_msgs/Joy`)

**발행 토픽**:
- `/servo_node/delta_twist_cmds` (`geometry_msgs/TwistStamped`)
  - `twist.linear.x/y/z`: 선형 속도 (m/s)
  - `twist.angular.z`: 각속도 (rad/s)
- `/joy_to_twist/status` (`std_msgs/Float64`) - 디버깅용

**조이스틱 매핑** (DualShock 3 기준):
- **L1 (Button 4)**: Deadman switch (필수)
- **왼쪽 스틱 X (Axis 0)**: Linear X
- **왼쪽 스틱 Y (Axis 1)**: Linear Y
- **오른쪽 스틱 X (Axis 3)**: Linear Z
- **L1만 누름**: Angular Z = +1.5 rad/s
- **L1 + L2 (Button 3)**: Angular Z = -1.5 rad/s

**주요 기능**:
- **정규화**: Twist 벡터의 최대 성분이 1.0을 넘지 않도록 스케일링
  - MoveIt Servo가 "Component > 1" 경고를 내지 않도록 방지
- **Deadman 보호**: L1을 누르지 않으면 모든 속도를 0으로 발행

**파라미터**:
- `linear_scale`: `1.0` (선형 속도 스케일)
- `angular_scale`: `1.5` (각속도 스케일)
- `deadman_button`: `4` (L1 버튼 인덱스)
- `frame_id`: `panda_link0` (Twist 명령의 기준 프레임)

---

### 3. `joy_to_gripper_node`

**파일**: `src/joy_to_gripper_node.py`  
**역할**: 조이스틱 버튼을 그리퍼 열기/닫기 명령으로 변환

**구독 토픽**:
- `/joy` (`sensor_msgs/Joy`)

**발행 토픽**:
- `/gripper/position` (`std_msgs/Float64`)
  - `data`: 그리퍼 위치 (0.0 = 완전 열림, 0.8 = 완전 닫힘)

**조이스틱 매핑**:
- **Triangle (Button 2)**: 그리퍼 열기 (값 감소)
- **Cross/X (Button 3)**: 그리퍼 닫기 (값 증가)

**주요 기능**:
- **증분 제어**: 버튼을 누를 때마다 `step`만큼 위치 변경
- **범위 제한**: `[min_position, max_position]` 범위 내에서만 동작
- **상태 유지**: 내부에서 현재 그리퍼 위치를 추적

**파라미터**:
- `button_open`: `2` (열기 버튼 인덱스)
- `button_close`: `3` (닫기 버튼 인덱스)
- `step`: `0.02` (버튼 누를 때마다 변경되는 양)
- `min_position`: `0.0` (최소 위치 = 완전 열림)
- `max_position`: `0.8` (최대 위치 = 완전 닫힘)

---

### 4. `servo_node` (MoveIt Servo)

**패키지**: `moveit_servo`  
**역할**: Cartesian 속도 명령(Twist)을 관절 궤적(Joint Trajectory)으로 변환

**구독 토픽**:
- `/servo_node/delta_twist_cmds` (`geometry_msgs/TwistStamped`)
- `/joint_states` (`sensor_msgs/JointState`) - 현재 관절 상태

**발행 토픽**:
- `/panda_arm_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`)
  - 7개 팔 관절 (`panda_joint1` ~ `panda_joint7`)의 목표 위치/속도

**서비스**:
- `/servo_node/start_servo` (`std_srvs/srv/Trigger`) - Servo 활성화
- `/servo_node/pause_servo` (`std_srvs/srv/Trigger`) - Servo 일시정지
- `/servo_node/unpause_servo` (`std_srvs/srv/Trigger`) - Servo 재개

**주요 기능**:
- **Inverse Kinematics (IK)**: Cartesian 속도 → 관절 속도 변환
- **Jacobian 계산**: 실시간으로 Jacobian 행렬 계산
- **충돌 검사**: (설정에 따라) 자체 충돌 및 환경 충돌 검사
- **특이점 회피**: 특이점 근처에서 자동 감속
- **관절 한계 보호**: 관절 한계 근처에서 자동 감속

**설정 파일**: `config/servo_config.yaml`

**주요 파라미터**:
- `command_in_type`: `"speed_units"` (속도 단위 명령)
- `scale.linear`: `1.0` (m/s) - 최대 선형 속도
- `scale.rotational`: `1.5` (rad/s) - 최대 각속도
- `scale.joint`: `1.0` - 관절 속도 한계 (URDF 최대값 사용)
- `override_velocity_scaling_factor`: `1.0` - 속도 스케일링 (1.0 = 최대)
- `check_collisions`: `false` - 충돌 검사 비활성화 (개발용)

---

### 5. `trajectory_to_joint_states`

**파일**: `src/trajectory_to_joint_states.py`  
**역할**: Joint Trajectory와 그리퍼 위치를 Joint State로 통합

**구독 토픽**:
- `/panda_arm_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`)
  - 팔 관절 궤적 (7개 관절)
- `/gripper/position` (`std_msgs/Float64`)
  - 그리퍼 위치 (0.0 ~ 0.8)

**발행 토픽**:
- `/joint_states` (`sensor_msgs/JointState`)
  - **팔 관절**: `panda_joint1` ~ `panda_joint7` (servo_node에서 받음)
  - **그리퍼 관절**: `finger_joint` + mimic 관절들 (joy_to_gripper_node에서 받음)
    - `left_inner_knuckle_joint`
    - `left_inner_finger_joint`
    - `right_inner_knuckle_joint`
    - `right_inner_finger_joint`
    - `right_outer_knuckle_joint`

**주요 기능**:
- **궤적 추출**: Joint Trajectory의 마지막 포인트에서 위치 추출
- **그리퍼 mimic 관계**: URDF의 mimic 관계를 따라 그리퍼 관절 계산
  - `finger_joint` 값에 따라 다른 그리퍼 관절들이 자동으로 계산됨
- **NaN 필터링**: 유효하지 않은 값(NaN/Inf)을 0으로 대체
- **초기 상태 발행**: 시작 시 "ready" 자세로 초기 `/joint_states` 발행

**그리퍼 mimic 관계** (URDF 기준):
```
finger_joint (메인)
    ├─> left_inner_knuckle_joint = finger_joint × 1
    ├─> left_inner_finger_joint = finger_joint × (-1)
    ├─> right_inner_knuckle_joint = finger_joint × (-1)
    ├─> right_inner_finger_joint = finger_joint × 1
    └─> right_outer_knuckle_joint = finger_joint × (-1)
```

**QoS 설정**:
- `/joint_states` 발행: `BEST_EFFORT` (robot_state_publisher와 호환)

---

### 6. `robot_state_publisher`

**패키지**: `robot_state_publisher`  
**역할**: Joint State를 기반으로 TF 변환 발행

**구독 토픽**:
- `/joint_states` (`sensor_msgs/JointState`)

**발행 토픽**:
- `/tf` (`tf2_msgs/TFMessage`) - 동적 변환
- `/tf_static` (`tf2_msgs/TFMessage`) - 정적 변환

**주요 기능**:
- URDF를 기반으로 로봇의 모든 링크 간 변환 계산
- `/joint_states`의 관절 값을 사용하여 실시간으로 TF 업데이트
- RViz가 로봇을 시각화할 수 있도록 TF 트리 제공

---

### 7. `rviz2`

**패키지**: `rviz2`  
**역할**: 3D 시각화

**구독 토픽**:
- `/tf`, `/tf_static` - 로봇 모델 표시
- 기타 시각화용 토픽 (설정에 따라)

**주요 기능**:
- 로봇 모델 시각화
- 관절 움직임 실시간 표시
- 그리퍼 열기/닫기 시각화

---

## 서비스

### `/servo_node/start_servo`

**타입**: `std_srvs/srv/Trigger`  
**역할**: MoveIt Servo 활성화

**사용 시점**: Launch 파일에서 자동 호출 (5초 지연 후)

**수동 호출**:
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
```

### `/servo_node/pause_servo`

**타입**: `std_srvs/srv/Trigger`  
**역할**: MoveIt Servo 일시정지

### `/servo_node/unpause_servo`

**타입**: `std_srvs/srv/Trigger`  
**역할**: MoveIt Servo 재개

---

## 설정 파일

### `config/servo_config.yaml`

MoveIt Servo 노드의 설정 파일

**주요 섹션**:
- `scale`: 속도 스케일 (linear, rotational, joint)
- `joystick`: 조이스틱 버튼/축 매핑 (참고용, 실제 사용은 커스텀 노드)
- `check_collisions`: 충돌 검사 설정
- `joint_limit_margin`: 관절 한계 여유

### `config/panda_robotiq.srdf`

MoveIt용 Semantic Robot Description

**주요 내용**:
- Planning groups (`panda_arm`, `gripper`)
- Collision pairs (비활성화된 충돌 검사)
- Group states (예: "ready" 자세)

### `urdf/panda_with_robotiq.urdf`

로봇의 기하학적/물리적 모델

**주요 내용**:
- 링크 정의 (팔 + 그리퍼)
- 관절 정의 및 한계
- 그리퍼 mimic 관계

---

## 노드 시작 순서

Launch 파일에서 다음 순서로 노드가 시작됩니다:

1. **t=0s**: `robot_state_publisher`, `trajectory_to_joint_states` (즉시 시작)
2. **t=2s**: `joy_node`, `servo_node`, `joy_to_twist_node`, `joy_to_gripper_node` (지연 시작)
3. **t=3s**: `rviz2` (지연 시작)
4. **t=5s**: `/servo_node/start_servo` 서비스 호출 (Servo 활성화)

**지연 이유**:
- `trajectory_to_joint_states`가 초기 `/joint_states`를 발행할 시간 확보
- `servo_node`가 `/joint_states`를 받을 수 있도록 보장
- RViz가 모든 TF가 준비된 후 시작

---

## 토픽 요약

| 토픽명 | 타입 | 발행 노드 | 구독 노드 | 설명 |
|--------|------|-----------|-----------|------|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `joy_to_twist_node`, `joy_to_gripper_node` | 조이스틱 입력 |
| `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | `joy_to_twist_node` | `servo_node` | Cartesian 속도 명령 |
| `/gripper/position` | `std_msgs/Float64` | `joy_to_gripper_node` | `trajectory_to_joint_states` | 그리퍼 위치 |
| `/panda_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | `servo_node` | `trajectory_to_joint_states` | 팔 관절 궤적 |
| `/joint_states` | `sensor_msgs/JointState` | `trajectory_to_joint_states` | `robot_state_publisher`, `servo_node` | 통합 관절 상태 |
| `/tf` | `tf2_msgs/TFMessage` | `robot_state_publisher` | `rviz2` | 동적 변환 |
| `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` | `rviz2` | 정적 변환 |
| `/servo_node/status` | `std_msgs/Int8` | `servo_node` | - | Servo 상태 (0=정지, 1=활성, 3=관절한계, 4=충돌) |

---

## 디버깅 팁

### 조이스틱 입력 확인
```bash
ros2 topic echo /joy
```

### Twist 명령 확인
```bash
ros2 topic echo /servo_node/delta_twist_cmds
```

### Servo 상태 확인
```bash
ros2 topic echo /servo_node/status
```

### Joint Trajectory 확인
```bash
ros2 topic echo /panda_arm_controller/joint_trajectory
```

### Joint States 확인
```bash
ros2 topic echo /joint_states
```

### TF 확인
```bash
ros2 run tf2_ros tf2_echo panda_link0 panda_link8
```

### 노드 그래프 시각화
```bash
rqt_graph
```

---

## 참고 자료

- [MoveIt Servo 문서](https://moveit.picknik.ai/main/doc/how_to_guides/real_time_servo.html)
- [ROS2 Joy 패키지](https://github.com/ros/joystick_drivers)
- [Robot State Publisher](https://github.com/ros/robot_state_publisher)

---

**작성일**: 2026-01-14  
**패키지 버전**: custom_panda_description
