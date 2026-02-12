# 📊 Workspace Summary

**위치**: `/home/etri/ros2_ws`  
**날짜**: 2026-01-28  
**ROS2**: Humble

---

## 🎯 한눈에 보는 시스템

```
┌────────────────────────────────────────────────────────────────┐
│             Panda Robot Teleoperation System                   │
│                                                                │
│  🤖 2개 컨트롤러  |  🔒 안전 제어  |  📊 자동 데이터 수집      │
└────────────────────────────────────────────────────────────────┘
```

---

## 📦 패키지 (16개)

### ⭐ Core Packages (우리가 만든 것)
```
✅ panda_common              (공통 유틸리티)
✅ panda_teleop_omy_l100     (OMY L100 컨트롤러)
✅ panda_teleop_joystick     (조이스틱 컨트롤러)
✅ custom_panda_description  (Panda + Robotiq URDF)
```

### 🤖 Hardware Interface (OMY L100용)
```
📦 open_manipulator_bringup
📦 open_manipulator_description
📦 open_manipulator_collision
📦 dynamixel_hardware_interface
📦 dynamixel_interfaces
📦 DynamixelSDK
```

### 🎛️ ROS2 Controllers (OMY L100용)
```
📦 om_gravity_compensation_controller
📦 om_joint_trajectory_command_broadcaster
📦 om_spring_actuator_controller
```

### 🛠️ Utilities
```
📦 realtime_tools
📁 scripts/ (데이터 분석 도구)
```

---

## 🚀 실행 명령어

### OMY L100 모드
```bash
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py
# 데이터 수집: enable_data_collection:=true 추가
```

### 조이스틱 모드
```bash
ros2 launch panda_teleop_joystick joystick_teleop.launch.py
# 데이터 수집: enable_data_collection:=true 추가
```

---

## 📊 데이터 수집

### 저장 위치
```
data/
├── omy_l100/
│   └── session_YYYYMMDD_HHMMSS/
│       ├── episode_001.jsonl
│       ├── episode_001_meta.json
│       └── ...
├── joystick/
│   └── session_YYYYMMDD_HHMMSS/
│        └── ...
└── vive/
    └── session_YYYYMMDD_HHMMSS/
        └── ...
```


## 🔧 핵심 노드

### 공통 (panda_common)
- `data_collection_node` - 15Hz 데이터 수집
- `clutch_pedal_node` - 안전 클러치
- `trajectory_to_joint_states` - 메시지 변환

### OMY L100 전용
- `omy_l100_to_twist_node` - FK 변환
- `omy_l100_to_gripper_node` - 그리퍼 매핑

### 조이스틱 전용
- `joy_to_twist_node` - 조이스틱 변환
- `joy_to_gripper_node` - 그리퍼 제어

### 표준 ROS2
- `servo_node` - MoveIt Servo (IK)
- `robot_state_publisher` - TF 발행

---

## 📡 주요 토픽

```
/clutch/active                    → 클러치 상태
/servo_node/delta_twist_cmds     → 속도 명령
/gripper/position                 → 그리퍼 명령
/panda_arm_controller/joint_trajectory → 관절 궤적
/joint_states                     → 관절 상태
/tf, /tf_static                   → 좌표 변환
```
---
## 🛠️ 빌드 상태

```bash
# 최근 빌드
Summary: 16 packages finished [14.6s] ✅

# 핵심 패키지
✅ panda_common
✅ panda_teleop_joystick
✅ panda_teleop_omy_l100
✅ custom_panda_description
```

---

## 📈 성능

### 제어 주파수
- MoveIt Servo: **50Hz**
- 데이터 수집: **15Hz**
- 조이스틱: **~100Hz**
- OMY L100: **~30Hz**

### 지연시간
- OMY L100 → Panda: **~40ms**
- 조이스틱 → Panda: **~20ms**

---

## 🤖 로봇 정보

### Follower Arm
- **모델**: Franka Emika Panda
- **자유도**: 7-DOF
- **엔드이펙터**: Robotiq 2F-85 Gripper
- **제어점**: `gripper_tip_link` (손끝 중앙 +11cm)

### Leader Arm (선택)
- **모델**: ROBOTIS OMY L100
- **자유도**: 7-DOF
- **통신**: Dynamixel Protocol 2.0

---

## 🔍 다음 단계

### 1️⃣ 시스템 이해하기
```bash
# 문서 읽기
cat ~/ros2_ws/README.md
cat ~/ros2_ws/ARCHITECTURE.md
```

### 2️⃣ 시스템 실행하기
```bash
# 조이스틱으로 시작 (간단)
ros2 launch panda_teleop_joystick joystick_teleop.launch.py
```

### 3️⃣ 데이터 수집하기
```bash
# 데이터 수집 활성화
ros2 launch panda_teleop_joystick joystick_teleop.launch.py \
    enable_data_collection:=true
```

### 4️⃣ 데이터 분석하기
```bash
# 수집한 데이터 검증
python3 scripts/validate_dataset.py data/joystick/session_*/

# 에피소드 시각화
python3 scripts/visualize_episode.py data/joystick/session_*/episode_001.jsonl
```