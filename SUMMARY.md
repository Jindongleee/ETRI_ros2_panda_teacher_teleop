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
└── joystick/
    └── session_YYYYMMDD_HHMMSS/
        └── ...
```

### 수집률
- **설정**: 15Hz
- **예상**: ~900 samples/episode (60초)
- **실제**: ~530-840 samples/episode

---

## 🎮 조이스틱 매핑

| 입력 | 기능 | 최대 속도 |
|------|------|----------|
| **Left Stick** | Linear X/Y | 0.5 m/s |
| **Right Stick** | Linear Z | 0.5 m/s |
| **L1/L2** | Angular X | 1.0 rad/s |
| **△/✕** | Gripper | ±0.05 |
| **페달** | 안전 제어 | - |

---

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

## 📚 문서 (5개)

| 문서 | 크기 | 용도 |
|------|------|------|
| **README.md** | 7.9KB | 시작 가이드 |
| **ARCHITECTURE.md** | 36KB | 완전한 아키텍처 ⭐⭐⭐ |
| **QUICK_REFERENCE.md** | 2.4KB | 빠른 참조 |
| **TOPIC_INFORMATION.md** | 5.8KB | 토픽 레퍼런스 |
| **DOCS_INDEX.md** | 3.3KB | 문서 네비게이션 |

**총 문서 크기**: ~56KB

---

## 🎯 주요 특징

```
✅ 모듈식 설계        → 컨트롤러 독립적
✅ 공통 유틸리티      → 코드 재사용
✅ 안전 제어          → 클러치 페달
✅ 자동 데이터 수집   → Episode 관리
✅ 확장 가능          → 새 컨트롤러 추가 쉬움
✅ 실시간 시각화      → RViz 통합
✅ 표준 준수          → ROS2 Best Practices
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

---

## 📞 문제 해결

### 로봇이 안 움직임
```bash
ros2 topic echo /clutch/active        # 클러치 확인
ros2 topic echo /servo_node/delta_twist_cmds  # 명령 확인
ros2 node list                        # 노드 확인
```

### RViz에 로봇 안 보임
```bash
ros2 run tf2_ros tf2_echo panda_link0 gripper_tip_link  # TF 확인
ros2 topic echo /joint_states         # 상태 확인
```

---

## 🎓 학습 자료

### 필수 읽기
1. 📖 **README.md** - 시작점
2. 📘 **ARCHITECTURE.md** - 핵심 문서 (969줄!)
3. 🚀 **QUICK_REFERENCE.md** - 일상 사용

### 참고 자료
- 🔍 **TOPIC_INFORMATION.md** - 토픽 상세
- 📚 **DOCS_INDEX.md** - 문서 네비게이션

---

## 📊 통계

```
패키지:        16개
핵심 패키지:    4개
Python 노드:   10개
Launch 파일:    2개
Config 파일:   15개
문서:          5개 (56KB)
총 코드 줄:    ~3000줄
```

---

## 💡 Tip

**즐겨찾기 추천**:
- 📖 `DOCS_INDEX.md` - 문서 시작점
- 🚀 `QUICK_REFERENCE.md` - 일상 명령어
- 📘 `ARCHITECTURE.md` - 깊은 이해

**디버깅 명령어**:
```bash
# 노드 목록
ros2 node list

# 토픽 목록
ros2 topic list

# TF 트리
ros2 run tf2_tools view_frames
```

---

**🎊 즐거운 로봇 프로그래밍 되세요!**

*Last Updated: 2026-01-28*
