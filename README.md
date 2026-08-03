# Panda Robot Teleoperation Workspace

Robotiq 그리퍼를 장착한 Panda 로봇을 위한 멀티 컨트롤러 텔레오퍼레이션 시스템입니다.

## 📦 Package Structure

### Core Packages

#### **panda_common** ✨
모든 Panda 텔레오퍼레이션 컨트롤러가 공통으로 쓰는 유틸리티와 노드 모음입니다.
- **목적**: 모든 컨트롤러가 공유하는 기능
- **구성**:
  - `data_collection_node.py` - 모방학습용 데이터 수집
  - `clutch_pedal_node.py` - 안전 클러치 페달 인터페이스
  - `trajectory_to_joint_states.py` - Trajectory를 joint states로 변환
  - 데이터 수집용 설정 파일

#### **custom_panda_description**
Panda 로봇 URDF 및 로봇 설명 파일입니다.
- Panda + Robotiq 그리퍼 URDF
- MoveIt 설정용 SRDF
- RViz 설정
- Kinematics 설정

### Controller-Specific Packages

#### **panda_teleop_omy_l100**
OMY L100 leader arm을 이용한 텔레오퍼레이션입니다.
- **목적**: OMY L100 leader arm으로 Panda follower arm 제어
- **구성**:
  - `omy_l100_to_twist_node.py` - 순기구학 변환기
  - `omy_l100_to_gripper_node.py` - 그리퍼 매핑
  - Launch 파일 및 설정
- **의존성**: `panda_common`, `open_manipulator_*`, `dynamixel_*`

#### **panda_teleop_joystick**
게임 컨트롤러(조이스틱)를 이용한 텔레오퍼레이션입니다.
- **목적**: 조이스틱으로 Panda follower arm 제어
- **구성**:
  - `joy_to_twist_node.py` - 조이스틱을 twist로 변환
  - `joy_to_gripper_node.py` - 그리퍼 제어
  - Launch 파일 및 설정
- **의존성**: `panda_common`, `custom_panda_description`

### Hardware Interface Packages

#### **OpenMANIPULATOR Packages** (OMY L100용)
- `open_manipulator_bringup` - 하드웨어 bringup
- `open_manipulator_description` - URDF 및 메시
- `open_manipulator_collision` - 자체 충돌 감지

#### **Dynamixel Packages** (OMY L100용)
- `DynamixelSDK` - 다이나믹셀 모터 SDK
- `dynamixel_hardware_interface` - ROS2 control 인터페이스
- `dynamixel_interfaces` - 커스텀 메시지/서비스

#### **ROS2 Controllers** (OMY L100용)
- `om_gravity_compensation_controller`
- `om_joint_trajectory_command_broadcaster`
- `om_spring_actuator_controller`
- `realtime_tools`

---

## 🚀 Quick Start

### 1. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch with OMY L100 Controller

```bash
# 데이터 수집 없이
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py

# 데이터 수집 포함
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py enable_data_collection:=true
```

### 3. Launch with Joystick Controller

```bash
# 데이터 수집 없이
ros2 launch panda_teleop_joystick joystick_teleop.launch.py

# 데이터 수집 포함
ros2 launch panda_teleop_joystick joystick_teleop.launch.py enable_data_collection:=true
```

---

## 📊 Data Collection

모든 컨트롤러는 `panda_common`의 동일한 데이터 수집 시스템을 사용합니다.

### Data Storage Structure

```
data/
├── omy_l100/          # OMY L100 컨트롤러 데이터
│   ├── session_YYYYMMDD_HHMMSS/
│   │   ├── episode_001.jsonl
│   │   ├── episode_001_meta.json
│   │   └── ...
├── joystick/          # 조이스틱 컨트롤러 데이터
│   └── session_YYYYMMDD_HHMMSS/
│       └── ...
└── <future_controller>/  # 새 컨트롤러 추가도 쉽게!
```

### Data Format

각 샘플은 다음을 포함합니다:
- **State**: End-effector pose (x, y, z, qx, qy, qz, qw) + joint angles (7개 관절)
- **Action**: 속도 명령 (vx, vy, vz, wx, wy, wz)

수집 주기: **15Hz** (설정 가능)

---

## 🎮 Controller Details

### OMY L100 Controller

**하드웨어**: 7-DOF 협동로봇 팔
**제어 방식**: 순기구학을 통한 위치 미러링
**에피소드 관리**:
- 목표 자동 생성
- 목표 도달(2cm + 3초 정지) 또는 타임아웃(60초) 시 에피소드 종료

### Joystick Controller

**하드웨어**: 게임 컨트롤러 (PS4/Xbox 호환 확인됨)
**제어 방식**: 
- 왼쪽 스틱: 선형 X/Y
- 오른쪽 스틱: 선형 Z
- L1/L2 버튼: 각속도 X (Roll)
- 클러치 페달: 안전 제어

---

## 🔧 Adding a New Controller

새 컨트롤러를 추가하려면:

1. **새 패키지 생성**:
   ```bash
   ros2 pkg create panda_teleop_<your_controller> --build-type ament_cmake
   ```

2. **의존성 추가** — `package.xml`에 `panda_common` 추가:
   ```xml
   <depend>panda_common</depend>
   ```

3. **컨버터 노드 구현**:
   - 컨트롤러 입력을 `TwistStamped` 메시지로 변환
   - `/servo_node/delta_twist_cmds`로 publish

4. **Launch 파일 작성**, 다음을 포함:
   - 작성한 컨버터 노드
   - `panda_common`의 노드들 (데이터 수집, 클러치 등)
   - `custom_panda_description`의 Panda 로봇

5. **완료!** 이제 새 컨트롤러도 데이터 수집 기능을 온전히 지원합니다.

---

## 🛠️ Data Analysis Tools

`scripts/` 디렉토리에 있습니다.

### **validate_dataset.py**
수집된 데이터셋의 완전성과 정확성을 검증합니다.

```bash
cd ~/ros2_ws
python3 scripts/validate_dataset.py data/omy_l100/session_YYYYMMDD_HHMMSS/
```

### **visualize_episode.py**
에피소드의 궤적과 액션을 시각화합니다.

```bash
cd ~/ros2_ws
python3 scripts/visualize_episode.py data/omy_l100/session_YYYYMMDD_HHMMSS/episode_001.jsonl
```

---

## 📚 Documentation

### 📖 주요 문서
- **`README.md`** (이 파일) - 프로젝트 개요 및 Quick Start
- **`ARCHITECTURE.md`** 📘 - **완전한 시스템 아키텍처** (필독!)
  - 969줄의 상세한 기술 문서
  - 전체 시스템 다이어그램
  - 각 컨트롤러별 노드 상세 설명
  - 토픽 플로우 및 의존성 그래프
  - 데이터 수집 시스템 완벽 가이드
  - 새 컨트롤러 추가 가이드
- **`QUICK_REFERENCE.md`** 🚀 - 빠른 참조 카드
  - 자주 쓰는 명령어 모음
  - 조이스틱 조작법
  - 문제 해결 체크리스트
- **`TOPIC_INFORMATION.md`** - ROS2 토픽 상세 레퍼런스

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────┐
│         Controller Input                │
│  (OMY L100, Joystick, or Custom)       │
└──────────────┬──────────────────────────┘
               │
               ↓
┌──────────────────────────────────────────┐
│  Controller-Specific Converter Node      │
│  (omy_l100_to_twist, joy_to_twist, etc.) │
└──────────────┬───────────────────────────┘
               │
               ↓ /servo_node/delta_twist_cmds
┌──────────────────────────────────────────┐
│          MoveIt Servo                     │
│  (Inverse Kinematics + Safety)           │
└──────────────┬───────────────────────────┘
               │
               ↓ /panda_arm_controller/joint_trajectory
┌──────────────────────────────────────────┐
│    Panda Robot (Follower Arm)            │
└───────────────────────────────────────────┘
               │
               ↓ /joint_states
┌──────────────────────────────────────────┐
│   panda_common Utilities (shared)        │
│  - Data Collection                        │
│  - Clutch Pedal Safety                   │
│  - State Conversion                      │
└───────────────────────────────────────────┘
```

---

## 📝 License

Apache-2.0

## 👥 Maintainer

이진동 (jindong1019@gmail.com)
