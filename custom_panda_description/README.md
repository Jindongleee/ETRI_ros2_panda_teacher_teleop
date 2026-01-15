# 🤖 Custom Panda Robot with Robotiq 2F-85 Gripper

Franka Panda 로봇 팔에 Robotiq 2F-85 그리퍼를 장착한 커스텀 로봇 모델입니다.

---

## 📑 목차

- [빠른 시작](#-빠른-시작)
- [Launch 파일 구조](#-launch-파일-구조)
- [로봇 사양](#-로봇-사양)
- [패키지 구조](#-패키지-구조)
- [사용 방법](#-사용-방법)
  - [RViz 시각화](#1-rviz에서-로봇-시각화)
  - [조이스틱 테스트](#2-dualshock-3-조이스틱-테스트)
  - [조이스틱 제어](#3-moveit-servo--조이스틱-제어)
- [조이스틱 매핑](#-조이스틱-매핑)
- [설정 커스터마이징](#-설정-커스터마이징)
- [트러블슈팅](#-트러블슈팅)
- [참고 자료](#-참고-자료)

---

## ⚡ 빠른 시작

### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select custom_panda_description
source install/setup.bash
```

### 🎯 Launch 파일 사용법

#### 1️⃣ **RViz로 로봇 시각화만 보기** (권장 - 개발용)
```bash
ros2 launch custom_panda_description display.launch.py
```

#### 2️⃣ **조이스틱으로 로봇 제어 (통합 버전)** (권장 - 데모용)
```bash
ros2 launch custom_panda_description servo_complete.launch.py
```
- 모든 기능 포함 (로봇 + RViz + 조이스틱 + MoveIt Servo)
- RViz 없이 실행: `use_rviz:=false`

#### 3️⃣ **모듈식 실행** (권장 - 디버깅용)
```bash
# 터미널 1: 기본 로봇
ros2 launch custom_panda_description base/robot.launch.py

# 터미널 2: RViz (선택사항)
ros2 launch custom_panda_description base/rviz.launch.py

# 터미널 3: 조이스틱 제어
ros2 launch custom_panda_description servo_joystick.launch.py
```

### 📁 Launch 파일 구조
```
launch/
├── display.launch.py           # 로봇 + GUI (joint_state_publisher_gui)
├── servo_complete.launch.py    # 전체 통합 (조이스틱 제어 + RViz)
├── servo_joystick.launch.py    # 조이스틱 노드만 (robot.launch.py 필요)
└── base/
    ├── robot.launch.py         # robot_state_publisher만
    └── rviz.launch.py          # RViz만
```

### 조이스틱 버튼 확인

```bash
# 터미널 1
ros2 run joy joy_node

# 터미널 2
ros2 topic echo /joy
```

---

## 🔧 로봇 사양

### Panda 로봇 팔
- **자유도(DOF)**: 7개
- **최대 도달 거리**: 855 mm
- **페이로드**: 3 kg

### Robotiq 2F-85 그리퍼

| 항목 | 값 | 설명 |
|------|-----|------|
| **질량** | 0.9 kg | 전체 무게 |
| **핑거 스트로크** | 0 ~ 85 mm | 그리퍼 개폐 범위 |
| **최대 파지력** | 235 N | 최대 그립 힘 |
| **최대 속도** | 150 mm/s | 핑거 이동 속도 |

### 링크별 상세 사양

| 링크 이름 | 질량 (kg) | CoM (xyz, m) | 관성 (kg·m²) |
|-----------|-----------|--------------|--------------|
| left_outer_knuckle | 0.0085 | [-0.0002, 0.0199, 0.0292] | ixx=2.89e-6, iyy=1.87e-6, izz=1.22e-6 |
| left_outer_finger | 0.0226 | [0.0003, 0.0374, -0.0208] | ixx=1.53e-5, iyy=6.18e-6, izz=1.16e-5 |
| left_inner_knuckle | 0.0271 | [0.0001, 0.0508, 0.0010] | ixx=2.62e-5, iyy=2.83e-6, izz=2.84e-5 |
| left_inner_finger | 0.0104 | [0.0003, 0.0160, -0.0137] | ixx=2.72e-6, iyy=7.69e-7, izz=2.30e-6 |

---

## 📁 패키지 구조

```
custom_panda_description/
├── 📄 CMakeLists.txt                    # CMake 빌드 설정
├── 📄 package.xml                       # ROS 2 패키지 정의
├── 📄 README.md                         # 본 문서
│
├── 📂 urdf/
│   └── panda_with_robotiq.urdf          # 통합 로봇 URDF 모델
│
├── 📂 config/
│   ├── panda_robotiq.srdf               # MoveIt 의미론적 설정
│   ├── ros2_controllers.yaml            # 컨트롤러 파라미터
│   ├── panda_robotiq.ros2_control.xacro # ros2_control 인터페이스
│   ├── servo_config.yaml                # MoveIt Servo + 조이스틱 설정
│   └── view_robot.rviz                  # RViz 시각화 설정
│
├── 📂 launch/
│   ├── display.launch.py                # 로봇 + GUI (joint_state_publisher_gui)
│   ├── servo_complete.launch.py         # 전체 통합 (조이스틱 제어 + RViz)
│   ├── servo_joystick.launch.py         # 조이스틱 노드만
│   └── base/
│       ├── robot.launch.py              # robot_state_publisher만
│       └── rviz.launch.py               # RViz만
│
└── 📂 meshes/
    ├── visual/                          # 시각적 메시 파일 (*.dae)
    └── collision/                       # 충돌 메시 파일 (*.stl)
```

---

## 🗂️ Launch 파일 구조

### 📊 파일별 기능 및 사용 시나리오

| Launch 파일 | 포함 노드 | 사용 시나리오 |
|------------|---------|-------------|
| **display.launch.py** | robot_state_publisher, joint_state_publisher_gui, RViz | 개발 초기 - 로봇 모델 확인 |
| **servo_complete.launch.py** | robot_state_publisher, RViz, joy, servo, joy_to_servo | 데모/테스트 - 조이스틱으로 제어 |
| **base/robot.launch.py** | robot_state_publisher | 모듈식 개발 - 기본 로봇만 |
| **base/rviz.launch.py** | RViz | 모듈식 개발 - 시각화만 |
| **servo_joystick.launch.py** | joy, servo, joy_to_servo | 모듈식 개발 - 조이스틱만 |

### 🔄 데이터 흐름 (servo_complete.launch.py 기준)

```
조이스틱(하드웨어)
    ↓
joy_node → /joy 토픽
    ↓
joy_to_servo_node → /servo_node/delta_twist_cmds 토픽
    ↓
servo_node → /joint_states 토픽
    ↓
robot_state_publisher → /tf, /tf_static 토픽
    ↓
RViz (시각화)
```

### 💡 시나리오별 추천 사용법

#### 🔧 **시나리오 1: 로봇 모델 개발/수정 중**
```bash
ros2 launch custom_panda_description display.launch.py
```
- joint_state_publisher_gui로 각 조인트 수동 조작
- URDF 수정 후 즉시 확인 가능

#### 🎮 **시나리오 2: 조이스틱 제어 데모**
```bash
ros2 launch custom_panda_description servo_complete.launch.py
```
- 한 번에 모든 기능 실행
- 발표/시연용으로 최적

#### 🐛 **시나리오 3: 디버깅/개발**
```bash
# 터미널 1
ros2 launch custom_panda_description base/robot.launch.py

# 터미널 2
ros2 launch custom_panda_description base/rviz.launch.py

# 터미널 3
ros2 launch custom_panda_description servo_joystick.launch.py

# 터미널 4
rqt_console  # 로그 모니터링
```
- 각 컴포넌트 개별 제어
- 문제 발생시 해당 노드만 재시작

#### 🏭 **시나리오 4: 실제 로봇 운영 (RViz 제외)**
```bash
ros2 launch custom_panda_description servo_complete.launch.py use_rviz:=false
```
- 리소스 절약 (GUI 없음)
- 서버나 임베디드 시스템용

---

## 🚀 사용 방법

### 1. RViz에서 로봇 시각화

로봇 모델을 시각화하고 조인트를 수동으로 제어합니다.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch custom_panda_description display.launch.py
```

**실행되는 노드:**
- `robot_state_publisher`: URDF를 읽어 TF 브로드캐스트
- `joint_state_publisher_gui`: 조인트 제어 GUI (슬라이더)
- `rviz2`: 3D 시각화

**GUI에서 할 수 있는 것:**
- 각 조인트를 슬라이더로 제어
- `robotiq_left_outer_knuckle_joint` 슬라이더로 그리퍼 개폐
- 로봇의 움직임 범위 확인

---

### 2. DualShock 3 조이스틱 테스트

조이스틱이 제대로 인식되는지 확인하고 버튼 인덱스를 파악합니다.

#### 2.1 조이스틱 연결 확인

```bash
# 연결된 조이스틱 장치 확인
ls /dev/input/js*

# 예상 출력: /dev/input/js0
```

#### 2.2 조이스틱 데이터 확인

```bash
# 터미널 1: Joy 노드 실행
ros2 run joy joy_node

# 터미널 2: Joy 메시지 확인
ros2 topic echo /joy
```

#### 2.3 버튼 인덱스 확인

각 버튼을 하나씩 눌러보면서 `buttons` 배열에서 `1`로 바뀌는 인덱스를 확인하세요.

```yaml
# /joy 토픽 출력 예시
header:
  stamp: ...
  frame_id: joy
axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 스틱 위치
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]  # L1 버튼 눌림 (index 10)
```

#### 2.4 버튼 매핑이 다를 경우

실제 버튼 인덱스가 다르면 `config/servo_config.yaml` 파일을 수정하세요.

---

### 3. MoveIt Servo + 조이스틱 제어

조이스틱으로 실시간 로봇 제어를 시작합니다.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch custom_panda_description servo_joystick.launch.py
```

**실행되는 노드:**
- `robot_state_publisher`: 로봇 모델 퍼블리시
- `joy_node`: 조이스틱 입력 읽기
- `servo_node`: MoveIt Servo 실시간 제어
- `joy_to_servo_node`: 조이스틱 → Twist 메시지 변환

**제어 방법:**
1. **L1 버튼을 누른 상태로 유지** (Deadman 스위치)
2. 왼쪽 스틱으로 X/Y 이동
3. 오른쪽 스틱으로 Z 이동 및 회전
4. Triangle/X 버튼으로 그리퍼 개폐

⚠️ **안전 주의사항:**
- L1 버튼을 누르지 않으면 로봇이 움직이지 않습니다 (안전장치)
- R1 버튼은 긴급 정지용입니다
- 처음에는 천천히 테스트하세요

---

### 4. Gazebo 시뮬레이션 (추후 추가 예정)

Gazebo에서 사용하려면:

1. `config/servo_config.yaml` 수정:
   ```yaml
   use_gazebo: true
   ```

2. Gazebo 런치 파일 생성 필요 (TODO)

---

## 🎮 조이스틱 매핑

### DualShock 3 기본 매핑

| 컨트롤 | 타입 | 인덱스 | 기능 | 설명 |
|--------|------|--------|------|------|
| **왼쪽 스틱 좌우** | Axis | 0 | X축 이동 | 좌우 직선 이동 |
| **왼쪽 스틱 상하** | Axis | 1 | Y축 이동 | 전후 직선 이동 |
| **오른쪽 스틱 상하** | Axis | 3 | Z축 이동 | 상하 직선 이동 |
| **오른쪽 스틱 좌우** | Axis | 2 | Yaw 회전 | Z축 중심 회전 |
| | | | | |
| **L1** | Button | 10 | 🔴 **Deadman** | **필수: 누르고 있어야 작동** |
| **R1** | Button | 11 | 🛑 긴급 정지 | 즉시 모든 움직임 중단 |
| **Triangle (△)** | Button | 12 | 그리퍼 열기 | 그리퍼 완전히 열기 |
| **Cross (✕)** | Button | 14 | 그리퍼 닫기 | 그리퍼 완전히 닫기 |
| **Select** | Button | 8 | 속도 감소 | 이동 속도 10% 감소 |
| **Start** | Button | 9 | 속도 증가 | 이동 속도 10% 증가 |

### 시각적 레이아웃

```
        DualShock 3 Controller
    ┌─────────────────────────────┐
    │                             │
    │  Select   Start   PS        │
    │    [8]     [9]               │
    │                             │
    │   [Left Stick]  [Right Stick] │
    │    Axis 0,1      Axis 2,3   │
    │                             │
    │  L1[10]           R1[11]    │
    │  L2                    R2   │
    │                             │
    │      △[12]                  │
    │    ◻[13] ○[15]              │
    │      ✕[14]                  │
    └─────────────────────────────┘
```

---

## ⚙️ 설정 커스터마이징

### 조이스틱 버튼 재매핑

버튼 인덱스가 다른 경우 `config/servo_config.yaml` 수정:

```yaml
joystick:
  # 축 매핑
  axis_linear_x: 0        # 좌우 이동 (왼쪽 스틱 X)
  axis_linear_y: 1        # 전후 이동 (왼쪽 스틱 Y)
  axis_linear_z: 3        # 상하 이동 (오른쪽 스틱 Y)
  axis_angular_z: 2       # 회전 (오른쪽 스틱 X)
  
  # 사용하지 않는 축은 -1로 설정
  axis_angular_x: -1
  axis_angular_y: -1
  
  # 버튼 매핑
  button_deadman: 10           # 안전 버튼 (L1)
  button_stop: 11              # 긴급 정지 (R1)
  button_slower: 8             # 속도 감소 (Select)
  button_faster: 9             # 속도 증가 (Start)
  button_gripper_open: 12      # 그리퍼 열기 (Triangle)
  button_gripper_close: 14     # 그리퍼 닫기 (Cross)
```

---

### 속도 조정

이동 속도를 변경하려면 `config/servo_config.yaml` 수정:

```yaml
scale:
  linear: 0.4          # 최대 선형 속도 (m/s)
  rotational: 0.8      # 최대 각속도 (rad/s)
  joint: 0.5           # 조인트 속도 배율 (0.0~1.0)

# 또는 전역 스케일 조정
override_velocity_scaling_factor: 0.4      # 0.0~1.0 (낮을수록 느림)
override_acceleration_scaling_factor: 0.4  # 0.0~1.0 (낮을수록 부드러움)
```

**권장 설정:**
- 🐢 **느리고 안전**: `linear: 0.2, rotational: 0.4`
- 🚶 **보통 속도**: `linear: 0.4, rotational: 0.8` (기본값)
- 🏃 **빠른 속도**: `linear: 0.8, rotational: 1.5`

---

### 조이스틱 데드존 조정

조이스틱이 너무 민감하거나 반응이 없다면:

```yaml
# launch/servo_joystick.launch.py 파일에서
joy_node = Node(
    package='joy',
    executable='joy_node',
    parameters=[{
        'deadzone': 0.05,      # 0.0~1.0 (기본값: 0.05)
        'autorepeat_rate': 20.0  # Hz
    }]
)
```

---

### 그리퍼 조인트 한계 수정

그리퍼의 움직임 범위를 변경하려면 `urdf/panda_with_robotiq.urdf` 수정:

```xml
<joint name="robotiq_left_outer_knuckle_joint" type="revolute">
    <limit 
        lower="0.0"        <!-- 최소 각도 (라디안) -->
        upper="0.8"        <!-- 최대 각도 (라디안) -->
        velocity="2.0"     <!-- 최대 속도 (rad/s) -->
        effort="1000"      <!-- 최대 토크 (Nm) -->
    />
</joint>
```

**참고:** 0.8 rad ≈ 45.8° (그리퍼 완전히 닫힌 상태)

---

### 안전 한계 조정

특이점(singularity) 및 충돌 감지 설정 (`config/servo_config.yaml`):

```yaml
# 특이점 회피
lower_singularity_threshold: 17.0       # 감속 시작 (도)
hard_stop_singularity_threshold: 30.0   # 완전 정지 (도)

# 조인트 한계 여유
joint_limit_margin: 0.1                 # 조인트 한계에서 0.1 rad 전에 감속

# 충돌 감지
check_collisions: true
collision_check_rate: 10.0              # Hz
self_collision_proximity_threshold: 0.01     # 자기 충돌 거리 (m)
scene_collision_proximity_threshold: 0.02    # 환경 충돌 거리 (m)
```

---

## 🔧 트러블슈팅

### 문제 1: 조이스틱이 인식되지 않음

#### 증상
- `/dev/input/js0` 파일이 없음
- `joy_node` 실행 시 에러

#### 해결 방법

```bash
# 1. 조이스틱 장치 확인
ls -l /dev/input/js*

# 2. 권한 부여 (임시)
sudo chmod 666 /dev/input/js0

# 3. 사용자를 input 그룹에 추가 (영구)
sudo usermod -a -G input $USER
# ⚠️ 로그아웃 후 다시 로그인 필요

# 4. 확인
groups $USER  # 'input' 그룹이 있는지 확인
```

#### 다른 조이스틱 장치 사용

`launch/servo_joystick.launch.py`에서 수정:

```python
joy_node = Node(
    package='joy',
    executable='joy_node',
    parameters=[{
        'dev': '/dev/input/js1',  # js0 → js1로 변경
        'deadzone': 0.05
    }]
)
```

---

### 문제 2: RViz에서 로봇이 보이지 않음

#### 증상
- RViz 창이 열리지만 로봇 모델이 표시되지 않음
- 경고: "No transform from [panda_link0] to [world]"

#### 해결 방법

```bash
# 1. TF 트리 확인
ros2 run tf2_tools view_frames
# 생성된 frames.pdf 파일을 확인

# 2. Robot description 토픽 확인
ros2 topic echo /robot_description --once

# 3. RViz Fixed Frame 확인
# RViz → Global Options → Fixed Frame을 'panda_link0'으로 설정

# 4. RobotModel Display 추가
# RViz → Add → RobotModel
```

---

### 문제 3: 빌드 오류

#### 증상
- `colcon build` 실행 시 에러

#### 해결 방법

```bash
# 1. 캐시 삭제 후 재빌드
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select custom_panda_description

# 2. 의존성 설치
sudo apt update
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-moveit-servo \
    ros-humble-joy

# 3. 소싱 확인
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

### 문제 4: MoveIt Servo가 작동하지 않음

#### 증상
- 조이스틱을 움직여도 로봇이 반응하지 않음

#### 해결 방법

```bash
# 1. 토픽 확인
ros2 topic list | grep servo

# 2. Twist 명령 확인
ros2 topic echo /servo_node/delta_twist_cmds

# 3. Servo 상태 확인
ros2 topic echo /servo_node/status

# 4. Deadman 버튼 확인
# L1 버튼을 누르고 있는지 확인!
ros2 topic echo /joy  # buttons[10]이 1인지 확인
```

---

### 문제 5: 그리퍼가 움직이지 않음

#### 증상
- Triangle/Cross 버튼을 눌러도 그리퍼 반응 없음

#### 임시 해결 (수동 제어)

```bash
# 그리퍼 조인트 수동 제어
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['robotiq_left_outer_knuckle_joint'], position: [0.8]}" \
--once
```

---

### 문제 6: 패키지를 찾을 수 없음

#### 증상
- `Package 'custom_panda_description' not found`

#### 해결 방법

```bash
# 1. 빌드 확인
cd ~/ros2_ws
colcon build --packages-select custom_panda_description

# 2. 소싱 확인
source install/setup.bash

# 3. 패키지 확인
ros2 pkg list | grep custom_panda

# 4. 환경 변수 확인
echo $ROS_PACKAGE_PATH
```

---

## 📝 개발 로드맵

### ✅ 완료된 작업

- [x] Robotiq 2F-85 그리퍼 URDF 모델링 (2026-01-13)
- [x] MoveIt SRDF 설정
- [x] ros2_control 인터페이스 구성
- [x] RViz 시각화 설정
- [x] 조이스틱 매핑 및 Servo 설정
- [x] 런치 파일 작성

### ⏳ 진행 예정

- [ ] Gazebo 시뮬레이션 통합
- [ ] MoveIt 모션 플래닝 설정
- [ ] 궤적 생성 및 실행 예제
- [ ] 그리퍼 액션 서버 구현
- [ ] 실제 로봇 하드웨어 인터페이스
- [ ] 픽 앤 플레이스 데모

---

## 📚 참고 자료

### 공식 문서

- **MoveIt 2 Documentation**: [https://moveit.picknik.ai/](https://moveit.picknik.ai/)
- **MoveIt Servo Tutorial**: [Realtime Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- **ROS 2 Humble Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)

### 관련 패키지

- **joy Package**: [https://index.ros.org/p/joy/](https://index.ros.org/p/joy/)
- **robot_state_publisher**: [https://github.com/ros/robot_state_publisher](https://github.com/ros/robot_state_publisher)
- **moveit_resources**: [https://github.com/moveit/moveit_resources](https://github.com/moveit/moveit_resources)

### 하드웨어

- **Franka Panda**: [https://www.franka.de/](https://www.franka.de/)
- **Robotiq 2F-85 Gripper**: [https://robotiq.com/products/2f85-140-adaptive-robot-gripper](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
- **Robotiq Manual**: [User Documentation](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_PDF_20190206.pdf)

### 튜토리얼

- **URDF Tutorial**: [http://wiki.ros.org/urdf/Tutorials](http://wiki.ros.org/urdf/Tutorials)
- **MoveIt Getting Started**: [MoveIt Quickstart](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)

---

## 👨‍💻 프로젝트 정보

| 항목 | 내용 |
|------|------|
| **패키지 이름** | custom_panda_description |
| **버전** | 0.0.1 |
| **생성일** | 2026-01-13 |
| **ROS 버전** | ROS 2 Humble |
| **기반 패키지** | moveit_resources_panda_description |
| **라이선스** | TODO |

---

## 🤝 기여 방법

이슈나 개선 사항이 있으시면:

1. 버그를 발견하면 이슈를 등록해주세요
2. 새로운 기능 제안은 환영합니다
3. Pull Request를 보내주세요

---

## 📧 문의

질문이나 도움이 필요하시면:
- GitHub Issues
- 이메일: etri@todo.todo

---

**Happy Robotics! 🤖✨**
