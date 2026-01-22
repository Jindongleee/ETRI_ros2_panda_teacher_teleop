# Panda Teleop OMY L100

OMY_L100 리더암을 사용한 Franka Emika Panda 로봇 텔레오퍼레이션 시스템

## 시스템 구조

```
OMY_L100 리더암 (관절값)
        ↓
/leader/joint_states
        ↓
omy_l100_to_twist_node (FK + Offset + Clutch)
        ↓
/servo_node/delta_twist_cmds (속도 명령)
        ↓
servo_node (MoveIt Servo, IK)
        ↓
/panda_arm_controller/joint_trajectory
        ↓
trajectory_to_joint_states
        ↓
/joint_states → robot_state_publisher → TF
```

## 주요 기능

### 1. Forward Kinematics (FK)
- OMY_L100 리더암의 관절값을 TF를 통해 엔드이펙터 위치/자세로 변환
- 이전 프레임과 비교하여 속도 계산
- TwistStamped 메시지로 발행

### 2. Inverse Kinematics (IK)
- MoveIt Servo를 사용하여 Panda 로봇 전용 IK 수행
- Cartesian 속도 명령을 관절 궤적으로 변환
- 특이점 회피, 관절 한계 보호 기능 포함

### 3. Offset 보정
- 리더암과 팔로워암의 엔드이펙터 초기 위치 차이를 자동 보정
- **수식**: `P(offset) = P(follower_start) - P(leader_start)`
- 첫 번째 움직임에서 자동 계산
- Clutch 토글 시 재계산

### 4. Clutch 제어 (안전 기능)
- **키보드 'b' 키를 누르고 있을 때만** 텔레오퍼레이션 활성화
- **Hold-to-Run**: 'b' 키를 놓으면 즉시 멈춤 (안전!)
- Clutch OFF (키 놓음): 리더암 재배치 가능 (팔로워암 정지)
- Clutch ON (키 누름): Offset 재계산 및 정상 동작
- 팔 길이 차이 극복 가능

### 5. 그리퍼 제어
- OMY_L100 그리퍼 → Panda Robotiq 그리퍼 매핑
- 범위 자동 변환

## 빠른 시작

### 1. 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select panda_teleop_omy_l100 --symlink-install
source install/setup.bash
```

### 2. 실행

```bash
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py
```

**옵션**:
- `use_rviz:=false` - RViz 비활성화
- `port_name:=/dev/ttyUSB1` - OMY_L100 포트 변경

### 3. Clutch 사용 (안전 기능)

launch 실행 시 xterm 창이 자동으로 열립니다:
- **'b' 키 누르고 있기**: 텔레오퍼레이션 활성화 (로봇 동작)
- **'b' 키 놓기**: 텔레오퍼레이션 일시정지 (로봇 정지)
- **'q' 키**: 종료

**중요**: 'b' 키를 누르고 있는 동안만 로봇이 움직입니다! (Hold-to-Run 안전 기능)

**xterm이 없는 경우**:
```bash
sudo apt install xterm
```

## 파일 구조

```
panda_teleop_omy_l100/
├── launch/
│   └── panda_teleop_omy_l100.launch.py  # 메인 launch 파일
├── src/
│   ├── omy_l100_to_twist_node.py        # FK + Offset + Clutch
│   ├── trajectory_to_joint_states.py    # Trajectory → JointState
│   ├── omy_l100_to_gripper_node.py      # 그리퍼 매핑
│   └── clutch_control_node.py           # 키보드 Clutch 제어
├── config/
│   ├── servo_config.yaml                # MoveIt Servo 설정
│   └── kinematics.yaml                  # IK solver 설정
├── CLUTCH_USAGE.md                      # Clutch 사용 가이드
├── README_KR.md                         # 이 파일
└── test_system.sh                       # 시스템 진단 스크립트
```

## 문제 해결

### servo_node가 실행되지 않음

**진단**:
```bash
cd ~/ros2_ws/panda_teleop_omy_l100
./test_system.sh
```

**확인 사항**:
1. `ros2 node list` → servo_node 확인
2. `servo_config.yaml` 문법 오류 확인
3. `kinematics.yaml` 파일 존재 확인
4. Launch 로그에서 에러 메시지 확인

**수동 시작**:
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
```

### 팔로워암이 움직이지 않음

**체크 리스트**:
1. ✓ **'b' 키를 누르고 있는가?** (가장 흔한 원인!)
2. ✓ servo_node 실행 중인가?
3. ✓ Clutch가 ON인가? (`ros2 topic echo /clutch/active` → **true**)
4. ✓ 리더암이 움직이고 있는가?
5. ✓ Twist 명령이 발행되는가? (`ros2 topic echo /servo_node/delta_twist_cmds`)
6. ✓ Joint states가 발행되는가? (`ros2 topic echo /joint_states`)

### 팔로워암이 반대 방향으로 움직임

**증상**: 리더암을 위로 올리면 팔로워암이 아래로 가거나, 방향이 반대

**해결**:
launch 파일에서 축 반전 파라미터 조정:
```python
'invert_linear_z': -1.0,  # Z축 반전 (위로 올리면 위로 가도록)
# 다른 축도 필요시 반전
'invert_linear_x': -1.0,  # X축 반전
'invert_linear_y': -1.0,  # Y축 반전
```

### Offset이 이상함

**재계산 방법**:
1. Clutch OFF ('b' 키 놓음)
2. 리더암을 원하는 위치로 이동
3. 팔로워암을 원하는 위치로 이동 (수동 또는 다른 방법)
4. Clutch ON ('b' 키 누름) → Offset 자동 재계산

## 개발 정보

### 노드 설명

#### omy_l100_to_twist_node
- **구독**: `/leader/joint_states`, `/clutch/active`
- **발행**: `/servo_node/delta_twist_cmds`
- **주요 기능**: FK, 속도 계산, Offset 적용, Clutch 처리

#### clutch_control_node
- **발행**: `/clutch/active` (Bool)
- **주요 기능**: 키보드 입력 감지, Clutch 상태 토글

#### servo_node (MoveIt Servo)
- **구독**: `/servo_node/delta_twist_cmds`, `/joint_states`
- **발행**: `/panda_arm_controller/joint_trajectory`
- **주요 기능**: IK, Jacobian 계산, 관절 한계 보호

#### trajectory_to_joint_states
- **구독**: `/panda_arm_controller/joint_trajectory`, `/gripper/position`
- **발행**: `/joint_states`
- **주요 기능**: Trajectory와 그리퍼를 JointState로 통합

### 파라미터 튜닝

**omy_l100_to_twist_node** (launch 파일에서 수정):
```python
'linear_scale': 10.0,    # 선속도 배율 (클수록 빠름)
'angular_scale': 5.0,    # 각속도 배율
'publish_rate': 100.0,   # 발행 주파수 (Hz)
```

**servo_config.yaml**:
```yaml
scale:
  linear: 1.0           # MoveIt Servo 선속도 배율
  rotational: 1.0       # MoveIt Servo 각속도 배율
publish_period: 0.01    # Servo 제어 주기 (100 Hz)
```

## 라이선스

이 패키지는 ROS2 생태계의 일부로 개발되었습니다.

## 참고 자료

- [MoveIt Servo Documentation](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [OMY_L100 GitHub](https://github.com/ROBOTIS-GIT/open_manipulator)
- [Franka Panda](https://frankaemika.github.io/)

## 버전 정보

- **버전**: 1.0.0
- **날짜**: 2026-01-22
- **주요 기능**: Offset 보정, Clutch 제어, FK/IK 텔레오퍼레이션
