# OMY-L100 관절값 토픽 발행 설정 가이드

## 목표
OMY-L100의 관절값을 ROS 2 토픽으로 발행하고 `ros2 topic echo`로 확인하기

## 문제 상황
- OMY-L100 모델 실행 필요
- 관절값만 토픽으로 발행되어야 함
- Ubuntu 22.04 환경 (ROS 2 Humble)
- USB로 하드웨어 연결됨 (`/dev/ttyUSB0`)
- **주의**: OMY-L100 지원은 `jazzy` 브랜치에만 있음 (Humble 브랜치에는 없음)

## 해결 과정

### 1. 문제 발견
- 처음에는 `open_manipulator_x_bringup` 패키지를 사용했으나, 이는 **OpenMANIPULATOR-X** 모델용
- OMY-L100 전용 패키지인 `open_manipulator_bringup`이 필요함

### 2. 패키지 확인 및 빌드
```bash
cd /home/etri/ros2_ws/omy_l100/open_manipulator
git checkout jazzy  # OMY 지원 브랜치로 전환 (Humble 브랜치에는 OMY-L100 지원 없음)
git pull origin jazzy

# OMY-L100 전용 패키지 빌드
cd /home/etri/ros2_ws
colcon build --packages-select open_manipulator_description open_manipulator_bringup
source install/setup.bash
```

**중요**: ROS 2 Humble을 사용 중이더라도 `jazzy` 브랜치를 사용해야 합니다. OMY-L100 지원은 `jazzy` 브랜치에만 포함되어 있으며, 하위 호환성으로 인해 Humble에서도 정상 동작합니다.

### 3. USB 포트 확인
```bash
ls -la /dev/ttyUSB0  # FTDI USB Serial Converter 확인
```

### 4. 실행 명령
```bash
# 터미널 1: OMY-L100 Launch 파일 실행 (실제 하드웨어 연결)
cd /home/etri/ros2_ws
source install/setup.bash
ros2 launch open_manipulator_bringup omy_l100_follower_ai.launch.py port_name:=/dev/ttyUSB0 start_rviz:=false

# 터미널 2: 관절값 확인
cd /home/etri/ros2_ws
source install/setup.bash
ros2 topic echo /joint_states
```

## 결과

### 토픽 정보
- **토픽 이름**: `/joint_states`
- **토픽 타입**: `sensor_msgs/msg/JointState`
- **발행 주기**: 약 300Hz (controller_manager 설정에 따라)

### 발행되는 관절 (OMY-L100: 6-DOF + 로봇 핸드)
- `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6` (6-DOF 팔 관절)
- `rh_r1_joint` (로봇 핸드 관절)

### 메시지 구조
```yaml
header:
  stamp: {sec, nanosec}
  frame_id: base_link
name: [joint1, joint2, joint3, joint4, joint5, joint6, rh_r1_joint]
position: [각 관절의 위치값 (라디안)]
velocity: [각 관절의 속도값]
effort: [각 관절의 토크값]
```

## 핵심 포인트
1. **올바른 브랜치 사용**: `jazzy` 브랜치 (OMY-L100 지원은 jazzy에만 있음, Humble 브랜치에는 없음)
2. **ROS 2 버전 호환성**: ROS 2 Humble 사용 중이더라도 jazzy 브랜치 코드가 정상 동작함 (하위 호환성)
3. **올바른 패키지 사용**: `open_manipulator_bringup` (OMY 전용) vs `open_manipulator_x_bringup` (OM-X 전용)
4. **올바른 Launch 파일**: `omy_l100_follower_ai.launch.py` 또는 `omy_l100_leader_ai.launch.py`
5. **실제 하드웨어 연결**: USB 포트 (`/dev/ttyUSB0`)로 DYNAMIXEL 모터와 통신
6. `joint_state_broadcaster` 컨트롤러가 자동으로 `/joint_states` 토픽 발행

## 차이점 정리
| 항목 | OpenMANIPULATOR-X | OMY-L100 |
|------|-------------------|----------|
| 패키지 | `open_manipulator_x_bringup` | `open_manipulator_bringup` |
| 관절 수 | 4-DOF + 그리퍼 | 6-DOF + 로봇 핸드 |
| Launch 파일 | `hardware.launch.py` | `omy_l100_follower_ai.launch.py` |
| URDF 경로 | `open_manipulator_x/urdf/` | `open_manipulator/urdf/omy_l100/` |
| 보드레이트 | 1,000,000 bps | 4,000,000 bps |
| 업데이트 레이트 | 100 Hz | 300 Hz |

## 통신 오류 관련 참고사항

### 발생 가능한 오류
실행 중 다음과 같은 DYNAMIXEL 통신 오류가 발생할 수 있습니다:
```
[ERROR] [dynamixel_hardware_interface]: Communication Fail --> BULK_READ_FAIL
FastBulkRead Rx Fail [Dxl Size : 7] [Error code : -3002]
```

### 오류 코드 의미
- **-3002 (SDK_COMM_RX_CORRUPT)**: 손상된 상태 패킷 수신
- 이는 통신 케이블 노이즈, 전원 불안정, 또는 높은 업데이트 레이트로 인한 일시적 오류일 수 있습니다

### 중요 사항
- **관절값은 정상적으로 발행됨**: 오류가 발생해도 `/joint_states` 토픽은 약 300Hz로 정상 발행됩니다
- **기본 동작 정상**: 일부 통신 오류가 있어도 로봇 제어는 정상적으로 동작합니다
- 오류가 지속적으로 발생하거나 로봇 제어에 문제가 있다면:
  - 통신 케이블 점검
  - 전원 공급 안정성 확인
  - 보드레이트 설정 확인 (4,000,000 bps)
