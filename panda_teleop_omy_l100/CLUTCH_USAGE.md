# Clutch 기능 사용 가이드

## 개요

Clutch 기능은 **안전 장치**로, 'b' 키를 **누르고 있을 때만** 텔레오퍼레이션이 활성화됩니다. 이를 통해 리더암(OMY_L100)과 팔로워암(Panda) 간의 엔드이펙터 위치 차이를 보정하고, 안전하게 제어할 수 있습니다.

## 구조

### 데이터 흐름

```
OMY_L100 리더암 → /leader/joint_states
                    ↓
          omy_l100_to_twist_node (FK + Offset + Clutch)
                    ↓
          /servo_node/delta_twist_cmds
                    ↓
          servo_node (IK)
                    ↓
          /panda_arm_controller/joint_trajectory
                    ↓
          trajectory_to_joint_states
                    ↓
          /joint_states → robot_state_publisher → TF
```

### Clutch 제어

```
키보드 'b' 입력 → clutch_control_node
                       ↓
                 /clutch/active (Bool)
                       ↓
              omy_l100_to_twist_node
```

## 사용 방법

### 1. 시스템 시작

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py
```

### 2. Clutch 제어

launch 실행 시 **xterm** 창이 별도로 열립니다. 이 창에서:

- **'b' 키 누르고 있기**: Clutch 활성화 → 텔레오퍼레이션 동작
- **'b' 키 놓기**: Clutch 비활성화 → 텔레오퍼레이션 일시정지 (안전)
- **'q' 키**: Clutch 노드 종료

**중요**: 'b' 키를 누르고 있는 동안만 로봇이 움직입니다. 이는 안전 기능입니다!

### 3. Clutch 사용 시나리오

#### 시나리오 1: 정상 작업 (Hold-to-Run)

1. launch 실행 후 시스템이 초기화됩니다 (약 6초)
2. **'b' 키를 누릅니다** → Clutch 활성화
   - 터미널: `[CLUTCH] PRESSED - Teleoperation ENABLED`
3. **'b' 키를 누른 상태에서 리더암을 움직입니다**
   - 첫 움직임에서 **자동으로 offset이 계산**됩니다
   - `P(offset) = P(follower_start) - P(leader_start)`
   - 팔로워암이 리더암을 따라 움직입니다
4. **'b' 키를 놓으면** 즉시 멈춥니다 (안전!)
   - 터미널: `[CLUTCH] RELEASED - Teleoperation PAUSED (safety)`

#### 시나리오 2: 리더암 재배치

리더암이 작업 공간 한계에 도달했거나 재배치가 필요한 경우:

1. **'b' 키를 놓습니다** → Clutch OFF
   - 터미널: `[CLUTCH] RELEASED - Teleoperation PAUSED (safety)`
   - 팔로워암이 멈춥니다 (0 속도 명령 발행)

2. **리더암을 원하는 위치로 이동**
   - 예: 작업 공간 중앙으로 복귀
   - 팔로워암은 움직이지 않습니다 (안전!)

3. **'b' 키를 다시 누릅니다** → Clutch ON
   - 터미널: `[CLUTCH] PRESSED - Teleoperation ENABLED`
   - 새로운 offset이 자동 계산됩니다
   - `P(offset_new) = P(follower_current) - P(leader_current)`

4. **'b' 키를 누른 상태에서 작업 계속**
   - 리더암 움직임이 다시 팔로워암에 반영됩니다

## 파라미터

### omy_l100_to_twist_node 파라미터

launch 파일에서 조정 가능:

```python
parameters=[{
    'base_frame': 'leader_link0',
    'ee_frame': 'leader_link7',
    'follower_ee_frame': 'panda_link8',
    'joint_states_topic': '/leader/joint_states',
    'linear_scale': 10.0,     # 선속도 스케일
    'angular_scale': 5.0,     # 각속도 스케일
    'publish_rate': 100.0,    # 발행 주파수 (Hz)
    'initial_offset_x': 0.0,  # 초기 offset (선택사항)
    'initial_offset_y': 0.0,
    'initial_offset_z': 0.0
}]
```

## 로그 확인

### omy_l100_to_twist_node 로그

```bash
# Offset 계산 로그
[OFFSET] Auto-initializing offset on first motion...
[OFFSET] Recalculated - Position: [0.123, -0.456, 0.789] m
[OFFSET] Orientation: [1.000, 0.000, 0.000, 0.000]

# Clutch 상태 로그
[CLUTCH] PRESSED - Teleoperation ENABLED
[CLUTCH] RELEASED - Teleoperation PAUSED (safety)
```

### clutch_control_node 로그

```bash
Clutch Control Node started
===================================
HOLD "b" key to ENABLE teleoperation
RELEASE "b" key to PAUSE teleoperation
===================================
Current state: PAUSED (waiting for clutch)

>>> Clutch PRESSED - Teleoperation ACTIVE
>>> Clutch RELEASED - Teleoperation PAUSED
```

## 문제 해결

### Clutch xterm 창이 열리지 않음

**원인**: xterm이 설치되지 않음

**해결**:
```bash
sudo apt install xterm
```

### Offset이 계산되지 않음

**원인**: TF 변환이 준비되지 않음

**해결**:
1. 시스템 초기화 대기 (launch 후 6초)
2. **'b' 키를 누른 상태에서** 리더암을 약간 움직여서 첫 번째 pose 획득
3. 'b' 키를 놓았다가 다시 눌러서 수동으로 offset 재계산

### 팔로워암이 움직이지 않음

**체크 리스트**:
1. **'b' 키를 누르고 있나요?** → 안전 기능으로 'b' 키를 눌러야만 동작합니다!
2. `ros2 node list` → servo_node 실행 확인
3. `ros2 topic echo /servo_node/delta_twist_cmds` → twist 명령 발행 확인
4. `ros2 topic echo /clutch/active` → clutch 상태 확인 (**true = active**)
5. `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger` → servo 수동 시작

## 고급 기능

### 수동 Offset 설정

launch 파일에서 초기 offset 설정:

```python
omy_l100_to_twist_node = Node(
    ...
    parameters=[{
        ...
        'initial_offset_x': 0.5,  # 500mm offset
        'initial_offset_y': 0.0,
        'initial_offset_z': 0.2   # 200mm offset
    }]
)
```

### Clutch 없이 Offset 재계산

Python 코드나 별도 노드에서 `/clutch/active` 토픽 발행:

```python
from std_msgs.msg import Bool

# Clutch ON (텔레오퍼레이션 활성화)
clutch_msg = Bool()
clutch_msg.data = True
clutch_pub.publish(clutch_msg)

# 작업 수행...

# Clutch OFF (텔레오퍼레이션 일시정지)
clutch_msg.data = False
clutch_pub.publish(clutch_msg)
```

## 참고

- **안전 기능**: 'b' 키를 누르고 있을 때만 로봇이 움직입니다 (Hold-to-Run)
- Offset은 **위치(position)**와 **자세(orientation)** 모두 포함
- Clutch 상태 변경 시 이전 pose가 리셋되어 갑작스러운 움직임 방지
- Clutch OFF (키를 놓을 때) 중에는 0 속도 명령이 지속적으로 발행됨
- 자동 offset 계산은 첫 번째 유효한 pose 획득 시 1회 수행
- Clutch ON (키를 누를 때)마다 offset이 재계산됨

## 관련 파일

- **노드**: `src/clutch_control_node.py`, `src/omy_l100_to_twist_node.py`
- **Launch**: `launch/panda_teleop_omy_l100.launch.py`
- **설정**: `config/servo_config.yaml`, `config/kinematics.yaml`
