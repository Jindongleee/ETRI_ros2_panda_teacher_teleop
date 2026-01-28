# 🚀 Quick Reference Card

## 실행 명령어

### OMY L100
```bash
# 기본 실행
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py

# 데이터 수집
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py enable_data_collection:=true
```

### 조이스틱
```bash
# 기본 실행
ros2 launch panda_teleop_joystick joystick_teleop.launch.py

# 데이터 수집
ros2 launch panda_teleop_joystick joystick_teleop.launch.py enable_data_collection:=true
```

---

## 조이스틱 조작법

| 입력 | 기능 | 속도 |
|------|------|------|
| **Left Stick X ←→** | Linear X (좌우) | 0.5 m/s |
| **Left Stick Y ↑↓** | Linear Y (전후) | 0.5 m/s |
| **L1 / L2** | Linear Z (상하) | 0.5 m/s |
| **Right Stick Y ↑↓** | Angular X (Roll) | 1.0 rad/s |
| **Right Stick X ←→** | Angular Y (Pitch) | 1.0 rad/s |
| **R1 / R2** | Angular Z (Yaw) | 1.0 rad/s |
| **○ (Circle)** | 그리퍼 열기 | -0.02 |
| **□ (Square)** | 그리퍼 닫기 | +0.02 |
| **클러치 페달** | 로봇 활성화 | - |

---

## 주요 토픽

```bash
# 클러치 상태 확인
ros2 topic echo /clutch/active

# Twist 명령 확인
ros2 topic echo /servo_node/delta_twist_cmds

# 관절 상태 확인
ros2 topic echo /joint_states

# TF 확인
ros2 run tf2_ros tf2_echo panda_link0 gripper_tip_link
```

---

## 데이터 분석

```bash
# 데이터셋 검증
python3 ~/ros2_ws/scripts/validate_dataset.py \
    ~/ros2_ws/data/omy_l100/session_YYYYMMDD_HHMMSS/

# 에피소드 시각화
python3 ~/ros2_ws/scripts/visualize_episode.py \
    ~/ros2_ws/data/omy_l100/session_YYYYMMDD_HHMMSS/episode_001.jsonl
```

---

## 패키지 구조

```
panda_common/              → 공통 유틸리티
panda_teleop_omy_l100/    → OMY L100 컨트롤러
panda_teleop_joystick/    → 조이스틱 컨트롤러
custom_panda_description/ → Panda URDF
```

---

## 문제 해결

### 로봇이 안 움직임
1. 클러치 페달 확인: `ros2 topic echo /clutch/active`
2. Twist 명령 확인: `ros2 topic echo /servo_node/delta_twist_cmds`
3. 노드 실행 확인: `ros2 node list`

### RViz에 로봇 안 보임
1. TF 확인: `ros2 run tf2_ros tf2_echo panda_link0 gripper_tip_link`
2. joint_states 확인: `ros2 topic echo /joint_states`

### 데이터 수집 안 됨
1. 노드 확인: `ros2 node list | grep data_collection`
2. 클러치 활성화 후 로봇 움직이기

---

📘 **상세 정보**: `ARCHITECTURE.md` 참조
