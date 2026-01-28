# 🎮 조이스틱 매핑 레퍼런스

**업데이트**: 2026-01-28  
**패키지**: `panda_teleop_joystick`

---

## 📋 전체 매핑 테이블

| 입력 | 타입 | 기능 | 설명 |
|------|------|------|------|
| **Left Stick X** | Axis 0 | Linear X | 좌우 이동 |
| **Left Stick Y** | Axis 1 | Linear Y | 전후 이동 |
| **L1 버튼** | Button 6 | Linear Z + | 위로 이동 |
| **L2 버튼** | Button 8 | Linear Z - | 아래로 이동 |
| **Right Stick Y** | Axis 3 | Angular X | Roll 회전 |
| **Right Stick X** | Axis 2 | Angular Y | Pitch 회전 |
| **R1 버튼** | Button 7 | Angular Z + | Yaw 회전 (시계방향) |
| **R2 버튼** | Button 9 | Angular Z - | Yaw 회전 (반시계) |
| **○ Circle** | Button 1 | Gripper Open | 그리퍼 열기 |
| **□ Square** | Button 3 | Gripper Close | 그리퍼 닫기 |
| **클러치 페달** | - | Safety | 안전 제어 (필수) |

---

## 🕹️ 조이스틱 레이아웃

```
        L1 (6)              R1 (7)
        L2 (8)              R2 (9)
        
    ┌─────────────────────────┐
    │                         │
    │   [Left Stick]          │
    │   Axis 0/1              │
    │   Linear X/Y            │
    │                         │
    │              [Right Stick]
    │              Axis 2/3    │
    │              Angular Y/X │
    │                         │
    │  □(3)  △(2)            │
    │  ✕(0)  ○(1)            │
    └─────────────────────────┘
```

---

## 📊 상세 매핑

### Linear 제어 (이동)

| 입력 | Axis/Button | 방향 | 최대 속도 |
|------|-------------|------|----------|
| Left Stick X → | Axis 0 | +X (우측) | 0.5 m/s |
| Left Stick X ← | Axis 0 | -X (좌측) | 0.5 m/s |
| Left Stick Y ↑ | Axis 1 | +Y (전방) | 0.5 m/s |
| Left Stick Y ↓ | Axis 1 | -Y (후방) | 0.5 m/s |
| L1 (Hold) | Button 6 | +Z (상승) | 0.5 m/s |
| L2 (Hold) | Button 8 | -Z (하강) | 0.5 m/s |

### Angular 제어 (회전)

| 입력 | Axis/Button | 회전 | 최대 속도 |
|------|-------------|------|----------|
| Right Stick Y ↑ | Axis 3 | +Roll (오른쪽 기울임) | 1.0 rad/s |
| Right Stick Y ↓ | Axis 3 | -Roll (왼쪽 기울임) | 1.0 rad/s |
| Right Stick X → | Axis 2 | +Pitch (위로 기울임) | 1.0 rad/s |
| Right Stick X ← | Axis 2 | -Pitch (아래로 기울임) | 1.0 rad/s |
| R1 (Hold) | Button 7 | +Yaw (시계방향) | 1.0 rad/s |
| R2 (Hold) | Button 9 | -Yaw (반시계방향) | 1.0 rad/s |

### Gripper 제어

| 입력 | Button | 동작 | 변화량 |
|------|--------|------|--------|
| ○ Circle | Button 1 | 열기 | -0.02 per press |
| □ Square | Button 3 | 닫기 | +0.02 per press |

---

## 🔒 안전 제어

### 클러치 페달
- **필수**: 클러치를 밟고 있어야만 로봇이 움직입니다
- **해제 시**: 모든 명령이 무시됩니다 (안전)
- **토픽**: `/clutch/active` (std_msgs/Bool)

---

## 🎯 사용 예시

### 기본 이동
1. **클러치 페달 밟기**
2. **Left Stick** 움직이기 → X/Y 평면 이동
3. **L1/L2** 버튼 → 상하 이동

### 회전
1. **클러치 페달 밟기**
2. **Right Stick** 움직이기 → Roll/Pitch 회전
3. **R1/R2** 버튼 → Yaw 회전

### 그리퍼
1. **Circle (○)** 연타 → 그리퍼 열기
2. **Square (□)** 연타 → 그리퍼 닫기

---

## ⚙️ 파라미터 설정

### joy_to_twist_node.py

```yaml
# Linear/Angular scales
linear_scale: 10.0          # Linear 속도 스케일
angular_scale: 10.0         # Angular 속도 스케일

# Button mapping
button_linear_z_up: 6       # L1
button_linear_z_down: 8     # L2
button_angular_z_pos: 7     # R1
button_angular_z_neg: 9     # R2

# Axis mapping
axis_linear_x: 0            # Left Stick X
axis_linear_y: 1            # Left Stick Y
axis_angular_x: 3           # Right Stick Y
axis_angular_y: 2           # Right Stick X

# Axis inversion
invert_linear_x: 1          # 1=normal, -1=inverted
invert_linear_y: 1
invert_angular_x: 1
invert_angular_y: 1

# Deadzone
axis_deadzone: 0.25         # 25% deadzone
```

### joy_to_gripper_node.py

```yaml
button_open: 1              # Circle button
button_close: 3             # Square button
step: 0.02                  # Position change per press
min_position: 0.0           # Fully open
max_position: 0.8           # Fully closed
```

---

## 🔧 커스터마이징

### 버튼 변경하기

Launch 파일에서 파라미터를 변경:

```python
ros2 launch panda_teleop_joystick joystick_teleop.launch.py \
    button_linear_z_up:=4 \
    button_angular_z_pos:=5
```

### 스케일 조정하기

```python
ros2 launch panda_teleop_joystick joystick_teleop.launch.py \
    linear_scale:=5.0 \
    angular_scale:=5.0
```

더 빠르게: 스케일 증가  
더 느리게: 스케일 감소

---

## 🐛 문제 해결

### 로봇이 안 움직임
```bash
# 1. 클러치 확인
ros2 topic echo /clutch/active
# → True가 나와야 함

# 2. 조이스틱 입력 확인
ros2 topic echo /joy
# → axes와 buttons 값 확인

# 3. Twist 명령 확인
ros2 topic echo /servo_node/delta_twist_cmds
# → 조이스틱 움직이면 값이 변해야 함
```

### 잘못된 방향으로 움직임
```bash
# Axis inversion 조정
ros2 launch panda_teleop_joystick joystick_teleop.launch.py \
    invert_linear_x:=-1 \
    invert_angular_y:=-1
```

### Dead zone이 너무 큼/작음
```bash
# Dead zone 조정 (0.0 ~ 1.0)
ros2 launch panda_teleop_joystick joystick_teleop.launch.py \
    axis_deadzone:=0.1  # 작게 (더 민감)
    # 또는
    axis_deadzone:=0.4  # 크게 (덜 민감)
```

---

## 📚 참고 문서

- **ARCHITECTURE.md** - 시스템 아키텍처
- **QUICK_REFERENCE.md** - 빠른 참조
- **README.md** - 프로젝트 개요

---

**최종 업데이트**: 2026-01-28  
**버전**: 2.0 (전면 개편)
