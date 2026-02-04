# panda_teleop_vive

HTC Vive 컨트롤러로 Panda 로봇을 텔레오퍼레이션하기 위한 ROS2 패키지입니다.  
**panda_teleop_omy_l100**과 같은 구조로, 한 번의 launch로 Panda + Servo + Vive 브리지 + twist/gripper + 클러치 + (선택) 데이터 수집을 올립니다.

## 요구사항

- **vive_ros2** 패키지: Vive 포즈·버튼을 `/controller_data` (VRControllerData)로 발행
- Vive 드라이버 실행: `vive_input` + `vive_node` (SteamVR·OpenVR 환경 필요)
- (선택) 클러치 페달: `/clutch/active` — 발로 밟을 때만 로봇 이동

## 노드

| 노드 | 구독 | 발행 | 설명 |
|------|------|------|------|
| `vive_ros2_bridge_node` | `controller_data` (VRControllerData) | `/vive/controller/pose`, `/vive/controller/buttons` | vive_ros2 → PoseStamped + Joy 변환 |
| `vive_to_twist_node` | `/vive/controller/pose`, `/vive/controller/buttons`, `/clutch/active` | `/servo_node/delta_twist_cmds` | 포즈 변화 → Twist (Servo) |
| `vive_to_gripper_node` | `/vive/controller/buttons` | `/gripper/position` | 그립/트리거 → 그리퍼 열기/닫기 |

## 실행 (RViz 시뮬 한 PC 기준)

**한 번에 올리기** (vive_input + vive_node + Panda + Servo + RViz 포함)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch panda_teleop_vive panda_teleop_vive.launch.py
```

- t=0: vive_input(OpenVR), Panda, trajectory_to_joint_states  
- t=2s: vive_node (100 Hz)  
- t=3s: 브리지, Servo, twist/gripper, 클러치, RViz  
- t=6s: Servo 활성화 (start_servo), t=8s: 재시도  

옵션: `use_rviz:=false`, `enable_data_collection:=true`, `controller_role:=1` (왼손).

### RViz에서 Panda가 안 움직일 때

1. **클러치 또는 그립**을 누른 상태에서 컨트롤러를 움직여 보세요. (클러치 페달이 없으면 Vive 그립 버튼만으로도 동작합니다.)
2. **Servo가 켜졌는지 확인**: 터미널에서 `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger` 를 한 번 실행해 보세요.
3. **트래젝토리 발행 여부 확인**: 움직이면서 `ros2 topic echo /panda_arm_controller/joint_trajectory` 로 메시지가 나오는지 확인하세요. 나오면 Servo는 정상이고, `trajectory_to_joint_states` → `/joint_states` → RViz 경로가 동작하는 것입니다.

## 파라미터

- **vive_ros2_bridge_node**: `controller_data_topic`, `controller_role` (0=오른손, 1=왼손)
- **vive_to_twist_node**: `linear_scale`, `angular_scale`, `use_clutch`, `deadman_button`
- **vive_to_gripper_node**: `button_open`(1=그립), `button_close`(0=트리거), `step`, `min_position`, `max_position`

## 데이터 수집

`panda_common`의 `data_collection_config_vive.yaml`을 사용합니다.

```bash
ros2 launch panda_teleop_vive panda_teleop_vive.launch.py enable_data_collection:=true
```
