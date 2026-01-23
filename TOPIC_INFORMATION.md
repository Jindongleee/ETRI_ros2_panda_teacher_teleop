# 시스템 토픽 정보 (Topic Information)

## 시스템 개요

## 2. OMY L100 리더암 텔레오퍼레이션 모드 (OMY L100 Leader-Follower Teleoperation Mode)

### 2.1 omy_l100_leader_ai (리더암 시스템)
- **Pub**: `/leader/joint_states` (MT: `sensor_msgs/msg/JointState`)
  - OMY L100 리더암의 관절 상태
    header:
      stamp:
        sec: 1769131809
        nanosec: 552979969
      frame_id: base_link
    name:
    - joint6
    - joint5
    - joint4
    - joint1
    - joint3
    - joint2
    - rh_r1_joint
    position:
    - 0.030679615757505996
    - 1.5186409800065777
    - -0.5476311412753807
    - 0.030679615757505996
    - 1.871456561220275
    - -1.0998642249142119
    - -0.35128160042601886
    velocity:
    - -0.239691227
    - 0.0239691227
    - 0.0958764908
    - -0.1677838589
    - -2.2530975338
    - 1.438147362
    - 3.4275845461000003
    effort:
    - -0.0029024390219603248
    - 0.0
    - -0.07159349587502134
    - -0.0045
    - -0.5984999999999999
    - 1.008
    - 0.010064205444023686
    ---


### 2.2 omy_l100_to_twist_node
- **Pub**: `/servo_node/delta_twist_cmds` (MT: `geometry_msgs/msg/TwistStamped`)
  - 리더암 엔드이펙터 움직임을 변환한 팔로워암 속도 명령
    header:
      stamp:
        sec: 1769131809
        nanosec: 526113534
      frame_id: panda_link8
    twist:
      linear:
        x: 0.02752228327680855
        y: 0.008586265535301743
        z: -0.9995843135860969
      angular:
        x: 0.12986621144266458
        y: 1.9819310022048486
        z: -0.23470038266865503
    ---


### 2.3 omy_l100_to_gripper_node
- **Pub**: `/gripper/position` (MT: `std_msgs/msg/Float64`)
  - 리더암 그리퍼를 변환한 팔로워암 그리퍼 위치 명령
    data: 0.21285434387786906
    ---
    data: 0.21776308239910308
    ---
    data: 0.2220582286051828
    ---
    data: 0.22819415175672547
    ---
    data: 0.23248929796280518
    ---
    data: 0.23678444416888508
    ---
    data: 0.2416931826901191
    ---
    data: 0.24598832889619882
    ---
    data: 0.25335143667805
    ---
    data: 0.2594873598295925
    ---

### 2.4 servo_node (MoveIt Servo)
- **Pub**: `/panda_arm_controller/joint_trajectory` (MT: `trajectory_msgs/msg/JointTrajectory`)
  - 역기구학으로 계산된 관절 궤적 명령
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: panda_link0
    joint_names:
    - panda_joint1
    - panda_joint2
    - panda_joint3
    - panda_joint4
    - panda_joint5
    - panda_joint6
    - panda_joint7
    points:
    - positions:
      - -0.022078108365264908
      - -0.7780648290903465
      - 0.005392359727014979
      - -2.3828612692787816
      - -0.040884196614071666
      - 1.6230490298422027
      - 0.7997726106997719
      velocities:
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      accelerations: []
      effort: []
      time_from_start:
        sec: 0
        nanosec: 10000000
    ---


### 2.5 trajectory_to_joint_states (panda_teleop_omy_l100)
- **Pub**: `/joint_states` (MT: `sensor_msgs/msg/JointState`)
  - 로봇의 모든 관절 상태 (위치, 속도)
    header:
      stamp:
        sec: 1769131809
        nanosec: 554407344
      frame_id: ''
    name:
    - panda_joint1
    - panda_joint2
    - panda_joint3
    - panda_joint4
    - panda_joint5
    - panda_joint6
    - panda_joint7
    - finger_joint
    - left_inner_knuckle_joint
    - left_inner_finger_joint
    - right_inner_knuckle_joint
    - right_inner_finger_joint
    - right_outer_knuckle_joint
    position:
    - -0.022078108365264908
    - -0.7780648290903465
    - 0.005392359727014979
    - -2.3828612692787816
    - -0.040884196614071666
    - 1.6230490298422027
    - 0.7997726106997719
    - 0.2594873598295925
    - 0.2594873598295925
    - -0.2594873598295925
    - -0.2594873598295925
    - 0.2594873598295925
    - -0.2594873598295925
    velocity:
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    effort: []
    ---

### 2.6 robot_state_publisher
- **Pub**: 
  - `/tf` (MT: `tf2_msgs/msg/TFMessage`)
  - `/tf_static` (MT: `tf2_msgs/msg/TFMessage`)

### * clutch_pedal_node
- **Pub**: `/clutch/active` (MT: `std_msgs/msg/Bool`)
  - 클러치 페달 상태 (True: 활성/텔레오퍼레이션 활성화, False: 비활성/일시정지)

    data: true
    ---
    data: false
    ---
    data: false
    ---
    data: true
    ---


## 5. 시스템 아키텍처 흐름

### 조이스틱 모드:
```
clutch_pedal_node → /clutch/active
joy_node → joy_to_twist_node → servo_node → trajectory_to_joint_states → robot_state_publisher
         → joy_to_gripper_node → trajectory_to_joint_states
         (joy_to_twist_node는 /clutch/active를 구독하여 안전 제어)
```

### 텔레오퍼레이션 모드:
```
omy_l100_leader (리더암 joint_states 발행)
    
    ↓ /leader/joint_states

omy_l100_to_twist_node (리더암 EEF pose를 FK로 계산 → 속도(twist) 계산 → 스케일 적용 → 팔로워암 twist 명령 발행)
    
    ↓ /servo_node/delta_twist_cmds

servo_node (MoveIt Servo: twist 명령을 IK로 변환하여 관절 궤적 계산)
    
    ↓ /panda_arm_controller/joint_trajectory

trajectory_to_joint_states (관절 궤적을 joint_states로 변환)

    ↓ /joint_states

robot_state_publisher (joint_states를 TF로 변환하여 RViz 시각화)

omy_l100_leader (리더암 joint_states 발행)

    ↓ /leader/joint_states

omy_l100_to_gripper_node (리더암 그리퍼 값을 팔로워암 그리퍼 명령으로 변환)

    ↓ /gripper/position
    
trajectory_to_joint_states (그리퍼 명령을 joint_states에 반영)

clutch_pedal_node → /clutch/active (모든 제어 노드에 클러치 신호 전달하여 안전 제어)
```
