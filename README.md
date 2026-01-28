# Panda Robot Teleoperation Workspace

Multi-controller teleoperation system for Panda robot with Robotiq gripper.

## 📦 Package Structure

### Core Packages

#### **panda_common** ✨
Common utilities and nodes for all Panda teleoperation controllers.
- **Purpose**: Shared functionality across all controllers
- **Contents**:
  - `data_collection_node.py` - Imitation learning data collection
  - `clutch_pedal_node.py` - Safety clutch pedal interface
  - `trajectory_to_joint_states.py` - Trajectory to joint states converter
  - Config files for data collection

#### **custom_panda_description**
Panda robot URDF and robot description files.
- Panda + Robotiq gripper URDF
- SRDF for MoveIt configuration
- RViz configuration
- Kinematics configuration

### Controller-Specific Packages

#### **panda_teleop_omy_l100**
Teleoperation using OMY L100 leader arm.
- **Purpose**: Control Panda follower arm using OMY L100 leader arm
- **Contents**:
  - `omy_l100_to_twist_node.py` - Forward kinematics converter
  - `omy_l100_to_gripper_node.py` - Gripper mapping
  - Launch files and configurations
- **Dependencies**: `panda_common`, `open_manipulator_*`, `dynamixel_*`

#### **panda_teleop_joystick**
Teleoperation using game controller (joystick).
- **Purpose**: Control Panda follower arm using joystick
- **Contents**:
  - `joy_to_twist_node.py` - Joystick to twist converter
  - `joy_to_gripper_node.py` - Gripper control
  - Launch files and configurations
- **Dependencies**: `panda_common`, `custom_panda_description`

### Hardware Interface Packages

#### **OpenMANIPULATOR Packages** (for OMY L100)
- `open_manipulator_bringup` - Hardware bringup
- `open_manipulator_description` - URDF and meshes
- `open_manipulator_collision` - Self-collision detection

#### **Dynamixel Packages** (for OMY L100)
- `DynamixelSDK` - Dynamixel motor SDK
- `dynamixel_hardware_interface` - ROS2 control interface
- `dynamixel_interfaces` - Custom messages/services

#### **ROS2 Controllers** (for OMY L100)
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
# Without data collection
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py

# With data collection
ros2 launch panda_teleop_omy_l100 panda_teleop_omy_l100.launch.py enable_data_collection:=true
```

### 3. Launch with Joystick Controller

```bash
# Without data collection
ros2 launch panda_teleop_joystick joystick_teleop.launch.py

# With data collection
ros2 launch panda_teleop_joystick joystick_teleop.launch.py enable_data_collection:=true
```

---

## 📊 Data Collection

All controllers use the same data collection system from `panda_common`.

### Data Storage Structure

```
data/
├── omy_l100/          # OMY L100 controller data
│   ├── session_YYYYMMDD_HHMMSS/
│   │   ├── episode_001.jsonl
│   │   ├── episode_001_meta.json
│   │   └── ...
├── joystick/          # Joystick controller data
│   └── session_YYYYMMDD_HHMMSS/
│       └── ...
└── <future_controller>/  # Easy to add more!
```

### Data Format

Each sample contains:
- **State**: End-effector pose (x, y, z, qx, qy, qz, qw) + joint angles (7 joints)
- **Action**: Velocity command (vx, vy, vz, wx, wy, wz)

Collection rate: **15Hz** (configurable)

---

## 🎮 Controller Details

### OMY L100 Controller

**Hardware**: 7-DOF collaborative arm
**Control Method**: Position mirroring via forward kinematics
**Episode Management**:
- Automatic target generation
- Episode ends when target reached (2cm + 3s stationary) or timeout (60s)

### Joystick Controller

**Hardware**: Game controller (tested with PS4/Xbox compatible)
**Control Method**: 
- Left stick: Linear X/Y
- Right stick: Linear Z
- L1/L2 buttons: Angular X (Roll)
- Clutch pedal: Safety control

---

## 🔧 Adding a New Controller

To add a new controller:

1. **Create new package**:
   ```bash
   ros2 pkg create panda_teleop_<your_controller> --build-type ament_cmake
   ```

2. **Add dependency** to `panda_common` in `package.xml`:
   ```xml
   <depend>panda_common</depend>
   ```

3. **Implement converter node**:
   - Convert your controller input to `TwistStamped` messages
   - Publish to `/servo_node/delta_twist_cmds`

4. **Create launch file** that includes:
   - Your converter node
   - Nodes from `panda_common` (data collection, clutch, etc.)
   - Panda robot from `custom_panda_description`

5. **Done!** Your controller now has full data collection support.

---

## 🛠️ Data Analysis Tools

Located in `scripts/` directory:

### **validate_dataset.py**
Validates collected datasets for completeness and correctness.

```bash
cd ~/ros2_ws
python3 scripts/validate_dataset.py data/omy_l100/session_YYYYMMDD_HHMMSS/
```

### **visualize_episode.py**
Visualizes episode trajectories and actions.

```bash
cd ~/ros2_ws
python3 scripts/visualize_episode.py data/omy_l100/session_YYYYMMDD_HHMMSS/episode_001.jsonl
```

---

## 📚 Documentation

- Workspace README (this file) - Complete system overview
- `TOPIC_INFORMATION.md` - ROS2 topic reference

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

etri (jindong1019@gmail.com)
