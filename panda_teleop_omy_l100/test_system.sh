#!/bin/bash
# Test System Script for Panda Teleop with OMY L100
# This script helps diagnose and test the teleoperation system

echo "=========================================="
echo "Panda Teleop OMY L100 System Test"
echo "=========================================="
echo ""

# Source ROS2 workspace
echo "[1/6] Sourcing ROS2 workspace..."
source /home/etri/ros2_ws/install/setup.bash
echo "✓ Workspace sourced"
echo ""

# Check running nodes
echo "[2/6] Checking running nodes..."
echo "Running nodes:"
ros2 node list
echo ""

# Check if servo_node is running
if ros2 node list | grep -q "servo_node"; then
    echo "✓ servo_node is running"
else
    echo "✗ servo_node is NOT running (this is the main issue!)"
fi
echo ""

# Check topics
echo "[3/6] Checking key topics..."
echo "Available topics:"
ros2 topic list | grep -E "(servo|clutch|joint_states|twist)"
echo ""

# Check servo_node topics
if ros2 topic list | grep -q "/servo_node/delta_twist_cmds"; then
    echo "✓ /servo_node/delta_twist_cmds exists"
else
    echo "✗ /servo_node/delta_twist_cmds does NOT exist"
fi

if ros2 topic list | grep -q "/clutch/active"; then
    echo "✓ /clutch/active exists"
else
    echo "✗ /clutch/active does NOT exist"
fi
echo ""

# Check servo_node services
echo "[4/6] Checking servo_node services..."
if ros2 service list | grep -q "/servo_node/start_servo"; then
    echo "✓ /servo_node/start_servo service exists"
    echo "  You can manually start servo with:"
    echo "  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger"
else
    echo "✗ /servo_node/start_servo service does NOT exist"
    echo "  servo_node may not be running"
fi
echo ""

# Check TF frames
echo "[5/6] Checking TF frames..."
echo "Key frames (should include: leader_link0, leader_link7, panda_link0, panda_link8):"
ros2 run tf2_ros tf2_echo leader_link0 leader_link7 --timeout 1.0 2>&1 | head -n 3
echo ""

# Monitor twist commands (sample)
echo "[6/6] Monitoring /servo_node/delta_twist_cmds (5 seconds)..."
echo "This will show if omy_l100_to_twist_node is publishing commands:"
timeout 5 ros2 topic echo /servo_node/delta_twist_cmds --once 2>&1 | head -n 10 || echo "No messages received (check if leader arm is moving)"
echo ""

echo "=========================================="
echo "Test Complete"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. If servo_node is NOT running, check launch logs for errors"
echo "2. Make sure xterm is installed for clutch control: sudo apt install xterm"
echo "3. Move the leader arm to test twist commands"
echo "4. Press 'b' in the xterm window to toggle clutch"
echo ""
echo "For detailed usage, see: CLUTCH_USAGE.md"
