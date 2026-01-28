#!/usr/bin/env python3

"""
Joy to Twist Node
Converts joystick inputs to TwistStamped commands for MoveIt Servo

Safety control:
- By default, uses clutch pedal (/clutch/active topic) for safety control
- Can fall back to joystick deadman button (L1) if use_clutch:=false

Button mapping for your joystick:
- Clutch pedal (default): Must be held for movement (safety feature)
- Left Stick X (Axis 0): Linear X (좌: 0.5, 우: -0.5)
- Left Stick Y (Axis 1): Linear Y (상: 0.5, 하: -0.5)
- Right Stick Y (Axis 3): Linear Z (상: 0.5, 하: -0.5)
- L1 (Button 4): Angular X = +0.8 (Roll rotation when clutch is active)
- L2 (Button 8): Angular X = -0.8 (Roll rotation when clutch is active)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, Bool


class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist_node')
        
        # Parameters (these are dimensionless scales applied to joystick axes)
        # We keep the result roughly in [-1, 1] so MoveIt Servo doesn't skip commands.
        self.declare_parameter('linear_scale', 3.0)
        self.declare_parameter('angular_scale', 3.5)
        self.declare_parameter('deadman_button', 6)  # L1 button (deprecated, use clutch instead)
        self.declare_parameter('l2_button', 8)  # L2 button
        self.declare_parameter('frame_id', 'gripper_tip_link')  # End-effector 기준
        self.declare_parameter('use_clutch', True)  # Use clutch pedal instead of deadman button
        
        # Joystick axis mapping (which axis controls which linear/angular direction)
        self.declare_parameter('axis_linear_x', 0)   # Left stick X
        self.declare_parameter('axis_linear_y', 1)   # Left stick Y
        self.declare_parameter('axis_linear_z', 3)   # Right stick Y
        self.declare_parameter('axis_angular_x', -1)  # -1 = button mode (L1/L2)
        
        # Axis inversion (set to -1 to invert, 1 to keep normal)
        self.declare_parameter('invert_linear_x', 1)
        self.declare_parameter('invert_linear_y', 1)
        self.declare_parameter('invert_linear_z', 1)
        self.declare_parameter('invert_angular_x', 1)
        
        # Deadzone for joystick axes (ignore small values to prevent drift)
        self.declare_parameter('axis_deadzone', 0.1)  # Default: 0.1 (10% of full range)
        
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.l2_button = self.get_parameter('l2_button').value
        self.frame_id = self.get_parameter('frame_id').value
        self.axis_deadzone = self.get_parameter('axis_deadzone').value
        self.use_clutch = self.get_parameter('use_clutch').value
        
        # Clutch state (when use_clutch is True)
        self.clutch_active = False
        
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_linear_z = self.get_parameter('axis_linear_z').value
        self.axis_angular_x = self.get_parameter('axis_angular_x').value
        
        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_linear_z = self.get_parameter('invert_linear_z').value
        self.invert_angular_x = self.get_parameter('invert_angular_x').value
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Clutch subscriber (if using clutch instead of deadman button)
        if self.use_clutch:
            self.clutch_sub = self.create_subscription(
                Bool,
                '/clutch/active',
                self.clutch_callback,
                10
            )
            self.get_logger().info('Using clutch pedal for safety control')
        else:
            self.get_logger().info('Using joystick deadman button for safety control')
        
        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Status publisher for debugging
        self.status_pub = self.create_publisher(
            Float64,
            '/joy_to_twist/status',
            10
        )
        
        self.last_joy_time = self.get_clock().now()
        self.joy_received = False
    
    def clutch_callback(self, msg: Bool):
        """Handle clutch state changes"""
        self.clutch_active = msg.data
        
    def joy_callback(self, msg: Joy):
        """Convert joystick input to twist command"""
        
        if not self.joy_received:
            self.get_logger().info('Joystick input received!')
            self.joy_received = True
        
        # Create twist message (always initialize to zero)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.frame_id
        # Initialize all values to zero for safety
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        
        # Check if control is enabled (clutch or deadman button)
        if self.use_clutch:
            # Use clutch pedal state
            control_enabled = self.clutch_active
        else:
            # Use deadman button (legacy mode)
            control_enabled = len(msg.buttons) > self.deadman_button and msg.buttons[self.deadman_button] == 1
        
        l2_pressed = len(msg.buttons) > self.l2_button and msg.buttons[self.l2_button] == 1
        
        if control_enabled:
            # Helper function to apply deadzone
            def apply_deadzone(value, deadzone):
                """Apply deadzone: if |value| < deadzone, return 0.0"""
                if abs(value) < deadzone:
                    return 0.0
                return value
            
            # Linear X: from configured axis with inversion
            if len(msg.axes) > abs(self.axis_linear_x):
                axis_val = msg.axes[abs(self.axis_linear_x)]
                axis_val = apply_deadzone(axis_val, self.axis_deadzone)
                twist.twist.linear.x = axis_val * self.invert_linear_x * self.linear_scale
            
            # Linear Y: from configured axis with inversion
            if len(msg.axes) > abs(self.axis_linear_y):
                axis_val = msg.axes[abs(self.axis_linear_y)]
                axis_val = apply_deadzone(axis_val, self.axis_deadzone)
                twist.twist.linear.y = axis_val * self.invert_linear_y * self.linear_scale
            
            # Linear Z: from configured axis with inversion
            if len(msg.axes) > abs(self.axis_linear_z):
                axis_val = msg.axes[abs(self.axis_linear_z)]
                axis_val = apply_deadzone(axis_val, self.axis_deadzone)
                twist.twist.linear.z = axis_val * self.invert_linear_z * self.linear_scale
            
            # Angular X: from configured axis with inversion, or use button mode if axis_angular_x < 0
            # L1 button (deadman_button) for control, L2 for direction change
            l1_pressed = len(msg.buttons) > self.deadman_button and msg.buttons[self.deadman_button] == 1
            
            if self.axis_angular_x < 0:
                # Button mode: L1 for +angular_scale, L2 for -angular_scale
                if l1_pressed and not l2_pressed:
                    twist.twist.angular.x = self.angular_scale
                elif l2_pressed and not l1_pressed:
                    twist.twist.angular.x = -self.angular_scale
                else:
                    twist.twist.angular.x = 0.0
            else:
                # Axis mode: use joystick axis
                if len(msg.axes) > abs(self.axis_angular_x):
                    axis_val = msg.axes[abs(self.axis_angular_x)]
                    axis_val = apply_deadzone(axis_val, self.axis_deadzone)
                    twist.twist.angular.x = axis_val * self.invert_angular_x * self.angular_scale
            
            # Normalize command if any component magnitude > 1.0 (to satisfy MoveIt Servo expectation)
            max_component = max(
                abs(twist.twist.linear.x),
                abs(twist.twist.linear.y),
                abs(twist.twist.linear.z),
                abs(twist.twist.angular.x),
            )
            if max_component > 1.0:
                scale = 1.0 / max_component
                twist.twist.linear.x *= scale
                twist.twist.linear.y *= scale
                twist.twist.linear.z *= scale
                twist.twist.angular.x *= scale
            
            # Log active movement (only when significant)
            if abs(twist.twist.linear.x) > 0.01 or \
               abs(twist.twist.linear.y) > 0.01 or \
               abs(twist.twist.linear.z) > 0.01 or \
               abs(twist.twist.angular.x) > 0.01:
                self.get_logger().info(
                    f'Moving - Linear: [{twist.twist.linear.x:.2f}, '
                    f'{twist.twist.linear.y:.2f}, {twist.twist.linear.z:.2f}] '
                    f'Angular X: [{twist.twist.angular.x:.2f}]',
                    throttle_duration_sec=1.0  # Log once per second
                )
        else:
            # Deadman button not pressed - send zero velocity
            pass
        
        # Always publish (even zeros for safety)
        self.twist_pub.publish(twist)
        
        # Publish status (1.0 if control enabled, 0.0 otherwise)
        status = Float64()
        if control_enabled:
            status.data = 1.0
        else:
            status.data = 0.0
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Shutting down joy_to_twist_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()