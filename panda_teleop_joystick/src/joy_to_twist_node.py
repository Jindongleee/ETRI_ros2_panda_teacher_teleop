#!/usr/bin/env python3

"""
Joy to Twist Node
Converts joystick inputs to TwistStamped commands for MoveIt Servo

Safety control:
- By default, uses clutch pedal (/clutch/active topic) for safety control
- Can fall back to joystick deadman button if use_clutch:=false

Button mapping for your joystick:
- Clutch pedal (default): Must be held for movement (safety feature)
- Left Stick X (Axis 0): Linear X (좌/우)
- Left Stick Y (Axis 1): Linear Y (전/후)
- L1 (Button 6) / L2 (Button 8): Linear Z (상/하)
- Right Stick Y (Axis 3): Angular X (Roll)
- Right Stick X (Axis 2): Angular Y (Pitch)
- R1 (Button 7) / R2 (Button 9): Angular Z (Yaw)
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
        self.declare_parameter('frame_id', 'gripper_tip_link')  # End-effector 기준
        self.declare_parameter('use_clutch', True)  # Use clutch pedal instead of deadman button
        
        # Button mapping for button-based controls
        self.declare_parameter('button_linear_z_up', 6)    # L1 button
        self.declare_parameter('button_linear_z_down', 8)  # L2 button
        self.declare_parameter('button_angular_z_pos', 7)  # R1 button
        self.declare_parameter('button_angular_z_neg', 9)  # R2 button
        
        # Joystick axis mapping (which axis controls which linear/angular direction)
        self.declare_parameter('axis_linear_x', 0)   # Left stick X
        self.declare_parameter('axis_linear_y', 1)   # Left stick Y
        self.declare_parameter('axis_angular_x', 3)  # Right stick Y
        self.declare_parameter('axis_angular_y', 2)  # Right stick X
        
        # Axis inversion (set to -1 to invert, 1 to keep normal)
        self.declare_parameter('invert_linear_x', 1)
        self.declare_parameter('invert_linear_y', 1)
        self.declare_parameter('invert_angular_x', 1)
        self.declare_parameter('invert_angular_y', 1)
        
        # Deadzone for joystick axes (ignore small values to prevent drift)
        self.declare_parameter('axis_deadzone', 0.1)  # Default: 0.1 (10% of full range)
        
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.frame_id = self.get_parameter('frame_id').value
        self.axis_deadzone = self.get_parameter('axis_deadzone').value
        self.use_clutch = self.get_parameter('use_clutch').value
        
        # Button mapping
        self.button_linear_z_up = self.get_parameter('button_linear_z_up').value
        self.button_linear_z_down = self.get_parameter('button_linear_z_down').value
        self.button_angular_z_pos = self.get_parameter('button_angular_z_pos').value
        self.button_angular_z_neg = self.get_parameter('button_angular_z_neg').value
        
        # Clutch state (when use_clutch is True)
        self.clutch_active = False
        
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_angular_x = self.get_parameter('axis_angular_x').value
        self.axis_angular_y = self.get_parameter('axis_angular_y').value
        
        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_angular_x = self.get_parameter('invert_angular_x').value
        self.invert_angular_y = self.get_parameter('invert_angular_y').value
        
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
        
        # Check if control is enabled (clutch pedal)
        control_enabled = self.clutch_active if self.use_clutch else True
        
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
            
            # Linear Z: button-based control (L1/L2)
            l1_pressed = len(msg.buttons) > self.button_linear_z_up and msg.buttons[self.button_linear_z_up] == 1
            l2_pressed = len(msg.buttons) > self.button_linear_z_down and msg.buttons[self.button_linear_z_down] == 1
            
            if l1_pressed and not l2_pressed:
                twist.twist.linear.z = self.linear_scale  # L1: up
            elif l2_pressed and not l1_pressed:
                twist.twist.linear.z = -self.linear_scale  # L2: down
            else:
                twist.twist.linear.z = 0.0
            
            # Angular X: from Right Stick Y (axis 3)
            if len(msg.axes) > abs(self.axis_angular_x):
                axis_val = msg.axes[abs(self.axis_angular_x)]
                axis_val = apply_deadzone(axis_val, self.axis_deadzone)
                twist.twist.angular.x = axis_val * self.invert_angular_x * self.angular_scale
            
            # Angular Y: from Right Stick X (axis 2)
            if len(msg.axes) > abs(self.axis_angular_y):
                axis_val = msg.axes[abs(self.axis_angular_y)]
                axis_val = apply_deadzone(axis_val, self.axis_deadzone)
                twist.twist.angular.y = axis_val * self.invert_angular_y * self.angular_scale
            
            # Angular Z: button-based control (R1/R2)
            r1_pressed = len(msg.buttons) > self.button_angular_z_pos and msg.buttons[self.button_angular_z_pos] == 1
            r2_pressed = len(msg.buttons) > self.button_angular_z_neg and msg.buttons[self.button_angular_z_neg] == 1
            
            if r1_pressed and not r2_pressed:
                twist.twist.angular.z = self.angular_scale  # R1: positive
            elif r2_pressed and not r1_pressed:
                twist.twist.angular.z = -self.angular_scale  # R2: negative
            else:
                twist.twist.angular.z = 0.0
            
            # Normalize command if any component magnitude > 1.0 (to satisfy MoveIt Servo expectation)
            max_component = max(
                abs(twist.twist.linear.x),
                abs(twist.twist.linear.y),
                abs(twist.twist.linear.z),
                abs(twist.twist.angular.x),
                abs(twist.twist.angular.y),
                abs(twist.twist.angular.z),
            )
            if max_component > 1.0:
                scale = 1.0 / max_component
                twist.twist.linear.x *= scale
                twist.twist.linear.y *= scale
                twist.twist.linear.z *= scale
                twist.twist.angular.x *= scale
                twist.twist.angular.y *= scale
                twist.twist.angular.z *= scale
            
            # Log active movement (only when significant)
            if abs(twist.twist.linear.x) > 0.01 or \
               abs(twist.twist.linear.y) > 0.01 or \
               abs(twist.twist.linear.z) > 0.01 or \
               abs(twist.twist.angular.x) > 0.01 or \
               abs(twist.twist.angular.y) > 0.01 or \
               abs(twist.twist.angular.z) > 0.01:
                self.get_logger().info(
                    f'Moving - Linear: [{twist.twist.linear.x:.2f}, '
                    f'{twist.twist.linear.y:.2f}, {twist.twist.linear.z:.2f}] '
                    f'Angular: [{twist.twist.angular.x:.2f}, '
                    f'{twist.twist.angular.y:.2f}, {twist.twist.angular.z:.2f}]',
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