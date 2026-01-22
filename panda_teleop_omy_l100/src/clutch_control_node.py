#!/usr/bin/env python3

"""
Clutch Control Node
-------------------
Keyboard-based clutch control for teleoperation.

This node:
- Monitors keyboard input for 'b' key
- Clutch is active ONLY while 'b' key is pressed (hold-to-activate)
- Publishes clutch state to /clutch/active topic
- When clutch is pressed (active), teleoperation is enabled
- When clutch is released (inactive), teleoperation is paused

Usage:
    Hold 'b' key to enable teleoperation
    Release 'b' key to pause teleoperation
    This is a safety feature - robot only moves when operator actively holds the clutch
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ClutchControlNode(Node):
    def __init__(self):
        super().__init__('clutch_control_node')
        
        # Clutch state (False = paused, True = active)
        self.clutch_active = False
        self.last_key_state = False
        
        # Time tracking for hold detection
        self.last_b_key_time = None
        self.key_timeout = 0.1  # 100ms timeout - if no 'b' key within this time, deactivate
        
        # Publisher
        self.clutch_pub = self.create_publisher(
            Bool,
            '/clutch/active',
            10
        )
        
        # Timer for keyboard polling (50 Hz for responsive control)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Terminal settings for raw input
        self.old_settings = None
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception as e:
            self.get_logger().warn(f'Could not set terminal to raw mode: {e}')
        
        # Publish initial state (paused)
        self.publish_clutch_state()
        
        self.get_logger().info('Clutch Control Node started')
        self.get_logger().info('===================================')
        self.get_logger().info('HOLD "b" key to ENABLE teleoperation')
        self.get_logger().info('RELEASE "b" key to PAUSE teleoperation')
        self.get_logger().info('===================================')
        self.get_logger().info('Current state: PAUSED (waiting for clutch)')
        self.get_logger().info('Press "q" to quit')
        self.get_logger().info(f'Key timeout: {self.key_timeout*1000:.0f}ms (for hold detection)')
    
    def timer_callback(self):
        """Check for keyboard input - hold 'b' to activate
        
        Uses time-based hold detection:
        - When 'b' key is detected, update timestamp
        - Clutch stays active while 'b' keys keep arriving within timeout
        - If no 'b' key for timeout duration, clutch deactivates
        """
        try:
            current_time = self.get_clock().now()
            
            # Check if input is available (non-blocking)
            # Read ALL available characters to drain buffer
            b_key_detected = False
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                
                # Check for 'b' key
                if key.lower() == 'b':
                    b_key_detected = True
                    self.last_b_key_time = current_time
                
                # Check for 'q' key to quit
                elif key.lower() == 'q':
                    self.get_logger().info('Quit key pressed - shutting down')
                    rclpy.shutdown()
                    return
            
            # Determine clutch state based on recent 'b' key activity
            prev_state = self.clutch_active
            
            if self.last_b_key_time is not None:
                # Check if 'b' key was pressed recently (within timeout)
                time_since_last_b = (current_time - self.last_b_key_time).nanoseconds * 1e-9
                self.clutch_active = (time_since_last_b < self.key_timeout)
            else:
                # No 'b' key pressed yet
                self.clutch_active = False
            
            # Only log and publish when state changes
            if prev_state != self.clutch_active:
                self.publish_clutch_state()
                
                if self.clutch_active:
                    self.get_logger().info('>>> Clutch PRESSED - Teleoperation ACTIVE')
                else:
                    self.get_logger().info('>>> Clutch RELEASED - Teleoperation PAUSED')
        
        except Exception as e:
            # Silently ignore read errors (happens when terminal is not interactive)
            pass
    
    def publish_clutch_state(self):
        """Publish current clutch state"""
        msg = Bool()
        msg.data = self.clutch_active
        self.clutch_pub.publish(msg)
    
    def cleanup(self):
        """Restore terminal settings"""
        if self.old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ClutchControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cleanup()
            node.get_logger().info('Shutting down clutch_control_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
