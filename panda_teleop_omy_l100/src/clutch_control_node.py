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
    
    def timer_callback(self):
        """Check for keyboard input - hold 'b' to activate"""
        try:
            # Check if 'b' key is currently pressed
            key_pressed = False
            
            # Check if input is available (non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                
                # Check for 'b' key being held
                if key.lower() == 'b':
                    key_pressed = True
                
                # Check for 'q' key to quit
                elif key.lower() == 'q':
                    self.get_logger().info('Quit key pressed - shutting down')
                    rclpy.shutdown()
                    return
            
            # Update clutch state based on key press
            prev_state = self.clutch_active
            self.clutch_active = key_pressed
            
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
