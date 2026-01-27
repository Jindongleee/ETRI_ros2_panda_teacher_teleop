#!/usr/bin/env python3

"""
Clutch Pedal Node (evdev-based)
--------------------------------
Reads PCsensor FootSwitch input device directly and publishes clutch state.

This node:
- Monitors Linux input device (e.g., /dev/input/event18) for 'b' key events
- Clutch is active ONLY while 'b' key is physically pressed (true hold-to-activate)
- Publishes clutch state to /clutch/active topic
- No terminal dependency - reads hardware directly via evdev

Advantages over stdin-based approach:
- Direct hardware access (no terminal/stdin buffering issues)
- True key press/release detection (not time-based approximation)
- More reliable and responsive
- Works even when terminal is not focused

Requirements:
- Python evdev library: pip install evdev
- User must be in 'input' group: sudo usermod -a -G input $USER
  (or run with sudo, but not recommended)

Usage:
    ros2 run panda_teleop_omy_l100 clutch_pedal_node.py
    
Optional parameters:
    device_path:=/dev/input/event18  (auto-detects PCsensor by default)
    key_code:=48                      (KEY_B = 48, can be changed)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys

try:
    import evdev
    from evdev import InputDevice, ecodes
except ImportError:
    print("ERROR: evdev library not found!")
    print("Please install: pip install evdev")
    print("Or: sudo apt install python3-evdev")
    sys.exit(1)


class ClutchPedalNode(Node):
    def __init__(self):
        super().__init__('clutch_pedal_node')
        
        # Parameters
        self.declare_parameter('device_path', '')  # Empty = auto-detect
        self.declare_parameter('device_name', 'PCsensor FootSwitch')  # Device name to search for
        self.declare_parameter('key_code', ecodes.KEY_B)  # 48 = KEY_B
        
        device_path = self.get_parameter('device_path').value
        device_name = self.get_parameter('device_name').value
        self.key_code = self.get_parameter('key_code').value
        
        # Clutch state
        self.clutch_active = False
        
        # Publisher
        self.clutch_pub = self.create_publisher(
            Bool,
            '/clutch/active',
            10
        )
        
        # Find and open input device
        self.device = None
        if device_path:
            # Use specified device path
            try:
                self.device = InputDevice(device_path)
                self.get_logger().info(f'Opened device: {device_path}')
                self.get_logger().info(f'Device name: {self.device.name}')
            except Exception as e:
                self.get_logger().error(f'Failed to open device {device_path}: {e}')
                sys.exit(1)
        else:
            # Auto-detect PCsensor FootSwitch
            self.device = self.find_device_by_name(device_name)
            if self.device is None:
                self.get_logger().error(f'Could not find device: {device_name}')
                self.get_logger().error('Available devices:')
                for dev in [InputDevice(path) for path in evdev.list_devices()]:
                    self.get_logger().error(f'  {dev.path}: {dev.name}')
                self.get_logger().error('')
                self.get_logger().error('Solutions:')
                self.get_logger().error('1. Check if PCsensor FootSwitch is connected')
                self.get_logger().error('2. Specify device path manually: device_path:=/dev/input/eventXX')
                self.get_logger().error('3. Check permissions: sudo usermod -a -G input $USER (then logout/login)')
                sys.exit(1)
        
        # Check if we have permission to read the device
        try:
            self.device.capabilities()
        except Exception as e:
            self.get_logger().error(f'Permission denied to read device: {e}')
            self.get_logger().error('Solutions:')
            self.get_logger().error('1. Add user to input group: sudo usermod -a -G input $USER')
            self.get_logger().error('2. Logout and login again')
            self.get_logger().error('3. Or run with sudo (not recommended)')
            sys.exit(1)
        
        # Grab device (REQUIRED - prevents other programs like X11 from reading it)
        # Without grabbing, input events go to X11/keyboard handler instead of our node
        try:
            self.device.grab()
            self.get_logger().info('Device grabbed (exclusive access) - input events will be captured')
        except Exception as e:
            self.get_logger().error(f'Could not grab device: {e}')
            self.get_logger().error('Input events may be captured by other programs (X11, etc.)')
            self.get_logger().error('Try running with sudo or check permissions')
        
        # Publish initial state (paused)
        self.publish_clutch_state()
        
        # Timer for reading events (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # Periodic publisher to ensure topic is always active (10 Hz)
        self.publish_timer = self.create_timer(0.1, self.periodic_publish_callback)
        
        self.get_logger().info('===================================')
        self.get_logger().info('Clutch Pedal Node started')
        self.get_logger().info(f'Device: {self.device.path}')
        self.get_logger().info(f'Device name: {self.device.name}')
        self.get_logger().info(f'Key code: {self.key_code}')
        self.get_logger().info('===================================')
        self.get_logger().info('HOLD pedal to ENABLE teleoperation')
        self.get_logger().info('RELEASE pedal to PAUSE teleoperation')
        self.get_logger().info('===================================')
        self.get_logger().info('Current state: PAUSED (waiting for pedal)')
    
    def find_device_by_name(self, name):
        """Find input device by name (case-insensitive substring match)"""
        devices = [InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if name.lower() in device.name.lower():
                self.get_logger().info(f'Found device: {device.path} ({device.name})')
                return device
        return None
    
    def timer_callback(self):
        """Read input events from device (non-blocking)"""
        try:
            # Read all available events using read_one() which is non-blocking
            # Keep reading until no more events are available
            while True:
                try:
                    event = self.device.read_one()
                    if event is None:
                        # No more events available
                        break
                    
                    # Filter key events only
                    if event.type == ecodes.EV_KEY:
                        # Check if it's our target key
                        if event.code == self.key_code:
                            prev_state = self.clutch_active
                            
                            # Key states:
                            # 0 = key up (released)
                            # 1 = key down (pressed)
                            # 2 = key hold (auto-repeat)
                            if event.value == 1:  # Key pressed
                                self.clutch_active = True
                            elif event.value == 0:  # Key released
                                self.clutch_active = False
                            # Ignore value == 2 (auto-repeat), keep current state
                            
                            # Log and publish on state change
                            if prev_state != self.clutch_active:
                                self.publish_clutch_state()
                                
                                if self.clutch_active:
                                    self.get_logger().info('>>> Pedal PRESSED - Teleoperation ACTIVE')
                                else:
                                    self.get_logger().info('>>> Pedal RELEASED - Teleoperation PAUSED')
                
                except BlockingIOError:
                    # No events available - this is normal, exit loop
                    break
        
        except OSError as e:
            # Device might have been disconnected
            self.get_logger().error(f'Device error: {e}')
            # Try to republish current state
            self.publish_clutch_state()
        except Exception as e:
            self.get_logger().error(f'Error reading device: {e}')
    
    def publish_clutch_state(self):
        """Publish current clutch state"""
        msg = Bool()
        msg.data = self.clutch_active
        self.clutch_pub.publish(msg)
    
    def periodic_publish_callback(self):
        """Periodically publish state to ensure topic is active"""
        self.publish_clutch_state()
    
    def cleanup(self):
        """Release device"""
        if self.device is not None:
            try:
                self.device.ungrab()
            except Exception:
                pass
            try:
                self.device.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ClutchPedalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cleanup()
            node.get_logger().info('Shutting down clutch_pedal_node')
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
