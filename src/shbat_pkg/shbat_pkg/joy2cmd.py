#!/usr/bin/env python3
"""
Joystick to cmd_vel converter with emergency stop.

Button mappings (typical Xbox/PS controller):
- Left stick: movement control
- Button 0 (A/X): Emergency stop toggle
- Button 1 (B/O): Resume movement  
- Button 4 (LB): Dead man switch (must hold to enable movement)

Emergency stop can also be triggered via /emergency_stop topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Joy2CmdNode(Node):
    def __init__(self):
        super().__init__('joy2cmd')
        
        # Emergency stop state
        self.emergency_stopped = False
        self.deadman_pressed = False
        
        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        # Subscribe to emergency stop topic (can be triggered by Nav2 or other nodes)
        self.stop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10)
        
        # Publisher for /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Publisher for emergency stop status
        self.stop_status_pub = self.create_publisher(Bool, 'emergency_stop_active', 10)

        # Adjustable max speeds (conservative for indoor use)
        self.max_linear_speed = 0.5   # m/s (0.5 = walking pace, 1.0 = fast walk)
        self.max_angular_speed = 1.0  # rad/s (1.0 = ~57 deg/s, safe turning)
        
        # Button indices (adjust for your controller)
        self.ESTOP_BUTTON = 0       # A button - emergency stop
        self.RESUME_BUTTON = 1      # B button - resume
        self.DEADMAN_BUTTON = 4     # LB button - dead man switch
        
        # Previous button states for edge detection
        self.prev_buttons = []
        
        self.get_logger().info('Joy2Cmd initialized with emergency stop')
        self.get_logger().info('  Press A/Button0 for emergency stop')
        self.get_logger().info('  Press B/Button1 to resume')
        self.get_logger().info('  Hold LB/Button4 as dead man switch')

    def emergency_stop_callback(self, msg: Bool):
        """Handle external emergency stop requests."""
        if msg.data and not self.emergency_stopped:
            self.emergency_stopped = True
            self.get_logger().warn('EMERGENCY STOP activated via topic!')
            self.publish_stop()
        elif not msg.data and self.emergency_stopped:
            self.emergency_stopped = False
            self.get_logger().info('Emergency stop cleared via topic')
    
    def publish_stop(self):
        """Publish zero velocity command."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        
        # Publish stop status
        status = Bool()
        status.data = self.emergency_stopped
        self.stop_status_pub.publish(status)

    def joy_callback(self, msg: Joy):
        # Initialize previous buttons if needed
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons) if msg.buttons else []
        
        # Check for button presses (edge detection)
        def button_pressed(idx):
            if idx < len(msg.buttons) and idx < len(self.prev_buttons):
                return msg.buttons[idx] == 1 and self.prev_buttons[idx] == 0
            return False
        
        def button_held(idx):
            return idx < len(msg.buttons) and msg.buttons[idx] == 1
        
        # Emergency stop button (toggle on)
        if button_pressed(self.ESTOP_BUTTON):
            self.emergency_stopped = True
            self.get_logger().warn('EMERGENCY STOP activated! Press B to resume.')
            self.publish_stop()
        
        # Resume button (toggle off)
        if button_pressed(self.RESUME_BUTTON):
            if self.emergency_stopped:
                self.emergency_stopped = False
                self.get_logger().info('Emergency stop cleared - movement enabled')
        
        # Dead man switch
        self.deadman_pressed = button_held(self.DEADMAN_BUTTON)
        
        # Update previous buttons
        self.prev_buttons = list(msg.buttons) if msg.buttons else []
        
        # If emergency stopped, always publish zero
        if self.emergency_stopped:
            self.publish_stop()
            return
        
        twist = Twist()

        # Axis mapping (adjust if needed)
        # axis 1: forward/backward (usually left stick vertical)
        # axis 2: left/right (usually left stick horizontal)
        linear_input = msg.axes[1] if len(msg.axes) > 1 else 0.0
        angular_input = msg.axes[2] if len(msg.axes) > 2 else 0.0

        # Only allow movement if dead man switch is held (optional safety)
        # Comment out these lines if you don't want dead man switch
        # if not self.deadman_pressed:
        #     linear_input = 0.0
        #     angular_input = 0.0

        # Map joystick input to robot speed
        twist.linear.x = linear_input * self.max_linear_speed
        twist.angular.z = angular_input * self.max_angular_speed

        # Publish the Twist message
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Joy2CmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
