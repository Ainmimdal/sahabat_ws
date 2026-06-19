#!/usr/bin/env python3
"""
Joystick to cmd_vel converter with emergency stop.

Button mappings (typical Xbox/PS controller):
- Left stick: movement control
- Button 0 (A/X): Emergency stop toggle
- Button 1 (B/O): Clear E-stop in standalone/local mode

Emergency stop can also be triggered via /emergency_stop topic.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class Joy2CmdNode(Node):
    def __init__(self):
        super().__init__('joy2cmd')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('allow_estop_clear', True)

        # Emergency stop state
        self.emergency_stopped = False
        self.localization_recovery_active = False

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
        self.active_pub = self.create_publisher(
            Bool, '/operator/local_joystick_active', 10
        )
        recovery_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.recovery_sub = self.create_subscription(
            Bool,
            '/localization/recovery_active',
            self.recovery_active_callback,
            recovery_qos,
        )

        # Command the base controller's latched emergency-stop input. Publishing
        # only a zero Twist is not sufficient because Nav2 may publish another
        # velocity command immediately afterwards.
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)

        # Adjustable max speeds (conservative for indoor use)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.allow_estop_clear = bool(
            self.get_parameter('allow_estop_clear').value
        )
        # Button indices (adjust for your controller)
        self.ESTOP_BUTTON = 0       # A button - emergency stop
        self.RESUME_BUTTON = 1      # B button - resume

        # Previous button states for edge detection
        self.prev_buttons = []

        self.get_logger().info('Joy2Cmd initialized with emergency stop')
        self.get_logger().info('  Press A/Button0 for emergency stop')
        if self.allow_estop_clear:
            self.get_logger().info('  Press B/Button1 to clear emergency stop')
        else:
            self.get_logger().info('  Clear E-stop from the operator UI')
        self.get_logger().info('  Left stick controls movement (no deadman button)')

    def recovery_active_callback(self, msg: Bool):
        """Yield cmd_vel while automatic localization recovery is rotating."""
        self.localization_recovery_active = msg.data
        self.active_pub.publish(Bool(data=False))

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

        # Emergency stop button (toggle on)
        if button_pressed(self.ESTOP_BUTTON):
            self.emergency_stopped = True
            self.get_logger().warn('EMERGENCY STOP activated! Press B to resume.')
            self.estop_pub.publish(Bool(data=True))
            self.publish_stop()

        # Do not clear the shared stop from a raw joystick button. Clearing is
        # deliberately centralized in the operator backend and requires the
        # active control lease plus an explicit confirmation.
        if button_pressed(self.RESUME_BUTTON):
            if self.allow_estop_clear and self.emergency_stopped:
                self.emergency_stopped = False
                self.estop_pub.publish(Bool(data=False))
                self.get_logger().info('Emergency stop cleared')
            else:
                self.get_logger().warn('Clear E-stop from the operator UI')

        # Update previous buttons
        self.prev_buttons = list(msg.buttons) if msg.buttons else []

        # If emergency stopped, always publish zero
        if self.emergency_stopped:
            self.active_pub.publish(Bool(data=False))
            self.publish_stop()
            return

        # The recovery node owns cmd_vel while it performs its controlled spin.
        # Do not publish even a zero Twist here, because that would compete with
        # the recovery command at the base controller.
        if self.localization_recovery_active:
            self.active_pub.publish(Bool(data=False))
            return

        twist = Twist()

        # Axis mapping for Xbox 360 controller:
        # axis 0: left stick X (steering/rotation)
        # axis 1: left stick Y (forward/backward)
        # axis 2: LT trigger (1.0 released, -1.0 pressed) — NOT used
        # axis 3: right stick X
        # axis 4: right stick Y
        # axis 5: RT trigger (1.0 released, -1.0 pressed) — NOT used

        linear_input = 0.0
        angular_input = 0.0

        if len(msg.axes) > 1:
            linear_input = msg.axes[1]  # Left stick Y

        if len(msg.axes) > 0:
            angular_input = msg.axes[0]  # Xbox left/right

        # Deadzone: ignore small inputs to prevent drift
        deadzone = 0.08
        if abs(linear_input) < deadzone:
            linear_input = 0.0
        if abs(angular_input) < deadzone:
            angular_input = 0.0

        # Map joystick input to robot speed
        twist.linear.x = linear_input * self.max_linear_speed
        twist.angular.z = angular_input * self.max_angular_speed

        self.active_pub.publish(Bool(data=(
            linear_input != 0.0 or angular_input != 0.0
        )))

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
