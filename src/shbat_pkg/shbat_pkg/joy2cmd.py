import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

#!/usr/bin/env python3


class Joy2CmdNode(Node):
    def __init__(self):
        super().__init__('joy2cmd')
        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        # Publisher for /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Adjustable max speeds
        self.max_linear_speed = 17.0   # m/s, adjust as needed
        self.max_angular_speed = 50.0 # rad/s, adjust as needed

    def joy_callback(self, msg: Joy):
        twist = Twist()

        # Axis mapping (adjust if needed)
        # axis 1: forward/backward (usually left stick vertical)
        # axis 2: left/right (usually left stick horizontal)
        linear_input = msg.axes[1] if len(msg.axes) > 1 else 0.0
        angular_input = msg.axes[2] if len(msg.axes) > 2 else 0.0

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
