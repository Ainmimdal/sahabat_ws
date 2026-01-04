#!/usr/bin/env python3
"""
Simple motor test script for ZLAC8015D differential drive robot.

Usage:
  ros2 run shbat_pkg test_motor

Controls:
  w - Forward
  s - Backward
  a - Turn left
  d - Turn right
  q - Rotate left (in place)
  e - Rotate right (in place)
  x - Stop
  + - Increase speed
  - - Decrease speed
  ESC or Ctrl+C - Exit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Start with very slow speeds
        self.linear_speed = 0.1   # m/s (very slow)
        self.angular_speed = 0.3  # rad/s (very slow turn)
        self.speed_increment = 0.05
        
        self.get_logger().info('=== Motor Test Node Started ===')
        self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
        self.print_controls()

    def print_controls(self):
        print('\n--- Controls ---')
        print('  w - Forward')
        print('  s - Backward')
        print('  a - Turn left (while moving)')
        print('  d - Turn right (while moving)')
        print('  q - Rotate left (in place)')
        print('  e - Rotate right (in place)')
        print('  x - Stop')
        print('  + - Increase speed')
        print('  - - Decrease speed')
        print('  ESC - Exit')
        print('----------------\n')

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent: linear={linear:.2f} m/s, angular={angular:.2f} rad/s')

    def stop(self):
        self.send_velocity(0.0, 0.0)

    def run(self):
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            print('Ready! Press keys to control robot...\r')
            
            while rclpy.ok():
                # Read single character
                ch = sys.stdin.read(1)
                
                if ch == '\x1b':  # ESC
                    print('\nExiting...\r')
                    self.stop()
                    break
                elif ch == '\x03':  # Ctrl+C
                    print('\nExiting...\r')
                    self.stop()
                    break
                elif ch == 'w':
                    print(f'Forward at {self.linear_speed:.2f} m/s\r')
                    self.send_velocity(self.linear_speed, 0.0)
                elif ch == 's':
                    print(f'Backward at {self.linear_speed:.2f} m/s\r')
                    self.send_velocity(-self.linear_speed, 0.0)
                elif ch == 'a':
                    print(f'Forward-Left\r')
                    self.send_velocity(self.linear_speed, self.angular_speed)
                elif ch == 'd':
                    print(f'Forward-Right\r')
                    self.send_velocity(self.linear_speed, -self.angular_speed)
                elif ch == 'q':
                    print(f'Rotate Left at {self.angular_speed:.2f} rad/s\r')
                    self.send_velocity(0.0, self.angular_speed)
                elif ch == 'e':
                    print(f'Rotate Right at {self.angular_speed:.2f} rad/s\r')
                    self.send_velocity(0.0, -self.angular_speed)
                elif ch == 'x' or ch == ' ':
                    print('Stop\r')
                    self.stop()
                elif ch == '+' or ch == '=':
                    self.linear_speed += self.speed_increment
                    self.angular_speed += self.speed_increment
                    print(f'Speed increased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}\r')
                elif ch == '-':
                    self.linear_speed = max(0.05, self.linear_speed - self.speed_increment)
                    self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
                    print(f'Speed decreased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}\r')
                    
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MotorTest()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
