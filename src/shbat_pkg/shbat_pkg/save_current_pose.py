#!/usr/bin/env python3
"""
Save current robot pose as default initial pose for localization.
Run this when the robot is at its "home" position.

Usage:
    ros2 run shbat_pkg save_current_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import yaml
import os
import math

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        
        self.current_pose = None
        
        # Try to get pose from AMCL first (most accurate when localized)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            1
        )
        
        # Fallback to odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1
        )
        
        # Wait a bit then save
        self.create_timer(2.0, self.save_pose)
        
        self.get_logger().info('Waiting for pose data...')
    
    def amcl_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }
        self.pose_source = 'AMCL'
    
    def odom_callback(self, msg):
        # Only use odom if we don't have AMCL pose
        if self.current_pose is None:
            self.current_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
            }
            self.pose_source = 'Odometry'
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def save_pose(self):
        if self.current_pose is None:
            self.get_logger().error('No pose data received! Make sure robot is running.')
            rclpy.shutdown()
            return
        
        # Save to a simple file that can be sourced
        home_pose_file = os.path.expanduser('~/.ros/sahabat_home_pose.yaml')
        
        pose_data = {
            'initial_pose_x': round(self.current_pose['x'], 3),
            'initial_pose_y': round(self.current_pose['y'], 3),
            'initial_pose_yaw': round(self.current_pose['yaw'], 3),
        }
        
        with open(home_pose_file, 'w') as f:
            yaml.dump(pose_data, f, default_flow_style=False)
        
        self.get_logger().info(f'\n' + '='*50)
        self.get_logger().info(f'Pose saved from {self.pose_source}!')
        self.get_logger().info(f'  X:   {pose_data["initial_pose_x"]:.3f} m')
        self.get_logger().info(f'  Y:   {pose_data["initial_pose_y"]:.3f} m')
        self.get_logger().info(f'  Yaw: {pose_data["initial_pose_yaw"]:.3f} rad ({math.degrees(pose_data["initial_pose_yaw"]):.1f}°)')
        self.get_logger().info(f'='*50)
        self.get_logger().info(f'Saved to: {home_pose_file}')
        self.get_logger().info(f'\nTo use this pose for localization:')
        self.get_logger().info(f'  ros2 launch shbat_pkg slam_nav_launch.py mode:=localization \\')
        self.get_logger().info(f'      map_file:=/path/to/map \\')
        self.get_logger().info(f'      initial_pose_x:={pose_data["initial_pose_x"]} \\')
        self.get_logger().info(f'      initial_pose_y:={pose_data["initial_pose_y"]} \\')
        self.get_logger().info(f'      initial_pose_yaw:={pose_data["initial_pose_yaw"]}')
        
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PoseSaver()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
