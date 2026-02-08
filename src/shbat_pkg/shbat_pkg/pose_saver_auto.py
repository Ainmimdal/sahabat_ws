#!/usr/bin/env python3
"""
Auto Pose Saver - Automatically saves and restores robot pose for AMCL.

On startup:
1. Reads saved pose from ~/.ros/sahabat_saved_pose.yaml
2. Publishes it to /initialpose to set AMCL's position

While running:
1. Listens for /initialpose (2D Pose Estimate from RViz)
2. Saves it to ~/.ros/sahabat_saved_pose.yaml
3. Next launch reads this file and publishes to AMCL
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import os
import math
import time


class AutoPoseSaver(Node):
    def __init__(self):
        super().__init__('auto_pose_saver')
        
        self.pose_file = os.path.expanduser('~/.ros/sahabat_saved_pose.yaml')
        
        # Publisher to set AMCL initial pose
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Listen for 2D Pose Estimate from RViz (to save it)
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        self.get_logger().info(f'Auto Pose Saver running.')
        
        # Publish saved pose after a delay (wait for AMCL to be ready)
        self.create_timer(3.0, self.publish_saved_pose_once)
        self.pose_published = False
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def publish_saved_pose_once(self):
        """Publish the saved pose to AMCL once on startup."""
        if self.pose_published:
            return
            
        self.pose_published = True
        
        # Check if saved pose exists
        if not os.path.exists(self.pose_file):
            self.get_logger().info('No saved pose found. Use 2D Pose Estimate in RViz.')
            return
        
        try:
            with open(self.pose_file, 'r') as f:
                saved_pose = yaml.safe_load(f)
            
            x = saved_pose.get('initial_pose_x', 0.0)
            y = saved_pose.get('initial_pose_y', 0.0)
            yaw = saved_pose.get('initial_pose_yaw', 0.0)
            
            # Create and publish initial pose message
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
            
            # Small covariance for precise positioning
            msg.pose.covariance[0] = 0.1  # x variance
            msg.pose.covariance[7] = 0.1  # y variance
            msg.pose.covariance[35] = 0.05  # yaw variance
            
            # Publish multiple times to ensure AMCL receives it
            for _ in range(3):
                self.initialpose_pub.publish(msg)
                time.sleep(0.1)
            
            self.get_logger().info(f'Published saved pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load saved pose: {e}')
    
    def initialpose_callback(self, msg):
        """Called when user sets 2D Pose Estimate in RViz - save it."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        pose_data = {
            'initial_pose_x': round(x, 4),
            'initial_pose_y': round(y, 4),
            'initial_pose_yaw': round(yaw, 4),
        }
        
        # Save to file
        os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)
        with open(self.pose_file, 'w') as f:
            yaml.dump(pose_data, f, default_flow_style=False)
        
        self.get_logger().info(f'Pose saved! x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = AutoPoseSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
