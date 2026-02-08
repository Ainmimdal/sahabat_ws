#!/usr/bin/env python3
"""
Laser Scan Filter Node

Filters out specific angle ranges from LIDAR scan to remove
static obstacles like robot frame beams.

Subscribes: /scan_raw (original LIDAR data)
Publishes:  /scan (filtered LIDAR data)

Configure the filter zones in the parameters below.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        
        # Declare parameters for filter zones
        # Filter zones as flat list: [start1, end1, start2, end2, ...]
        # Angles are measured counter-clockwise from front (0°)
        # Front = 0°, Left = 90°, Back = 180°/-180°, Right = -90°/270°
        
        # Default: filter out left and right beams (adjust these!)
        self.declare_parameter('filter_zones', [85.0, 95.0, -95.0, -85.0])
        
        # Minimum range filter (ignore very close readings)
        self.declare_parameter('min_range', 0.10)  # 10cm
        
        # Maximum range filter
        self.declare_parameter('max_range', 12.0)  # 12m
        
        # Input/output topics
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        
        # Get parameters
        filter_zones_flat = self.get_parameter('filter_zones').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Parse flat list into pairs: [s1, e1, s2, e2] -> [(s1,e1), (s2,e2)]
        self.filter_zones_rad = []
        for i in range(0, len(filter_zones_flat), 2):
            if i + 1 < len(filter_zones_flat):
                start_deg = filter_zones_flat[i]
                end_deg = filter_zones_flat[i + 1]
                start_rad = math.radians(start_deg)
                end_rad = math.radians(end_deg)
                self.filter_zones_rad.append((min(start_rad, end_rad), max(start_rad, end_rad)))
        
        self.get_logger().info(f'Scan filter initialized:')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Min range: {self.min_range}m')
        self.get_logger().info(f'  Max range: {self.max_range}m')
        self.get_logger().info(f'  Filter zones (deg): {filter_zones_flat}')
        
        # Publisher and subscriber
        self.pub = self.create_publisher(LaserScan, output_topic, 10)
        self.sub = self.create_subscription(LaserScan, input_topic, self.scan_callback, 10)
        
    def scan_callback(self, msg: LaserScan):
        """Filter the scan and republish."""
        
        # Create a copy of the message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = max(msg.range_min, self.min_range)
        filtered_msg.range_max = min(msg.range_max, self.max_range)
        
        # Filter the ranges
        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if msg.intensities else []
        
        for i, r in enumerate(ranges):
            # Calculate angle for this ray
            angle = msg.angle_min + i * msg.angle_increment
            
            # Check if angle is in any filter zone
            in_filter_zone = False
            for zone_min, zone_max in self.filter_zones_rad:
                if zone_min <= angle <= zone_max:
                    in_filter_zone = True
                    break
            
            # Filter out: set to infinity (will be ignored by costmap)
            if in_filter_zone:
                ranges[i] = float('inf')
            elif r < self.min_range or r > self.max_range:
                ranges[i] = float('inf')
        
        filtered_msg.ranges = ranges
        filtered_msg.intensities = intensities
        
        self.pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
