#!/usr/bin/env python3
"""
Waypoint Collector - GUI-based waypoint recording using RViz

Usage:
  1. Run this node
  2. Use "2D Goal Pose" button in RViz to click waypoints
  3. Each click adds a waypoint to the list
  4. When done, call the save service

Services:
  /waypoint_collector/save - Save waypoints to file
  /waypoint_collector/clear - Clear all waypoints
  /waypoint_collector/undo - Remove last waypoint
  /waypoint_collector/list - Print current waypoints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os
import math


class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')
        
        # Parameters
        self.declare_parameter('output_file', 
            os.path.expanduser('~/sahabat_ws/src/shbat_pkg/config/patrol_waypoints.yaml'))
        self.output_file = self.get_parameter('output_file').value
        
        # Waypoint storage
        self.waypoints = []
        
        # Subscribe to goal pose (from RViz "2D Goal Pose" button)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Services
        self.save_srv = self.create_service(
            Trigger, '/waypoint_collector/save', self.save_callback)
        self.clear_srv = self.create_service(
            Trigger, '/waypoint_collector/clear', self.clear_callback)
        self.undo_srv = self.create_service(
            Trigger, '/waypoint_collector/undo', self.undo_callback)
        self.list_srv = self.create_service(
            Trigger, '/waypoint_collector/list', self.list_callback)
        
        # Timer to publish markers
        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('='*50)
        self.get_logger().info('WAYPOINT COLLECTOR - GUI Mode')
        self.get_logger().info('='*50)
        self.get_logger().info('Use "2D Goal Pose" in RViz to add waypoints')
        self.get_logger().info('')
        self.get_logger().info('Commands:')
        self.get_logger().info('  Save:  ros2 service call /waypoint_collector/save std_srvs/srv/Trigger')
        self.get_logger().info('  Undo:  ros2 service call /waypoint_collector/undo std_srvs/srv/Trigger')
        self.get_logger().info('  Clear: ros2 service call /waypoint_collector/clear std_srvs/srv/Trigger')
        self.get_logger().info('  List:  ros2 service call /waypoint_collector/list std_srvs/srv/Trigger')
        self.get_logger().info('='*50)

    def goal_callback(self, msg: PoseStamped):
        """Called when user clicks 2D Goal Pose in RViz"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        waypoint_num = len(self.waypoints) + 1
        waypoint = {
            'name': f'waypoint_{waypoint_num}',
            'x': round(x, 3),
            'y': round(y, 3),
            'yaw': round(yaw, 3)
        }
        
        self.waypoints.append(waypoint)
        
        self.get_logger().info(f'✓ Added waypoint {waypoint_num}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)')
        self.get_logger().info(f'  Total waypoints: {len(self.waypoints)}')
        
        # Update markers
        self.publish_markers()

    def publish_markers(self):
        """Publish visualization markers for waypoints"""
        marker_array = MarkerArray()
        
        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, wp in enumerate(self.waypoints):
            # Sphere marker for position
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = wp['x']
            text_marker.pose.position.y = wp['y']
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{i+1}: {wp['name']}"
            marker_array.markers.append(text_marker)
            
            # Arrow for direction
            arrow_marker = Marker()
            arrow_marker.header.frame_id = 'map'
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = 'waypoint_arrows'
            arrow_marker.id = i + 2000
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = wp['x']
            arrow_marker.pose.position.y = wp['y']
            arrow_marker.pose.position.z = 0.1
            # Set orientation from yaw
            arrow_marker.pose.orientation.z = math.sin(wp['yaw'] / 2)
            arrow_marker.pose.orientation.w = math.cos(wp['yaw'] / 2)
            arrow_marker.scale.x = 0.5
            arrow_marker.scale.y = 0.1
            arrow_marker.scale.z = 0.1
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.5
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 0.8
            marker_array.markers.append(arrow_marker)
        
        self.marker_pub.publish(marker_array)

    def save_callback(self, request, response):
        """Save waypoints to YAML file"""
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints to save!'
            return response
        
        try:
            data = {'waypoints': self.waypoints}
            
            # Create directory if needed
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            
            with open(self.output_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            response.success = True
            response.message = f'Saved {len(self.waypoints)} waypoints to {self.output_file}'
            self.get_logger().info(f'✓ {response.message}')
            
        except Exception as e:
            response.success = False
            response.message = f'Error saving: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def clear_callback(self, request, response):
        """Clear all waypoints"""
        count = len(self.waypoints)
        self.waypoints = []
        self.publish_markers()
        response.success = True
        response.message = f'Cleared {count} waypoints'
        self.get_logger().info(f'✓ {response.message}')
        return response

    def undo_callback(self, request, response):
        """Remove last waypoint"""
        if self.waypoints:
            removed = self.waypoints.pop()
            self.publish_markers()
            response.success = True
            response.message = f'Removed waypoint: {removed["name"]}'
            self.get_logger().info(f'✓ {response.message}')
        else:
            response.success = False
            response.message = 'No waypoints to remove'
        return response

    def list_callback(self, request, response):
        """List all waypoints"""
        if not self.waypoints:
            response.success = True
            response.message = 'No waypoints recorded'
        else:
            lines = [f'{len(self.waypoints)} waypoints:']
            for i, wp in enumerate(self.waypoints):
                lines.append(f'  {i+1}. {wp["name"]}: x={wp["x"]}, y={wp["y"]}, yaw={wp["yaw"]}')
            response.success = True
            response.message = '\n'.join(lines)
            self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
