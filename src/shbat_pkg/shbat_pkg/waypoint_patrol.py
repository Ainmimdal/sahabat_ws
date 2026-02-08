#!/usr/bin/env python3
"""
Waypoint Patrol Node for Sahabat Robot

This node allows the robot to patrol through a sequence of waypoints.

Features:
- Define waypoints via YAML file or programmatically
- Continuous patrol loop or single run
- Pause/Resume/Stop controls via services
- Status feedback via topic

Usage:
    ros2 run shbat_pkg waypoint_patrol
    ros2 run shbat_pkg waypoint_patrol --ros-args -p waypoints_file:=/path/to/waypoints.yaml

Services:
    /patrol/start   - Start patrol (std_srvs/Trigger)
    /patrol/stop    - Stop patrol (std_srvs/Trigger)
    /patrol/pause   - Pause patrol (std_srvs/Trigger)
    /patrol/resume  - Resume patrol (std_srvs/Trigger)

Topics:
    /patrol/status  - Current patrol status (std_msgs/String)
    /patrol/add_waypoint - Add waypoint at current position (std_msgs/Empty)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

import yaml
import os
import math
from enum import Enum


class PatrolState(Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    PAUSED = "paused"
    NAVIGATING = "navigating"
    WAITING = "waiting"


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')
        
        # Parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('loop', True)  # Continuous patrol
        self.declare_parameter('wait_duration', 2.0)  # Seconds to wait at each waypoint
        self.declare_parameter('frame_id', 'map')
        
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.loop = self.get_parameter('loop').value
        self.wait_duration = self.get_parameter('wait_duration').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # State
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.state = PatrolState.IDLE
        self.current_pose = None
        
        # Callback group for concurrent callbacks
        self.cb_group = ReentrantCallbackGroup()
        
        # Action client for Nav2
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.cb_group
        )
        
        # Services
        self.start_srv = self.create_service(
            Trigger, 'patrol/start', self.start_callback, callback_group=self.cb_group)
        self.stop_srv = self.create_service(
            Trigger, 'patrol/stop', self.stop_callback, callback_group=self.cb_group)
        self.pause_srv = self.create_service(
            Trigger, 'patrol/pause', self.pause_callback, callback_group=self.cb_group)
        self.resume_srv = self.create_service(
            Trigger, 'patrol/resume', self.resume_callback, callback_group=self.cb_group)
        self.save_srv = self.create_service(
            Trigger, 'patrol/save_waypoints', self.save_waypoints_callback, callback_group=self.cb_group)
        self.clear_srv = self.create_service(
            Trigger, 'patrol/clear_waypoints', self.clear_waypoints_callback, callback_group=self.cb_group)
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'patrol/status', 10)
        
        # Subscribers
        self.add_waypoint_sub = self.create_subscription(
            Empty, 'patrol/add_waypoint', self.add_waypoint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Goal handle
        self.current_goal_handle = None
        
        # Load waypoints from file if specified
        if self.waypoints_file and os.path.exists(self.waypoints_file):
            self.load_waypoints(self.waypoints_file)
        
        self.get_logger().info('Waypoint Patrol node initialized')
        self.get_logger().info(f'  Loop mode: {self.loop}')
        self.get_logger().info(f'  Wait duration: {self.wait_duration}s')
        self.get_logger().info(f'  Waypoints loaded: {len(self.waypoints)}')
        self.get_logger().info('Services: /patrol/start, /patrol/stop, /patrol/pause, /patrol/resume')
        self.get_logger().info('Add waypoint: ros2 topic pub /patrol/add_waypoint std_msgs/Empty "{}" --once')

    def odom_callback(self, msg: Odometry):
        """Store current robot pose."""
        self.current_pose = msg.pose.pose

    def add_waypoint_callback(self, msg: Empty):
        """Add current position as a waypoint."""
        if self.current_pose is None:
            self.get_logger().warn('No odometry received yet, cannot add waypoint')
            return
        
        waypoint = {
            'x': self.current_pose.position.x,
            'y': self.current_pose.position.y,
            'yaw': self.quaternion_to_yaw(self.current_pose.orientation)
        }
        self.waypoints.append(waypoint)
        self.get_logger().info(
            f'Added waypoint {len(self.waypoints)}: '
            f'x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f}, yaw={waypoint["yaw"]:.2f}'
        )

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def load_waypoints(self, filepath):
        """Load waypoints from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('waypoints', [])
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

    def save_waypoints_callback(self, request, response):
        """Save current waypoints to file."""
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints to save'
            return response
        
        filepath = self.waypoints_file if self.waypoints_file else '/tmp/patrol_waypoints.yaml'
        try:
            with open(filepath, 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False)
            response.success = True
            response.message = f'Saved {len(self.waypoints)} waypoints to {filepath}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save: {e}'
        return response

    def clear_waypoints_callback(self, request, response):
        """Clear all waypoints."""
        self.waypoints = []
        self.current_waypoint_idx = 0
        response.success = True
        response.message = 'Waypoints cleared'
        self.get_logger().info('Waypoints cleared')
        return response

    def start_callback(self, request, response):
        """Start patrol."""
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints defined. Add waypoints first.'
            return response
        
        if self.state == PatrolState.PATROLLING or self.state == PatrolState.NAVIGATING:
            response.success = False
            response.message = 'Patrol already running'
            return response
        
        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = 'Nav2 action server not available'
            return response
        
        self.current_waypoint_idx = 0
        self.state = PatrolState.PATROLLING
        response.success = True
        response.message = f'Starting patrol with {len(self.waypoints)} waypoints'
        self.get_logger().info(response.message)
        
        # Start navigating to first waypoint
        self.navigate_to_waypoint(self.current_waypoint_idx)
        
        return response

    def stop_callback(self, request, response):
        """Stop patrol."""
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
        
        self.state = PatrolState.IDLE
        self.current_waypoint_idx = 0
        response.success = True
        response.message = 'Patrol stopped'
        self.get_logger().info('Patrol stopped')
        return response

    def pause_callback(self, request, response):
        """Pause patrol."""
        if self.state not in [PatrolState.PATROLLING, PatrolState.NAVIGATING]:
            response.success = False
            response.message = 'Not currently patrolling'
            return response
        
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
        
        self.state = PatrolState.PAUSED
        response.success = True
        response.message = f'Patrol paused at waypoint {self.current_waypoint_idx + 1}'
        self.get_logger().info(response.message)
        return response

    def resume_callback(self, request, response):
        """Resume patrol."""
        if self.state != PatrolState.PAUSED:
            response.success = False
            response.message = 'Patrol not paused'
            return response
        
        self.state = PatrolState.PATROLLING
        response.success = True
        response.message = f'Resuming patrol from waypoint {self.current_waypoint_idx + 1}'
        self.get_logger().info(response.message)
        
        self.navigate_to_waypoint(self.current_waypoint_idx)
        return response

    def navigate_to_waypoint(self, idx):
        """Send navigation goal to waypoint."""
        if idx >= len(self.waypoints):
            if self.loop:
                idx = 0
                self.current_waypoint_idx = 0
            else:
                self.state = PatrolState.IDLE
                self.get_logger().info('Patrol complete!')
                return
        
        waypoint = self.waypoints[idx]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        goal_msg.pose.pose.position.z = 0.0
        
        q = self.yaw_to_quaternion(waypoint.get('yaw', 0.0))
        goal_msg.pose.pose.orientation.x = q['x']
        goal_msg.pose.pose.orientation.y = q['y']
        goal_msg.pose.pose.orientation.z = q['z']
        goal_msg.pose.pose.orientation.w = q['w']
        
        self.state = PatrolState.NAVIGATING
        self.get_logger().info(
            f'Navigating to waypoint {idx + 1}/{len(self.waypoints)}: '
            f'x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f}'
        )
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        pass  # Could add progress logging here

    def nav_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        
        if self.state == PatrolState.PAUSED or self.state == PatrolState.IDLE:
            return
        
        # Move to next waypoint
        self.current_waypoint_idx += 1
        
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.loop:
                self.current_waypoint_idx = 0
                self.get_logger().info('Completed patrol loop, restarting...')
            else:
                self.state = PatrolState.IDLE
                self.get_logger().info('Patrol complete!')
                return
        
        # Wait at waypoint then continue
        self.state = PatrolState.WAITING
        self.get_logger().info(f'Reached waypoint, waiting {self.wait_duration}s...')
        self.create_timer(
            self.wait_duration, 
            self.wait_complete_callback,
            callback_group=self.cb_group
        )

    def wait_complete_callback(self):
        """Called after waiting at waypoint."""
        # One-shot timer - destroy it
        # Note: In ROS2, we should manage timer lifecycle better
        if self.state == PatrolState.WAITING:
            self.state = PatrolState.PATROLLING
            self.navigate_to_waypoint(self.current_waypoint_idx)

    def publish_status(self):
        """Publish patrol status."""
        status_msg = String()
        status_msg.data = (
            f'State: {self.state.value} | '
            f'Waypoint: {self.current_waypoint_idx + 1}/{len(self.waypoints)} | '
            f'Loop: {self.loop}'
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointPatrol()
    
    # Use multi-threaded executor for concurrent service calls
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
