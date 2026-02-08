#!/usr/bin/env python3
"""
Exhibit Navigator - Smart route-based navigation for tour guide robot

Instead of using the default path planner (which may take awkward routes),
this node uses predefined routes between exhibits.

The Pi/LLM sends exhibit names (e.g., "go to exhibit_a"), and this node:
1. Looks up the exhibit location
2. Finds a predefined route from current location to destination
3. Navigates through waypoints in the route
4. Announces arrival (publishes to /exhibit_arrival)

This ensures the robot takes sensible, pre-planned paths through the gallery.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState
from action_msgs.msg import GoalStatus

import yaml
import time
import os
import math
import json
from enum import Enum
from typing import Dict, List, Optional, Tuple


class ExhibitNavState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    WAITING_AT_EXHIBIT = "waiting"
    TOURING = "touring"
    PAUSED = "paused"
    ERROR = "error"
    OBSTRUCTED = "obstructed"


class EventType(Enum):
    """Event types for Pi/LLM notifications"""
    EXHIBIT_REACHED = "exhibit_reached"
    NAVIGATION_STARTED = "navigation_started"
    NAVIGATION_FAILED = "navigation_failed"
    NAVIGATION_CANCELLED = "navigation_cancelled"
    TOUR_STARTED = "tour_started"
    TOUR_COMPLETED = "tour_completed"
    TOUR_PAUSED = "tour_paused"
    TOUR_RESUMED = "tour_resumed"
    ROBOT_OBSTRUCTED = "robot_obstructed"
    ROBOT_RECOVERED = "robot_recovered"
    LOW_BATTERY = "low_battery"
    CRITICAL_BATTERY = "critical_battery"
    ERROR = "error"
    STATUS_UPDATE = "status_update"


class ExhibitNavigator(Node):
    def __init__(self):
        super().__init__('exhibit_navigator')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # State
        self.state = ExhibitNavState.IDLE
        self.current_exhibit: Optional[str] = None
        self.target_exhibit: Optional[str] = None
        self.current_route: List[List[float]] = []
        self.route_index = 0
        self.tour_index = 0
        self.is_touring = False
        
        # Configuration
        self.config_file = os.path.expanduser(
            '~/sahabat_ws/src/shbat_pkg/config/exhibit_routes.yaml')
        self.exhibits: Dict = {}
        self.routes: Dict = {}
        self.default_tour: List[str] = []
        self.settings: Dict = {}
        self.load_config()
        
        # Current robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Battery status
        self.battery_level = 100.0
        self.battery_low_warned = False
        self.battery_critical_warned = False
        
        # Obstruction detection
        self.last_progress_time = time.time()
        self.last_position = (0.0, 0.0)
        self.obstruction_timeout = 10.0  # default, can be overridden by config
        self.is_obstructed = False
        
        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group)
        
        self.nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses',
            callback_group=self.callback_group)
        
        # Publishers
        self.arrival_pub = self.create_publisher(String, '/exhibit_arrival', 10)
        self.status_pub = self.create_publisher(String, '/exhibit_nav_status', 10)
        self.event_pub = self.create_publisher(String, '/robot_events', 10)  # For Pi/LLM
        
        # Subscribers
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10,
            callback_group=self.callback_group)
        
        self.command_sub = self.create_subscription(
            String, '/exhibit_command', self.command_callback, 10,
            callback_group=self.callback_group)
        
        # Battery subscription (optional - may not exist on all robots)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10,
            callback_group=self.callback_group)
        
        # Timer for status updates
        self.create_timer(1.0, self.publish_status, callback_group=self.callback_group)
        
        # Timer for obstruction detection
        self.create_timer(2.0, self.check_obstruction, callback_group=self.callback_group)
        
        self.get_logger().info('Exhibit Navigator started')
        self.get_logger().info(f'Loaded {len(self.exhibits)} exhibits and {len(self.routes)} routes')
    
    def load_config(self):
        """Load exhibit and route configuration"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f)
                    self.exhibits = config.get('exhibits', {})
                    self.routes = config.get('routes', {})
                    self.default_tour = config.get('default_tour', [])
                    self.settings = config.get('settings', {})
                    
                    # Apply settings
                    self.obstruction_timeout = self.settings.get('obstruction_timeout', 10.0)
                    
                    self.get_logger().info(f'Loaded config: {len(self.exhibits)} exhibits')
            else:
                self.get_logger().warn(f'Config file not found: {self.config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
    
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Update current robot pose"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Update current exhibit if we're close to one
        self.current_exhibit = self.find_nearest_exhibit(threshold=0.5)
        
        # Check for movement (for obstruction detection)
        dx = self.robot_x - self.last_position[0]
        dy = self.robot_y - self.last_position[1]
        movement = math.sqrt(dx*dx + dy*dy)
        
        if movement > 0.05:  # Moved at least 5cm
            self.last_progress_time = time.time()
            self.last_position = (self.robot_x, self.robot_y)
            
            # Clear obstruction if we were obstructed
            if self.is_obstructed:
                self.is_obstructed = False
                self.publish_event(EventType.ROBOT_RECOVERED, {
                    'message': 'Robot is moving again'
                })
    
    def battery_callback(self, msg: BatteryState):
        """Handle battery state updates"""
        self.battery_level = msg.percentage * 100.0  # Convert to percentage
        
        low_threshold = self.settings.get('battery_low_threshold', 30.0)
        critical_threshold = self.settings.get('battery_critical_threshold', 15.0)
        
        # Critical battery warning
        if self.battery_level <= critical_threshold and not self.battery_critical_warned:
            self.battery_critical_warned = True
            self.publish_event(EventType.CRITICAL_BATTERY, {
                'battery_level': self.battery_level,
                'message': f'Critical battery! {self.battery_level:.0f}% remaining. Return to charging station immediately!'
            })
            self.get_logger().error(f'CRITICAL BATTERY: {self.battery_level:.0f}%')
        
        # Low battery warning
        elif self.battery_level <= low_threshold and not self.battery_low_warned:
            self.battery_low_warned = True
            self.publish_event(EventType.LOW_BATTERY, {
                'battery_level': self.battery_level,
                'message': f'Low battery warning: {self.battery_level:.0f}% remaining'
            })
            self.get_logger().warn(f'Low battery: {self.battery_level:.0f}%')
        
        # Reset warnings if charged
        if self.battery_level > low_threshold + 5:
            self.battery_low_warned = False
        if self.battery_level > critical_threshold + 5:
            self.battery_critical_warned = False
    
    def check_obstruction(self):
        """Check if robot is obstructed (not making progress)"""
        if self.state not in [ExhibitNavState.NAVIGATING, ExhibitNavState.TOURING]:
            return
        
        time_since_progress = time.time() - self.last_progress_time
        
        if time_since_progress > self.obstruction_timeout and not self.is_obstructed:
            self.is_obstructed = True
            self.state = ExhibitNavState.OBSTRUCTED
            self.publish_event(EventType.ROBOT_OBSTRUCTED, {
                'target': self.target_exhibit,
                'position': {'x': self.robot_x, 'y': self.robot_y},
                'time_stuck': time_since_progress,
                'message': f'Robot appears to be blocked. No movement for {time_since_progress:.0f}s'
            })
            self.get_logger().warn(f'Robot obstructed! No movement for {time_since_progress:.0f}s')
    
    def publish_event(self, event_type: EventType, data: dict):
        """Publish event to /robot_events for Pi/LLM"""
        msg = String()
        event = {
            'event': event_type.value,
            'timestamp': time.time(),
            **data
        }
        msg.data = json.dumps(event)
        self.event_pub.publish(msg)
        self.get_logger().info(f'Event: {event_type.value}')
    
    def find_nearest_exhibit(self, threshold: float = 1.0) -> Optional[str]:
        """Find which exhibit we're currently at (within threshold distance)"""
        for name, exhibit in self.exhibits.items():
            dx = self.robot_x - exhibit['x']
            dy = self.robot_y - exhibit['y']
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < threshold:
                return name
        return None
    
    def find_route(self, from_exhibit: str, to_exhibit: str) -> List[List[float]]:
        """Find predefined route between two exhibits"""
        # Try direct route
        route_key = f"{from_exhibit}->{to_exhibit}"
        if route_key in self.routes:
            return self.routes[route_key]
        
        # Try reverse route (reversed waypoints)
        reverse_key = f"{to_exhibit}->{from_exhibit}"
        if reverse_key in self.routes:
            return list(reversed(self.routes[reverse_key]))
        
        # No predefined route
        return []
    
    def command_callback(self, msg: String):
        """Handle commands from Pi/LLM"""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '')
            
            if action == 'goto':
                exhibit_name = cmd.get('exhibit')
                if exhibit_name:
                    self.navigate_to_exhibit(exhibit_name)
                    
            elif action == 'start_tour':
                exhibits = cmd.get('exhibits', self.default_tour)
                self.start_tour(exhibits)
                
            elif action == 'stop':
                self.stop_navigation()
                
            elif action == 'pause':
                self.pause_navigation()
                
            elif action == 'resume':
                self.resume_navigation()
                
            elif action == 'list_exhibits':
                self.publish_exhibit_list()
                
            elif action == 'reload_config':
                self.load_config()
                
        except json.JSONDecodeError:
            # Treat as simple exhibit name
            self.navigate_to_exhibit(msg.data.strip())
        except Exception as e:
            self.get_logger().error(f'Command error: {e}')
    
    def navigate_to_exhibit(self, exhibit_name: str):
        """Navigate to a specific exhibit using predefined route"""
        if exhibit_name not in self.exhibits:
            self.get_logger().error(f'Unknown exhibit: {exhibit_name}')
            self.state = ExhibitNavState.ERROR
            return False
        
        self.target_exhibit = exhibit_name
        exhibit = self.exhibits[exhibit_name]
        
        # Find route from current location
        from_exhibit = self.current_exhibit or self.find_nearest_exhibit(threshold=2.0)
        
        if from_exhibit and from_exhibit != exhibit_name:
            route = self.find_route(from_exhibit, exhibit_name)
            if route:
                self.get_logger().info(f'Using predefined route: {from_exhibit} -> {exhibit_name} ({len(route)} waypoints)')
                self.current_route = route
                self.route_index = 0
                self.state = ExhibitNavState.NAVIGATING
                self.navigate_through_route(exhibit)
                return True
        
        # No route found or same exhibit, use direct navigation
        if self.settings.get('allow_direct_navigation', True):
            self.get_logger().info(f'Direct navigation to {exhibit_name}')
            self.current_route = []
            self.state = ExhibitNavState.NAVIGATING
            self.navigate_to_pose(exhibit['x'], exhibit['y'], exhibit.get('yaw', 0.0))
            return True
        else:
            self.get_logger().error(f'No route found and direct navigation disabled')
            self.state = ExhibitNavState.ERROR
            return False
    
    def navigate_through_route(self, final_exhibit: Dict):
        """Navigate through waypoints in route, then to final exhibit"""
        # Build list of all poses: route waypoints + final destination
        poses = []
        
        # Add route waypoints
        for wp in self.current_route:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            yaw = wp[2] if len(wp) > 2 else 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        
        # Add final destination
        final_pose = PoseStamped()
        final_pose.header.frame_id = 'map'
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.pose.position.x = final_exhibit['x']
        final_pose.pose.position.y = final_exhibit['y']
        yaw = final_exhibit.get('yaw', 0.0)
        final_pose.pose.orientation.z = math.sin(yaw / 2.0)
        final_pose.pose.orientation.w = math.cos(yaw / 2.0)
        poses.append(final_pose)
        
        # Use NavigateThroughPoses action
        if not self.nav_through_poses_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateThroughPoses not available, using sequential navigation')
            # Fallback: navigate to first waypoint
            if self.current_route:
                wp = self.current_route[0]
                self.navigate_to_pose(wp[0], wp[1], wp[2] if len(wp) > 2 else 0.0)
            else:
                self.navigate_to_pose(final_exhibit['x'], final_exhibit['y'], final_exhibit.get('yaw', 0.0))
            return
        
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        
        self.get_logger().info(f'Navigating through {len(poses)} poses')
        future = self.nav_through_poses_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response_callback)
    
    def navigate_to_pose(self, x: float, y: float, yaw: float):
        """Navigate to a single pose"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Navigation server not available')
            self.state = ExhibitNavState.ERROR
            return
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.state = ExhibitNavState.ERROR
            self.publish_event(EventType.NAVIGATION_FAILED, {
                'target': self.target_exhibit,
                'reason': 'Goal rejected by navigation server',
                'message': f'Failed to start navigation to {self.target_exhibit}'
            })
            return
        
        self.get_logger().info('Navigation goal accepted')
        self.publish_event(EventType.NAVIGATION_STARTED, {
            'target': self.target_exhibit,
            'from': self.current_exhibit,
            'message': f'Navigating to {self.target_exhibit}'
        })
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached {self.target_exhibit}!')
            self.on_exhibit_reached()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation cancelled')
            self.state = ExhibitNavState.IDLE
            self.publish_event(EventType.NAVIGATION_CANCELLED, {
                'target': self.target_exhibit,
                'message': f'Navigation to {self.target_exhibit} was cancelled'
            })
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            self.state = ExhibitNavState.ERROR
            self.publish_event(EventType.NAVIGATION_FAILED, {
                'target': self.target_exhibit,
                'status_code': status,
                'position': {'x': self.robot_x, 'y': self.robot_y},
                'message': f'Navigation to {self.target_exhibit} failed (status: {status})'
            })
    
    def on_exhibit_reached(self):
        """Called when robot arrives at an exhibit"""
        exhibit_info = self.exhibits.get(self.target_exhibit, {})
        
        # Publish arrival message (legacy topic)
        arrival_msg = String()
        arrival_msg.data = json.dumps({
            'exhibit': self.target_exhibit,
            'description': exhibit_info.get('description', ''),
            'x': self.robot_x,
            'y': self.robot_y,
        })
        self.arrival_pub.publish(arrival_msg)
        
        # Publish to new event topic
        self.publish_event(EventType.EXHIBIT_REACHED, {
            'exhibit': self.target_exhibit,
            'description': exhibit_info.get('description', ''),
            'position': {'x': self.robot_x, 'y': self.robot_y, 'yaw': self.robot_yaw},
            'message': f'Arrived at {exhibit_info.get("description", self.target_exhibit)}'
        })
        
        self.current_exhibit = self.target_exhibit
        
        # If touring, continue to next exhibit
        if self.is_touring:
            self.state = ExhibitNavState.WAITING_AT_EXHIBIT
            wait_time = self.settings.get('wait_at_exhibit', 5.0)
            self.get_logger().info(f'Waiting {wait_time}s at {self.target_exhibit}')
            self.create_timer(wait_time, self.continue_tour, callback_group=self.callback_group)
        else:
            self.state = ExhibitNavState.IDLE
    
    def start_tour(self, exhibits: List[str]):
        """Start a tour through multiple exhibits"""
        # Validate exhibits
        valid_exhibits = [e for e in exhibits if e in self.exhibits]
        if not valid_exhibits:
            self.get_logger().error('No valid exhibits in tour')
            self.publish_event(EventType.ERROR, {
                'error': 'invalid_tour',
                'message': 'No valid exhibits specified for tour'
            })
            return
        
        self.tour_exhibits = valid_exhibits
        self.tour_index = 0
        self.is_touring = True
        self.state = ExhibitNavState.TOURING
        
        self.get_logger().info(f'Starting tour: {" -> ".join(valid_exhibits)}')
        self.publish_event(EventType.TOUR_STARTED, {
            'exhibits': valid_exhibits,
            'total_stops': len(valid_exhibits),
            'message': f'Starting tour with {len(valid_exhibits)} stops'
        })
        
        self.navigate_to_exhibit(self.tour_exhibits[0])
    
    def continue_tour(self):
        """Continue to next exhibit in tour"""
        if not self.is_touring:
            return
        
        self.tour_index += 1
        if self.tour_index < len(self.tour_exhibits):
            self.get_logger().info(f'Tour: going to exhibit {self.tour_index + 1}/{len(self.tour_exhibits)}')
            self.navigate_to_exhibit(self.tour_exhibits[self.tour_index])
        else:
            self.get_logger().info('Tour complete!')
            self.is_touring = False
            self.state = ExhibitNavState.IDLE
            self.publish_event(EventType.TOUR_COMPLETED, {
                'exhibits_visited': self.tour_exhibits,
                'message': 'Tour completed successfully!'
            })
    
    def stop_navigation(self):
        """Stop all navigation"""
        was_touring = self.is_touring
        self.is_touring = False
        self.state = ExhibitNavState.IDLE
        # Cancel any active goals
        if self.nav_client.server_is_ready():
            self.nav_client._cancel_goal_async(None)  # Cancel all
        self.get_logger().info('Navigation stopped')
        
        if was_touring:
            self.publish_event(EventType.NAVIGATION_CANCELLED, {
                'message': 'Tour was stopped by user',
                'progress': f'{self.tour_index + 1}/{len(self.tour_exhibits)}'
            })
    
    def pause_navigation(self):
        """Pause navigation (for tours)"""
        if self.state == ExhibitNavState.TOURING:
            self.state = ExhibitNavState.PAUSED
            self.get_logger().info('Tour paused')
            self.publish_event(EventType.TOUR_PAUSED, {
                'current_exhibit': self.target_exhibit,
                'progress': f'{self.tour_index + 1}/{len(self.tour_exhibits)}',
                'message': 'Tour paused'
            })
    
    def resume_navigation(self):
        """Resume paused navigation"""
        if self.state == ExhibitNavState.PAUSED:
            self.state = ExhibitNavState.TOURING
            self.navigate_to_exhibit(self.tour_exhibits[self.tour_index])
            self.get_logger().info('Tour resumed')
            self.publish_event(EventType.TOUR_RESUMED, {
                'next_exhibit': self.tour_exhibits[self.tour_index],
                'progress': f'{self.tour_index + 1}/{len(self.tour_exhibits)}',
                'message': 'Tour resumed'
            })
    
    def publish_exhibit_list(self):
        """Publish list of available exhibits"""
        msg = String()
        msg.data = json.dumps({
            'exhibits': {name: exhibit.get('description', '') 
                        for name, exhibit in self.exhibits.items()},
            'default_tour': self.default_tour
        })
        self.status_pub.publish(msg)
    
    def publish_status(self):
        """Publish current navigation status"""
        msg = String()
        msg.data = json.dumps({
            'state': self.state.value,
            'current_exhibit': self.current_exhibit,
            'target_exhibit': self.target_exhibit,
            'is_touring': self.is_touring,
            'tour_progress': f"{self.tour_index + 1}/{len(self.tour_exhibits)}" if self.is_touring else None,
            'robot_position': {'x': self.robot_x, 'y': self.robot_y, 'yaw': self.robot_yaw},
            'battery_level': self.battery_level,
            'is_obstructed': self.is_obstructed
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExhibitNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
