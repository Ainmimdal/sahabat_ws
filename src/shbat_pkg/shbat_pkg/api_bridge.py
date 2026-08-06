#!/usr/bin/env python3
"""
API Bridge - REST API server for LLM/Pi communication

This node provides HTTP endpoints for:
- Sending waypoint/navigation commands to the robot
- Receiving robot status (battery, position, navigation state, stuck detection)

Endpoints:
  POST /navigate          - Navigate to a single pose
  POST /waypoints         - Send list of waypoints for patrol
  POST /patrol/start      - Start patrol with loaded waypoints
  POST /patrol/stop       - Stop current patrol
  POST /patrol/pause      - Pause patrol
  POST /patrol/resume     - Resume patrol
  GET  /status            - Get robot status (battery, pose, nav state)
  GET  /waypoints         - Get current waypoint list
  POST /cancel            - Cancel current navigation goal
  POST /emergency_stop    - Emergency stop all motion
  
Author: Sahabat Robot Team
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
from sahabat_interfaces.msg import OperatorStatus
from sahabat_interfaces.srv import (
    ControlLease,
    GetWaypoints,
    ListWaypointSets,
    PatrolCommand,
)

from shbat_pkg.tour_session import (
    DOCK_WAYPOINT_ID,
    TourConfigurationError,
    TourSession,
)

from flask import Flask, request, jsonify
from flask_cors import CORS
import signal
import threading
import math
import json
import yaml
import os
import time
from dataclasses import dataclass, asdict
from typing import List, Optional, Dict, Any
from enum import Enum


class NavState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    PATROLLING = "patrolling"
    PAUSED = "paused"
    STUCK = "stuck"
    REACHED_GOAL = "reached_goal"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class RobotStatus:
    battery_percentage: float = 0.0
    battery_voltage: float = 0.0
    battery_charging: bool = False
    position_x: float = 0.0
    position_y: float = 0.0
    orientation_yaw: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    nav_state: str = NavState.IDLE.value
    current_goal_x: Optional[float] = None
    current_goal_y: Optional[float] = None
    current_waypoint_index: int = 0
    total_waypoints: int = 0
    emergency_stop_active: bool = False
    is_stuck: bool = False
    stuck_duration: float = 0.0
    error_message: str = ""
    localization_healthy: bool = False
    map_healthy: bool = False
    scan_healthy: bool = False
    tf_healthy: bool = False
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class APIBridgeNode(Node):
    """ROS2 Node that bridges HTTP API to ROS2 topics/actions"""
    
    def __init__(self):
        super().__init__('api_bridge')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Robot status
        self.status = RobotStatus()
        self.waypoints: List[Dict] = []
        self.patrol_loop = True
        
        # Stuck detection
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.stuck_check_time = self.get_clock().now()
        self.stuck_threshold_distance = 0.05  # meters
        self.stuck_threshold_time = 10.0  # seconds
        
        # Goal handle
        self.current_goal_handle = None
        
        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group)
        self.waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints',
            callback_group=self.callback_group)
        self.operator_waypoints_client = self.create_client(
            GetWaypoints,
            '/operator/waypoints/get',
            callback_group=self.callback_group,
        )
        self.operator_lease_client = self.create_client(
            ControlLease,
            '/operator/control_lease',
            callback_group=self.callback_group,
        )
        self.operator_patrol_client = self.create_client(
            PatrolCommand,
            '/operator/patrol',
            callback_group=self.callback_group,
        )
        self.operator_waypoint_sets_client = self.create_client(
            ListWaypointSets,
            '/operator/waypoint_sets/list',
            callback_group=self.callback_group,
        )
        self.operator_active_map = ''
        self.operator_command_lock = threading.RLock()
        self.tour_lock = threading.RLock()
        self.navigation_state_condition = threading.Condition(self.tour_lock)
        self.catalog_refreshed_at = 0.0
        self.declare_parameter(
            'tour_profile', os.environ.get('SAHABOT_TOUR_PROFILE', 'auto')
        )
        self.declare_parameter('tour_arrival_tolerance_m', 0.5)
        self.declare_parameter('tour_almost_there_min_trip_m', 6.0)
        self.declare_parameter('tour_almost_there_distance_m', 2.5)
        self.declare_parameter('tour_almost_there_min_time_s', 8.0)
        self.tour = TourSession(
            profile=str(self.get_parameter('tour_profile').value),
            arrival_tolerance_m=float(
                self.get_parameter('tour_arrival_tolerance_m').value
            ),
            almost_there_min_trip_m=float(
                self.get_parameter('tour_almost_there_min_trip_m').value
            ),
            almost_there_distance_m=float(
                self.get_parameter('tour_almost_there_distance_m').value
            ),
            almost_there_min_time_s=float(
                self.get_parameter('tour_almost_there_min_time_s').value
            ),
        )

        estop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        
        # Publishers
        self.estop_pub = self.create_publisher(
            Bool, '/emergency_stop', estop_qos
        )
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.exhibit_cmd_pub = self.create_publisher(String, '/exhibit_command', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.callback_group)
        
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10,
            callback_group=self.callback_group)
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10,
            callback_group=self.callback_group)

        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, estop_qos,
            callback_group=self.callback_group)
        self.operator_status_sub = self.create_subscription(
            OperatorStatus,
            '/operator/status',
            self.operator_status_callback,
            10,
            callback_group=self.callback_group,
        )
        
        # Timer for stuck detection and status publishing
        self.status_timer = self.create_timer(
            1.0, self.status_check_callback,
            callback_group=self.callback_group)
        
        # Load existing waypoints
        self.waypoint_file = os.path.expanduser(
            '~/sahabat_ws/src/shbat_pkg/config/patrol_waypoints.yaml')
        self.load_waypoints()
        
        self.get_logger().info('API Bridge Node started')
        self.get_logger().info('Waiting for navigation action servers...')
        
    def load_waypoints(self):
        """Load waypoints from YAML file"""
        try:
            if os.path.exists(self.waypoint_file):
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        self.waypoints = data['waypoints']
                        self.status.total_waypoints = len(self.waypoints)
                        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
    
    def save_waypoints(self):
        """Save waypoints to YAML file"""
        try:
            os.makedirs(os.path.dirname(self.waypoint_file), exist_ok=True)
            with open(self.waypoint_file, 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False)
            self.get_logger().info(f'Saved {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')
    
    def odom_callback(self, msg: Odometry):
        """Update velocity from odometry"""
        self.status.linear_velocity = msg.twist.twist.linear.x
        self.status.angular_velocity = msg.twist.twist.angular.z
    
    def battery_callback(self, msg: BatteryState):
        """Update battery status"""
        self.status.battery_percentage = msg.percentage * 100
        self.status.battery_voltage = msg.voltage
        self.status.battery_charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
    
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update position from AMCL"""
        self.status.position_x = msg.pose.pose.position.x
        self.status.position_y = msg.pose.pose.position.y
        
        # Calculate yaw from quaternion
        q = msg.pose.pose.orientation
        self.status.orientation_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def estop_callback(self, msg: Bool):
        """Mirror the latched stop state for local web clients."""
        self.status.emergency_stop_active = bool(msg.data)

    def operator_status_callback(self, msg: OperatorStatus):
        """Mirror the active map and high-level navigation state."""
        self.operator_active_map = msg.active_map
        self.status.position_x = msg.pose.x
        self.status.position_y = msg.pose.y
        self.status.orientation_yaw = msg.pose.theta
        self.status.linear_velocity = msg.linear_velocity
        self.status.angular_velocity = msg.angular_velocity
        self.status.battery_percentage = msg.battery_percentage
        self.status.emergency_stop_active = msg.emergency_stop
        self.status.localization_healthy = msg.localization_healthy
        self.status.map_healthy = msg.map_healthy
        self.status.scan_healthy = msg.scan_healthy
        self.status.tf_healthy = msg.tf_healthy
        if msg.navigation_state:
            self.status.nav_state = msg.navigation_state
        with self.navigation_state_condition:
            self.tour.update_pose(
                msg.pose.x, msg.pose.y, msg.pose.theta
            )
            self.tour.observe_navigation_state(msg.navigation_state)
            self.navigation_state_condition.notify_all()

    @staticmethod
    def _call_service(client, request, timeout: float = 3.0):
        """Call a ROS service from a Flask worker while ROS spins elsewhere."""
        if not client.wait_for_service(timeout_sec=min(timeout, 1.0)):
            raise RuntimeError(f'ROS service unavailable: {client.srv_name}')
        completed = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout):
            raise RuntimeError(f'ROS service timed out: {client.srv_name}')
        error = future.exception()
        if error is not None:
            raise RuntimeError(str(error))
        return future.result()

    def _active_map_id(self) -> str:
        """Return the live operator map, with the editor state file as fallback."""
        if self.operator_active_map:
            return self.operator_active_map
        selected = os.path.expanduser('~/sahabat_ws/maps/last_selected_map')
        try:
            with open(selected, 'r', encoding='utf-8') as stream:
                return stream.read().strip()
        except OSError:
            return ''

    def get_operator_waypoints(self):
        """Load the active editor waypoint set through operator_backend."""
        map_id = self._active_map_id()
        if not map_id:
            raise RuntimeError('The waypoint editor has no active map')
        request = GetWaypoints.Request()
        request.map_id = map_id
        request.set_id = ''
        response = self._call_service(
            self.operator_waypoints_client, request
        )
        if not response.set_id:
            raise RuntimeError(response.message or 'No active waypoint set')
        return map_id, response

    def get_operator_dock(self, map_id: str):
        """Return the optional map-scoped dock through operator_backend."""
        request = ListWaypointSets.Request()
        request.map_id = map_id
        response = self._call_service(
            self.operator_waypoint_sets_client, request
        )
        if not response.has_dock:
            return None
        return {
            'id': response.dock.id or 'dock',
            'name': 'dock',
            'x': response.dock.pose.x,
            'y': response.dock.pose.y,
            'yaw': response.dock.pose.theta,
        }

    @staticmethod
    def _waypoint_dict(waypoint):
        return {
            'id': waypoint.id,
            'name': waypoint.name,
            'x': waypoint.pose.x,
            'y': waypoint.pose.y,
            'yaw': waypoint.pose.theta,
            'enabled': waypoint.enabled,
        }

    def refresh_tour_catalog(self):
        """Resolve the active map/set into the logical SahaBot catalog."""
        map_id, response = self.get_operator_waypoints()
        dock = self.get_operator_dock(map_id)
        waypoint_data = [
            self._waypoint_dict(waypoint)
            for waypoint in response.waypoints
        ]
        with self.tour_lock:
            self.tour.configure(
                map_id, response.set_id, waypoint_data, dock=dock
            )
            self.catalog_refreshed_at = time.monotonic()
            return self.tour.status_dict(), waypoint_data

    def list_operator_waypoints(self):
        """Return the map-aware logical catalog plus legacy waypoint data."""
        tour_status, waypoints = self.refresh_tour_catalog()
        return {
            **tour_status,
            'waypoints': [
                item for item in waypoints if item['enabled']
            ],
        }

    def _wait_for_goal_decision(self, event_id: int, timeout: float = 2.5):
        """Wait in the Flask worker for operator/Nav2 goal acceptance."""
        deadline = time.monotonic() + timeout
        with self.navigation_state_condition:
            while time.monotonic() < deadline:
                if self.tour.state in ('navigating', 'blocked'):
                    event = self.tour.latest_event
                    if event and event['id'] > event_id:
                        return dict(event)
                    return None
                if self.tour.state == 'failed':
                    raise RuntimeError('Navigation goal was rejected')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.navigation_state_condition.wait(min(0.1, remaining))
        return None

    def operator_patrol_command(
        self,
        command: int,
        waypoint_name: str = '',
        *,
        touring: Optional[bool] = None,
        resumed: bool = False,
    ):
        """Send a named command through operator_backend's lease boundary."""
        with self.operator_command_lock:
            set_id = ''
            waypoint_id = ''
            previous_event_id = 0
            if command == PatrolCommand.Request.NAVIGATE:
                unhealthy = []
                if self.status.emergency_stop_active:
                    unhealthy.append('emergency stop is active')
                for label, healthy in (
                    ('localization', self.status.localization_healthy),
                    ('map', self.status.map_healthy),
                    ('laser scan', self.status.scan_healthy),
                    ('transform tree', self.status.tf_healthy),
                ):
                    if not healthy:
                        unhealthy.append(f'{label} is not healthy')
                if unhealthy:
                    raise ValueError(
                        'Navigation is unavailable: ' + ', '.join(unhealthy)
                    )
                self.refresh_tour_catalog()
                with self.tour_lock:
                    if (
                        self.tour.target is not None
                        and self.tour.state in (
                            'goal_pending', 'navigating', 'blocked'
                        )
                    ):
                        raise ValueError(
                            'Navigation is already active; pause or cancel it first'
                        )
                    try:
                        target = (
                            self.tour.resolve_dock()
                            if waypoint_name == DOCK_WAYPOINT_ID
                            else self.tour.resolve_station(waypoint_name)
                        )
                    except TourConfigurationError as error:
                        raise ValueError(str(error)) from error
                    previous_event_id = (
                        self.tour.latest_event['id']
                        if self.tour.latest_event else 0
                    )
                    self.tour.prepare_navigation(
                        target,
                        touring=touring,
                        resumed=resumed,
                    )
                    set_id = self.tour.set_id if not target.is_dock else ''
                    waypoint_id = target.waypoint_id
            elif command == PatrolCommand.Request.PAUSE:
                with self.tour_lock:
                    if self.tour.target is None or self.tour.state not in (
                        'goal_pending', 'navigating', 'blocked'
                    ):
                        raise ValueError('There is no active navigation to pause')

            lease_request = ControlLease.Request()
            lease_request.action = ControlLease.Request.ACQUIRE
            lease_request.client_id = 'sahabot'
            lease = self._call_service(
                self.operator_lease_client, lease_request
            )
            if not lease.success:
                raise RuntimeError(lease.message or 'Control lease denied')

            try:
                patrol_request = PatrolCommand.Request()
                patrol_request.command = command
                patrol_request.set_id = set_id
                patrol_request.waypoint_id = waypoint_id
                patrol_request.loop = False
                patrol_request.lease_id = lease.lease_id
                result = self._call_service(
                    self.operator_patrol_client, patrol_request
                )
                if not result.success:
                    with self.tour_lock:
                        self.tour.mark_failed(result.message)
                    raise RuntimeError(
                        result.message or 'Operator command rejected'
                    )
                interaction = None
                if command == PatrolCommand.Request.NAVIGATE:
                    interaction = self._wait_for_goal_decision(
                        previous_event_id
                    )
                elif command == PatrolCommand.Request.PAUSE:
                    with self.tour_lock:
                        interaction = self.tour.mark_paused()
                elif command == PatrolCommand.Request.STOP:
                    with self.tour_lock:
                        interaction = self.tour.mark_cancelled()
                with self.tour_lock:
                    tour_status = self.tour.status_dict()
                return {
                    'success': True,
                    'message': result.message,
                    'interaction': interaction,
                    'tour': tour_status,
                }
            finally:
                release = ControlLease.Request()
                release.action = ControlLease.Request.RELEASE
                release.client_id = 'sahabot'
                release.lease_id = lease.lease_id
                try:
                    self._call_service(
                        self.operator_lease_client, release, timeout=1.0
                    )
                except RuntimeError as error:
                    self.get_logger().warn(str(error))

    def navigate_next(self, *, start_tour: bool = False):
        """Navigate to the next available exhibit and then wait there."""
        self.refresh_tour_catalog()
        with self.tour_lock:
            station_id = self.tour.next_station_id()
            if station_id is None:
                interaction = self.tour.mark_tour_complete()
                return {
                    'success': True,
                    'complete': True,
                    'message': interaction['text'],
                    'interaction': interaction,
                    'tour': self.tour.status_dict(),
                }
        return self.operator_patrol_command(
            PatrolCommand.Request.NAVIGATE,
            station_id,
            touring=True if start_tour else None,
        )

    def resume_tour_navigation(self):
        """Resume only the interrupted target, never the whole waypoint set."""
        with self.tour_lock:
            if self.tour.target is None or self.tour.state != 'paused':
                raise ValueError('There is no paused exhibit target')
            waypoint_name = (
                DOCK_WAYPOINT_ID
                if self.tour.target.is_dock
                else self.tour.target.station_id
            )
            touring = self.tour.touring
        return self.operator_patrol_command(
            PatrolCommand.Request.NAVIGATE,
            waypoint_name,
            touring=touring,
            resumed=True,
        )

    def tour_status(self, refresh: bool = True):
        """Return tour state merged with live pose, health, and battery data."""
        if refresh and time.monotonic() - self.catalog_refreshed_at > 2.0:
            try:
                self.refresh_tour_catalog()
            except RuntimeError as error:
                self.status.error_message = str(error)
        base = self.get_status()
        with self.tour_lock:
            self.tour.update_pose(
                base['position_x'],
                base['position_y'],
                base['orientation_yaw'],
            )
            self.tour.tick(is_stuck=base['is_stuck'])
            tour = self.tour.status_dict()
            if not base['localization_healthy']:
                tour['location_text'] = (
                    'My current gallery location is not available until '
                    'localization is ready.'
                )
        return {
            **tour,
            'robot_position': {
                'x': base['position_x'],
                'y': base['position_y'],
                'yaw': base['orientation_yaw'],
            },
            'battery_level': base['battery_percentage'],
            'is_obstructed': base['is_stuck'],
            'emergency_stop_active': base['emergency_stop_active'],
            'localization_healthy': base['localization_healthy'],
            'map_healthy': base['map_healthy'],
            'scan_healthy': base['scan_healthy'],
            'tf_healthy': base['tf_healthy'],
            'error_message': base['error_message'],
        }
    
    def status_check_callback(self):
        """Check for stuck condition and publish status"""
        # Stuck detection (only when navigating)
        if self.status.nav_state in [NavState.NAVIGATING.value, NavState.PATROLLING.value]:
            dx = self.status.position_x - self.last_pose_x
            dy = self.status.position_y - self.last_pose_y
            distance_moved = math.sqrt(dx*dx + dy*dy)
            
            now = self.get_clock().now()
            elapsed = (now - self.stuck_check_time).nanoseconds / 1e9
            
            if distance_moved < self.stuck_threshold_distance:
                self.status.stuck_duration += elapsed
                if self.status.stuck_duration > self.stuck_threshold_time:
                    self.status.is_stuck = True
                    self.get_logger().warn(f'Robot appears stuck for {self.status.stuck_duration:.1f}s')
            else:
                self.status.stuck_duration = 0.0
                self.status.is_stuck = False
            
            self.stuck_check_time = now
        else:
            self.status.stuck_duration = 0.0
            self.status.is_stuck = False
            self.stuck_check_time = self.get_clock().now()

        with self.tour_lock:
            self.tour.update_pose(
                self.status.position_x,
                self.status.position_y,
                self.status.orientation_yaw,
            )
            self.tour.tick(is_stuck=self.status.is_stuck)
        
        self.last_pose_x = self.status.position_x
        self.last_pose_y = self.status.position_y
        
        # Publish status to ROS2 topic (for other nodes)
        status_msg = String()
        status_msg.data = json.dumps(self.status.to_dict())
        self.status_pub.publish(status_msg)
    
    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send navigation goal"""
        if self.status.emergency_stop_active:
            self.status.error_message = "Emergency stop is active"
            return False
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.status.error_message = "Navigation server not available"
            return False
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.status.current_goal_x = x
        self.status.current_goal_y = y
        self.status.nav_state = NavState.NAVIGATING.value
        self.status.error_message = ""
        
        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(self.nav_goal_response_callback)
        
        return True
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status.nav_state = NavState.FAILED.value
            self.status.error_message = "Navigation goal rejected"
            self.get_logger().warn('Navigation goal rejected')
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Navigation goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Could add distance remaining, ETA, etc.
        pass
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status.nav_state = NavState.REACHED_GOAL.value
            self.status.current_goal_x = None
            self.status.current_goal_y = None
            self.get_logger().info('Navigation goal reached!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.status.nav_state = NavState.CANCELLED.value
            self.get_logger().info('Navigation cancelled')
        else:
            self.status.nav_state = NavState.FAILED.value
            self.status.error_message = f"Navigation failed with status {status}"
            self.get_logger().warn(f'Navigation failed: status={status}')
        
        self.current_goal_handle = None
    
    def start_patrol(self, waypoints: Optional[List[Dict]] = None, loop: bool = True) -> bool:
        """Start waypoint patrol using FollowWaypoints action"""
        if waypoints:
            self.waypoints = waypoints
            self.save_waypoints()
        
        if not self.waypoints:
            self.status.error_message = "No waypoints to patrol"
            return False
        
        if not self.waypoint_client.wait_for_server(timeout_sec=2.0):
            self.status.error_message = "Waypoint follower server not available"
            return False
        
        self.patrol_loop = loop
        self.status.current_waypoint_index = 0
        self.status.total_waypoints = len(self.waypoints)
        self.status.nav_state = NavState.PATROLLING.value
        
        # Build waypoint list
        poses = []
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp.get('x', 0.0)
            pose.pose.position.y = wp.get('y', 0.0)
            yaw = wp.get('yaw', 0.0)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        
        self.get_logger().info(f'Starting patrol with {len(poses)} waypoints, loop={loop}')
        
        future = self.waypoint_client.send_goal_async(goal)
        future.add_done_callback(self.patrol_goal_response_callback)
        
        return True
    
    def patrol_goal_response_callback(self, future):
        """Handle patrol goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status.nav_state = NavState.FAILED.value
            self.status.error_message = "Patrol goal rejected"
            return
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.patrol_result_callback)
    
    def patrol_result_callback(self, future):
        """Handle patrol completion - restart if loop mode"""
        result = future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Patrol completed')
            if self.patrol_loop and self.status.nav_state == NavState.PATROLLING.value:
                self.get_logger().info('Restarting patrol (loop mode)')
                self.start_patrol(loop=True)
            else:
                self.status.nav_state = NavState.IDLE.value
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.status.nav_state = NavState.CANCELLED.value
        else:
            self.status.nav_state = NavState.FAILED.value
        
        self.current_goal_handle = None
    
    def stop_patrol(self) -> bool:
        """Stop current patrol"""
        return self.cancel_navigation()
    
    def cancel_navigation(self) -> bool:
        """Cancel any active navigation goal"""
        if self.current_goal_handle:
            self.get_logger().info('Cancelling navigation...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            self.status.nav_state = NavState.CANCELLED.value
            return True
        return False
    
    def set_emergency_stop(self, active: bool = True):
        """Set the base controller's latched emergency-stop state."""
        self.status.emergency_stop_active = active
        if active:
            self.cancel_navigation()
            self.status.nav_state = NavState.IDLE.value

        self.estop_pub.publish(Bool(data=active))
        message = 'executed' if active else 'cleared'
        self.get_logger().warn(f'EMERGENCY STOP {message}')
    
    def send_exhibit_command(self, action: str, exhibit: str = None, exhibits: list = None):
        """Send a command to the exhibit navigator via /exhibit_command topic"""
        cmd = {'action': action}
        if exhibit:
            cmd['exhibit'] = exhibit
        if exhibits:
            cmd['exhibits'] = exhibits
        msg = String()
        msg.data = json.dumps(cmd)
        self.exhibit_cmd_pub.publish(msg)
        self.get_logger().info(f'Exhibit command: {cmd}')
    
    def get_status(self) -> Dict[str, Any]:
        """Get current robot status"""
        return self.status.to_dict()


# Global node reference for Flask routes
ros_node: Optional[APIBridgeNode] = None


def create_flask_app() -> Flask:
    """Create and configure Flask application"""
    app = Flask(__name__)
    CORS(app)  # Allow cross-origin requests from Pi
    
    @app.route('/status', methods=['GET'])
    def get_status():
        """Get robot status"""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        return jsonify(ros_node.get_status())
    
    @app.route('/navigate', methods=['POST'])
    def navigate():
        """Navigate to a single pose
        
        Body: {"x": 1.0, "y": 2.0, "yaw": 0.0}
        """
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        
        data = request.get_json()
        if not data:
            return jsonify({'error': 'No JSON data provided'}), 400
        
        x = data.get('x')
        y = data.get('y')
        yaw = data.get('yaw', 0.0)
        
        if x is None or y is None:
            return jsonify({'error': 'x and y are required'}), 400
        
        success = ros_node.navigate_to_pose(float(x), float(y), float(yaw))
        return jsonify({'success': success, 'message': ros_node.status.error_message or 'Navigation started'})
    
    @app.route('/exhibit/list', methods=['GET'])
    def exhibit_list():
        """List named waypoints from the active online editor set."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.list_operator_waypoints())
        except RuntimeError as error:
            return jsonify({'error': str(error)}), 503

    @app.route('/exhibit/status', methods=['GET'])
    def exhibit_status():
        """Return the live map-aware SahaBot tour status."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        return jsonify(ros_node.tour_status())

    @app.route('/exhibit/goto', methods=['POST'])
    def exhibit_goto():
        """Navigate to a named waypoint in the active online editor set.
        
        Body: {"exhibit": "waypoint_1"}
        """
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        
        data = request.get_json()
        if not data:
            return jsonify({'error': 'No JSON data provided'}), 400
        
        exhibit = data.get('exhibit')
        if not isinstance(exhibit, str) or not exhibit.strip():
            return jsonify({'error': 'exhibit name is required'}), 400
        try:
            result = ros_node.operator_patrol_command(
                PatrolCommand.Request.NAVIGATE,
                exhibit.strip(),
                touring=False,
            )
            return jsonify({
                **result,
                'action': 'goto',
                'exhibit': exhibit.strip(),
            })
        except ValueError as error:
            return jsonify({'success': False, 'error': str(error)}), 404
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/next', methods=['POST'])
    def exhibit_next():
        """Navigate to the next configured exhibit without wrapping."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.navigate_next())
        except ValueError as error:
            return jsonify({'success': False, 'error': str(error)}), 404
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/dock', methods=['POST'])
    def exhibit_dock():
        """Return to the map dock after explicit operator confirmation."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        data = request.get_json(silent=True) or {}
        if data.get('confirm') is not True:
            return jsonify({
                'success': False,
                'error': 'Operator confirmation is required',
            }), 400
        try:
            return jsonify(ros_node.operator_patrol_command(
                PatrolCommand.Request.NAVIGATE,
                DOCK_WAYPOINT_ID,
                touring=False,
            ))
        except ValueError as error:
            return jsonify({'success': False, 'error': str(error)}), 404
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/stop', methods=['POST'])
    def exhibit_stop():
        """Cancel navigation through the operator safety boundary."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.operator_patrol_command(
                PatrolCommand.Request.STOP
            ))
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/pause', methods=['POST'])
    def exhibit_pause():
        """Pause navigation through the operator safety boundary."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.operator_patrol_command(
                PatrolCommand.Request.PAUSE
            ))
        except ValueError as error:
            return jsonify({'success': False, 'error': str(error)}), 409
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/resume', methods=['POST'])
    def exhibit_resume():
        """Resume only the paused exhibit destination."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.resume_tour_navigation())
        except ValueError as error:
            return jsonify({'success': False, 'error': str(error)}), 409
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/start_tour', methods=['POST'])
    def exhibit_start_tour():
        """Start at the first available exhibit, then wait for Next."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.navigate_next(start_tour=True))
        except ValueError as error:
            return jsonify({'success': False, 'error': str(error)}), 404
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503
    
    @app.route('/waypoints', methods=['GET'])
    def get_waypoints():
        """Get current waypoint list"""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        return jsonify({'waypoints': ros_node.waypoints})
    
    @app.route('/waypoints', methods=['POST'])
    def set_waypoints():
        """Set waypoint list
        
        Body: {"waypoints": [{"name": "A", "x": 1.0, "y": 2.0, "yaw": 0.0}, ...]}
        """
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        
        data = request.get_json()
        if not data or 'waypoints' not in data:
            return jsonify({'error': 'waypoints array required'}), 400
        
        ros_node.waypoints = data['waypoints']
        ros_node.save_waypoints()
        ros_node.status.total_waypoints = len(ros_node.waypoints)
        
        return jsonify({'success': True, 'count': len(ros_node.waypoints)})
    
    @app.route('/patrol/start', methods=['POST'])
    def start_patrol():
        """Start patrol
        
        Body (optional): {"waypoints": [...], "loop": true}
        """
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        
        data = request.get_json() or {}
        waypoints = data.get('waypoints')
        loop = data.get('loop', True)
        
        success = ros_node.start_patrol(waypoints, loop)
        return jsonify({'success': success, 'message': ros_node.status.error_message or 'Patrol started'})
    
    @app.route('/patrol/stop', methods=['POST'])
    def stop_patrol():
        """Stop patrol"""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        
        success = ros_node.stop_patrol()
        return jsonify({'success': success})
    
    @app.route('/cancel', methods=['POST'])
    def cancel():
        """Cancel current navigation"""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        
        success = ros_node.cancel_navigation()
        return jsonify({'success': success})
    
    @app.route('/emergency_stop', methods=['POST'])
    def emergency_stop():
        """Set or clear the latched emergency stop.

        An empty request activates it. To resume, send ``{"active": false}``.
        """
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503

        data = request.get_json(silent=True) or {}
        active = data.get('active', True)
        if not isinstance(active, bool):
            return jsonify({'error': 'active must be a boolean'}), 400

        ros_node.set_emergency_stop(active)
        action = 'executed' if active else 'cleared'
        return jsonify({
            'success': True,
            'active': active,
            'message': f'Emergency stop {action}',
        })
    
    @app.route('/health', methods=['GET'])
    def health():
        """Health check endpoint"""
        return jsonify({'status': 'ok', 'ros_node': ros_node is not None})
    
    return app


_flask_stop = False

def run_flask(app: Flask, host: str, port: int):
    """Run Flask with clean shutdown support"""
    from werkzeug.serving import make_server
    global _flask_stop
    server = make_server(host, port, app, threaded=True)
    server.timeout = 1.0
    ctx = app.app_context()
    ctx.push()
    while not _flask_stop:
        server.handle_request()


def _handle_sigint(sig, frame):
    global _flask_stop
    _flask_stop = True
    try:
        rclpy.shutdown()
    except Exception:
        pass


def main(args=None):
    global ros_node
    
    rclpy.init(args=args)
    
    # Create ROS node
    ros_node = APIBridgeNode()
    
    # Create Flask app
    app = create_flask_app()
    
    # Get host/port from parameters or environment
    host = os.environ.get('API_HOST', '127.0.0.1')
    port = int(os.environ.get('API_PORT', '5000'))

    ros_node.get_logger().info(f'Starting API server on http://{host}:{port}')

    # Handle SIGINT for clean shutdown
    signal.signal(signal.SIGINT, _handle_sigint)

    # Start Flask in daemon thread
    flask_thread = threading.Thread(
        target=run_flask, args=(app, host, port), daemon=True)
    flask_thread.start()
    
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        _flask_stop = True
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
