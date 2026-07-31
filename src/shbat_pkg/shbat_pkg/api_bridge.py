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
from sahabat_interfaces.srv import ControlLease, GetWaypoints, PatrolCommand

from flask import Flask, request, jsonify
from flask_cors import CORS
import signal
import threading
import math
import json
import yaml
import os
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
        self.operator_active_map = ''
        self.operator_command_lock = threading.Lock()

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
        if msg.navigation_state:
            self.status.nav_state = msg.navigation_state

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

    def list_operator_waypoints(self):
        """Return enabled named waypoints from the active editor set."""
        map_id, response = self.get_operator_waypoints()
        return {
            'map_id': map_id,
            'set_id': response.set_id,
            'waypoints': [
                {
                    'id': waypoint.id,
                    'name': waypoint.name,
                    'enabled': waypoint.enabled,
                }
                for waypoint in response.waypoints
                if waypoint.enabled
            ],
        }

    def operator_patrol_command(
        self, command: int, waypoint_name: str = ''
    ):
        """Send a named command through operator_backend's lease boundary."""
        with self.operator_command_lock:
            set_id = ''
            waypoint_id = ''
            if command == PatrolCommand.Request.NAVIGATE:
                _map_id, waypoints = self.get_operator_waypoints()
                selected = next(
                    (
                        waypoint for waypoint in waypoints.waypoints
                        if waypoint.enabled
                        and waypoint_name in (waypoint.name, waypoint.id)
                    ),
                    None,
                )
                if selected is None:
                    available = ', '.join(
                        waypoint.name for waypoint in waypoints.waypoints
                        if waypoint.enabled
                    )
                    raise ValueError(
                        f'Waypoint {waypoint_name!r} not found in active set '
                        f'{waypoints.set_id!r}; available: {available or "none"}'
                    )
                set_id = waypoints.set_id
                waypoint_id = selected.id

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
                    raise RuntimeError(
                        result.message or 'Operator command rejected'
                    )
                return {'success': True, 'message': result.message}
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
        """Return the status shape expected by Ainmimdal's May client."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        status = ros_node.get_status()
        return jsonify({
            'state': status['nav_state'],
            'current_exhibit': '',
            'target_exhibit': '',
            'is_touring': status['nav_state'] == NavState.PATROLLING.value,
            'tour_progress': (
                f"{status['current_waypoint_index']}/"
                f"{status['total_waypoints']}"
            ),
            'robot_position': {
                'x': status['position_x'],
                'y': status['position_y'],
                'yaw': status['orientation_yaw'],
            },
            'battery_level': status['battery_percentage'],
            'is_obstructed': status['is_stuck'],
            'emergency_stop_active': status['emergency_stop_active'],
            'error_message': status['error_message'],
        })

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
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/resume', methods=['POST'])
    def exhibit_resume():
        """Resume the active set through the operator safety boundary."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.operator_patrol_command(
                PatrolCommand.Request.RESUME
            ))
        except RuntimeError as error:
            return jsonify({'success': False, 'error': str(error)}), 503

    @app.route('/exhibit/start_tour', methods=['POST'])
    def exhibit_start_tour():
        """Start the active editor waypoint set once."""
        if ros_node is None:
            return jsonify({'error': 'ROS node not initialized'}), 503
        try:
            return jsonify(ros_node.operator_patrol_command(
                PatrolCommand.Request.START
            ))
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
    host = os.environ.get('API_HOST', '0.0.0.0')
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
