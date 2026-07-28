#!/usr/bin/env python3
"""Central authority for remote Sahabat operation and persistent state."""

import json
import math
from pathlib import Path
import re
import shutil
import threading
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.srv import LoadMap as Nav2LoadMap
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sahabat_interfaces.msg import MapInfo, OperatorStatus, RouteSegment
from sahabat_interfaces.msg import TeleopCommand
from sahabat_interfaces.msg import Waypoint
from sahabat_interfaces.msg import WaypointSetInfo
from sahabat_interfaces.srv import ControlLease, GetWaypointGraph
from sahabat_interfaces.srv import GetWaypoints, ListMaps
from sahabat_interfaces.srv import ListWaypointSets, ManageWaypointSet
from sahabat_interfaces.srv import LocalizationRecovery
from sahabat_interfaces.srv import LoadMap, PatrolCommand, SaveMap
from sahabat_interfaces.srv import SaveDock, SaveWaypointGraph, SaveWaypoints
from sahabat_interfaces.srv import SetEmergencyStop, SetMode
from sensor_msgs.msg import BatteryState, Joy, LaserScan
from slam_toolbox.srv import SaveMap as SlamSaveMap
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .waypoint_store import WaypointStore


VALID_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
MAP_DOCK_WAYPOINT_ID = '__map_dock__'


class OperatorBackend(Node):
    """Own operator leases, safety state, maps, waypoints, and Nav2 actions."""

    def __init__(self) -> None:
        """Initialize the fail-safe operator boundary."""
        super().__init__('operator_backend')
        self.group = ReentrantCallbackGroup()
        self.declare_parameter('maps_directory', '~/sahabat_ws/maps')
        self.declare_parameter('lease_timeout', 5.0)
        self.declare_parameter('teleop_timeout', 0.25)
        self.declare_parameter('max_linear_speed', 0.50)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('active_map', '')
        self.declare_parameter('localization_backend', 'amcl')

        self.maps_directory = Path(
            str(self.get_parameter('maps_directory').value)
        ).expanduser()
        self.maps_directory.mkdir(parents=True, exist_ok=True)
        self.lease_timeout = float(
            self.get_parameter('lease_timeout').value
        )
        self.teleop_timeout = float(
            self.get_parameter('teleop_timeout').value
        )
        self.max_linear = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular = float(
            self.get_parameter('max_angular_speed').value
        )
        self.localization_backend = str(
            self.get_parameter('localization_backend').value
        )

        self.lease_id = ''
        self.lease_owner = ''
        self.lease_deadline = 0.0
        self.estop_active = False
        self.remote_active = False
        self.last_remote_command = 0.0
        self.last_sequence = None
        self.mode = OperatorStatus.MODE_IDLE
        self.active_map = str(self.get_parameter('active_map').value)
        self.navigation_state = 'idle'
        self.active_operation = ''
        self.localization_recovery_active = False
        self.localization_recovery_status = 'Not running'
        self.motor_enabled = False
        self.battery_percentage = math.nan
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.current_goal_handle = None
        self.cancel_requested = False
        self.last_map = 0.0
        self.last_scan = 0.0
        self.last_amcl_pose = 0.0
        self.localization_covariance_good = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.estop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            transient,
        )
        self.remote_pub = self.create_publisher(
            Twist,
            '/cmd_vel_remote',
            10,
        )
        self.remote_active_pub = self.create_publisher(
            Bool,
            '/operator/remote_active',
            10,
        )
        self.status_pub = self.create_publisher(
            OperatorStatus,
            '/operator/status',
            transient,
        )
        self.marker_refresh_pub = self.create_publisher(
            String,
            '/operator/waypoints_changed',
            transient,
        )
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            transient,
        )

        self.create_subscription(
            TeleopCommand,
            '/operator/teleop_command',
            self._teleop,
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            Joy,
            '/operator/foxglove_joy',
            self._foxglove_joy,
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            Bool,
            '/emergency_stop',
            self._external_estop,
            transient,
            callback_group=self.group,
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom,
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            Bool,
            '/motor_enabled',
            lambda message: setattr(
                self, 'motor_enabled', bool(message.data)
            ),
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self._battery,
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            lambda _message: setattr(self, 'last_map', self._now()),
            1,
            callback_group=self.group,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            lambda _message: setattr(self, 'last_scan', self._now()),
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose,
            10,
            callback_group=self.group,
        )
        self.create_subscription(
            String,
            '/operator/mode_state',
            self._mode_state,
            transient,
            callback_group=self.group,
        )
        self.create_subscription(
            Bool,
            '/localization/recovery_active',
            self._recovery_active,
            transient,
            callback_group=self.group,
        )
        self.create_subscription(
            String,
            '/localization/recovery_status',
            self._recovery_status,
            transient,
            callback_group=self.group,
        )

        self.slam_save = self.create_client(
            SlamSaveMap,
            '/slam_toolbox/save_map',
            callback_group=self.group,
        )
        self.slam_serialize = self.create_client(
            SerializePoseGraph,
            '/slam_toolbox/serialize_map',
            callback_group=self.group,
        )
        self.map_loader = self.create_client(
            Nav2LoadMap,
            '/map_server/load_map',
            callback_group=self.group,
        )
        self.mode_client = self.create_client(
            SetMode,
            '/operator/internal/set_mode',
            callback_group=self.group,
        )
        self.recovery_start_client = self.create_client(
            Trigger,
            '/localization/start_recovery',
            callback_group=self.group,
        )
        self.recovery_stop_client = self.create_client(
            Trigger,
            '/localization/stop_recovery',
            callback_group=self.group,
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.group,
        )
        self.patrol_client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints',
            callback_group=self.group,
        )

        self.create_service(
            ControlLease,
            '/operator/control_lease',
            self._control_lease,
            callback_group=self.group,
        )
        self.create_service(
            SetEmergencyStop,
            '/operator/set_emergency_stop',
            self._set_estop_service,
            callback_group=self.group,
        )
        self.create_service(
            ListMaps,
            '/operator/maps/list',
            self._list_maps_service,
            callback_group=self.group,
        )
        self.create_service(
            SaveMap,
            '/operator/maps/save',
            self._save_map_service,
            callback_group=self.group,
        )
        self.create_service(
            LoadMap,
            '/operator/maps/load',
            self._load_map_service,
            callback_group=self.group,
        )
        self.create_service(
            ListWaypointSets,
            '/operator/waypoint_sets/list',
            self._list_waypoint_sets_service,
            callback_group=self.group,
        )
        self.create_service(
            ManageWaypointSet,
            '/operator/waypoint_sets/manage',
            self._manage_waypoint_set_service,
            callback_group=self.group,
        )
        self.create_service(
            GetWaypoints,
            '/operator/waypoints/get',
            self._get_waypoints_service,
            callback_group=self.group,
        )
        self.create_service(
            SaveWaypoints,
            '/operator/waypoints/save',
            self._save_waypoints_service,
            callback_group=self.group,
        )
        self.create_service(
            GetWaypointGraph,
            '/operator/waypoint_graph/get',
            self._get_waypoint_graph_service,
            callback_group=self.group,
        )
        self.create_service(
            SaveWaypointGraph,
            '/operator/waypoint_graph/save',
            self._save_waypoint_graph_service,
            callback_group=self.group,
        )
        self.create_service(
            SaveDock,
            '/operator/dock/save',
            self._save_dock_service,
            callback_group=self.group,
        )
        self.create_service(
            PatrolCommand,
            '/operator/patrol',
            self._patrol_service,
            callback_group=self.group,
        )
        self.create_service(
            SetMode,
            '/operator/set_mode',
            self._set_mode_service,
            callback_group=self.group,
        )
        self.create_service(
            LocalizationRecovery,
            '/operator/localization/recovery',
            self._localization_recovery_service,
            callback_group=self.group,
        )

        self.create_timer(0.05, self._safety_timer)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(2.0, self._publish_waypoint_markers)
        self.get_logger().info('Operator backend started')

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _lease_valid(self, lease_id: str) -> bool:
        return all((
            lease_id,
            lease_id == self.lease_id,
            self._now() < self.lease_deadline,
        ))

    @staticmethod
    def _call_with_timeout(client, request, timeout: float):
        """Wait for an async ROS call while another executor thread serves it."""
        future = client.call_async(request)
        complete = threading.Event()
        future.add_done_callback(lambda _future: complete.set())
        if not complete.wait(timeout):
            future.cancel()
            return None
        return future.result()

    def _control_lease(self, request, response):
        now = self._now()
        if request.action == ControlLease.Request.ACQUIRE:
            if self.lease_id and now < self.lease_deadline:
                response.message = f'Control held by {self.lease_owner}'
            elif not request.client_id.strip():
                response.message = 'client_id is required'
            else:
                self.lease_id = uuid.uuid4().hex
                self.lease_owner = request.client_id.strip()[:64]
                self.lease_deadline = now + self.lease_timeout
                response.success = True
                response.message = 'Control acquired'
        elif request.action == ControlLease.Request.RENEW:
            if self._lease_valid(request.lease_id):
                self.lease_deadline = now + self.lease_timeout
                response.success = True
                response.message = 'Control renewed'
            else:
                response.message = 'Lease is invalid or expired'
        elif request.action == ControlLease.Request.RELEASE:
            if self._lease_valid(request.lease_id):
                self._stop_remote()
                self.lease_id = ''
                self.lease_owner = ''
                self.lease_deadline = 0.0
                response.success = True
                response.message = 'Control released'
            else:
                response.message = 'Lease is invalid or expired'
        else:
            response.message = 'Unknown lease action'
        response.lease_id = self.lease_id if response.success else ''
        response.expires_in = max(0.0, self.lease_deadline - now)
        return response

    def _teleop(self, message: TeleopCommand) -> None:
        if not self._lease_valid(message.lease_id) or self.estop_active:
            self._stop_remote()
            return
        if self.last_sequence is not None and message.sequence <= self.last_sequence:
            return
        self.last_sequence = message.sequence
        self.last_remote_command = self._now()
        if not message.deadman:
            self._stop_remote()
            return

        command = Twist()
        command.linear.x = max(
            -self.max_linear,
            min(self.max_linear, message.twist.linear.x),
        )
        command.angular.z = max(
            -self.max_angular,
            min(self.max_angular, message.twist.angular.z),
        )
        self.remote_active = True
        self.remote_active_pub.publish(Bool(data=True))
        self.remote_pub.publish(command)

    def _foxglove_joy(self, message: Joy) -> None:
        """Accept Foxglove teleop through a standard, bridge-known type."""
        if (
            not self._lease_valid(message.header.frame_id)
            or self.estop_active
        ):
            self._stop_remote()
            return
        if len(message.axes) < 2 or not message.buttons:
            self._stop_remote()
            self.get_logger().warning('Invalid Foxglove teleop message')
            return

        linear = float(message.axes[0])
        angular = float(message.axes[1])
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._stop_remote()
            self.get_logger().warning('Non-finite Foxglove teleop command')
            return

        self.last_remote_command = self._now()
        if not bool(message.buttons[0]):
            self._stop_remote()
            return

        command = Twist()
        command.linear.x = max(
            -self.max_linear,
            min(self.max_linear, linear),
        )
        command.angular.z = max(
            -self.max_angular,
            min(self.max_angular, angular),
        )
        self.remote_active = True
        self.remote_active_pub.publish(Bool(data=True))
        self.remote_pub.publish(command)

    def _stop_remote(self) -> None:
        self.remote_active = False
        self.remote_active_pub.publish(Bool(data=False))
        self.remote_pub.publish(Twist())

    def _latch_estop(self, reason: str) -> None:
        self.estop_active = True
        self._stop_remote()
        self.estop_pub.publish(Bool(data=True))
        self.get_logger().warn(f'Software E-stop active: {reason}')

    def _external_estop(self, message: Bool) -> None:
        """Accept stop requests, but never raw clear requests."""
        if message.data and not self.estop_active:
            self._latch_estop('External safety request')

    def _set_estop_service(self, request, response):
        if request.active:
            self._latch_estop('Operator request')
            response.success = True
        elif not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required to clear'
        elif request.confirmation != 'CLEAR':
            response.message = 'confirmation must be CLEAR'
        elif abs(self.linear_velocity) > 0.01 or abs(self.angular_velocity) > 0.02:
            response.message = 'Robot must be stationary before clearing'
        else:
            self.estop_active = False
            self.estop_pub.publish(Bool(data=False))
            response.success = True
            response.message = 'Software E-stop cleared'
        response.active = self.estop_active
        if not response.message:
            response.message = 'Software E-stop activated'
        return response

    def _safety_timer(self) -> None:
        now = self._now()
        if self.lease_id and now >= self.lease_deadline:
            self.lease_id = ''
            self.lease_owner = ''
            self.lease_deadline = 0.0
            self._stop_remote()
            self.get_logger().warning('Control lease expired; remote stopped')
        if self.remote_active and now - self.last_remote_command > self.teleop_timeout:
            self._stop_remote()
            self.get_logger().warning('Remote command timed out; remote stopped')

    def _odom(self, message: Odometry) -> None:
        self.linear_velocity = message.twist.twist.linear.x
        self.angular_velocity = message.twist.twist.angular.z
        self.pose_x = message.pose.pose.position.x
        self.pose_y = message.pose.pose.position.y
        q = message.pose.pose.orientation
        self.pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _status_pose(self):
        """Return the robot pose in map when localization provides that TF."""
        odom_pose = ('odom', self.pose_x, self.pose_y, self.pose_yaw)
        if self.mode == OperatorStatus.MODE_IDLE:
            return odom_pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            ).transform
        except Exception:
            return odom_pose
        q = transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (
            'map',
            transform.translation.x,
            transform.translation.y,
            yaw,
        )

    def _battery(self, message: BatteryState) -> None:
        value = float(message.percentage)
        self.battery_percentage = value * 100.0 if value <= 1.0 else value

    def _amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        covariance = message.pose.covariance
        variances = (covariance[0], covariance[7], covariance[35])
        self.last_amcl_pose = self._now()
        self.localization_covariance_good = all(
            math.isfinite(value) and value <= 0.20
            for value in variances
        )

    def _mode_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            names = {
                'idle': OperatorStatus.MODE_IDLE,
                'mapping': OperatorStatus.MODE_MAPPING,
                'localization': OperatorStatus.MODE_LOCALIZATION,
                'operations': OperatorStatus.MODE_OPERATIONS,
            }
            next_mode = names.get(
                state.get('mode'), OperatorStatus.MODE_IDLE
            )
            if next_mode != self.mode:
                self.last_amcl_pose = 0.0
                self.localization_covariance_good = False
            self.mode = next_mode
            self.active_map = state.get('map_id', '')
        except (TypeError, ValueError):
            self.get_logger().warning('Ignored invalid mode-state payload')

    def _recovery_active(self, message: Bool) -> None:
        self.localization_recovery_active = bool(message.data)
        if message.data:
            self._stop_remote()
            self.active_operation = 'localization recovery'
        elif self.active_operation == 'localization recovery':
            self.active_operation = ''

    def _recovery_status(self, message: String) -> None:
        self.localization_recovery_status = message.data

    def _publish_status(self) -> None:
        message = OperatorStatus()
        pose_frame, pose_x, pose_y, pose_yaw = self._status_pose()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = pose_frame
        message.mode = self.mode
        message.active_map = self.active_map
        message.navigation_state = self.navigation_state
        message.emergency_stop = self.estop_active
        message.motor_enabled = self.motor_enabled
        message.control_owner = self.lease_owner
        message.lease_expires_in = max(0.0, self.lease_deadline - self._now())
        message.battery_percentage = float(self.battery_percentage)
        message.pose.x = pose_x
        message.pose.y = pose_y
        message.pose.theta = pose_yaw
        message.linear_velocity = self.linear_velocity
        message.angular_velocity = self.angular_velocity
        message.map_healthy, message.scan_healthy, message.tf_healthy = (
            self._health()
        )
        if self.localization_backend == 'slam_toolbox':
            localization_pose_ok = pose_frame == 'map'
        else:
            localization_pose_ok = (
                self.last_amcl_pose > 0.0
                and self.localization_covariance_good
            )
        message.localization_healthy = all((
            message.map_healthy,
            message.scan_healthy,
            message.tf_healthy,
            localization_pose_ok,
        ))
        message.diagnostic_level = (
            OperatorStatus.DIAGNOSTIC_WARN
            if self.estop_active else OperatorStatus.DIAGNOSTIC_OK
        )
        message.diagnostic_message = (
            'Software E-stop active' if self.estop_active else 'Ready'
        )
        message.active_operation = self.active_operation
        message.localization_recovery_active = (
            self.localization_recovery_active
        )
        message.localization_recovery_status = (
            self.localization_recovery_status
        )
        self.status_pub.publish(message)

    def _localization_recovery_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response

        if request.action == LocalizationRecovery.Request.STOP:
            client = self.recovery_stop_client
        elif request.action == LocalizationRecovery.Request.START:
            if self.mode not in (
                OperatorStatus.MODE_LOCALIZATION,
                OperatorStatus.MODE_OPERATIONS,
            ):
                response.message = 'Load a map in Operate mode first'
                return response
            if self.estop_active:
                response.message = 'Clear E-stop before localization recovery'
                return response
            if (
                abs(self.linear_velocity) > 0.01
                or abs(self.angular_velocity) > 0.02
            ):
                response.message = 'Robot must be stationary before recovery'
                return response
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
                self.navigation_state = 'idle'
            self._stop_remote()
            client = self.recovery_start_client
        else:
            response.message = 'Unknown localization recovery action'
            return response

        if not client.wait_for_service(timeout_sec=1.0):
            response.message = 'Localization recovery is unavailable'
            return response
        result = self._call_with_timeout(client, Trigger.Request(), 3.0)
        if result is None:
            response.message = 'Localization recovery request timed out'
            return response
        response.success = result.success
        response.message = result.message
        return response

    def _health(self):
        now = self._now()
        map_healthy = now - self.last_map < 3.0
        scan_healthy = now - self.last_scan < 1.0
        try:
            tf_healthy = bool(self.tf_buffer.can_transform(
                'map', 'base_link', rclpy.time.Time()
            ))
        except Exception:
            tf_healthy = False
        return map_healthy, scan_healthy, tf_healthy

    def _wait_for_health(self, timeout: float) -> bool:
        deadline = self._now() + timeout
        while self._now() < deadline:
            map_ok, scan_ok, tf_ok = self._health()
            if map_ok and scan_ok and tf_ok:
                return True
            time.sleep(0.1)
        return False

    def _map_directory(self, map_id: str) -> Path:
        if not VALID_ID.fullmatch(map_id):
            raise ValueError('Map ID must use letters, numbers, _ or -')
        return self.maps_directory / map_id

    def _map_yaml_path(self, map_id: str) -> Path:
        if not VALID_ID.fullmatch(map_id):
            raise ValueError('Map ID must use letters, numbers, _ or -')
        yaml_path = self.maps_directory / f'{map_id}.yaml'
        if yaml_path.exists():
            return yaml_path
        return self.maps_directory / map_id / 'map.yaml'

    def _remember_map(self, map_id: str) -> None:
        if not VALID_ID.fullmatch(map_id):
            return
        try:
            (self.maps_directory / 'last_selected_map').write_text(
                f'{map_id}\n', encoding='utf-8'
            )
        except OSError:
            pass

    def _map_info(self, path: Path) -> MapInfo:
        info = MapInfo()
        map_id = path.stem if path.is_file() else path.name
        info.map_id = map_id
        info.display_name = map_id
        metadata = path.with_suffix('.metadata.yaml') if path.is_file() else path / 'metadata.yaml'
        if metadata.exists():
            try:
                data = yaml.safe_load(metadata.read_text()) or {}
                info.display_name = str(data.get('display_name', map_id))
            except (OSError, yaml.YAMLError):
                pass
        info.directory = str(path.parent if path.is_file() else path)
        session_base = path.with_suffix('') if path.is_file() else path / 'session'
        info.has_editable_session = all((
            session_base.with_suffix('.posegraph').exists(),
            session_base.with_suffix('.data').exists(),
        ))
        stamp = path.stat().st_mtime
        info.modified_at.sec = int(stamp)
        info.modified_at.nanosec = int((stamp - int(stamp)) * 1_000_000_000)
        return info

    def _list_maps_service(self, _request, response):
        yaml_files = sorted(
            path for path in self.maps_directory.glob('*.yaml')
            if not path.name.startswith('.') and path.name != 'dock.yaml'
        )
        directories = sorted(
            path for path in self.maps_directory.iterdir()
            if all((
                path.is_dir(),
                not path.name.startswith('.'),
                (path / 'map.yaml').exists(),
            ))
        )
        seen = set()
        maps = []
        for path in yaml_files + directories:
            map_id = path.stem if path.is_file() else path.name
            if map_id in seen:
                continue
            seen.add(map_id)
            maps.append(self._map_info(path))
        response.maps = maps
        response.active_map = self.active_map
        response.message = f'{len(response.maps)} map(s) available'
        return response

    def _save_map_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        try:
            destination = self._map_directory(request.map_id)
        except ValueError as error:
            response.message = str(error)
            return response
        if destination.exists() and not request.overwrite:
            response.message = 'Map exists; confirm overwrite to archive it'
            return response
        if not self.slam_save.wait_for_service(timeout_sec=1.0):
            response.message = 'SLAM save service is unavailable in this mode'
            return response

        staging = self.maps_directory / f'.staging-{request.map_id}-{uuid.uuid4().hex}'
        staging.mkdir(parents=True)
        self.active_operation = f'saving map {request.map_id}'
        try:
            save_request = SlamSaveMap.Request()
            save_request.name.data = str(staging / 'map')
            result = self._call_with_timeout(
                self.slam_save, save_request, 20.0
            )
            if result is None or result.result != SlamSaveMap.Response.RESULT_SUCCESS:
                raise RuntimeError('SLAM failed to save navigation map')
            if request.save_editable_session:
                if not self.slam_serialize.wait_for_service(timeout_sec=1.0):
                    raise RuntimeError('SLAM session service is unavailable')
                session_request = SerializePoseGraph.Request()
                session_request.filename = str(staging / 'session')
                session = self._call_with_timeout(
                    self.slam_serialize,
                    session_request,
                    30.0,
                )
                if session is None or session.result != 0:
                    raise RuntimeError('SLAM failed to save editable session')
            metadata = {
                'map_id': request.map_id,
                'display_name': request.display_name or request.map_id,
                'created_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            }
            (staging / 'metadata.yaml').write_text(
                yaml.safe_dump(metadata, sort_keys=False)
            )
            if destination.exists():
                archive = self.maps_directory / '.archive'
                archive.mkdir(exist_ok=True)
                suffix = time.strftime('%Y%m%d-%H%M%S')
                shutil.move(str(destination), str(archive / f'{request.map_id}-{suffix}'))
            staging.rename(destination)
            response.success = True
            response.map = self._map_info(destination)
            response.message = f'Map saved to {destination}'
        except (OSError, RuntimeError) as error:
            response.message = str(error)
        finally:
            self.active_operation = ''
        return response

    def _load_map_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        try:
            yaml_path = self._map_yaml_path(request.map_id)
        except ValueError as error:
            response.message = str(error)
            return response
        if not yaml_path.exists():
            response.message = f'Map does not exist: {yaml_path}'
            return response
        self._stop_remote()
        if abs(self.linear_velocity) > 0.01 or abs(self.angular_velocity) > 0.02:
            response.message = 'Robot did not become stationary'
            return response
        if not self.map_loader.wait_for_service(timeout_sec=1.0):
            response.message = 'Map server unavailable; switch to localization first'
            return response
        load_request = Nav2LoadMap.Request()
        load_request.map_url = str(yaml_path)
        result = self._call_with_timeout(
            self.map_loader, load_request, 10.0
        )
        if result is None or result.result != Nav2LoadMap.Response.RESULT_SUCCESS:
            response.message = 'Nav2 rejected the selected map'
            return response
        self.active_map = request.map_id
        self._remember_map(request.map_id)
        response.success = True
        response.yaml_path = str(yaml_path)
        response.message = 'Map loaded; set initial pose before clearing E-stop'
        return response

    def _waypoint_store(self, map_id: str) -> WaypointStore:
        if not VALID_ID.fullmatch(map_id):
            raise ValueError('Invalid map id')
        return WaypointStore(
            self.maps_directory,
            sets_directory=self.maps_directory / 'waypoint_sets' / map_id,
            legacy_path=self.maps_directory / f'{map_id}_waypoints.yaml',
            dock_path=self.maps_directory / 'waypoint_sets' / map_id / 'dock.yaml',
        )

    def _read_waypoints(self, map_id: str, set_id: str = ''):
        revision, items = self._waypoint_store(map_id).read_set(set_id)
        return revision, [self._waypoint_message(item) for item in items]

    def _read_waypoint_graph(self, map_id: str, set_id: str = ''):
        revision, items, segments, _settings = self._waypoint_store(
            map_id
        ).read_graph(set_id)
        waypoint_messages = [self._waypoint_message(item) for item in items]
        segment_messages = []
        for item in segments:
            segment = RouteSegment()
            segment.id = str(item.get('id') or uuid.uuid4().hex)
            segment.name = str(item.get('name') or 'route')
            segment.from_waypoint_id = str(item.get('from_waypoint_id') or '')
            segment.to_waypoint_id = str(item.get('to_waypoint_id') or '')
            segment.bidirectional = bool(item.get('bidirectional', False))
            segment.enabled = bool(item.get('enabled', True))
            segment.via_points = [
                self._waypoint_message(waypoint)
                for waypoint in item.get('via_points', [])
            ]
            segment_messages.append(segment)
        return revision, waypoint_messages, segment_messages

    @staticmethod
    def _waypoint_message(item):
        waypoint = Waypoint()
        waypoint.id = str(item.get('id') or uuid.uuid4().hex)
        waypoint.name = str(item.get('name') or 'waypoint')
        waypoint.pose.x = float(item.get('x', 0.0))
        waypoint.pose.y = float(item.get('y', 0.0))
        waypoint.pose.theta = float(item.get('yaw', 0.0))
        waypoint.dwell_seconds = float(item.get('dwell_seconds', 0.0))
        waypoint.enabled = bool(item.get('enabled', True))
        return waypoint

    @staticmethod
    def _waypoint_data(item):
        return {
            'id': item.id or uuid.uuid4().hex,
            'name': item.name,
            'x': item.pose.x,
            'y': item.pose.y,
            'yaw': item.pose.theta,
            'dwell_seconds': item.dwell_seconds,
            'enabled': item.enabled,
        }

    @staticmethod
    def _segment_data(item):
        return {
            'id': item.id or uuid.uuid4().hex,
            'name': item.name,
            'from_waypoint_id': item.from_waypoint_id,
            'to_waypoint_id': item.to_waypoint_id,
            'bidirectional': item.bidirectional,
            'enabled': item.enabled,
            'via_points': [
                OperatorBackend._waypoint_data(waypoint)
                for waypoint in item.via_points
            ],
        }

    def _list_waypoint_sets_service(self, request, response):
        try:
            store = self._waypoint_store(request.map_id)
            response.active_set_id = store.active_set_id()
            for item in store.list_sets():
                info = WaypointSetInfo()
                info.id = item['id']
                info.name = item['name']
                info.revision = item['revision']
                info.waypoint_count = item['waypoint_count']
                response.sets.append(info)
            dock = store.load_dock()
            if dock:
                response.has_dock = True
                response.dock.id = str(dock.get('id') or 'dock')
                response.dock.name = 'dock'
                response.dock.pose.x = float(dock.get('x', 0.0))
                response.dock.pose.y = float(dock.get('y', 0.0))
                response.dock.pose.theta = float(dock.get('yaw', 0.0))
                response.dock.enabled = True
            response.message = f'{len(response.sets)} waypoint set(s) loaded'
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
        return response

    def _manage_waypoint_set_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        if (
            request.action in (
                ManageWaypointSet.Request.DELETE,
                ManageWaypointSet.Request.SELECT,
            )
            and self.navigation_state in ('navigating', 'patrolling')
        ):
            response.message = 'Stop navigation before changing waypoint sets'
            return response
        try:
            store = self._waypoint_store(request.map_id)
            if request.action == ManageWaypointSet.Request.CREATE:
                response.set_id = store.create_set(request.name)
            elif request.action == ManageWaypointSet.Request.RENAME:
                store.rename_set(request.set_id, request.name)
                response.set_id = request.set_id
            elif request.action == ManageWaypointSet.Request.DELETE:
                store.delete_set(request.set_id)
                response.set_id = request.set_id
            elif request.action == ManageWaypointSet.Request.SELECT:
                store.select_set(request.set_id)
                response.set_id = request.set_id
            else:
                response.message = 'Unknown waypoint-set action'
                return response
            response.active_set_id = store.active_set_id()
            response.success = True
            response.message = 'Waypoint set updated'
            self.marker_refresh_pub.publish(String(data=request.map_id))
            self._publish_waypoint_markers()
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
        return response

    def _get_waypoints_service(self, request, response):
        try:
            response.revision, response.waypoints = self._read_waypoints(
                request.map_id, request.set_id
            )
            response.set_id = (
                request.set_id
                or self._waypoint_store(request.map_id).active_set_id()
            )
            response.message = f'{len(response.waypoints)} waypoint(s) loaded'
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
        return response

    def _save_waypoints_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        try:
            store = self._waypoint_store(request.map_id)
            set_id = request.set_id or store.active_set_id()
            current, _waypoints = self._read_waypoints(request.map_id, set_id)
            if request.expected_revision != current:
                response.revision = current
                response.message = 'Waypoint set changed; reload before saving'
                return response
            revision = store.save_set(
                set_id,
                current,
                [self._waypoint_data(item) for item in request.waypoints],
            )
            response.success = True
            response.revision = revision
            response.set_id = set_id
            response.message = f'{len(request.waypoints)} waypoint(s) saved'
            self.marker_refresh_pub.publish(String(data=request.map_id))
            self._publish_waypoint_markers()
        except RuntimeError as error:
            if str(error).startswith('revision:'):
                response.revision = int(str(error).split(':', 1)[1])
                response.message = 'Waypoint set changed; reload before saving'
            else:
                response.message = str(error)
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
        return response

    def _get_waypoint_graph_service(self, request, response):
        try:
            response.revision, response.waypoints, response.segments = (
                self._read_waypoint_graph(request.map_id, request.set_id)
            )
            response.set_id = (
                request.set_id
                or self._waypoint_store(request.map_id).active_set_id()
            )
            response.message = (
                f'{len(response.waypoints)} waypoint(s), '
                f'{len(response.segments)} route segment(s) loaded'
            )
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
        return response

    def _save_waypoint_graph_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        try:
            store = self._waypoint_store(request.map_id)
            set_id = request.set_id or store.active_set_id()
            current, _waypoints, _segments = self._read_waypoint_graph(
                request.map_id, set_id
            )
            if request.expected_revision != current:
                response.revision = current
                response.message = 'Waypoint graph changed; reload before saving'
                return response
            revision = store.save_graph(
                set_id,
                current,
                [self._waypoint_data(item) for item in request.waypoints],
                [self._segment_data(item) for item in request.segments],
            )
            response.success = True
            response.revision = revision
            response.set_id = set_id
            response.message = (
                f'{len(request.waypoints)} waypoint(s), '
                f'{len(request.segments)} route segment(s) saved'
            )
            self.marker_refresh_pub.publish(String(data=request.map_id))
            self._publish_waypoint_markers()
        except RuntimeError as error:
            if str(error).startswith('revision:'):
                response.revision = int(str(error).split(':', 1)[1])
                response.message = 'Waypoint graph changed; reload before saving'
            else:
                response.message = str(error)
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
        return response

    def _save_dock_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        try:
            self._waypoint_store(request.map_id).save_dock(
                self._waypoint_data(request.dock)
            )
            response.success = True
            response.message = 'Dock saved for this map'
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
        return response

    def _publish_waypoint_markers(self) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if self.active_map:
            try:
                _revision, waypoints = self._read_waypoints(self.active_map)
                for index, waypoint in enumerate(waypoints):
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'sahabat_waypoints'
                    marker.id = index
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    marker.pose.position.x = waypoint.pose.x
                    marker.pose.position.y = waypoint.pose.y
                    marker.pose.orientation.z = math.sin(waypoint.pose.theta / 2.0)
                    marker.pose.orientation.w = math.cos(waypoint.pose.theta / 2.0)
                    marker.scale.x = 0.35
                    marker.scale.y = 0.08
                    marker.scale.z = 0.08
                    marker.color.r = 0.40
                    marker.color.g = 0.82
                    marker.color.b = 0.77
                    marker.color.a = 1.0 if waypoint.enabled else 0.3
                    markers.markers.append(marker)
            except (OSError, ValueError, yaml.YAMLError):
                pass
        self.waypoint_marker_pub.publish(markers)

    def _patrol_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        if self.estop_active and request.command not in (
            PatrolCommand.Request.PAUSE,
            PatrolCommand.Request.STOP,
        ):
            response.message = 'Clear E-stop before starting movement'
            return response
        if request.command in (
            PatrolCommand.Request.STOP,
            PatrolCommand.Request.PAUSE,
        ):
            self.cancel_requested = True
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.navigation_state = (
                'paused'
                if request.command == PatrolCommand.Request.PAUSE
                else 'idle'
            )
            response.success = True
            response.message = self.navigation_state
            return response
        try:
            store = self._waypoint_store(self.active_map)
            if (
                request.command == PatrolCommand.Request.NAVIGATE
                and request.waypoint_id == MAP_DOCK_WAYPOINT_ID
            ):
                dock_data = store.load_dock()
                if not dock_data:
                    response.message = 'This map has no dock pose'
                    return response
                dock = Waypoint()
                dock.id = MAP_DOCK_WAYPOINT_ID
                dock.name = 'dock'
                dock.pose.x = float(dock_data.get('x', 0.0))
                dock.pose.y = float(dock_data.get('y', 0.0))
                dock.pose.theta = float(dock_data.get('yaw', 0.0))
                dock.enabled = True
                waypoints = [dock]
            else:
                set_id = request.set_id or store.active_set_id()
                _revision, waypoints = self._read_waypoints(
                    self.active_map, set_id
                )
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.message = str(error)
            return response
        enabled = [item for item in waypoints if item.enabled]
        if request.command == PatrolCommand.Request.NAVIGATE:
            selected = next((item for item in enabled if item.id == request.waypoint_id), None)
            if selected is None:
                response.message = 'Waypoint not found'
                return response
            if not self.navigate_client.wait_for_server(timeout_sec=1.0):
                response.message = 'NavigateToPose action unavailable'
                return response
            self.cancel_requested = False
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_from_waypoint(selected)
            future = self.navigate_client.send_goal_async(goal)
            future.add_done_callback(self._goal_started)
            self.navigation_state = 'navigating'
        else:
            if not enabled:
                response.message = 'No enabled waypoints'
                return response
            if not self.patrol_client.wait_for_server(timeout_sec=1.0):
                response.message = 'FollowWaypoints action unavailable'
                return response
            self.cancel_requested = False
            goal = FollowWaypoints.Goal()
            goal.poses = [self._pose_from_waypoint(item) for item in enabled]
            future = self.patrol_client.send_goal_async(goal)
            future.add_done_callback(self._goal_started)
            self.navigation_state = 'patrolling'
        response.success = True
        response.message = self.navigation_state
        return response

    def _pose_from_waypoint(self, waypoint: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.pose.x
        pose.pose.position.y = waypoint.pose.y
        pose.pose.orientation.z = math.sin(waypoint.pose.theta / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.pose.theta / 2.0)
        return pose

    def _goal_started(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.navigation_state = 'rejected'
                return
            if self.cancel_requested or self.navigation_state not in (
                'navigating',
                'patrolling',
            ):
                handle.cancel_goal_async()
                return
            self.current_goal_handle = handle
            result = handle.get_result_async()
            result.add_done_callback(self._goal_finished)
        except Exception as error:
            self.navigation_state = f'failed: {error}'

    def _goal_finished(self, future) -> None:
        try:
            result = future.result()
            self.navigation_state = 'complete' if result.status == 4 else 'idle'
        except Exception as error:
            self.navigation_state = f'failed: {error}'
        self.current_goal_handle = None

    def _set_mode_service(self, request, response):
        if not self._lease_valid(request.lease_id):
            response.message = 'A valid control lease is required'
            return response
        if request.mode not in ('idle', 'mapping', 'localization', 'operations'):
            response.message = 'Mode must be idle, mapping, localization, or operations'
            return response
        if request.mode in ('localization', 'operations'):
            try:
                yaml_path = self._map_yaml_path(request.map_id)
            except ValueError as error:
                response.message = str(error)
                return response
            if not yaml_path.exists():
                response.message = 'Selected map does not exist'
                return response
        self._stop_remote()
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            response.message = 'Mode manager is unavailable'
            return response
        internal = SetMode.Request()
        internal.mode = request.mode
        internal.map_id = request.map_id
        result = self._call_with_timeout(
            self.mode_client, internal, 20.0
        )
        if result is None:
            response.message = 'Mode manager timed out'
            return response
        response.success = result.success
        response.message = result.message
        if result.success and request.mode in ('localization', 'operations'):
            if not self._wait_for_health(30.0):
                response.success = False
                response.message = (
                    'Mode started but map/scan/TF did not become healthy; '
                    'check Health and run localization recovery if needed'
                )
        return response


def main(args=None) -> None:
    """Run the backend with enough threads for synchronous service wrapping."""
    rclpy.init(args=args)
    node = OperatorBackend()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
