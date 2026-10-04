#!/usr/bin/env python3
"""Initialize saved-map localization near the map's dock without manual help.

Startup policy (AMCL backend):

1. Wait until AMCL is listening on /initialpose.
2. If saved AprilTags exist for this map, give the AprilTag landmark manager
   ``tag_wait_timeout`` seconds to localize the robot.
3. Otherwise (or if no tag localized the robot in time) search a bounded
   window around the dock pose with the live lidar scan against the saved map.
   The robot does not have to be exactly on the dock: it only has to be
   within ``dock_search_xy_window`` / ``dock_search_yaw_window`` of it.
4. Publish /initialpose only when the lidar match is good and unambiguous.
   A rejected match is retried every ``retry_interval`` seconds (visitors
   standing in front of the lidar are a common cause) and the robot stays
   unlocalized instead of being given a guessed pose.

The node never commands motion. Set ``refine_with_scan:=false`` to get the
legacy behaviour (publish the stored dock pose blindly).
"""

import math
import os
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from shbat_pkg.scan_localization import ScanLocalizationSupport


def amcl_covariance_good(covariance, limit=0.20):
    """Return True when AMCL reports a converged planar pose."""
    values = (covariance[0], covariance[7], covariance[35])
    return all(math.isfinite(value) and value <= limit for value in values)


class DockPoseInitializer(Node):
    """Lidar-confirmed dock initialization with AprilTag priority."""

    def __init__(self):
        super().__init__('dock_pose_initializer')
        self.declare_parameter(
            'waypoint_file',
            '~/sahabat_ws/src/shbat_pkg/config/patrol_waypoints.yaml',
        )
        self.declare_parameter('dock_name', 'dock')
        self.declare_parameter('startup_delay', 4.0)
        self.declare_parameter('maps_directory', '~/sahabat_ws/maps')
        self.declare_parameter('map_id', '')
        self.declare_parameter('skip_if_apriltag_configured', True)
        self.declare_parameter('refine_with_scan', True)
        self.declare_parameter('tag_wait_timeout', 25.0)
        self.declare_parameter('dock_search_xy_window', 0.75)
        self.declare_parameter('dock_search_yaw_window', 0.61)  # 35 deg
        self.declare_parameter('minimum_inlier_ratio', 0.55)
        self.declare_parameter('maximum_ambiguity', 0.92)
        self.declare_parameter('retry_interval', 5.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stationary_linear_threshold', 0.02)
        self.declare_parameter('stationary_angular_threshold', 0.04)

        get = self.get_parameter
        self.waypoint_file = os.path.expanduser(str(get('waypoint_file').value))
        self.dock_name = str(get('dock_name').value)
        self.maps_directory = Path(str(get('maps_directory').value)).expanduser()
        self.map_id = str(get('map_id').value).strip()
        self.skip_if_apriltag_configured = bool(get('skip_if_apriltag_configured').value)
        self.refine_with_scan = bool(get('refine_with_scan').value)
        self.tag_wait_timeout = float(get('tag_wait_timeout').value)
        self.xy_window = float(get('dock_search_xy_window').value)
        self.yaw_window = float(get('dock_search_yaw_window').value)
        self.minimum_inlier_ratio = float(get('minimum_inlier_ratio').value)
        self.maximum_ambiguity = float(get('maximum_ambiguity').value)
        self.retry_interval = float(get('retry_interval').value)
        self.linear_threshold = float(get('stationary_linear_threshold').value)
        self.angular_threshold = float(get('stationary_angular_threshold').value)

        self.apriltag_configured = False
        self.localized = False
        self.last_odom_at = 0.0
        self.moving = False
        self.started_at = None
        self.next_attempt_at = 0.0
        self.pending_message = None
        self.pending_count = 0
        self.last_status = ''

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.status_pub = self.create_publisher(
            String, '/localization/startup_status', latched
        )
        self.create_subscription(
            Bool, '/localization/apriltag_configured',
            self.apriltag_configured_callback, latched,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_support = (
            ScanLocalizationSupport(self, self.tf_buffer, str(get('base_frame').value))
            if self.refine_with_scan else None
        )

        self.dock_pose = self.load_dock_pose()
        delay = float(get('startup_delay').value)
        self.start_timer = self.create_timer(delay, self.begin)
        self.tick_timer = None

    # ------------------------------------------------------------------ setup
    def load_dock_pose(self):
        for path in self.dock_candidates():
            dock = self.read_dock_file(path)
            if dock is not None:
                self.get_logger().info(f'Loaded dock pose from {path}')
                return dock

        # Compatibility fallback for standalone legacy waypoint files.
        try:
            with open(self.waypoint_file, encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError):
            data = {}
        for waypoint in data.get('waypoints', []) if isinstance(data, dict) else []:
            if str(waypoint.get('name', '')).strip().lower() == (
                self.dock_name.strip().lower()
            ):
                try:
                    return (
                        float(waypoint['x']),
                        float(waypoint['y']),
                        float(waypoint['yaw']),
                    )
                except (KeyError, TypeError, ValueError) as exc:
                    self.get_logger().error(
                        f'Dock waypoint has invalid coordinates: {exc}'
                    )
                    return None
        return None

    def dock_candidates(self):
        if not self.map_id:
            return []
        return [
            self.maps_directory / 'waypoint_sets' / self.map_id / 'dock.yaml',
            self.maps_directory / self.map_id / 'dock.yaml',
        ]

    @staticmethod
    def read_dock_file(path):
        try:
            with path.open(encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError):
            return None
        dock = data.get('dock') if isinstance(data, dict) else None
        if not dock:
            return None
        try:
            return (
                float(dock['x']),
                float(dock['y']),
                float(dock['yaw']),
            )
        except (KeyError, TypeError, ValueError):
            return None

    # -------------------------------------------------------------- callbacks
    def apriltag_configured_callback(self, message):
        self.apriltag_configured = bool(message.data)

    def amcl_pose_callback(self, message):
        if amcl_covariance_good(message.pose.covariance):
            self.localized = True

    def odom_callback(self, message):
        self.last_odom_at = self.now()
        self.moving = (
            math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
            > self.linear_threshold
            or abs(message.twist.twist.angular.z) > self.angular_threshold
        )

    # ------------------------------------------------------------------ logic
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_status(self, text, level='info'):
        if text == self.last_status:
            return
        self.last_status = text
        getattr(self.get_logger(), level)(text)
        self.status_pub.publish(String(data=text))

    def begin(self):
        self.start_timer.cancel()
        self.started_at = self.now()
        if self.dock_pose is None:
            self.set_status(
                f'No dock pose for map "{self.map_id}". Capture a "dock" '
                'waypoint in the waypoint editor (or save AprilTags) so the '
                'robot can localize itself at startup.',
                'warn',
            )
        self.tick_timer = self.create_timer(0.25, self.tick)

    def tick(self):
        if self.pending_message is not None:
            self.publish_pending()
            return
        if self.localized:
            self.set_status('Robot is localized.')
            self.tick_timer.cancel()
            return
        if self.count_subscribers('/initialpose') == 0:
            self.set_status('Waiting for AMCL to listen on /initialpose.')
            return

        waiting_for_tags = (
            self.skip_if_apriltag_configured
            and self.apriltag_configured
            and self.now() - self.started_at < self.tag_wait_timeout
        )
        if waiting_for_tags:
            self.set_status(
                'Saved AprilTags configured; waiting for tag localization '
                f'(dock fallback in {self.tag_wait_timeout:.0f} s).'
            )
            return
        if self.dock_pose is None:
            return
        if self.now() < self.next_attempt_at:
            return
        self.next_attempt_at = self.now() + self.retry_interval

        if self.scan_support is None:
            self.queue_pose(*self.dock_pose, 0.10, 0.05, 'stored dock pose (unverified)')
            return
        if self.moving:
            self.set_status('Waiting for the robot to be stationary with odometry.')
            self.next_attempt_at = self.now() + 1.0
            return
        ok, result, detail = self.scan_support.match(
            self.dock_pose,
            self.xy_window,
            self.yaw_window,
            self.minimum_inlier_ratio,
            self.maximum_ambiguity,
        )
        if result is None:
            self.set_status(f'Dock localization: {detail}.')
            self.next_attempt_at = self.now() + 1.0
            return
        if not ok:
            self.set_status(
                f'Dock localization rejected: {detail}. The robot may be '
                f'more than {self.xy_window:.2f} m / '
                f'{math.degrees(self.yaw_window):.0f} deg from the dock, or the '
                f'lidar view is blocked. Retrying in {self.retry_interval:.0f} s; '
                'use 2D Pose Estimate if this persists.',
                'warn',
            )
            return
        dx = result.x - self.dock_pose[0]
        dy = result.y - self.dock_pose[1]
        self.queue_pose(
            result.x, result.y, result.yaw, 0.08, 0.05,
            f'lidar match {math.hypot(dx, dy):.2f} m from dock ({detail})',
        )

    def queue_pose(self, x, y, yaw, position_std, yaw_std, reason):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = position_std ** 2
        message.pose.covariance[7] = position_std ** 2
        message.pose.covariance[35] = yaw_std ** 2
        self.pending_message = message
        self.pending_count = 3
        self.set_status(
            f'Initialized localization: x={x:.3f}, y={y:.3f}, '
            f'yaw={math.degrees(yaw):.1f} deg from {reason}.'
        )

    def publish_pending(self):
        self.pending_message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pending_message)
        self.pending_count -= 1
        if self.pending_count <= 0:
            self.pending_message = None


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
