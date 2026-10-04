#!/usr/bin/env python3
"""ROS glue that lets a node confirm a candidate pose against the live lidar.

Owns /map and /scan subscriptions plus the base->laser lookup and delegates
the matching to :mod:`shbat_pkg.scan_matcher`.
"""

from __future__ import annotations

from typing import Optional, Tuple

from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

from shbat_pkg.scan_matcher import (
    LikelihoodField,
    MatchResult,
    accept,
    evaluate,
    occupancy_from_message,
    planar_rotation,
    scan_to_points,
    search,
)


class ScanLocalizationSupport:
    """Attach to a node that already owns a tf2 Buffer."""

    def __init__(
        self,
        node,
        tf_buffer,
        base_frame: str = 'base_link',
        map_topic: str = '/map',
        scan_topic: str = '/scan',
        scan_timeout: float = 1.0,
    ):
        self.node = node
        self.tf_buffer = tf_buffer
        self.base_frame = base_frame
        self.scan_timeout = float(scan_timeout)
        self.field: Optional[LikelihoodField] = None
        self.map_error = ''
        self.scan: Optional[LaserScan] = None
        self.scan_received_at = 0.0
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(OccupancyGrid, map_topic, self._map, map_qos)
        node.create_subscription(LaserScan, scan_topic, self._scan, qos_profile_sensor_data)

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def _map(self, message: OccupancyGrid) -> None:
        try:
            grid, resolution, origin_x, origin_y = occupancy_from_message(message)
            self.field = LikelihoodField(grid, resolution, origin_x, origin_y)
            self.map_error = ''
            self.node.get_logger().info(
                f'Scan matcher loaded map {message.info.width}x{message.info.height} '
                f'@ {resolution:.3f} m.'
            )
        except ValueError as error:
            self.field = None
            self.map_error = str(error)
            self.node.get_logger().error(f'Scan matcher cannot use /map: {error}')

    def _scan(self, message: LaserScan) -> None:
        self.scan = message
        self.scan_received_at = self._now()

    def ready(self) -> Tuple[bool, str]:
        """Report whether map, a fresh scan and the laser transform exist."""
        if self.field is None:
            return False, self.map_error or 'waiting for /map'
        if self.scan is None or self._now() - self.scan_received_at > self.scan_timeout:
            return False, 'waiting for a fresh /scan'
        if self._sensor_pose() is None:
            return False, f'waiting for {self.base_frame} -> {self.scan.header.frame_id} TF'
        return True, 'ready'

    def _sensor_pose(self):
        if self.scan is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.scan.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return None
        t = transform.transform
        q = t.rotation
        # Full rotation, not yaw: the lidar is mounted upside down
        # (lidar_joint rpy 3.14 0 3.14) and a yaw-only projection mirrors it.
        return t.translation.x, t.translation.y, planar_rotation(q.x, q.y, q.z, q.w)

    def _points(self):
        sensor_x, sensor_y, rotation = self._sensor_pose()
        scan = self.scan
        return scan_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            sensor_x,
            sensor_y,
            sensor_rotation=rotation,
        )

    def confirm(
        self, pose: Tuple[float, float, float], minimum_inlier_ratio: float
    ) -> Tuple[bool, str]:
        """Check that the current scan fits the map at ``pose`` (no search)."""
        is_ready, reason = self.ready()
        if not is_ready:
            return False, reason
        result = evaluate(self.field, self._points(), pose)
        detail = f'{result.inlier_ratio:.0%} of lidar beams on mapped walls'
        return result.inlier_ratio >= minimum_inlier_ratio, detail

    def match(
        self,
        center: Tuple[float, float, float],
        xy_window: float,
        yaw_window: float,
        minimum_inlier_ratio: float,
        maximum_ambiguity: float,
    ) -> Tuple[bool, Optional[MatchResult], str]:
        """Search around ``center`` and apply the acceptance gate."""
        is_ready, reason = self.ready()
        if not is_ready:
            return False, None, reason
        points = self._points()
        try:
            result = search(
                self.field, points, center,
                xy_window=xy_window, yaw_window=yaw_window,
            )
        except ValueError as error:
            return False, None, str(error)
        ok, message = accept(result, minimum_inlier_ratio, maximum_ambiguity)
        return ok, result, message
