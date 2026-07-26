#!/usr/bin/env python3
"""Initialize saved-map localization from the map's dock pose."""

import math
import os
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class DockPoseInitializer(Node):
    """Publish the dock waypoint to /initialpose after localization starts."""

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

        self.waypoint_file = os.path.expanduser(
            str(self.get_parameter('waypoint_file').value)
        )
        self.dock_name = str(self.get_parameter('dock_name').value)
        self.maps_directory = Path(
            str(self.get_parameter('maps_directory').value)
        ).expanduser()
        self.map_id = str(self.get_parameter('map_id').value).strip()
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.dock_pose = self.load_dock_pose()
        self.publish_count = 0
        delay = float(self.get_parameter('startup_delay').value)
        self.start_timer = self.create_timer(delay, self.begin_publishing)
        self.publish_timer = None

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
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(
                f'Cannot load waypoint file {self.waypoint_file}: {exc}'
            )
            return None

        for waypoint in data.get('waypoints', []):
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

        self.get_logger().error(
            f'No waypoint named "{self.dock_name}" in {self.waypoint_file}'
        )
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

    def begin_publishing(self):
        self.start_timer.cancel()
        if self.dock_pose is None:
            return
        self.publish_dock_pose()
        self.publish_timer = self.create_timer(0.25, self.publish_dock_pose)

    def publish_dock_pose(self):
        if self.publish_count >= 3:
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            return

        x, y, yaw = self.dock_pose
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.10
        message.pose.covariance[7] = 0.10
        message.pose.covariance[35] = 0.05
        self.publisher.publish(message)
        self.publish_count += 1

        if self.publish_count == 1:
            self.get_logger().info(
                f'Initialized localization at dock: x={x:.3f}, y={y:.3f}, '
                f'yaw={yaw:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseInitializer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
