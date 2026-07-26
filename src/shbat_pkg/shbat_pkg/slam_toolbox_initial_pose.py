#!/usr/bin/env python3
"""Apply RViz 2D Pose Estimate to SLAM Toolbox localization."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph


class SlamToolboxInitialPose(Node):
    """Bridge /initialpose into slam_toolbox's deserialize localization API."""

    def __init__(self) -> None:
        super().__init__('slam_toolbox_initial_pose')
        self.declare_parameter('map_file_name', '')
        self.map_file_name = str(self.get_parameter('map_file_name').value)
        self.client = self.create_client(
            DeserializePoseGraph,
            '/slam_toolbox/deserialize_map',
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10,
        )
        self.get_logger().info(
            'SLAM Toolbox initial-pose bridge ready. Use RViz 2D Pose Estimate.'
        )

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def initial_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if not self.map_file_name:
            self.get_logger().error(
                'Cannot set SLAM Toolbox pose: map_file_name is empty.'
            )
            return
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                'SLAM Toolbox deserialize_map service is not available yet.'
            )
            return

        request = DeserializePoseGraph.Request()
        request.filename = self.map_file_name
        request.match_type = DeserializePoseGraph.Request.LOCALIZE_AT_POSE
        request.initial_pose.x = message.pose.pose.position.x
        request.initial_pose.y = message.pose.pose.position.y
        request.initial_pose.theta = self.yaw_from_quaternion(
            message.pose.pose.orientation
        )
        future = self.client.call_async(request)
        future.add_done_callback(self.deserialize_done)
        self.get_logger().info(
            'Requested SLAM Toolbox localization at '
            f'x={request.initial_pose.x:.3f}, '
            f'y={request.initial_pose.y:.3f}, '
            f'yaw={request.initial_pose.theta:.3f}'
        )

    def deserialize_done(self, future) -> None:
        try:
            future.result()
        except Exception as error:
            self.get_logger().error(
                f'SLAM Toolbox localization pose request failed: {error}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamToolboxInitialPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
