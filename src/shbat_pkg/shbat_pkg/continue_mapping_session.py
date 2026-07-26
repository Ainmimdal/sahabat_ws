#!/usr/bin/env python3
"""Load a saved SLAM Toolbox posegraph for continuing mapping."""

import argparse
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph, Pause


MATCH_TYPES = {
    'first': DeserializePoseGraph.Request.START_AT_FIRST_NODE,
    'pose': DeserializePoseGraph.Request.START_AT_GIVEN_POSE,
    'localize': DeserializePoseGraph.Request.LOCALIZE_AT_POSE,
}


class ContinueMappingSession(Node):
    def __init__(self):
        super().__init__('continue_mapping_session')
        self.deserialize_client = self.create_client(
            DeserializePoseGraph,
            '/slam_toolbox/deserialize_map',
        )
        self.pause_client = self.create_client(
            Pause,
            '/slam_toolbox/pause_new_measurements',
        )

    def call_pause(self, timeout_sec: float) -> bool:
        if not self.pause_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn('pause_new_measurements service unavailable')
            return False
        future = self.pause_client.call_async(Pause.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().warn('Timed out toggling pause_new_measurements')
            return False
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - report ROS service failure.
            self.get_logger().warn(f'pause_new_measurements failed: {error}')
            return False
        self.get_logger().info(
            f'pause_new_measurements status: {response.status}'
        )
        return True

    def deserialize(self, request: DeserializePoseGraph.Request, timeout_sec: float) -> bool:
        if not self.deserialize_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('/slam_toolbox/deserialize_map unavailable')
            return False
        self.get_logger().info(
            'Requesting SLAM Toolbox deserialize. Large sessions can take minutes.'
        )
        future = self.deserialize_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(
                'Timed out waiting for deserialize response. Restart mapping before retrying.'
            )
            return False
        try:
            future.result()
        except Exception as error:  # noqa: BLE001 - report ROS service failure.
            self.get_logger().error(f'Deserialize failed: {error}')
            return False
        self.get_logger().info('SLAM Toolbox deserialize completed')
        return True


def _session_stem(value: str) -> Path:
    path = Path(value).expanduser()
    if path.suffix in ('.posegraph', '.data'):
        path = path.with_suffix('')
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Deserialize a saved SLAM Toolbox session for continued mapping.',
    )
    parser.add_argument(
        'session',
        help='Session stem, e.g. ~/sahabat_ws/maps/gallerysq',
    )
    parser.add_argument(
        '--match',
        choices=sorted(MATCH_TYPES),
        default='localize',
        help='Posegraph match mode. localize is best for continuing away from the original start.',
    )
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--theta', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument(
        '--pause-scans',
        action='store_true',
        help='Toggle SLAM Toolbox pause_new_measurements before deserializing.',
    )
    args = parser.parse_args(argv)

    stem = _session_stem(args.session)
    missing = [
        str(path)
        for path in (stem.with_suffix('.posegraph'), stem.with_suffix('.data'))
        if not path.exists()
    ]
    if missing:
        for path in missing:
            print(f'Missing required session file: {path}', file=sys.stderr)
        return 2

    request = DeserializePoseGraph.Request()
    request.filename = str(stem)
    request.match_type = MATCH_TYPES[args.match]
    request.initial_pose = Pose2D(x=args.x, y=args.y, theta=args.theta)

    rclpy.init(args=None)
    node = ContinueMappingSession()
    try:
        if args.pause_scans:
            node.call_pause(min(10.0, args.timeout))
        return 0 if node.deserialize(request, args.timeout) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
