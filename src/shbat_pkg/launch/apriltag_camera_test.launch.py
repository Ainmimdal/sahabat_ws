"""Motion-free ZED 2i + AprilTag detection test.

Starts ONLY the ZED (image only, depth off) and apriltag_ros, plus a viewer
window that draws detected tags with their ID, distance and quality. No motors,
lidar, AMCL or Nav2 are started.

Run without rebuilding:
  ros2 launch ~/sahabat_ws/src/shbat_pkg/launch/apriltag_camera_test.launch.py
Optional: resolution:=VGA  (default HD720)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.realpath(__file__))
PKG_SRC = os.path.dirname(HERE)


def generate_launch_description():
    zed_share = get_package_share_directory('zed_wrapper')
    resolution = LaunchConfiguration('resolution')

    zed = ComposableNodeContainer(
        name='zed_container',
        namespace='zed',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[ComposableNode(
            package='zed_components',
            namespace='zed',
            plugin='stereolabs::ZedCamera',
            name='zed_node',
            parameters=[
                os.path.join(zed_share, 'config', 'common_stereo.yaml'),
                os.path.join(zed_share, 'config', 'zed2i.yaml'),
                {
                    'general.camera_name': 'zed2i',
                    'general.camera_model': 'zed2i',
                    'general.grab_resolution': resolution,
                    'general.grab_frame_rate': 15,
                    'general.pub_frame_rate': 15.0,
                    'depth.depth_mode': 'NONE',
                    'pos_tracking.pos_tracking_enabled': False,
                    'pos_tracking.publish_tf': False,
                    'pos_tracking.publish_map_tf': False,
                    'object_detection.od_enabled': False,
                    'body_tracking.bt_enabled': False,
                    'sensors.publish_imu_tf': False,
                },
            ],
        )],
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[os.path.join(PKG_SRC, 'config', 'apriltag_landmarks.yaml')],
        remappings=[
            ('image_rect', '/zed/zed_node/left/image_rect_color'),
            ('camera_info', '/zed/zed_node/left/camera_info'),
            ('detections', '/apriltag/detections'),
        ],
    )

    viewer = ExecuteProcess(
        cmd=['python3', os.path.join(PKG_SRC, 'scripts', 'apriltag_viewer.py')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='HD720'),
        zed,
        apriltag,
        viewer,
    ])
