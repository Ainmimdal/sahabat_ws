"""
Canonical gallery operations entry point.

The implementation delegates to the established localization and patrol launch
so the operational behavior remains compatible while callers gain one stable
command.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('shbat_pkg')

    names_and_defaults = [
        ('use_zed', 'false'),
        ('use_rviz', 'true'),
        ('use_foxglove', 'false'),
        ('use_api', 'false'),
        ('use_waypoint_gui', 'true'),
        ('joy_cmd_topic', 'cmd_vel'),
        ('smoothed_cmd_topic', 'cmd_vel'),
        ('recovery_cmd_topic', 'cmd_vel'),
        ('operator_safety', 'false'),
        ('use_saved_initial_pose', 'false'),
        ('initialize_from_dock', 'true'),
        ('initial_pose_x', '0.0'),
        ('initial_pose_y', '0.0'),
        ('initial_pose_yaw', '0.0'),
        (
            'waypoint_file',
            os.path.expanduser(
                '~/sahabat_ws/src/shbat_pkg/config/patrol_waypoints.yaml'
            ),
        ),
    ]

    arguments = [
        DeclareLaunchArgument(
            'map_file', description='Map path without the .yaml extension'
        )
    ]
    arguments.extend(
        DeclareLaunchArgument(name, default_value=default)
        for name, default in names_and_defaults
    )

    operations = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization_patrol_launch.py')
        ),
        launch_arguments={
            'map_file': LaunchConfiguration('map_file'),
            **{
                name: LaunchConfiguration(name)
                for name, _default in names_and_defaults
            },
        }.items(),
    )

    return LaunchDescription(arguments + [operations])
