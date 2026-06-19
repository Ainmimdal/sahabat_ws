"""
Canonical hardware bringup entry point.

This compatibility layer keeps the proven ``sahabat_launch.py`` implementation
behind a stable operational name.  Existing commands remain supported.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('shbat_pkg')

    arguments = [
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('publish_robot_state', default_value='true'),
    ]

    legacy_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sahabat_launch.py')
        ),
        launch_arguments={
            'use_kalman_filter': LaunchConfiguration('use_ekf'),
            'publish_robot_state': LaunchConfiguration('publish_robot_state'),
        }.items(),
    )

    return LaunchDescription(arguments + [legacy_bringup])
