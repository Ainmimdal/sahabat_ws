"""
Canonical hardware bringup entry point.

This compatibility layer keeps the proven ``sahabat_launch.py`` implementation
behind a stable operational name.  Existing commands remain supported.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('shbat_pkg')

    arguments = [
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('publish_robot_state', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('joy_cmd_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('lidar_scan_topic', default_value='scan'),
        DeclareLaunchArgument('use_scan_filter', default_value='false'),
        DeclareLaunchArgument('use_battery_monitor', default_value='true'),
        DeclareLaunchArgument('battery_port', default_value='/dev/junctek'),
    ]

    legacy_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sahabat_launch.py')
        ),
        launch_arguments={
            'use_kalman_filter': LaunchConfiguration('use_ekf'),
            'publish_robot_state': LaunchConfiguration('publish_robot_state'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'joy_cmd_topic': LaunchConfiguration('joy_cmd_topic'),
            'lidar_scan_topic': LaunchConfiguration('lidar_scan_topic'),
            'use_scan_filter': LaunchConfiguration('use_scan_filter'),
        }.items(),
    )

    battery_monitor = Node(
        package='shbat_pkg',
        executable='junctek_battery',
        name='junctek_battery',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_battery_monitor')),
        parameters=[
            os.path.join(pkg_share, 'config', 'junctek_battery.yaml'),
            {'port': LaunchConfiguration('battery_port')},
        ],
    )

    return LaunchDescription(arguments + [legacy_bringup, battery_monitor])
