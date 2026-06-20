"""Start the persistent remote operator core without moving the robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the remote core; navigation starts only by explicit GUI action."""
    share = get_package_share_directory('shbat_pkg')
    maps_directory = LaunchConfiguration('maps_directory')
    arguments = [
        DeclareLaunchArgument(
            'maps_directory',
            default_value=os.path.expanduser('~/sahabat_ws/maps'),
        ),
        DeclareLaunchArgument('mobile_gateway', default_value='false'),
        DeclareLaunchArgument('mobile_certificate', default_value=''),
        DeclareLaunchArgument('mobile_private_key', default_value=''),
        DeclareLaunchArgument('mobile_token_file', default_value=''),
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'joy_cmd_topic': 'cmd_vel_joy',
            'lidar_scan_topic': 'scan_raw',
            'use_scan_filter': 'true',
        }.items(),
    )

    backend = Node(
        package='shbat_pkg',
        executable='operator_backend',
        name='operator_backend',
        output='screen',
        parameters=[{'maps_directory': maps_directory}],
    )
    arbiter = Node(
        package='shbat_pkg',
        executable='command_arbiter',
        name='command_arbiter',
        output='screen',
    )
    mode_manager = Node(
        package='shbat_pkg',
        executable='operator_mode_manager',
        name='operator_mode_manager',
        output='screen',
        parameters=[{'maps_directory': maps_directory}],
    )
    bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            os.path.join(share, 'config', 'foxglove_operator.yaml')
        ],
    )
    mobile_gateway = Node(
        package='shbat_pkg',
        executable='mobile_gateway',
        name='mobile_gateway',
        output='screen',
        condition=IfCondition(LaunchConfiguration('mobile_gateway')),
        parameters=[{
            'certificate': LaunchConfiguration('mobile_certificate'),
            'private_key': LaunchConfiguration('mobile_private_key'),
            'token_file': LaunchConfiguration('mobile_token_file'),
        }],
    )
    return LaunchDescription(
        arguments + [
            hardware,
            backend,
            arbiter,
            mode_manager,
            bridge,
            mobile_gateway,
        ]
    )
