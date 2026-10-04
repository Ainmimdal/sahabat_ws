"""Sahabat robot app: one entry point, modes switched from the web console.

Starts only the persistent operator layer:

- operator_backend: control lease, E-stop latch, teleop, maps, waypoints
- operator_mode_manager (stack:=full): starts/stops the complete robot stack
  for the selected mode, exactly as the desktop shortcuts do
- web_console: browser UI on port 8088

The robot boots into Idle (no drivers running) unless start_mode is given:

  ros2 launch shbat_pkg robot.launch.py
  ros2 launch shbat_pkg robot.launch.py start_mode:=operations map_id:=gallerysq4
  ros2 launch shbat_pkg robot.launch.py start_mode:=mapping

The web console has no authentication; keep it on a trusted network.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maps_directory = LaunchConfiguration('maps_directory')
    arguments = [
        DeclareLaunchArgument(
            'maps_directory',
            default_value=os.path.expanduser('~/sahabat_ws/maps'),
        ),
        DeclareLaunchArgument(
            'start_mode', default_value='idle',
            choices=['idle', 'mapping', 'operations'],
            description='Mode to start once the operator layer is up',
        ),
        DeclareLaunchArgument(
            'map_id', default_value='',
            description='Map for start_mode:=operations (file stem in maps/)',
        ),
        DeclareLaunchArgument('web_port', default_value='8088'),
    ]

    backend = Node(
        package='shbat_pkg',
        executable='operator_backend',
        name='operator_backend',
        output='screen',
        parameters=[{'maps_directory': maps_directory}],
    )
    mode_manager = Node(
        package='shbat_pkg',
        executable='operator_mode_manager',
        name='operator_mode_manager',
        output='screen',
        parameters=[{
            'maps_directory': maps_directory,
            'stack': 'full',
            'initial_mode': LaunchConfiguration('start_mode'),
            'initial_map': LaunchConfiguration('map_id'),
        }],
    )
    web_console = Node(
        package='shbat_pkg',
        executable='web_console',
        name='web_console',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'maps_directory': maps_directory,
            'port': LaunchConfiguration('web_port'),
        }],
    )
    return LaunchDescription(arguments + [backend, mode_manager, web_console])
