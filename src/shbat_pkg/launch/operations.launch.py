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
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('shbat_pkg')

    names_and_defaults = [
        ('maps_directory', '~/sahabat_ws/maps'),
        ('map_id', ''),
        ('use_zed', 'false'),
        ('use_keepout', 'true'),
        ('keepout_mask_file', ''),
        ('use_rviz', 'true'),
        ('use_foxglove', 'false'),
        ('use_api', 'false'),
        ('use_waypoint_gui', 'true'),
        ('localization_backend', 'amcl'),
        ('joy_cmd_topic', 'cmd_vel_joy'),
        ('smoothed_cmd_topic', 'cmd_vel_nav_smoothed'),
        ('recovery_cmd_topic', 'cmd_vel_recovery'),
        ('use_command_arbiter', 'true'),
        ('operator_safety', 'false'),
        ('use_hardware', 'true'),
        ('use_saved_initial_pose', 'false'),
        ('initialize_from_dock', 'true'),
        ('initial_pose_x', '0.0'),
        ('initial_pose_y', '0.0'),
        ('initial_pose_yaw', '0.0'),
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
    arguments.append(DeclareLaunchArgument(
        'waypoint_file',
        default_value=PythonExpression([
            "'", LaunchConfiguration('map_file'),
            "'.rsplit('/', 1)[0] + '/waypoints.yaml'",
        ]),
        description='Legacy waypoint path; its directory owns waypoint sets',
    ))

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
            'waypoint_file': LaunchConfiguration('waypoint_file'),
        }.items(),
    )

    return LaunchDescription(arguments + [operations])
