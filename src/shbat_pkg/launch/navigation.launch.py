"""Launch canonical odometry, mapping, or localization navigation modes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _mode_is(mode, expected):
    return IfCondition(PythonExpression(["'", mode, "' == '", expected, "'"]))


def generate_launch_description():
    pkg_share = get_package_share_directory('shbat_pkg')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map_file')
    use_ekf = LaunchConfiguration('use_ekf')
    use_rviz = LaunchConfiguration('use_rviz')
    use_zed = LaunchConfiguration('use_zed')
    localization_backend = LaunchConfiguration('localization_backend')
    continue_mapping = LaunchConfiguration('continue_mapping')
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_mapping_panel = LaunchConfiguration('use_mapping_panel')
    joy_cmd_topic = LaunchConfiguration('joy_cmd_topic')
    smoothed_cmd_topic = LaunchConfiguration('smoothed_cmd_topic')
    operator_safety = LaunchConfiguration('operator_safety')
    use_hardware = LaunchConfiguration('use_hardware')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    arguments = [
        DeclareLaunchArgument(
            'mode', default_value='odom_only',
            choices=['odom_only', 'mapping', 'localization'],
            description='Navigation mode',
        ),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_zed', default_value='false'),
        DeclareLaunchArgument(
            'localization_backend',
            default_value='amcl',
            choices=['amcl', 'slam_toolbox'],
            description='Saved-map localization backend for localization mode',
        ),
        DeclareLaunchArgument('continue_mapping', default_value='false'),
        DeclareLaunchArgument('use_foxglove', default_value='false'),
        DeclareLaunchArgument('use_mapping_panel', default_value='true'),
        DeclareLaunchArgument('joy_cmd_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('smoothed_cmd_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('operator_safety', default_value='false'),
        DeclareLaunchArgument('use_hardware', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
    ]

    odom_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_test_launch.py')
        ),
        launch_arguments={
            'use_ekf': use_ekf,
            'use_rviz': use_rviz,
        }.items(),
        condition=_mode_is(mode, 'odom_only'),
    )

    slam_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam_nav_launch.py')
        ),
        launch_arguments={
            'mode': mode,
            'map_file': map_file,
            'use_rviz': use_rviz,
            'use_zed': use_zed,
            'localization_backend': localization_backend,
            'continue_mapping': continue_mapping,
            'use_foxglove': use_foxglove,
            'use_mapping_panel': use_mapping_panel,
            'joy_cmd_topic': joy_cmd_topic,
            'smoothed_cmd_topic': smoothed_cmd_topic,
            'operator_safety': operator_safety,
            'use_hardware': use_hardware,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' != 'odom_only'"])
        ),
    )

    return LaunchDescription(arguments + [odom_only, slam_navigation])
