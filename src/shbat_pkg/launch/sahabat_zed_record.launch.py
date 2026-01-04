#!/usr/bin/env python3
"""
Launch file for ZED camera only - for recording data to rosbag.

This launch file brings up:
- ZED2i camera wrapper (for visual odometry and sensor data)
- Robot State Publisher (to publish the TF tree from URDF)
- Optional: RViz for visualization while recording

Use this to record a rosbag, then play it back with RTAB-Map for offline mapping.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shbat_pkg_dir = get_package_share_directory("shbat_pkg")
    pkg_src_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    # URDF
    urdf_file = os.path.join(
        shbat_pkg_dir, "urdf", "sahabat_robot_with_zed.urdf.xacro"
    )
    robot_description = Command(
        [
            "xacro ",
            urdf_file,
            " camera_name:=zed2i",
            " camera_model:=zed2i",
            " use_zed_localization:=true", # ZED publishes odom->camera_link
        ]
    )
    robot_description_param = ParameterValue(robot_description, value_type=str)

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz2 for visualization"
    )

    use_rviz = LaunchConfiguration("use_rviz")

    # Base Robot Launch (motors only - no mapping)
    base_launch_file = os.path.join(shbat_pkg_dir, 'launch', 'sahabat_launch.py')
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            'publish_robot_state': 'false',
            'use_n100_imu': 'false',
            'use_kalman_filter': 'false'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_param}],
    )

    # ZED wrapper launch
    zed_wrapper_dir = get_package_share_directory("zed_wrapper")
    zed_launch_file = os.path.join(zed_wrapper_dir, "launch", "zed_camera.launch.py")
    zed_override_src = os.path.join(
        pkg_src_root, "config", "zed_pos_tracking_override.yaml"
    )
    zed_override_params = (
        zed_override_src
        if os.path.exists(zed_override_src)
        else os.path.join(shbat_pkg_dir, "config", "zed_pos_tracking_override.yaml")
    )

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            "camera_model": "zed2i",
            "camera_name": "zed2i",
            "publish_tf": "true",
            "publish_map_tf": "false",
            "publish_urdf": "false",
            "pos_tracking_enabled": "true",
            "sensors_fusion": "true",
            "ros_params_override_path": zed_override_params,
        }.items(),
    )

    # RViz
    rviz_config = os.path.join(shbat_pkg_dir, "rviz", "zed_rtabmap.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            base_robot_launch,
            robot_state_publisher_node,
            zed_wrapper_launch,
            rviz_node,
        ]
    )
