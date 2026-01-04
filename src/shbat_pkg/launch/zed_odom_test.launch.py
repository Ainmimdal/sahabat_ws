#!/usr/bin/env python3
"""
Minimal ZED odometry test launch: bring up base + ZED only, no RTAB-Map.

Use this to validate ZED odometry and TFs in RViz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shbat_pkg_dir = get_package_share_directory('shbat_pkg')

    # URDF with camera-as-root so ZED publishes odom -> zed2i_camera_link
    urdf_file = os.path.join(shbat_pkg_dir, 'urdf', 'sahabat_robot_with_zed.urdf.xacro')
    robot_description = Command([
        'xacro ', urdf_file,
        ' camera_name:=zed2i',
        ' camera_model:=zed2i',
        ' use_zed_localization:=true'
    ])
    robot_description_param = ParameterValue(robot_description, value_type=str)

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Start RViz to visualize odometry'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    # Base bringup, but don't publish robot_state (we do it here) and disable fusion/extra IMU
    base_launch_file = os.path.join(shbat_pkg_dir, 'launch', 'sahabat_launch.py')
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            'publish_robot_state': 'false',
            'use_n100_imu': 'false',
            'use_kalman_filter': 'false',
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}]
    )

    # ZED wrapper (odometry + TF only)
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_launch_file = os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')

    # Use the positional tracking override already present in shbat_pkg
    zed_override_src = os.path.join(os.path.dirname(__file__), '..', 'config', 'zed_pos_tracking_override.yaml')
    zed_override_params = os.path.abspath(zed_override_src)
    if not os.path.exists(zed_override_params):
        zed_override_params = os.path.join(shbat_pkg_dir, 'config', 'zed_pos_tracking_override.yaml')

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'publish_urdf': 'false',
            'pos_tracking_enabled': 'true',
            'ros_params_override_path': zed_override_params,
        }.items()
    )

    # Provide IMU TF in case URDF variant omitted it
    static_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_imu_tf',
        output='screen',
        arguments=['0','0','0','0','0','0','zed2i_camera_link','zed2i_imu_link']
    )

    # RViz for odom visualization (simple config)
    rviz_cfg = os.path.join(shbat_pkg_dir, 'rviz', 'zed_odom_test.rviz')
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_rviz_arg,
        base_robot_launch,
        robot_state_publisher_node,
        zed_wrapper_launch,
        static_imu_tf,
        rviz_node,
    ])
