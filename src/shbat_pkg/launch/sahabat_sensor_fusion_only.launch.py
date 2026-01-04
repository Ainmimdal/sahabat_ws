#!/usr/bin/env python3
"""
Sensor Fusion Only Launch - Robot with EKF, No SLAM

This launch file provides:
- Base robot (motors, IMU, LiDAR)
- ZED2i camera (visual odometry + point cloud)
- robot_localization EKF (fuses wheel+IMU with ZED)
- Static transforms
- RViz2 for visualization (optional)

NO SLAM - Just sensor fusion for testing navigation with odometry only.

Usage:
    # Basic launch
    ros2 launch shbat_pkg sahabat_sensor_fusion_only.launch.py
    
    # With RViz
    ros2 launch shbat_pkg sahabat_sensor_fusion_only.launch.py use_rviz:=true
    
Then in another terminal:
    ros2 launch shbat_pkg sahabat_nav.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package directories
    shbat_pkg_dir = get_package_share_directory('shbat_pkg')
    ekf_config = os.path.join(shbat_pkg_dir, 'config', 'ekf.yaml')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    
    # ZED wrapper launch file
    zed_launch_file = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    )
    
    # Base robot launch (motors, IMU, LiDAR, kalman_filter)
    base_launch_file = os.path.join(shbat_pkg_dir, 'launch', 'sahabat_launch.py')
    
    # Base Robot Launch
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file)
    )
    
    # ZED2i Camera Launch
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'publish_tf': 'false',  # EKF publishes odom->base_link
            'publish_map_tf': 'false',
            'pos_tracking_enabled': 'true',  # Enable visual odometry
            'depth_mode': 'PERFORMANCE',
            'depth_stabilization': '1',
            'base_frame': 'base_link',
        }.items()
    )
    
    # robot_localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )
    
    # Static transform: base_link -> zed2i_camera_link
    zed_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_zed_tf',
        output='screen',
        arguments=[
            '0.05', '0.0', '0.67',     # x, y, z (67cm height)
            '0.0', '0.13963', '0.0',   # roll, pitch (+8° to match physical downward angle), yaw
            'base_link',
            'zed2i_camera_link'
        ]
    )
    
    # RViz2
    rviz_config = os.path.join(shbat_pkg_dir, 'rviz', 'rtabmap.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        
        # Nodes
        base_robot_launch,     # Base robot (motors, LiDAR, kalman_filter)
        zed_wrapper_launch,    # ZED camera (visual odometry + point cloud)
        ekf_node,              # EKF sensor fusion
        zed_base_link_tf,      # Static transform
        rviz_node,             # RViz2 (optional)
    ])
