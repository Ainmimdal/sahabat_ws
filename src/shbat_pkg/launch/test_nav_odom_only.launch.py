#!/usr/bin/env python3
"""
Quick Test Launch - Navigation with Odometry Only (No Mapping)

WARNING: This is for TESTING ONLY!
- No map building
- No localization
- Robot navigates using only odometry
- Will drift over time without SLAM corrections
- Good for testing odometry quality and basic navigation

For production use: Use sahabat_mapping.launch.py (slam_toolbox) or sahabat_rtabmap.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get package directory
    shbat_pkg_dir = get_package_share_directory('shbat_pkg')
    nav2_params = os.path.join(shbat_pkg_dir, 'config', 'nav2_params.yaml')
    
    # Note: This assumes ZED is already running and publishing /odom
    # Launch sahabat_launch.py first!
    
    # Nav2 nodes for local planning only (no global map)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )
    
    # RViz for sending goals
    rviz_config = os.path.join(shbat_pkg_dir, 'rviz', 'nav2.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        rviz,
    ])
