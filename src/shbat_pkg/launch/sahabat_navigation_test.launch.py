#!/usr/bin/env python3
"""
Complete Navigation Testing Launch File
========================================

This launch file provides complete navigation testing with all robot sensors:
- Base robot (motors, wheel encoders, N100 IMU, M200 LiDAR)
- ZED2i camera (visual odometry, depth, point cloud)
- robot_localization EKF (fuses wheel + IMU + ZED odometry)
- Nav2 navigation stack (costmaps use LiDAR + ZED point cloud)
- Robot state publisher with proper URDF
- TF tree management
- RViz for visualization

USE CASE:
---------
1. Test navigation with sensor fusion (NO SLAM - odometry-based)
2. Verify all sensors working together
3. Test Nav2 configuration before adding SLAM
4. Prepare for later SLAM Toolbox 2.5D mapping integration

NAVIGATION MODES:
-----------------
Mode 1: Pure Odometry Navigation (default)
  - No map, navigation based on odometry only
  - Good for: Testing sensor fusion, short distances
  - Limitation: Will drift over time without SLAM corrections

Mode 2: With Pre-built Map (use_map:=true)
  - Load existing 2D map from SLAM Toolbox
  - Good for: Localization testing, long-distance navigation
  - Requires: Previously saved map file

FUTURE SLAM TOOLBOX INTEGRATION:
---------------------------------
To switch to SLAM Toolbox 2.5D mapping later:
1. Run this launch file as-is (provides all sensors + Nav2)
2. In separate terminal: ros2 launch slam_toolbox online_async_launch.py
3. SLAM Toolbox will:
   - Use /scan from LiDAR for 2D mapping
   - Optionally use /zed2i/zed_node/point_cloud for 2.5D features
   - Publish map->odom transform
   - Build map in real-time

USAGE:
------
# Basic - Pure odometry navigation (no map)
ros2 launch shbat_pkg sahabat_navigation_test.launch.py

# With pre-built map (localization mode)
ros2 launch shbat_pkg sahabat_navigation_test.launch.py use_map:=true

# With RViz visualization
ros2 launch shbat_pkg sahabat_navigation_test.launch.py use_rviz:=true

# All sensors + RViz + pre-built map
ros2 launch shbat_pkg sahabat_navigation_test.launch.py use_rviz:=true use_map:=true

# Disable external N100 IMU (if using ZED IMU only)
ros2 launch shbat_pkg sahabat_navigation_test.launch.py use_n100_imu:=false

TESTING CHECKLIST:
------------------
1. ✓ Launch this file
2. ✓ Check RViz - all TFs should be green
3. ✓ Verify topics publishing:
   - ros2 topic hz /scan (LiDAR - should be ~10 Hz)
   - ros2 topic hz /zed2i/zed_node/odom (ZED odometry - ~30 Hz)
   - ros2 topic hz /odometry/filtered (EKF output - ~50 Hz)
   - ros2 topic hz /zed2i/zed_node/point_cloud/cloud_registered (ZED depth)
4. ✓ Check costmaps in RViz (local + global)
5. ✓ Send 2D Nav Goal - robot should navigate
6. ✓ Monitor odometry drift (without SLAM, expect drift over long distances)

THEN INTEGRATE SLAM TOOLBOX:
-----------------------------
Once navigation works well with pure odometry:

Terminal 1 (keep this running):
  ros2 launch shbat_pkg sahabat_navigation_test.launch.py use_rviz:=true

Terminal 2 (add SLAM):
  ros2 launch slam_toolbox online_async_launch.py \\
    slam_params_file:=/home/sahabat/sahabat_ws/src/shbat_pkg/config/mapper_params_online_async.yaml \\
    use_sim_time:=false

Now you have:
- ✓ All sensors fused
- ✓ Nav2 navigation working
- ✓ SLAM Toolbox building 2.5D map in real-time
- ✓ No odometry drift (map->odom correction)

Author: ROS2 Navigation Stack
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package directories
    shbat_pkg_dir = get_package_share_directory('shbat_pkg')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    
    # Configuration files
    ekf_config = os.path.join(shbat_pkg_dir, 'config', 'ekf.yaml')
    nav2_params = os.path.join(shbat_pkg_dir, 'config', 'nav2_params.yaml')
    nav2_params_odom = os.path.join(shbat_pkg_dir, 'config', 'nav2_params_odom.yaml')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    use_map_arg = DeclareLaunchArgument(
        'use_map',
        default_value='false',
        description='Load pre-built map (true) or use odometry only (false)'
    )
    # Accept common typo alias 'use_maps' and treat it as use_map
    use_maps_arg = DeclareLaunchArgument(
        'use_maps',
        default_value='false',
        description='Alias of use_map (typo-safe)'
    )
    
    use_n100_imu_arg = DeclareLaunchArgument(
        'use_n100_imu',
        default_value='true',
        description='Use external Wheeltec N100 IMU (false to use ZED IMU only)'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Enable SLAM Toolbox for mapping and localization'
    )
    
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        choices=['mapping', 'localization'],
        description='SLAM mode: mapping (build new map) or localization (use existing map)'
    )
    
    # (Removed dynamic EKF toggle to keep stack simple and consistent)
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_map = LaunchConfiguration('use_map')
    use_maps = LaunchConfiguration('use_maps')
    # Effective map flag: true if either use_map or use_maps is true
    use_map_effective = PythonExpression([
        '"', use_map, '" == "true" or "', use_maps, '" == "true"'
    ])
    use_n100_imu = LaunchConfiguration('use_n100_imu')
    use_slam = LaunchConfiguration('use_slam')
    slam_mode = LaunchConfiguration('slam_mode')
    # (No EKF toggle LC)
    
    # ========================================================================
    # PART 1: BASE ROBOT (Motors, LiDAR, IMU, Wheel Odometry)
    # ========================================================================
    
    base_launch_file = os.path.join(shbat_pkg_dir, 'launch', 'sahabat_launch.py')
    
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            'use_n100_imu': use_n100_imu,
            'use_kalman_filter': 'true',  # Keep wheel+IMU fusion
            'publish_robot_state': 'true',  # Publish base robot URDF
        }.items()
    )
    
    # ========================================================================
    # PART 2: ZED2i CAMERA (Visual Odometry + Depth Point Cloud)
    # ========================================================================
    
    zed_launch_file = os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
    
    # Single ZED launch (EKF will publish odom->base_link, so ZED publish_tf=false)
    # ZED official robot integration pattern:
    # - base_frame: base_link
    # - pos_tracking_enabled: true
    # - publish_tf: false (EKF publishes odom->base_link)
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed2i',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'pos_tracking_enabled': 'true',
            'depth_mode': 'PERFORMANCE',
            'depth_stabilization': '1',
            'base_frame': 'base_link',
            'publish_urdf': 'false',
            'use_sim_time': 'false',
            'grab_resolution': 'VGA',
            'grab_frame_rate': '30',
        }.items()
    )
    
    # ========================================================================
    # PART 3: SENSOR FUSION (robot_localization EKF)
    # ========================================================================
    # Fuses:
    # - /wheel_odom (from kalman_filter.py - wheel encoders + N100 IMU)
    # - /zed2i/zed_node/odom (ZED visual odometry)
    # Output:
    # - /odometry/filtered (best fused estimate)
    # - odom->base_link TF
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            # EKF publishes to /odometry/filtered by default
        ]
    )
    
    # ========================================================================
    # PART 4: STATIC TRANSFORMS
    # ========================================================================
    
    # Static transform: base_link -> zed2i_camera_center (Stereolabs standard)
    # Physical mount: 5cm forward, 67cm up, flat (no tilt for now)
    zed_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_zed_tf',
        output='screen',
        arguments=[
            '0.05', '0.0', '0.67',     # x, y, z
            '0.0', '0.0', '0.0',        # roll, pitch, yaw (flat mount)
            'base_link',
            'zed2i_camera_center'
        ]
    )
    
    # Static transform: map -> odom (identity)
    # Only publish if NOT using SLAM (SLAM Toolbox will publish map->odom)
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        arguments=[
            '0.0', '0.0', '0.0',     # x, y, z
            '0.0', '0.0', '0.0',     # roll, pitch, yaw
            'map',
            'odom'
        ],
        condition=UnlessCondition(use_slam)
    )
    
    # ========================================================================
    # PART 5: NAV2 NAVIGATION STACK
    # ========================================================================
    
    nav2_launch_file = os.path.join(shbat_pkg_dir, 'launch', 'sahabat_nav.launch.py')
    
    # Nav2 with map mode
    nav2_launch_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(use_map_effective)
    )

    # Nav2 odom-only mode (no static_layer, odom frames)
    nav2_launch_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_odom,
            'autostart': 'true',
        }.items(),
        condition=UnlessCondition(use_map_effective)
    )
    
    # Delay Nav2 bringup briefly to allow EKF/ZED to publish odom
    nav2_delayed = TimerAction(
        period=5.0,
        actions=[nav2_launch_map, nav2_launch_odom]
    )
    
    # ========================================================================
    # PART 6: SLAM TOOLBOX (Optional - for mapping and localization)
    # ========================================================================
    
    slam_params_file = os.path.join(shbat_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': False,
                'mode': slam_mode,  # 'mapping' or 'localization'
            }
        ],
        condition=IfCondition(use_slam)
    )
    
    # ========================================================================
    # PART 7: MAP SERVER (Optional - only if using pre-built map WITHOUT SLAM)
    # ========================================================================
    
    map_file = os.path.join(shbat_pkg_dir, 'map', 'map_save.yaml')
    
    # Only use map_server if use_map=true AND NOT using SLAM
    # (SLAM Toolbox handles map publishing when enabled)
    use_map_server = PythonExpression([
        '("', use_map_effective, '" == "True" or "', use_map_effective, '" == "true") and "', use_slam, '" != "true"'
    ])
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': False}
        ],
        condition=IfCondition(use_map_server)
    )
    
    # Lifecycle manager for map_server
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']}
        ],
        condition=IfCondition(use_map_server)
    )
    
    # ========================================================================
    # PART 8: RVIZ2 VISUALIZATION
    # ========================================================================
    
    rviz_config = os.path.join(shbat_pkg_dir, 'rviz', 'navigation_test.rviz')
    # Fallback to default config if custom doesn't exist
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(shbat_pkg_dir, 'rviz', 'rtabmap.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz)
    )
    
    # ========================================================================
    # BUILD LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_map_arg,
        use_maps_arg,
        use_n100_imu_arg,
        use_slam_arg,
        slam_mode_arg,
        
        # Base robot (motors, LiDAR, wheel odom, kalman filter)
        base_robot_launch,
        
    # ZED2i camera (visual odometry, depth, point cloud)
    zed_camera_launch,
        
        # Sensor fusion (EKF)
        ekf_node,
        
        # Static transforms
        zed_base_link_tf,
        map_odom_tf,  # Add map->odom for planner (only if no SLAM)
        
        # Nav2 navigation stack (delayed)
        nav2_delayed,
        
        # SLAM Toolbox (optional)
        slam_toolbox_node,
        
        # Map server (optional - only if not using SLAM)
        map_server_node,
        map_lifecycle_manager,
        
        # RViz (optional)
        rviz_node,
    ])
