#!/usr/bin/env python3
"""
RTAB-Map launch file for offline mapping from rosbag playback.

This launch file brings up RTAB-Map to process pre-recorded data.
Play your rosbag separately, and RTAB-Map will process it.

Usage:
  # Terminal 1: Launch RTAB-Map
  ros2 launch shbat_pkg sahabat_rtabmap_from_bag.launch.py

  # Terminal 2: Play your rosbag
  ros2 bag play your_bag_folder --clock
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shbat_pkg_dir = get_package_share_directory("shbat_pkg")

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="Launch RViz2 for visualization"
    )
    use_rtabmap_viz_arg = DeclareLaunchArgument(
        "use_rtabmap_viz",
        default_value="true",
        description="Launch RTAB-Map visualization GUI (recommended for offline mapping)",
    )
    delete_db_arg = DeclareLaunchArgument(
        "delete_db",
        default_value="false",
        description="Delete existing RTAB-Map database to start fresh",
    )
    database_path_arg = DeclareLaunchArgument(
        "database_path",
        default_value=os.path.expanduser("~/.ros/rtabmap_from_bag.db"),
        description="Path to RTAB-Map database file",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    delete_db = LaunchConfiguration("delete_db")
    database_path = LaunchConfiguration("database_path")

    # Set RTAB-Map memory limit for Jetson
    rtabmap_env = SetEnvironmentVariable("RTABMAP_MAX_MEMORY", "2048")

    # RGBD Sync Node
    rgbd_sync_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,  # CRITICAL: use bag time
                "frame_id": "zed2i_camera_link",
                "approx_sync": True,
                "approx_sync_max_interval": 0.05,
                "sync_queue_size": 30,
            }
        ],
        remappings=[
            ("rgb/image", "/zed2i/zed_node/rgb/image_rect_color"),
            ("rgb/camera_info", "/zed2i/zed_node/rgb/camera_info"),
            ("depth/image", "/zed2i/zed_node/depth/depth_registered"),
        ],
    )

    # RTAB-Map parameters
    rtabmap_common_params = {
        'use_sim_time': True,  # CRITICAL: use bag time
        'frame_id': 'zed2i_camera_link',
        'subscribe_depth': False,
        'subscribe_rgbd': True,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'subscribe_imu': False,  # Disable IMU to avoid TF warnings
        'subscribe_user_data': False,
        'subscribe_odom_info': False,  # Disable - not in bag
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,  # Allow 100ms sync window for bag playback
        'publish_tf': True,  # Publish map->odom
        'map_frame_id': 'map',
        'odom_frame_id': 'odom', 
        'wait_for_transform': 0.2,
        'wait_imu_to_init': False,
        'sync_queue_size': 20,  # Larger queue for bag timing
        'queue_size': 20,
        'database_path': database_path,

        # OPTIMIZED FOR JETSON ORIN NANO - Reduced computational load
        'Reg/Strategy': '0',  # Visual for loop closure verification
        'Reg/Force3DoF': 'true',  # Force 2D mode
        'RGBD/NeighborLinkRefining': 'false',  # Don't refine - saves compute
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.2',  # Fewer keyframes = less compute
        'RGBD/LinearUpdate': '0.2',  # Fewer keyframes = less compute
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '3.0',
        
        # Reduced visual features for Jetson
        'Vis/MinInliers': '15',  # Less strict
        'Vis/InlierDistance': '0.1',
        'Vis/MaxFeatures': '200',  # REDUCED from 400 - major speedup!
        'Vis/FeatureType': '6',  # GFTT (fast)
        'Vis/MaxDepth': '4.0',  # Limit depth range to process
        
        # Lighter graph optimization
        'RGBD/OptimizeStrategy': '0',  # TORO (faster than g2o)
        'Optimizer/Strategy': '0',  # TORO
        'Optimizer/Epsilon': '0.001',  # Less precise but faster
        'Optimizer/Iterations': '10',  # REDUCED from 20
        
        # Loop closure
        'RGBD/LoopClosureReextractFeatures': 'false',  # Don't reextract - saves time
        'Mem/STMSize': '20',  # Smaller short-term memory
        'Kp/MaxFeatures': '200',  # REDUCED
        'Mem/ImageKept': 'false',  # Don't keep images in memory

        # Loop Closure
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/LoopClosureReextractFeatures': 'true',
        'Mem/RehearsalSimilarity': '0.45',

        # Graph
        'RGBD/OptimizeFromGraphEnd': 'false',

        'Mem/IncrementalMemory': 'true',  # Mapping mode (must be string)
    }

    # RTAB-Map SLAM Node - Use odometry from bag
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_common_params],
        remappings=[
            ("odom", "/zed2i/zed_node/odom"),
        ],
        condition=UnlessCondition(delete_db),
    )

    rtabmap_node_fresh = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_common_params],
        remappings=[
            ("odom", "/zed2i/zed_node/odom"),
        ],
        arguments=["--delete_db_on_start"],
        condition=IfCondition(delete_db),
    )

    # RTAB-Map Visualization
    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "frame_id": "zed2i_camera_link",
                "subscribe_rgbd": True,
                "subscribe_scan": False,
                "subscribe_odom_info": False,
                "approx_sync": True,
            }
        ],
        remappings=[("odom", "/zed2i/zed_node/odom")],
        condition=IfCondition(use_rtabmap_viz),
    )

    # RViz
    rviz_config = os.path.join(shbat_pkg_dir, "rviz", "zed_rtabmap.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_rtabmap_viz_arg,
            delete_db_arg,
            database_path_arg,
            rtabmap_env,
            rgbd_sync_node,
            rtabmap_node,
            rtabmap_node_fresh,
            rtabmap_viz_node,
            rviz_node,
        ]
    )
