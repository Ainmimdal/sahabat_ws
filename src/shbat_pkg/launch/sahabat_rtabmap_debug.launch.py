#!/usr/bin/env python3
"""
RTAB-Map Debug Launch File - Visuals-Only Mode

This launch file runs RTAB-Map with only RGB-D and Odometry inputs,
disabling the IMU and LiDAR scan to isolate visual SLAM performance.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Get package directories
    shbat_pkg_dir = get_package_share_directory('shbat_pkg')
    pkg_src_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    
    # Process URDF with xacro
    urdf_file = os.path.join(shbat_pkg_dir, 'urdf', 'sahabat_robot_with_zed.urdf.xacro')
    robot_description = Command([
        'xacro ', urdf_file,
        ' camera_name:=zed2i',
        ' camera_model:=zed2i',
        ' use_zed_localization:=true'
    ])
    robot_description_param = ParameterValue(robot_description, value_type=str)
    
    # Launch arguments
    use_rtabmap_viz_arg = DeclareLaunchArgument(
        'use_rtabmap_viz',
        default_value='true',
        description='Launch RTAB-Map visualization GUI'
    )
    
    delete_db_arg = DeclareLaunchArgument(
        'delete_db',
        default_value='true',
        description='Delete existing RTAB-Map database to start fresh'
    )

    mem_incremental_arg = DeclareLaunchArgument(
        'mem_incremental',
        default_value='true',
        description='RTAB-Map Mem/IncrementalMemory (true=mapping, false=localization)'
    )
    
    # Get launch configurations
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    delete_db = LaunchConfiguration('delete_db')
    mem_incremental = LaunchConfiguration('mem_incremental')
    
    # Set RTAB-Map to use less memory on Jetson
    rtabmap_env = SetEnvironmentVariable('RTABMAP_MAX_MEMORY', '4096')
    
    # Get ZED wrapper launch file
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_launch_file = os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
    
    # Get base robot launch file (includes motors, IMU, LiDAR, wheel odometry)
    base_launch_file = os.path.join(shbat_pkg_dir, 'launch', 'sahabat_launch.py')
    
    # Base Robot Launch (motors, IMU, LiDAR)
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={'publish_robot_state': 'false', 'use_n100_imu': 'false', 'use_kalman_filter': 'false'}.items()
    )
    
    # Robot State Publisher - publishes robot TF tree from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}]
    )
    
    # ZED Camera Launch
    zed_override_src = os.path.join(pkg_src_root, 'config', 'zed_pos_tracking_override.yaml')
    zed_override_params = zed_override_src if os.path.exists(zed_override_src) \
        else os.path.join(shbat_pkg_dir, 'config', 'zed_pos_tracking_override.yaml')

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
    
    # RGBD Sync Node - syncs RGB + Depth + Camera Info
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{
            'frame_id': 'zed2i_camera_link',
            'approx_sync': True,
            'approx_sync_max_interval': 0.01,
            'sync_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/zed2i/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed2i/zed_node/rgb/camera_info'),
            ('depth/image', '/zed2i/zed_node/depth/depth_registered'),
        ]
    )
    
    # RTAB-Map parameters for visuals-only mode, with advanced tuning
    rtabmap_common_params = {
        'publish_tf': True,
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'frame_id': 'zed2i_camera_link',
        'subscribe_rgbd': True,
        'subscribe_scan': False, # DISABLED
        'subscribe_imu': False,  # DISABLED
        'subscribe_odom_info': False,
        'approx_sync': True,
        'sync_queue_size': 30,
        'wait_for_transform': 1.0,
        # Core visual SLAM tuning from the main launch file
        'Reg/Force3DoF': 'true',
        'Reg/Strategy': '2',
        'Icp/PointToPlane': 'true',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.20',
        'Icp/Iterations': '30',
        'Icp/CorrespondenceRatio': '0.2',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '30',
        'RGBD/ProximityMaxGraphDepth': '100',
        'RGBD/LoopClosureReextractFeatures': 'true',
        'Vis/MinInliers': '15',
        'Vis/MaxFeatures': '1500',
        'RGBD/LinearUpdate': '0.30',
        'RGBD/AngularUpdate': '0.30',
        'RGBD/OptimizeFromGraphEnd': 'true',
        'Optimizer/Robust': 'true',
        'Optimizer/Epsilon': '0',
        'RGBD/OptimizeMaxError': '50',
        'Mem/IncrementalMemory': ParameterValue(mem_incremental, value_type=str),
    }

    # RTAB-Map SLAM Node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_common_params],
        remappings=[
            ('odom', '/zed2i/zed_node/odom'),
        ],
        arguments=['--delete_db_on_start' if delete_db else '']
    )
    
    # RTAB-Map Visualization Node
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            {
                'frame_id': 'zed2i_camera_link',
                'subscribe_rgbd': True,
                'subscribe_scan': False, # DISABLED
                'subscribe_odom_info': False,
                'approx_sync': True,
                'sync_queue_size': 30,
            }
        ],
        remappings=[
            ('odom', '/zed2i/zed_node/odom'),
        ],
        condition=IfCondition(use_rtabmap_viz)
    )
    
    return LaunchDescription([
        use_rtabmap_viz_arg,
        delete_db_arg,
        mem_incremental_arg,
        rtabmap_env,
        base_robot_launch,
        robot_state_publisher_node,
        zed_wrapper_launch,
        rgbd_sync_node,
        rtabmap_node,
        rtabmap_viz,
    ])
