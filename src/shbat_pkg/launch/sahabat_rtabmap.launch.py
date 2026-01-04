#!/usr/bin/env python3
"""
RTAB-Map + ZED2i Launch File for Gallery Tour Guide Robot - Pure ZED Localization

This launch file provides complete robot operation with pure ZED visual localization.
It launches:
- Base robot (motors, IMU, LiDAR)
- ZED2i camera wrapper (with visual odometry and TF publishing)
- RTAB-Map visual SLAM node  
- Robot State Publisher (publishes robot URDF with ZED attached)
- RViz2 for visualization (optional)

PURE ZED LOCALIZATION ARCHITECTURE:
- ZED wrapper: publishes /zed2i/zed_node/odom + TF (odom → zed2i_camera_link)
- URDF: defines zed2i_camera_link → base_link (camera is root)
- Robot State Publisher: publishes robot TF tree from URDF
- RTAB-Map: uses ZED odometry directly for SLAM
- Nav2: uses ZED odometry for navigation

TF Tree: map → odom → zed2i_camera_link → base_link → [robot parts]

Usage:
    # NORMAL OPERATION - Build a map:
    ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rviz:=true
    
    # With RTAB-Map visualization:
    ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rtabmap_viz:=true
    
    # Start fresh (delete old map):
    ros2 launch shbat_pkg sahabat_rtabmap.launch.py delete_db:=true
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
    # Prefer source config during development; fallback to installed share
    pkg_src_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    rtabmap_config_src = os.path.join(pkg_src_root, 'config', 'rtabmap_params.yaml')
    rtabmap_config = rtabmap_config_src if os.path.exists(rtabmap_config_src) \
        else os.path.join(shbat_pkg_dir, 'config', 'rtabmap_params.yaml')
    
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
    use_network_arg = DeclareLaunchArgument(
        'use_network',
        default_value='false',
        description='Use network/IP camera instead of USB'
    )
    
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.1.10',
        description='IP address of ZED2i camera (only if use_network:=true)'
    )
    
    serial_number_arg = DeclareLaunchArgument(
        'serial_number',
        default_value='0',
        description='Serial number of USB ZED camera (0=first available)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    use_rtabmap_viz_arg = DeclareLaunchArgument(
        'use_rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map visualization GUI'
    )
    
    delete_db_arg = DeclareLaunchArgument(
        'delete_db',
        default_value='false',
        description='Delete existing RTAB-Map database to start fresh'
    )

    # Mapping vs Localization toggle for RTAB-Map
    mem_incremental_arg = DeclareLaunchArgument(
        'mem_incremental',
        default_value='true',
        description='RTAB-Map Mem/IncrementalMemory (true=mapping, false=localization)'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
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
    # Tell it NOT to publish robot_state - we handle that with ZED URDF
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
    # Following official rtabmap_examples/zed.launch.py
    # Flat mount: disable floor_alignment compensation
    # Override ZED positional tracking to lock Z and align with gravity
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
    # This is REQUIRED by official example
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[{
            'frame_id': 'zed2i_camera_link',
            'approx_sync': True,
            'approx_sync_max_interval': 0.01,  # Suppress sync warnings
            'sync_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/zed2i/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed2i/zed_node/rgb/camera_info'),
            ('depth/image', '/zed2i/zed_node/depth/depth_registered'),
        ]
    )

    # Provide IMU TF if URDF doesn't include it (some variants of the robot URDF may omit zed2i_imu_link).
    # Align with ZED ROS2 wrapper conventions: zed2i_camera_link -> zed2i_imu_link (identity).
    static_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_imu_tf',
        output='screen',
        arguments=['0','0','0','0','0','0','zed2i_camera_link','zed2i_imu_link']
    )
    
    # Compose minimal RTAB-Map parameters inline to avoid parsing external YAML files
    # that may be malformed in install space. This covers the critical settings.
    rtabmap_common_params = {
        'publish_tf': True,
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'frame_id': 'zed2i_camera_link',
        'subscribe_rgbd': True,
        'subscribe_scan': False, # VISUALS-ONLY TEST: Disable LiDAR
        'subscribe_imu': False, # DISABLED - ZED odom already includes IMU fusion
        'subscribe_odom_info': False,
        'approx_sync': True,
        'sync_queue_size': 30,
    # Allow more time to fetch TF to avoid short extrapolation windows
    'wait_for_transform': 1.0,
        # Constrain pose estimation to planar motion to reduce lateral drift at rest
        'Reg/Force3DoF': 'true',
        # Stronger alignment: use Visual + ICP and refine constraints with ICP
        'Reg/Strategy': '0',                    # VISUALS-ONLY TEST: 0=Visual, 1=ICP, 2=Visual+ICP
        'Icp/PointToPlane': 'true',
        'Icp/VoxelSize': '0.05',               # Downsample for robust ICP
        'Icp/MaxCorrespondenceDistance': '0.20',
        'Icp/Iterations': '30',
        'Icp/CorrespondenceRatio': '0.2',
        # Enable spatial proximity loop closures to fix walls when revisiting
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '30',
        'RGBD/ProximityMaxGraphDepth': '100',
        'RGBD/LoopClosureReextractFeatures': 'true',
        # Make feature matching a bit more permissive & denser
        'Vis/MinInliers': '15',
        'Vis/MaxFeatures': '1500',
        # Increase thresholds so no new keyframes are added for tiny motions/noise
        'RGBD/LinearUpdate': '0.30',
        'RGBD/AngularUpdate': '0.30',
        # Reduce map shifts at stop: optimize from graph end so last pose moves less
        'RGBD/OptimizeFromGraphEnd': 'true',
        # Make optimizer more tolerant while stabilizing IMU/TF wiring
        'Optimizer/Robust': 'true',
        'Optimizer/Epsilon': '0',
        'RGBD/OptimizeMaxError': '50',
        # Force string type so rtabmap_slam (which declares this as string) 
        # doesn't reject a coerced boolean value.
        'Mem/IncrementalMemory': ParameterValue(mem_incremental, value_type=str),
        'Mem/InitWMWithAllNodes': 'false',
    }

    # RTAB-Map SLAM Node
    # Following official example: frame_id should be CAMERA link
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_common_params],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/zed2i/zed_node/odom'),
            ('imu', '/zed2i/zed_node/imu/data'),
        ],
        condition=UnlessCondition(delete_db)
    )
    
    # RTAB-Map SLAM Node (with delete_db_on_start)
    rtabmap_node_fresh = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_common_params],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/zed2i/zed_node/odom'),
            ('imu', '/zed2i/zed_node/imu/data'),
        ],
        arguments=['--delete_db_on_start'],
        condition=IfCondition(delete_db)
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
                'subscribe_scan': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'sync_queue_size': 30,
                'Mem/InitWMWithAllNodes': 'false',
            }
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/zed2i/zed_node/odom'),
        ],
        condition=IfCondition(use_rtabmap_viz)
    )
    
    # RViz2 for navigation visualization
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
    use_rtabmap_viz_arg,
        delete_db_arg,
    mem_incremental_arg,
        
        # Environment
        rtabmap_env,
        
        # Nodes
        base_robot_launch,          # Base robot (motors, LiDAR, IMU)
        robot_state_publisher_node, # Publishes robot URDF with ZED
        zed_wrapper_launch,         # ZED camera
        rgbd_sync_node,             # Sync RGB+Depth (REQUIRED)
    static_imu_tf,              # Ensure IMU frame exists for ZED
        rtabmap_node,               # Visual SLAM (continues previous map)
        rtabmap_node_fresh,         # Visual SLAM (deletes old map) - conditional
        rtabmap_viz,                # RTAB-Map visualization (optional)
        rviz_node,                  # RViz2 visualization (optional)
    ])
