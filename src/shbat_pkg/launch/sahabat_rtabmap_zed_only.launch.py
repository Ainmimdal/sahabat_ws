#!/usr/bin/env python3
"""
RTAB-Map launch file using only a ZED camera for sensing and odometry.

This launch file brings up:
- ZED2i camera wrapper (for visual odometry and sensor data)
- Robot State Publisher (to publish the TF tree from URDF)
- RTAB-Map for SLAM
- Optional: RViz and RTAB-Map's visualization tool
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
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
            " use_zed_localization:=true", # ZED publishes odom->camera_link, camera is root
        ]
    )
    robot_description_param = ParameterValue(robot_description, value_type=str)

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz2 for visualization"
    )
    use_rtabmap_viz_arg = DeclareLaunchArgument(
        "use_rtabmap_viz",
        default_value="false",
        description="Launch RTAB-Map visualization GUI",
    )
    delete_db_arg = DeclareLaunchArgument(
        "delete_db",
        default_value="false",
        description="Delete existing RTAB-Map database to start fresh",
    )
    mem_incremental_arg = DeclareLaunchArgument(
        "mem_incremental",
        default_value="true",
        description="RTAB-Map Mem/IncrementalMemory (true=mapping, false=localization)",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    delete_db = LaunchConfiguration("delete_db")
    mem_incremental = LaunchConfiguration("mem_incremental")

    # Set RTAB-Map memory limit
    rtabmap_env = SetEnvironmentVariable("RTABMAP_MAX_MEMORY", "4096")

    # Base Robot Launch (motors, IMU, LiDAR)
    # Tell it NOT to publish robot_state - we handle that with ZED URDF
    base_launch_file = os.path.join(shbat_pkg_dir, 'launch', 'sahabat_launch.py')
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={'publish_robot_state': 'false', 'use_n100_imu': 'false', 'use_kalman_filter': 'false'}.items()
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
            "publish_tf": "true",  # ZED publishes odom->camera_link
            "publish_map_tf": "false", # RTAB-Map handles map->odom
            "publish_urdf": "false",
            "pos_tracking_enabled": "true",
            "sensors_fusion": "true", # Enable IMU fusion
            "ros_params_override_path": zed_override_params,
        }.items(),
    )

    # RGBD Sync Node
    rgbd_sync_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        output="screen",
        parameters=[
            {
                "frame_id": "zed2i_camera_link",  # Match RTAB-Map frame_id
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

    # Static TF for IMU link - NOT NEEDED, ZED wrapper already publishes this
    # static_imu_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_imu_tf",
    #     output="screen",
    #     arguments=["0", "0", "0", "0", "0", "0", "zed2i_camera_link", "zed2i_imu_link"],
    # )

    # RTAB-Map parameters from official ZED ROS2 examples
    # https://github.com/stereolabs/zed-ros2-examples/blob/master/examples/zed_rtabmap_launch/launch/zed_rtabmap.launch.py
    rtabmap_common_params = {
        'frame_id': 'zed2i_camera_link',  # ZED is the odometry root
        'subscribe_depth': False,
        'subscribe_rgbd': True,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'subscribe_user_data': False,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'publish_tf': True, # RTAB-Map publishes map->odom, ZED publishes odom->base_link
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'wait_for_transform': 0.2,
        'queue_size': 100,

        # Performance
        'RGBD/NeighborLinkRefining': 'true',
        'Reg/Strategy': '1',  # 0=Visual, 1=ICP
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.05',
        'Vis/MaxFeatures': '1000',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/MaxRotation': '0.78',
        'Icp/MaxTranslation': '0.2',
        'Icp/CorrespondenceRatio': '0.2',

        # Loop Closure
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/LoopClosureReextractFeatures': 'true',
        'Mem/RehearsalSimilarity': '0.45',

        # Graph
        'RGBD/OptimizeFromGraphEnd': 'false',

        'Mem/IncrementalMemory': ParameterValue(mem_incremental, value_type=str),
    }

    # RTAB-Map SLAM Node
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_common_params],
        remappings=[
            ("odom", "/zed2i/zed_node/odom"),
            ("imu", "/zed2i/zed_node/imu/data"),
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
            ("imu", "/zed2i/zed_node/imu/data"),
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
                "frame_id": "zed2i_camera_link",  # Match RTAB-Map frame_id
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
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_rtabmap_viz_arg,
            delete_db_arg,
            mem_incremental_arg,
            rtabmap_env,
            base_robot_launch,
            robot_state_publisher_node,
            zed_wrapper_launch,
            rgbd_sync_node,
            # static_imu_tf,  # Not needed - ZED wrapper handles this
            rtabmap_node,
            rtabmap_node_fresh,
            rtabmap_viz_node,
            rviz_node,
        ]
    )
