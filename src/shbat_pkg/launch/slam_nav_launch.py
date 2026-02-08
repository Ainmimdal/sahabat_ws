"""
SLAM Toolbox Navigation Launch File

This launch file provides full navigation with 2D SLAM mapping/localization.
Optimized for Orin Nano with async SLAM processing.

Usage:
  # Mapping mode (create new map)
  ros2 launch shbat_pkg slam_nav_launch.py

  # Localization mode (use saved map)
  ros2 launch shbat_pkg slam_nav_launch.py mode:=localization map_file:=/path/to/map

  # Save map after mapping
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/sahabat/maps/my_map'}}"

What this launches:
  1. Robot state publisher (URDF)
  2. Base controller (motor driver + wheel odometry)
  3. LIDAR driver + scan filter
  4. IMU driver
  5. EKF sensor fusion
  6. slam_toolbox (mapping or localization)
  7. Nav2 stack
  8. RViz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import xacro
import yaml

# Import USB detection
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from sahabat_launch import smart_detect_devices
    lidar_port, imu_port, motor_port = smart_detect_devices()
except Exception as e:
    print(f"Warning: Could not import device detection: {e}")
    lidar_port, imu_port, motor_port = None, None, '/dev/ttyUSB0'

# Try to load saved pose from previous session
SAVED_POSE_FILE = os.path.expanduser('~/.ros/sahabat_saved_pose.yaml')
saved_pose = {'initial_pose_x': 0.0, 'initial_pose_y': 0.0, 'initial_pose_yaw': 0.0}
try:
    if os.path.exists(SAVED_POSE_FILE):
        with open(SAVED_POSE_FILE, 'r') as f:
            saved_pose = yaml.safe_load(f)
            print(f"[slam_nav_launch] Loaded saved pose: x={saved_pose['initial_pose_x']}, y={saved_pose['initial_pose_y']}, yaw={saved_pose['initial_pose_yaw']}")
except Exception as e:
    print(f"[slam_nav_launch] Could not load saved pose: {e}")


def generate_launch_description():
    
    pkg_name = 'shbat_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    # ========== Launch Arguments ==========
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping or localization'
    )
    mode = LaunchConfiguration('mode')
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map file for localization (without extension)'
    )
    map_file = LaunchConfiguration('map_file')
    
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true' if lidar_port else 'false',
        description='Enable LIDAR'
    )
    use_lidar = LaunchConfiguration('use_lidar')
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true' if imu_port else 'false',
        description='Enable IMU'
    )
    use_imu = LaunchConfiguration('use_imu')
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    use_rviz = LaunchConfiguration('use_rviz')
    
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='false',
        description='Launch Foxglove bridge for remote visualization'
    )
    use_foxglove = LaunchConfiguration('use_foxglove')
    
    use_zed_arg = DeclareLaunchArgument(
        'use_zed',
        default_value='false',
        description='Enable ZED camera for obstacle detection (depth to laserscan)'
    )
    use_zed = LaunchConfiguration('use_zed')
    
    # Initial pose for localization mode - defaults to saved pose from last session
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value=str(saved_pose.get('initial_pose_x', 0.0)),
        description='Initial X position for localization (auto-loaded from saved pose)'
    )
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value=str(saved_pose.get('initial_pose_y', 0.0)),
        description='Initial Y position for localization (auto-loaded from saved pose)'
    )
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    
    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value=str(saved_pose.get('initial_pose_yaw', 0.0)),
        description='Initial yaw (radians) for localization (auto-loaded from saved pose)'
    )
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    
    motor_port_arg = DeclareLaunchArgument(
        'motor_port',
        default_value=motor_port if motor_port else '/dev/ttyUSB0',
        description='Motor controller serial port'
    )
    motor_port_cfg = LaunchConfiguration('motor_port')
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=lidar_port if lidar_port else '/dev/ttyUSB1',
        description='LIDAR serial port'
    )
    lidar_port_cfg = LaunchConfiguration('lidar_port')
    
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value=imu_port if imu_port else '/dev/ttyUSB2',
        description='IMU serial port'
    )
    imu_port_cfg = LaunchConfiguration('imu_port')

    # ========== Robot Description (URDF) ==========
    
    # Use ZED URDF when use_zed is enabled (includes ZED camera frames)
    # Note: We need to process this at launch time based on use_zed arg
    # For simplicity, we'll use the non-ZED URDF and add ZED TF via static publisher
    xacro_file = os.path.join(pkg_share, 'urdf', 'sahabat_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # ========== Base Controller ==========
    
    base_controller = Node(
        package='shbat_pkg',
        executable='base_controller',
        name='base_controller',
        output='screen',
        parameters=[
            {'port': motor_port_cfg},
            {'baudrate': 115200},
            {'wheel_radius': 0.0875},  # 175mm diameter / 2
            {'wheel_base': 0.33},
            {'publish_odom_tf': False},     # EKF publishes TF
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'odom_topic': 'wheel_odom'},
            {'accel_time_ms': 200},
            {'decel_time_ms': 200},
            {'max_linear_vel': 0.5},
            {'max_angular_vel': 1.5},
            {'cmd_vel_timeout': 0.5},
            {'odom_rate': 20.0},
        ]
    )

    # ========== LIDAR ==========
    
    lidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='oradar_scan_node',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar_link'},
            {'scan_topic': 'scan_raw'},
            {'port_name': lidar_port_cfg},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.0},
            {'range_max': 12.0},
            {'clockwise': False},
            {'motor_speed': 10}
        ],
        condition=IfCondition(use_lidar)
    )
    
    scan_filter_config = os.path.join(pkg_share, 'config', 'scan_filter.yaml')
    scan_filter_node = Node(
        package='shbat_pkg',
        executable='scan_filter',
        name='scan_filter',
        output='screen',
        parameters=[scan_filter_config],
        condition=IfCondition(use_lidar)
    )

    # ========== IMU ==========
    
    imu_node = Node(
        package='wheeltec_n100_imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'serial_port': imu_port_cfg,
            'serial_baud': 921600,
            'imu_topic': 'imu',
            'imu_frame': 'imu_link',
        }],
        condition=IfCondition(use_imu)
    )

    # ========== ZED Camera (Obstacle Detection) ==========
    
    zed_depth_config = os.path.join(pkg_share, 'config', 'zed_depth_to_scan.yaml')
    
    # ZED common config
    zed_config_common = os.path.join(
        get_package_share_directory('zed_wrapper'), 'config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(
        get_package_share_directory('zed_wrapper'), 'config', 'zed2i.yaml')
    
    # Static transform: base_link -> zed_camera_link
    # Adjust these values based on where ZED is mounted on your robot!
    # x=forward, y=left, z=up (in meters)
    # base_link is at wheel axle height (8.75cm above floor for 175mm wheels)
    # ZED is 70cm from floor, so Z = 70 - 8.75 = 61.25cm
    zed_static_tf = Node(
        condition=IfCondition(use_zed),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_base_link_tf',
        arguments=[
            '--x', '0.05',      # 5cm forward from base_link
            '--y', '0.0',       # centered
            '--z', '0.6125',    # 61.25cm above base_link (70cm from floor - 8.75cm axle height)
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed2i_camera_link',  # Match ZED camera_name
        ],
    )
    
    # ZED Wrapper component
    # Position tracking disabled - we use our own EKF for odometry
    # Internal camera TFs will be published via additional static transforms below
    zed_wrapper_component = ComposableNode(
        package='zed_components',
        namespace='zed',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            zed_config_common,
            zed_config_camera,
            {
                'general.camera_name': 'zed2i',
                'general.camera_model': 'zed2i',
                'general.grab_resolution': 'VGA',
                'general.grab_frame_rate': 15,
                'general.pub_frame_rate': 15.0,
                # Depth settings
                'depth.depth_mode': 'PERFORMANCE',
                'depth.min_depth': 0.3,
                'depth.max_depth': 10.0,
                'depth.depth_stabilization': 0,         # Must be 0 when pos_tracking disabled
                'depth.point_cloud_freq': 10.0,
                # Position tracking - DISABLED (we use EKF)
                'pos_tracking.pos_tracking_enabled': False,
                'pos_tracking.publish_tf': False,
                'pos_tracking.publish_map_tf': False,
                # Disable features we don't need
                'object_detection.od_enabled': False,
                'body_tracking.bt_enabled': False,
                'sensors.publish_imu_tf': False,
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # ZED internal camera TFs (since pos_tracking is disabled, ZED won't publish these)
    # These are the transforms from zed_camera_link to the optical frames
    # Values from ZED 2i specs: left camera is 6cm from center
    zed_left_camera_tf = Node(
        condition=IfCondition(use_zed),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_left_camera_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.06',      # Left camera is 6cm to the left
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'zed2i_camera_link',
            '--child-frame-id', 'zed2i_left_camera_frame',
        ],
    )
    
    # Optical frame has different orientation (Z forward, X right, Y down)
    zed_left_optical_tf = Node(
        condition=IfCondition(use_zed),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_left_optical_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '-1.5708',   # -90 degrees
            '--pitch', '0.0',
            '--yaw', '-1.5708',    # -90 degrees
            '--frame-id', 'zed2i_left_camera_frame',
            '--child-frame-id', 'zed2i_left_camera_optical_frame',
        ],
    )
    
    # Container for ZED components (just ZED wrapper, pointcloud goes directly to costmap)
    zed_container = ComposableNodeContainer(
        condition=IfCondition(use_zed),
        name='zed_container',
        namespace='zed',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            zed_wrapper_component,
            # depth_to_laserscan removed - using VoxelLayer with PointCloud2 instead
        ],
        output='screen',
    )

    # ========== EKF Sensor Fusion ==========
    
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', 'odom'),
        ]
    )

    # ========== SLAM Toolbox (Mapping Mode Only) ==========
    
    slam_config_mapping = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    
    # Mapping mode - use slam_toolbox
    slam_toolbox_mapping = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_mapping],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    )
    
    # ========== AMCL + Map Server (Localization Mode) ==========
    
    amcl_config = os.path.join(pkg_share, 'config', 'amcl.yaml')
    
    # Static map->odom transform (identity) - used until AMCL takes over
    # This prevents TF errors during startup
    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    )
    
    # Map server - serves the saved map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            amcl_config,
            {'yaml_filename': [map_file, '.yaml']}
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    )
    
    # AMCL - Adaptive Monte Carlo Localization (supports 2D Pose Estimate!)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                # If initial pose is provided via launch args, use it
                # Otherwise falls back to global localization from amcl.yaml
                'set_initial_pose': True,
                'initial_pose.x': initial_pose_x,
                'initial_pose.y': initial_pose_y,
                'initial_pose.yaw': initial_pose_yaw,
            }
        ],
        remappings=[
            ('scan', '/scan')
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    )
    
    # Lifecycle manager for map_server and amcl
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'use_sim_time': False,
            'node_names': ['map_server', 'amcl']
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    )

    # ========== Joystick Control ==========
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    joy2cmd = Node(
        package='shbat_pkg',
        executable='joy2cmd',
        name='joy2cmd',
        output='screen'
    )

    # ========== Nav2 Stack ==========
    
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_odom_only.yaml')
    
    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )
    
    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    nav2_smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config]
    )
    
    nav2_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config]
    )
    
    nav2_velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )
    
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': nav2_lifecycle_nodes},
            {'bond_timeout': 0.0}
        ]
    )

    # ========== RViz ==========
    
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_nav.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = ''
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if rviz_config else [],
        condition=IfCondition(use_rviz)
    )

    # ========== Foxglove Bridge (for remote visualization) ==========
    
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
            'use_compression': True,
        }],
        condition=IfCondition(use_foxglove)
    )

    # ========== Auto Pose Saver ==========
    # Runs in background, saves pose whenever you use 2D Pose Estimate in RViz
    # Next launch will automatically use the saved pose
    
    auto_pose_saver = Node(
        package='shbat_pkg',
        executable='pose_saver_auto',
        name='auto_pose_saver',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    )

    # ========== Return Launch Description ==========
    
    return LaunchDescription([
        # Arguments
        mode_arg,
        map_file_arg,
        use_lidar_arg,
        use_imu_arg,
        use_rviz_arg,
        use_foxglove_arg,
        use_zed_arg,
        motor_port_arg,
        lidar_port_arg,
        imu_port_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        
        # Robot description
        robot_state_publisher,
        joint_state_publisher,
        
        # Sensors
        lidar_node,
        scan_filter_node,
        imu_node,
        
        # ZED Camera (obstacle detection)
        zed_static_tf,
        zed_left_camera_tf,
        zed_left_optical_tf,
        zed_container,
        
        # Motor controller
        base_controller,
        
        # EKF
        ekf_node,
        
        # SLAM Toolbox (mapping mode only)
        slam_toolbox_mapping,
        
        # AMCL + Map Server (localization mode only)
        static_map_odom_tf,
        map_server,
        amcl_node,
        lifecycle_manager_localization,
        
        # Auto pose saver (localization mode only)
        auto_pose_saver,
        
        # Joystick
        joy_node,
        joy2cmd,
        
        # Nav2 stack
        nav2_controller,
        nav2_planner,
        nav2_smoother,
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_waypoint_follower,
        nav2_velocity_smoother,
        nav2_lifecycle_manager,
        
        # RViz
        rviz_node,
        
        # Foxglove (remote visualization)
        foxglove_bridge,
    ])
