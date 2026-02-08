"""
Nav2 Odom-Only Test Launch File

This launch file starts everything needed to test Nav2 navigation
using only wheel odometry + IMU (no pre-built map required).

Usage:
  ros2 launch shbat_pkg nav2_test_launch.py

  # With LIDAR disabled (motor testing only):
  ros2 launch shbat_pkg nav2_test_launch.py use_lidar:=false

  # With EKF disabled (raw wheel odom):
  ros2 launch shbat_pkg nav2_test_launch.py use_ekf:=false

What this launches:
  1. Robot state publisher (URDF)
  2. Base controller (motor driver + wheel odometry)
  3. LIDAR driver (if connected)
  4. IMU driver (if connected)  
  5. EKF sensor fusion
  6. Static map->odom transform
  7. Nav2 stack (controller, planner, behavior tree, etc.)
  8. RViz2 for visualization and goal setting

Test procedure:
  1. Launch this file
  2. In RViz, use "2D Goal Pose" to set a navigation goal
  3. Robot should plan a path and navigate to goal
  4. LIDAR obstacles will be avoided
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

# Import USB detection from main launch
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from sahabat_launch import smart_detect_devices
    lidar_port, imu_port, motor_port = smart_detect_devices()
except Exception as e:
    print(f"Warning: Could not import device detection: {e}")
    lidar_port, imu_port, motor_port = None, None, '/dev/ttyUSB0'


def generate_launch_description():
    
    pkg_name = 'shbat_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    # ========== Launch Arguments ==========
    
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
    
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Enable EKF sensor fusion'
    )
    use_ekf = LaunchConfiguration('use_ekf')
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    use_rviz = LaunchConfiguration('use_rviz')
    
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

    # ========== Base Controller (Motors + Wheel Odometry) ==========
    
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
            {'max_linear_vel': 0.5},        # Limited for Nav2 testing
            {'max_angular_vel': 1.5},
            {'cmd_vel_timeout': 0.5},
            {'odom_rate': 20.0},
        ]
    )
    
    # Base controller without EKF (publishes its own TF)
    base_controller_no_ekf = Node(
        package='shbat_pkg',
        executable='base_controller',
        name='base_controller',
        output='screen',
        parameters=[
            {'port': motor_port_cfg},
            {'baudrate': 115200},
            {'wheel_radius': 0.0875},  # 175mm diameter / 2
            {'wheel_base': 0.33},
            {'publish_odom_tf': True},      # Publish TF when no EKF
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'odom_topic': 'wheel_odom'},
            {'accel_time_ms': 200},
            {'decel_time_ms': 200},
            {'max_linear_vel': 0.5},
            {'max_angular_vel': 1.5},
            {'cmd_vel_timeout': 0.5},
            {'odom_rate': 20.0},
        ],
        # Also remap wheel_odom to odom when no EKF
        remappings=[('wheel_odom', 'odom')],
        condition=UnlessCondition(use_ekf)
    )

    # ========== LIDAR ==========
    
    # LIDAR publishes to /scan_raw, then scan_filter filters to /scan
    lidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='oradar_scan_node',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar_link'},
            {'scan_topic': 'scan_raw'},      # Changed: output to scan_raw
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
    
    # Scan filter - removes beam readings from LIDAR
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
        ],
        condition=IfCondition(use_ekf)
    )

    # ========== Static Transform: map -> odom (identity) ==========
    # Required for Nav2 when not using AMCL/map-based localization
    
    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
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
        remappings=[('cmd_vel', 'cmd_vel_nav')]  # Remap to avoid joystick conflict
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
            ('cmd_vel', 'cmd_vel_nav'),          # Input from controller
            ('cmd_vel_smoothed', 'cmd_vel')      # Output to robot
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
            {'bond_timeout': 0.0}   # Disable bond for testing
        ]
    )

    # ========== Costmap Nodes ==========
    
    nav2_local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[nav2_config],
        remappings=[('costmap', 'local_costmap/costmap'),
                    ('costmap_updates', 'local_costmap/costmap_updates')]
    )
    
    nav2_global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[nav2_config],
        remappings=[('costmap', 'global_costmap/costmap'),
                    ('costmap_updates', 'global_costmap/costmap_updates')]
    )

    # ========== RViz ==========
    
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_test.rviz')
    
    # Use default config if custom doesn't exist
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

    # ========== Return Launch Description ==========
    
    return LaunchDescription([
        # Arguments
        use_lidar_arg,
        use_imu_arg,
        use_ekf_arg,
        use_rviz_arg,
        motor_port_arg,
        lidar_port_arg,
        imu_port_arg,
        
        # Robot description
        robot_state_publisher,
        joint_state_publisher,
        
        # Static TF (map -> odom)
        static_map_odom_tf,
        
        # Sensors
        lidar_node,
        scan_filter_node,    # Filter out beam readings
        imu_node,
        
        # Motor controller (conditional on EKF)
        GroupAction([base_controller], condition=IfCondition(use_ekf)),
        base_controller_no_ekf,
        
        # EKF
        ekf_node,
        
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
    ])
