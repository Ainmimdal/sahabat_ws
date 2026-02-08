"""
Localization + Patrol Launch File

Launches everything needed for tour guide patrol:
1. Full SLAM/Nav2 stack in localization mode
2. Waypoint Manager GUI for patrol control
3. Optional API Bridge for external control

Usage:
  # Basic localization with patrol GUI
  ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/my_map

  # With ZED camera for better obstacle detection
  ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/my_map use_zed:=true

  # With API Bridge (for Pi/LLM control)
  ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/my_map use_api:=true

  # With Foxglove for remote monitoring (no local RViz)
  ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/my_map use_foxglove:=true use_rviz:=false

  # Full setup with known start position
  ros2 launch shbat_pkg localization_patrol_launch.py \\
      map_file:=/home/sahabat/maps/gallery \\
      initial_pose_x:=1.5 \\
      initial_pose_y:=2.0 \\
      initial_pose_yaw:=1.57 \\
      use_zed:=true \\
      use_api:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    pkg_share = get_package_share_directory('shbat_pkg')
    
    # ========== Launch Arguments ==========
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        description='Path to map file (without .yaml extension)'
    )
    map_file = LaunchConfiguration('map_file')
    
    use_zed_arg = DeclareLaunchArgument(
        'use_zed',
        default_value='false',
        description='Enable ZED camera for obstacle detection (PointCloud to costmap)'
    )
    use_zed = LaunchConfiguration('use_zed')
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    use_rviz = LaunchConfiguration('use_rviz')
    
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='false',
        description='Launch Foxglove Bridge for remote visualization'
    )
    use_foxglove = LaunchConfiguration('use_foxglove')
    
    use_api_arg = DeclareLaunchArgument(
        'use_api',
        default_value='false',
        description='Launch API Bridge for external control (Pi/LLM)'
    )
    use_api = LaunchConfiguration('use_api')
    
    use_waypoint_gui_arg = DeclareLaunchArgument(
        'use_waypoint_gui',
        default_value='true',
        description='Launch Waypoint Manager GUI'
    )
    use_waypoint_gui = LaunchConfiguration('use_waypoint_gui')
    
    # Initial pose for auto-localization
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial X position (use saved pose from save_current_pose script)'
    )
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial Y position'
    )
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    
    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial yaw (radians)'
    )
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    
    # Waypoint file
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(pkg_share, 'config', 'patrol_waypoints.yaml'),
        description='Path to waypoint patrol file'
    )
    waypoint_file = LaunchConfiguration('waypoint_file')
    
    # ========== Include SLAM Navigation Launch ==========
    
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam_nav_launch.py')
        ),
        launch_arguments={
            'mode': 'localization',
            'map_file': map_file,
            'use_zed': use_zed,
            'use_rviz': use_rviz,
            'use_foxglove': use_foxglove,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }.items()
    )
    
    # ========== Waypoint Manager GUI ==========
    # Delay start to ensure Nav2 is up
    
    waypoint_manager = TimerAction(
        period=10.0,  # Wait 10 seconds for Nav2 to initialize
        actions=[
            Node(
                package='shbat_pkg',
                executable='waypoint_manager',
                name='waypoint_manager',
                output='screen',
                condition=IfCondition(use_waypoint_gui),
            )
        ]
    )
    
    # ========== API Bridge ==========
    
    api_bridge = Node(
        package='shbat_pkg',
        executable='api_bridge',
        name='api_bridge',
        output='screen',
        condition=IfCondition(use_api),
    )
    
    # ========== Return Launch Description ==========
    
    return LaunchDescription([
        # Arguments
        map_file_arg,
        use_zed_arg,
        use_rviz_arg,
        use_foxglove_arg,
        use_api_arg,
        use_waypoint_gui_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        waypoint_file_arg,
        
        # Core navigation (localization mode)
        slam_nav_launch,
        
        # Waypoint Manager (delayed start)
        waypoint_manager,
        
        # API Bridge (optional)
        api_bridge,
    ])
