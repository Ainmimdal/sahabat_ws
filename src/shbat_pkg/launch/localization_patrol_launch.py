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

    maps_directory_arg = DeclareLaunchArgument(
        'maps_directory',
        default_value='~/sahabat_ws/maps',
        description='Directory containing saved maps and waypoint sets',
    )
    maps_directory = LaunchConfiguration('maps_directory')

    map_id_arg = DeclareLaunchArgument(
        'map_id',
        default_value='',
        description='Saved map file stem, e.g. rdlfront',
    )
    map_id = LaunchConfiguration('map_id')
    
    use_zed_arg = DeclareLaunchArgument(
        'use_zed',
        default_value='false',
        description='Enable ZED camera for obstacle detection (PointCloud to costmap)'
    )
    use_zed = LaunchConfiguration('use_zed')

    use_keepout_arg = DeclareLaunchArgument(
        'use_keepout',
        default_value='true',
        description='Use the map keepout mask when available',
    )
    use_keepout = LaunchConfiguration('use_keepout')

    keepout_mask_file_arg = DeclareLaunchArgument(
        'keepout_mask_file',
        default_value='',
        description='Optional explicit keepout mask YAML path',
    )
    keepout_mask_file = LaunchConfiguration('keepout_mask_file')

    localization_backend_arg = DeclareLaunchArgument(
        'localization_backend',
        default_value='amcl',
        choices=['amcl', 'slam_toolbox'],
        description='Use AMCL or SLAM Toolbox for saved-map localization',
    )
    localization_backend = LaunchConfiguration('localization_backend')
    
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

    tour_profile_arg = DeclareLaunchArgument(
        'tour_profile',
        default_value='auto',
        choices=['auto', 'production', 'test'],
        description=(
            'Strict six-exhibit production validation or permissive test map'
        ),
    )
    tour_profile = LaunchConfiguration('tour_profile')
    
    use_waypoint_gui_arg = DeclareLaunchArgument(
        'use_waypoint_gui',
        default_value='true',
        description='Launch Waypoint Manager GUI'
    )
    use_waypoint_gui = LaunchConfiguration('use_waypoint_gui')

    joy_cmd_topic_arg = DeclareLaunchArgument(
        'joy_cmd_topic', default_value='cmd_vel_joy'
    )
    joy_cmd_topic = LaunchConfiguration('joy_cmd_topic')
    smoothed_cmd_topic_arg = DeclareLaunchArgument(
        'smoothed_cmd_topic', default_value='cmd_vel_nav_smoothed'
    )
    smoothed_cmd_topic = LaunchConfiguration('smoothed_cmd_topic')
    recovery_cmd_topic_arg = DeclareLaunchArgument(
        'recovery_cmd_topic', default_value='cmd_vel_recovery'
    )
    recovery_cmd_topic = LaunchConfiguration('recovery_cmd_topic')
    use_command_arbiter_arg = DeclareLaunchArgument(
        'use_command_arbiter',
        default_value='true',
        description='Give /cmd_vel one priority-selecting publisher',
    )
    use_command_arbiter = LaunchConfiguration('use_command_arbiter')
    operator_safety_arg = DeclareLaunchArgument(
        'operator_safety', default_value='false'
    )
    operator_safety = LaunchConfiguration('operator_safety')
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware', default_value='true'
    )
    use_hardware = LaunchConfiguration('use_hardware')

    use_saved_initial_pose_arg = DeclareLaunchArgument(
        'use_saved_initial_pose', default_value='false'
    )
    use_saved_initial_pose = LaunchConfiguration('use_saved_initial_pose')

    initialize_from_dock_arg = DeclareLaunchArgument(
        'initialize_from_dock', default_value='true'
    )
    initialize_from_dock = LaunchConfiguration('initialize_from_dock')
    
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
        default_value=PythonExpression([
            "'", map_file, "'.rsplit('/', 1)[0] + '/waypoints.yaml'"
        ]),
        description='Legacy waypoint path; its directory owns waypoint sets'
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
            'maps_directory': maps_directory,
            'map_id': map_id,
            'use_zed': use_zed,
            'use_keepout': use_keepout,
            'keepout_mask_file': keepout_mask_file,
            'localization_backend': localization_backend,
            'use_rviz': use_rviz,
            'use_foxglove': use_foxglove,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
            'joy_cmd_topic': joy_cmd_topic,
            'smoothed_cmd_topic': smoothed_cmd_topic,
            'operator_safety': operator_safety,
            'use_saved_initial_pose': use_saved_initial_pose,
            'use_hardware': use_hardware,
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
                parameters=[{'waypoint_file': waypoint_file}],
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
        parameters=[{'tour_profile': tour_profile}],
        condition=IfCondition(use_api),
    )

    # ========== Exhibit Navigator ==========

    exhibit_navigator = Node(
        package='shbat_pkg',
        executable='exhibit_navigator',
        name='exhibit_navigator',
        output='screen',
        condition=IfCondition(use_api),
    )

    localization_recovery = Node(
            package='shbat_pkg',
            executable='localization_recovery',
            name='localization_recovery',
            output='screen',
            parameters=[{'cmd_vel_topic': recovery_cmd_topic}],
            condition=IfCondition(PythonExpression([
                "'", localization_backend, "' == 'amcl'"
            ])),
        )

    command_arbiter = Node(
        package='shbat_pkg',
        executable='command_arbiter',
        name='command_arbiter',
        output='screen',
        condition=IfCondition(use_command_arbiter),
    )

    dock_pose_initializer = Node(
        package='shbat_pkg',
        executable='dock_pose_initializer',
        name='dock_pose_initializer',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'maps_directory': maps_directory,
            'map_id': map_id,
        }],
        condition=IfCondition(initialize_from_dock),
    )
    
    # ========== Return Launch Description ==========
    
    return LaunchDescription([
        # Arguments
        map_file_arg,
        maps_directory_arg,
        map_id_arg,
        use_zed_arg,
        use_keepout_arg,
        keepout_mask_file_arg,
        localization_backend_arg,
        use_rviz_arg,
        use_foxglove_arg,
        use_api_arg,
        tour_profile_arg,
        use_waypoint_gui_arg,
        joy_cmd_topic_arg,
        smoothed_cmd_topic_arg,
        recovery_cmd_topic_arg,
        use_command_arbiter_arg,
        operator_safety_arg,
        use_hardware_arg,
        use_saved_initial_pose_arg,
        initialize_from_dock_arg,
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
        exhibit_navigator,
        localization_recovery,
        command_arbiter,
        dock_pose_initializer,
    ])
