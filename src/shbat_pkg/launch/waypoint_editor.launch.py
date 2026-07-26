"""Saved-map waypoint editor.

By default this launch is offline and motion-free: it starts only a map server
and RViz with the RViz-native waypoint_editor plugin. For live robot operation,
run it with waypoint_backend:=operator start_map_server:=false alongside the
operations stack so it edits through /operator waypoint services without
starting duplicate map infrastructure.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _normalize_map_yaml(path: Path) -> Path:
    if path.suffix == '.yaml':
        return path
    return path.with_suffix('.yaml')


def _launch_setup(context, *_args, **_kwargs):
    package_share = Path(get_package_share_directory('shbat_pkg'))
    maps_directory = Path(
        LaunchConfiguration('maps_directory').perform(context)
    ).expanduser()
    map_id = LaunchConfiguration('map_id').perform(context).strip()
    map_file = LaunchConfiguration('map_file').perform(context).strip()
    waypoint_file = LaunchConfiguration('waypoint_file').perform(context).strip()
    waypoint_backend = LaunchConfiguration('waypoint_backend').perform(context).strip()
    start_map_server = LaunchConfiguration('start_map_server').perform(context).strip().lower()
    rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()

    if map_file:
        map_yaml = _normalize_map_yaml(Path(map_file).expanduser())
        map_id = map_yaml.stem
    else:
        map_yaml = maps_directory / f'{map_id}.yaml'
        if not map_yaml.exists():
            map_yaml = maps_directory / map_id / 'map.yaml'

    waypoint_sets_directory = maps_directory / 'waypoint_sets' / map_id

    if not waypoint_file:
        waypoint_file = str(maps_directory / f'{map_id}_waypoints.yaml')

    if not rviz_config:
        rviz_config = str(package_share / 'rviz' / 'waypoint_editor.rviz')

    nodes = []
    if start_map_server not in ('false', '0', 'no', 'off'):
        nodes.extend([
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': str(map_yaml)}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_waypoint_editor',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'use_sim_time': False,
                    'node_names': ['map_server'],
                }],
            ),
        ])

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_waypoint_editor',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'waypoint_file': waypoint_file,
                'maps_directory': str(maps_directory),
                'map_id': map_id,
                'waypoint_sets_directory': str(waypoint_sets_directory),
                'waypoint_backend': waypoint_backend,
            }],
        ),
    )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'maps_directory',
            default_value='~/sahabat_ws/maps',
            description='Directory containing saved map folders',
        ),
        DeclareLaunchArgument(
            'map_id',
            default_value='rdlfront',
            description='Saved map file stem under maps_directory, e.g. rdlfront for rdlfront.yaml',
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Optional explicit map YAML path, with or without .yaml',
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='',
            description='Optional explicit legacy waypoint path',
        ),
        DeclareLaunchArgument(
            'waypoint_backend',
            default_value='local',
            description='Waypoint storage backend: local for offline YAML, operator for live /operator services',
        ),
        DeclareLaunchArgument(
            'start_map_server',
            default_value='true',
            description='Start an editor-only map_server; set false when running alongside live operations',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional RViz config path',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
