import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    launch_slam = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(get_package_share_directory('shbat_pkg'), 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'false'
        }.items()
    )


    # Run the nodes
    return LaunchDescription([
        launch_slam
    ])