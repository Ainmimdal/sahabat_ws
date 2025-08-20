from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node_joy2cmd = Node(
        package='shbat_pkg',  
        executable='joy2cmd',
        name='joy2cmd',
        output='screen'
    )

    node_joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    node_base_controller = Node(
        package='shbat_pkg',
        executable='base_controller',
        name='base_controller',
        output='screen'
    )


    # Run the nodes
    return LaunchDescription([
        node_joy_node,
        node_joy2cmd,   
        node_base_controller
    ])