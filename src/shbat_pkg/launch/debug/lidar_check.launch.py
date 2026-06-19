"""Run only the RPLIDAR S2 and optional Sahabat scan filter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create a motor-free launch description for LIDAR diagnosis."""
    pkg_share = get_package_share_directory('shbat_pkg')

    lidar_port = LaunchConfiguration('lidar_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    scan_mode = LaunchConfiguration('scan_mode')
    use_filter = LaunchConfiguration('use_filter')

    arguments = [
        DeclareLaunchArgument(
            'lidar_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
            ),
            description=(
                'RPLIDAR serial device, for example /dev/ttyUSB0 or '
                '/dev/serial/by-id/...'
            ),
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='1000000',
            description='RPLIDAR S2 serial baud rate',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='DenseBoost',
            description='RPLIDAR scan mode; try Standard for comparison',
        ),
        DeclareLaunchArgument(
            'use_filter',
            default_value='true',
            description='Publish filtered /scan in addition to /scan_raw',
        ),
    ]

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_diagnostic',
        output='screen',
        emulate_tty=True,
        respawn=False,
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': ParameterValue(
                serial_baudrate, value_type=int
            ),
            'frame_id': 'lidar_link',
            'angle_min': -3.14,
            'angle_max': 3.14,
            'inverted': False,
            'clockwise': True,
            'angle_compensate': True,
            'scan_mode': ParameterValue(scan_mode, value_type=str),
        }],
        remappings=[('scan', 'scan_raw')],
    )

    scan_filter = Node(
        package='shbat_pkg',
        executable='scan_filter',
        name='scan_filter',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'scan_filter.yaml')],
        condition=IfCondition(use_filter),
    )

    return LaunchDescription(arguments + [lidar, scan_filter])
