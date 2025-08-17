import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess

def detect_lidar_port():
    try:
        # Run lsusb to get USB devices
        lsusb_output = subprocess.check_output(['lsusb']).decode()
        # Check for the FT232 device
        if '0403:6001' in lsusb_output:
            # List ttyUSB devices
            for i in range(10):
                port = f'/dev/ttyUSB{i}'
                try:
                    # Use udevadm to check device info
                    udevadm_output = subprocess.check_output(['udevadm', 'info', '-q', 'all', '-n', port]).decode()
                    if 'ID_VENDOR_ID=0403' in udevadm_output and 'ID_MODEL_ID=6001' in udevadm_output:
                        print(f"Detected LIDAR port: {port}")
                        return port
                except Exception:
                    continue
    except Exception:
        pass
    print("LIDAR port not detected, using default: /dev/ttyUSB0")
    return '/dev/ttyUSB0'  # Default fallback

lidar_port = detect_lidar_port()

def detect_imu_port():
    try:
        lsusb_output = subprocess.check_output(['lsusb']).decode()
        if '10c4:ea60' in lsusb_output:
            for i in range(10):
                port = f'/dev/ttyUSB{i}'
                try:
                    udevadm_output = subprocess.check_output(['udevadm', 'info', '-q', 'all', '-n', port]).decode()
                    if 'ID_VENDOR_ID=10c4' in udevadm_output and 'ID_MODEL_ID=ea60' in udevadm_output:
                        print(f"Detected IMU port: {port}")
                        return port
                except Exception:
                    continue
    except Exception:
        pass
    print("IMU port not detected, using default: /dev/ttyUSB0")
    return '/dev/ttyUSB0'

imu_port = detect_imu_port()

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'shbat_pkg'
    file_subpath = 'urdf/sahabat_robot.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw},
                    {'use_sim_time': True}],
    )

    # Configure the RViz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    node_lidar_scan = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='oradar_scan_node',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar_link'},
            {'scan_topic': 'scan'},
            {'port_name': lidar_port},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.0},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10}
        ]  # Change as needed
    )

    node_imu = Node(
        package='wheeltec_n100_imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{'serial_port': imu_port}]  # Change '/dev/ttyUSB0' to your actual serial port if needed
    )
    node_rpm2odom = Node(
        package='shbat_pkg',
        executable='rpm_to_odom',
        name='rpm_to_odom',
        output='screen'
    )
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

    node_kalman_filter = Node(
        package='shbat_pkg',
        executable='kalman_filter',
        name='kalman_filter',
        output='screen'
    )

    # Run the nodes
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        # node_rviz,

        node_joy_node,
        node_joy2cmd,   

        # node_rpm2odom,
        node_kalman_filter,

        node_imu,
        node_lidar_scan,
        node_base_controller
    ])