import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
import serial

# ============================================================================
# USB Device Detection Functions
# ============================================================================
# Detection Strategy:
#   1. CH340 (1a86:7523) → Always Motor RS485 (unique chip)
#   2. CP2102 (10c4:ea60) → Always IMU (unique chip)
#   3. FTDI (0403:6001) → Could be LIDAR or Motor RS485
#      → Probe with Modbus: if responds → Motor, else → LIDAR
#
# LIDAR (Oradar MS200): 230400 baud, sends continuous binary scan data
# Motor (ZLAC8015D): 115200 baud, responds to Modbus RTU queries
# ============================================================================

def is_zlac8015d_motor(port, timeout=0.5):
    """
    Probe a serial port to check if it's a ZLAC8015D motor controller.
    Sends a Modbus RTU read request and checks for valid response.
    
    Returns: True if motor controller responds, False otherwise
    """
    try:
        # ZLAC8015D uses 115200 baud, 8N1
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=timeout
        )
        
        # Clear any buffered data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Calculate Modbus CRC16
        def calc_crc(data):
            crc = 0xFFFF
            for byte in data:
                crc ^= byte
                for _ in range(8):
                    if crc & 0x0001:
                        crc = (crc >> 1) ^ 0xA001
                    else:
                        crc >>= 1
            return crc
        
        # Modbus RTU request: Read holding register 0x200D (control mode)
        # Slave ID: 0x01, Function: 0x03 (read holding), Addr: 0x200D, Count: 0x0001
        request_data = bytes([0x01, 0x03, 0x20, 0x0D, 0x00, 0x01])
        crc = calc_crc(request_data)
        modbus_request = request_data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        
        ser.write(modbus_request)
        ser.flush()
        
        # Wait for response (motor should reply within timeout)
        import time
        time.sleep(0.15)  # Give motor time to respond
        
        response = ser.read(7)  # Expected response: 7 bytes (ID + Func + ByteCount + 2 data + 2 CRC)
        ser.close()
        
        # Check if we got a valid Modbus response
        if len(response) >= 5:
            # Valid response starts with slave ID (0x01) and function code (0x03)
            if response[0] == 0x01 and response[1] == 0x03:
                return True
        
        return False
        
    except Exception as e:
        try:
            ser.close()
        except:
            pass
        return False

def is_oradar_lidar(port, timeout=0.3):
    """
    Probe a serial port to check if it's an Oradar MS200 LIDAR.
    LIDAR sends continuous binary data at 230400 baud.
    
    Returns: True if looks like LIDAR data, False otherwise
    """
    try:
        # Oradar MS200 uses 230400 baud
        ser = serial.Serial(
            port=port,
            baudrate=230400,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=timeout
        )
        
        # Clear buffer and wait for data
        ser.reset_input_buffer()
        
        import time
        time.sleep(0.2)  # Wait for LIDAR to send scan data
        
        # LIDAR continuously sends data, should have something in buffer
        available = ser.in_waiting
        
        if available > 50:  # LIDAR sends lots of data continuously
            data = ser.read(min(available, 100))
            ser.close()
            
            # Oradar packets typically start with 0x54 (header byte)
            # and contain structured scan data
            if data.count(b'\x54') >= 2:  # Multiple packet headers
                return True
            
            # If we got lots of binary data, it's probably LIDAR
            # Motor wouldn't send unsolicited data
            if len(data) > 50:
                return True
        
        ser.close()
        return False
        
    except Exception as e:
        try:
            ser.close()
        except:
            pass
        return False

def get_all_ttyusb_ports():
    """Get list of all available /dev/ttyUSB ports."""
    ports = []
    for i in range(10):
        port = f'/dev/ttyUSB{i}'
        if os.path.exists(port):
            ports.append(port)
    return ports

def get_port_info(port):
    """Get USB device info for a serial port."""
    try:
        udevadm_output = subprocess.check_output(
            ['udevadm', 'info', '-q', 'all', '-n', port], 
            stderr=subprocess.DEVNULL
        ).decode()
        return udevadm_output
    except Exception:
        return ""

def classify_ports():
    """
    Classify all USB serial ports by their chip type.
    Returns dict: {'ftdi': [...], 'cp2102': [...], 'ch340': [...], 'other': [...]}
    """
    classified = {'ftdi': [], 'cp2102': [], 'ch340': [], 'other': []}
    
    for port in get_all_ttyusb_ports():
        info = get_port_info(port)
        if 'ID_VENDOR_ID=0403' in info and 'ID_MODEL_ID=6001' in info:
            classified['ftdi'].append(port)
        elif 'ID_VENDOR_ID=10c4' in info and 'ID_MODEL_ID=ea60' in info:
            classified['cp2102'].append(port)
        elif 'ID_VENDOR_ID=1a86' in info and 'ID_MODEL_ID=7523' in info:
            classified['ch340'].append(port)
        else:
            classified['other'].append(port)
    
    return classified

def smart_detect_devices():
    """
    Smart device detection using protocol probing for FTDI devices.
    
    Returns:
        tuple: (lidar_port, imu_port, motor_port)
        
    Detection Strategy:
    1. CH340 (1a86:7523) → Motor RS485 (chip unique to motor adapters)
    2. CP2102 (10c4:ea60) → IMU (Wheeltec N100)
    3. FTDI (0403:6001) → Probe to identify:
       - Send Modbus query → if responds → Motor
       - Check for LIDAR data stream → if present → LIDAR
    """
    classified = classify_ports()
    
    lidar_port = None
    imu_port = None
    motor_port = None
    
    # IMU: CP2102 is unique to Wheeltec N100
    if classified['cp2102']:
        imu_port = classified['cp2102'][0]
        print(f"✓ Detected IMU port (CP2102): {imu_port}")
    else:
        print("⚠ IMU port not detected (CP2102 not found)")
    
    # Motor: CH340 is unique to RS485 adapters
    if classified['ch340']:
        motor_port = classified['ch340'][0]
        print(f"✓ Detected Motor controller port (CH340): {motor_port}")
    
    # FTDI devices - need to probe to identify
    ftdi_ports = classified['ftdi']
    unassigned_ftdi = []
    
    if ftdi_ports:
        print(f"  Probing {len(ftdi_ports)} FTDI device(s)...")
        
        for port in ftdi_ports:
            print(f"    Testing {port}...", end=" ", flush=True)
            
            # First try Modbus (motor) - quick check
            if motor_port is None and is_zlac8015d_motor(port):
                motor_port = port
                print("→ Motor (Modbus OK)")
                continue
            
            # Then check for LIDAR data stream
            if lidar_port is None and is_oradar_lidar(port):
                lidar_port = port
                print("→ LIDAR (data stream detected)")
                continue
            
            # Couldn't identify - add to unassigned
            print("→ Unknown")
            unassigned_ftdi.append(port)
        
        # Assign unassigned FTDI ports
        for port in unassigned_ftdi:
            if motor_port is None:
                motor_port = port
                print(f"  ⚠ Assigning {port} to Motor (unidentified FTDI)")
            elif lidar_port is None:
                lidar_port = port
                print(f"  ⚠ Assigning {port} to LIDAR (unidentified FTDI)")
    
    # Report final status
    if motor_port:
        print(f"✓ Motor port: {motor_port}")
    else:
        print("⚠ Motor port not detected")
        # Fallback
        all_ports = get_all_ttyusb_ports()
        if all_ports:
            motor_port = all_ports[0]
            print(f"  → Using fallback: {motor_port}")
        else:
            motor_port = '/dev/ttyUSB0'
            print(f"  → Using default: {motor_port}")
    
    if lidar_port:
        print(f"✓ LIDAR port: {lidar_port}")
    else:
        print("⚠ LIDAR port not detected")
    
    return lidar_port, imu_port, motor_port

# ============================================================================
# Use fixed udev symlinks instead of auto-detection
# Symlinks created by /etc/udev/rules.d/99-sahabat-robot.rules:
#   /dev/motor → Motor controller (ZLAC8015D)
#   /dev/lidar → LIDAR (Oradar MS200)
#   /dev/imu   → IMU (Wheeltec N100)
# ============================================================================

def use_fixed_ports():
    """Use udev symlinks for fixed port assignment."""
    motor_port = '/dev/motor' if os.path.exists('/dev/motor') else None
    lidar_port = '/dev/lidar' if os.path.exists('/dev/lidar') else None
    imu_port = '/dev/imu' if os.path.exists('/dev/imu') else None
    
    print("\n" + "="*50)
    print("USB Devices (udev symlinks)")
    print("="*50)
    
    if motor_port:
        real_port = os.path.realpath(motor_port)
        print(f"✓ Motor: {motor_port} → {real_port}")
    else:
        print("✗ Motor: /dev/motor not found - fallback to detection")
        
    if lidar_port:
        real_port = os.path.realpath(lidar_port)
        print(f"✓ LIDAR: {lidar_port} → {real_port}")
    else:
        print("✗ LIDAR: /dev/lidar not found - fallback to detection")
        
    if imu_port:
        real_port = os.path.realpath(imu_port)
        print(f"✓ IMU:   {imu_port} → {real_port}")
    else:
        print("✗ IMU:   /dev/imu not found - fallback to detection")
    
    print("="*50 + "\n")
    
    return lidar_port, imu_port, motor_port

# Try fixed ports first, fall back to auto-detection
lidar_port, imu_port, motor_port = use_fixed_ports()

# If any port is missing, try auto-detection as fallback
if not all([lidar_port, imu_port, motor_port]):
    print("Some devices not found via udev, trying auto-detection...")
    detected_lidar, detected_imu, detected_motor = smart_detect_devices()
    if not lidar_port:
        lidar_port = detected_lidar
    if not imu_port:
        imu_port = detected_imu
    if not motor_port:
        motor_port = detected_motor

def generate_launch_description():
    
    # ========== Launch Arguments ==========
    
    # Control whether to run the external N100 IMU
    use_n100_imu_arg = DeclareLaunchArgument(
        'use_n100_imu',
        default_value='true' if imu_port else 'false',  # Auto-disable if not detected
        description='Start Wheeltec N100 IMU driver (auto-disabled if not detected)'
    )
    use_n100_imu = LaunchConfiguration('use_n100_imu')

    # Control whether to run the LIDAR
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true' if lidar_port else 'false',  # Auto-disable if not detected
        description='Start LIDAR driver (auto-disabled if not detected)'
    )
    use_lidar = LaunchConfiguration('use_lidar')

    # Control whether to run the Kalman filter fusion node
    use_kalman_filter_arg = DeclareLaunchArgument(
        'use_kalman_filter',
        default_value='true',
        description='Start Kalman filter node (set false to disable fusion and use raw odom)'
    )
    use_kalman_filter = LaunchConfiguration('use_kalman_filter')
    
    # Motor controller port argument
    motor_port_arg = DeclareLaunchArgument(
        'motor_port',
        default_value=motor_port if motor_port else '/dev/ttyUSB0',
        description='Serial port for ZLAC8015D motor controller'
    )
    motor_port_config = LaunchConfiguration('motor_port')
    
    # LIDAR port argument
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=lidar_port if lidar_port else '/dev/ttyUSB0',
        description='Serial port for LIDAR'
    )
    lidar_port_config = LaunchConfiguration('lidar_port')
    
    # IMU port argument
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value=imu_port if imu_port else '/dev/ttyUSB0',
        description='Serial port for IMU'
    )
    imu_port_config = LaunchConfiguration('imu_port')
    
    # Launch argument to control robot_state_publisher
    publish_robot_state_arg = DeclareLaunchArgument(
        'publish_robot_state',
        default_value='true',
        description='Whether to publish robot_state_publisher (set to false when using external URDF)'
    )
    publish_robot_state = LaunchConfiguration('publish_robot_state')

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'shbat_pkg'
    file_subpath = 'urdf/sahabat_robot.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the robot_state_publisher node (only if publish_robot_state:=true)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw},
                    {'use_sim_time': True}],
        condition=IfCondition(publish_robot_state)
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
        parameters=[{'robot_description': robot_description_raw}],
        condition=IfCondition(publish_robot_state)
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
            {'port_name': lidar_port_config},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.0},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10}
        ],
        condition=IfCondition(use_lidar)
    )

    node_imu = Node(
        package='wheeltec_n100_imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'serial_port': imu_port_config,
            'serial_baud': 921600,
            'imu_topic': 'imu',
            'imu_frame': 'imu_link',
        }],
        condition=IfCondition(use_n100_imu)
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
        output='screen',
        parameters=[
            {'port': motor_port_config},
            {'baudrate': 115200},
            {'wheel_radius': 0.0875},      # 175mm diameter wheel
            {'wheel_base': 0.33},          # Distance between wheels
            {'publish_odom_tf': False},    # EKF publishes odom->base_link TF instead
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'odom_topic': 'wheel_odom'},  # Odometry topic name
            {'accel_time_ms': 200},
            {'decel_time_ms': 200},
            {'max_linear_vel': 1.0},       # m/s
            {'max_angular_vel': 2.0},      # rad/s
            {'cmd_vel_timeout': 0.5},      # seconds
            {'odom_rate': 20.0},           # Hz
        ]
    )

    # Legacy simple kalman filter (disabled - using robot_localization EKF instead)
    node_kalman_filter = Node(
        package='shbat_pkg',
        executable='kalman_filter',
        name='kalman_filter',
        output='screen',
        condition=IfCondition('false')  # Disabled - use EKF instead
    )

    # EKF from robot_localization package for sensor fusion
    # Fuses wheel odometry + IMU for better state estimation
    pkg_share = get_package_share_directory(pkg_name)
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('odometry/filtered', 'odom'),  # Output filtered odometry
        ],
        condition=IfCondition(use_kalman_filter)
    )

    # Run the nodes
    return LaunchDescription([
        # Launch arguments
        use_n100_imu_arg,
        use_lidar_arg,
        use_kalman_filter_arg,
        motor_port_arg,
        lidar_port_arg,
        imu_port_arg,
        publish_robot_state_arg,
        
        # Core nodes (always run)
        node_robot_state_publisher,
        node_joint_state_publisher,
        # node_rviz,

        # Joystick control
        node_joy_node,
        node_joy2cmd,   

        # Sensor fusion - EKF (conditional)
        # node_kalman_filter,  # Legacy simple filter - disabled
        node_ekf,              # robot_localization EKF

        # Sensors (conditional based on detection)
        node_imu,
        node_lidar_scan,
        
        # Motor controller
        node_base_controller
    ])