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
#   1. FTDI (0403:6001):
#      → Serial A50285BI: Motor (known adapter for ZLAC8015D, skip probe)
#      → Serial A5069RR4: RPLIDAR S2 (probe for scan data at 115200)
#   2. CH340 (1a86:7523): Could be Motor RS485 or BNO055 IMU
#      → Probe Modbus: if responds → Motor
#      → Probe BNO055: CHIP_ID query → IMU
#
# RPLIDAR S2: 1,000,000 baud (DenseBoost scan mode), 10 Hz, 30m range
# BNO055: 115200 baud, responds to UART register read commands
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

def is_rplidar_s2(port, timeout=0.5):
    """
    Probe a serial port to check if it's an RPLIDAR S2.

    Queries device information at the S2's documented 1,000,000 baud. Device
    detection must not reset the sensor or pretend that DTR removes USB power.

    Returns: True if looks like RPLIDAR S2, False otherwise
    """
    try:
        from shbat_pkg.lidar_recovery import get_device_info

        with serial.Serial(
            port=port,
            baudrate=1_000_000,
            timeout=timeout,
            write_timeout=timeout,
            exclusive=True,
        ) as connection:
            get_device_info(connection)
        return True
    except Exception:
        return False


def is_bno055(port, timeout=0.5):
    """
    Probe a serial port to check if it's a BNO055 IMU via UART.
    
    Strategy:
    1. Open at 115200 baud (BNO055 default UART baud)
    2. Send UART READ command (0x01) for CHIP_ID register (0x00)
    3. BNO055 CHIP_ID is always 0xA0
    
    BNO055 UART protocol:
      Read: 0xAA + 0x01 (READ) + reg_addr + length
      Response success: 0xBB + resp_len + data...
      Response error:   0xEE + error_code
    
    Returns: True if BNO055 detected, False otherwise
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=timeout
        )
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Read CHIP_ID register (0x00), length 1
        # Format: START_BYTE(0xAA) + READ(0x01) + REG_ADDR + LENGTH
        ser.write(bytes([0xAA, 0x01, 0x00, 0x01]))
        ser.flush()
        
        import time
        time.sleep(0.08)
        
        # Response: RESP_OK(0xBB) + length(0x01) + data(0xA0)
        response = ser.read(3)
        ser.close()
        
        if len(response) >= 3:
            if response[0] == 0xBB and response[2] == 0xA0:
                return True
        
        return False
        
    except Exception:
        try:
            ser.close()
        except:
            pass
        return False

def is_hwt901b(port, timeout=0.5):
    """
    Probe a serial port to check if it's an HWT901B / WT901B IMU.
    
    Strategy:
    1. Open at 115200 baud (HWT901B default)
    2. HWT901B continuously outputs WITMotion protocol data
    3. Check for 0x55 header bytes followed by known packet types (0x51-0x59)
    
    Returns: True if looks like HWT901B, False otherwise
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=timeout
        )
        
        ser.reset_input_buffer()
        
        import time
        time.sleep(0.25)
        
        available = ser.in_waiting
        
        if available > 200:
            # HWT901B continuously outputs ~10-100 Hz of WITMotion packets
            data = ser.read(min(available, 200))
            ser.close()
            
            # WITMotion packets start with 0x55 + type byte 0x50-0x59
            for i in range(len(data) - 1):
                if data[i] == 0x55 and 0x50 <= data[i+1] <= 0x59:
                    return True
            
        ser.close()
        return False
        
    except Exception:
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

def get_port_serial(port):
    """Extract serial number from udevadm info."""
    try:
        info = get_port_info(port)
        for line in info.splitlines():
            if 'ID_SERIAL_SHORT=' in line:
                return line.split('=', 1)[1].strip()
    except Exception:
        pass
    return None


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
    Smart device detection using protocol probing.
    
    Returns:
        tuple: (lidar_port, imu_port, motor_port)
        
    Detection Strategy:
    1. FTDI (0403:6001):
       - Serial A50285BI → Motor (known FTDI adapter for ZLAC8015D)
       - Other FTDI: probe for RPLIDAR S2, then BNO055
    2. CH340 (1a86:7523):
       - Probe Modbus first → Motor
       - Probe BNO055 → IMU
    """
    classified = classify_ports()
    
    lidar_port = None
    imu_port = None
    motor_port = None
    
    # --- FTDI devices: probe to identify ---
    ftdi_ports = classified['ftdi']
    unassigned_ftdi = []
    
    if ftdi_ports:
        print(f"  Probing {len(ftdi_ports)} FTDI device(s)...")
        
        for port in ftdi_ports:
            print(f"    Testing {port}...", end=" ", flush=True)
            
            # Check for known motor serial
            port_serial = get_port_serial(port)
            if port_serial == 'A50285BI':
                if motor_port is None:
                    motor_port = port
                    print(f"→ Motor (known serial {port_serial})")
                else:
                    print(f"→ Skipped (known serial, already have motor)")
                continue

            if port_serial == 'A5069RR4' and lidar_port is None:
                lidar_port = port
                print(f"→ RPLIDAR S2 (known serial {port_serial})")
                continue
            
            # Probe for RPLIDAR S2
            if lidar_port is None and is_rplidar_s2(port):
                lidar_port = port
                print("→ RPLIDAR S2 (scan data / STOP response)")
                continue
            
            # Probe for BNO055
            if imu_port is None and is_bno055(port):
                imu_port = port
                print("→ BNO055 IMU (chip ID A0)")
                continue
            
            # Probe for HWT901B (WITMotion protocol)
            if imu_port is None and is_hwt901b(port):
                imu_port = port
                print("→ HWT901B IMU (WITMotion data stream)")
                continue
            
            # Couldn't identify
            print("→ Unknown")
            unassigned_ftdi.append(port)
        
        # Assign unassigned FTDI ports as fallback
        for port in unassigned_ftdi:
            if motor_port is None:
                motor_port = port
                print(f"  ⚠ Assigning {port} to Motor (unidentified FTDI)")
            elif lidar_port is None:
                lidar_port = port
                print(f"  ⚠ Assigning {port} to LIDAR (unidentified FTDI)")
            elif imu_port is None:
                imu_port = port
                print(f"  ⚠ Assigning {port} to IMU (unidentified FTDI)")
    
    # --- CH340 devices: HWT901B IMU (or Motor via Modbus) ---
    ch340_ports = classified['ch340']
    if ch340_ports:
        print(f"  Probing {len(ch340_ports)} CH340 device(s)...")
        for port in ch340_ports:
            print(f"    Testing {port}...", end=" ", flush=True)
            
            # Probe Modbus first (motor)
            if motor_port is None and is_zlac8015d_motor(port):
                motor_port = port
                print("→ Motor (Modbus OK)")
                continue
            
            # If not motor, assign to IMU directly (don't probe — avoids port conflict)
            if imu_port is None:
                imu_port = port
                print("→ IMU (CH340 — HWT901B)")
            else:
                print("→ Skipped (IMU already assigned)")
    
    # --- CP2102 devices: IMU (or Motor via Modbus) ---
    if classified['cp2102']:
        print(f"  Probing {len(classified['cp2102'])} CP2102 device(s)...")
        for port in classified['cp2102']:
            print(f"    Testing {port}...", end=" ", flush=True)
            
            # Probe Modbus first (motor)
            if motor_port is None and is_zlac8015d_motor(port):
                motor_port = port
                print("→ Motor (Modbus OK)")
                continue
            
            # If not motor, assign to IMU directly (avoid probe port conflict)
            if imu_port is None:
                imu_port = port
                print("→ IMU (CP2102)")
            else:
                print("→ Skipped (IMU already assigned)")
    
    # Fallback for motor if not detected
    if not motor_port:
        print("  Trying Modbus probe for motor...")
        for port in get_all_ttyusb_ports():
            if port != lidar_port and port != imu_port:
                if is_zlac8015d_motor(port):
                    motor_port = port
                    print(f"✓ Detected Motor via Modbus: {port}")
                    break
        if not motor_port:
            all_ports = get_all_ttyusb_ports()
            if all_ports:
                motor_port = all_ports[0]
                print(f"⚠ Motor port not detected — using fallback: {motor_port}")
            else:
                motor_port = '/dev/ttyUSB0'
                print(f"⚠ No serial ports found — using default: {motor_port}")
    
    # Report final status
    print(f"\n  Final assignments:")
    print(f"    Motor: {motor_port}")
    print(f"    LIDAR: {lidar_port if lidar_port else 'NOT DETECTED'}")
    print(f"    IMU:   {imu_port if imu_port else 'NOT DETECTED'}")
    
    return lidar_port, imu_port, motor_port

# ============================================================================
# Use fixed udev symlinks instead of auto-detection
# Symlinks created by /etc/udev/rules.d/99-sahabat-robot.rules:
#   /dev/motor → Motor controller (ZLAC8015D)
#   /dev/lidar → LIDAR (RPLIDAR S2)
#   /dev/imu   → IMU (BNO055)
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
    
    # Control whether to run the IMU
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true' if imu_port else 'false',  # Auto-disable if not detected
        description='Start BNO055 IMU driver (auto-disabled if not detected)'
    )
    use_imu = LaunchConfiguration('use_imu')

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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for the robot description publishers'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_cmd_topic_arg = DeclareLaunchArgument(
        'joy_cmd_topic',
        default_value='cmd_vel',
        description='Joystick velocity output topic'
    )
    joy_cmd_topic = LaunchConfiguration('joy_cmd_topic')

    lidar_scan_topic_arg = DeclareLaunchArgument(
        'lidar_scan_topic',
        default_value='scan',
        description='Raw output topic from the lidar driver'
    )
    lidar_scan_topic = LaunchConfiguration('lidar_scan_topic')

    use_scan_filter_arg = DeclareLaunchArgument(
        'use_scan_filter',
        default_value='false',
        description='Filter lidar_scan_topic into /scan'
    )
    use_scan_filter = LaunchConfiguration('use_scan_filter')

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'shbat_pkg'
    pkg_share = get_package_share_directory(pkg_name)
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
                    {'use_sim_time': use_sim_time}],
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
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {'channel_type': 'serial'},
            {'serial_port': lidar_port_config},
            {'serial_baudrate': 1000000},
            {'frame_id': 'lidar_link'},
            {'angle_min': -3.14},
            {'angle_max': 3.14},
            {'inverted': False},
            {'clockwise': True},
            {'angle_compensate': True},
            {'scan_mode': 'DenseBoost'},
        ],
        remappings=[('scan', lidar_scan_topic)],
        condition=IfCondition(use_lidar)
    )

    node_scan_filter = Node(
        package='shbat_pkg',
        executable='scan_filter',
        name='scan_filter',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'scan_filter.yaml')],
        condition=IfCondition(use_scan_filter)
    )

    node_imu = Node(
        package='witmotion_ros2',
        executable='witmotion_ros2',
        name='witmotion_node',
        output='screen',
        parameters=[{
            'port': imu_port_config,
            'baud_rate': 115200,
            'update_rate': 50.0,
            'frame_id': 'imu_link',
            'topic_name': '/witmotion',
        }],
        remappings=[('/witmotion/imu', '/imu')],
        condition=IfCondition(use_imu)
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
        output='screen',
        remappings=[('cmd_vel', joy_cmd_topic)]
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
        use_imu_arg,
        use_lidar_arg,
        use_kalman_filter_arg,
        motor_port_arg,
        lidar_port_arg,
        imu_port_arg,
        publish_robot_state_arg,
        use_sim_time_arg,
        joy_cmd_topic_arg,
        lidar_scan_topic_arg,
        use_scan_filter_arg,
        
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
        node_scan_filter,
        
        # Motor controller
        node_base_controller
    ])
