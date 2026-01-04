# ZLAC8015D Motor Driver Integration

## Overview
This document summarizes the ZLAC8015D dual-channel AC servo motor driver integration for a differential drive robot with ROS2 Humble.

## Hardware Specifications
- **Motor Driver**: ZLAC8015D (RS485 Modbus RTU)
- **Wheels**: 173mm diameter (marketed as 6.5 inch)
- **Encoder**: 4096 PPR → 16384 CPR (quadrature decoded)
- **Wheel Base**: 0.33m (330mm between wheel centers)
- **Communication**: USB-to-RS485 adapter (FTDI FT232 - 0403:6001)
- **Baud Rate**: 115200, 8N1
- **Modbus Slave ID**: 1

## Key Parameters
```python
wheel_radius = 0.0865  # meters (173mm / 2)
wheel_base = 0.33      # meters
cpr = 16384            # Counts per revolution
travel_per_rev = 2 * π * 0.0865 = 0.5435  # meters
```

## Motor Direction Configuration
- **Motors are mounted facing opposite directions**
- **Right motor is inverted** in software for standard differential drive
- When commanding forward: Left = +RPM, Right = -RPM (raw)
- Both wheels report positive velocity for forward motion after inversion

## Files Created/Modified

### 1. ZLAC8015D Driver Module
**Path**: `src/shbat_pkg/shbat_pkg/zlac8015d/ZLAC8015D.py`

Key features:
- pymodbus 3.x compatible (uses `device_id=` not `slave=`)
- Proper signed 16-bit handling for RPM feedback
- Right motor inversion for differential drive
- Velocity control, position control, and odometry support

Important methods:
```python
# Initialize
motors = ZLAC8015D.Controller(port='/dev/ttyUSB0')

# Enable/disable
motors.enable_motor()
motors.disable_motor()

# Set mode (3 = velocity control)
motors.set_mode(3)

# Set velocity (m/s) - handles motor inversion internally
motors.set_velocity(left_vel, right_vel)

# Get velocities (m/s) - both positive = forward
vl, vr = motors.get_linear_velocities()

# Get raw RPM feedback
l_rpm, r_rpm = motors.get_rpm()

# Get encoder ticks
l_ticks, r_ticks = motors.get_wheels_tick()
```

### 2. Base Controller Node
**Path**: `src/shbat_pkg/shbat_pkg/base_controller.py`

Features:
- Subscribes to `/cmd_vel` (geometry_msgs/Twist)
- Publishes `/wheel_odom` (nav_msgs/Odometry) at 20Hz
- Publishes TF: odom → base_link
- Watchdog timer stops motors if no cmd_vel received
- Covariance matrices for sensor fusion compatibility

Parameters:
```yaml
port: '/dev/ttyUSB0'
wheel_radius: 0.0865
wheel_base: 0.33
max_linear_vel: 1.0
max_angular_vel: 2.0
odom_topic: 'wheel_odom'
odom_rate: 20.0
cmd_vel_timeout: 0.5
```

### 3. Joystick to Velocity Node
**Path**: `src/shbat_pkg/shbat_pkg/joy2cmd.py`

Safe speed limits:
```python
max_linear_speed = 0.5   # m/s
max_angular_speed = 1.0  # rad/s
```

### 4. Launch File with Smart USB Detection
**Path**: `src/shbat_pkg/launch/sahabat_launch.py`

Features:
- **Protocol-based device detection**: Probes FTDI devices to identify Motor vs LIDAR
- Sends Modbus query to identify motor controller
- Checks for LIDAR data stream to identify LIDAR
- Auto-disables unavailable devices (LIDAR, IMU)
- Conditional node launching based on detected hardware

USB Device IDs:
```
LIDAR (Oradar MS200):  FTDI 0403:6001 (230400 baud)
Motor RS485:           FTDI 0403:6001 (115200 baud, Modbus)
IMU (Wheeltec N100):   CP2102 10c4:ea60
Motor RS485 (alt):     CH340 1a86:7523
```

## Testing Commands

### Test Motor Directly
```bash
python3 << 'EOF'
import sys
sys.path.insert(0, '/home/sahabat/sahabat_ws/src/shbat_pkg/shbat_pkg')
from zlac8015d import ZLAC8015D
import time

motors = ZLAC8015D.Controller(port='/dev/ttyUSB0')
motors.enable_motor()
motors.set_mode(3)

# Forward at 0.1 m/s
motors.set_velocity(0.1, 0.1)
time.sleep(2)

# Stop
motors.set_rpm(0, 0)
motors.disable_motor()
EOF
```

### Launch Robot
```bash
cd ~/sahabat_ws
source install/setup.bash
ros2 launch shbat_pkg sahabat_launch.py
```

### Monitor Odometry
```bash
# Position
ros2 topic echo /wheel_odom --field pose.pose.position

# Velocity
ros2 topic echo /wheel_odom --field twist.twist
```

### View in RViz
```bash
rviz2
# Set Fixed Frame = "odom"
# Add: RobotModel
# Add: By topic → /wheel_odom → Odometry
```

## Differential Drive Kinematics

### Forward Kinematics (wheel → robot)
```python
v_robot = (v_right + v_left) / 2.0
omega_robot = (v_right - v_left) / wheel_base
```

### Inverse Kinematics (robot → wheel)
```python
v_left = v_robot - (omega_robot * wheel_base / 2.0)
v_right = v_robot + (omega_robot * wheel_base / 2.0)
```

## Dependencies
- pymodbus >= 3.0
- pyserial
- numpy
- ROS2 Humble

## Next Steps
1. **Add LIDAR** (Oradar MS200) for obstacle detection and mapping
2. **Add IMU** (Wheeltec N100) for orientation
3. **Configure EKF** (robot_localization) to fuse wheel_odom + IMU
4. **Test Nav2** for autonomous navigation
5. **Integrate ZED2i** camera for visual odometry and depth sensing

## Troubleshooting

### Modbus Communication Error
- Check motor controller is powered (48V DC)
- Verify RS485 wiring: A+ to A+, B- to B-
- Ensure correct port: `ls -la /dev/ttyUSB*`

### Robot Rotates Instead of Going Straight
- Motor inversion issue - check `set_velocity()` and `get_linear_velocities()`
- One motor should be inverted (currently right motor)

### Odometry Drift
- Verify wheel_radius (measure actual wheel diameter)
- Check wheel_base (measure center to center)
- Encoder CPR should match motor spec (16384 for 4096 PPR motor)
