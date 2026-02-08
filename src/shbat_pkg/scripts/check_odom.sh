#!/bin/bash
# Odometry diagnostics script
# Run this while the robot is running to check if odometry is working

echo "========================================"
echo "ODOMETRY DIAGNOSTICS"
echo "========================================"
echo ""

# Source ROS
source /opt/ros/humble/setup.bash
source /home/sahabat/sahabat_ws/install/setup.bash

echo "1. Checking /wheel_odom topic (from base_controller)..."
timeout 3 ros2 topic echo /wheel_odom --once 2>/dev/null
if [ $? -ne 0 ]; then
    echo "   ERROR: /wheel_odom not publishing!"
else
    echo "   OK: /wheel_odom is publishing"
fi
echo ""

echo "2. Checking /odom topic (from EKF)..."
timeout 3 ros2 topic echo /odom --once 2>/dev/null
if [ $? -ne 0 ]; then
    echo "   ERROR: /odom not publishing!"
else
    echo "   OK: /odom is publishing"
fi
echo ""

echo "3. Checking /imu topic..."
timeout 3 ros2 topic echo /imu --once 2>/dev/null
if [ $? -ne 0 ]; then
    echo "   WARNING: /imu not publishing (EKF may work with just wheel odom)"
else
    echo "   OK: /imu is publishing"
fi
echo ""

echo "4. Checking TF tree (odom -> base_link)..."
ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10
echo ""

echo "5. Checking topic frequencies..."
echo "   /wheel_odom:"
timeout 3 ros2 topic hz /wheel_odom 2>&1 | head -3
echo ""
echo "   /odom:"
timeout 3 ros2 topic hz /odom 2>&1 | head -3
echo ""

echo "6. Active nodes:"
ros2 node list 2>/dev/null | grep -E "base_controller|ekf|imu"
echo ""

echo "========================================"
echo "DIAGNOSIS COMPLETE"
echo "========================================"
