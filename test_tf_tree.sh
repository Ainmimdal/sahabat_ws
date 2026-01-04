#!/bin/bash
# Test TF Tree for Pure ZED Localization

echo "==========================================="
echo "Testing TF Tree Structure"
echo "==========================================="
echo ""

echo "Waiting 3 seconds for TF data..."
sleep 3

echo "1. Checking odom → zed2i_camera_link (ZED publishes this)"
timeout 2 ros2 run tf2_ros tf2_echo odom zed2i_camera_link 2>/dev/null && echo "   ✓ Transform exists" || echo "   ✗ MISSING - ZED not publishing!"
echo ""

echo "2. Checking zed2i_camera_link → base_link (URDF defines this)"
timeout 2 ros2 run tf2_ros tf2_echo zed2i_camera_link base_link 2>/dev/null && echo "   ✓ Transform exists" || echo "   ✗ MISSING - robot_state_publisher not running!"
echo ""

echo "3. Checking full chain: odom → base_link"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null && echo "   ✓ Full chain working!" || echo "   ✗ BROKEN - check above transforms"
echo ""

echo "4. Checking map → odom (RTAB-Map should publish this)"
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>/dev/null && echo "   ✓ RTAB-Map running" || echo "   ⚠ RTAB-Map not started yet or not publishing map"
echo ""

echo "5. Generating TF tree PDF..."
cd /tmp
ros2 run tf2_tools view_frames
if [ -f "frames.pdf" ]; then
    echo "   ✓ frames.pdf created in /tmp"
    echo "   View with: evince /tmp/frames.pdf"
else
    echo "   ✗ Failed to generate frames.pdf"
fi
echo ""

echo "6. Checking camera pitch (should be close to 0 or small negative)"
echo "   If robot appears tilted in RViz, pitch is wrong!"
timeout 2 ros2 run tf2_ros tf2_echo zed2i_camera_link base_link 2>/dev/null | grep -A 3 "Rotation" || echo "   Can't check rotation"
echo ""

echo "==========================================="
echo "Expected TF Tree:"
echo "==========================================="
echo "map (RTAB-Map)"
echo " └─ odom (ZED)"
echo "     └─ zed2i_camera_link (ZED publishes)"
echo "         └─ base_link (robot_state_publisher)"
echo "             └─ [wheels, lidar, etc]"
echo ""
