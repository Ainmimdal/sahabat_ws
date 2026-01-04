#!/bin/bash
# RTAB-Map Diagnostic Script
# Run this AFTER launching sahabat_rtabmap.launch.py

echo "============================================"
echo "RTAB-Map System Diagnostics"
echo "============================================"
echo ""

echo "1. Checking if RTAB-Map node is running..."
ros2 node list | grep rtabmap
echo ""

echo "2. Checking ZED odometry publishing rate..."
timeout 3 ros2 topic hz /zed2i/zed_node/odom 2>/dev/null || echo "   ⚠️  No odometry data!"
echo ""

echo "3. Checking RTAB-Map map data publishing..."
timeout 3 ros2 topic hz /rtabmap/mapData 2>/dev/null || echo "   ⚠️  No map data! RTAB-Map might not be adding nodes."
echo ""

echo "4. Checking RTAB-Map info..."
timeout 3 ros2 topic hz /rtabmap/info 2>/dev/null || echo "   ⚠️  No info data!"
echo ""

echo "5. Checking RGB image..."
timeout 3 ros2 topic hz /zed2i/zed_node/rgb/image_rect_color 2>/dev/null || echo "   ⚠️  No RGB images!"
echo ""

echo "6. Checking depth image..."
timeout 3 ros2 topic hz /zed2i/zed_node/depth/depth_registered 2>/dev/null || echo "   ⚠️  No depth images!"
echo ""

echo "7. Checking TF tree (odom -> base_link)..."
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null || echo "   ⚠️  TF not available!"
echo ""

echo "8. Checking current RTAB-Map statistics..."
timeout 2 ros2 topic echo /rtabmap/info --once 2>/dev/null | grep -A 5 "loop_closure_id\|local_map_size" || echo "   ⚠️  No stats available"
echo ""

echo "============================================"
echo "TROUBLESHOOTING TIPS:"
echo "============================================"
echo "• If odometry rate is 0 Hz: ZED camera not tracking"
echo "• If mapData rate is 0 Hz: Robot not moving OR thresholds not met"
echo "• To see map build: Move robot >10cm forward/backward"
echo "• For loop closure: Return to previously mapped area"
echo ""
echo "To manually check node count:"
echo "  ros2 topic echo /rtabmap/info --once | grep local_map_size"
echo ""
