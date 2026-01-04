#!/bin/bash
# Navigation System Health Check Script
# Tests all sensors and Nav2 components before navigation

echo "========================================="
echo "Navigation System Health Check"
echo "========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/sahabat/sahabat_ws/install/setup.bash

check_topic() {
    local topic=$1
    local expected_hz=$2
    local name=$3
    
    echo -n "Checking $name ($topic)... "
    
    # Check if topic exists
    if ros2 topic list | grep -q "^${topic}$"; then
        # Get actual Hz
        actual_hz=$(timeout 3 ros2 topic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}')
        
        if [ ! -z "$actual_hz" ]; then
            # Compare with expected
            if (( $(echo "$actual_hz > $expected_hz * 0.5" | bc -l) )); then
                echo -e "${GREEN}✓ OK${NC} (${actual_hz} Hz)"
            else
                echo -e "${YELLOW}⚠ LOW${NC} (${actual_hz} Hz, expected ~${expected_hz} Hz)"
            fi
        else
            echo -e "${RED}✗ NO DATA${NC}"
        fi
    else
        echo -e "${RED}✗ NOT FOUND${NC}"
    fi
}

check_node() {
    local node=$1
    local name=$2
    
    echo -n "Checking $name node... "
    
    if ros2 node list | grep -q "$node"; then
        echo -e "${GREEN}✓ RUNNING${NC}"
    else
        echo -e "${RED}✗ NOT RUNNING${NC}"
    fi
}

check_tf() {
    local parent=$1
    local child=$2
    
    echo -n "Checking TF: $parent → $child... "
    
    if timeout 2 ros2 run tf2_ros tf2_echo $parent $child &>/dev/null; then
        echo -e "${GREEN}✓ OK${NC}"
    else
        echo -e "${RED}✗ FAILED${NC}"
    fi
}

echo "1. CHECKING ROS2 NODES"
echo "----------------------"
check_node "/base_controller" "Base Controller"
check_node "/ekf_filter_node" "EKF Sensor Fusion"
check_node "/zed_node" "ZED Camera"
check_node "/oradar_scan_node" "LiDAR"
check_node "/controller_server" "Nav2 Controller"
check_node "/planner_server" "Nav2 Planner"
echo ""

echo "2. CHECKING SENSOR TOPICS"
echo "-------------------------"
check_topic "/scan" 10 "LiDAR Scan"
check_topic "/zed2i/zed_node/odom" 30 "ZED Odometry"
check_topic "/wheel_odom" 50 "Wheel Odometry"
check_topic "/odometry/filtered" 50 "EKF Fused Odometry"
check_topic "/zed2i/zed_node/point_cloud/cloud_registered" 15 "ZED Point Cloud"
check_topic "/imu" 50 "IMU Data"
echo ""

echo "3. CHECKING TF TREE"
echo "-------------------"
check_tf "map" "odom"
check_tf "odom" "base_link"
check_tf "base_link" "zed2i_camera_link"
check_tf "base_link" "lidar_link"
echo ""

echo "4. CHECKING NAV2 STATUS"
echo "-----------------------"
echo -n "Controller Server lifecycle... "
state=$(ros2 lifecycle get /controller_server 2>/dev/null | grep "active" || echo "inactive")
if [[ $state == *"active"* ]]; then
    echo -e "${GREEN}✓ ACTIVE${NC}"
else
    echo -e "${RED}✗ INACTIVE${NC}"
fi

echo -n "Planner Server lifecycle... "
state=$(ros2 lifecycle get /planner_server 2>/dev/null | grep "active" || echo "inactive")
if [[ $state == *"active"* ]]; then
    echo -e "${GREEN}✓ ACTIVE${NC}"
else
    echo -e "${RED}✗ INACTIVE${NC}"
fi
echo ""

echo "5. CHECKING COSTMAPS"
echo "--------------------"
check_topic "/local_costmap/costmap" 2 "Local Costmap"
check_topic "/global_costmap/costmap" 1 "Global Costmap"
echo ""

echo "========================================="
echo "Health Check Complete"
echo "========================================="
echo ""
echo "If all checks pass (green ✓), you can:"
echo "1. Open RViz and set '2D Pose Estimate'"
echo "2. Send '2D Nav Goal' to test navigation"
echo ""
echo "If checks fail (red ✗):"
echo "- Restart launch file"
echo "- Check hardware connections"
echo "- Review logs: ros2 node list"
echo ""
