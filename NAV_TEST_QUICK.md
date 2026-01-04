# Navigation Test - Quick Start

## 🚀 Run It

```bash
# Launch everything
ros2 launch shbat_pkg sahabat_navigation_test.launch.py use_rviz:=true

# Check if working
./test_navigation_setup.sh
```

In RViz:
1. Click "2D Pose Estimate" → set robot position
2. Click "2D Nav Goal" → robot navigates there

## 🗺️ Add SLAM (optional - for mapping)

```bash
# Terminal 2: Start SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(ros2 pkg prefix shbat_pkg)/share/shbat_pkg/config/mapper_params_online_async.yaml \
  use_sim_time:=false

# Save map when done
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 🐛 If broken

```bash
# Check what's wrong
./test_navigation_setup.sh

# Check topics
ros2 topic hz /scan
ros2 topic hz /zed2i/zed_node/odom
ros2 topic hz /odometry/filtered
```

## What's running

- Base robot (motors, LiDAR, IMU)
- ZED camera (visual odometry, depth)
- EKF sensor fusion
- Nav2 navigation

Done!
