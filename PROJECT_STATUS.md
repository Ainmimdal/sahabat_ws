# Sahabat Robot - Project Status

**Last Updated:** February 9, 2026

## ✅ Current Working Setup

### Hardware Configuration

| Component | Model | Connection | Port | Baudrate | Status |
|-----------|-------|------------|------|----------|--------|
| Motor Controller | ZLAC8015D | RS485/FTDI | `/dev/motor` → ttyUSB0 | 115200 | ✅ Working |
| LIDAR | Oradar MS200 | FTDI | `/dev/lidar` → ttyUSB1 | 230400 | ✅ Working (10 Hz) |
| IMU | Wheeltec N100 | CP2102 | `/dev/imu` → ttyUSB2 | 921600 | ✅ Working |
| Camera | ZED 2i | USB 3.0 | Direct | - | ✅ Working |

**Important Notes:**
- IMU must be connected **directly to USB port** (not through hub) - 921600 baud is sensitive to signal quality
- Udev rules installed at `/etc/udev/rules.d/99-sahabat-robot.rules` for consistent device naming
- LIDAR runs at 10 Hz (motor_speed: 10)
- **Angular velocity limited to 0.5 rad/s** to prevent scan mismatch during rotation (10Hz LIDAR sync)

### Robot Physical Specs

| Parameter | Value |
|-----------|-------|
| Wheel Diameter | 175mm (6.5 inch hoverboard wheels) |
| Wheel Radius | 0.0875m |
| Wheel Base | 0.33m (center to center) |
| Robot Radius | ~0.25m |
| Drive Type | Differential drive |
| ZED Mount | 5cm forward, 70cm from floor |

### Software Stack

| Component | Package/Node | Status |
|-----------|--------------|--------|
| Motor Driver | `shbat_pkg/base_controller` | ✅ Working |
| LIDAR Driver | `oradar_lidar/oradar_scan` | ✅ Working |
| Scan Filter | `shbat_pkg/scan_filter` | ✅ Working (filters beam readings + noise) |
| IMU Driver | `wheeltec_n100_imu/imu_node` | ✅ Working |
| EKF Fusion | `robot_localization/ekf_node` | ✅ Working |
| Nav2 Stack | Full navigation stack | ✅ Working |
| SLAM Toolbox | 2D LIDAR SLAM (mapping) | ✅ Working |
| AMCL | Localization with saved map | ✅ Working |
| Joystick | `shbat_pkg/joy2cmd` | ✅ Working + Emergency Stop |
| Waypoint Manager | `shbat_pkg/waypoint_manager` | ✅ Working (GUI) |
| ZED Obstacle Detection | VoxelLayer + PointCloud2 | ✅ Working |

---

## 🚀 Working Features

### 1. Basic Navigation (Nav2)
- **Launch:** `ros2 launch shbat_pkg nav2_test_launch.py`
- Path planning with A* planner
- Regulated Pure Pursuit controller
- Local/global costmaps with obstacle avoidance
- Goal pose via RViz "2D Goal Pose" button

### 2. Joystick Control
- Left stick: Forward/backward + rotation
- **Emergency Stop:** Button A (stops robot immediately)
- **Resume:** Button B (clears emergency stop)

### 3. Sensor Fusion (EKF)
- Fuses wheel odometry + IMU
- Publishes `/odom` topic and `odom → base_link` TF
- Config: `config/ekf.yaml`

### 4. LIDAR Filtering
- Filters out structural beam readings (2040 aluminum extrusions)
- Input: `/scan_raw` → Output: `/scan`
- Config: `config/scan_filter.yaml`

### 5. Udev Rules
- Automatic device naming regardless of plug order
- `/dev/motor`, `/dev/lidar`, `/dev/imu` symlinks

### 6. SLAM Toolbox (2D Mapping)
- **Launch:** `ros2 launch shbat_pkg slam_nav_launch.py mode:=mapping`
- Lightweight 2D SLAM using LIDAR only
- RViz panel for interactive control (save/load maps, pause, localize)
- Noise reduction tuned (scan buffering, minimum score filtering)
- Config: `config/slam_toolbox.yaml`

### 7. Waypoint Manager (GUI)
- **Launch:** `ros2 run shbat_pkg waypoint_manager`
- All-in-one tkinter GUI for waypoint collection and patrol
- Add waypoints by clicking in RViz (2D Goal Pose or Publish Point)
- Add current robot pose as waypoint with button
- Reorder waypoints (up/down), rename, delete
- Navigate to single waypoint or patrol all
- Previous/Next waypoint buttons
- Loop or single-run mode
- Save/load waypoints to YAML file
- Waypoints visualized in RViz as colored markers with direction arrows
- Config: `config/patrol_waypoints.yaml`

### 8. AMCL Localization
- Used in localization mode (with saved map)
- Supports 2D Pose Estimate in RViz for re-localization
- **Global localization** - spreads particles across entire map on startup
- **Auto pose restore** - saves 2D Pose Estimate and restores on next launch
- Config: `config/amcl.yaml`

### 9. API Bridge (LLM/Pi Integration)
- REST API server for external control (e.g., Raspberry Pi with LLM)
- **Launch:** `ros2 run shbat_pkg api_bridge`
- Default port: 5000 (configurable via `API_PORT` env var)
- Listens on all interfaces (0.0.0.0) for network access
- Endpoints:
  - `GET /status` - Battery, position, navigation state, stuck detection
  - `POST /navigate` - Navigate to pose `{"x": 1.0, "y": 2.0, "yaw": 0.0}`
  - `POST /waypoints` - Set waypoint list
  - `POST /patrol/start` - Start waypoint patrol
  - `POST /patrol/stop` - Stop patrol
  - `POST /cancel` - Cancel current navigation
  - `POST /emergency_stop` - Emergency stop
- Pi client library: `scripts/robot_client.py`
- Includes LLM tool definitions for function calling

### 10. Foxglove Bridge (Remote Visualization)
- **Enabled via:** `use_foxglove:=true` launch argument
- Default port: 8765
- Connect from laptop using Foxglove Studio
- **Install:** `sudo apt install ros-humble-foxglove-bridge`
- Perfect for mapping in the field without monitor

### 11. ZED 3D Obstacle Detection (VoxelLayer + PointCloud2)
- **Enabled via:** `use_zed:=true` launch argument
- Uses ZED PointCloud2 directly in costmap VoxelLayer
- **Detects obstacles from 5cm to 100cm height** (catches low and tall obstacles)
- Much better than previous depth_to_laserscan (single horizontal slice)
- ZED pointcloud added to both local and global costmaps
- PointCloud visualization available in RViz (disabled by default)

### 12. Localization + Patrol Launch (Production Ready)
- **Launch:** `ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/path/to/map`
- One-command launch for tour guide operation
- Includes: Localization, Nav2, Waypoint Manager GUI
- Optional: ZED obstacle detection, API Bridge, Foxglove
- **Auto pose saver** - remembers 2D Pose Estimate for next boot

### 13. Auto Pose Saver (NEW)
- Runs automatically in localization mode
- When you use **2D Pose Estimate** in RViz, it saves the pose to `~/.ros/sahabat_saved_pose.yaml`
- On next launch, **automatically publishes saved pose to AMCL** (3 second delay)
- No manual coordinate copying needed!

### 14. Save Current Pose Script
- **Launch:** `ros2 run shbat_pkg save_current_pose`
- Saves robot's current position to `~/.ros/sahabat_home_pose.yaml`
- Prints launch command with exact coordinates for next startup

---

## 📁 Key Configuration Files

| File | Purpose |
|------|---------|
| `config/nav2_odom_only.yaml` | Nav2 parameters (planner, controller, costmaps) |
| `config/ekf.yaml` | EKF sensor fusion settings |
| `config/scan_filter.yaml` | LIDAR angle filtering + noise reduction |
| `config/slam_toolbox.yaml` | SLAM mapping parameters |
| `config/amcl.yaml` | AMCL localization parameters |
| `config/patrol_waypoints.yaml` | Waypoint patrol locations |
| `urdf/sahabat_robot.urdf.xacro` | Robot model (175mm wheels) |
| `rviz/slam_nav.rviz` | RViz config with waypoint markers + PointCloud |
| `/etc/udev/rules.d/99-sahabat-robot.rules` | USB device symlinks |

---

## 📋 Launch Files

| Launch File | Purpose |
|-------------|---------|
| `localization_patrol_launch.py` | **Production** - Localization + Nav2 + Waypoint GUI |
| `slam_nav_launch.py` | SLAM + Nav2 (mapping or localization mode) |
| `nav2_test_launch.py` | Nav2 navigation testing (odom-only) |
| `sahabat_launch.py` | Basic robot bringup (no Nav2) |


---

## 🔧 Quick Commands

```bash
# Source workspace
cd ~/sahabat_ws
source install/setup.bash

# === PRODUCTION: LOCALIZATION + PATROL (RECOMMENDED FOR TOUR GUIDE) ===
# Basic - with saved map and waypoint GUI
ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/gallery

# With ZED obstacle detection (recommended)
ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/gallery use_zed:=true

# With API Bridge for external control (Pi/LLM)
ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/gallery use_zed:=true use_api:=true

# Full production with known start position
ros2 launch shbat_pkg localization_patrol_launch.py \
    map_file:=/home/sahabat/maps/gallery \
    initial_pose_x:=1.5 \
    initial_pose_y:=2.0 \
    initial_pose_yaw:=1.57 \
    use_zed:=true \
    use_api:=true

# === SLAM MAPPING (create new map) ===
ros2 launch shbat_pkg slam_nav_launch.py mode:=mapping

# Save map after mapping (creates .pgm + .yaml for AMCL)
ros2 run nav2_map_server map_saver_cli -f /home/sahabat/maps/my_map

# === SLAM LOCALIZATION (use existing map) ===
# Note: map_file should NOT include .yaml extension
ros2 launch shbat_pkg slam_nav_launch.py mode:=localization map_file:=/home/sahabat/maps/my_map

# With ZED obstacle detection
ros2 launch shbat_pkg slam_nav_launch.py mode:=localization map_file:=/path/to/map use_zed:=true

# === WAYPOINT MANAGER (GUI) ===
ros2 run shbat_pkg waypoint_manager

# === SAVE CURRENT POSE (for auto-localization) ===
ros2 run shbat_pkg save_current_pose

# === NAV2 ONLY (no SLAM) ===
ros2 launch shbat_pkg nav2_test_launch.py

# === EMERGENCY STOP ===
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Clear emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once

# === DIAGNOSTICS ===
ros2 topic echo /odom --once          # Check odometry
ros2 topic echo /imu --once           # Check IMU
ros2 topic echo /scan --once          # Check LIDAR
ros2 topic hz /scan                   # Check LIDAR rate (should be ~10 Hz)
ros2 run tf2_tools view_frames        # Check TF tree

# === CPU/GPU MONITORING ===
htop                                  # CPU usage
jtop                                  # Jetson stats (CPU/GPU/RAM)
nvidia-smi                            # GPU usage

# === API BRIDGE (for Pi/LLM) ===
ros2 run shbat_pkg api_bridge         # Start REST API on port 5000
# Or with custom port:
API_PORT=8080 ros2 run shbat_pkg api_bridge

# === FOXGLOVE (remote visualization from laptop) ===
# First install: sudo apt install ros-humble-foxglove-bridge
ros2 launch shbat_pkg slam_nav_launch.py mode:=mapping use_foxglove:=true use_rviz:=false
# Then connect Foxglove Studio to ws://ROBOT_IP:8765
```

### Map Saving Notes

| Method | Files Created | Can Load With |
|--------|---------------|---------------|
| `map_saver_cli` (recommended) | `.pgm` + `.yaml` | AMCL, map_server |
| slam_toolbox RViz "Save Map" | `.posegraph` + `.data` | slam_toolbox only |

**Always use `map_saver_cli` to save maps** - this creates the standard format that works with AMCL for localization.

---

## 🎯 Next Steps / TODO

- [x] ~~Tune Nav2 parameters for smoother navigation~~
- [x] ~~Add SLAM (using slam_toolbox - lightweight alternative to RTAB-Map)~~
- [x] ~~Implement waypoint patrol~~
- [x] ~~Add Waypoint Manager GUI with visualization~~
- [x] ~~Add AMCL for localization mode (supports 2D Pose Estimate)~~
- [x] ~~Add API Bridge for LLM/Pi integration~~
- [x] ~~Add ZED camera for obstacle detection~~ (VoxelLayer + PointCloud2)
- [x] ~~Auto-localization without manual 2D Pose Estimate~~
- [x] ~~Save current pose script for repeatable startup~~
- [x] ~~Production launch file for tour guide operation~~
- [x] ~~Reduce angular velocity for 10Hz LIDAR sync~~
- [x] ~~Auto pose saver - remembers 2D Pose Estimate for next boot~~
- [x] ~~Goal tolerance tuning - 20cm position, 11° orientation~~
- [ ] Add voice/audio feedback for tour guide functionality
- [ ] Test full patrol workflow in production environment
- [ ] Remote visualization solution for gallery deployment (Foxglove/VNC)
- [ ] Battery monitoring integration
- [ ] AprilTag docking for precise exhibit positioning (optional)

---

## 🐛 Known Issues & Solutions

### Scan Mismatch During Rotation
**Symptom:** LIDAR scan doesn't match walls when robot rotates
**Solution:** Angular velocity limited to 0.5 rad/s (~28°/s) for 10Hz LIDAR sync. At this speed, only 5° rotation per scan.

### Map Orientation Wrong on Boot
**Symptom:** Map appears rotated 90° from laser scan on startup
**Solution:** Use 2D Pose Estimate in RViz to set correct position. The `pose_saver_auto` node will save it and restore on next boot.

### Robot Oscillates at Goal
**Symptom:** Robot keeps retrying when close to waypoint
**Solution:** Goal checker uses `stateful: True` with 20cm tolerance - once reached, stays "reached".

### IMU Won't Open
**Symptom:** `Unable to open serial port /dev/imu`
**Solution:** Connect IMU directly to USB port (not through hub) - 921600 baud is sensitive

### Robot Doesn't Stop at Goal
**Symptom:** Robot overshoots goal position
**Solution:** Check EKF is publishing `/odom`, verify TF tree is complete

### LIDAR Shows False Obstacles
**Symptom:** Beams (2040 extrusions) detected as obstacles
**Solution:** scan_filter node filters these angles - adjust `config/scan_filter.yaml`

### Ghost Obstacles in SLAM Map
**Symptom:** Black dots appear in map where nothing exists
**Solutions:**
- Increased `min_range: 0.15` in scan_filter.yaml (filters close noise)
- Reduced `max_range: 8.0` (far readings are noisy)
- Added `scan_buffer_size: 5` in slam_toolbox.yaml (averages scans)
- Added `minimum_score: 0.5` (rejects poor scan matches)

### RTAB-Map Lags on Orin Nano
**Symptom:** System becomes unresponsive with RTAB-Map
**Solution:** Use slam_toolbox instead (2D LIDAR-only, much lighter on CPU)

---

## 📊 Topic Reference

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | Twist | Nav2/Joystick | Velocity commands |
| `/odom` | Odometry | EKF | Filtered odometry |
| `/wheel_odom` | Odometry | base_controller | Raw wheel odometry |
| `/imu` | Imu | imu_node | IMU data |
| `/scan` | LaserScan | scan_filter | Filtered LIDAR |
| `/scan_raw` | LaserScan | oradar_scan | Raw LIDAR |
| `/zed/zed_node/point_cloud/cloud_registered` | PointCloud2 | ZED | 3D point cloud for obstacle detection |
| `/emergency_stop` | Bool | joy2cmd | Emergency stop trigger |
| `/waypoint_markers` | MarkerArray | waypoint_manager | Waypoint visualization |
| `/goal_pose` | PoseStamped | RViz | Goal for navigation |
| `/initialpose` | PoseWithCovarianceStamped | RViz/pose_saver_auto | Initial pose for AMCL |
| `/robot_status` | String (JSON) | api_bridge | Robot status for external systems |

---

## 🔌 TF Tree

```
map
 └── odom (from AMCL in localization mode, or static in mapping mode)
      └── base_link (from EKF)
           ├── base_footprint
           ├── body_link
           │    ├── lidar_link
           │    ├── front_caster_wheel_link
           │    └── rear_caster_wheel_link
           ├── left_wheel_link
           ├── right_wheel_link
           └── imu_link
```

---

## 🖥️ Waypoint Manager GUI

The Waypoint Manager provides a tkinter GUI for creating and managing patrol routes:

**Features:**
- Mode toggle: Add Waypoints vs Navigate
- Add waypoints via RViz clicks or current robot pose
- Reorder, rename, delete waypoints
- Go to selected/next/previous waypoint
- Start/Pause/Resume/Stop patrol
- Loop mode checkbox
- Save/Load waypoints to YAML
- Waypoints visible in RViz as markers

**Launch:**
```bash
ros2 run shbat_pkg waypoint_manager
```
