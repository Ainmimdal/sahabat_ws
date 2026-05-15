# Sahabat Robot - Project Status

**Last Updated:** May 12, 2026

## ✅ Current Working Setup

### Hardware Configuration

| Component | Model | Connection | Port | Baudrate | Status |
|-----------|-------|------------|------|----------|--------|
| Motor Controller | ZLAC8015D | RS485/FTDI | `/dev/motor` → ttyUSBx | 115200 | ✅ Working |
| LIDAR | RPLIDAR S2 | FTDI (custom) | `/dev/ttyUSBx` (auto-probed) | 1,000,000 | ✅ Working (10 Hz, DenseBoost) |
| IMU | HWT901B (WITMotion) | CH340/CP2102 | `/dev/ttyUSBx` (auto-probed) | 115200 | ✅ Working (USB power-dependent) |
| Camera | ZED 2i | USB 3.0 | Direct | - | ✅ Working |

**Important Notes:**
- **RPLIDAR S2**: Mounted upside-down, 7cm forward of wheel axle. Connected via custom FTDI adapter (original USB cable broken). Needs **direct USB port** (not through hub) for adequate motor power. DenseBoost scan mode at 10 Hz / 32 KHz.
- **HWT901B IMU**: CH340 USB-to-UART adapter. CH341 kernel module conflicts on Jetson — CP2102 adapter recommended if issues persist. Outputs WITMotion protocol at 115200 baud.
- **Smart device detection**: All sensors auto-probed at launch — RPLIDAR by scan data at 1Mbaud, HWT901B by continuous data stream, BNO055 by chip ID query, Motor by known FTDI serial (A50285BI). No fixed port assignments needed.
- **Xbox 360 Controller**: Requires `xpad` kernel module (compiled from source for Jetson kernel). Wireless adapter supported.
- Udev rules at `udev/99-sahabat-robot.rules` (must be installed to `/etc/udev/rules.d/`).
- **Angular velocity limited to 0.5 rad/s** to prevent scan mismatch during rotation (10Hz LIDAR sync).

### Robot Physical Specs

| Parameter | Value |
|-----------|-------|
| Wheel Diameter | 175mm (6.5 inch hoverboard wheels) |
| Wheel Radius | 0.0875m |
| Wheel Base | 0.33m (center to center) |
| Robot Radius | ~0.25m |
| Drive Type | Differential drive |
| LIDAR Mount | 7cm forward of wheel axle, 25cm above body_link |
| ZED Mount | 5cm forward, 70cm from floor |

### URDF Notes
- `lidar_link`: positioned at `xyz="0.22 0 0.25"` from body_link (net +0.07m from base_link)
- `lidar_joint`: rotated `rpy="3.14 0 3.14"` — roll=π fixes upside-down mirroring, yaw=π fixes front/back orientation
- Right motor: mechanically inverted (set_velocity() handles the `-` sign)
- Odometry direction: matched to physical robot motion after motor polarity fix

### Software Stack

| Component | Package/Node | Status |
|-----------|--------------|--------|
| Motor Driver | `shbat_pkg/base_controller` | ✅ Working |
| LIDAR Driver | `rplidar_ros/rplidar_node` | ✅ Working (DenseBoost, 10 Hz) |
| Scan Filter | `shbat_pkg/scan_filter` | ✅ Working (180° front-only FOV) |
| IMU Driver | `witmotion_ros2/witmotion_ros2` | ✅ Working (CH340 USB-dependent) |
| EKF Fusion | `robot_localization/ekf_node` | ✅ Working |
| Nav2 Stack | Full navigation stack | ✅ Working (odom-only mode) |
| SLAM Toolbox | 2D LIDAR SLAM (mapping) | ✅ Working |
| AMCL | Localization with saved map | ✅ Working |
| Joystick | `shbat_pkg/joy2cmd` | ✅ Working + Emergency Stop |
| Waypoint Manager | `shbat_pkg/waypoint_manager` | ✅ Working (GUI) |
| ZED Obstacle Detection | VoxelLayer + PointCloud2 | ✅ Working |

---

## 🚀 Working Features

### 1. Basic Navigation (Nav2 — Odom-Only)
- **Launch:** `ros2 launch shbat_pkg nav2_test_launch.py`
- Global costmap: rolling window (no map needed), voxel + inflation layers
- Path planning with A* planner through free space + LIDAR obstacles
- Goal pose via RViz "2D Goal Pose" button

### 2. Joystick Control
- Left stick: Forward/backward + rotation (Xbox 360 mapping)
- **Emergency Stop:** Button A (stops robot immediately)
- **Resume:** Button B (clears emergency stop)

### 3. Sensor Fusion (EKF)
- Fuses wheel odometry + IMU
- Publishes `/odom` topic and `odom → base_link` TF
- Config: `config/ekf.yaml`

### 4. LIDAR Filtering
- Input: `/scan_raw` → Output: `/scan`
- RPLIDAR S2 mounted upside-down + backward: TF `rpy="3.14 0 3.14"` corrects orientation
- 180° front-only FOV: filters rear half `[-90°, 90°]` where robot body/beams are
- Config: `config/scan_filter.yaml`

### 5. Smart Device Detection
- Protocol-based probing at launch — no fixed USB port numbers
- RPLIDAR S2: scan data at 1,000,000 baud
- HWT901B: continuous WITMotion data stream (0x55 headers)
- BNO055 fallback: UART chip ID query (0xAA 0x01 command)
- Motor: known FTDI serial (A50285BI) or Modbus probe
- Auto-disables sensors not detected

### 6. SLAM Toolbox (2D Mapping)
- **Launch:** `ros2 launch shbat_pkg slam_nav_launch.py mode:=mapping`
- Lightweight 2D SLAM using LIDAR only
- Config: `config/slam_toolbox.yaml`

### 7. Waypoint Manager (GUI)
- **Launch:** `ros2 run shbat_pkg waypoint_manager`
- All-in-one tkinter GUI for waypoint collection and patrol
- **Save Waypoints** — saves to `patrol_waypoints.yaml`
- **Save as Exhibits** — exports waypoints to `exhibit_routes.yaml` for Pi/LLM navigation
  - Use **Rename** button to set meaningful names (e.g., `station_1`, `entrance`) before exporting
  - Preserves existing routes and settings in the exhibit config
- Config: `config/patrol_waypoints.yaml`, `config/exhibit_routes.yaml`

### 8. Exhibit Navigator + Pi/LLM Integration
- **Launch:** `ros2 run shbat_pkg exhibit_navigator`
- Headless node for route-based navigation controlled by external systems
- Listens on `/exhibit_command` topic for commands
- Publishes events to `/robot_events` and `/exhibit_arrival`
- API Bridge exposes `POST /exhibit/goto` HTTP endpoint on port 5000
- Pi sends: `{"exhibit": "station_1"}` → robot navigates to that exhibit
- Supports tour mode, pause/resume, obstruction detection
- Config: `config/exhibit_routes.yaml`

### 9. API Bridge (LLM/Pi Integration)
- REST API server for external control
- **Launch:** `ros2 run shbat_pkg api_bridge` or `use_api:=true` in launch
- Default port: 5000
- Endpoints:
  - `POST /exhibit/goto` — Navigate to exhibit `{"exhibit": "station_1"}`
  - `POST /navigate` — Navigate to pose `{"x": 1.0, "y": 2.0, "yaw": 0.0}`
  - `POST /emergency_stop` — Emergency stop
  - `POST /cancel` — Cancel current navigation
  - `GET /status` — Robot status

### 10. Auto-Reconnect RPLIDAR
- Probe does DTR power cycle (1s off, 1s on) + STOP/RESET before each launch
- `respawn=True, respawn_delay=3.0` on rplidar_node — auto-retries on health status 2
- LIDAR recovers without manual replug in most cases

### 9. API Bridge (LLM/Pi Integration)
- REST API server for external control
- **Launch:** `ros2 run shbat_pkg api_bridge`
- Default port: 5000

### 10. Foxglove Bridge (Remote Visualization)
- **Enabled via:** `use_foxglove:=true` launch argument
- Default port: 8765

### 11. ZED 3D Obstacle Detection (VoxelLayer + PointCloud2)
- **Enabled via:** `use_zed:=true` launch argument

### 12. Localization + Patrol Launch (Production Ready)
- **Launch:** `ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/path/to/map`

---

## 📁 Key Configuration Files

| File | Purpose |
|------|---------|
| `config/nav2_odom_only.yaml` | Nav2 parameters (planner, controller, rolling costmaps) |
| `config/ekf.yaml` | EKF sensor fusion settings |
| `config/scan_filter.yaml` | LIDAR angle filtering (180° front-only FOV) |
| `config/slam_toolbox.yaml` | SLAM mapping parameters |
| `config/amcl.yaml` | AMCL localization parameters |
| `config/patrol_waypoints.yaml` | Waypoint patrol locations |
| `config/exhibit_routes.yaml` | Exhibit coordinates + routes for Pi/LLM navigation |
| `urdf/sahabat_robot.urdf.xacro` | Robot model (RPLIDAR at +7cm, lidar_joint rpy=3.14 0 3.14) |
| `rviz/slam_nav.rviz` | RViz config with waypoint markers + PointCloud |
| `udev/99-sahabat-robot.rules` | USB device symlinks |

---

## 📋 Launch Files

| Launch File | Purpose |
|-------------|---------|
| `localization_patrol_launch.py` | **Production** - Localization + Nav2 + Waypoint GUI |
| `slam_nav_launch.py` | SLAM + Nav2 (mapping or localization mode) |
| `nav2_test_launch.py` | Nav2 navigation testing (odom-only, rolling global costmap) |
| `sahabat_launch.py` | Basic robot bringup (no Nav2) |

---

## 🔧 Quick Commands

```bash
# Source workspace
cd ~/sahabat_ws
source install/setup.bash

# === NAV2 ODOM-ONLY (no map needed) ===
ros2 launch shbat_pkg nav2_test_launch.py

# === BASIC BRINGUP (joystick only) ===
ros2 launch shbat_pkg sahabat_launch.py

# === PRODUCTION: LOCALIZATION + PATROL ===
ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/gallery

# With ZED obstacle detection
ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/home/sahabat/maps/gallery use_zed:=true

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

# Save map after mapping
ros2 run nav2_map_server map_saver_cli -f /home/sahabat/maps/my_map

# === SLAM LOCALIZATION (use existing map) ===
ros2 launch shbat_pkg slam_nav_launch.py mode:=localization map_file:=/home/sahabat/maps/my_map

# === WAYPOINT MANAGER (GUI) ===
ros2 run shbat_pkg waypoint_manager

# === SAVE CURRENT POSE ===
ros2 run shbat_pkg save_current_pose

# === EMERGENCY STOP ===
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once

# === DIAGNOSTICS ===
ros2 topic echo /odom --once
ros2 topic echo /imu --once
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 run tf2_tools view_frames

# === XBOX CONTROLLER ===
# Load xpad kernel module (compiled from source)
sudo insmod /tmp/xpad/xpad.ko
ls /dev/input/js*
```

---

## 🎯 Next Steps / TODO

- [x] ~~Hardware swap: RPLIDAR S2 + HWT901B IMU~~
- [x] ~~Smart probe-based device detection~~
- [x] ~~Fix RPLIDAR orientation (upside-down + backward mounting)~~
- [x] ~~Fix motor polarity and odometry direction~~
- [x] ~~Fix Nav2 global costmap for odom-only mode (rolling window)~~
- [x] ~~Fix Xbox 360 joystick mapping~~
- [ ] Make xpad kernel module persistent across reboots
- [ ] Fix HWT901B CH340 port lock after shutdown
- [ ] Add voice/audio feedback for tour guide functionality
- [ ] Test full patrol workflow in production environment
- [ ] Remote visualization solution for gallery deployment (Foxglove/VNC)
- [ ] Battery monitoring integration
- [ ] AprilTag docking for precise exhibit positioning (optional)

---

## 🐛 Known Issues & Solutions

### RPLIDAR Internal Error (Health Status 2)
**Symptom:** `RPLidar internal error detected` on startup
**Solution:** RPLIDAR needs sufficient USB power — plug directly into Jetson USB port (not through hub). May need power cycle (unplug/replug) between runs.

### HWT901B IMU Port Lock After Shutdown
**Symptom:** `Failed to open the serial port: Input/output error` on relaunch
**Solution:** CH340 driver on Jetson sometimes locks the port. Unplug/replug the HWT901B USB adapter. Alternatively, use a CP2102 adapter.

### Scan Mismatch During Rotation
**Symptom:** LIDAR scan doesn't match walls when robot rotates
**Solution:** Angular velocity limited to 0.5 rad/s (~28°/s).

### Map Orientation Wrong on Boot
**Symptom:** Map appears rotated 90° from laser scan on startup
**Solution:** Use 2D Pose Estimate in RViz to set correct position. `pose_saver_auto` saves and restores on next boot.

### Robot Doesn't Move to Goal Pose
**Symptom:** Nav2 receives goal but robot doesn't move
**Solution:** Check TF tree (`ros2 run tf2_tools view_frames`). EKF must publish `odom → base_link`. Check IMU is running.

### IMU Driver Not Detected
**Symptom:** witmotion_ros2 node exits with serial error
**Solution:** The probe may have locked the CH340 port. Replug IMU and relaunch. CH340 is auto-assigned to IMU without probing.

---

## 📊 Topic Reference

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | Twist | Nav2/Joystick | Velocity commands |
| `/odom` | Odometry | EKF | Filtered odometry |
| `/wheel_odom` | Odometry | base_controller | Raw wheel odometry |
| `/imu` | Imu | witmotion_node | IMU data (remapped from /witmotion/imu) |
| `/scan` | LaserScan | scan_filter | Filtered LIDAR |
| `/scan_raw` | LaserScan | rplidar_node | Raw LIDAR (remapped from /scan) |
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
           │    ├── lidar_link (rpy=3.14 0 3.14, 7cm forward of axle)
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
