# ZED2i + RTAB-Map Setup Guide for Gallery Tour Guide Robot

## Overview

This setup adds visual SLAM capabilities to your robot using a **multi-sensor fusion approach**:
- **ZED2i stereo camera** - Visual odometry (primary) + depth and RGB
- **Wheel encoders + N100 IMU** - Backup odometry (kalman_filter → /wheel_odom)
- **M200 LiDAR** - Obstacle avoidance + additional SLAM features
- **RTAB-Map** - Visual SLAM with loop closure detection

**Architecture (Middle Ground - Best of Both Worlds):**
```
Wheel encoders + N100 IMU → kalman_filter → /wheel_odom (backup odometry)
                                                              
ZED2i camera → Visual Odometry → /odom (primary, most accurate)
            ↓
            RGB-D depth ──────────→ RTAB-Map ← /odom (from ZED)
                                         ↓
M200 LiDAR → /scan ─────────────────→ RTAB-Map + Nav2
                                         ↓
                                  Loop-closed map + localization
```

**Why this approach:**
- ✅ Uses ZED visual odometry (best accuracy in good lighting)
- ✅ Keeps wheel+IMU odometry as backup (works in all conditions)
- ✅ LiDAR for obstacles and additional features
- ✅ RTAB-Map adds loop closures for drift correction
- ✅ All sensors utilized, nothing wasted
- ✅ Simple setup, robust performance

All components work together without replacing your existing working system.

---

## What Was Added

### 1. Modified Files
- `src/shbat_pkg/shbat_pkg/kalman_filter.py` 
  - Now publishes to `/wheel_odom` (backup odometry)
  - Added covariance for sensor fusion
  - Uses different TF frame (`wheel_odom->base_link`)

### 2. New Files Created
- `src/shbat_pkg/config/rtabmap_params.yaml` - RTAB-Map configuration
- `src/shbat_pkg/launch/sahabat_rtabmap.launch.py` - Launch file for ZED + RTAB-Map
- `src/shbat_pkg/rviz/rtabmap.rviz` - Pre-configured RViz visualization
- `RTABMAP_SETUP.md` - This guide

### 3. Launch Configuration
- ZED positional tracking enabled (publishes `/odom` with visual odometry)
- RTAB-Map subscribes to ZED's `/odom` (primary odometry source)
- Wheel+IMU odometry available on `/wheel_odom` (backup/validation)

---

## Hardware Setup

### Camera Mounting (Your Current Setup)
Your ZED2i is mounted:
- **Height**: 0.67m above base_link (center of robot)
- **Position**: Center of robot (0.05m forward/back, 0m left/right)
- **Pitch**: 8° downward
- **Connection**: USB (not network)

These values are set as defaults in the launch file. If you change the mount, adjust via launch arguments.

### USB Connection (ZED2i)
1. Connect ZED2i to Jetson via USB 3.0
2. Verify camera detected:
   ```bash
   lsusb | grep -i stereolabs
   # Should show: "Stereolabs ZED"
   ```
3. Check camera device:
   ```bash
   ls /dev/video*
   # ZED typically uses /dev/video0 and /dev/video1
   ```

---

## Installation

### 1. Install RTAB-Map (if not already installed)

**Option A: Binary install (recommended for speed)**
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

**Option B: From source (for latest features)**
```bash
cd ~/sahabat_ws/src
git clone https://github.com/introlab/rtabmap.git
git clone https://github.com/introlab/rtabmap_ros.git
cd ~/sahabat_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Verify ZED SDK is installed
```bash
ls /usr/local/zed
# Should show SDK folders: lib, include, tools, etc.
```

### 3. Build the workspace
```bash
cd ~/sahabat_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Usage

### Basic Workflow

**Terminal 1: Start base robot (wheels, IMU, LiDAR, odometry)**
```bash
cd ~/sahabat_ws
source install/setup.bash
ros2 launch shbat_pkg sahabat_launch.py
```

**Terminal 2: Start ZED + RTAB-Map**
```bash
cd ~/sahabat_ws
source install/setup.bash
ros2 launch shbat_pkg sahabat_rtabmap.launch.py
```

That's it! The launch file will auto-detect your USB ZED2i.

### Launch Arguments

Customize camera mount and behavior:

```bash
# If you change mount position (example: 0.5m high, 0.2m forward, -10 deg pitch)
ros2 launch shbat_pkg sahabat_rtabmap.launch.py \
  camera_height:=0.5 \
  camera_forward:=0.2 \
  camera_pitch:=-0.1745

# Start fresh (delete old map)
ros2 launch shbat_pkg sahabat_rtabmap.launch.py delete_db:=true

# Localization mode (use existing map, no new mapping)
ros2 launch shbat_pkg sahabat_rtabmap.launch.py localization_mode:=true

# Use specific camera by serial number (if multiple ZED cameras)
ros2 launch shbat_pkg sahabat_rtabmap.launch.py serial_number:=12345678

# Network/IP mode (if you switch to network camera later)
ros2 launch shbat_pkg sahabat_rtabmap.launch.py \
  use_network:=true \
  camera_ip:=192.168.1.10
```

**Available arguments:**
- `camera_height` - Mount height in meters (default: 1.0)
- `camera_forward` - Forward offset in meters (default: 0.0 = center)
- `camera_pitch` - Downward tilt in radians (default: -0.13963 = -8°)
- `serial_number` - USB camera serial (default: 0 = first available)
- `use_network` - Use IP camera instead of USB (default: false)
- `camera_ip` - IP address if use_network=true (default: 192.168.1.10)
- `use_rviz` - Launch RViz2 (default: true)
- `delete_db` - Delete existing map (default: false)
- `localization_mode` - Use existing map only (default: false)

---

## Verification & Testing

### 1. Check Topics
```bash
# List all topics
ros2 topic list

# Expected topics:
# /odom                          <- Your wheel+IMU odometry
# /scan                          <- ORADAR LiDAR
# /zed2i/zed_node/rgb/image_rect_color  <- ZED camera
# /zed2i/zed_node/depth/depth_registered
# /rtabmap/grid_map              <- RTAB-Map occupancy grid
# /rtabmap/mapData               <- Map data for loop closure
```

### 2. Visualize Camera Stream
```bash
# View RGB image
ros2 run rqt_image_view rqt_image_view
# Select: /zed2i/zed_node/rgb/image_rect_color

# View depth
ros2 run rqt_image_view rqt_image_view
# Select: /zed2i/zed_node/depth/depth_registered
```

### 3. Check TF Tree
```bash
# Generate TF tree PDF
ros2 run tf2_tools view_frames

# View in terminal
ros2 run tf2_ros tf2_echo map base_link
# Should show transform updating as robot moves
```

### 4. Monitor RTAB-Map Stats
```bash
# Watch loop closures and map quality
ros2 topic echo /rtabmap/info

# Check how many locations in map
ros2 service call /rtabmap/get_map_data rtabmap_msgs/srv/GetMap
```

### 5. Test Drive
1. Drive robot around gallery slowly (0.2-0.5 m/s)
2. Watch RViz - you should see:
   - Point cloud building up
   - Occupancy grid forming
   - Loop closures when revisiting areas (green links in graph)
3. Return to start - RTAB-Map should recognize and correct drift

---

## Tuning Tips

### If odometry drifts badly:
Edit `kalman_filter.py` covariance values (smaller = more trusted):
```python
odom.pose.covariance[0] = 0.005   # Reduce x variance
odom.pose.covariance[7] = 0.005   # Reduce y variance
odom.pose.covariance[35] = 0.03   # Reduce yaw variance
```

### If RTAB-Map uses too much memory on Jetson:
Edit `config/rtabmap_params.yaml`:
```yaml
Mem/ImagePreDecimation: 4  # Increase (was 2)
Mem/STMSize: 20  # Decrease (was 30)
Kp/MaxFeatures: 200  # Decrease (was 400)
```

### If loop closures fail:
```yaml
Rtabmap/DetectionRate: 2.0  # Increase (was 1.0)
Kp/MaxFeatures: 600  # Increase (was 400)
```

### If performance is slow:
In launch file, lower ZED resolution:
```python
'video.resolution': 3,  # VGA (was 2=HD720)
'video.grab_frame_rate': 10,  # Lower FPS (was 15)
```

---

## Troubleshooting

### Camera not detected
```bash
# Check USB connection
lsusb | grep -i stereolabs

# Check video devices
ls -la /dev/video*

# Check ZED SDK
ls /usr/local/zed
# If missing, reinstall ZED SDK

# Check ZED wrapper built correctly
ros2 pkg list | grep zed

# Test camera with ZED tools
/usr/local/zed/tools/ZED_Explorer  # or ZED_Diagnostic
```

### RTAB-Map crashes / high memory
```bash
# Reduce memory usage
export RTABMAP_MAX_MEMORY=2048

# Or edit launch file rtabmap_env variable
```

### No loop closures detected
- Drive slower (0.2-0.3 m/s) so camera can track features
- Ensure good lighting in gallery
- Avoid reflective surfaces / glass directly in view
- Check feature detection: `ros2 topic echo /rtabmap/info`

### TF errors "frame does not exist"
```bash
# Check TF tree
ros2 run tf2_ros tf2_monitor

# Verify static transforms published
ros2 topic echo /tf_static
# Should include base_link -> zed_camera_center
```

### Odometry jumps / discontinuities
- Verify IMU is publishing: `ros2 topic hz /imu`
- Check wheel RPM topics: `ros2 topic echo /rpm_left`
- Ensure kalman_filter node running: `ros2 node list | grep kalman`

---

## Map Saving & Loading

### Save current map
```bash
# RTAB-Map saves automatically to ~/.ros/rtabmap.db

# Export to different location
ros2 service call /rtabmap/set_mode_mapping rtabmap_msgs/srv/SetMode "{}"
# Map is at ~/.ros/rtabmap.db - copy it:
cp ~/.ros/rtabmap.db ~/my_gallery_map.db
```

### Load existing map (localization mode)
```bash
# Copy map to default location
cp ~/my_gallery_map.db ~/.ros/rtabmap.db

# Launch in localization mode
ros2 launch shbat_pkg sahabat_rtabmap.launch.py \
  camera_ip:=192.168.1.10 \
  localization_mode:=true
```

---

## Performance Benchmarks (Jetson Orin Nano)

Expected performance with default settings:
- **CPU Usage**: 40-60% (4 cores)
- **Memory**: 2-3 GB
- **Camera FPS**: 10-15 Hz
- **RTAB-Map update rate**: 1-2 Hz
- **Odometry rate**: 50 Hz

If performance issues, follow tuning tips above.

---

## Integration with Nav2

Once you have a good map, integrate with Nav2:

1. **Map server** - Serve RTAB-Map's occupancy grid
2. **AMCL** - Use RTAB-Map's localization directly (disable AMCL)
3. **Costmaps** - Subscribe to `/rtabmap/grid_map` and ZED point cloud

This is already partially configured in your `nav2_params.yaml`. The RTAB-Map grid_map will be used by Nav2 costmap layers.

---

## Next Steps

1. ✅ Test camera and verify topics
2. ✅ Drive robot around gallery to build initial map
3. ✅ Verify loop closures work when revisiting areas
4. ✅ Save map for production use
5. ✅ Tune parameters if needed
6. ✅ Integrate with Nav2 for autonomous navigation

---

## Support

**Check logs:**
```bash
# RTAB-Map node logs
ros2 node info /rtabmap

# ZED wrapper logs  
ros2 node info /zed_node
```

**Useful commands:**
```bash
# Reset RTAB-Map database
rm ~/.ros/rtabmap.db

# View RTAB-Map database info
rtabmap-databaseViewer ~/.ros/rtabmap.db

# Export map to image
rtabmap-export --format 0 --output map.png ~/.ros/rtabmap.db
```

---

**Questions or issues?** Check the logs and verify each component (camera, odometry, LiDAR) works independently before running the full stack.
