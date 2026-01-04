# Pure ZED Localization Setup

## Overview
Your robot now uses **pure ZED visual localization** where the ZED2i camera is the root of the TF tree.

## TF Tree Structure
```
map (from RTAB-Map)
 └─ odom (from ZED at ~30 Hz)
     └─ zed2i_camera_link (from ZED TF publisher)
         └─ base_link (defined in URDF: -0.05m, 0.0m, -1.0m, pitch=+8°)
             ├─ body_link
             │   ├─ lidar_link (LiDAR mounted on top)
             │   ├─ front_caster_wheel_link
             │   └─ rear_caster_wheel_link
             ├─ left_wheel_link
             └─ right_wheel_link
```

**Key Point**: Camera is the reference frame. The robot "hangs" below it at (-5cm, 0cm, -67cm).

## Files Modified

### 1. `sahabat_robot_with_zed.urdf.xacro` (NEW)
- Includes full robot description + ZED camera macro
- Uses `use_zed_localization:=true` to invert transform
- Camera link is parent, base_link is child

### 2. `sahabat_rtabmap.launch.py`
**Changes:**
- ✅ ZED wrapper: `publish_tf: true` (ZED publishes odom→camera)
- ✅ ZED wrapper: `publish_urdf: true` with custom URDF path
- ✅ ZED wrapper: `use_zed_localization: true`
- ❌ **REMOVED** `ekf_node` - no sensor fusion needed
- ❌ **REMOVED** `zed_base_link_tf` - URDF defines this
- ✅ RTAB-Map uses `/zed2i/zed_node/odom` directly (not filtered)

### 3. `setup.py`
- Added `sahabat_robot_with_zed.urdf.xacro` to installation

## How to Use

### Launch with RTAB-Map SLAM:
```bash
source ~/sahabat_ws/install/setup.bash
ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rviz:=true
```

### What's Running:
1. **Base Robot** (sahabat_launch.py):
   - Motor controller
   - IMU (still publishes `/imu/data`)
   - LiDAR (publishes `/scan`)
   - kalman_filter.py (still runs but NOT used for navigation)

2. **ZED Wrapper**:
   - Publishes TF: `odom` → `zed2i_camera_link`
   - Publishes odometry: `/zed2i/zed_node/odom`
   - Publishes RGB: `/zed2i/zed_node/rgb/image_rect_color`
   - Publishes depth: `/zed2i/zed_node/depth/depth_registered`
   - Publishes point cloud: `/zed2i/zed_node/point_cloud/cloud_registered`
   - Publishes URDF with robot attached

3. **RTAB-Map**:
   - Uses ZED odometry
   - Creates 3D visual map
   - Publishes TF: `map` → `odom`
   - Publishes `/map` topic for navigation

4. **RViz** (optional):
   - Visualize TF tree
   - View camera feed
   - View costmaps

## Advantages of Pure ZED Localization

✅ **Simpler TF tree** - One source of truth for odometry
✅ **No sensor fusion complexity** - ZED handles everything
✅ **Better for visual-rich environments** - Gallery with artwork/features
✅ **Drift correction** - ZED has built-in loop closure
✅ **6-DOF odometry** - Full pose estimation (x, y, z, roll, pitch, yaw)

## Disadvantages

❌ **Visual dependency** - Needs good lighting and features
❌ **No wheel encoder backup** - If ZED fails, robot is blind
❌ **More CPU usage** - Visual odometry is compute-intensive on Jetson

## Verification Commands

### Check TF tree:
```bash
ros2 run tf2_tools view_frames
# Should show: map → odom → zed2i_camera_link → base_link
```

### Check ZED odometry:
```bash
ros2 topic hz /zed2i/zed_node/odom
# Should show ~30 Hz
```

### Check TF publishing rate:
```bash
ros2 topic hz /tf
# Should see odom→zed2i_camera_link at ~30 Hz
```

### Verify URDF loaded:
```bash
ros2 param get /robot_state_publisher robot_description | grep "sahabat_robot"
# Should see your robot description
```

## Camera Mounting (Physical Reality)
- **Height**: 67cm above base_link center
- **Forward**: 5cm in front of base_link
- **Pitch**: 8° downward (looking at floor ~2-3m ahead)
- **Mounted on**: Top of robot body

## Troubleshooting

### If ZED isn't publishing odometry:
```bash
# Check ZED status
ros2 topic echo /zed2i/zed_node/status/heartbeat --once

# Check if camera detected
ros2 node info /zed2i/zed_node
```

### If TF tree is broken:
```bash
# Check for transform errors
ros2 run tf2_ros tf2_echo map base_link

# View all transforms
ros2 topic echo /tf --once
```

### If RTAB-Map crashes:
- Check memory usage: `htop`
- Reduce image quality in ZED settings
- Lower RTAB-Map detection rate in config

## Navigation Integration

For Nav2, use:
```bash
ros2 launch shbat_pkg sahabat_nav.launch.py
```

Nav2 will use:
- **Global frame**: `map` (from RTAB-Map)
- **Odom frame**: `odom` (from ZED)
- **Base frame**: `base_link`

## When to Use This vs EKF Fusion?

**Use Pure ZED** when:
- Gallery has rich visual features (artwork, decorations)
- Good lighting conditions
- Want simplest setup
- Trust ZED's visual odometry

**Use EKF Fusion** when:
- Need backup odometry (wheel encoders)
- Poor lighting or blank walls
- Want redundancy for reliability
- Robot moves in featureless corridors

---

**Current Status**: ✅ Configured for Pure ZED Localization
**Last Updated**: October 26, 2025
