# Navigation Testing Guide
## Test Multi-Sensor Obstacle Detection Without Building a Map

### Overview

This guide shows how to test your gallery robot's **multi-sensor obstacle detection** and navigation **without building a map first**. Perfect for quick testing and verification!

---

## 🎯 What You'll See in RViz

Your updated RViz configuration now displays:

| Display | What It Shows | Color | Why It's Important |
|---------|---------------|-------|-------------------|
| **LaserScan** | LiDAR obstacles (ground level) | White points | Ground-level obstacles (0-30cm) |
| **PointCloud2** | ZED camera depth points | RGB colors | Table-height obstacles (30-200cm) |
| **Local Costmap** | Nav2 obstacle avoidance zone | Red = obstacle, Blue = free | What the robot uses for navigation |
| **Global Costmap** | Larger planning area | Red = obstacle, Blue = free | Long-range path planning |
| **Local Plan** | Current path being followed | Green line | Real-time trajectory |
| **Global Plan** | Full path to goal | Red line | Overall route plan |
| **RobotModel** | 3D robot visualization | Gray/colored | Robot position and orientation |
| **TF** | Coordinate frames | Colored axes | Shows sensor mounting positions |

---

## ✅ Step-by-Step Testing Procedure

### **Step 1: Launch Robot with Sensor Fusion**

```bash
# Terminal 1: Start robot + ZED + EKF (no SLAM)
source /home/sahabat/sahabat_ws/install/setup.bash
ros2 launch shbat_pkg sahabat_sensor_fusion_only.launch.py use_rviz:=true
```

**Wait for:**
- ✅ ZED camera initialization (green "Ready" message)
- ✅ LiDAR spinning and publishing `/scan`
- ✅ EKF publishing `/odometry/filtered`
- ✅ RViz opens showing the robot

---

### **Step 2: Launch Nav2 Navigation**

```bash
# Terminal 2: Start Nav2 navigation stack
source /home/sahabat/sahabat_ws/install/setup.bash
ros2 launch shbat_pkg sahabat_nav.launch.py
```

**Wait for:**
- ✅ "controller_server" is active
- ✅ "planner_server" is active
- ✅ Costmaps appear in RViz

---

### **Step 3: Verify Displays in RViz**

**Check these items:**

1. **Fixed Frame** (top of RViz): Should be `odom`
   - If it's `map`, change it to `odom` (we're not using a map)

2. **LaserScan** (white points):
   - ✅ Should show walls, furniture at ground level
   - ✅ Updates in real-time as robot moves

3. **PointCloud2** (colored 3D points):
   - ✅ Should show walls, tables, displays above LiDAR height
   - ✅ RGB colors match what the camera sees
   - ✅ Updates at ~10 Hz

4. **Local Costmap** (red/blue grid around robot):
   - ✅ **Red areas** = obstacles detected
   - ✅ **Gradient** = inflation zone (safety margin)
   - ✅ **Blue** = free space
   - ✅ Should show BOTH LiDAR AND ZED obstacles!

5. **TF Frames** (colored axes):
   - ✅ `odom` → `base_link` (50 Hz from EKF)
   - ✅ `base_link` → `zed2i_camera_link` (static)
   - ✅ `base_link` → `lidar_link` (static)

---

### **Step 4: Test Multi-Height Obstacle Detection**

**Setup obstacles at different heights:**

1. **Ground level (0-30cm)**: Place a cardboard box
   - Should appear in **LaserScan** (white points)
   - Should appear in **Local Costmap** (red)

2. **Table height (50-100cm)**: Place a chair or display stand
   - Should appear in **PointCloud2** (colored points)
   - Should appear in **Local Costmap** (red)
   - **Will NOT appear in LaserScan** (above LiDAR!)

3. **Hanging (100-200cm)**: Hold something above the robot
   - Should appear in **PointCloud2**
   - Should appear in **Local Costmap**
   - Proves ZED is protecting against overhead obstacles

---

### **Step 5: Send a Navigation Goal**

1. **Click "2D Goal Pose" button** (top toolbar in RViz)

2. **Click and drag** on the floor (blue area in costmap)
   - Click where you want the robot to go
   - Drag to set the orientation (direction robot will face)

3. **Robot should:**
   - ✅ Plan a path (red Global Plan line appears)
   - ✅ Start moving (green Local Plan updates)
   - ✅ Avoid obstacles (both LiDAR and ZED detected)
   - ✅ Reach the goal pose

---

### **Step 6: Verify Sensor Fusion is Working**

**Monitor odometry:**

```bash
# Terminal 3: Check EKF is fusing sensors
ros2 topic hz /odometry/filtered
# Should show: ~50 Hz

# Check wheel odometry input
ros2 topic hz /wheel_odom
# Should show: ~50 Hz

# Check ZED visual odometry input
ros2 topic hz /zed2i/zed_node/odom
# Should show: ~30 Hz

# Verify both are being used
ros2 topic echo /diagnostics | grep -A 5 ekf_filter_node
```

---

## 🧪 Test Scenarios

### **Test 1: LiDAR-Only Obstacle (Ground Level)**

**Setup:**
- Place a low box (< 30cm height) in front of robot
- Box is below camera field of view

**Expected:**
- ✅ LaserScan shows white points
- ✅ Local costmap shows red obstacle
- ✅ Robot avoids it

---

### **Test 2: ZED-Only Obstacle (Table Height)**

**Setup:**
- Place a table leg or display stand (50-100cm height)
- Above LiDAR plane

**Expected:**
- ✅ PointCloud2 shows colored 3D points
- ✅ LaserScan shows NOTHING (obstacle too high)
- ✅ Local costmap shows red obstacle (from ZED!)
- ✅ Robot avoids it

**This proves ZED point cloud integration works!**

---

### **Test 3: Multi-Height Obstacles**

**Setup:**
- Place both ground-level and table-height obstacles
- Create a narrow passage between them

**Expected:**
- ✅ Both appear in costmap (red)
- ✅ Robot navigates between them
- ✅ Uses combined sensor data for path planning

---

### **Test 4: Sensor Fallback (Cover ZED)**

**Setup:**
- Send a navigation goal
- While robot is moving, cover the ZED camera lenses

**Expected:**
- ✅ Robot continues moving (using wheel odometry)
- ✅ May become less accurate over time (no visual corrections)
- ✅ LiDAR still detects ground obstacles
- ✅ No crash - EKF falls back to wheels

**This proves sensor fusion redundancy!**

---

## 🎨 RViz Display Tips

### **Adjust PointCloud2 Visibility**

If the point cloud is too dense or distracting:

1. Click on **PointCloud2** in the Displays panel
2. Adjust **Size (m)**: `0.01` (smaller) or `0.03` (bigger)
3. Change **Style**: 
   - **Points** (default, dots)
   - **Boxes** (more visible)
   - **Spheres** (best visibility)
4. Adjust **Color Transformer**:
   - **RGB8** (camera colors - realistic)
   - **AxisColor** (height-based colors - shows elevation)
   - **Intensity** (brightness-based)

### **Filter PointCloud by Height**

To see only obstacles in navigation range:

1. Click **PointCloud2** → **Filter**
2. Add filter: **FieldValueFilter**
3. Field: `z`
4. Min: `0.3` (ignore ground)
5. Max: `2.0` (ignore ceiling)

### **Costmap Transparency**

If costmaps block other displays:

1. Click **Local Costmap** or **Global Costmap**
2. Adjust **Alpha**: `0.5` (50% transparent)
3. Toggle **Draw Behind**: `true` (draws under other stuff)

---

## 📊 Performance Monitoring

### **Check CPU Usage**

```bash
# Monitor overall system
htop

# Check specific nodes
ros2 run ros2_diagnostic diagnostic_aggregator
```

**Expected on Jetson Orin Nano:**
- ZED wrapper: ~15-20% CPU
- Nav2 nodes: ~10-15% CPU
- EKF: <5% CPU
- Total: ~40-50% CPU

### **Check Memory Usage**

```bash
free -h

# Expected: 3-4 GB used out of 8 GB
```

### **Check Topic Rates**

```bash
# All should be publishing
ros2 topic hz /scan                                    # ~10 Hz (LiDAR)
ros2 topic hz /zed2i/zed_node/point_cloud/cloud_registered  # ~10 Hz (ZED)
ros2 topic hz /odometry/filtered                       # ~50 Hz (EKF)
ros2 topic hz /local_costmap/costmap                   # ~5 Hz (Nav2)
```

---

## ⚠️ Troubleshooting

### **Issue: No obstacles in Local Costmap**

**Check:**
```bash
# Verify point cloud is publishing
ros2 topic echo /zed2i/zed_node/point_cloud/cloud_registered --no-arr

# Verify costmap is configured for point cloud
ros2 param get /local_costmap/voxel_layer observation_sources
# Should show: scan point_cloud
```

**Fix:** Already configured in your `nav2_params.yaml`

---

### **Issue: Robot ignores ZED obstacles**

**Check costmap parameters:**
```bash
ros2 param get /local_costmap/voxel_layer point_cloud.topic
# Should show: /zed2i/zed_node/point_cloud/cloud_registered

ros2 param get /local_costmap/voxel_layer point_cloud.min_obstacle_height
# Should show: 0.3

ros2 param get /local_costmap/voxel_layer point_cloud.max_obstacle_height
# Should show: 2.0
```

---

### **Issue: PointCloud2 display shows "No messages received"**

**Check topic:**
```bash
ros2 topic list | grep point_cloud

# Should see: /zed2i/zed_node/point_cloud/cloud_registered

# Check if publishing
ros2 topic hz /zed2i/zed_node/point_cloud/cloud_registered
```

**If not publishing:** ZED camera may not be initialized properly. Check ZED node:
```bash
ros2 node info /zed2i/zed_node
```

---

### **Issue: "Transform timeout" errors in Nav2**

**Check TF tree:**
```bash
ros2 run tf2_ros tf2_echo odom base_link
# Should show current transform, not error

ros2 run tf2_tools view_frames
evince frames_*.pdf
# Verify: odom → base_link exists (from EKF)
```

---

## 📈 Expected Results

### **Successful Test Checklist:**

- ✅ Robot navigates to goals in RViz
- ✅ Avoids ground-level obstacles (LiDAR)
- ✅ Avoids table-height obstacles (ZED point cloud)
- ✅ Both obstacle types show in Local Costmap (red)
- ✅ Path planning works around all obstacles
- ✅ No TF errors in console
- ✅ Odometry drift is minimal (< 10cm over 5m)
- ✅ CPU usage stays under 60%

---

## 🚀 Next Steps

### **After successful testing:**

1. **Build a map with RTAB-Map:**
   ```bash
   ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rviz:=true
   # Drive around for 5-10 minutes
   ```

2. **Use the map for localization:**
   ```bash
   ros2 launch shbat_pkg sahabat_rtabmap.launch.py localization_mode:=true
   ros2 launch shbat_pkg sahabat_nav.launch.py
   ```

3. **Deploy in gallery:**
   - Use the saved map for tours
   - Multi-sensor fusion provides reliability
   - ZED detects displays/artwork, LiDAR detects visitors

---

## 📝 Summary

**Your RViz now shows:**
- ✅ LaserScan (LiDAR - white points)
- ✅ PointCloud2 (ZED - colored 3D points)
- ✅ Local Costmap (Nav2 obstacle avoidance - red/blue)
- ✅ Global Costmap (Nav2 global planning - red/blue)
- ✅ Local Plan (current trajectory - green)
- ✅ Global Plan (full path - red)
- ✅ RobotModel (3D robot visualization)
- ✅ TF frames (sensor positions)

**You can now:**
- Test navigation without building a map first
- Verify multi-height obstacle detection works
- See exactly what Nav2 uses for planning
- Confirm sensor fusion is active

**Happy testing!** 🎉
