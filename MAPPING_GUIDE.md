# RTAB-Map Mapping Guide

## Quick Start - Building Your First Map

### Step 1: Launch the System
```bash
# First time OR to start a fresh map:
ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rtabmap_viz:=true delete_db:=true

# Continue building existing map:
ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rtabmap_viz:=true
```

### Step 2: Wait for Initialization (~10 seconds)
- ZED camera starts and initializes
- RTAB-Map loads and waits for data
- Look for: `[rtabmap]: rtabmap started.` in terminal

### Step 3: Check if System is Ready
Open a NEW terminal and run:
```bash
cd ~/sahabat_ws
./check_rtabmap.sh
```

You should see:
- ✅ ZED odometry: ~15-30 Hz
- ✅ RGB/Depth images: ~15 Hz  
- ✅ TF tree working (odom → base_link)

### Step 4: Move the Robot to Start Mapping

**CRITICAL**: RTAB-Map only adds nodes when robot moves:
- **Minimum movement**: 10cm forward/backward OR 6° rotation
- **Recommended speed**: 0.2-0.3 m/s (slow walking pace)
- **Movement pattern**: Smooth, continuous motion

**Using joystick:**
```bash
# Move forward slowly
# Look around (rotate) to see different angles
# Return to starting point for loop closure
```

**Using keyboard:**
```bash
# In another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

### Step 5: Monitor Mapping Progress

**In rtabmapviz window:**
- Top-left shows node count (should increase as you move)
- 3D view shows accumulated point cloud
- Graph view shows robot trajectory
- Green = good tracking, Red = lost

**In terminal:**
```bash
# Check node count (should increase)
ros2 topic echo /rtabmap/info --once | grep "local_map_size"

# Check if map is being published
ros2 topic hz /rtabmap/mapData
# Should show updates when you move >10cm
```

## Understanding the Map Building Process

### What RTAB-Map Does:
1. **Captures keyframes** when robot moves >10cm or rotates >6°
2. **Extracts visual features** from RGB-D images
3. **Builds 3D point cloud** from depth data
4. **Detects loop closures** when returning to known areas
5. **Optimizes the graph** to reduce drift

### Why Map Might Not Build:
- ❌ **Robot not moving** - Check `ros2 topic hz /zed2i/zed_node/odom`
- ❌ **Poor lighting** - ZED needs decent light for features
- ❌ **Moving too fast** - Slow down to <0.5 m/s
- ❌ **Moving too slow** - Need >10cm displacement
- ❌ **Blank walls** - Not enough visual features (move to textured areas)
- ❌ **TF issues** - Check `ros2 run tf2_tools view_frames`

## Mapping Best Practices

### Good Mapping Technique:
1. **Start in feature-rich area** (not blank wall)
2. **Move slowly and smoothly** (0.2-0.3 m/s)
3. **Look around** (rotate) to capture different angles
4. **Close loops** - return to starting point to reduce drift
5. **Avoid rapid movements** - causes motion blur

### Coverage Pattern:
```
1. Map main hallway/room first
2. Make small loops (~5m diameter)
3. Revisit starting area every 2-3 minutes
4. Gradually expand coverage
5. End where you started
```

## Map Management

### Check Map Size:
```bash
ls -lh ~/.ros/rtabmap.db
# Shows database file size
```

### Delete Old Map:
```bash
# Option 1: Launch with delete flag
ros2 launch shbat_pkg sahabat_rtabmap.launch.py delete_db:=true

# Option 2: Manual deletion
rm ~/.ros/rtabmap.db
```

### Export Map for Navigation:
```bash
# The map is automatically published to /map topic
# Nav2 will use it automatically

# To save map as image:
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Troubleshooting

### Problem: "No map data" or node count stays at 0
**Solution:**
1. Check odometry: `ros2 topic hz /zed2i/zed_node/odom`
2. Move robot at least 10cm: `ros2 topic echo /zed2i/zed_node/odom`
3. Check images: `ros2 topic hz /zed2i/zed_node/rgb/image_rect_color`
4. Verify thresholds in rtabmap_params.yaml:
   ```yaml
   RGBD/LinearUpdate: 0.1   # 10cm minimum
   RGBD/AngularUpdate: 0.1  # 6° minimum
   ```

### Problem: Map only shows front area
**Cause:** Not moving enough or threshold not met
**Solution:** 
- Drive forward at least 15cm
- Or rotate at least 10°
- Check `ros2 topic echo /rtabmap/info --once` for node updates

### Problem: Map is jumpy/jittery
**Cause:** Moving too fast or poor tracking
**Solution:**
- Slow down to 0.2 m/s
- Add more visual features (posters, markers)
- Check lighting conditions

### Problem: "Lost tracking" message
**Cause:** No visual features or too fast movement
**Solution:**
- Move to area with more texture
- Slow down
- Add visual markers if environment is bland

## Advanced Tips

### Optimize for Jetson Performance:
Already configured in rtabmap_params.yaml:
```yaml
Mem/ImagePreDecimation: 2  # Reduce image size
depth_mode: 'PERFORMANCE'  # Faster but less accurate
```

### Loop Closure Detection:
- Happens automatically when you return to known area
- Shows green line connecting nodes in graph
- Significantly reduces accumulated drift
- Aim for loops every 2-3 minutes

### Save Area Memory:
```bash
# In rtabmapviz: File > Save memory
# Or via service:
ros2 service call /rtabmap/save_area_memory std_srvs/srv/Empty
```

## Quick Commands Reference

```bash
# Run diagnostics
./check_rtabmap.sh

# Check node count
watch -n 1 'ros2 topic echo /rtabmap/info --once | grep local_map_size'

# Monitor odometry
ros2 topic echo /zed2i/zed_node/odom

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check topic rates
ros2 topic hz /rtabmap/mapData
ros2 topic hz /zed2i/zed_node/odom

# Emergency: Reset RTAB-Map
ros2 service call /rtabmap/reset std_srvs/srv/Empty
```

## Success Indicators

✅ **Good mapping session:**
- Node count increasing steadily (1 node per 10cm)
- Loop closures detected (green lines in graph)
- Point cloud accumulating smoothly
- No "tracking lost" warnings

❌ **Problems:**
- Node count stuck at 0 or 1
- "Waiting for transform" errors
- Odometry rate = 0 Hz
- CPU usage > 90% sustained

---

**Remember**: Mapping quality depends on:
1. **Movement** - Smooth, continuous, not too fast
2. **Features** - Textured environment, good lighting  
3. **Coverage** - Small loops, revisit start area
4. **Hardware** - Keep Jetson cool, good USB connection
