# Quick Start Guide - Gallery Tour Guide Robot with ZED2i + RTAB-Map

## First Time Setup - Build a Map

**Step 1: Start the base robot**
```bash
# Terminal 1
cd ~/sahabat_ws
source install/setup.bash
ros2 launch shbat_pkg sahabat_launch.py
```

**Step 2: Start ZED + RTAB-Map in MAPPING mode (first time)**
```bash
# Terminal 2
cd ~/sahabat_ws
source install/setup.bash

# MAPPING MODE - builds a new map
ros2 launch shbat_pkg sahabat_rtabmap.launch.py
```

**Step 3: Drive around to build the map**
- Use your joystick to drive the robot
- Move slowly (0.2-0.5 m/s)
- Cover all areas of the gallery systematically
- Drive for 5-10 minutes minimum
- Return to starting point occasionally (helps with loop closures)

**Step 4: Check map is being built**
```bash
# In another terminal
ros2 topic echo /rtabmap/info | grep -E "loop|node"
# You should see node count increasing and occasional loop closures
```

**Step 5: Save the map (automatic)**
- Stop the launch (Ctrl+C in Terminal 2)
- Map auto-saves to `~/.ros/rtabmap.db`
- Backup your map:
```bash
cp ~/.ros/rtabmap.db ~/gallery_map_$(date +%Y%m%d).db
```

---

## Daily Use - Localization Mode (After You Have a Map)

**Once you have a map built, use localization mode:**

```bash
# Terminal 1: Base robot
ros2 launch shbat_pkg sahabat_launch.py

# Terminal 2: ZED + RTAB-Map in LOCALIZATION mode
ros2 launch shbat_pkg sahabat_rtabmap.launch.py localization_mode:=true
```

In localization mode:
- Robot uses existing map
- No new mapping (saves resources)
- Only tracks position within known map
- Perfect for daily tours

---

## Common Commands

### Check if map exists
```bash
ls -lh ~/.ros/rtabmap.db
```

### Start fresh (delete old map)
```bash
ros2 launch shbat_pkg sahabat_rtabmap.launch.py delete_db:=true
```

### Monitor odometry sources
```bash
# ZED visual odometry (primary)
ros2 topic hz /odom

# Wheel odometry (backup)
ros2 topic hz /wheel_odom

# Which is being used by RTAB-Map?
ros2 node info /rtabmap | grep -A5 Subscriptions
```

### View with RViz (optional)
```bash
# Headless (default - saves resources)
ros2 launch shbat_pkg sahabat_rtabmap.launch.py

# With RViz visualization
ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rviz:=true

# With RTAB-Map GUI
ros2 launch shbat_pkg sahabat_rtabmap.launch.py use_rtabmap_viz:=true
```

### Troubleshooting

**RTAB-Map crashes immediately:**
- Check you're not using localization_mode without a map
- Ensure camera is connected: `ros2 topic hz /zed2i/zed_node/rgb/image_rect_color`
- Ensure odometry exists: `ros2 topic hz /odom`

**No visual odometry (/odom not publishing):**
```bash
# Check ZED node is running
ros2 node list | grep zed

# Check ZED positional tracking status
ros2 topic echo /zed2i/zed_node/pose --once
```

**Camera loses tracking:**
- Wheel odometry backup automatically activates
- Check: `ros2 topic echo /wheel_odom`
- Improve lighting or move slower

**Map drift/inaccurate:**
- Drive slower (0.2 m/s)
- Revisit same areas multiple times
- Check for loop closures: `ros2 topic echo /rtabmap/info | grep loop`

---

## Sensor Status Check

Run this to verify all sensors are working:
```bash
#!/bin/bash
echo "=== Sensor Status ==="
echo -n "ZED Camera: "; ros2 topic hz /zed2i/zed_node/rgb/image_rect_color --once 2>&1 | grep -q "Hz" && echo "✓" || echo "✗"
echo -n "ZED Visual Odom: "; ros2 topic hz /odom --once 2>&1 | grep -q "Hz" && echo "✓" || echo "✗"
echo -n "Wheel Odom: "; ros2 topic hz /wheel_odom --once 2>&1 | grep -q "Hz" && echo "✓" || echo "✗"
echo -n "LiDAR: "; ros2 topic hz /scan --once 2>&1 | grep -q "Hz" && echo "✓" || echo "✗"
echo -n "IMU: "; ros2 topic hz /imu --once 2>&1 | grep -q "Hz" && echo "✓" || echo "✗"
echo -n "RTAB-Map: "; ros2 node list | grep -q rtabmap && echo "✓" || echo "✗"
```

---

## Architecture Summary

```
📡 Sensors:
├─ ZED2i Camera → /odom (visual odometry - PRIMARY)
├─ Wheel + IMU → /wheel_odom (backup)
└─ M200 LiDAR → /scan (obstacles)

🗺️  SLAM:
└─ RTAB-Map → uses /odom + depth + /scan
              → publishes /rtabmap/grid_map

🎮 Control:
└─ Joystick → /cmd_vel → base_controller → wheels
```

All sensors active, nothing wasted! 🚀
