# Quick Navigation Test Guide (Without RTAB-Map)

## Test Autonomous Navigation Now!

You can test navigation **immediately** using your existing setup with slam_toolbox (simpler than RTAB-Map).

---

## Method 1: Navigation with 2D SLAM (slam_toolbox) - RECOMMENDED

This is the **easiest** way to test navigation right now:

### Step 1: Launch Base Robot + ZED
```bash
# Terminal 1
cd ~/sahabat_ws
source install/setup.bash
ros2 launch shbat_pkg sahabat_launch.py
```

This starts:
- ✅ Wheel encoders + IMU → `/wheel_odom` (backup)
- ✅ ZED visual odometry → `/odom` (primary)
- ✅ M200 LiDAR → `/scan`
- ✅ Base controller, joystick

### Step 2: Launch SLAM (2D mapping)
```bash
# Terminal 2
source install/setup.bash
ros2 launch shbat_pkg sahabat_mapping.launch.py
```

This starts:
- ✅ slam_toolbox → builds 2D map in real-time
- ✅ RViz → visualize map building

### Step 3: Build a Small Map First
- Use joystick to drive around slowly
- Watch RViz - you'll see the map being built
- Drive for 2-3 minutes to map a small area
- Return to starting point (helps SLAM)

### Step 4: Launch Navigation
```bash
# Terminal 3
source install/setup.bash
ros2 launch shbat_pkg sahabat_nav.launch.py
```

This starts:
- ✅ Nav2 navigation stack
- ✅ Path planner
- ✅ Controller
- ✅ Costmaps (using LiDAR + ZED depth)

### Step 5: Send a Navigation Goal
In RViz:
1. Click "2D Goal Pose" button (top toolbar)
2. Click on the map where you want the robot to go
3. Drag to set orientation
4. Robot will navigate autonomously!

**What you'll see:**
- Green path (planned route)
- Robot follows path autonomously
- Avoids obstacles using LiDAR
- Uses ZED visual odometry for accurate positioning

---

## Method 2: Test Odometry Only (No Map) - FOR TESTING ONLY

**Warning:** This is ONLY for testing odometry quality. No obstacle avoidance!

```bash
# Terminal 1: Base robot
source install/setup.bash
ros2 launch shbat_pkg sahabat_launch.py

# Terminal 2: Navigation (no SLAM)
source install/setup.bash
ros2 launch shbat_pkg test_nav_odom_only.launch.py
```

Send goals in RViz - robot navigates using only odometry (will drift over time).

---

## Comparison Table

| Method | Map Building | Localization | Obstacle Avoidance | Best For |
|--------|--------------|--------------|-------------------|----------|
| **slam_toolbox** (Method 1) | ✅ Real-time | ✅ Yes | ✅ LiDAR + ZED | **Production use** |
| **RTAB-Map** (from other guide) | ✅ Visual 3D | ✅ Yes | ✅ LiDAR + ZED | **Complex environments** |
| **Odom only** (Method 2) | ❌ No | ❌ Drifts | ⚠️ Limited | **Testing only** |

---

## Which Should You Use?

### For Your Gallery Tour Guide:

**Start with Method 1 (slam_toolbox)** because:
- ✅ Simple and fast
- ✅ Works immediately
- ✅ Good 2D maps for navigation
- ✅ Uses all your sensors (ZED odom + LiDAR)
- ✅ Already configured in your workspace

**Later upgrade to RTAB-Map when:**
- You want 3D mapping
- You need better loop closure (long-term drift correction)
- Gallery is very large
- You want to save/reload maps for multi-session use

---

## Quick Test Workflow

```bash
# 1. Launch everything
ros2 launch shbat_pkg sahabat_launch.py      # Terminal 1
ros2 launch shbat_pkg sahabat_mapping.launch.py  # Terminal 2
ros2 launch shbat_pkg sahabat_nav.launch.py      # Terminal 3

# 2. Drive around for 2-3 minutes with joystick

# 3. Send goal in RViz → Watch it navigate!
```

---

## Monitor During Testing

```bash
# Check odometry sources
ros2 topic hz /odom          # ZED visual (should be ~15-30 Hz)
ros2 topic hz /wheel_odom    # Wheel backup (should be ~50 Hz)

# Check SLAM
ros2 topic echo /map | head  # Should see occupancy grid

# Check navigation status
ros2 topic echo /plan        # Should show planned path
```

---

## Troubleshooting

**Robot doesn't move when goal is set:**
- Check joystick is not pressed (override)
- Verify Nav2 nodes running: `ros2 node list | grep nav`
- Check path is published: `ros2 topic hz /plan`

**Navigation path looks weird:**
- Map might be incomplete - drive around more
- Check costmaps are updating: `ros2 topic hz /local_costmap/costmap`

**Robot loses position:**
- ZED might have lost tracking - check lighting
- Falls back to wheel odometry automatically
- Check: `ros2 topic echo /odom` (should update continuously)

---

**Ready to test? Use Method 1 (slam_toolbox) - it's already set up and works great!** 🚀
