# Recording and Offline Mapping Workflow

This guide explains how to record sensor data while moving the robot, then create a map offline without moving the robot again.

## Workflow Overview

1. **Record**: Launch ZED camera, drive robot around, record rosbag
2. **Map Offline**: Play back rosbag and run RTAB-Map to build the map
3. **Localize**: Use the created map for localization later

---

## Step 1: Record the Room

### Launch ZED Camera Only (No RTAB-Map)

```bash
# Source workspace
source install/setup.bash

# Launch ZED camera with robot base
ros2 launch shbat_pkg sahabat_zed_record.launch.py use_rviz:=true
```

### Record the Rosbag

In a **new terminal**, start recording:

```bash
# Source workspace
source install/setup.bash

# Record all ZED topics and TF
ros2 bag record \
  /zed2i/zed_node/rgb/image_rect_color \
  /zed2i/zed_node/rgb/camera_info \
  /zed2i/zed_node/depth/depth_registered \
  /zed2i/zed_node/odom \
  /zed2i/zed_node/imu/data \
  /tf \
  /tf_static \
  -o my_room_recording
```

**Or record everything** (larger file size):
```bash
ros2 bag record -a -o my_room_recording
```

### Drive the Robot

- Use your joystick or teleop to drive the robot around the room
- Move slowly and smoothly for best results
- Cover all areas you want to map
- Return to starting position for better loop closure

### Stop Recording

When done, press `Ctrl+C` in the rosbag terminal.

---

## Step 2: Transfer Bag to Powerful Computer (Recommended!)

If your robot (Jetson) is struggling with RTAB-Map, copy the bag to a more powerful laptop/desktop.

### Option A: Copy via Network (Fast)

**On laptop:**
```bash
# Find your laptop IP
ip addr show

# Start receiving (e.g., laptop IP is 192.168.1.100)
nc -l 9999 | tar xzvf -
```

**On Jetson:**
```bash
# Copy bag folder to laptop
tar czf - my_room_recording | nc 192.168.1.100 9999
```

### Option B: Copy via USB Drive

```bash
# On Jetson: Copy to USB
cp -r my_room_recording /media/usb/

# On laptop: Copy from USB
cp -r /media/usb/my_room_recording ~/
```

## Step 3: Offline Mapping on Powerful Computer

### Install RTAB-Map on Laptop

```bash
# Install ROS 2 (if not already installed)
# Then install RTAB-Map
sudo apt update
sudo apt install ros-humble-rtabmap-ros

# Install tools
sudo apt install ros-humble-rtabmap
```

### Launch RTAB-Map for Offline Processing

```bash
# Create maps directory
mkdir -p ~/maps

# Launch RTAB-Map directly (no need for workspace)
ros2 launch rtabmap_slam rtabmap.launch.py \
  use_sim_time:=true \
  frame_id:=zed2i_camera_link \
  subscribe_rgbd:=true \
  subscribe_scan:=false \
  approx_sync:=true \
  database_path:=~/maps/my_room_map.db \
  rtabmap_viz:=true \
  args:="--delete_db_on_start"
```

**Or use the launch file from your workspace** (if you copy it to laptop):
```bash
ros2 launch shbat_pkg sahabat_rtabmap_from_bag.launch.py \
  delete_db:=true \
  database_path:=~/maps/my_room_map.db
```

**Note:** Radeon graphics will work perfectly fine with RTAB-Map. OpenGL/visualization will work great!

### Play Back the Rosbag

In a **new terminal**:

```bash
# Play back with clock (IMPORTANT: --clock flag!)
ros2 bag play my_room_recording --clock --rate 1.0
```

**On a powerful laptop, you can play at full speed!**

**CRITICAL flags:**
- `--clock`: Publishes simulation time from bag timestamps (required!)
- `--rate 1.0`: Full speed on powerful hardware
- `--rate 0.5`: If laptop still struggles (older hardware)
- `--loop`: Loop the bag (optional, for testing)

**Jetson vs Laptop:**
- **Jetson Orin Nano:** Use `--rate 0.2` to 0.3 (very slow)
- **Modern Laptop:** Use `--rate 1.0` (full speed!)
- **Radeon Graphics:** Works great with RTAB-Map visualization!

### Watch the Mapping

- `rtabmap_viz` will show the map being built in real-time from the recorded data
- You'll see the 3D point cloud, loop closures, and optimization graph
- RTAB-Map will detect loop closures automatically
- The map is saved to the database file you specified

**About the odometry drift:** It's normal for visual odometry to drift during recording. RTAB-Map will correct this drift when it detects loop closures (when you revisit the same area).

---

## Step 3: Verify and Export Map

### Check Map Statistics

```bash
# Get map info
rtabmap-info ~/maps/my_room_map.db

# View map in database viewer (GUI)
rtabmap-databaseViewer ~/maps/my_room_map.db
```

### Export Map for Navigation

```bash
# Export as point cloud
rtabmap-export --poses ~/maps/my_room_map.db

# Export as occupancy grid for Nav2
rtabmap-export --grid ~/maps/my_room_map.db
```

---

## Tips for Best Results

### Recording:
- **Move slowly** (0.3 m/s max) for better visual odometry
- **Good lighting** is essential for ZED camera
- **Overlap**: revisit areas to create loop closures
- **Smooth motion**: avoid jerky movements
- **Return to start**: helps close the loop

### Offline Mapping:
- **On Jetson:** Play bag at **0.2x to 0.3x speed** (very slow!)
- **On Laptop:** Play bag at **0.5x to 1.0x speed** (much faster!)
- Use **--clock** flag for proper timing
- If you see lateral drift, slow down playback rate
- Monitor CPU usage with `htop` - if maxed out, slow down
- Radeon/Intel/NVIDIA graphics all work fine with RTAB-Map

### Bag Size Estimation:
- Full resolution RGB+Depth: ~50-100 MB/minute
- Compressed: ~20-40 MB/minute
- Default storage is sqlite3 (works on all ROS 2 installations)

---

## Example Complete Workflow

```bash
# Terminal 1: Recording session
source install/setup.bash
ros2 launch shbat_pkg sahabat_zed_record.launch.py

# Terminal 2: Record bag
source install/setup.bash
ros2 bag record -a -o living_room
# Drive robot around, then Ctrl+C

# --- Later, robot stationary ---

# Create maps directory
mkdir -p ~/maps

# Terminal 1: RTAB-Map offline
source install/setup.bash
ros2 launch shbat_pkg sahabat_rtabmap_from_bag.launch.py \
  delete_db:=true \
  database_path:=~/maps/living_room.db

# Terminal 2: Play bag at SLOW rate
source install/setup.bash
ros2 bag play living_room --clock --rate 0.5

# Wait for completion, then verify
rtabmap-info ~/maps/living_room.db
```

---

## Troubleshooting

**"No transform between odom and camera_link"**
- Make sure you recorded `/tf` and `/tf_static` topics
- Use `--clock` flag when playing bag

**RTAB-Map not processing frames**
- Check `use_sim_time: True` is set (already done in launch file)
- Verify bag is playing with `ros2 topic hz /zed2i/zed_node/rgb/image_rect_color`

**Memory issues**
- Reduce bag playback rate: `--rate 0.5`
- Set lower `RTABMAP_MAX_MEMORY` (already set to 4096 MB)

**Map quality issues**
- Re-record with slower robot motion
- Ensure good lighting conditions
- Cover areas with more overlap

