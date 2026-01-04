# Real-Time RTAB-Map Streaming: Jetson → Laptop

This guide shows how to run RTAB-Map on a powerful laptop while the Jetson streams ZED camera data over the network in real-time.

## Architecture

```
Jetson (Robot):                     Laptop (Powerful):
- ZED Camera                        - RTAB-Map SLAM
- Robot Base                        - RTAB-Map Viz
- Publishes sensor data  ------>    - Mapping & Visualization
```

## Benefits
- ✅ Jetson only handles sensors (low compute)
- ✅ Laptop does heavy SLAM processing
- ✅ Real-time mapping, no bag recording needed
- ✅ Can visualize mapping as robot moves

---

## Setup

### Prerequisites
- Jetson and Laptop on **same network**
- Both have ROS 2 Humble installed
- Good WiFi connection (5GHz recommended)

---

## Step 1: Network Configuration

### On Jetson (find IP address):
```bash
ip addr show
# Note Jetson IP, e.g., 192.168.1.50
```

### On Laptop (find IP address):
```bash
ip addr show
# Note Laptop IP, e.g., 192.168.1.100
```

### Configure ROS 2 Domain (Both machines):
```bash
# Add to ~/.bashrc on BOTH Jetson and Laptop
export ROS_DOMAIN_ID=42  # Use same domain ID
export ROS_LOCALHOST_ONLY=0  # Allow network communication

# Apply changes
source ~/.bashrc
```

### Test network discovery:
```bash
# On Jetson: publish test
ros2 topic pub /test std_msgs/String "data: 'hello from jetson'"

# On Laptop: check if visible
ros2 topic list
ros2 topic echo /test
```

If you see `/test` topic on laptop, network is configured correctly!

---

## Step 2: Launch on Jetson (Robot)

### Launch ZED Camera Only

```bash
# Source workspace
cd ~/sahabat_ws
source install/setup.bash

# Launch ZED recording mode (publishes all topics)
ros2 launch shbat_pkg sahabat_zed_record.launch.py use_rviz:=false
```

**This publishes:**
- `/zed2i/zed_node/rgb/image_rect_color`
- `/zed2i/zed_node/depth/depth_registered`
- `/zed2i/zed_node/rgb/camera_info`
- `/zed2i/zed_node/odom`
- `/tf` and `/tf_static`

---

## Step 3: Install RTAB-Map on Laptop

```bash
# Install ROS 2 Humble (if not already installed)
# See: https://docs.ros.org/en/humble/Installation.html

# Install RTAB-Map
sudo apt update
sudo apt install ros-humble-rtabmap-ros ros-humble-rtabmap

# Source ROS 2
source /opt/ros/humble/setup.bash
```

---

## Step 4: Launch RTAB-Map on Laptop

### Method A: Using Command Line (Simple)

**Terminal 1: RGBD Sync**
```bash
source /opt/ros/humble/setup.bash

ros2 run rtabmap_sync rgbd_sync \
  --ros-args \
  -p frame_id:=zed2i_camera_link \
  -p approx_sync:=true \
  -p approx_sync_max_interval:=0.05 \
  -r rgb/image:=/zed2i/zed_node/rgb/image_rect_color \
  -r rgb/camera_info:=/zed2i/zed_node/rgb/camera_info \
  -r depth/image:=/zed2i/zed_node/depth/depth_registered
```

**Terminal 2: RTAB-Map**
```bash
source /opt/ros/humble/setup.bash

# Create maps directory
mkdir -p ~/maps

ros2 launch rtabmap_slam rtabmap.launch.py \
  frame_id:=zed2i_camera_link \
  subscribe_rgbd:=true \
  subscribe_scan:=false \
  approx_sync:=true \
  rtabmap_viz:=true \
  database_path:=~/maps/live_stream_map.db \
  args:="--delete_db_on_start" \
  odom_topic:=/zed2i/zed_node/odom \
  Reg/Force3DoF:=true \
  Vis/MaxFeatures:=400 \
  RGBD/AngularUpdate:=0.1 \
  RGBD/LinearUpdate:=0.1
```

### Method B: Copy Launch File to Laptop

**Copy the optimized launch file from Jetson:**
```bash
# On Jetson
scp ~/sahabat_ws/src/shbat_pkg/launch/sahabat_rtabmap_from_bag.launch.py \
    user@laptop-ip:~/
```

**Modify for live streaming (on laptop):**
- Remove `use_sim_time: True` (use real-time)
- Keep everything else the same

**Launch on laptop:**
```bash
# Edit launch file: change use_sim_time from True to False
sed -i 's/use_sim_time.*True/use_sim_time\": False/g' sahabat_rtabmap_from_bag.launch.py

# Run it (no bag playback needed!)
ros2 launch sahabat_rtabmap_from_bag.launch.py \
  delete_db:=true \
  database_path:=~/maps/live_stream_map.db
```

---

## Step 5: Start Mapping!

1. **On Laptop:** You should see `rtabmap_viz` window showing live camera feed
2. **On Jetson:** Drive the robot around using joystick
3. **On Laptop:** Watch the map build in real-time!

---

## Bandwidth Optimization (If Network is Slow)

If you have WiFi bandwidth issues, compress the image stream:

### On Jetson: Republish with compression

**Terminal (in addition to ZED launch):**
```bash
# Install image transport plugins
sudo apt install ros-humble-compressed-image-transport

# Republish compressed
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/zed2i/zed_node/rgb/image_rect_color \
  -r out/compressed:=/zed2i/zed_node/rgb/image_rect_color/compressed

ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/zed2i/zed_node/depth/depth_registered \
  -r out/compressed:=/zed2i/zed_node/depth/depth_registered/compressed
```

### On Laptop: Subscribe to compressed topics

Change the remappings to use `/compressed` topics:
```bash
-r rgb/image:=/zed2i/zed_node/rgb/image_rect_color/compressed \
-r depth/image:=/zed2i/zed_node/depth/depth_registered/compressed
```

---

## Troubleshooting

### Topics not visible on laptop
```bash
# Check ROS_DOMAIN_ID matches on both machines
echo $ROS_DOMAIN_ID

# Check firewall (disable temporarily to test)
sudo ufw disable

# Check network discovery
ros2 daemon stop
ros2 daemon start
```

### High latency / lag
- Use 5GHz WiFi instead of 2.4GHz
- Use compressed image transport (see above)
- Reduce ZED camera resolution in `zed_pos_tracking_override.yaml`

### "No transform between frames" error
- Make sure `/tf` and `/tf_static` are publishing from Jetson
- Check: `ros2 topic echo /tf_static`

### Laptop can't keep up
- Reduce feature count: `Vis/MaxFeatures:=200`
- Increase keyframe thresholds: `RGBD/LinearUpdate:=0.2`

---

## Comparison: Streaming vs Bag

| Method | Pros | Cons |
|--------|------|------|
| **Live Streaming** | Real-time feedback, no storage needed, see results immediately | Requires good network, can't replay |
| **Bag Recording** | Can replay many times, portable, works offline | Requires storage, delayed results |

**Recommendation:** Use streaming for initial testing, record bags for final maps!

---

## Performance Tips

### On Jetson:
- Close unnecessary applications
- Use `use_rviz:=false` (no visualization needed on robot)
- Consider reducing ZED frame rate in config

### On Laptop:
- Close browser/heavy apps during mapping
- Use wired ethernet if possible
- Monitor bandwidth: `iftop` or `nethogs`

### Network:
- Position laptop close to router
- Use 5GHz WiFi
- Consider direct ethernet connection between Jetson and laptop

---

## Saving the Map

Once mapping is complete:

```bash
# On laptop
cd ~/maps

# View database info
rtabmap-info live_stream_map.db

# Export for navigation
rtabmap-export --poses live_stream_map.db
rtabmap-export --grid live_stream_map.db

# Copy back to Jetson for navigation
scp live_stream_map.db user@jetson-ip:~/maps/
```

---

## Complete Example Workflow

### On Jetson:
```bash
cd ~/sahabat_ws
source install/setup.bash
ros2 launch shbat_pkg sahabat_zed_record.launch.py use_rviz:=false
# Drive robot around with joystick
```

### On Laptop (2 terminals):
```bash
# Terminal 1
source /opt/ros/humble/setup.bash
ros2 run rtabmap_sync rgbd_sync --ros-args -p frame_id:=zed2i_camera_link -p approx_sync:=true -r rgb/image:=/zed2i/zed_node/rgb/image_rect_color -r rgb/camera_info:=/zed2i/zed_node/rgb/camera_info -r depth/image:=/zed2i/zed_node/depth/depth_registered

# Terminal 2
mkdir -p ~/maps
ros2 launch rtabmap_slam rtabmap.launch.py frame_id:=zed2i_camera_link subscribe_rgbd:=true rtabmap_viz:=true database_path:=~/maps/my_map.db args:="--delete_db_on_start" odom_topic:=/zed2i/zed_node/odom Reg/Force3DoF:=true
```

Watch the map build in real-time! 🚀
