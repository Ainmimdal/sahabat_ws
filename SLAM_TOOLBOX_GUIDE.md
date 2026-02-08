# SLAM Toolbox Navigation Guide

**Last Updated:** January 5, 2026

## Overview

slam_toolbox provides lightweight 2D SLAM using only LIDAR data. It's optimized for low CPU usage on Orin Nano (chosen over RTAB-Map which caused lag).

## Current Status

- ✅ SLAM Toolbox configured and working
- ✅ RViz panel integrated with controls
- ✅ Noise reduction tuned (scan filtering + slam params)
- ✅ Nav2 integration working
- ⏳ Next: Add ZED camera for extra costmap layer

## Quick Start

### Step 1: Create a Map (Mapping Mode)

```bash
# Launch in mapping mode
ros2 launch shbat_pkg slam_nav_launch.py mode:=mapping
```

Then:
1. Drive the robot around your environment using joystick
2. Watch the map build in RViz
3. Cover all areas you want the robot to navigate

### Step 2: Save the Map

**Method 1: RViz Panel** (easiest)
- Click **Serialize Map** in slam_toolbox panel
- Enter path: `/home/sahabat/maps/my_map`

**Method 2: Command Line**
```bash
# Save as .pgm/.yaml for Nav2
ros2 run nav2_map_server map_saver_cli -f /home/sahabat/maps/my_map

# Or serialize for slam_toolbox (includes full graph)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/sahabat/maps/my_map'}"
```

### Step 3: Use the Map (Localization Mode)

```bash
# Launch in localization mode with saved map
ros2 launch shbat_pkg slam_nav_launch.py mode:=localization map_file:=/home/sahabat/maps/my_map
```

---

## RViz SLAM Toolbox Panel

### Buttons

| Button | Function |
|--------|----------|
| **Save Map** | Saves map as `.pgm` + `.yaml` (standard Nav2 format) |
| **Serialize Map** | Saves full SLAM graph (`.posegraph` + `.data`) for later editing/localization |
| **Deserialize Map** | Loads a `.posegraph` file to continue mapping or localize |
| **Clear Changes** | Discard recent map changes |
| **Clear Queue** | Clear pending scan processing |

### Checkboxes

| Checkbox | Function |
|----------|----------|
| **Pause New Measurements** | Stops processing LIDAR scans (map freezes). Use when someone walks in front. |
| **Interactive Mode** | Enable manual pose editing - drag nodes in RViz to fix map distortions |
| **Pause Processing** | Pause internal optimization (rarely needed in async mode) |

### Radio Buttons

| Option | Function |
|--------|----------|
| **Continue Mapping** | Default - adds new scans to map as you drive |
| **Localize** | Freezes map, robot only tracks position within existing map |

---

## Commands Reference

### Mapping Mode
```bash
ros2 launch shbat_pkg slam_nav_launch.py mode:=mapping
```

### Localization Mode
```bash
ros2 launch shbat_pkg slam_nav_launch.py mode:=localization map_file:=/path/to/map
```

### Save Map
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/sahabat/maps/my_map'}}"
```

### Serialize Map (for sharing)
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/sahabat/maps/my_map'}"
```

---

## Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                     MAPPING MODE                             │
│  1. Launch with mode:=mapping                                │
│  2. Drive robot around environment                           │
│  3. Save map when done                                       │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                   LOCALIZATION MODE                          │
│  1. Launch with mode:=localization map_file:=/path/to/map   │
│  2. Robot localizes within saved map                         │
│  3. Navigate using Nav2 goal poses                           │
│  4. Run waypoint patrol for autonomous operation             │
└─────────────────────────────────────────────────────────────┘
```

---

## Tips

1. **Good Mapping Practice**
   - Drive slowly during mapping
   - Visit all areas at least twice
   - Include loops (return to start) for loop closure
   - Avoid fast rotations

2. **Map Quality**
   - Check for gaps or misalignment in RViz
   - Re-map areas that look wrong
   - Use `map_update_interval` to control CPU usage

3. **Localization**
   - Robot needs to "see" familiar features to localize
   - If lost, drive to an area with distinctive features
   - Initial position can be set with "2D Pose Estimate" in RViz

---

## Parameters (config/slam_toolbox.yaml)

### Key Parameters

| Parameter | Current Value | Description |
|-----------|---------------|-------------|
| `resolution` | 0.03 | Map resolution (meters per cell) |
| `map_update_interval` | 3.0 | Seconds between map updates |
| `max_laser_range` | 8.0 | Max LIDAR range to use |
| `do_loop_closing` | true | Enable loop closure detection |
| `scan_buffer_size` | 5 | Buffers scans to reduce noise |
| `minimum_score` | 0.5 | Reject poor scan matches |

### Scan Filter (config/scan_filter.yaml)

| Parameter | Current Value | Description |
|-----------|---------------|-------------|
| `min_range` | 0.15 | Ignore readings < 15cm (robot body + noise) |
| `max_range` | 8.0 | Ignore readings > 8m (noisy far readings) |
| `filter_zones` | [80,100,-100,-80] | Block beam angles (left/right uprights) |

---

## Troubleshooting

### Ghost obstacles / black dots in map
- **Cause:** LIDAR noise, dust particles, or reflective surfaces
- **Solutions applied:**
  - Increased `min_range` to 0.15m (filters close noise)
  - Reduced `max_range` to 8.0m (far readings are noisy)
  - Added `scan_buffer_size: 5` (averages multiple scans)
  - Added `minimum_score: 0.5` (rejects poor matches)
- **If persists:** Enable `Pause New Measurements` when disturbance passes

### Map not building
- Check `/scan` topic is publishing: `ros2 topic echo /scan --once`
- Verify TF tree: `ros2 run tf2_tools view_frames`

### High CPU usage
- Increase `map_update_interval` (e.g., 5.0)
- Decrease `max_laser_range`
- Set `throttle_scans: 2` to skip every other scan

### Robot not localizing
- Ensure map file path is correct
- Try setting initial pose in RViz (2D Pose Estimate button)
- Drive to a distinctive area with clear features

---

## Hardware Notes

### USB Device Connections (CRITICAL)
| Device | Port | Baud Rate | Notes |
|--------|------|-----------|-------|
| ZLAC8015D Motor | FTDI (A50285BI) | 115200 | USB hub OK |
| Oradar MS200 LIDAR | FTDI (BG00Y2Y1) | 230400 | USB hub OK |
| Wheeltec N100 IMU | CP2102 (0001) | 921600 | **DIRECT USB ONLY** - hub causes I/O errors |

### Robot Specs
- Wheel diameter: 173mm (radius: 0.0865m)
- Wheel base: 0.33m
- LIDAR height: ~0.15m from ground

---

## Next Steps

After setting up SLAM:
1. ✅ Create a map of your environment
2. ✅ Set up waypoint patrol with persistent waypoints  
3. ⏳ Add ZED camera for depth-based obstacle detection in costmap
