# Waypoint Patrol Guide

## Quick Start

### 1. Start Nav2 (in one terminal)
```bash
ros2 launch shbat_pkg nav2_test_launch.py
```

### 2. Start Waypoint Patrol (in another terminal)
```bash
ros2 run shbat_pkg waypoint_patrol
```

Or with a pre-defined waypoints file:
```bash
ros2 run shbat_pkg waypoint_patrol --ros-args -p waypoints_file:=/home/sahabat/sahabat_ws/install/shbat_pkg/share/shbat_pkg/config/patrol_waypoints.yaml
```

---

## Recording Waypoints (Teach Mode)

The easiest way to define waypoints is to drive the robot around and record positions:

### Step 1: Start patrol node (without starting patrol)
```bash
ros2 run shbat_pkg waypoint_patrol
```

### Step 2: Drive robot to first waypoint position using joystick

### Step 3: Record current position as waypoint
```bash
ros2 topic pub /patrol/add_waypoint std_msgs/Empty "{}" --once
```

### Step 4: Repeat steps 2-3 for each waypoint

### Step 5: Save waypoints to file
```bash
ros2 service call /patrol/save_waypoints std_srvs/srv/Trigger
```
Waypoints will be saved to `/tmp/patrol_waypoints.yaml` (or specified file)

### Step 6: Start patrol
```bash
ros2 service call /patrol/start std_srvs/srv/Trigger
```

---

## Control Commands

| Command | Description |
|---------|-------------|
| Start patrol | `ros2 service call /patrol/start std_srvs/srv/Trigger` |
| Stop patrol | `ros2 service call /patrol/stop std_srvs/srv/Trigger` |
| Pause patrol | `ros2 service call /patrol/pause std_srvs/srv/Trigger` |
| Resume patrol | `ros2 service call /patrol/resume std_srvs/srv/Trigger` |
| Add waypoint | `ros2 topic pub /patrol/add_waypoint std_msgs/Empty "{}" --once` |
| Save waypoints | `ros2 service call /patrol/save_waypoints std_srvs/srv/Trigger` |
| Clear waypoints | `ros2 service call /patrol/clear_waypoints std_srvs/srv/Trigger` |

---

## Monitor Status

```bash
ros2 topic echo /patrol/status
```

Output example:
```
data: 'State: navigating | Waypoint: 2/5 | Loop: True'
```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `waypoints_file` | '' | Path to YAML file with waypoints |
| `loop` | true | Continuously loop through waypoints |
| `wait_duration` | 2.0 | Seconds to wait at each waypoint |
| `frame_id` | 'map' | TF frame for waypoint coordinates |

Example with custom parameters:
```bash
ros2 run shbat_pkg waypoint_patrol --ros-args \
  -p waypoints_file:=/path/to/waypoints.yaml \
  -p loop:=false \
  -p wait_duration:=5.0
```

---

## Waypoints File Format

```yaml
waypoints:
  - x: 1.0      # meters
    y: 0.0      # meters  
    yaw: 0.0    # radians (0 = facing +X, 1.57 = facing +Y)
    
  - x: 2.0
    y: 1.0
    yaw: 1.57
```

---

## Tips

1. **Test first waypoint manually** - Use RViz "2D Goal Pose" to verify Nav2 can reach it

2. **Start simple** - Begin with 2-3 waypoints close together

3. **Use teach mode** - Easier than manually typing coordinates

4. **Watch for obstacles** - Make sure patrol path is clear

5. **Emergency stop** - Press joystick Button A to stop immediately
