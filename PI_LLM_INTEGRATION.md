# Sahabat Robot - Pi/LLM Integration Guide

This guide explains how to control the Sahabat robot from a Raspberry Pi running an LLM.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Raspberry Pi                               │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │                    LLM Agent                            │ │
│  │  "Take me to the Modern Art exhibit"                   │ │
│  └─────────────────────────────────────────────────────────┘ │
│                          │                                   │
│                          │ Sends exhibit name                │
│                          ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │     POST /exhibit/goto {"exhibit": "exhibit_a"}        │ │
│  └─────────────────────────────────────────────────────────┘ │
└───────────────────────────│─────────────────────────────────┘
                            │ HTTP REST API (port 5000)
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                   Jetson Orin Nano (Robot)                  │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              Exhibit Navigator                          │ │
│  │  1. Looks up "exhibit_a" → coordinates                 │ │
│  │  2. Finds predefined route from current location       │ │
│  │  3. Navigates through waypoints (avoids awkward paths) │ │
│  │  4. Publishes /exhibit_arrival when reached            │ │
│  └─────────────────────────────────────────────────────────┘ │
│                          │                                   │
│                          │ Nav2 NavigateThroughPoses         │
│                          ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              Nav2 / SLAM / Motors                       │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Key Concept: Route-Based Navigation

Instead of sending raw coordinates (which may result in awkward paths), the Pi sends **exhibit names**. The robot uses **predefined routes** between exhibits.

**Benefits:**
- Robot takes sensible paths (not through narrow gaps or awkward areas)
- Routes are tested and known to work
- Pi/LLM doesn't need to know coordinates
- Easy to modify routes without changing Pi code

## Setup on Robot (Jetson Orin Nano)

### 1. Configure Exhibits and Routes

Edit `~/sahabat_ws/src/shbat_pkg/config/exhibit_routes.yaml`:

```yaml
# Named exhibit locations
exhibits:
  entrance:
    x: 0.0
    y: 0.0
    yaw: 0.0
    description: "Main entrance"
  
  exhibit_a:
    x: 3.0
    y: 1.5
    yaw: 1.57
    description: "Modern Art Gallery"
  
  exhibit_b:
    x: 5.0
    y: 0.0
    yaw: 0.0
    description: "Sculpture Garden"

# Predefined routes between exhibits
# These waypoints ensure the robot takes sensible paths
routes:
  "entrance->exhibit_a":
    - [1.5, 0.5, 0.5]    # Turn at corridor
    - [2.5, 1.0, 1.0]    # Approach exhibit
    
  "entrance->exhibit_b":
    - [2.0, 0.0, 0.0]    # Go straight
    - [4.0, 0.0, 0.0]    # Continue down main hall

# Default tour order
default_tour:
  - entrance
  - exhibit_a
  - exhibit_b
  - entrance
```

### 2. Build and Launch

```bash
cd ~/sahabat_ws
colcon build --packages-select shbat_pkg --symlink-install
source install/setup.bash

# Terminal 1: Start localization + navigation
ros2 launch shbat_pkg localization_patrol_launch.py map_file:=/path/to/map use_api:=true

# Terminal 2: Start exhibit navigator
ros2 run shbat_pkg exhibit_navigator
```

## ROS2 Topics (Control Interface)

Control the exhibit navigator via ROS2 topics:

```bash
# Go to an exhibit
ros2 topic pub /exhibit_command std_msgs/String '{"action": "goto", "exhibit": "exhibit_a"}' --once

# Start default tour
ros2 topic pub /exhibit_command std_msgs/String '{"action": "start_tour"}' --once

# Start custom tour
ros2 topic pub /exhibit_command std_msgs/String '{"action": "start_tour", "exhibits": ["exhibit_a", "exhibit_b"]}' --once

# Stop navigation
ros2 topic pub /exhibit_command std_msgs/String '{"action": "stop"}' --once

# Pause tour
ros2 topic pub /exhibit_command std_msgs/String '{"action": "pause"}' --once

# Resume tour
ros2 topic pub /exhibit_command std_msgs/String '{"action": "resume"}' --once

# List available exhibits
ros2 topic pub /exhibit_command std_msgs/String '{"action": "list_exhibits"}' --once

# Listen for arrivals (for TTS trigger)
ros2 topic echo /exhibit_arrival

# Monitor navigation status
ros2 topic echo /exhibit_nav_status

# Listen for ALL robot events (recommended for Pi/LLM)
ros2 topic echo /robot_events
```

## Robot Events (`/robot_events` topic)

The `/robot_events` topic is the **main way for the Pi/LLM to receive notifications** from the robot. All events are JSON strings with:

```json
{
  "event": "event_type",
  "timestamp": 1738000000.0,
  "message": "Human readable message",
  ...additional data...
}
```

### Event Types

| Event | When It Fires | Data Fields |
|-------|--------------|-------------|
| `exhibit_reached` | Arrived at destination | `exhibit`, `description`, `position` |
| `navigation_started` | Started navigating | `target`, `from` |
| `navigation_failed` | Nav2 couldn't reach goal | `target`, `reason`, `position` |
| `navigation_cancelled` | User stopped navigation | `target` |
| `tour_started` | Tour begins | `exhibits`, `total_stops` |
| `tour_completed` | Tour finished | `exhibits_visited` |
| `tour_paused` | Tour paused | `current_exhibit`, `progress` |
| `tour_resumed` | Tour resumed | `next_exhibit`, `progress` |
| `robot_obstructed` | Robot stuck (no movement) | `target`, `position`, `time_stuck` |
| `robot_recovered` | Robot moving again | - |
| `low_battery` | Battery ≤30% | `battery_level` |
| `critical_battery` | Battery ≤15% | `battery_level` |
| `error` | General error | `error`, `message` |

### Example Event Messages

```json
// Arrived at exhibit
{
  "event": "exhibit_reached",
  "timestamp": 1738000123.45,
  "exhibit": "exhibit_a",
  "description": "Modern Art Gallery",
  "position": {"x": 3.0, "y": 1.5, "yaw": 1.57},
  "message": "Arrived at Modern Art Gallery"
}

// Navigation failed
{
  "event": "navigation_failed",
  "timestamp": 1738000456.78,
  "target": "exhibit_b",
  "status_code": 4,
  "position": {"x": 2.1, "y": 0.8},
  "message": "Navigation to exhibit_b failed (status: 4)"
}

// Robot blocked
{
  "event": "robot_obstructed",
  "timestamp": 1738000789.01,
  "target": "exhibit_a",
  "position": {"x": 1.5, "y": 0.5},
  "time_stuck": 12.5,
  "message": "Robot appears to be blocked. No movement for 12s"
}

// Low battery
{
  "event": "low_battery",
  "timestamp": 1738001000.00,
  "battery_level": 25.0,
  "message": "Low battery warning: 25% remaining"
}
```

### Pi/LLM Event Handler Example

```python
import json
import subprocess
import threading

def listen_to_events():
    """Listen for robot events and handle them"""
    process = subprocess.Popen(
        ["ssh", "sahabat@ROBOT_IP", "ros2 topic echo /robot_events --no-arr"],
        stdout=subprocess.PIPE, text=True
    )
    
    buffer = ""
    for line in process.stdout:
        buffer += line
        if line.strip() == "---":
            try:
                # Parse the YAML-style output
                event = parse_ros_message(buffer)
                handle_event(event)
            except:
                pass
            buffer = ""

def handle_event(event):
    """Handle robot events - trigger TTS, update UI, etc."""
    event_type = event.get('event')
    message = event.get('message', '')
    
    if event_type == 'exhibit_reached':
        exhibit = event.get('description', event.get('exhibit'))
        speak(f"We have arrived at {exhibit}. Please enjoy the exhibit.")
        
    elif event_type == 'navigation_failed':
        speak(f"I'm sorry, I couldn't reach the destination. {message}")
        
    elif event_type == 'robot_obstructed':
        speak("I seem to be blocked. Could someone please clear the path?")
        
    elif event_type == 'low_battery':
        speak(f"Battery is getting low at {event.get('battery_level'):.0f}%")
        
    elif event_type == 'critical_battery':
        speak("Critical battery! I need to return to charging.")
        # Automatically go home
        send_command("goto", exhibit="charging_station")
        
    elif event_type == 'tour_completed':
        speak("The tour is now complete. Thank you for visiting!")

def speak(text):
    """Text to speech (implement with your preferred TTS)"""
    print(f"[TTS] {text}")
    # Use espeak, gTTS, or cloud TTS here

# Start event listener in background
threading.Thread(target=listen_to_events, daemon=True).start()
```

## Status Topic (`/exhibit_nav_status`)

The status topic publishes every 1 second with current robot state:

```json
{
  "state": "navigating",       // idle, navigating, waiting, touring, paused, error, obstructed
  "current_exhibit": "entrance",
  "target_exhibit": "exhibit_a",
  "is_touring": true,
  "tour_progress": "2/5",
  "robot_position": {"x": 1.5, "y": 0.8, "yaw": 0.5},
  "battery_level": 85.0,
  "is_obstructed": false
}
```

## Example: Simple Pi Client

```python
import requests
import json

ROBOT_IP = "192.168.1.100"

# Using ROS2 bridge or custom HTTP endpoint
def send_command(action, **kwargs):
    """Send command to exhibit navigator"""
    cmd = {"action": action, **kwargs}
    # Option 1: Via rosbridge websocket
    # Option 2: Via custom HTTP API (needs api_bridge modification)
    print(f"Sending: {cmd}")

def goto_exhibit(name):
    send_command("goto", exhibit=name)

def start_tour(exhibits=None):
    if exhibits:
        send_command("start_tour", exhibits=exhibits)
    else:
        send_command("start_tour")

def stop():
    send_command("stop")

# Usage
goto_exhibit("exhibit_a")
start_tour(["entrance", "exhibit_a", "exhibit_b"])
stop()
```

## Example: Voice-Controlled Tour Guide

```python
import subprocess
import json

def ros2_publish(topic, msg_type, data):
    """Publish to ROS2 topic from Pi (requires rosbridge or SSH)"""
    cmd = f"ros2 topic pub {topic} {msg_type} '{json.dumps(data)}' --once"
    subprocess.run(["ssh", "sahabat@ROBOT_IP", cmd])

def goto_exhibit(name):
    ros2_publish("/exhibit_command", "std_msgs/String", 
                 {"action": "goto", "exhibit": name})

def start_tour():
    ros2_publish("/exhibit_command", "std_msgs/String",
                 {"action": "start_tour"})

# Voice recognition loop
EXHIBITS = {
    "modern art": "exhibit_a",
    "sculptures": "exhibit_b", 
    "photography": "exhibit_c",
}

def handle_voice(text):
    text = text.lower()
    
    if "tour" in text:
        print("Starting tour...")
        start_tour()
        return
    
    if "stop" in text:
        ros2_publish("/exhibit_command", "std_msgs/String", {"action": "stop"})
        return
    
    for keyword, exhibit_id in EXHIBITS.items():
        if keyword in text:
            print(f"Going to {keyword}...")
            goto_exhibit(exhibit_id)
            return
    
    print("I don't understand")
```

## Listening for Arrivals (TTS Trigger)

When robot arrives at exhibit, announce it:

```python
# On the robot (or Pi with rosbridge)
import rclpy
from std_msgs.msg import String
import json
from gtts import gTTS
import os

def arrival_callback(msg):
    data = json.loads(msg.data)
    exhibit = data['exhibit']
    description = data['description']
    
    # Text to speech
    text = f"We have arrived at {description}. Please enjoy the exhibit."
    tts = gTTS(text=text, lang='en')
    tts.save("/tmp/announcement.mp3")
    os.system("mpg321 /tmp/announcement.mp3")

rclpy.init()
node = rclpy.create_node('arrival_announcer')
node.create_subscription(String, '/exhibit_arrival', arrival_callback, 10)
rclpy.spin(node)
```

## Creating Routes

### How to Create Good Routes:

1. **Drive the robot manually** using joystick to each exhibit
2. At key turning points, note the coordinates from `/amcl_pose`
3. Add waypoints to `exhibit_routes.yaml`
4. Test the route

### Tips:
- Add waypoints at **corners** and **narrow passages**
- Keep waypoints **30cm+ away from walls**
- Routes work both directions (auto-reversed)
- Test routes before deployment

### Quick Coordinate Capture:
```bash
# Get current robot position
ros2 topic echo /amcl_pose --once | grep -A3 position
```

## Troubleshooting

### Robot takes weird path
- Check if route is defined in exhibit_routes.yaml
- Add intermediate waypoints to guide the robot

### "Unknown exhibit" error
- Check exhibit name spelling
- Reload config: `{"action": "reload_config"}`

### Robot doesn't reach exhibit
- Verify exhibit coordinates match actual location
- Check for obstacles in the way
