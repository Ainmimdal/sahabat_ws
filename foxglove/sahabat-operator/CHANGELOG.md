# Changelog

## 0.1.2

- Publish Foxglove teleop through the standard `sensor_msgs/Joy` schema so the
  bridge does not depend on a client-side custom message definition.

## 0.1.1

- Keep connection state stable between status frames.
- Publish keyboard commands at 10 Hz so the 250 ms safety watchdog stays fed.
- Preserve the deadman while controls are centred, avoiding false E-stops.
- Show map, lidar and TF health separately in the console.
- Supply the complete ROS 2 datatype graph for TeleopCommand publication.

## 0.1.0

- Add lease-aware console, fail-safe keyboard/gamepad teleoperation, named map
  management and revision-aware waypoint editing.
