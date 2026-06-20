# Changelog

## 0.2.2

- Use a responsive two-column console and give it more layout width.
- Collapse duplicate health cards into a compact status and recovery block.
- Keep hardware bringup alive in Idle so teleoperation does not require a map.
- Start and stop only navigation layers when switching operating modes.

## 0.2.1

- Replace tabbed navigation with one continuous operator screen.
- Capture WASD at the Foxglove window level so the 3D panel can retain focus.
- Keep gamepad polling active regardless of panel focus.
- Add independent forward and turn speed controls.
- Remove software E-stop controls and mode-switch latching from remote UI.
- Keep localization recovery visible directly below manual drive.

## 0.2.0

- Replace the crowded console tabs with Drive, Maps, Routes, and Health tasks.
- Add lease-checked global localization recovery with live progress and stop.
- Open saved maps in complete operations mode with map-specific routes and dock.
- Use AMCL covariance in localization health instead of topic presence alone.
- Remove the gamepad deadman setting; neutral stick and disconnect command zero.
- Add dock capture, enabled-waypoint editing, lease release, and responsive cards.

## 0.1.3

- Drive directly with W/A/S/D and stop without automatically latching E-stop
  on key release, focus loss, disconnect, lease expiry, or command timeout.
- Clear the startup software E-stop automatically when entering mapping while
  stationary.

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
