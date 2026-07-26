# Sahabat RViz Waypoint Editor Plan

This plan tracks the migration from the current Tkinter waypoint manager to a
native RViz waypoint and route editor for gallery tours. The old UI stays
available until the RViz workflow passes equivalent physical robot tests.

## Goals

- Edit gallery tour waypoints directly inside RViz on top of saved maps.
- Support offline editing without robot hardware, sensors, Nav2, or motors.
- Keep map-scoped waypoint sets, dock pose, revision checks, and existing data.
- Model preferred gallery travel paths so the robot avoids technically possible
  but undesirable routes.
- Support abrupt skip/go-back requests by routing through known preferred
  segments, with explicit direct-navigation fallback when allowed.

## Concepts

| Concept | Meaning |
| --- | --- |
| Map | Saved gallery floor map under `maps/<map_id>/` |
| Waypoint set | One editable tour configuration for a map |
| Stop | Main exhibit/tour pose where the robot may wait or talk |
| Route segment | Preferred path from one stop to another |
| Via point | Optional sub-waypoint inside a route segment to shape Nav2 behavior |
| Dock | Map-level start/home pose shared by all waypoint sets |

Route segments are one-way by default. A reverse direction needs its own segment,
a bidirectional flag, or direct fallback.

## Target Storage

Waypoint sets remain map-scoped:

```text
maps/<map_id>/
  map.yaml
  map.pgm
  dock.yaml
  waypoint_sets/
    index.yaml
    default.yaml
```

Extended set format:

```yaml
name: Default
revision: 2
waypoints:
  - id: exhibit-a
    name: Exhibit A
    x: 1.2
    y: 0.5
    yaw: 1.57
    dwell_seconds: 8.0
    enabled: true
segments:
  - id: exhibit-a-to-exhibit-b
    name: Exhibit A to Exhibit B
    from_waypoint_id: exhibit-a
    to_waypoint_id: exhibit-b
    bidirectional: false
    enabled: true
    via_points:
      - id: via-1
        name: via
        x: 2.0
        y: 0.8
        yaw: 0.0
        dwell_seconds: 0.0
        enabled: true
settings:
  direct_fallback: warn
```

Existing waypoint-only files remain valid. Missing `segments` means no preferred
route segments have been authored yet.

## Offline Editor Mode

Target command:

```bash
ros2 launch shbat_pkg waypoint_editor.launch.py map_id:=rdlfront
```

Offline mode must start only map visualization and editing services:

- `map_server`
- map lifecycle manager
- waypoint editor backend
- RViz with the Sahabat waypoint editor panel

Current implementation vendors `github.com/Ainmimdal/waypoint_editor` as the
RViz-native editing surface. Offline launch uses its panel/tool for drag/rotate
interactive markers, context menus, load/save, undo/redo, clear, and map loading.
Sahabat map-scoped set integration and route segment/via-point editing are the
next phases.

Offline mode must not start:

- motor controller
- joystick
- lidar
- IMU
- EKF
- Nav2 controller/planner/behavior servers
- velocity smoother
- localization recovery
- any `/cmd_vel` publisher

## RViz UI Behavior

The RViz editor should follow the behavior of
`https://github.com/ainmimdal/waypoint_editor` where it fits Sahabat:

- Toolbar tool for adding stops or via points.
- Click-and-drag to set pose and yaw.
- Interactive markers for dragging and rotating stops/via points.
- Right-click context menu for delete, rename, enable/disable, and edit metadata.
- Undo, redo, clear selected route/waypoints.
- Route lines and direction arrows shown on the map.
- Distance display for selected segment and total route.

Sahabat-specific additions:

- Saved map selector.
- Waypoint set selector with new/rename/archive/save/reload.
- Edit modes: Stops, Route Segment, Dock.
- Segment selector: from-stop, to-stop, bidirectional, enabled.
- Explicit fallback warning when no preferred route exists.

## Navigation Semantics

- Main stops are tour/exhibit stops and can have dwell time.
- Via points shape route travel and never get dwell time.
- For a requested stop-to-stop leg, use a direct route segment if available.
- For skip/go-back, resolve a multi-segment path through the route graph.
- If no preferred path exists, direct fallback may be allowed, warned, or blocked
  based on settings.
- Live execution should use `NavigateThroughPoses` for each leg so via points are
  pass-through points and the final stop is the only stop for that leg.

## Implementation Phases

1. Extend waypoint storage and interfaces for route segments.
2. Add a storage-only offline waypoint editor backend.
3. Add native RViz panel/tool in `shbat_rviz_plugins`.
4. Add `waypoint_editor.launch.py` and `rviz/waypoint_editor.rviz`.
5. Add route graph resolver and fallback warnings.
6. Add live route testing and `NavigateThroughPoses` execution.
7. Migrate `exhibit_navigator.py` to read map-scoped route graphs directly.
8. Keep Tkinter waypoint manager until physical tests prove replacement parity.

## Validation Order

Run least invasive checks first:

```bash
python3 -m pytest src/shbat_pkg/test/test_waypoint_store.py
colcon build --packages-select sahabat_interfaces shbat_rviz_plugins shbat_pkg --symlink-install
ros2 launch shbat_pkg waypoint_editor.launch.py map_id:=rdlfront
```

Offline validation must confirm no hardware probing and no motor movement.
Physical route tests require a person beside the emergency stop.
