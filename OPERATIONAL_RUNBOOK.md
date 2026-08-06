# Sahabat Robot Operational Runbook

This runbook defines the canonical entry points for normal operation and
diagnostics. The older launch commands remain supported during migration.

## Build

```bash
cd ~/sahabat_ws
colcon build --packages-select shbat_pkg --symlink-install
source install/setup.bash
```

Build `sahabat_interfaces` before `shbat_pkg` after changing operator messages:

```bash
colcon build --packages-select sahabat_interfaces shbat_pkg --symlink-install
```

Do not routinely remove `build/` or `install/`; they support fast incremental
builds and contain the currently runnable workspace.

## Canonical launches

Remote Foxglove operation (safe idle, E-stop active at startup):

```bash
ros2 launch shbat_pkg remote_operations.launch.py
```

See `FOXGLOVE_OPERATIONS.md` for extension setup, keyboard/gamepad controls,
named maps, waypoints, Android gateway and the two-person acceptance procedure.

Basic hardware bringup without Nav2:

```bash
ros2 launch shbat_pkg bringup.launch.py
```

Odometry-only Nav2 diagnostics:

```bash
ros2 launch shbat_pkg navigation.launch.py mode:=odom_only
```

SLAM mapping:

```bash
ros2 launch shbat_pkg navigation.launch.py mode:=mapping
```

Mapping opens RViz plus the **Sahabat Mapping** panel. Enter a meaningful map
name and press **Save Map**. Maps go to `~/sahabat_ws/maps/` by default; the
panel shows the directory and lets you choose another one. A normal save makes
`<name>.yaml` and `<name>.pgm` for navigation. The recommended editable-session
option also makes `<name>.posegraph` and `<name>.data` so development can
continue from the same SLAM graph later.

The old SLAM Toolbox terms are intentionally hidden from the normal workflow:
"serialize" means save an editable mapping session, and "deserialize" means
load that session again. Neither is required simply to use a finished map for
navigation.

Saved-map localization:

```bash
ros2 launch shbat_pkg navigation.launch.py \
  mode:=localization map_file:=/home/sahabat/sahabat_ws/maps/gallery_map
```

### Keepout masks

Keepout zones are stored separately from the occupancy map so AMCL continues
matching lidar scans against the original mapped walls. For a flat map named
`maps/gallery_map.yaml`, use:

```text
maps/gallery_map_keepout.yaml
maps/gallery_map_keepout.pgm
```

For a directory map at `maps/gallery_map/map.yaml`, use
`maps/gallery_map/keepout.yaml` and its referenced image. The mask YAML must
have the same `resolution` and `origin` as the navigation map, with origin yaw
equal to zero. Use a white mask for permitted space and solid black regions for
keepout zones. Draw enough clearance for the complete robot footprint because
costmap filters are not inflated by the normal inflation layer.

Localization automatically enables the mask when the conventional path exists.
An explicit path or intentional disable can be supplied with:

```bash
ros2 launch shbat_pkg navigation.launch.py \
  mode:=localization \
  map_file:=/home/sahabat/sahabat_ws/maps/gallery_map \
  keepout_mask_file:=/home/sahabat/sahabat_ws/maps/custom_keepout.yaml

ros2 launch shbat_pkg navigation.launch.py \
  mode:=localization \
  map_file:=/home/sahabat/sahabat_ws/maps/gallery_map \
  use_keepout:=false
```

Gallery operation with the waypoint GUI and external API:

```bash
ros2 launch shbat_pkg operations.launch.py \
  map_file:=/home/sahabat/sahabat_ws/maps/gallery_map \
  use_api:=true
```

### Local SahaBot web app

The SahaBot LLM interface is installed separately at `~/sahabot`. Run its ROS
API bridge on loopback so the unauthenticated command API is not exposed:

```bash
cd ~/sahabat_ws
source install/setup.bash
API_HOST=127.0.0.1 ros2 launch shbat_pkg operations.launch.py \
  map_file:=/home/sahabat/sahabat_ws/maps/rdlfront \
  use_api:=true
```

Start the built web app in a second terminal:

```bash
cd ~/sahabot
~/sahabat_ws/scripts/start_sahabot_desktop.sh
```

Open `http://127.0.0.1:8000`. Ainmimdal's May 15 web gateway maps its
`station-1` through `station-6` cards to the logical names `waypoint_1` through
`waypoint_6`. The robot API resolves those names against the active online
waypoint-editor map and set through `/operator/...` services. The browser does
not send map coordinates.

When using **Sahabat Waypoint Editor Live**, its launcher starts the same
loopback-only API automatically. Select an editor set containing waypoints
named `waypoint_1` through `waypoint_6` before using the corresponding SahaBot
station buttons. Keep the API on loopback; it is unauthenticated.

#### SahaBot tour profiles and first-version interaction

SahaBot resolves logical stations against the active map waypoint set. It does
not receive or store map coordinates in the browser. The `auto` tour profile
treats `gallerysq4` as strict production data and other maps as permissive test
data. Production requires all six names from `waypoint_1` through
`waypoint_6`; test maps may provide any subset.

Run the strict gallery profile with:

```bash
ros2 launch shbat_pkg operations.launch.py \
  map_file:=/home/sahabat/sahabat_ws/maps/gallerysq4 \
  map_id:=gallerysq4 use_api:=true tour_profile:=production
```

Exercise the partial `rdlsabtu/tests` set without a motor controller with:

```bash
ros2 launch shbat_pkg operations.launch.py \
  map_file:=/home/sahabat/sahabat_ws/maps/rdlsabtu \
  map_id:=rdlsabtu use_api:=true tour_profile:=test \
  use_hardware:=false use_zed:=false use_battery_monitor:=false
```

That non-motion launch validates the profile, station availability, UI, and
rejection paths; navigation stays disabled because live localization and scan
health are required. Exercise departure, pause/resume, blocked, arrival, and
Next behavior on `rdlsabtu` only in a separate supervised `use_hardware:=true`
run with a person beside the E-stop.

In test mode, missing station cards are dimmed but remain browseable; their
**Take Me There** action is disabled. A direct request for a missing station is
rejected before a Nav2 goal is sent. **Next Exhibit** skips missing stations
and stops at the final available exhibit; it never wraps to the first station.
After every confirmed arrival the robot stays in place until a visitor or
operator requests another destination. **Start Tour** goes to the first
available exhibit only; it does not launch an unattended waypoint patrol.

Visitor speech is event-driven. Departure is announced only after Nav2 accepts
the goal, **almost there** is emitted once only on a sufficiently long trip,
and exhibit narration begins only after Nav2 succeeds within the configured
arrival tolerance. Merely mentioning or asking about a station does not move
the robot; the request must contain an explicit movement instruction. A new
goal is rejected while another goal is active. The navigation banner provides
Pause, Resume, and Cancel controls, while the voice card provides a local
**Stop speaking** control. Movement requests remain unavailable until map,
scan, TF, and localization health are all ready and E-stop is clear.

The map dock remains separate from the exhibit order. Capture it with the
waypoint editor, then use the confirmed **Return to Dock** button under SahaBot
Settings -> Operator Controls. The button is disabled when the active map has
no `maps/waypoint_sets/<map_id>/dock.yaml`. The robot does not automatically
return to dock after the final exhibit.

Deepgram speech audio is cached under `~/.cache/sahabot/tts` using the provider,
voice, normalized text, language, and format as the cache identity. Changing
the Deepgram voice in SahaBot Settings selects a different cache namespace, so
audio from the previous voice is not replayed. The kiosk browser remembers the
selected voice and reapplies it when the app reconnects. The default cache
limit is 512 MB and can be changed with `TTS_CACHE_MAX_MB`.

### Preferred routes and doorway approaches

Waypoint sets may contain a sparse graph of human-approved `segments`. Each
segment connects two normal destination waypoint IDs and contains zero or more
`via_points`. The operator backend finds the closest current destination,
searches the connected graph, and sends all resulting route points through one
`NavigateThroughPoses` action. Intermediate points shape the path but do not
produce waypoint-follower pauses.

Use a bidirectional segment when the same path is safe in both directions. Its
via-point order is reversed automatically. Use two one-way segments when a
door or blind corner needs different entry geometry in each direction. For a
narrow doorway, place one point on each side of the opening, aligned through
its center; do not place only one point directly in the opening.

Edit these routes in the same RViz waypoint editor. For an offline
`gallerysq4` session with no robot bringup:

```bash
cd ~/sahabat_ws
source install/setup.bash
ros2 launch shbat_pkg waypoint_editor.launch.py \
  map_id:=gallerysq4 waypoint_backend:=local start_map_server:=true
```

Open the **Routes** tab. Choose an entry under **1. Select route** to edit it;
that route becomes thick and bright cyan while other routes are faded. The
large **SELECTED ROUTE** banner stays fixed at the top of the panel while its
contents scroll, and route selection is preserved across list refreshes. The
**Points in selected route (travel order)** list shows exactly what Nav2 will
receive between the two destination waypoints. Select a row to **Move Up**,
**Move Down**, or **Remove** it from the selected route.

On the map, the selected path is the thick bright-cyan line. Its route points
use their permanent map-wide IDs, such as `rp-001`, directly. The same ID is
shown in both sidebar lists; there is no separate generated display number to
change when points are added or removed. Only points in the selected route
capture viewport clicks; their larger cyan halo and label are both draggable
selection targets. New points receive the next unused `rp-NNN` ID.

The **All route points on this map** list is the reusable point library. It
shows how many routes use every point and marks points already in the selected
route. Multiple rows may be selected at once:

- **Add New Point to Selected Route** switches RViz to the waypoint tool. The
  next map click creates a draggable cyan point on the selected route.
- **Add to Route** attaches the selected existing points to the
  end of the route. Shared points keep one ID and pose, so moving or renaming
  one updates every route using it.
- **Delete from Map** removes the selected points from every route and requires
  confirmation. Use **Remove** instead when the point
  should remain available to other routes.

To make a new route, open **New Route**, choose **From** and **To**, set
**Bidirectional**, then press **Create Route**. Return to **Routes** to define
its ordered route points.

Right-click a cyan point to rename it, move it earlier or later within the
selected route, or remove it from that route. Use **Save WPs + Routes** to
persist both destinations and route points. Fully restart RViz after rebuilding
the plugin because an already-running RViz process keeps the previous shared
library loaded.

The active `gallerysq4/new-tour` set contains six one-way connections forming a
clockwise ring: `waypoint_1` through `waypoint_6`, then back to `waypoint_1`.
Every destination can reach every other destination by continuing clockwise.
Its `rp-NNN` points keep each Nav2 request on the approved corridor and doorway
approaches. The cyan `/waypoint_markers` lines are the saved preferred routes.
The thick magenta `/plan` line is the actual current Nav2 plan and may deviate
locally to avoid an obstacle.

Route settings support:

- `route_origin_tolerance`: maximum distance from the robot to the nearest
  destination used as the graph origin.
- `direct_fallback: warn`: log and use direct Nav2 planning if the graph cannot
  be used.
- `direct_fallback: reject`: reject navigation unless a preferred graph route
  exists. Use this only after every expected origin, including the dock, has
  been connected and physically tested.

ZED and remote visualization remain opt-in:

```bash
ros2 launch shbat_pkg operations.launch.py \
  map_file:=/home/sahabat/maps/gallery \
  use_zed:=true use_foxglove:=true use_api:=true
```

## Emergency stop

Stop and latch the motor controller:

```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once
```

For the remote operations stack, clear the latch only through the operator UI
while holding its control lease. The raw command below is retained only for
legacy launches and supervised diagnostics:

```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once
```

The API endpoint activates the latch when called with an empty body. Clear it
explicitly with `{"active": false}`.

## Pre-motion checks

```bash
ros2 topic echo /wheel_odom --once
ros2 topic echo /odom --once
ros2 topic echo /imu --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_link
```

Before sending a navigation goal, confirm that the scan aligns with walls,
odometry moves in the correct direction, and the emergency stop latches.

For gallery route and motion tuning, confirm that normal velocity has exactly
one final publisher and record both the requested and delivered commands:

```bash
ros2 topic info /cmd_vel --verbose
ros2 bag record \
  /plan /global_costmap/costmap /local_costmap/costmap \
  /cmd_vel_nav /cmd_vel_nav_smoothed /cmd_vel \
  /joy /odom /wheel_odom
```

In the canonical operations launch, `/cmd_vel` should list only
`/command_arbiter`. The base-controller startup log must also show the motor
acceleration and deceleration register readback before the motor test begins.

### Known-good Nav2 motion baseline

The following values in `config/nav2_odom_only.yaml` produced good supervised
motion on June 20, 2026 and are the rollback point for future tuning:

- 0.3 m/s linear cruise with a 0.5 m/s command cap
- 0.5 rad/s angular cap for 10 Hz lidar alignment
- 0.5 m lookahead with a 0.3-0.9 m adaptive range
- 0.3 rad rotate-to-heading threshold
- 30 Hz velocity smoothing, 0.6 m/s² linear acceleration, and 0.5 m/s² normal
  deceleration

This combination reduced curve overshoot and abrupt normal stopping without
reintroducing the low-angular-command drivetrain deadlock.

The July 31 gallery candidate changes the cruise to 0.42 m/s, uses
SmacPlanner2D plus collision-checked path smoothing, pivots only above 0.6 rad,
and accepts 0.10 m / 0.12 rad final pose error. These values are not yet a
physical baseline. The command arbiter must be the sole normal `/cmd_vel`
publisher during this test.

After changing Nav2 speed or controller tuning, use a clear straight test lane
with a person beside the emergency stop. Start with a short goal at 0.2 m/s,
then repeat at 0.3 m/s before testing the 0.42 m/s candidate or allowing the
0.5 m/s command cap. Test a wide 90-degree turn separately and confirm the
lidar scan stays aligned with walls. Stop the test if wheel odometry jumps, the
scan smears, the controller oscillates, or stopping distance is unsafe. Keep
angular velocity at or below
0.5 rad/s until a higher rate passes a logged lidar/localization test.
During the turn test, small heading corrections should form a continuous arc;
larger initial heading errors may cause an in-place pivot. Stop and
retune if navigation repeatedly alternates between pivoting, creeping forward,
and braking.

## JUNCTEK KG-F battery monitor

The KG110F uses the kernel's existing `cdc_acm` USB serial driver. Install the
workspace rule once to grant access and create a stable `/dev/junctek` link for
this meter's USB-RS485 adapter:

```bash
sudo cp /home/sahabat/sahabat_ws/udev/99-sahabat-robot.rules \
  /etc/udev/rules.d/99-sahabat-robot.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/junctek
```

If the link does not appear, unplug and reconnect only the KG-F USB adapter,
then repeat `ls -l /dev/junctek`. The rule is tied to adapter serial
`5C83118549`, so it does not rename the motor RS485 adapter.

The canonical hardware launches start the read-only battery node by default.
It publishes standard ROS battery data plus detailed meter state:

```bash
ros2 topic echo /battery_state --once
ros2 topic echo /junctek/state --once
ros2 topic echo /diagnostics --once
```

For a battery-only check that does not start motors, sensors, or Nav2:

```bash
source /home/sahabat/sahabat_ws/install/setup.bash
ros2 run shbat_pkg junctek_battery --ros-args \
  --params-file /home/sahabat/sahabat_ws/src/shbat_pkg/config/junctek_battery.yaml
```

Set `use_battery_monitor:=false` on `bringup.launch.py`,
`navigation.launch.py`, or `operations.launch.py` when intentionally running
without the meter.

The desktop **Sahabat JUNCTEK Battery Monitor** provides live voltage, signed
current, power, temperature, remaining Ah, battery percentage, history graphs,
all documented settings, and guarded maintenance actions. Stop the ROS battery
node or robot launch before opening it because the GUI directly owns the
serial port:

```bash
ros2 run shbat_pkg junctek_monitor
```

Reading is automatic. Every setting write requires an explicit confirmation;
factory reset additionally requires typing `RESET`. Confirm protection limits,
capacity, shunt/current ratio, relay type, and calibration against the physical
battery system before changing them.

If `/battery_state` reports zero `charge` and `percentage` while the battery is
not actually empty, the KG-F coulomb counter has not been initialized. Fully
charge the battery, confirm its configured Ah capacity, stop the ROS battery
node, then use **Maintenance → Set remaining percentage → 100%** in the desktop
monitor. Do not initialize it to 100% from voltage alone. The value then tracks
charge and discharge by coulomb counting.

## LIDAR-only diagnostics

Stop other robot launches first so only one process opens the serial port. Find
the stable device path when available:

The preferred entry point is the graphical control panel. It combines port
detection, health checks, STOP/RESET, bounded recovery, optional DTR cycling,
and starting/stopping the motor-free scan diagnostic:

```bash
ros2 run shbat_pkg lidar_control_panel
```

The panel enables **Start Scan** only after the device reports healthy. The
panel rejects a short-lived `OK` during RESET and requires four consecutive
healthy replies before enabling the scan. Log lines identify their source as
`DEVICE`, `ACTION`, `GUIDANCE`, `ROS`, or `ERROR`. The command-line recovery
operations below remain available for headless access and remote
troubleshooting.

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```

Query the RPLIDAR using its stable USB serial identity. This does not start scan
streaming or any ROS nodes:

```bash
ros2 run shbat_pkg lidar_recovery --action check
```

Run a bounded STOP/RESET/health recovery sequence:

```bash
ros2 run shbat_pkg lidar_recovery --action recover --attempts 3
```

The custom FTDI cable may use DTR for motor power or enable control. DTR cycling
is therefore available explicitly, but is not the default because the official
Slamtec SDK says S1/S2 do not use the usual `startMotor()` DTR path:

```bash
ros2 run shbat_pkg lidar_recovery \
  --action recover --attempts 3 --dtr-cycle
```

Start only the RPLIDAR S2 and scan filter after health reports `OK`. The launch
defaults to the stable `/dev/serial/by-id/` path for FTDI serial `A5069RR4`, not
a changing `ttyUSB` number. It does not start the robot motor, IMU, EKF,
joystick, Nav2, or robot state publisher, and it does not respawn the driver:

```bash
ros2 launch shbat_pkg lidar_check.launch.py
```

In another terminal:

```bash
source ~/sahabat_ws/install/setup.bash
ros2 topic echo /scan_raw --once
ros2 topic hz /scan_raw
ros2 topic hz /scan
```

To distinguish a scan-mode problem from a hardware-health problem, retry using
the standard mode:

```bash
ros2 launch shbat_pkg lidar_check.launch.py \
  lidar_port:=/dev/ttyUSB0 scan_mode:=Standard use_filter:=false
```

`Health status 2` is reported by the LIDAR itself. If both modes report it,
stop the launch, disconnect the LIDAR USB/power for at least ten seconds, then
connect it directly to the Jetson rather than through a hub. Also confirm that
no other process has the port open:

```bash
fuser -v /dev/ttyUSB0
```

The recovery tool also prints the 16-bit device error code. On this robot the
S2 has returned `0x0004` even after protocol RESET and optional DTR cycles.
Community S2 troubleshooting material associates code 4 with voltage
protection, while Slamtec's public protocol does not provide the model-specific
error-code mapping. Measure power at the LIDAR connector under load. Slamtec's
S2 specification requires 4.9-5.2 V, up to 1.5 A during startup, and no more
than 150 mV supply ripple. DTR cycling is not a true power cycle because it does
not disconnect USB VBUS.

## Compatibility commands

The commands in `PROJECT_STATUS.md` remain available. The canonical launches
delegate to those established implementations while the migration is tested on
the physical robot.

## Automatic localization recovery

The RViz window opened by `operations.launch.py` includes a
**Localization Recovery** panel. Put the robot somewhere with room to rotate,
ensure no navigation goal is active, then press **Global Relocalize + Rotate**.
It spreads AMCL particles over the full map and rotates at 0.25 rad/s until the
AMCL pose covariance remains good for eight updates. It stops automatically on
success, after 60 seconds, if lidar data becomes stale, or if E-stop activates.

The direct robot gamepad does not require a deadman button. The left stick is
live whenever E-stop is clear. Button 0 still activates E-stop.

Operations initializes AMCL from the waypoint named `dock` in the configured
waypoint file. Place the robot at the dock before launch. If it starts away from
the dock or the lidar points do not align with the displayed map, use the
recovery button. Use `initialize_from_dock:=false` when intentionally starting
somewhere else.
