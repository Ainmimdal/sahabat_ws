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

Gallery operation with the waypoint GUI and external API:

```bash
ros2 launch shbat_pkg operations.launch.py \
  map_file:=/home/sahabat/sahabat_ws/maps/gallery_map \
  use_api:=true
```

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

After changing Nav2 speed or controller tuning, use a clear straight test lane
with a person beside the emergency stop. Start with a short goal at 0.2 m/s,
then repeat at the configured 0.3 m/s cruise before allowing the 0.5 m/s
command cap. Test a wide 90-degree turn separately and confirm the lidar scan
stays aligned with walls. Stop the test if wheel odometry jumps, the scan
smears, the controller oscillates, or stopping distance is unsafe. Keep angular
velocity at or below
0.5 rad/s until a higher rate passes a logged lidar/localization test.
During the turn test, small heading corrections should form a continuous arc;
larger initial heading errors may cause an in-place pivot. Stop and
retune if navigation repeatedly alternates between pivoting, creeping forward,
and braking.

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
