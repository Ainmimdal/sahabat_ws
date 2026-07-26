# Sahabat Foxglove and Mobile Operations

Foxglove Desktop is the primary remote interface. The Android application is
optional. Both use the same robot-side lease, map, waypoint and E-stop services.
The software E-stop is not safety-rated: gallery operation still requires a
driver and a spotter beside the robot.

## Install and build

```bash
cd ~/sahabat_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sahabat_interfaces shbat_rviz_plugins shbat_pkg --symlink-install
source install/setup.bash

cd foxglove/sahabat-operator
npm install
npm run build
npm run local-install
```

Do not open the `.foxe` package with Foxglove's **Open data source** command.
Until this extension is published in Foxglove's registry, clone the repository
and use `npm run local-install` to install it into Foxglove Desktop.

Restart Foxglove Desktop after installing the extension. Import the four JSON
layouts in `foxglove/layouts/`: **Operate**, **Mapping**, **Waypoints**, and
**Diagnostics**.

## Safe robot startup

```bash
ros2 launch shbat_pkg remote_operations.launch.py
```

This starts hardware bringup once, plus the persistent operator backend,
command arbiter, restricted Foxglove bridge and mode manager. Manual driving is
available in Idle after taking control. Mapping and localization add only their
navigation layers, so switching modes does not restart the motor, lidar or
odometry nodes.

Connect Foxglove Desktop to `ws://ROBOT_IP:8765`, add **Sahabat Operator**, and
press **Take control**. Confirm the spotter is ready and the robot is stationary
before starting mapping.

The panel uses three columns: Drive, Maps, and Routes. Health stays in the thin
status strip, and localization recovery is one compact row inside Drive. It
collapses to one column only when the panel is very narrow.
Keyboard driving uses `W/A/S/D` directly. Gamepad driving requires selecting a
controller but has no deadman button; moving the left stick commands motion and
returning it to neutral commands zero. Selecting a different input mode, losing
focus, losing the gamepad, losing the lease, or stopping teleop messages also
commands zero. The panel provides independent speed sliders up to 0.50 m/s and
1.20 rad/s.

Open a saved map from **Maps** to start complete operation mode. This loads
localization, the map-level dock pose and the active named waypoint set. If the
lidar overlay does not match the map, select **Global relocalize + rotate**
directly below Drive. The robot rotates at 0.25 rad/s until AMCL covariance is
stable, or stops on E-stop, stale lidar, operator cancellation, or timeout.

Maps are stored as:

```text
maps/<map_id>/
  map.yaml
  map.pgm
  metadata.yaml
  session.posegraph       # optional editable SLAM session
  session.data            # optional editable SLAM session
  dock.yaml
  waypoint_sets/
    index.yaml            # active set
    default.yaml
    <set_id>.yaml
  waypoints.yaml          # retained legacy source after first migration
```

The first access copies an existing `waypoints.yaml` into the **Default** set,
moves its named `dock` into map-level storage, and leaves the original file
unchanged. Foxglove and RViz then select and edit the same sets. Deleting a set
archives its file, and each set has an independent revision check so a stale
editor cannot silently overwrite it. Map overwrites still move the previous
map directory under `maps/.archive/`.

Selecting a set loads it immediately; **Save changes** writes edits to that
selected set and does not require restarting either UI. The shared map dock is
available through **Go to dock** from every set.

## Optional Android gateway

Generate a private-router TLS certificate and a random token outside source
control. Start the gateway only after firewalling port 8443 to the phone:

```bash
ros2 launch shbat_pkg remote_operations.launch.py \
  mobile_gateway:=true \
  mobile_certificate:=/etc/sahabat/mobile.crt \
  mobile_private_key:=/etc/sahabat/mobile.key \
  mobile_token_file:=/etc/sahabat/mobile.token
```

Open `android/SahabatRemote` in Android Studio, set the robot host and SHA-256
certificate pin in `app/build.gradle.kts`, provision the matching token through
the app, then build/sign it. The phone supports live occupancy map, robot pose,
lidar, mapping mode, named save/load, quick waypoint capture, E-stop and a USB-C
gamepad. Backgrounding, screen lock, controller removal or stale/network input
stops remote commands within the 250 ms teleop timeout without latching E-stop.

## Network boundary

Keep the Jetson wired to the robot router and allow TCP 8765 only from the
authorized laptop and TCP 8443 only from the authorized phone. Foxglove
parameter access is disabled, service access is restricted to `/operator/*`,
and client publication is limited to operator candidate/teleop topics. Never
port-forward either service to a public or guest network.

## Physical acceptance

Perform these with the driver holding the lease and the spotter beside the
physical E-stop:

1. Map and save once with WASD, once with the laptop gamepad, and once
   with the optional Android controller.
2. Confirm keyboard and gamepad cannot command at the same time.
3. While moving slowly, test focus loss, gamepad removal and Wi-Fi loss; verify
   zero velocity within 500 ms.
4. Save/load a named map, create and switch between two waypoint sets, then
   edit/click-add/reorder, navigate and patrol with no Jetson HDMI peripherals.
5. Open the same map in the RViz Waypoint Manager and confirm it shows the same
   active set, waypoint order and map-level dock.
6. Confirm map/scan/TF health and all E-stop paths before public operation.

Collision Monitor is intentionally bypassed for remote manual operation. The
operator is responsible for obstacle clearance while driving.
