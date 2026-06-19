# Sahabat Foxglove extension

The extension provides four focused views: Drive, Maps, Routes, and Health.
It includes lease-aware WASD/gamepad teleoperation, named map management,
map-specific dock and waypoint editing, patrol controls, diagnostics, and
automatic AMCL global relocalization. The robot-side bridge must be started
with `remote_operations.launch.py`.

```bash
npm install
npm run build
npm run local-install
```

Restart Foxglove Desktop, add **Sahabat Operator**, and connect to
`ws://ROBOT_IP:8765`. Network firewalling is still required: the bridge itself
does not authenticate clients. Do not expose it beyond the robot's private
router.

Import the layouts from `foxglove/layouts`. Each layout opens the operator
panel on the relevant view. Opening a saved map uses full **Operate** mode so
localization recovery, map-specific routes, and dock initialization are
available together.

The `.foxe` file is a distribution archive and is not opened as a data file in
Foxglove. For an unpublished local extension, use the commands above; the
installer places the built extension in Foxglove Desktop's local extension
directory.
