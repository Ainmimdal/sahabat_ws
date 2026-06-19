# Sahabat Foxglove extension

The extension provides the lease-aware operator console, keyboard/gamepad
teleoperation, named map manager and waypoint editor. The robot-side bridge
must be started with `remote_operations.launch.py`.

```bash
npm install
npm run build
npm run local-install
```

Restart Foxglove Desktop, add **Sahabat Operator**, and connect to
`ws://ROBOT_IP:8765`. Network firewalling is still required: the bridge itself
does not authenticate clients. Do not expose it beyond the robot's private
router.
