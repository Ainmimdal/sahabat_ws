#!/usr/bin/env python3
"""Manage one navigation-mode launch process behind a ROS service."""

import json
import os
from pathlib import Path
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sahabat_interfaces.srv import SetMode
from std_msgs.msg import String


def _map_exists(maps_directory: Path, map_id: str) -> bool:
    if not map_id or not all(ch.isalnum() or ch in '_-' for ch in map_id):
        return False
    stem = maps_directory / map_id
    return stem.with_suffix('.yaml').exists() or (stem / 'map.yaml').exists()


def full_stack_command(mode: str, map_id: str, maps_directory: Path):
    """Commands for the 'full' profile used by robot.launch.py.

    Each mode starts the complete, physically proven stack (hardware drivers
    included) exactly as the desktop shortcuts do: mapping matches
    "Sahabat New Mapping" and operations matches "Sahabat Waypoint Editor
    Live", both without RViz. Switching mode therefore restarts the drivers.
    """
    if mode == 'mapping':
        return [
            'ros2', 'launch', 'shbat_pkg', 'navigation.launch.py',
            'mode:=mapping', 'use_rviz:=false', 'use_mapping_panel:=false',
            'use_zed:=false',
        ]
    if mode == 'operations' and _map_exists(maps_directory, map_id):
        return [
            'ros2', 'launch', 'shbat_pkg', 'operations.launch.py',
            f'map_file:={maps_directory / map_id}',
            f'maps_directory:={maps_directory}',
            f'map_id:={map_id}',
            'use_rviz:=false',
            'use_waypoint_gui:=false',
            'use_api:=true',
            'use_zed:=true',
            'localization_backend:=amcl',
        ]
    return None


class OperatorModeManager(Node):
    """Start and stop mapping, localization, or gallery launch processes."""

    def __init__(self) -> None:
        """Create the internal mode-management service."""
        super().__init__('operator_mode_manager')
        self.declare_parameter('maps_directory', '~/sahabat_ws/maps')
        # 'core': navigation layers on top of remote_operations' persistent
        # hardware bringup (Foxglove setup). 'full': complete stacks,
        # including hardware, as started by robot.launch.py.
        self.declare_parameter('stack', 'core')
        self.declare_parameter('initial_mode', 'idle')
        self.declare_parameter('initial_map', '')
        self.stack = str(self.get_parameter('stack').value)
        self.maps_directory = Path(
            str(self.get_parameter('maps_directory').value)
        ).expanduser()
        self.process = None
        self.mode = 'idle'
        self.map_id = ''
        self.lock = threading.Lock()
        transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.state_pub = self.create_publisher(
            String,
            '/operator/mode_state',
            transient,
        )
        self.create_service(
            SetMode,
            '/operator/internal/set_mode',
            self._set_mode,
        )
        self.create_timer(1.0, self._monitor)
        self._publish_state('ready')
        initial_mode = str(self.get_parameter('initial_mode').value)
        if initial_mode != 'idle':
            request = SetMode.Request()
            request.mode = initial_mode
            request.map_id = str(self.get_parameter('initial_map').value)
            result = self._set_mode(request, SetMode.Response())
            self.get_logger().info(f'Initial mode: {result.message}')

    def _set_mode(self, request, response):
        """Replace the managed process with the requested mode."""
        with self.lock:
            self._stop_process()
            if request.mode == 'idle':
                self.mode = 'idle'
                self.map_id = ''
                self._publish_state('idle')
                response.success = True
                response.message = 'Navigation stack stopped; core remains active'
                return response

            command = self._command(request.mode, request.map_id)
            if command is None:
                response.message = 'Invalid mode or map'
                return response
            try:
                child_environment = os.environ.copy()
                if self.stack == 'full':
                    # Keep the unauthenticated SahaBot API on loopback.
                    child_environment['API_HOST'] = '127.0.0.1'
                else:
                    child_environment['SAHABAT_SKIP_DEVICE_DETECTION'] = '1'
                self.process = subprocess.Popen(
                    command,
                    start_new_session=True,
                    env=child_environment,
                )
            except OSError as error:
                response.message = f'Could not start mode: {error}'
                self.process = None
                return response
            self.mode = request.mode
            self.map_id = request.map_id
            self._publish_state('starting')
            response.success = True
            response.message = f'Starting {request.mode}'
            return response

    def _command(self, mode: str, map_id: str):
        """Build the established launch command for one requested mode."""
        if self.stack == 'full':
            command = full_stack_command(mode, map_id, self.maps_directory)
            if command is not None and mode == 'operations':
                try:
                    (self.maps_directory / 'last_selected_map').write_text(
                        f'{map_id}\n', encoding='utf-8'
                    )
                except OSError:
                    pass
            return command
        if mode == 'mapping':
            return [
                'ros2', 'launch', 'shbat_pkg', 'navigation.launch.py',
                'mode:=mapping', 'use_rviz:=false', 'use_foxglove:=false',
                'use_mapping_panel:=false', 'joy_cmd_topic:=cmd_vel_joy',
                'smoothed_cmd_topic:=cmd_vel_nav_smoothed',
                'use_command_arbiter:=false',
                'operator_safety:=true',
                'use_hardware:=false',
            ]
        if mode in ('localization', 'operations'):
            map_stem = self.maps_directory / map_id
            if not (
                map_stem.with_suffix('.yaml').exists()
                or (map_stem / 'map.yaml').exists()
            ):
                return None
            if mode == 'operations':
                waypoint_file = self.maps_directory / f'{map_id}_waypoints.yaml'
                try:
                    (self.maps_directory / 'last_selected_map').write_text(
                        f'{map_id}\n', encoding='utf-8'
                    )
                except OSError:
                    pass
                return [
                    'ros2', 'launch', 'shbat_pkg', 'operations.launch.py',
                    f'map_file:={map_stem}', 'use_rviz:=false',
                    'use_foxglove:=false', 'use_waypoint_gui:=false',
                    'joy_cmd_topic:=cmd_vel_joy',
                    'smoothed_cmd_topic:=cmd_vel_nav_smoothed',
                    'recovery_cmd_topic:=cmd_vel_recovery',
                    'use_command_arbiter:=false',
                    'operator_safety:=true',
                    'use_hardware:=false',
                    f'waypoint_file:={waypoint_file}',
                ]
            return [
                'ros2', 'launch', 'shbat_pkg', 'navigation.launch.py',
                'mode:=localization', f'map_file:={map_stem}',
                'use_rviz:=false', 'use_foxglove:=false',
                'joy_cmd_topic:=cmd_vel_joy',
                'smoothed_cmd_topic:=cmd_vel_nav_smoothed',
                'use_command_arbiter:=false',
                'operator_safety:=true',
                'use_hardware:=false',
            ]
        return None

    def _stop_process(self) -> None:
        """Gracefully stop the complete managed process group."""
        process = self.process
        self.process = None
        if process is None or process.poll() is not None:
            return
        # SIGINT lets ros2 launch stop its nodes cleanly (base_controller
        # disables the motors in destroy_node); escalate only if it hangs.
        for sig, timeout in ((signal.SIGINT, 20.0), (signal.SIGTERM, 5.0),
                             (signal.SIGKILL, 5.0)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    f'Mode process ignored {sig.name}; escalating')

    def _monitor(self) -> None:
        """Report an unexpected child-process exit."""
        with self.lock:
            if self.process is None:
                return
            result = self.process.poll()
            if result is None:
                self._publish_state('running')
                return
            self.process = None
            previous = self.mode
            self.mode = 'idle'
            self._publish_state(f'{previous} exited with code {result}')

    def _publish_state(self, detail: str) -> None:
        """Publish current mode as a latched JSON status."""
        message = {
            'mode': self.mode,
            'map_id': self.map_id,
            'detail': detail,
        }
        self.state_pub.publish(String(data=json.dumps(message)))

    def destroy_node(self):
        """Stop the child before the persistent manager exits."""
        with self.lock:
            self._stop_process()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the navigation mode manager."""
    rclpy.init(args=args)
    node = OperatorModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
