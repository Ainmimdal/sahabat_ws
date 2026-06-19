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


class OperatorModeManager(Node):
    """Start and stop mapping, localization, or gallery launch processes."""

    def __init__(self) -> None:
        """Create the internal mode-management service."""
        super().__init__('operator_mode_manager')
        self.declare_parameter('maps_directory', '~/sahabat_ws/maps')
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
                self.process = subprocess.Popen(
                    command,
                    start_new_session=True,
                    env=os.environ.copy(),
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
        if mode == 'mapping':
            return [
                'ros2', 'launch', 'shbat_pkg', 'navigation.launch.py',
                'mode:=mapping', 'use_rviz:=false', 'use_foxglove:=false',
                'use_mapping_panel:=false', 'joy_cmd_topic:=cmd_vel_joy',
                'smoothed_cmd_topic:=cmd_vel_nav_smoothed',
                'operator_safety:=true',
            ]
        if mode in ('localization', 'operations'):
            map_stem = self.maps_directory / map_id / 'map'
            if not map_stem.with_suffix('.yaml').exists():
                return None
            if mode == 'operations':
                waypoint_file = self.maps_directory / map_id / 'waypoints.yaml'
                return [
                    'ros2', 'launch', 'shbat_pkg', 'operations.launch.py',
                    f'map_file:={map_stem}', 'use_rviz:=false',
                    'use_foxglove:=false', 'use_waypoint_gui:=false',
                    'joy_cmd_topic:=cmd_vel_joy',
                    'smoothed_cmd_topic:=cmd_vel_nav_smoothed',
                    'recovery_cmd_topic:=cmd_vel_recovery',
                    'operator_safety:=true',
                    f'waypoint_file:={waypoint_file}',
                ]
            return [
                'ros2', 'launch', 'shbat_pkg', 'navigation.launch.py',
                'mode:=localization', f'map_file:={map_stem}',
                'use_rviz:=false', 'use_foxglove:=false',
                'joy_cmd_topic:=cmd_vel_joy',
                'smoothed_cmd_topic:=cmd_vel_nav_smoothed',
                'operator_safety:=true',
            ]
        return None

    def _stop_process(self) -> None:
        """Gracefully stop the complete managed process group."""
        process = self.process
        self.process = None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except ProcessLookupError:
            pass

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
