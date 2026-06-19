#!/usr/bin/env python3
"""Authenticated TLS WebSocket gateway for the optional Android remote."""

import asyncio
import base64
import json
import math
from pathlib import Path
import ssl
import threading
import uuid

from aiohttp import web
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sahabat_interfaces.msg import OperatorStatus, TeleopCommand, Waypoint
from sahabat_interfaces.srv import ControlLease, GetWaypoints, SaveMap
from sahabat_interfaces.srv import SaveWaypoints, SetEmergencyStop, SetMode
from sensor_msgs.msg import LaserScan


class MobileGateway(Node):
    """Expose a deliberately small, authenticated operator protocol."""

    def __init__(self) -> None:
        super().__init__('mobile_gateway')
        self.declare_parameter('address', '0.0.0.0')
        self.declare_parameter('port', 8443)
        self.declare_parameter('certificate', '')
        self.declare_parameter('private_key', '')
        self.declare_parameter('token_file', '')
        self.address = str(self.get_parameter('address').value)
        self.port = int(self.get_parameter('port').value)
        self.certificate = Path(str(self.get_parameter('certificate').value)).expanduser()
        self.private_key = Path(str(self.get_parameter('private_key').value)).expanduser()
        token_file = Path(str(self.get_parameter('token_file').value)).expanduser()
        if not token_file.is_file() or not self.certificate.is_file() or not self.private_key.is_file():
            raise RuntimeError('certificate, private_key, and token_file must all exist')
        self.token = token_file.read_text(encoding='utf-8').strip()
        if len(self.token) < 32:
            raise RuntimeError('Mobile gateway token must contain at least 32 characters')

        self.clients = set()
        self.active_map = ''
        self.current_pose = (0.0, 0.0, 0.0)
        self.loop = asyncio.new_event_loop()
        self.teleop_pub = self.create_publisher(
            TeleopCommand, '/operator/teleop_command', 10
        )
        self.create_subscription(
            OperatorStatus, '/operator/status', self._status, 10
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map, 1
        )
        self.create_subscription(
            LaserScan, '/scan', self._scan, qos_profile_sensor_data
        )
        self.lease_client = self.create_client(
            ControlLease, '/operator/control_lease'
        )
        self.estop_client = self.create_client(
            SetEmergencyStop, '/operator/set_emergency_stop'
        )
        self.mode_client = self.create_client(SetMode, '/operator/set_mode')
        self.save_map_client = self.create_client(SaveMap, '/operator/maps/save')
        self.get_waypoints_client = self.create_client(
            GetWaypoints, '/operator/waypoints/get'
        )
        self.save_waypoints_client = self.create_client(
            SaveWaypoints, '/operator/waypoints/save'
        )

        self.thread = threading.Thread(target=self._run_server, daemon=True)
        self.thread.start()

    def _run_server(self) -> None:
        asyncio.set_event_loop(self.loop)
        application = web.Application(client_max_size=1024 * 1024)
        application.router.add_get('/operator', self._websocket)
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(self.certificate, self.private_key)
        runner = web.AppRunner(application)

        async def start() -> None:
            await runner.setup()
            site = web.TCPSite(
                runner, self.address, self.port, ssl_context=context
            )
            await site.start()

        self.loop.run_until_complete(start())
        self.get_logger().info(f'Mobile WSS listening on {self.address}:{self.port}')
        self.loop.run_forever()

    async def _websocket(self, request):
        supplied = request.headers.get('Authorization', '')
        if supplied != f'Bearer {self.token}':
            raise web.HTTPUnauthorized()
        socket = web.WebSocketResponse(heartbeat=2.0, receive_timeout=6.0)
        await socket.prepare(request)
        self.clients.add(socket)
        try:
            async for incoming in socket:
                if incoming.type == web.WSMsgType.TEXT:
                    await self._command(socket, json.loads(incoming.data))
        finally:
            self.clients.discard(socket)
        return socket

    async def _command(self, socket, data) -> None:
        command = str(data.get('command', ''))
        if command == 'teleop':
            message = TeleopCommand()
            message.header.frame_id = str(data.get('client_id', 'android'))
            message.lease_id = str(data.get('lease_id', ''))
            message.sequence = int(data.get('sequence', 0))
            message.deadman = bool(data.get('deadman', False))
            message.twist.linear.x = max(-0.20, min(0.20, float(data.get('linear', 0))))
            message.twist.angular.z = max(-0.60, min(0.60, float(data.get('angular', 0))))
            self.teleop_pub.publish(message)
            return
        if command in ('acquire', 'renew', 'release'):
            request = ControlLease.Request()
            request.action = {'acquire': 0, 'renew': 1, 'release': 2}[command]
            request.client_id = str(data.get('client_id', 'android'))
            request.lease_id = str(data.get('lease_id', ''))
            self._reply(socket, command, self.lease_client.call_async(request))
        elif command == 'estop':
            request = SetEmergencyStop.Request()
            request.active = bool(data.get('active', True))
            request.lease_id = str(data.get('lease_id', ''))
            request.confirmation = str(data.get('confirmation', 'android'))
            self._reply(socket, command, self.estop_client.call_async(request))
        elif command == 'mode':
            request = SetMode.Request()
            request.mode = str(data.get('mode', 'idle'))
            request.map_id = str(data.get('map_id', ''))
            request.lease_id = str(data.get('lease_id', ''))
            self._reply(socket, command, self.mode_client.call_async(request))
        elif command == 'save_map':
            request = SaveMap.Request()
            request.map_id = str(data.get('map_id', ''))
            request.display_name = str(data.get('display_name', request.map_id))
            request.include_editable_session = bool(data.get('editable', True))
            request.overwrite = bool(data.get('overwrite', False))
            request.lease_id = str(data.get('lease_id', ''))
            self._reply(socket, command, self.save_map_client.call_async(request))
        elif command == 'quick_waypoint':
            request = GetWaypoints.Request()
            request.map_id = self.active_map
            pending = self.get_waypoints_client.call_async(request)
            pending.add_done_callback(
                lambda done: self._quick_waypoint(socket, data, done)
            )
        else:
            await socket.send_json({'type': 'error', 'message': 'Unknown command'})

    def _reply(self, socket, command, future) -> None:
        def finished(done) -> None:
            try:
                response = done.result()
                payload = dict(message_to_ordereddict(response))
                payload.update({'type': 'reply', 'command': command})
            except Exception as error:
                payload = {'type': 'error', 'message': str(error)}
            asyncio.run_coroutine_threadsafe(socket.send_json(payload), self.loop)
        future.add_done_callback(finished)

    def _quick_waypoint(self, socket, data, future) -> None:
        try:
            current = future.result()
            waypoint = Waypoint()
            waypoint.id = uuid.uuid4().hex
            waypoint.name = str(data.get('name', 'Quick waypoint'))[:64]
            waypoint.pose.x, waypoint.pose.y, waypoint.pose.theta = self.current_pose
            waypoint.enabled = True
            request = SaveWaypoints.Request()
            request.map_id = self.active_map
            request.expected_revision = current.revision
            request.waypoints = list(current.waypoints) + [waypoint]
            request.lease_id = str(data.get('lease_id', ''))
            self._reply(
                socket,
                'quick_waypoint',
                self.save_waypoints_client.call_async(request),
            )
        except Exception as error:
            asyncio.run_coroutine_threadsafe(
                socket.send_json({'type': 'error', 'message': str(error)}),
                self.loop,
            )

    def _status(self, message: OperatorStatus) -> None:
        self.active_map = message.active_map
        self.current_pose = (
            message.pose.x, message.pose.y, message.pose.theta
        )
        payload = {
            'type': 'status', 'mode': int(message.mode),
            'active_map': message.active_map,
            'estop': message.emergency_stop,
            'lease_owner': message.control_owner,
            'lease_remaining': message.lease_expires_in,
            'pose': [message.pose.x, message.pose.y, message.pose.theta],
            'velocity': [message.linear_velocity, message.angular_velocity],
            'diagnostic': message.diagnostic_message,
        }
        self._broadcast(payload)

    def _map(self, message: OccupancyGrid) -> None:
        encoded = base64.b64encode(
            bytes((value + 1) & 0xff for value in message.data)
        ).decode('ascii')
        self._broadcast({
            'type': 'map', 'width': message.info.width,
            'height': message.info.height, 'resolution': message.info.resolution,
            'origin': [message.info.origin.position.x, message.info.origin.position.y],
            'cells': encoded,
        })

    def _scan(self, message: LaserScan) -> None:
        step = max(1, len(message.ranges) // 180)
        self._broadcast({
            'type': 'scan', 'angle_min': message.angle_min,
            'angle_increment': message.angle_increment * step,
            'ranges': [
                round(float(value), 3) if math.isfinite(value) else 0.0
                for value in message.ranges[::step]
            ],
        })

    def _broadcast(self, payload) -> None:
        for socket in tuple(self.clients):
            asyncio.run_coroutine_threadsafe(
                socket.send_json(payload), self.loop
            )

    def destroy_node(self):
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MobileGateway()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
