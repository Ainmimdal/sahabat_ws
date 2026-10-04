#!/usr/bin/env python3
"""Browser operator console: map, waypoints, localization, teleop, E-stop.

A single-process web replacement for the live RViz waypoint editor. It is a
thin client of ``operator_backend``: every motion or file change goes through
the existing ``/operator/...`` services and the backend control lease, so the
backend's safety checks (lease, E-stop latch, teleop timeout) still apply.

Transport is plain HTTP from the Python standard library:
  GET  /                 static UI (share/shbat_pkg/web)
  GET  /api/stream       Server-Sent Events with robot state
  GET  /api/map.png      current /map as a PNG
  GET  /api/camera.mjpg  ZED left image with AprilTag outlines (MJPEG)
  POST /api/cmd          JSON command {"op": ..., "client": ...}

WARNING: there is no authentication. Bind to a trusted network only.
"""

import collections
import json
import re
import shutil
import math
import queue
import struct
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import zlib

from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sahabat_interfaces.msg import (
    AprilTagLandmarkArray,
    OperatorStatus,
    RouteSegment,
    TeleopCommand,
    Waypoint,
)
from sahabat_interfaces.srv import (
    ControlLease,
    GetWaypointGraph,
    ListMaps,
    ListWaypointSets,
    LocalizationRecovery,
    ManageAprilTagLandmark,
    ManageWaypointSet,
    PatrolCommand,
    SaveDock,
    SaveWaypointGraph,
    SetEmergencyStop,
    SetMode,
)
from sensor_msgs.msg import Image, LaserScan
from slam_toolbox.srv import SaveMap as SlamSaveMap
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

MODE_NAMES = {
    OperatorStatus.MODE_IDLE: 'idle',
    OperatorStatus.MODE_MAPPING: 'mapping',
    OperatorStatus.MODE_LOCALIZATION: 'localization',
    OperatorStatus.MODE_OPERATIONS: 'operations',
}
DIAG_NAMES = {0: 'ok', 1: 'warn', 2: 'error'}
VALID_MAP_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
MAP_FILE_SUFFIXES = ('.yaml', '.pgm', '.posegraph', '.data')
HEARTBEAT_TIMEOUT = 3.0
MAX_SCAN_POINTS = 720


def yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def planar_projection(transform):
    """Project a 3D transform onto the target XY plane.

    Returns (r00, r01, r10, r11, tx, ty) so that a point (x, y, 0) in the
    source frame lands at (r00*x + r01*y + tx, r10*x + r11*y + ty). Using the
    full rotation matters for frames that are rolled, such as the
    upside-down lidar (rpy 3.14 0 3.14): yaw alone mirrors the scan.
    """
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    return (
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
        2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
        transform.translation.x, transform.translation.y,
    )


def write_map_files(grid, stem: Path) -> list:
    """Write <stem>.pgm and <stem>.yaml exactly like nav2 map_saver (trinary).

    Cells <= 25 are free (254), >= 65 occupied (0), the rest and -1 unknown
    (205); row 0 of the PGM is the top (maximum y) of the map.
    """
    info = grid.info
    width, height = info.width, info.height
    lut = bytearray(256)
    for value in range(256):
        signed = value - 256 if value > 127 else value
        if signed < 0:
            lut[value] = 205
        elif signed <= 25:
            lut[value] = 254
        elif signed >= 65:
            lut[value] = 0
        else:
            lut[value] = 205
    raw = grid_bytes(grid.data).translate(bytes(lut))
    rows = [raw[row * width:(row + 1) * width]
            for row in range(height - 1, -1, -1)]
    pgm = f'P5\n{width} {height}\n255\n'.encode() + b''.join(rows)
    yaw = yaw_of(info.origin.orientation)
    yaml_text = (
        f'image: {stem.name}.pgm\n'
        'mode: trinary\n'
        f'resolution: {info.resolution:.3g}\n'
        f'origin: [{info.origin.position.x:.6g}, {info.origin.position.y:.6g}, '
        f'{yaw:.6g}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )
    written = []
    for suffix, data in (('.pgm', pgm), ('.yaml', yaml_text.encode())):
        target = stem.with_suffix(suffix)
        partial = target.with_name(f'.{target.name}.partial')
        partial.write_bytes(data)
        partial.replace(target)
        written.append(target)
    return written


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def grid_bytes(data) -> bytes:
    """OccupancyGrid data as unsigned bytes (fast path for rclpy arrays)."""
    try:
        return data.tobytes()
    except AttributeError:
        return bytes(v & 0xFF for v in data)


def encode_map_png(width: int, height: int, data) -> bytes:
    """Encode an OccupancyGrid as an RGBA PNG (row 0 = top = max y)."""
    free = b'\xf4\xf5\xf7\xff'
    unknown = b'\x00\x00\x00\x00'
    lut = []
    for value in range(256):
        signed = value - 256 if value > 127 else value
        if signed < 0:
            lut.append(unknown)
        elif signed >= 65:
            lut.append(b'\x1f\x29\x37\xff')
        elif signed <= 25:
            lut.append(free)
        else:
            shade = int(244 - (signed - 25) * 4)
            lut.append(bytes((shade, shade, shade + 2, 255)))
    raw = grid_bytes(data)
    rows = []
    for row in range(height - 1, -1, -1):
        start = row * width
        line = raw[start:start + width]
        rows.append(b'\x00' + b''.join(lut[v] for v in line))

    def chunk(tag, body):
        out = struct.pack('>I', len(body)) + tag + body
        return out + struct.pack('>I', zlib.crc32(tag + body) & 0xFFFFFFFF)

    header = struct.pack('>IIBBBBB', width, height, 8, 6, 0, 0, 0)
    return (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', header)
        + chunk(b'IDAT', zlib.compress(b''.join(rows), 6))
        + chunk(b'IEND', b'')
    )


class Hub:
    """Fan-out of JSON events to SSE clients, with a replayable snapshot."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = set()
        self.snapshot = {}

    def publish(self, event: str, payload, keep: bool = True) -> None:
        text = f'event: {event}\ndata: {json.dumps(payload, separators=(",", ":"))}\n\n'
        with self.lock:
            if keep:
                self.snapshot[event] = text
            clients = list(self.clients)
        for client in clients:
            try:
                client.put_nowait(text)
            except queue.Full:
                pass

    def subscribe(self):
        client = queue.Queue(maxsize=64)
        with self.lock:
            for text in self.snapshot.values():
                client.put_nowait(text)
            self.clients.add(client)
        return client

    def unsubscribe(self, client) -> None:
        with self.lock:
            self.clients.discard(client)


class CameraFeed:
    """Lazily encoded MJPEG frames with AprilTag detection outlines.

    The image subscription only exists while at least one browser is
    watching, so the console costs nothing on the camera path otherwise.
    """

    def __init__(self, node: Node, topic: str, fps: float, width: int):
        self.node = node
        self.topic = topic
        self.period = 1.0 / max(0.5, fps)
        self.max_width = width
        self.cond = threading.Condition()
        self.new_image = threading.Event()
        self.encode_ms = 0.0
        self.viewers = 0
        self.subscription = None
        self.latest = None
        self.latest_at = 0.0
        self.jpeg = b''
        self.sequence = 0
        # Recent detection arrays as (stamp_ns, received_at, tags); the
        # detector lags the image, so frames are matched by header stamp.
        self.detections = collections.deque(maxlen=30)
        self.image_count = 0
        self.rate_started = time.monotonic()
        self.image_hz = 0.0
        node.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections',
            self._detections, qos_profile_sensor_data)
        threading.Thread(target=self._encoder, daemon=True).start()

    def _detections(self, message) -> None:
        tags = [(
            d.id,
            [(c.x, c.y) for c in d.corners],
            d.decision_margin,
        ) for d in message.detections]
        self.detections.append(
            (stamp_ns(message.header.stamp), time.monotonic(), tags))

    def _tags_for(self, image_stamp: int):
        """Detections computed from this frame, else the closest earlier ones."""
        history = list(self.detections)
        if image_stamp:
            best = None
            for stamp, _received, tags in history:
                if stamp == image_stamp:
                    return tags
                if stamp and stamp <= image_stamp and image_stamp - stamp < 300_000_000:
                    if best is None or stamp > best[0]:
                        best = (stamp, tags)
            if best is not None:
                return best[1]
            if any(stamp for stamp, _r, _t in history):
                return []
        # Unstamped sources: fall back to anything received recently.
        if history and time.monotonic() - history[-1][1] < 0.5:
            return history[-1][2]
        return []

    def _image(self, message: Image) -> None:
        self.latest = message
        self.latest_at = time.monotonic()
        self.image_count += 1
        self.new_image.set()

    def attach(self) -> None:
        with self.cond:
            self.viewers += 1
            if self.subscription is None:
                self.subscription = self.node.create_subscription(
                    Image, self.topic, self._image, qos_profile_sensor_data)
                self.rate_started = time.monotonic()
                self.image_count = 0
            self.cond.notify_all()

    def detach(self) -> None:
        with self.cond:
            self.viewers = max(0, self.viewers - 1)
            if self.viewers == 0 and self.subscription is not None:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None
                self.latest = None

    def wait_frame(self, last_sequence: int, timeout: float):
        with self.cond:
            self.cond.wait_for(
                lambda: self.sequence != last_sequence, timeout=timeout)
            return self.sequence, self.jpeg

    def _encoder(self) -> None:
        try:
            from PIL import Image as PilImage, ImageDraw, ImageFont
        except ImportError:
            self.node.get_logger().warning('python3-pil missing; camera view disabled')
            return
        try:
            self.font = ImageFont.truetype(
                '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 16)
        except OSError:
            self.font = ImageFont.load_default()
        while rclpy.ok():
            with self.cond:
                self.cond.wait_for(lambda: self.viewers > 0, timeout=1.0)
                if self.viewers == 0:
                    continue
            # Encode each new camera frame (capped at camera_fps); fall back
            # to a placeholder once a second when no image is arriving.
            self.new_image.wait(timeout=1.0)
            self.new_image.clear()
            started = time.monotonic()
            elapsed = started - self.rate_started
            if elapsed >= 2.0:
                self.image_hz = self.image_count / elapsed
                self.image_count = 0
                self.rate_started = started
            message = self.latest
            if message is None or started - self.latest_at > 2.0:
                frame = self._placeholder(PilImage, ImageDraw)
            else:
                try:
                    frame = self._render(message, PilImage, ImageDraw)
                    self.encode_ms = 0.8 * self.encode_ms + 200.0 * (
                        time.monotonic() - started)
                except ValueError as error:
                    frame = self._placeholder(PilImage, ImageDraw, str(error))
            with self.cond:
                self.jpeg = frame
                self.sequence += 1
                self.cond.notify_all()
            # Cap at camera_fps without skipping frames that arrive on time.
            time.sleep(max(0.0, 0.8 * self.period - (time.monotonic() - started)))

    def _render(self, message: Image, PilImage, ImageDraw) -> bytes:
        size = (message.width, message.height)
        data = bytes(message.data)
        encoding = message.encoding.lower()
        # Decode straight to RGB (alpha dropped by the unpacker): one pass.
        if encoding == 'bgra8':
            image = PilImage.frombuffer('RGB', size, data, 'raw', 'BGRX', message.step, 1)
        elif encoding == 'rgba8':
            image = PilImage.frombuffer('RGB', size, data, 'raw', 'RGBX', message.step, 1)
        elif encoding == 'bgr8':
            image = PilImage.frombuffer('RGB', size, data, 'raw', 'BGR', message.step, 1)
        elif encoding == 'rgb8':
            image = PilImage.frombuffer('RGB', size, data, 'raw', 'RGB', message.step, 1)
        elif encoding == 'mono8':
            image = PilImage.frombuffer('L', size, data, 'raw', 'L', message.step, 1)
        else:
            raise ValueError(f'Unsupported encoding {message.encoding}')
        if image.mode != 'RGB':
            image = image.convert('RGB')
        scale = min(1.0, self.max_width / float(message.width))
        if scale < 1.0:
            image = image.resize(
                (int(message.width * scale), int(message.height * scale)),
                PilImage.BILINEAR)
        draw = ImageDraw.Draw(image)
        tags = self._tags_for(stamp_ns(message.header.stamp))
        for tag_id, corners, margin in tags:
            points = [(x * scale, y * scale) for x, y in corners]
            draw.line(points + points[:1], fill=(34, 197, 94), width=3)
            draw.ellipse([points[0][0] - 4, points[0][1] - 4,
                          points[0][0] + 4, points[0][1] + 4], fill=(239, 68, 68))
            cx = sum(p[0] for p in points) / 4.0
            cy = sum(p[1] for p in points) / 4.0
            label = f'ID {tag_id}  margin {margin:.0f}'
            box = draw.textbbox((cx, cy), label, font=self.font, anchor='mm')
            draw.rectangle([box[0] - 4, box[1] - 3, box[2] + 4, box[3] + 3],
                           fill=(0, 0, 0))
            draw.text((cx, cy), label, fill=(255, 255, 255),
                      font=self.font, anchor='mm')
        status = (f'camera {self.image_hz:.1f} fps  {message.width}x{message.height}  '
                  f'encode {self.encode_ms:.0f} ms  tags: {len(tags)}')
        box = draw.textbbox((6, 4), status, font=self.font)
        draw.rectangle([0, 0, box[2] + 6, box[3] + 4], fill=(0, 0, 0))
        draw.text((6, 4), status, fill=(255, 255, 255), font=self.font)
        return self._jpeg(image)

    def _placeholder(self, PilImage, ImageDraw, text: str = '') -> bytes:
        image = PilImage.new('RGB', (640, 360), (24, 28, 35))
        draw = ImageDraw.Draw(image)
        draw.text((20, 170), text or f'Waiting for camera image on {self.topic}',
                  fill=(200, 205, 215), font=self.font)
        return self._jpeg(image)

    @staticmethod
    def _jpeg(image) -> bytes:
        import io
        buffer = io.BytesIO()
        image.save(buffer, format='JPEG', quality=72)
        return buffer.getvalue()


class WebConsole(Node):

    def __init__(self, hub: Hub):
        super().__init__('web_console')
        self.hub = hub
        self.group = ReentrantCallbackGroup()
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8088)
        self.declare_parameter('web_root', '')
        self.declare_parameter('scan_rate', 5.0)
        self.declare_parameter('maps_directory', '~/sahabat_ws/maps')
        self.declare_parameter(
            'camera_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('camera_fps', 15.0)
        self.declare_parameter('camera_width', 960)

        self.lock = threading.Lock()
        self.lease_id = ''
        self.lease_client = ''
        self.lease_client_name = ''
        self.last_heartbeat = 0.0
        self.teleop_sequence = 0
        self.active_map = ''
        self.graph = {'revision': 0, 'set_id': '', 'segments': []}
        self.map_png = b''
        self.latest_map = None
        self.map_version = 0
        self.last_scan_sent = 0.0
        self.last_plan_sent = 0.0
        self.scan_period = 1.0 / max(0.5, float(self.get_parameter('scan_rate').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.teleop_pub = self.create_publisher(
            TeleopCommand, '/operator/teleop_command', 10)

        sub = self.create_subscription
        sub(OccupancyGrid, '/map', self._map, latched, callback_group=self.group)
        sub(LaserScan, '/scan', self._scan, qos_profile_sensor_data,
            callback_group=self.group)
        sub(NavPath, '/plan', self._plan, 10, callback_group=self.group)
        sub(OperatorStatus, '/operator/status', self._status, latched,
            callback_group=self.group)
        sub(AprilTagLandmarkArray, '/apriltag_landmarks/state', self._tags, 10,
            callback_group=self.group)
        sub(String, '/localization/startup_status', self._startup, latched,
            callback_group=self.group)
        sub(String, '/operator/mode_state', self._mode_state_message, latched,
            callback_group=self.group)
        sub(String, '/operator/waypoints_changed',
            lambda _m: self._schedule_graph_refresh(), latched,
            callback_group=self.group)

        client = self.create_client
        self.services_ = {
            'lease': client(ControlLease, '/operator/control_lease',
                            callback_group=self.group),
            'estop': client(SetEmergencyStop, '/operator/set_emergency_stop',
                            callback_group=self.group),
            'maps': client(ListMaps, '/operator/maps/list',
                           callback_group=self.group),
            'set_mode': client(SetMode, '/operator/set_mode',
                               callback_group=self.group),
            'mode_manager': client(SetMode, '/operator/internal/set_mode',
                                   callback_group=self.group),
            'sets': client(ListWaypointSets, '/operator/waypoint_sets/list',
                           callback_group=self.group),
            'set_manage': client(ManageWaypointSet,
                                 '/operator/waypoint_sets/manage',
                                 callback_group=self.group),
            'graph_get': client(GetWaypointGraph,
                                '/operator/waypoint_graph/get',
                                callback_group=self.group),
            'graph_save': client(SaveWaypointGraph,
                                 '/operator/waypoint_graph/save',
                                 callback_group=self.group),
            'dock': client(SaveDock, '/operator/dock/save',
                           callback_group=self.group),
            'patrol': client(PatrolCommand, '/operator/patrol',
                             callback_group=self.group),
            'recovery': client(LocalizationRecovery,
                               '/operator/localization/recovery',
                               callback_group=self.group),
            'tags': client(ManageAprilTagLandmark, '/apriltag_landmarks/manage',
                           callback_group=self.group),
            'tags_localize': client(Trigger, '/localization/set_from_tags',
                                    callback_group=self.group),
        }

        self.camera = CameraFeed(
            self,
            str(self.get_parameter('camera_topic').value),
            float(self.get_parameter('camera_fps').value),
            int(self.get_parameter('camera_width').value),
        )

        self.maps_directory = Path(
            str(self.get_parameter('maps_directory').value)).expanduser()
        self.slam_save = self.create_client(
            SlamSaveMap, '/slam_toolbox/save_map', callback_group=self.group)
        self.slam_serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map',
            callback_group=self.group)
        self.mapping_active = None
        self.map_crc = 0
        self.map_saved_crc = 0
        self.mode_state = {'mode': '', 'map_id': '', 'detail': ''}
        self.managed = False
        self.switching = ''
        self.map_saving = ''
        self.create_timer(2.0, self._mapping_timer, callback_group=self.group)

        self.graph_refresh_pending = threading.Event()
        self.create_timer(1.0, self._lease_timer, callback_group=self.group)
        self.create_timer(0.5, self._graph_timer, callback_group=self.group)
        self._publish_lease()

    # ------------------------------------------------------------------ ROS in
    def _map(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        self.map_crc = zlib.crc32(grid_bytes(message.data))
        info = message.info
        png = encode_map_png(info.width, info.height, message.data)
        with self.lock:
            self.map_png = png
            self.map_version += 1
            version = self.map_version
        self.hub.publish('map', {
            'version': version,
            'frame': message.header.frame_id or 'map',
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin_x': info.origin.position.x,
            'origin_y': info.origin.position.y,
            'origin_yaw': yaw_of(info.origin.orientation),
        })

    def _transform_2d(self, target: str, source: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time()).transform
        except Exception:
            return None
        return (tf.translation.x, tf.translation.y, yaw_of(tf.rotation))

    def _scan_transform(self, target: str, message: LaserScan):
        """Full transform at the scan time (like RViz), else the latest."""
        source = message.header.frame_id
        for when in (rclpy.time.Time.from_msg(message.header.stamp),
                     rclpy.time.Time()):
            try:
                return self.tf_buffer.lookup_transform(
                    target, source, when).transform
            except Exception:
                continue
        return None

    def _scan(self, message: LaserScan) -> None:
        now = time.monotonic()
        if now - self.last_scan_sent < self.scan_period or not self.hub.clients:
            return
        self.last_scan_sent = now
        frame = 'map'
        transform = self._scan_transform('map', message)
        if transform is None:
            frame = 'odom'
            transform = self._scan_transform('odom', message)
        if transform is None:
            return
        r00, r01, r10, r11, tx, ty = planar_projection(transform)
        ranges = message.ranges
        step = max(1, len(ranges) // MAX_SCAN_POINTS)
        points = []
        for index in range(0, len(ranges), step):
            r = ranges[index]
            if not math.isfinite(r) or r < message.range_min or r > message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            lx, ly = r * math.cos(angle), r * math.sin(angle)
            points.append(round(r00 * lx + r01 * ly + tx, 3))
            points.append(round(r10 * lx + r11 * ly + ty, 3))
        self.hub.publish('scan', {'frame': frame, 'points': points})

    def _plan(self, message: NavPath) -> None:
        now = time.monotonic()
        if now - self.last_plan_sent < 0.5:
            return
        self.last_plan_sent = now
        poses = message.poses
        step = max(1, len(poses) // 400)
        points = []
        for item in poses[::step]:
            points.append(round(item.pose.position.x, 3))
            points.append(round(item.pose.position.y, 3))
        self.hub.publish('plan', {
            'frame': message.header.frame_id, 'points': points})

    def _status(self, message: OperatorStatus) -> None:
        previous_map = self.active_map
        self.active_map = message.active_map
        battery = message.battery_percentage
        frame = message.header.frame_id
        x, y, yaw = message.pose.x, message.pose.y, message.pose.theta
        if frame != 'map':
            # During SLAM mapping the backend has no active map and reports
            # odom; SLAM still publishes map->odom, so show the map pose.
            pose = self._transform_2d('map', 'base_link')
            if pose is not None:
                frame = 'map'
                x, y, yaw = pose
        self.hub.publish('status', {
            'frame': frame,
            'mode': MODE_NAMES.get(message.mode, str(message.mode)),
            'active_map': message.active_map,
            'navigation_state': message.navigation_state,
            'emergency_stop': message.emergency_stop,
            'motor_enabled': message.motor_enabled,
            'control_owner': message.control_owner,
            'battery': None if not math.isfinite(battery) else round(battery, 1),
            'x': x,
            'y': y,
            'yaw': yaw,
            'linear': message.linear_velocity,
            'angular': message.angular_velocity,
            'map_ok': message.map_healthy,
            'scan_ok': message.scan_healthy,
            'tf_ok': message.tf_healthy,
            'localized': message.localization_healthy,
            'diagnostic': DIAG_NAMES.get(message.diagnostic_level, '?'),
            'diagnostic_message': message.diagnostic_message,
            'operation': message.active_operation,
            'recovery_active': message.localization_recovery_active,
            'recovery_status': message.localization_recovery_status,
        })
        if message.active_map != previous_map:
            self._schedule_graph_refresh()

    def _tags(self, message: AprilTagLandmarkArray) -> None:
        self.hub.publish('tags', {
            'map_id': message.map_id,
            'status': message.status,
            'map_pose_available': message.map_pose_available,
            'auto_init': message.auto_initialization_enabled,
            'tags': [{
                'id': tag.id,
                'name': tag.name,
                'x': tag.pose.position.x,
                'y': tag.pose.position.y,
                'yaw': yaw_of(tag.pose.orientation),
                'saved': tag.saved,
                'visible': tag.visible,
                'capturing': tag.capture_in_progress,
                'samples': tag.sample_count,
                'distance': tag.distance_m,
                'margin': tag.decision_margin,
            } for tag in message.landmarks],
        })

    def _startup(self, message: String) -> None:
        self.hub.publish('startup', {'text': message.data})

    # ------------------------------------------------------------- services
    def _call(self, name: str, request, timeout: float = 5.0):
        client = self.services_[name]
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'{client.srv_name} is unavailable')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            future.cancel()
            raise RuntimeError(f'{client.srv_name} timed out')
        return future.result()

    def _schedule_graph_refresh(self) -> None:
        self.graph_refresh_pending.set()

    def _graph_timer(self) -> None:
        if not self.graph_refresh_pending.is_set():
            return
        self.graph_refresh_pending.clear()
        try:
            self.refresh_graph()
        except RuntimeError as error:
            self.get_logger().debug(f'Waypoint refresh failed: {error}')
            self.graph_refresh_pending.set()

    def refresh_graph(self) -> dict:
        if not self.active_map:
            return self.graph
        request = GetWaypointGraph.Request(map_id=self.active_map, set_id='')
        result = self._call('graph_get', request)
        sets = self._call('sets', ListWaypointSets.Request(map_id=self.active_map))
        segments = [self._segment_dict(item) for item in result.segments]
        with self.lock:
            self.graph = {
                'revision': result.revision,
                'set_id': result.set_id,
                'segments': segments,
            }
        self.hub.publish('waypoints', {
            'map_id': self.active_map,
            'set_id': result.set_id,
            'revision': result.revision,
            'waypoints': [self._waypoint_dict(w) for w in result.waypoints],
            'segments': segments,
            'sets': [{
                'id': s.id, 'name': s.name, 'count': s.waypoint_count,
            } for s in sets.sets],
            'dock': self._waypoint_dict(sets.dock) if sets.has_dock else None,
        })
        return self.graph

    @staticmethod
    def _waypoint_dict(item: Waypoint) -> dict:
        return {
            'id': item.id,
            'name': item.name,
            'x': item.pose.x,
            'y': item.pose.y,
            'yaw': item.pose.theta,
            'dwell': item.dwell_seconds,
            'enabled': item.enabled,
        }

    @staticmethod
    def _waypoint_msg(data: dict) -> Waypoint:
        item = Waypoint()
        item.id = str(data.get('id', ''))
        item.name = str(data.get('name', ''))[:80]
        item.pose.x = float(data['x'])
        item.pose.y = float(data['y'])
        item.pose.theta = float(data.get('yaw', 0.0))
        item.dwell_seconds = max(0.0, float(data.get('dwell', 0.0)))
        item.enabled = bool(data.get('enabled', True))
        return item

    def _segment_dict(self, item: RouteSegment) -> dict:
        return {
            'id': item.id,
            'name': item.name,
            'from': item.from_waypoint_id,
            'to': item.to_waypoint_id,
            'bidirectional': item.bidirectional,
            'enabled': item.enabled,
            'via': [self._waypoint_dict(v) for v in item.via_points],
        }

    def _segment_msg(self, data: dict) -> RouteSegment:
        item = RouteSegment()
        item.id = str(data.get('id', ''))
        item.name = str(data.get('name', ''))
        item.from_waypoint_id = str(data.get('from', ''))
        item.to_waypoint_id = str(data.get('to', ''))
        item.bidirectional = bool(data.get('bidirectional', False))
        item.enabled = bool(data.get('enabled', True))
        item.via_points = [self._waypoint_msg(v) for v in data.get('via', [])]
        return item

    # ----------------------------------------------------------------- lease
    def _publish_lease(self) -> None:
        self.hub.publish('lease', {
            'client': self.lease_client,
            'name': self.lease_client_name,
            'held': bool(self.lease_id),
        })

    def _drop_lease(self, release: bool) -> None:
        lease_id = self.lease_id
        self.lease_id = ''
        self.lease_client = ''
        self.lease_client_name = ''
        if release and lease_id:
            try:
                self._call('lease', ControlLease.Request(
                    action=ControlLease.Request.RELEASE, lease_id=lease_id))
            except RuntimeError:
                pass
        self._publish_lease()

    def _lease_timer(self) -> None:
        with self.lock:
            if not self.lease_id:
                return
            stale = time.monotonic() - self.last_heartbeat > HEARTBEAT_TIMEOUT
        if stale:
            self.get_logger().warning('Web client heartbeat lost; releasing control')
            with self.lock:
                self._drop_lease(release=True)
            return
        try:
            result = self._call('lease', ControlLease.Request(
                action=ControlLease.Request.RENEW, lease_id=self.lease_id),
                timeout=2.0)
        except RuntimeError as error:
            # A slow or briefly unavailable backend is not a lost lease; retry
            # next tick. The backend itself expires the lease after 5 s.
            self.get_logger().warning(f'Lease renewal delayed: {error}')
            return
        if not result.success:
            self.get_logger().warning(f'Control lease lost: {result.message}')
            with self.lock:
                self._drop_lease(release=False)

    def _require_lease(self, client: str) -> str:
        if not self.lease_id or client != self.lease_client:
            raise PermissionError('Take control first')
        return self.lease_id

    # -------------------------------------------------------------- commands
    def dispatch(self, cmd: dict):
        op = cmd.get('op', '')
        client = str(cmd.get('client', ''))
        handler = getattr(self, f'op_{op}', None)
        if handler is None:
            raise ValueError(f'Unknown op {op!r}')
        return handler(cmd, client)

    def op_heartbeat(self, _cmd, client):
        if client and client == self.lease_client:
            self.last_heartbeat = time.monotonic()
        return {'held': client == self.lease_client and bool(self.lease_id)}

    def op_take_control(self, cmd, client):
        if not client:
            raise ValueError('client id required')
        with self.lock:
            if self.lease_id and self.lease_client == client:
                self.last_heartbeat = time.monotonic()
                return 'Already in control'
            if self.lease_id:
                # Another browser holds the web lease: hand it over.
                self._drop_lease(release=True)
            name = str(cmd.get('name', '') or 'browser')[:24]
            result = self._call('lease', ControlLease.Request(
                action=ControlLease.Request.ACQUIRE, client_id=f'web:{name}'))
            if not result.success:
                raise RuntimeError(result.message)
            self.lease_id = result.lease_id
            self.lease_client = client
            self.lease_client_name = name
            self.last_heartbeat = time.monotonic()
        self._publish_lease()
        return result.message

    def op_release_control(self, _cmd, client):
        with self.lock:
            if self.lease_id and client == self.lease_client:
                self._drop_lease(release=True)
        return 'Control released'

    def op_estop(self, _cmd, _client):
        result = self._call('estop', SetEmergencyStop.Request(active=True))
        return result.message

    def op_estop_clear(self, cmd, client):
        lease = self._require_lease(client)
        result = self._call('estop', SetEmergencyStop.Request(
            active=False, lease_id=lease,
            confirmation=str(cmd.get('confirmation', ''))))
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def op_teleop(self, cmd, client):
        lease = self._require_lease(client)
        self.last_heartbeat = time.monotonic()
        message = TeleopCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.lease_id = lease
        with self.lock:
            self.teleop_sequence += 1
            message.sequence = self.teleop_sequence
        message.deadman = bool(cmd.get('deadman', False))
        linear = float(cmd.get('linear', 0.0))
        angular = float(cmd.get('angular', 0.0))
        if not (math.isfinite(linear) and math.isfinite(angular)):
            message.deadman = False
            linear = angular = 0.0
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.teleop_pub.publish(message)
        return None

    def op_initial_pose(self, cmd, client):
        self._require_lease(client)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        x, y, yaw = float(cmd['x']), float(cmd['y']), float(cmd['yaw'])
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Same defaults as the RViz "2D Pose Estimate" tool.
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06853891945200942
        self.initial_pose_pub.publish(message)
        return f'Pose estimate sent ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)'

    def op_goal(self, cmd, client):
        self._require_lease(client)
        x, y, yaw = float(cmd['x']), float(cmd['y']), float(cmd['yaw'])
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(message)
        return f'Goal sent ({x:.2f}, {y:.2f})'

    def op_patrol(self, cmd, client):
        lease = self._require_lease(client)
        commands = {
            'navigate': PatrolCommand.Request.NAVIGATE,
            'start': PatrolCommand.Request.START,
            'pause': PatrolCommand.Request.PAUSE,
            'resume': PatrolCommand.Request.RESUME,
            'stop': PatrolCommand.Request.STOP,
        }
        request = PatrolCommand.Request(
            command=commands[cmd['command']],
            set_id=str(cmd.get('set_id', '')),
            waypoint_id=str(cmd.get('waypoint_id', '')),
            loop=bool(cmd.get('loop', False)),
            lease_id=lease,
        )
        result = self._call('patrol', request, timeout=10.0)
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def op_save_waypoints(self, cmd, client):
        lease = self._require_lease(client)
        request = SaveWaypointGraph.Request(
            map_id=self.active_map,
            set_id=str(cmd.get('set_id', '')),
            expected_revision=int(cmd.get('revision', 0)),
            waypoints=[self._waypoint_msg(w) for w in cmd.get('waypoints', [])],
            segments=[self._segment_msg(s) for s in cmd.get('segments', [])],
            lease_id=lease,
        )
        result = self._call('graph_save', request)
        self._schedule_graph_refresh()
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def op_waypoint_set(self, cmd, client):
        lease = self._require_lease(client)
        actions = {
            'create': ManageWaypointSet.Request.CREATE,
            'rename': ManageWaypointSet.Request.RENAME,
            'delete': ManageWaypointSet.Request.DELETE,
            'select': ManageWaypointSet.Request.SELECT,
        }
        result = self._call('set_manage', ManageWaypointSet.Request(
            action=actions[cmd['action']],
            map_id=self.active_map,
            set_id=str(cmd.get('set_id', '')),
            name=str(cmd.get('name', ''))[:80],
            lease_id=lease,
        ))
        self._schedule_graph_refresh()
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def op_save_dock(self, cmd, client):
        lease = self._require_lease(client)
        dock = self._waypoint_msg({
            'id': 'dock', 'name': 'dock',
            'x': cmd['x'], 'y': cmd['y'], 'yaw': cmd['yaw'],
        })
        result = self._call('dock', SaveDock.Request(
            map_id=self.active_map, dock=dock, lease_id=lease))
        self._schedule_graph_refresh()
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def op_list_maps(self, _cmd, _client):
        result = self._call('maps', ListMaps.Request())
        return {
            'active': result.active_map,
            'maps': [{'id': m.map_id, 'name': m.display_name} for m in result.maps],
        }

    def op_recovery(self, cmd, client):
        lease = self._require_lease(client)
        action = (LocalizationRecovery.Request.START if cmd.get('start')
                  else LocalizationRecovery.Request.STOP)
        result = self._call('recovery', LocalizationRecovery.Request(
            action=action, lease_id=lease), timeout=10.0)
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def op_tag(self, cmd, client):
        self._require_lease(client)
        action = cmd['action']
        if action == 'localize':
            result = self._call('tags_localize', Trigger.Request(), timeout=10.0)
        else:
            actions = {
                'capture': ManageAprilTagLandmark.Request.CAPTURE,
                'delete': ManageAprilTagLandmark.Request.DELETE,
                'reload': ManageAprilTagLandmark.Request.RELOAD,
            }
            result = self._call('tags', ManageAprilTagLandmark.Request(
                action=actions[action],
                map_id=self.active_map,
                tag_id=int(cmd.get('tag_id', -1)),
                name=str(cmd.get('name', ''))[:80],
            ), timeout=15.0)
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    # --------------------------------------------------------------- mapping
    def _mapping_timer(self) -> None:
        active = self.slam_save.service_is_ready()
        if active != self.mapping_active:
            if active:
                # A new SLAM session starts with nothing saved.
                self.map_saved_crc = 0
            self.mapping_active = active
            self._publish_mapping()
        elif active:
            self._publish_mapping()
        managed = self.services_['mode_manager'].service_is_ready()
        if managed != self.managed:
            self.managed = managed
            self._publish_mode()

    def _publish_mapping(self) -> None:
        self.hub.publish('mapping', {
            'active': bool(self.mapping_active),
            'saving': self.map_saving,
            'directory': str(self.maps_directory),
            # The live map changed since the last successful save.
            'unsaved': bool(self.mapping_active)
            and self.map_crc != self.map_saved_crc,
        })

    def _mode_state_message(self, message) -> None:
        try:
            state = json.loads(message.data)
        except ValueError:
            return
        self.mode_state = {
            'mode': str(state.get('mode', '')),
            'map_id': str(state.get('map_id', '')),
            'detail': str(state.get('detail', '')),
        }
        self._publish_mode()

    def _publish_mode(self) -> None:
        self.hub.publish('mode', {
            **self.mode_state,
            'managed': self.managed,
            'switching': self.switching,
        })

    def op_set_mode(self, cmd, client):
        """Switch idle / mapping / operations through the mode manager."""
        lease = self._require_lease(client)
        mode = str(cmd.get('mode', ''))
        if mode not in ('idle', 'mapping', 'operations'):
            raise ValueError('mode must be idle, mapping or operations')
        if not self.managed:
            raise RuntimeError(
                'Mode switching needs the Sahabat Robot launcher; this stack '
                'was started by a fixed-mode shortcut')
        if self.switching:
            raise RuntimeError(f'Already switching to {self.switching}')
        self.switching = mode
        self._publish_mode()
        try:
            result = self._call('set_mode', SetMode.Request(
                mode=mode, map_id=str(cmd.get('map_id', '')), lease_id=lease),
                timeout=150.0)
        finally:
            self.switching = ''
            self._publish_mode()
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def _existing_map_files(self, name: str):
        stem = self.maps_directory / name
        return [stem.with_suffix(suffix) for suffix in MAP_FILE_SUFFIXES
                if stem.with_suffix(suffix).exists()]

    def op_map_name_check(self, cmd, _client):
        name = str(cmd.get('name', '')).strip()
        if not VALID_MAP_NAME.fullmatch(name):
            raise ValueError(
                'Use letters, numbers, hyphens and underscores (max 64)')
        return {'existing': [p.name for p in self._existing_map_files(name)]}

    def op_save_map(self, cmd, client):
        """Save the live SLAM map: <name>.yaml/.pgm plus <name>.posegraph/.data.

        The navigation map is written by this node from the /map it already
        holds, in nav2 map_saver format. slam_toolbox's own save_map runs
        map_saver_cli with a 2 s subscription timeout, which fails silently
        on a loaded Jetson while still reporting success. Every file is
        checked on disk before success is reported. Replaced files are first
        moved to <maps_directory>/.archive/<name>-<timestamp>/.
        """
        self._require_lease(client)
        name = str(cmd.get('name', '')).strip()
        if not VALID_MAP_NAME.fullmatch(name):
            raise ValueError(
                'Use letters, numbers, hyphens and underscores (max 64)')
        if self.map_saving:
            raise RuntimeError(f'Already saving {self.map_saving}')
        if not self.slam_save.service_is_ready():
            raise RuntimeError('SLAM mapping is not running')
        grid = self.latest_map
        if grid is None or not grid.info.width or not grid.info.height:
            raise RuntimeError('No map received from SLAM yet')
        existing = self._existing_map_files(name)
        if existing and not cmd.get('overwrite'):
            raise RuntimeError(f'Map {name} already exists')
        self.map_saving = name
        self._publish_mapping()
        try:
            self.maps_directory.mkdir(parents=True, exist_ok=True)
            archived = ''
            if existing:
                archive = (self.maps_directory / '.archive'
                           / f'{name}-{time.strftime("%Y%m%d-%H%M%S")}')
                archive.mkdir(parents=True)
                for path in existing:
                    shutil.move(str(path), str(archive / path.name))
                archived = f' Previous files moved to {archive}.'
            stem = self.maps_directory / name
            write_map_files(grid, stem)
            message = (f'Saved {name}.yaml and {name}.pgm '
                       f'({grid.info.width}x{grid.info.height} cells)')
            if cmd.get('session', True):
                request = SerializePoseGraph.Request()
                request.filename = str(stem)
                try:
                    result = self._call_slam(self.slam_serialize, request, 60.0)
                    ok = result.result == SerializePoseGraph.Response.RESULT_SUCCESS
                except RuntimeError:
                    ok = False
                session_files = [stem.with_suffix(s) for s in ('.posegraph', '.data')]
                if not ok or not all(p.exists() and p.stat().st_size
                                     for p in session_files):
                    raise RuntimeError(
                        f'{message}, but the editable session (.posegraph/.data) '
                        'was NOT saved. The map is usable for navigation; '
                        'save again to retry the session.')
                message += ' and the editable session'
            missing = [p.name for p in (stem.with_suffix('.yaml'), stem.with_suffix('.pgm'))
                       if not (p.exists() and p.stat().st_size)]
            if missing:
                raise RuntimeError(f'Save failed: missing {", ".join(missing)}')
            self.map_saved_crc = zlib.crc32(grid_bytes(grid.data))
            return message + '.' + archived
        finally:
            self.map_saving = ''
            self._publish_mapping()

    @staticmethod
    def _call_slam(client, request, timeout: float):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'{client.srv_name} is unavailable')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            future.cancel()
            raise RuntimeError(f'{client.srv_name} timed out')
        return future.result()

    def op_list_map_files(self, _cmd, _client):
        maps = []
        for path in sorted(self.maps_directory.glob('*.yaml')):
            if path.name.endswith(('_waypoints.yaml', '.metadata.yaml')):
                continue
            if not path.with_suffix('.pgm').exists():
                continue
            maps.append({
                'id': path.stem,
                'session': path.with_suffix('.posegraph').exists(),
                'modified': path.stat().st_mtime,
            })
        # Older maps saved by operator_backend: <maps>/<id>/map.yaml.
        known = {item['id'] for item in maps}
        for path in sorted(self.maps_directory.glob('*/map.yaml')):
            map_id = path.parent.name
            if map_id.startswith('.') or map_id in known:
                continue
            if not VALID_MAP_NAME.fullmatch(map_id):
                continue
            maps.append({
                'id': map_id,
                'session': (path.parent / 'session.posegraph').exists(),
                'modified': path.stat().st_mtime,
            })
        maps.sort(key=lambda item: item['modified'], reverse=True)
        return {'maps': maps}

    def op_refresh(self, _cmd, _client):
        self.refresh_graph()
        return 'Refreshed'


class QuietServer(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        # Browsers drop SSE/keep-alive sockets all the time; that is not an error.
        if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
            return
        super().handle_error(request, client_address)


def make_handler(node: WebConsole, hub: Hub, web_root: Path):
    content_types = {
        '.html': 'text/html; charset=utf-8',
        '.js': 'text/javascript; charset=utf-8',
        '.css': 'text/css; charset=utf-8',
        '.svg': 'image/svg+xml',
        '.png': 'image/png',
        '.ico': 'image/x-icon',
    }

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_args):
            pass

        def _send(self, code, body: bytes, ctype='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, payload):
            self._send(code, json.dumps(payload).encode())

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/stream':
                return self._stream()
            if path == '/api/camera.mjpg':
                return self._camera()
            if path == '/api/map.png':
                with node.lock:
                    png = node.map_png
                if not png:
                    return self._send(404, b'{}')
                return self._send(200, png, 'image/png')
            if path == '/':
                path = '/index.html'
            parts = [part for part in path.split('/') if part]
            target = web_root.joinpath(*parts)
            # Files may be symlinks (--symlink-install), so check the
            # requested path rather than the resolved one.
            if any(part in ('.', '..') for part in parts) or not target.is_file():
                return self._send(404, b'not found', 'text/plain')
            self._send(
                200, target.read_bytes(),
                content_types.get(target.suffix, 'application/octet-stream'))

        def do_POST(self):
            if self.path != '/api/cmd':
                return self._send(404, b'{}')
            try:
                length = int(self.headers.get('Content-Length', '0'))
                cmd = json.loads(self.rfile.read(min(length, 1 << 20)) or b'{}')
                result = node.dispatch(cmd)
                self._json(200, {'ok': True, 'result': result})
            except PermissionError as error:
                self._json(403, {'ok': False, 'error': str(error)})
            except (KeyError, ValueError, TypeError) as error:
                self._json(400, {'ok': False, 'error': f'Bad request: {error}'})
            except RuntimeError as error:
                self._json(409, {'ok': False, 'error': str(error)})

        def _camera(self):
            boundary = 'sahabatframe'
            self.send_response(200)
            self.send_header(
                'Content-Type', f'multipart/x-mixed-replace; boundary={boundary}')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            camera = node.camera
            # Optional per-viewer cap, e.g. ?fps=5 for a slow remote link.
            query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
            try:
                min_gap = 1.0 / max(0.5, float(query.get('fps', ['100'])[0]))
            except ValueError:
                min_gap = 0.0
            camera.attach()
            sequence = -1
            sent_at = 0.0
            try:
                while True:
                    sequence, frame = camera.wait_frame(sequence, timeout=5.0)
                    if not frame:
                        continue
                    now = time.monotonic()
                    # Tolerate frame-time jitter so ?fps=15 on a 15 fps
                    # camera does not drop every other frame.
                    if now - sent_at < 0.75 * min_gap:
                        continue
                    sent_at = now
                    self.wfile.write(
                        f'--{boundary}\r\nContent-Type: image/jpeg\r\n'
                        f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                camera.detach()
                self.close_connection = True

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            client = hub.subscribe()
            try:
                while True:
                    try:
                        text = client.get(timeout=10.0)
                    except queue.Empty:
                        text = ': keepalive\n\n'
                    self.wfile.write(text.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                hub.unsubscribe(client)
                self.close_connection = True

    return Handler


def main(args=None):
    rclpy.init(args=args)
    hub = Hub()
    node = WebConsole(hub)
    web_root = str(node.get_parameter('web_root').value)
    if not web_root:
        web_root = str(Path(get_package_share_directory('shbat_pkg')) / 'web')
    web_root = Path(web_root).expanduser().resolve()
    host = str(node.get_parameter('host').value)
    port = int(node.get_parameter('port').value)

    server = QuietServer((host, port), make_handler(node, hub, web_root))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(
        f'Web console on http://{host}:{port}/ (no authentication; '
        'trusted network only)')
    node.get_logger().info(f'Serving UI from {web_root}')

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.lease_id:
            node._drop_lease(release=True)
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
