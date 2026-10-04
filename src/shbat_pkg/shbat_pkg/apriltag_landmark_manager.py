#!/usr/bin/env python3
"""Calibrate fixed AprilTag landmarks and use them to initialize AMCL."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
from typing import Dict, Iterable, Optional

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Transform
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sahabat_interfaces.msg import AprilTagLandmark, AprilTagLandmarkArray
from sahabat_interfaces.srv import ManageAprilTagLandmark
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from shbat_pkg.scan_localization import ScanLocalizationSupport


MAP_ID_PATTERN = re.compile(r'^[A-Za-z0-9_.-]+$')


@dataclass
class LandmarkRecord:
    """One fixed tag pose in the active map frame."""

    tag_id: int
    name: str
    family: str
    size_m: float
    matrix: np.ndarray
    sample_count: int = 0
    translation_std_m: float = 0.0
    rotation_std_rad: float = 0.0


@dataclass
class DetectionState:
    """Latest quality metadata for one detector output."""

    family: str
    hamming: int
    decision_margin: float
    seen_at: float
    camera_frame: str


def normalize_family(family: str) -> str:
    """Compare tag families independent of the optional ``tag`` prefix.

    apriltag_ros reports ``tag36h11`` while the configuration says ``36h11``.
    """
    family = str(family or '').strip()
    return family[3:] if family.startswith('tag') else family


def valid_map_id(map_id: str) -> bool:
    """Accept file-stem map identifiers without allowing path traversal."""
    return bool(map_id) and bool(MAP_ID_PATTERN.fullmatch(map_id))


def derive_map_id(map_id: str, map_file: str) -> str:
    """Use the explicit map id, or derive it from a flat/directory map path."""
    map_id = map_id.strip()
    if map_id:
        return map_id
    path = Path(map_file).expanduser()
    if path.name == 'map.yaml':
        return path.parent.name
    if path.suffix == '.yaml':
        return path.stem
    return path.name


def tag_map_path(maps_directory: Path, map_id: str) -> Path:
    """Return the map-owned landmark path for flat or directory map layouts."""
    if not valid_map_id(map_id):
        raise ValueError(f'Invalid map id: {map_id!r}')
    directory_map = maps_directory / map_id / 'map.yaml'
    if directory_map.is_file():
        return directory_map.parent / 'tags.yaml'
    return maps_directory / f'{map_id}_tags.yaml'


def quaternion_to_matrix(quaternion: Iterable[float]) -> np.ndarray:
    """Convert an xyzw quaternion into a homogeneous rotation matrix."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('Quaternion has zero length')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Convert a homogeneous rotation matrix into a normalized xyzw quaternion."""
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    return quaternion / np.linalg.norm(quaternion)


def transform_to_matrix(transform: Transform) -> np.ndarray:
    """Convert a geometry transform into a homogeneous matrix."""
    matrix = quaternion_to_matrix((
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ))
    matrix[:3, 3] = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    return matrix


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Convert a geometry pose into a homogeneous matrix."""
    matrix = quaternion_to_matrix((
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ))
    matrix[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return matrix


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Convert a homogeneous matrix into a geometry pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(value) for value in matrix[:3, 3]
    )
    quaternion = matrix_to_quaternion(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(value) for value in quaternion
    )
    return pose


def yaw_from_matrix(matrix: np.ndarray) -> float:
    """Return planar yaw from a homogeneous matrix."""
    return math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))


def angle_difference(first: float, second: float) -> float:
    """Return the signed shortest angular difference."""
    return math.atan2(math.sin(first - second), math.cos(first - second))


def average_quaternions(quaternions: np.ndarray) -> np.ndarray:
    """Average unit quaternions while accounting for their double cover."""
    aligned = np.asarray(quaternions, dtype=float).copy()
    reference = aligned[0]
    for index in range(len(aligned)):
        if float(np.dot(aligned[index], reference)) < 0.0:
            aligned[index] *= -1.0
    accumulator = aligned.T @ aligned
    values, vectors = np.linalg.eigh(accumulator)
    result = vectors[:, int(np.argmax(values))]
    if float(np.dot(result, reference)) < 0.0:
        result *= -1.0
    return result / np.linalg.norm(result)


def summarize_transforms(matrices: Iterable[np.ndarray]):
    """Robustly average repeated transforms and report translational/rotational RMS."""
    matrices = [np.asarray(matrix, dtype=float) for matrix in matrices]
    if len(matrices) < 3:
        raise ValueError('At least three transform samples are required')
    translations = np.array([matrix[:3, 3] for matrix in matrices])
    median = np.median(translations, axis=0)
    distances = np.linalg.norm(translations - median, axis=1)
    distance_median = float(np.median(distances))
    mad = float(np.median(np.abs(distances - distance_median)))
    threshold = max(0.03, distance_median + 3.0 * 1.4826 * mad)
    keep = distances <= threshold
    if int(np.count_nonzero(keep)) < 3:
        raise ValueError('Too few consistent transform samples')
    kept = [matrix for matrix, accepted in zip(matrices, keep) if accepted]
    kept_translations = np.array([matrix[:3, 3] for matrix in kept])
    translation = np.mean(kept_translations, axis=0)
    quaternions = np.array([matrix_to_quaternion(matrix) for matrix in kept])
    quaternion = average_quaternions(quaternions)

    average = quaternion_to_matrix(quaternion)
    average[:3, 3] = translation
    translation_std = math.sqrt(float(np.mean(np.sum(
        (kept_translations - translation) ** 2, axis=1
    ))))
    angular_errors = [
        2.0 * math.acos(min(1.0, abs(float(np.dot(sample, quaternion)))))
        for sample in quaternions
    ]
    rotation_std = math.sqrt(float(np.mean(np.square(angular_errors))))
    return average, len(kept), translation_std, rotation_std


def robot_pose_from_tag(
    map_to_tag: np.ndarray,
    camera_to_tag: np.ndarray,
    base_to_camera: np.ndarray,
) -> np.ndarray:
    """Calculate map-to-base from one fixed map tag and one camera observation."""
    return map_to_tag @ np.linalg.inv(camera_to_tag) @ np.linalg.inv(base_to_camera)


def landmark_from_yaml(item: dict, default_family: str, default_size: float) -> LandmarkRecord:
    """Validate and convert one stored YAML landmark."""
    tag_id = int(item['id'])
    position = item['position']
    orientation = item['orientation_xyzw']
    if len(position) != 3 or len(orientation) != 4:
        raise ValueError(f'Tag {tag_id} must have 3D position and xyzw orientation')
    matrix = quaternion_to_matrix(orientation)
    matrix[:3, 3] = [float(value) for value in position]
    return LandmarkRecord(
        tag_id=tag_id,
        name=str(item.get('name') or f'tag_{tag_id}'),
        family=str(item.get('family') or default_family),
        size_m=float(item.get('size_m', default_size)),
        matrix=matrix,
        sample_count=int(item.get('sample_count', 0)),
        translation_std_m=float(item.get('translation_std_m', 0.0)),
        rotation_std_rad=float(item.get('rotation_std_rad', 0.0)),
    )


def landmark_to_yaml(record: LandmarkRecord) -> dict:
    """Convert one landmark into stable, human-readable YAML data."""
    quaternion = matrix_to_quaternion(record.matrix)
    return {
        'id': record.tag_id,
        'name': record.name,
        'family': record.family,
        'size_m': round(record.size_m, 6),
        'position': [round(float(value), 6) for value in record.matrix[:3, 3]],
        'orientation_xyzw': [round(float(value), 8) for value in quaternion],
        'sample_count': record.sample_count,
        'translation_std_m': round(record.translation_std_m, 6),
        'rotation_std_rad': round(record.rotation_std_rad, 6),
    }


class AprilTagLandmarkManager(Node):
    """Own fixed landmark calibration and guarded AMCL initialization."""

    def __init__(self):
        super().__init__('apriltag_landmark_manager')
        self.declare_parameter('maps_directory', '~/sahabat_ws/maps')
        self.declare_parameter('map_id', '')
        self.declare_parameter('map_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'zed2i_left_camera_optical_frame')
        self.declare_parameter('family', '36h11')
        self.declare_parameter('tag_size', 0.1651)
        self.declare_parameter('detection_topic', '/apriltag/detections')
        self.declare_parameter('minimum_decision_margin', 30.0)
        self.declare_parameter('maximum_hamming', 0)
        self.declare_parameter('detection_timeout', 0.5)
        self.declare_parameter('capture_duration', 2.0)
        self.declare_parameter('capture_minimum_samples', 12)
        self.declare_parameter('capture_max_translation_std', 0.06)
        self.declare_parameter('capture_max_rotation_std', 0.12)
        self.declare_parameter('require_stationary', True)
        self.declare_parameter('stationary_linear_threshold', 0.02)
        self.declare_parameter('stationary_angular_threshold', 0.04)
        self.declare_parameter('auto_initialize', True)
        self.declare_parameter('initialization_minimum_samples', 8)
        self.declare_parameter('initialization_window', 1.5)
        self.declare_parameter('initialization_max_position_std', 0.06)
        self.declare_parameter('initialization_max_yaw_std', 0.10)
        self.declare_parameter('maximum_tag_disagreement_position', 0.25)
        self.declare_parameter('maximum_tag_disagreement_yaw', 0.35)
        # Tags far from the camera give poor PnP orientation; ignore them for
        # initialization (they still show in the panel).
        self.declare_parameter('maximum_initialization_distance', 4.0)
        # Lidar confirmation of the tag pose before AMCL is seeded.
        self.declare_parameter('scan_validation', True)
        self.declare_parameter('scan_refine_xy_window', 0.40)
        self.declare_parameter('scan_refine_yaw_window', 0.26)
        self.declare_parameter('scan_minimum_inlier_ratio', 0.55)
        self.declare_parameter('scan_maximum_ambiguity', 0.92)
        self.declare_parameter('scan_retry_interval', 3.0)
        # Covariance handed to AMCL. Kept honest so AMCL can still correct.
        self.declare_parameter('initial_position_stddev', 0.10)
        self.declare_parameter('initial_yaw_stddev', 0.06)

        self.maps_directory = Path(str(
            self.get_parameter('maps_directory').value
        )).expanduser()
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.default_family = str(self.get_parameter('family').value)
        self.default_size = float(self.get_parameter('tag_size').value)
        self.minimum_margin = float(
            self.get_parameter('minimum_decision_margin').value
        )
        self.maximum_hamming = int(self.get_parameter('maximum_hamming').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.capture_duration = float(self.get_parameter('capture_duration').value)
        self.capture_minimum_samples = int(
            self.get_parameter('capture_minimum_samples').value
        )
        self.capture_max_translation_std = float(
            self.get_parameter('capture_max_translation_std').value
        )
        self.capture_max_rotation_std = float(
            self.get_parameter('capture_max_rotation_std').value
        )
        self.require_stationary = bool(self.get_parameter('require_stationary').value)
        self.stationary_linear_threshold = float(
            self.get_parameter('stationary_linear_threshold').value
        )
        self.stationary_angular_threshold = float(
            self.get_parameter('stationary_angular_threshold').value
        )
        self.auto_initialize = bool(self.get_parameter('auto_initialize').value)
        self.initialization_minimum_samples = int(
            self.get_parameter('initialization_minimum_samples').value
        )
        self.initialization_window = float(
            self.get_parameter('initialization_window').value
        )
        self.initialization_max_position_std = float(
            self.get_parameter('initialization_max_position_std').value
        )
        self.initialization_max_yaw_std = float(
            self.get_parameter('initialization_max_yaw_std').value
        )
        self.maximum_tag_disagreement_position = float(
            self.get_parameter('maximum_tag_disagreement_position').value
        )
        self.maximum_tag_disagreement_yaw = float(
            self.get_parameter('maximum_tag_disagreement_yaw').value
        )

        self.maximum_initialization_distance = float(
            self.get_parameter('maximum_initialization_distance').value
        )
        self.scan_validation = bool(self.get_parameter('scan_validation').value)
        self.scan_refine_xy_window = float(
            self.get_parameter('scan_refine_xy_window').value
        )
        self.scan_refine_yaw_window = float(
            self.get_parameter('scan_refine_yaw_window').value
        )
        self.scan_minimum_inlier_ratio = float(
            self.get_parameter('scan_minimum_inlier_ratio').value
        )
        self.scan_maximum_ambiguity = float(
            self.get_parameter('scan_maximum_ambiguity').value
        )
        self.scan_retry_interval = float(
            self.get_parameter('scan_retry_interval').value
        )
        self.initial_position_stddev = float(
            self.get_parameter('initial_position_stddev').value
        )
        self.initial_yaw_stddev = float(
            self.get_parameter('initial_yaw_stddev').value
        )
        self.scan_retry_after = 0.0

        explicit_map_id = str(self.get_parameter('map_id').value)
        map_file = str(self.get_parameter('map_file').value)
        self.active_map = derive_map_id(explicit_map_id, map_file)
        self.landmarks: Dict[int, LandmarkRecord] = {}
        self.detections: Dict[int, DetectionState] = {}
        self.capture = None
        self.capture_samples = []
        self.capture_transform_stamp = None
        self.pose_samples = deque()
        self.manual_localization_requested = False
        self.auto_initialization_complete = False
        self.awaiting_amcl_validation = False
        self.amcl_validation_deadline = 0.0
        self.pending_initial_pose = None
        self.pending_publish_count = 0
        self.pending_next_publish = 0.0
        self.last_amcl_pose_at = 0.0
        self.amcl_covariance_good = False
        self.last_odom_at = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.status = 'Waiting for AprilTag detections.'

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_support = (
            ScanLocalizationSupport(self, self.tf_buffer, self.base_frame)
            if self.scan_validation else None
        )

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.landmark_pub = self.create_publisher(
            AprilTagLandmarkArray,
            '/apriltag_landmarks/state',
            state_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/apriltag_landmarks/markers',
            state_qos,
        )
        self.configured_pub = self.create_publisher(
            Bool,
            '/localization/apriltag_configured',
            state_qos,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        self.create_subscription(
            AprilTagDetectionArray,
            str(self.get_parameter('detection_topic').value),
            self._detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose,
            10,
        )
        self.create_service(
            ManageAprilTagLandmark,
            '/apriltag_landmarks/manage',
            self._manage,
        )
        self.create_service(
            Trigger,
            '/localization/set_from_tags',
            self._request_localization,
        )
        self.create_timer(0.1, self._update)
        self.create_timer(0.5, self._publish_state)

        self._load_active_map()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _landmark_path(self) -> Optional[Path]:
        if not valid_map_id(self.active_map):
            return None
        return tag_map_path(self.maps_directory, self.active_map)

    def _load_active_map(self) -> None:
        self.landmarks = {}
        self.pose_samples.clear()
        self.auto_initialization_complete = False
        self.awaiting_amcl_validation = False
        self.pending_initial_pose = None
        self.pending_publish_count = 0
        path = self._landmark_path()
        if path is None:
            self.status = 'Select a valid map before recording tags.'
            self.configured_pub.publish(Bool(data=False))
            return
        if not path.is_file():
            self.status = f'No saved tags for {self.active_map}; localize normally, then capture one.'
            self.configured_pub.publish(Bool(data=False))
            return
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            if str(data.get('map_id', self.active_map)) != self.active_map:
                raise ValueError('tag file map_id does not match the active map')
            for item in data.get('tags', []):
                record = landmark_from_yaml(item, self.default_family, self.default_size)
                if record.tag_id in self.landmarks:
                    raise ValueError(f'duplicate tag id {record.tag_id}')
                self.landmarks[record.tag_id] = record
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            self.status = f'Could not load {path}: {error}'
            self.get_logger().error(self.status)
            self.configured_pub.publish(Bool(data=False))
            return
        self.status = f'Loaded {len(self.landmarks)} tag(s) for {self.active_map}.'
        self.get_logger().info(f'{self.status} Source: {path}')
        self.configured_pub.publish(Bool(data=bool(self.landmarks)))

    def _save_active_map(self) -> Path:
        path = self._landmark_path()
        if path is None:
            raise ValueError('A valid active map is required')
        data = {
            'version': 1,
            'map_id': self.active_map,
            'family': self.default_family,
            'default_size_m': self.default_size,
            'tags': [
                landmark_to_yaml(self.landmarks[tag_id])
                for tag_id in sorted(self.landmarks)
            ],
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f'.{path.name}.tmp')
        temporary.write_text(
            yaml.safe_dump(data, sort_keys=False),
            encoding='utf-8',
        )
        os.replace(temporary, path)
        self.configured_pub.publish(Bool(data=bool(self.landmarks)))
        return path

    def _detections(self, message: AprilTagDetectionArray) -> None:
        now = self._now()
        camera_frame = message.header.frame_id or self.camera_frame
        for detection in message.detections:
            self.detections[int(detection.id)] = DetectionState(
                family=str(detection.family),
                hamming=int(detection.hamming),
                decision_margin=float(detection.decision_margin),
                seen_at=now,
                camera_frame=camera_frame,
            )

    def _odom(self, message: Odometry) -> None:
        self.last_odom_at = self._now()
        self.linear_velocity = math.hypot(
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
        )
        self.angular_velocity = abs(message.twist.twist.angular.z)

    def _amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.last_amcl_pose_at = self._now()
        covariance = message.pose.covariance
        self.amcl_covariance_good = (
            math.isfinite(covariance[0])
            and math.isfinite(covariance[7])
            and math.isfinite(covariance[35])
            and covariance[0] <= 0.20
            and covariance[7] <= 0.20
            and covariance[35] <= 0.20
        )
        if self.amcl_covariance_good and self.awaiting_amcl_validation:
            self.awaiting_amcl_validation = False
            self.status = 'AMCL accepted the AprilTag initial pose.'
            self.get_logger().info(self.status)

    def _stationary(self) -> bool:
        if not self.require_stationary:
            return True
        return (
            self._now() - self.last_odom_at <= 1.0
            and self.linear_velocity <= self.stationary_linear_threshold
            and self.angular_velocity <= self.stationary_angular_threshold
        )

    def _map_pose_available(self) -> bool:
        try:
            return self.tf_buffer.can_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return False

    def _trusted_map_pose_available(self) -> bool:
        return bool(
            self._map_pose_available()
            and self._now() - self.last_amcl_pose_at <= 1.0
            and self.amcl_covariance_good
        )

    def _current_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        matrix = transform_to_matrix(transform.transform)
        return float(matrix[0, 3]), float(matrix[1, 3]), yaw_from_matrix(matrix)

    def _visible(self, tag_id: int) -> bool:
        detection = self.detections.get(tag_id)
        return bool(detection and self._now() - detection.seen_at <= self.detection_timeout)

    @staticmethod
    def _tag_frame(family: str, tag_id: int) -> str:
        # apriltag_ros uses "<family>:<id>" unless an explicit frame list is
        # configured.  Keep that default so any tag id can be added at runtime.
        return f'{family}:{tag_id}'

    def _quality_good(self, detection: DetectionState) -> bool:
        return (
            detection.hamming <= self.maximum_hamming
            and detection.decision_margin >= self.minimum_margin
        )

    def _camera_to_tag(self, tag_id: int, detection: DetectionState):
        try:
            transform = self.tf_buffer.lookup_transform(
                detection.camera_frame,
                self._tag_frame(detection.family, tag_id),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None, None
        return transform_to_matrix(transform.transform), transform.header.stamp

    def _map_to_tag(self, tag_id: int, detection: DetectionState):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self._tag_frame(detection.family, tag_id),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None, None
        return transform_to_matrix(transform.transform), transform.header.stamp

    def _base_to_camera(self, camera_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None
        return transform_to_matrix(transform.transform)

    def _manage(self, request, response):
        requested_map = request.map_id.strip()
        if requested_map:
            if not valid_map_id(requested_map):
                response.message = 'Map id contains unsupported characters.'
                return response
            if requested_map != self.active_map:
                self.active_map = requested_map
                self._load_active_map()

        if request.action == ManageAprilTagLandmark.Request.SET_MAP:
            response.success = bool(self.active_map)
            response.message = f'Active AprilTag map: {self.active_map or "none"}'
            return response
        if request.action == ManageAprilTagLandmark.Request.RELOAD:
            self._load_active_map()
            response.success = self._landmark_path() is not None
            response.message = self.status
            return response
        if request.action == ManageAprilTagLandmark.Request.DELETE:
            if request.tag_id not in self.landmarks:
                response.message = f'Tag {request.tag_id} is not saved.'
                return response
            del self.landmarks[request.tag_id]
            try:
                path = self._save_active_map()
            except (OSError, ValueError, yaml.YAMLError) as error:
                response.message = f'Could not save tag map: {error}'
                self._load_active_map()
                return response
            self.status = f'Deleted tag {request.tag_id}; saved {path}.'
            response.success = True
            response.message = self.status
            return response
        if request.action != ManageAprilTagLandmark.Request.CAPTURE:
            response.message = 'Unsupported AprilTag landmark action.'
            return response
        if self.capture is not None:
            response.message = 'Another tag capture is already running.'
            return response
        tag_id = int(request.tag_id)
        detection = self.detections.get(tag_id)
        if detection is None or not self._visible(tag_id):
            response.message = f'Tag {tag_id} is not currently visible.'
            return response
        if not self._quality_good(detection):
            response.message = (
                f'Tag {tag_id} quality is too low '
                f'(hamming={detection.hamming}, margin={detection.decision_margin:.1f}).'
            )
            return response
        if not self._stationary():
            response.message = 'Keep the robot stationary before capturing a tag.'
            return response
        if not self._trusted_map_pose_available():
            response.message = (
                'A recent, low-covariance AMCL pose is required before capturing.'
            )
            return response
        if self.scan_support is not None:
            current = self._current_map_pose()
            if current is None:
                response.message = 'Current map pose is unavailable.'
                return response
            confirmed, detail = self.scan_support.confirm(
                current, self.scan_minimum_inlier_ratio
            )
            if not confirmed:
                response.message = (
                    f'Refusing to capture: the lidar does not confirm the current '
                    f'AMCL pose ({detail}). Fix localization first.'
                )
                return response
        self.capture = {
            'tag_id': tag_id,
            'name': request.name.strip() or f'tag_{tag_id}',
            'family': detection.family,
            'deadline': self._now() + self.capture_duration,
        }
        self.capture_samples = []
        self.capture_transform_stamp = None
        self.status = f'Capturing tag {tag_id}; keep the robot still...'
        response.success = True
        response.message = self.status
        return response

    def _request_localization(self, _request, response):
        visible = [
            tag_id for tag_id in self.landmarks
            if self._visible(tag_id)
            and self._quality_good(self.detections[tag_id])
        ]
        if not visible:
            response.message = 'No saved, high-quality AprilTag is visible.'
            return response
        if not self._stationary():
            response.message = 'Keep the robot stationary before relocalizing.'
            return response
        self.manual_localization_requested = True
        self.pose_samples.clear()
        self.status = f'Collecting a stable pose from tag(s) {visible}...'
        response.success = True
        response.message = self.status
        return response

    def _update_capture(self) -> None:
        if self.capture is None:
            return
        if not self._stationary():
            self.status = 'Tag capture failed: the robot moved.'
            self.capture = None
            self.capture_samples = []
            return
        tag_id = self.capture['tag_id']
        detection = self.detections.get(tag_id)
        if detection and self._visible(tag_id) and self._quality_good(detection):
            matrix, stamp = self._map_to_tag(tag_id, detection)
            if matrix is not None:
                stamp_key = (stamp.sec, stamp.nanosec)
                if stamp_key != self.capture_transform_stamp:
                    self.capture_samples.append(matrix)
                    self.capture_transform_stamp = stamp_key
        if self._now() < self.capture['deadline']:
            return
        capture = self.capture
        samples = self.capture_samples
        self.capture = None
        self.capture_samples = []
        if len(samples) < self.capture_minimum_samples:
            self.status = (
                f'Tag {tag_id} capture failed: only {len(samples)} stable samples '
                f'(need {self.capture_minimum_samples}).'
            )
            return
        try:
            matrix, count, translation_std, rotation_std = summarize_transforms(samples)
        except ValueError as error:
            self.status = f'Tag {tag_id} capture failed: {error}'
            return
        if translation_std > self.capture_max_translation_std:
            self.status = (
                f'Tag {tag_id} capture rejected: position scatter '
                f'{translation_std:.3f} m is too high.'
            )
            return
        if rotation_std > self.capture_max_rotation_std:
            self.status = (
                f'Tag {tag_id} capture rejected: rotation scatter '
                f'{math.degrees(rotation_std):.1f} deg is too high.'
            )
            return
        self.landmarks[tag_id] = LandmarkRecord(
            tag_id=tag_id,
            name=capture['name'],
            family=capture['family'],
            size_m=self.default_size,
            matrix=matrix,
            sample_count=count,
            translation_std_m=translation_std,
            rotation_std_rad=rotation_std,
        )
        try:
            path = self._save_active_map()
        except (OSError, ValueError, yaml.YAMLError) as error:
            del self.landmarks[tag_id]
            self.status = f'Tag {tag_id} was measured but could not be saved: {error}'
            return
        self.status = (
            f'Saved tag {tag_id} to {path.name}: {count} samples, '
            f'{translation_std:.3f} m / {math.degrees(rotation_std):.1f} deg scatter.'
        )
        self.get_logger().info(self.status)

    def _collect_localization_candidates(self) -> None:
        automatic = (
            self.auto_initialize
            and bool(self.landmarks)
            and not self.auto_initialization_complete
            and not self.amcl_covariance_good
        )
        if not automatic and not self.manual_localization_requested:
            return
        if not self._stationary():
            self.pose_samples.clear()
            self.status = 'Waiting for the robot to remain stationary before tag localization.'
            return
        candidates = []
        contributing_tags = []
        for tag_id, record in self.landmarks.items():
            detection = self.detections.get(tag_id)
            if (
                detection is None
                or not self._visible(tag_id)
                or not self._quality_good(detection)
                or normalize_family(detection.family) != normalize_family(record.family)
            ):
                continue
            camera_to_tag, _stamp = self._camera_to_tag(tag_id, detection)
            base_to_camera = self._base_to_camera(detection.camera_frame)
            if camera_to_tag is None or base_to_camera is None:
                continue
            if float(np.linalg.norm(camera_to_tag[:3, 3])) > (
                self.maximum_initialization_distance
            ):
                continue
            candidate = robot_pose_from_tag(
                record.matrix,
                camera_to_tag,
                base_to_camera,
            )
            if abs(float(candidate[2, 3])) > 0.25:
                continue
            candidates.append(candidate)
            contributing_tags.append(tag_id)
        if not candidates:
            return
        if self._now() < self.scan_retry_after:
            return
        planar = [
            (float(matrix[0, 3]), float(matrix[1, 3]), yaw_from_matrix(matrix))
            for matrix in candidates
        ]
        for index, first in enumerate(planar):
            for second in planar[index + 1:]:
                if math.hypot(first[0] - second[0], first[1] - second[1]) > (
                    self.maximum_tag_disagreement_position
                ) or abs(angle_difference(first[2], second[2])) > (
                    self.maximum_tag_disagreement_yaw
                ):
                    self.pose_samples.clear()
                    self.status = (
                        f'Visible saved tags {contributing_tags} disagree; '
                        'localization was not reset.'
                    )
                    return
        x = float(np.mean([pose[0] for pose in planar]))
        y = float(np.mean([pose[1] for pose in planar]))
        yaw = math.atan2(
            float(np.mean([math.sin(pose[2]) for pose in planar])),
            float(np.mean([math.cos(pose[2]) for pose in planar])),
        )
        now = self._now()
        self.pose_samples.append((now, x, y, yaw))
        while self.pose_samples and now - self.pose_samples[0][0] > self.initialization_window:
            self.pose_samples.popleft()
        if len(self.pose_samples) < self.initialization_minimum_samples:
            return
        xs = np.array([sample[1] for sample in self.pose_samples])
        ys = np.array([sample[2] for sample in self.pose_samples])
        yaws = np.array([sample[3] for sample in self.pose_samples])
        mean_x = float(np.mean(xs))
        mean_y = float(np.mean(ys))
        mean_yaw = math.atan2(float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws))))
        position_std = math.sqrt(float(np.var(xs) + np.var(ys)))
        yaw_errors = np.array([angle_difference(value, mean_yaw) for value in yaws])
        yaw_std = math.sqrt(float(np.mean(yaw_errors ** 2)))
        if position_std > self.initialization_max_position_std:
            self.status = f'Tag pose is still moving ({position_std:.3f} m scatter).'
            return
        if yaw_std > self.initialization_max_yaw_std:
            self.status = f'Tag heading is still moving ({math.degrees(yaw_std):.1f} deg scatter).'
            return
        self._queue_initial_pose(mean_x, mean_y, mean_yaw, position_std, yaw_std)

    def _queue_initial_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        position_std: float,
        yaw_std: float,
    ) -> None:
        tag_pose = (x, y, yaw)
        if self.scan_support is not None:
            ok, result, detail = self.scan_support.match(
                tag_pose,
                self.scan_refine_xy_window,
                self.scan_refine_yaw_window,
                self.scan_minimum_inlier_ratio,
                self.scan_maximum_ambiguity,
            )
            if result is None and not ok:
                # Map/scan/TF not ready yet: keep sampling and try again.
                self.status = f'AprilTag pose ready; lidar check {detail}.'
                self.scan_retry_after = self._now() + 1.0
                self.pose_samples.clear()
                return
            if not ok:
                self.status = (
                    f'AprilTag pose x={x:.2f}, y={y:.2f}, '
                    f'yaw={math.degrees(yaw):.0f} deg rejected: {detail}. '
                    'Check that the tag has not moved; retrying.'
                )
                self.get_logger().warn(self.status)
                self.scan_retry_after = self._now() + self.scan_retry_interval
                self.manual_localization_requested = False
                self.pose_samples.clear()
                return
            self.get_logger().info(
                f'Lidar refined tag pose by '
                f'{math.hypot(result.x - x, result.y - y):.3f} m / '
                f'{math.degrees(angle_difference(result.yaw, yaw)):.1f} deg ({detail}).'
            )
            x, y, yaw = result.x, result.y, result.yaw
            position_std = 0.0
            yaw_std = 0.0
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        position_variance = max(
            self.initial_position_stddev ** 2, position_std * position_std
        )
        yaw_variance = max(self.initial_yaw_stddev ** 2, yaw_std * yaw_std)
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[35] = yaw_variance
        self.pending_initial_pose = message
        self.pending_publish_count = 3
        self.pending_next_publish = 0.0
        self.auto_initialization_complete = True
        self.awaiting_amcl_validation = True
        self.amcl_validation_deadline = self._now() + 5.0
        self.manual_localization_requested = False
        self.pose_samples.clear()
        self.status = (
            'AprilTag pose accepted'
            f'{" (lidar confirmed)" if self.scan_support else ""}: '
            f'x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg. '
            'Waiting for AMCL.'
        )
        self.get_logger().info(self.status)

    def _publish_pending_initial_pose(self) -> None:
        if self.pending_initial_pose is None or self.pending_publish_count <= 0:
            return
        now = self._now()
        if now < self.pending_next_publish:
            return
        self.pending_initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose_pub.publish(self.pending_initial_pose)
        self.pending_publish_count -= 1
        self.pending_next_publish = now + 0.25
        if self.pending_publish_count == 0:
            self.pending_initial_pose = None

    def _update(self) -> None:
        if (
            self.awaiting_amcl_validation
            and self.pending_publish_count == 0
            and self._now() >= self.amcl_validation_deadline
        ):
            self.awaiting_amcl_validation = False
            self.auto_initialization_complete = False
            self.pose_samples.clear()
            self.status = (
                'AMCL did not validate the tag pose; waiting to sample again.'
            )
        self._update_capture()
        self._collect_localization_candidates()
        self._publish_pending_initial_pose()

    def _publish_state(self) -> None:
        now = self._now()
        message = AprilTagLandmarkArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.map_id = self.active_map
        message.status = self.status
        message.map_pose_available = self._trusted_map_pose_available()
        message.auto_initialization_enabled = self.auto_initialize
        recent_detections = {
            tag_id for tag_id, detection in self.detections.items()
            if now - detection.seen_at <= 5.0
        }
        tag_ids = sorted(set(self.landmarks) | recent_detections)
        for tag_id in tag_ids:
            record = self.landmarks.get(tag_id)
            detection = self.detections.get(tag_id)
            landmark = AprilTagLandmark()
            landmark.id = tag_id
            landmark.saved = record is not None
            landmark.visible = bool(
                detection and now - detection.seen_at <= self.detection_timeout
            )
            landmark.capture_in_progress = bool(
                self.capture and self.capture['tag_id'] == tag_id
            )
            if record:
                landmark.name = record.name
                landmark.family = record.family
                landmark.size_m = record.size_m
                landmark.pose = matrix_to_pose(record.matrix)
                landmark.sample_count = record.sample_count
                landmark.translation_std_m = record.translation_std_m
                landmark.rotation_std_rad = record.rotation_std_rad
            elif detection:
                landmark.name = f'tag_{tag_id}'
                landmark.family = detection.family
                landmark.size_m = self.default_size
            if detection:
                landmark.hamming = detection.hamming
                landmark.decision_margin = detection.decision_margin
                if landmark.visible:
                    camera_to_tag, _stamp = self._camera_to_tag(tag_id, detection)
                    if camera_to_tag is not None:
                        landmark.distance_m = float(np.linalg.norm(camera_to_tag[:3, 3]))
            message.landmarks.append(landmark)
        self.landmark_pub.publish(message)
        self.configured_pub.publish(Bool(data=bool(self.landmarks)))
        self._publish_markers(message)

    def _publish_markers(self, state: AprilTagLandmarkArray) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_index = 0
        for landmark in state.landmarks:
            if not landmark.saved:
                continue
            tag_marker = Marker()
            tag_marker.header = state.header
            tag_marker.ns = 'saved_apriltags'
            tag_marker.id = marker_index
            marker_index += 1
            tag_marker.type = Marker.CUBE
            tag_marker.action = Marker.ADD
            tag_marker.pose = landmark.pose
            tag_marker.scale.x = landmark.size_m
            tag_marker.scale.y = landmark.size_m
            tag_marker.scale.z = 0.015
            if landmark.visible:
                tag_marker.color.r = 0.1
                tag_marker.color.g = 1.0
                tag_marker.color.b = 0.2
            else:
                tag_marker.color.r = 0.1
                tag_marker.color.g = 0.5
                tag_marker.color.b = 1.0
            tag_marker.color.a = 0.85
            markers.markers.append(tag_marker)

            label = Marker()
            label.header = state.header
            label.ns = 'saved_apriltag_labels'
            label.id = marker_index
            marker_index += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = landmark.pose
            label.pose.position.z += max(0.18, landmark.size_m * 0.75)
            label.scale.z = 0.16
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'{landmark.name} (ID {landmark.id})'
            markers.markers.append(label)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLandmarkManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
