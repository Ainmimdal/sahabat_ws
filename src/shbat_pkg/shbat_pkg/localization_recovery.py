#!/usr/bin/env python3
"""Global AMCL relocalization followed by a bounded in-place scan rotation."""

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger


class RecoveryState(Enum):
    IDLE = 'idle'
    WAITING_FOR_AMCL = 'waiting_for_amcl'
    ROTATING = 'rotating'


class LocalizationRecovery(Node):
    """Own a short automatic rotation while AMCL converges globally."""

    def __init__(self):
        super().__init__('localization_recovery')

        self.declare_parameter('angular_speed', 0.25)
        self.declare_parameter('max_duration', 60.0)
        self.declare_parameter('minimum_rotation_time', 8.0)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('position_variance_threshold', 0.20)
        self.declare_parameter('yaw_variance_threshold', 0.20)
        self.declare_parameter('consecutive_good_updates', 8)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.max_duration = float(self.get_parameter('max_duration').value)
        self.minimum_rotation_time = float(
            self.get_parameter('minimum_rotation_time').value
        )
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.position_variance_threshold = float(
            self.get_parameter('position_variance_threshold').value
        )
        self.yaw_variance_threshold = float(
            self.get_parameter('yaw_variance_threshold').value
        )
        self.consecutive_good_updates = int(
            self.get_parameter('consecutive_good_updates').value
        )

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.active_pub = self.create_publisher(
            Bool, '/localization/recovery_active', state_qos
        )
        self.status_pub = self.create_publisher(
            String, '/localization/recovery_status', state_qos
        )
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10,
        )
        estop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, estop_qos
        )

        self.global_client = self.create_client(
            Empty, '/reinitialize_global_localization'
        )
        self.create_service(
            Trigger, '/localization/start_recovery', self.start_callback
        )
        self.create_service(
            Trigger, '/localization/stop_recovery', self.stop_callback
        )
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.state = RecoveryState.IDLE
        self.global_future = None
        self.started_at = None
        self.last_scan_at = None
        self.estopped = False
        self.good_updates = 0
        self.last_variances = None
        self.publish_state('Ready. Press Global Relocalize when stationary.')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, _msg):
        self.last_scan_at = self.now_seconds()

    def estop_callback(self, msg):
        self.estopped = msg.data
        if self.estopped and self.state != RecoveryState.IDLE:
            self.finish('Stopped: emergency stop is active.', warning=True)

    def pose_callback(self, msg):
        covariance = msg.pose.covariance
        self.last_variances = (covariance[0], covariance[7], covariance[35])
        if self.state != RecoveryState.ROTATING:
            return

        x_var, y_var, yaw_var = self.last_variances
        finite = all(math.isfinite(value) for value in self.last_variances)
        good = (
            finite
            and x_var <= self.position_variance_threshold
            and y_var <= self.position_variance_threshold
            and yaw_var <= self.yaw_variance_threshold
        )
        self.good_updates = self.good_updates + 1 if good else 0

    def start_callback(self, _request, response):
        if self.state != RecoveryState.IDLE:
            response.success = False
            response.message = 'Localization recovery is already running.'
            return response
        if self.estopped:
            response.success = False
            response.message = 'Clear E-stop before starting recovery.'
            return response
        if self.last_scan_at is None or (
            self.now_seconds() - self.last_scan_at > self.scan_timeout
        ):
            response.success = False
            response.message = 'No fresh lidar scan; recovery was not started.'
            return response
        if not self.global_client.service_is_ready():
            response.success = False
            response.message = 'AMCL global-localization service is unavailable.'
            return response

        self.good_updates = 0
        self.last_variances = None
        self.state = RecoveryState.WAITING_FOR_AMCL
        self.global_future = self.global_client.call_async(Empty.Request())
        self.active_pub.publish(Bool(data=True))
        self.publish_state('Spreading AMCL particles across the map...')
        response.success = True
        response.message = 'Global relocalization started.'
        return response

    def stop_callback(self, _request, response):
        if self.state == RecoveryState.IDLE:
            response.success = False
            response.message = 'Localization recovery is not running.'
            return response
        self.finish('Stopped by operator.')
        response.success = True
        response.message = 'Localization recovery stopped.'
        return response

    def timer_callback(self):
        if self.state == RecoveryState.WAITING_FOR_AMCL:
            if self.global_future is None or not self.global_future.done():
                return
            try:
                self.global_future.result()
            except Exception as exc:  # ROS service transport failure
                self.finish(f'AMCL reinitialization failed: {exc}', warning=True)
                return
            self.state = RecoveryState.ROTATING
            self.started_at = self.now_seconds()
            self.publish_state('Rotating: matching lidar scan to the map...')
            return

        if self.state != RecoveryState.ROTATING:
            return

        elapsed = self.now_seconds() - self.started_at
        if self.last_scan_at is None or (
            self.now_seconds() - self.last_scan_at > self.scan_timeout
        ):
            self.finish('Stopped: lidar scan became stale.', warning=True)
            return
        if elapsed >= self.max_duration:
            self.finish(
                'Stopped after timeout; localization is still uncertain.',
                warning=True,
            )
            return
        if (
            elapsed >= self.minimum_rotation_time
            and self.good_updates >= self.consecutive_good_updates
        ):
            self.finish('Localized: lidar match is stable.')
            return

        command = Twist()
        command.angular.z = self.angular_speed
        self.cmd_pub.publish(command)

    def publish_state(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def finish(self, text, warning=False):
        self.publish_zero()
        self.state = RecoveryState.IDLE
        self.global_future = None
        self.started_at = None
        self.good_updates = 0
        self.active_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data=text))
        if warning:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def destroy_node(self):
        self.publish_zero()
        self.active_pub.publish(Bool(data=False))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationRecovery()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
