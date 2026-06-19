#!/usr/bin/env python3
"""Select one fresh velocity source and enforce the software E-stop lock."""

from dataclasses import dataclass, field
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@dataclass
class VelocitySource:
    """Last command and receipt time for one command source."""

    priority: int
    timeout: Duration
    command: Twist = field(default_factory=Twist)
    received_at: object = None


class CommandArbiter(Node):
    """Prioritize joystick, remote teleop, and Nav2 velocity commands."""

    def __init__(self) -> None:
        """Create source subscriptions and a fixed-rate safe output."""
        super().__init__('command_arbiter')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('joystick_timeout', 0.30)
        self.declare_parameter('remote_timeout', 0.25)
        self.declare_parameter('navigation_timeout', 0.50)
        self.declare_parameter('recovery_timeout', 0.25)

        self.sources: Dict[str, VelocitySource] = {
            'joystick': VelocitySource(
                100,
                Duration(
                    seconds=float(
                        self.get_parameter('joystick_timeout').value
                    )
                ),
            ),
            'remote': VelocitySource(
                80,
                Duration(
                    seconds=float(self.get_parameter('remote_timeout').value)
                ),
            ),
            'recovery': VelocitySource(
                90,
                Duration(
                    seconds=float(
                        self.get_parameter('recovery_timeout').value
                    )
                ),
            ),
            'navigation': VelocitySource(
                50,
                Duration(
                    seconds=float(
                        self.get_parameter('navigation_timeout').value
                    )
                ),
            ),
        }
        self.emergency_stop = True
        self.source_enabled = {
            'joystick': False,
            'remote': False,
            'navigation': True,
            'recovery': False,
        }
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        estop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        for name, topic in (
            ('joystick', '/cmd_vel_joy'),
            ('remote', '/cmd_vel_remote'),
            ('recovery', '/cmd_vel_recovery'),
            ('navigation', '/cmd_vel_nav_smoothed'),
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda message, source=name: self._command(source, message),
                command_qos,
            )
        self.create_subscription(
            Bool,
            '/emergency_stop',
            self._estop,
            estop_qos,
        )
        self.create_subscription(
            Bool,
            '/operator/local_joystick_active',
            lambda message: self._enable('joystick', message),
            10,
        )
        self.create_subscription(
            Bool,
            '/operator/remote_active',
            lambda message: self._enable('remote', message),
            10,
        )
        self.create_subscription(
            Bool,
            '/localization/recovery_active',
            lambda message: self._enable('recovery', message),
            estop_qos,
        )
        self.output = self.create_publisher(
            Twist,
            '/cmd_vel',
            output_qos,
        )
        self.active_source = self.create_publisher(
            String,
            '/operator/active_velocity_source',
            10,
        )
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._publish)

    def _command(self, source: str, message: Twist) -> None:
        """Record the latest command from a configured source."""
        state = self.sources[source]
        state.command = message
        state.received_at = self.get_clock().now()

    def _estop(self, message: Bool) -> None:
        """Track the latched software E-stop state."""
        self.emergency_stop = message.data

    def _enable(self, source: str, message: Bool) -> None:
        """Mark a source active while it owns velocity control."""
        self.source_enabled[source] = message.data

    def _publish(self) -> None:
        """Publish the highest-priority fresh command or a zero command."""
        selected_name = 'stopped'
        selected = Twist()
        if not self.emergency_stop:
            now = self.get_clock().now()
            if self.source_enabled['recovery']:
                allowed = ('recovery',)
            elif self.source_enabled['joystick']:
                allowed = ('joystick',)
            elif self.source_enabled['remote']:
                allowed = ('remote',)
            else:
                allowed = ('navigation',)
            fresh = [
                (state.priority, name, state)
                for name, state in self.sources.items()
                if name in allowed
                if state.received_at is not None and (
                    now - state.received_at <= state.timeout
                )
            ]
            if fresh:
                _priority, selected_name, state = max(fresh)
                selected = state.command

        self.output.publish(selected)
        self.active_source.publish(String(data=selected_name))


def main(args=None) -> None:
    """Run the command arbiter."""
    rclpy.init(args=args)
    node = CommandArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
