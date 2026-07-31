"""ROS 2 battery and diagnostics publisher for JUNCTEK KG-F monitors."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from .junctek_kgf import JunctekKGFClient, OUTPUT_STATUS_NAMES


TECHNOLOGIES = {
    'unknown': BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
    'nimh': BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH,
    'lion': BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
    'lipo': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
    'life': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE,
    'nicd': BatteryState.POWER_SUPPLY_TECHNOLOGY_NICD,
    'limn': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIMN,
}


class JunctekBatteryNode(Node):
    """Poll a KG-F module in a worker thread and publish non-blocking ROS data."""

    def __init__(self):
        super().__init__('junctek_battery')
        self.declare_parameter(
            'port', '/dev/junctek',
            ParameterDescriptor(description='Stable KG-F serial device path'),
        )
        self.declare_parameter(
            'address', 1,
            ParameterDescriptor(description='KG-F communication address (1-99)'),
        )
        self.declare_parameter(
            'baudrate', 115200,
            ParameterDescriptor(description='KG-F serial baud rate'),
        )
        self.declare_parameter(
            'poll_rate_hz', 1.0,
            ParameterDescriptor(description='Measurement query rate'),
        )
        self.declare_parameter(
            'settings_refresh_s', 60.0,
            ParameterDescriptor(description='Seconds between R51 settings reads'),
        )
        self.declare_parameter(
            'stale_timeout_s', 5.0,
            ParameterDescriptor(description='Age before data is marked stale'),
        )
        self.declare_parameter(
            'reconnect_delay_s', 2.0,
            ParameterDescriptor(description='Serial reconnect backoff'),
        )
        self.declare_parameter(
            'invert_current_direction', False,
            ParameterDescriptor(
                description='Invert KG-F charge/discharge direction if shunt is reversed'
            ),
        )
        self.declare_parameter(
            'low_percentage', 0.30,
            ParameterDescriptor(description='Diagnostic low-battery threshold, 0..1'),
        )
        self.declare_parameter(
            'critical_percentage', 0.15,
            ParameterDescriptor(
                description='Diagnostic critical-battery threshold, 0..1'
            ),
        )
        self.declare_parameter(
            'technology', 'unknown',
            ParameterDescriptor(
                description='Battery chemistry: unknown, lion, lipo, life, nimh, nicd, limn'
            ),
        )
        self.declare_parameter(
            'location', 'main_battery',
            ParameterDescriptor(description='Battery location label'),
        )
        self.declare_parameter(
            'frame_id', '',
            ParameterDescriptor(description='Optional BatteryState frame ID'),
        )

        self._port = str(self.get_parameter('port').value)
        self._address = int(self.get_parameter('address').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._poll_rate = max(0.1, float(self.get_parameter('poll_rate_hz').value))
        self._settings_refresh = max(
            5.0, float(self.get_parameter('settings_refresh_s').value)
        )
        self._stale_timeout = max(
            1.0, float(self.get_parameter('stale_timeout_s').value)
        )
        self._reconnect_delay = max(
            0.5, float(self.get_parameter('reconnect_delay_s').value)
        )
        self._invert_direction = bool(
            self.get_parameter('invert_current_direction').value
        )
        self._low_percentage = float(
            self.get_parameter('low_percentage').value
        )
        self._critical_percentage = float(
            self.get_parameter('critical_percentage').value
        )
        technology = str(self.get_parameter('technology').value).lower()
        self._technology = TECHNOLOGIES.get(
            technology, BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )
        self._location = str(self.get_parameter('location').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._battery_pub = self.create_publisher(
            BatteryState, '/battery_state', qos
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', qos
        )
        self._state_pub = self.create_publisher(String, '/junctek/state', qos)

        self._data_lock = threading.Lock()
        self._measurement = None
        self._settings = None
        self._identity = None
        self._last_sample_monotonic = 0.0
        self._last_error = 'Waiting for first KG-F response'
        self._connected = False
        self._last_logged_error = ''
        self._stop_event = threading.Event()
        self._worker = threading.Thread(
            target=self._serial_worker,
            name='junctek-kgf-serial',
            daemon=False,
        )
        self._worker.start()
        self._publish_timer = self.create_timer(1.0 / self._poll_rate, self._publish)
        self.get_logger().info(
            f'KG-F battery monitor configured on {self._port}, '
            f'address {self._address}'
        )

    def _serial_worker(self):
        client = JunctekKGFClient(
            port=self._port,
            address=self._address,
            baudrate=self._baudrate,
            timeout=min(1.0, max(0.2, 0.8 / self._poll_rate)),
        )
        next_settings = 0.0
        try:
            while not self._stop_event.is_set():
                try:
                    client.open()
                    if not self._connected:
                        identity = client.read_identity()
                        with self._data_lock:
                            self._identity = identity
                            self._connected = True
                            self._last_error = ''
                        self.get_logger().info(
                            f'Connected to KG-F address {identity.address} '
                            f'on {self._port}'
                        )
                    now = time.monotonic()
                    if now >= next_settings:
                        settings = client.read_settings()
                        with self._data_lock:
                            self._settings = settings
                        next_settings = now + self._settings_refresh
                    measurement = client.read_measurement()
                    with self._data_lock:
                        self._measurement = measurement
                        self._last_sample_monotonic = time.monotonic()
                        self._last_error = ''
                    self._stop_event.wait(1.0 / self._poll_rate)
                except Exception as error:
                    message = f'{type(error).__name__}: {error}'
                    with self._data_lock:
                        self._connected = False
                        self._last_error = message
                    if message != self._last_logged_error:
                        self.get_logger().warning(
                            f'KG-F unavailable on {self._port}: {message}'
                        )
                        self._last_logged_error = message
                    client.close()
                    self._stop_event.wait(self._reconnect_delay)
        finally:
            client.close()

    def _snapshot(self):
        with self._data_lock:
            return (
                self._measurement,
                self._settings,
                self._identity,
                self._last_sample_monotonic,
                self._last_error,
                self._connected,
            )

    def _signed_current(self, measurement) -> float:
        # KG-F direction 0 is the forward/discharge direction. BatteryState
        # requires negative current while discharging and positive while charging.
        current = abs(float(measurement.current_magnitude_a))
        current = -current if measurement.current_direction == 0 else current
        return -current if self._invert_direction else current

    @staticmethod
    def _health(measurement):
        if measurement.output_status == 0:
            return BatteryState.POWER_SUPPLY_HEALTH_GOOD
        if measurement.output_status == 1:
            return BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        if measurement.output_status == 6:
            return BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
        return BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

    @staticmethod
    def _supply_status(current: float, percentage: float):
        if current > 0.05:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if current < -0.05:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if math.isfinite(percentage) and percentage >= 0.995:
            return BatteryState.POWER_SUPPLY_STATUS_FULL
        return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

    def _publish(self):
        (
            measurement,
            settings,
            identity,
            last_sample,
            error,
            connected,
        ) = self._snapshot()
        age = time.monotonic() - last_sample if last_sample else math.inf
        stale = measurement is None or age > self._stale_timeout
        stamp = self.get_clock().now().to_msg()

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.header.frame_id = self._frame_id
        battery.temperature = math.nan
        battery.current = math.nan
        battery.charge = math.nan
        battery.capacity = math.nan
        battery.design_capacity = math.nan
        battery.percentage = math.nan
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = self._technology
        battery.present = not stale
        battery.location = self._location
        if identity is not None:
            battery.serial_number = str(identity.serial_raw)

        diagnostics = DiagnosticStatus()
        diagnostics.name = 'sahabat: JUNCTEK KG-F battery monitor'
        diagnostics.hardware_id = (
            f'KG-F-{identity.serial_raw}' if identity else 'KG-F-unknown'
        )
        state = {
            'connected': bool(connected),
            'stale': bool(stale),
            'port': self._port,
            'address': self._address,
            'age_s': age if math.isfinite(age) else None,
            'error': error,
        }

        if stale:
            battery.voltage = math.nan
            diagnostics.level = DiagnosticStatus.STALE
            diagnostics.message = error or 'KG-F measurements are stale'
        else:
            current = self._signed_current(measurement)
            capacity = settings.capacity_ah if settings else math.nan
            if math.isfinite(capacity) and capacity > 0.0:
                percentage = max(
                    0.0, min(1.0, measurement.remaining_ah / capacity)
                )
            else:
                percentage = math.nan
            battery.voltage = float(measurement.voltage_v)
            battery.temperature = float(measurement.temperature_c)
            battery.current = float(current)
            battery.charge = float(measurement.remaining_ah)
            battery.capacity = float(capacity)
            battery.design_capacity = float(capacity)
            battery.percentage = float(percentage)
            battery.power_supply_status = self._supply_status(
                current, percentage
            )
            battery.power_supply_health = self._health(measurement)

            diagnostics.level = DiagnosticStatus.OK
            diagnostics.message = measurement.output_status_name
            if measurement.output_status != 0:
                diagnostics.level = DiagnosticStatus.ERROR
            elif math.isfinite(percentage):
                if percentage <= self._critical_percentage:
                    diagnostics.level = DiagnosticStatus.ERROR
                    diagnostics.message = 'Critical battery level'
                elif percentage <= self._low_percentage:
                    diagnostics.level = DiagnosticStatus.WARN
                    diagnostics.message = 'Low battery level'

            state.update(measurement.as_dict())
            state['signed_current_a'] = current
            state['signed_power_w'] = measurement.voltage_v * current
            state['percentage'] = (
                percentage if math.isfinite(percentage) else None
            )
            state['capacity_ah'] = capacity if math.isfinite(capacity) else None
            if settings is not None:
                state['settings'] = settings.as_dict()
            if identity is not None:
                state['identity'] = identity.as_dict()

        diagnostics.values = [
            KeyValue(key='port', value=self._port),
            KeyValue(key='address', value=str(self._address)),
            KeyValue(key='connected', value=str(bool(connected)).lower()),
            KeyValue(
                key='sample_age_s',
                value=f'{age:.2f}' if math.isfinite(age) else 'never',
            ),
            KeyValue(key='voltage_v', value=f'{battery.voltage:.2f}'),
            KeyValue(key='current_a', value=f'{battery.current:.2f}'),
            KeyValue(key='remaining_ah', value=f'{battery.charge:.3f}'),
            KeyValue(key='capacity_ah', value=f'{battery.capacity:.1f}'),
            KeyValue(
                key='percentage',
                value=(
                    f'{battery.percentage * 100.0:.1f}'
                    if math.isfinite(battery.percentage) else 'unknown'
                ),
            ),
        ]
        if measurement is not None:
            diagnostics.values.extend([
                KeyValue(
                    key='output_status',
                    value=OUTPUT_STATUS_NAMES.get(
                        measurement.output_status,
                        str(measurement.output_status),
                    ),
                ),
                KeyValue(
                    key='temperature_c',
                    value=f'{measurement.temperature_c:.1f}',
                ),
                KeyValue(
                    key='power_w',
                    value=f'{measurement.voltage_v * self._signed_current(measurement):.2f}',
                ),
                KeyValue(
                    key='remaining_time_min',
                    value=str(measurement.battery_life_min),
                ),
                KeyValue(
                    key='internal_resistance_ohm',
                    value=f'{measurement.internal_resistance_ohm:.5f}',
                ),
            ])
        if error:
            diagnostics.values.append(KeyValue(key='last_error', value=error))

        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = stamp
        diagnostic_array.status = [diagnostics]
        self._battery_pub.publish(battery)
        self._diagnostics_pub.publish(diagnostic_array)
        state_message = String()
        state_message.data = json.dumps(state, allow_nan=False)
        self._state_pub.publish(state_message)

    def destroy_node(self):
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JunctekBatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
