"""JUNCTEK KG-F series ASCII protocol and serial client.

The KG-F measurement module uses a documented 115200 baud ASCII protocol on
its RS485 port.  Read commands are side-effect free.  Write commands are kept
in this shared module so the ROS monitor and the settings GUI use identical
encoding, checksum, and validation rules.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import fcntl
import math
import os
import time
from typing import Iterable, Optional

import serial


BAUDRATE = 115200
OUTPUT_STATUS_NAMES = {
    0: 'on',
    1: 'over-voltage protection',
    2: 'positive over-current protection',
    3: 'under-voltage protection',
    4: 'negative over-current protection',
    5: 'over-power protection',
    6: 'over-temperature protection',
    255: 'off',
}


class JunctekError(RuntimeError):
    """Base class for KG-F communication failures."""


class JunctekBusyError(JunctekError):
    """Raised when another local process owns the serial port."""


class JunctekProtocolError(JunctekError):
    """Raised when a response is malformed or fails checksum validation."""


@dataclass(frozen=True)
class Identity:
    address: int
    model_code: int
    firmware_raw: int
    serial_raw: int

    def as_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class Measurement:
    address: int
    voltage_v: float
    current_magnitude_a: float
    remaining_ah: float
    accumulated_ah: float
    energy_wh: float
    runtime_s: int
    temperature_c: float
    power_magnitude_w: float
    output_status: int
    current_direction: int
    battery_life_min: int
    internal_resistance_ohm: float

    @property
    def output_status_name(self) -> str:
        return OUTPUT_STATUS_NAMES.get(
            self.output_status, f'unknown ({self.output_status})'
        )

    def as_dict(self):
        values = asdict(self)
        values['output_status_name'] = self.output_status_name
        return values


@dataclass(frozen=True)
class Settings:
    address: int
    over_voltage_v: float
    under_voltage_v: float
    positive_over_current_a: float
    negative_over_current_a: float
    over_power_w: float
    over_temperature_c: float
    recovery_time_s: int
    delay_time_s: int
    capacity_ah: float
    voltage_calibration: int
    current_calibration: int
    temperature_calibration: int
    reserved: int
    relay_normally_closed: bool
    current_ratio: int
    voltage_curve_scale: Optional[int]
    current_curve_scale: Optional[int]

    def as_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class SettingSpec:
    function: int
    label: str
    unit: str
    minimum: float
    maximum: float
    scale: float = 1.0
    offset: float = 0.0
    integer: bool = False

    def encode(self, value: float) -> int:
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ValueError(f'{self.label} must be finite')
        if numeric < self.minimum or numeric > self.maximum:
            raise ValueError(
                f'{self.label} must be between {self.minimum} and '
                f'{self.maximum} {self.unit}'.rstrip()
            )
        raw = (numeric + self.offset) * self.scale
        return int(round(raw))

    def decode(self, raw: int) -> float:
        value = (float(raw) / self.scale) - self.offset
        return int(round(value)) if self.integer else value


SETTING_SPECS = {
    'over_voltage_v': SettingSpec(20, 'Over-voltage limit', 'V', 0, 120, 100),
    'under_voltage_v': SettingSpec(21, 'Under-voltage limit', 'V', 0, 120, 100),
    'positive_over_current_a': SettingSpec(
        22, 'Positive over-current limit', 'A', 0, 100, 100
    ),
    'negative_over_current_a': SettingSpec(
        23, 'Negative over-current limit', 'A', 0, 100, 100
    ),
    'over_power_w': SettingSpec(
        24, 'Over-power limit', 'W', 0, 99999.99, 100
    ),
    'over_temperature_c': SettingSpec(
        25, 'Over-temperature limit', '°C', 0, 120, 1, 100
    ),
    'recovery_time_s': SettingSpec(
        26, 'Protection recovery time', 's', 0, 99, integer=True
    ),
    'delay_time_s': SettingSpec(
        27, 'Protection delay', 's', 0, 99, integer=True
    ),
    'capacity_ah': SettingSpec(
        28, 'Battery capacity', 'Ah', 0.1, 9999.9, 10
    ),
    'voltage_calibration': SettingSpec(
        29, 'Voltage calibration', '', -100, 100, 1, 100, True
    ),
    'current_calibration': SettingSpec(
        30, 'Current calibration', '', -100, 100, 1, 100, True
    ),
    'temperature_calibration': SettingSpec(
        31, 'Temperature calibration', '°C', -100, 100, 1, 100, True
    ),
    'relay_normally_closed': SettingSpec(
        34, 'Relay type', '', 0, 1, integer=True
    ),
    'current_ratio': SettingSpec(
        36, 'Current ratio', '', 0, 99, integer=True
    ),
    'voltage_curve_scale': SettingSpec(
        37, 'Voltage curve scale', 'V/div', 1, 99, integer=True
    ),
    'current_curve_scale': SettingSpec(
        38, 'Current curve scale', 'A/div', 1, 99, integer=True
    ),
}


def kgf_checksum(values: Iterable[int]) -> int:
    """Return the KG-F checksum: ``sum(data) % 255 + 1``."""
    return (sum(int(value) for value in values) % 255) + 1


def _validate_address(address: int) -> int:
    address = int(address)
    if address < 0 or address > 99:
        raise ValueError('KG-F address must be between 0 and 99')
    return address


def build_read_command(function: int, address: int = 1) -> bytes:
    """Build a side-effect-free read command for R00, R50, or R51."""
    function = int(function)
    if function not in (0, 50, 51):
        raise ValueError(f'Unsupported KG-F read function R{function:02d}')
    address = _validate_address(address)
    data = 1
    checksum = kgf_checksum([data])
    return f':R{function:02d}={address},{checksum},{data},\r\n'.encode('ascii')


def build_write_command(function: int, raw_value: int, address: int = 1) -> bytes:
    """Build a KG-F write command from an already range-checked raw value."""
    function = int(function)
    address = _validate_address(address)
    raw_value = int(raw_value)
    checksum = kgf_checksum([raw_value])
    return (
        f':W{function:02d}={address},{checksum},{raw_value},\r\n'
    ).encode('ascii')


def parse_response(
    line: str,
    expected_function: Optional[int] = None,
    validate_checksum: bool = False,
):
    """Parse and checksum a ``:rXX=...`` response.

    Returns ``(function, address, payload)`` where payload excludes the address
    and checksum fields.
    """
    clean = line.strip()
    if len(clean) < 8 or not clean.startswith(':r') or '=' not in clean:
        raise JunctekProtocolError(f'Invalid KG-F response: {clean!r}')
    header, body = clean[1:].split('=', 1)
    try:
        function = int(header[1:])
        fields = [int(value) for value in body.split(',') if value != '']
    except ValueError as error:
        raise JunctekProtocolError(
            f'Non-numeric KG-F response: {clean!r}'
        ) from error
    if expected_function is not None and function != int(expected_function):
        raise JunctekProtocolError(
            f'Expected r{int(expected_function):02d}, received r{function:02d}'
        )
    if len(fields) < 3:
        raise JunctekProtocolError(f'Truncated KG-F response: {clean!r}')
    address, checksum, *payload = fields
    # Published KG-F examples are inconsistent: R00 follows the documented
    # ``+ 1`` rule, R51 omits it, and the sample R50 checksum matches neither.
    # Keep reads interoperable by default. Strict validation accepts both
    # documented variants and is useful for controlled tests/captures.
    checksum_variants = {
        sum(payload) % 255,
        kgf_checksum(payload),
    }
    if validate_checksum and checksum not in ({0} | checksum_variants):
        raise JunctekProtocolError(
            f'KG-F checksum mismatch: received {checksum}, '
            f'expected one of {sorted(checksum_variants)}'
        )
    return function, address, payload


def parse_identity(line: str) -> Identity:
    _function, address, payload = parse_response(line, 0)
    if len(payload) < 3:
        raise JunctekProtocolError('KG-F identity response has fewer than 3 fields')
    return Identity(address, payload[0], payload[1], payload[2])


def parse_measurement(line: str) -> Measurement:
    _function, address, values = parse_response(line, 50)
    if len(values) < 12:
        raise JunctekProtocolError(
            f'KG-F measurement response has {len(values)} fields; expected 12'
        )
    return Measurement(
        address=address,
        voltage_v=values[0] / 100.0,
        current_magnitude_a=values[1] / 100.0,
        remaining_ah=values[2] / 1000.0,
        accumulated_ah=values[3] / 1000.0,
        energy_wh=values[4] / 100.0,
        runtime_s=values[5],
        temperature_c=values[6] - 100.0,
        # R50 field 8 is reserved on the KG-F protocol. Calculate power from
        # voltage and current rather than presenting that reserved value.
        power_magnitude_w=(values[0] / 100.0) * (values[1] / 100.0),
        output_status=values[8],
        current_direction=values[9],
        battery_life_min=values[10],
        internal_resistance_ohm=values[11] / 100000.0,
    )


def parse_settings(line: str) -> Settings:
    _function, address, values = parse_response(line, 51)
    if len(values) < 15:
        raise JunctekProtocolError(
            f'KG-F settings response has {len(values)} fields; expected at least 15'
        )
    return Settings(
        address=address,
        over_voltage_v=values[0] / 100.0,
        under_voltage_v=values[1] / 100.0,
        positive_over_current_a=values[2] / 100.0,
        negative_over_current_a=values[3] / 100.0,
        over_power_w=values[4] / 100.0,
        over_temperature_c=values[5] - 100.0,
        recovery_time_s=values[6],
        delay_time_s=values[7],
        capacity_ah=values[8] / 10.0,
        voltage_calibration=values[9] - 100,
        current_calibration=values[10] - 100,
        temperature_calibration=values[11] - 100,
        reserved=values[12],
        relay_normally_closed=bool(values[13]),
        current_ratio=values[14],
        voltage_curve_scale=values[15] if len(values) > 15 else None,
        current_curve_scale=values[16] if len(values) > 16 else None,
    )


class JunctekKGFClient:
    """Exclusive serial owner for one KG-F measurement module."""

    def __init__(
        self,
        port: str = '/dev/junctek',
        address: int = 1,
        baudrate: int = BAUDRATE,
        timeout: float = 0.35,
    ):
        self.port = port
        self.address = _validate_address(address)
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self._serial = None
        self._lock_file = None

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    def open(self):
        if self.is_open:
            return
        lock_name = os.path.basename(os.path.realpath(self.port)) or 'junctek'
        lock_path = f'/tmp/sahabat_junctek_{lock_name}.lock'
        self._lock_file = open(lock_path, 'a+', encoding='utf-8')
        try:
            fcntl.flock(
                self._lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB
            )
        except BlockingIOError as error:
            self._lock_file.close()
            self._lock_file = None
            raise JunctekBusyError(
                f'{self.port} is already owned by another JUNCTEK process'
            ) from error
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=self.timeout,
                exclusive=True,
            )
        except Exception:
            self.close()
            raise

    def close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        if self._lock_file is not None:
            try:
                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
            finally:
                self._lock_file.close()
                self._lock_file = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()

    def _query(self, function: int) -> str:
        self.open()
        self._serial.reset_input_buffer()
        self._serial.write(build_read_command(function, self.address))
        self._serial.flush()
        deadline = time.monotonic() + self.timeout
        expected = f':r{function:02d}='
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='replace').strip()
            if line.startswith(expected):
                return line
        raise TimeoutError(
            f'No r{function:02d} response from KG-F address {self.address} '
            f'on {self.port}'
        )

    def read_identity(self) -> Identity:
        return parse_identity(self._query(0))

    def read_measurement(self) -> Measurement:
        return parse_measurement(self._query(50))

    def read_settings(self) -> Settings:
        return parse_settings(self._query(51))

    def write_raw(self, function: int, raw_value: int):
        """Send one explicit write. Callers must confirm intent first."""
        self.open()
        self._serial.reset_input_buffer()
        self._serial.write(
            build_write_command(function, raw_value, self.address)
        )
        self._serial.flush()
        time.sleep(0.12)
        response = self._serial.read_all().decode('ascii', errors='replace')
        return response.strip()

    def write_setting(self, name: str, value: float):
        spec = SETTING_SPECS.get(name)
        if spec is None:
            raise ValueError(f'Unsupported KG-F setting {name!r}')
        raw_value = spec.encode(value)
        return self.write_raw(spec.function, raw_value)
