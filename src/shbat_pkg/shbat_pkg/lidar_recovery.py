#!/usr/bin/env python3
"""Inspect and recover an RPLIDAR S2 without starting the robot."""

import argparse
import glob
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

import serial
from serial.tools import list_ports


SYNC_BYTE = 0xA5
SYNC_BYTE_2 = 0x5A
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
ANS_TYPE_INFO = 0x04
ANS_TYPE_HEALTH = 0x06

HEALTH_NAMES = {
    0: 'OK',
    1: 'WARNING',
    2: 'ERROR',
}

LogFunction = Callable[[str], None]


@dataclass
class Health:
    """Decoded RPLIDAR health response."""

    status: int
    error_code: int

    @property
    def name(self) -> str:
        """Return the documented name for the health status."""
        return HEALTH_NAMES.get(self.status, f'UNKNOWN({self.status})')


def discover_port(serial_id: str) -> str:
    """Find a serial device by USB serial number, preferring by-id paths."""
    by_id_matches = sorted(
        glob.glob(f'/dev/serial/by-id/*{serial_id}*')
    )
    if by_id_matches:
        return by_id_matches[0]

    for port in list_ports.comports():
        if port.serial_number == serial_id:
            return port.device

    raise RuntimeError(
        f'No serial device found with USB serial {serial_id!r}. '
        'Use --port to select it explicitly.'
    )


def _read_exact(connection: serial.Serial, size: int) -> bytes:
    """Read exactly size bytes or raise a useful timeout error."""
    data = connection.read(size)
    if len(data) != size:
        raise TimeoutError(
            f'Expected {size} response bytes, received {len(data)}'
        )
    return data


def _read_descriptor(connection: serial.Serial) -> tuple[int, int]:
    """Read an RPLIDAR response descriptor and return payload size and type."""
    deadline = time.monotonic() + connection.timeout
    previous = None

    while time.monotonic() < deadline:
        current = connection.read(1)
        if not current:
            continue
        value = current[0]
        if previous == SYNC_BYTE and value == SYNC_BYTE_2:
            descriptor_tail = _read_exact(connection, 5)
            size_and_mode = int.from_bytes(descriptor_tail[:4], 'little')
            payload_size = size_and_mode & 0x3FFFFFFF
            response_type = descriptor_tail[4]
            return payload_size, response_type
        previous = value

    raise TimeoutError('No RPLIDAR response descriptor received')


def _request(
    connection: serial.Serial,
    command: int,
    expected_type: int,
    expected_size: int,
) -> bytes:
    """Send a response-bearing command and return its payload."""
    connection.reset_input_buffer()
    connection.write(bytes((SYNC_BYTE, command)))
    connection.flush()

    payload_size, response_type = _read_descriptor(connection)
    if response_type != expected_type:
        raise RuntimeError(
            f'Unexpected response type 0x{response_type:02X}; '
            f'expected 0x{expected_type:02X}'
        )
    if payload_size != expected_size:
        raise RuntimeError(
            f'Unexpected payload size {payload_size}; expected {expected_size}'
        )
    return _read_exact(connection, payload_size)


def get_health(connection: serial.Serial) -> Health:
    """Query and decode the device health response."""
    payload = _request(
        connection,
        CMD_GET_HEALTH,
        expected_type=ANS_TYPE_HEALTH,
        expected_size=3,
    )
    return Health(
        status=payload[0],
        error_code=int.from_bytes(payload[1:3], 'little'),
    )


def get_device_info(connection: serial.Serial) -> str:
    """Query device information and return a printable summary."""
    payload = _request(
        connection,
        CMD_GET_INFO,
        expected_type=ANS_TYPE_INFO,
        expected_size=20,
    )
    model = payload[0]
    firmware = int.from_bytes(payload[1:3], 'little')
    hardware = payload[3]
    serial_number = payload[4:20].hex().upper()
    return (
        f'model={model} firmware={firmware >> 8}.{firmware & 0xFF:02d} '
        f'hardware={hardware} serial={serial_number}'
    )


def send_stop(connection: serial.Serial) -> None:
    """Stop scan streaming before recovery."""
    connection.write(bytes((SYNC_BYTE, CMD_STOP)))
    connection.flush()
    time.sleep(0.1)
    connection.reset_input_buffer()


def send_reset(connection: serial.Serial, settle_seconds: float) -> None:
    """Issue the official reset command and wait for firmware reboot."""
    connection.write(bytes((SYNC_BYTE, CMD_RESET)))
    connection.flush()
    time.sleep(settle_seconds)
    connection.reset_input_buffer()


def cycle_dtr(
    connection: serial.Serial,
    off_seconds: float,
    settle_seconds: float,
) -> None:
    """Optionally cycle adapter DTR for custom cable power/motor wiring."""
    connection.dtr = False
    time.sleep(off_seconds)
    connection.dtr = True
    time.sleep(settle_seconds)
    connection.reset_input_buffer()


def print_health(health: Health, prefix: str = 'Health') -> None:
    """Print a health response including its device error code."""
    print(
        f'{prefix}: {health.name} ({health.status}), '
        f'error_code=0x{health.error_code:04X}'
    )


def health_summary(health: Health, prefix: str = 'Health') -> str:
    """Return a health response as one display-friendly line."""
    return (
        f'{prefix}: {health.name} ({health.status}), '
        f'error_code=0x{health.error_code:04X}'
    )


def recovery_guidance(health: Health) -> list[str]:
    """Return hardware guidance for a health response."""
    if health.status != 2:
        return []

    lines = [
        'Protection Stop is still active after the requested operation.',
        (
            'Verify 4.9-5.2 V at the LIDAR connector under load, startup '
            'current capacity of at least 1.5 A, and low supply ripple.'
        ),
    ]
    if health.error_code == 0x0004:
        lines.append(
            'Health error 0x0004 has been reported for S2 voltage '
            'protection; the public Slamtec protocol does not publish a '
            'model-specific error-code table.'
        )
    lines.append(
        'DTR does not remove USB VBUS power. If voltage is in specification, '
        'disconnect power physically before retrying.'
    )
    return lines


def verify_stable_health(
    connection: serial.Serial,
    samples: int = 4,
    interval_seconds: float = 1.0,
    log: LogFunction = print,
) -> Health:
    """Require consecutive healthy replies before reporting device health."""
    if samples < 1:
        raise ValueError('Health verification samples must be at least 1')

    health: Optional[Health] = None
    for sample in range(1, samples + 1):
        health = get_health(connection)
        log(
            '[DEVICE] '
            + health_summary(
                health,
                prefix=f'Health sample {sample}/{samples}',
            )
        )
        if health.status != 0:
            return health
        if sample < samples:
            time.sleep(interval_seconds)

    if health is None:
        raise RuntimeError('No health response received')
    return health


def print_recovery_guidance(health: Health) -> None:
    """Print hardware guidance when protection stop remains active."""
    for line in recovery_guidance(health):
        print(line)


def recover(
    connection: serial.Serial,
    attempts: int,
    settle_seconds: float,
    use_dtr_cycle: bool,
    dtr_off_seconds: float,
    log: LogFunction = print,
) -> Health:
    """Retry bounded STOP/RESET/health cycles and return final health."""
    health: Optional[Health] = None

    for attempt in range(1, attempts + 1):
        try:
            health = verify_stable_health(connection, log=log)
            if health.status == 0:
                return health
        except (TimeoutError, RuntimeError) as error:
            log(
                f'[ERROR] Attempt {attempt}/{attempts}: '
                f'health query failed: {error}'
            )

        if attempt == attempts:
            break

        log(
            f'[ACTION] Recovery attempt {attempt + 1}/{attempts}: '
            'sending STOP and RESET'
        )
        send_stop(connection)
        send_reset(connection, settle_seconds)

        if use_dtr_cycle:
            log('[ACTION] Cycling DTR (custom-adapter recovery enabled)')
            cycle_dtr(connection, dtr_off_seconds, settle_seconds)

    if health is None:
        raise RuntimeError('No valid health response received')
    return health


def parse_arguments() -> argparse.Namespace:
    """Parse command-line options."""
    parser = argparse.ArgumentParser(
        description=(
            'Query or recover an RPLIDAR S2 without starting ROS nodes.'
        )
    )
    parser.add_argument(
        '--action',
        choices=('check', 'reset', 'recover'),
        default='check',
        help='check once, reset once, or retry bounded recovery',
    )
    parser.add_argument('--port', help='explicit serial device path')
    parser.add_argument(
        '--serial-id',
        default='A5069RR4',
        help='USB serial used for automatic discovery',
    )
    parser.add_argument('--baud', type=int, default=1_000_000)
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=1.0)
    parser.add_argument('--settle-seconds', type=float, default=2.0)
    parser.add_argument('--dtr-cycle', action='store_true')
    parser.add_argument('--dtr-off-seconds', type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    """Run the selected health or recovery operation."""
    arguments = parse_arguments()
    if arguments.attempts < 1:
        print('--attempts must be at least 1', file=sys.stderr)
        return 2

    try:
        port = arguments.port or discover_port(arguments.serial_id)
        print(f'Using RPLIDAR port: {port}')
        print(f'Baud rate: {arguments.baud}')

        with serial.Serial(
            port=port,
            baudrate=arguments.baud,
            timeout=arguments.timeout,
            write_timeout=arguments.timeout,
            exclusive=True,
        ) as connection:
            print(f'Device: {get_device_info(connection)}')

            if arguments.action == 'check':
                health = verify_stable_health(connection)
            elif arguments.action == 'reset':
                send_stop(connection)
                send_reset(connection, arguments.settle_seconds)
                health = verify_stable_health(connection)
            else:
                health = recover(
                    connection,
                    attempts=arguments.attempts,
                    settle_seconds=arguments.settle_seconds,
                    use_dtr_cycle=arguments.dtr_cycle,
                    dtr_off_seconds=arguments.dtr_off_seconds,
                )

            print_health(health, prefix='Final health')
            print_recovery_guidance(health)
            return 0 if health.status == 0 else 1

    except (
        OSError,
        serial.SerialException,
        TimeoutError,
        RuntimeError,
    ) as error:
        print(f'RPLIDAR recovery failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
