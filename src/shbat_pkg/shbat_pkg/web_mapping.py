#!/usr/bin/env python3
"""Start a new SLAM mapping session operated from the browser console.

Runs the canonical ``navigation.launch.py mode:=mapping`` stack plus the
``operator_backend`` (control lease, E-stop, browser teleop through
``/cmd_vel_remote``) and ``web_console``. RViz and the Tk mapping panel are
off by default because the console replaces them; enable with --rviz and
--panel.
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from shbat_pkg.live_waypoint_editor import _lan_addresses, _terminate


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Start SLAM mapping with the browser operator console.'
    )
    parser.add_argument('--maps-directory', default='~/sahabat_ws/maps')
    parser.add_argument('--web-port', type=int, default=8088)
    parser.add_argument(
        '--rviz', action='store_true', help='Also open RViz on the robot.')
    parser.add_argument(
        '--panel', action='store_true',
        help='Also open the Tk mapping control panel on the robot.')
    args = parser.parse_args(argv)

    maps_directory = Path(args.maps_directory).expanduser()
    print('Starting Sahabat NEW MAPPING with the web console.', flush=True)
    print(f'Maps are saved to: {maps_directory}', flush=True)

    mapping_cmd = [
        'ros2', 'launch', 'shbat_pkg', 'navigation.launch.py',
        'mode:=mapping',
        'use_zed:=false',
        f'use_rviz:={str(args.rviz).lower()}',
        f'use_mapping_panel:={str(args.panel).lower()}',
    ]
    # No active map: the backend serves the lease, E-stop and teleop only.
    backend_cmd = [
        'ros2', 'run', 'shbat_pkg', 'operator_backend',
        '--ros-args',
        '-p', f'maps_directory:={maps_directory}',
    ]
    web_cmd = [
        'ros2', 'run', 'shbat_pkg', 'web_console',
        '--ros-args',
        '-p', f'port:={args.web_port}',
        '-p', f'maps_directory:={maps_directory}',
    ]

    processes = []
    optional = []
    stopping = False

    def handle_signal(_signum, _frame):
        nonlocal stopping
        stopping = True
        _terminate(processes + optional)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        processes.append(subprocess.Popen(mapping_cmd, env=os.environ.copy()))
        time.sleep(2.0)
        processes.append(subprocess.Popen(backend_cmd))
        # Not in ``processes``: a web console failure must not stop mapping.
        optional.append(subprocess.Popen(web_cmd))
        for address in _lan_addresses():
            print(f'Web console: http://{address}:{args.web_port}/', flush=True)
        print('Web console has no authentication; trusted network only.', flush=True)
        print('Open the Map tab to save the map when done.', flush=True)

        while not stopping:
            for process in processes:
                if process.poll() is not None:
                    _terminate(processes + optional)
                    return process.returncode or 0
            time.sleep(0.25)
    finally:
        _terminate(processes + optional)
    return 0


if __name__ == '__main__':
    sys.exit(main())
