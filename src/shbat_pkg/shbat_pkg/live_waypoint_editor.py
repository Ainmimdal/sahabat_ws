#!/usr/bin/env python3
"""Start live robot operations with the RViz waypoint editor as the only RViz."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def _valid_map_id(value: str) -> bool:
    return bool(value) and all(ch.isalnum() or ch in '_-' for ch in value)


def _discover_maps(maps_directory: Path):
    maps = []
    for path in sorted(maps_directory.glob('*.yaml')):
        if path.name in ('dock.yaml',) or path.name.endswith('_waypoints.yaml'):
            continue
        if path.name.endswith('.metadata.yaml'):
            continue
        maps.append(path.stem)
    for path in sorted(maps_directory.glob('*/map.yaml')):
        map_id = path.parent.name
        if map_id not in maps:
            maps.append(map_id)
    return maps


def _select_map_id(maps_directory: Path, requested: str) -> str:
    maps = _discover_maps(maps_directory)
    last_path = maps_directory / 'last_selected_map'

    candidates = [requested, os.environ.get('SAHABAT_MAP_ID', '')]
    if last_path.exists():
        try:
            candidates.append(last_path.read_text(encoding='utf-8').strip())
        except OSError:
            pass
    candidates.extend(['rdlfront', *(maps or [])])

    for candidate in candidates:
        if not _valid_map_id(candidate):
            continue
        if candidate in maps or (maps_directory / f'{candidate}.yaml').exists() or (maps_directory / candidate / 'map.yaml').exists():
            try:
                last_path.write_text(candidate + '\n', encoding='utf-8')
            except OSError:
                pass
            return candidate

    raise RuntimeError(f'No map YAML found in {maps_directory}')


def _terminate(processes):
    for process in processes:
        if process.poll() is None:
            process.terminate()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if all(process.poll() is not None for process in processes):
            return
        time.sleep(0.1)
    for process in processes:
        if process.poll() is None:
            process.kill()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            'Start Sahabat Waypoint Editor Live: the main operations stack plus '
            'one waypoint-editor RViz window.'
        )
    )
    parser.add_argument('--maps-directory', default='~/sahabat_ws/maps')
    parser.add_argument('--map-id', default='')
    parser.add_argument('--startup-delay', type=float, default=8.0)
    parser.add_argument(
        '--localization-backend',
        choices=('amcl', 'slam_toolbox'),
        default='amcl',
        help=(
            'Localization backend. Keep the default AMCL for main operations; '
            'slam_toolbox is only for the explicit desktop test launcher.'
        ),
    )
    zed_group = parser.add_mutually_exclusive_group()
    zed_group.add_argument(
        '--use-zed',
        dest='use_zed',
        action='store_true',
        default=True,
        help='Enable the ZED camera in the operations launch (default).',
    )
    zed_group.add_argument(
        '--no-zed',
        dest='use_zed',
        action='store_false',
        help='Disable the ZED camera in the operations launch.',
    )
    args = parser.parse_args(argv)

    maps_directory = Path(args.maps_directory).expanduser()
    map_id = _select_map_id(maps_directory, args.map_id)
    map_stem = maps_directory / map_id

    if args.localization_backend == 'slam_toolbox':
        print('Starting SLAM Toolbox localization TEST workflow.', flush=True)
        print('This is not the normal gallery operations launcher.', flush=True)
        print('Sahabat New Mapping remains the only mapping desktop workflow.', flush=True)
    else:
        print('Starting Sahabat Waypoint Editor Live MAIN OPERATIONS.', flush=True)
    print(f'Localization backend: {args.localization_backend}', flush=True)
    print(f'ZED obstacle input: {"enabled" if args.use_zed else "disabled"}', flush=True)
    print(f'Map: {map_id}', flush=True)
    print(f'Waypoint sets: {maps_directory / "waypoint_sets" / map_id}', flush=True)

    operations_cmd = [
        'ros2', 'launch', 'shbat_pkg', 'operations.launch.py',
        f'map_file:={map_stem}',
        f'maps_directory:={maps_directory}',
        f'map_id:={map_id}',
        'use_rviz:=false',
        'use_waypoint_gui:=false',
        f'use_zed:={str(args.use_zed).lower()}',
        f'localization_backend:={args.localization_backend}',
    ]
    backend_cmd = [
        'ros2', 'run', 'shbat_pkg', 'operator_backend',
        '--ros-args',
        '-p', f'maps_directory:={maps_directory}',
        '-p', f'active_map:={map_id}',
        '-p', f'localization_backend:={args.localization_backend}',
    ]
    editor_cmd = [
        'ros2', 'launch', 'shbat_pkg', 'waypoint_editor.launch.py',
        f'maps_directory:={maps_directory}',
        f'map_id:={map_id}',
        'waypoint_backend:=operator',
        'start_map_server:=false',
    ]

    processes = []
    stopping = False

    def handle_signal(_signum, _frame):
        nonlocal stopping
        stopping = True
        _terminate(processes)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        processes.append(subprocess.Popen(operations_cmd))
        time.sleep(2.0)
        processes.append(subprocess.Popen(backend_cmd))
        time.sleep(max(0.0, args.startup_delay))
        for process in processes:
            if process.poll() is not None:
                return process.returncode or 1
        processes.append(subprocess.Popen(editor_cmd))

        while not stopping:
            for process in processes:
                if process.poll() is not None:
                    _terminate(processes)
                    return process.returncode or 0
            time.sleep(0.25)
    finally:
        _terminate(processes)
    return 0


if __name__ == '__main__':
    sys.exit(main())
