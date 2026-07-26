"""Map-scoped storage for named waypoint sets and a shared dock pose."""

from contextlib import contextmanager
import fcntl
from pathlib import Path
import re
import shutil
import uuid

import yaml


VALID_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')


def make_id(name: str) -> str:
    """Convert a display name to a stable filesystem-safe identifier."""
    value = re.sub(r'[^A-Za-z0-9_-]+', '-', name.strip()).strip('-_').lower()
    if not value:
        value = 'set'
    return value[:64]


class WaypointStore:
    """Own waypoint-set files beneath one saved-map directory."""

    def __init__(self, map_directory, sets_directory=None, legacy_path=None, dock_path=None):
        self.map_directory = Path(map_directory).expanduser()
        self.sets_directory = (
            Path(sets_directory).expanduser()
            if sets_directory is not None
            else self.map_directory / 'waypoint_sets'
        )
        self.index_path = self.sets_directory / 'index.yaml'
        self.dock_path = (
            Path(dock_path).expanduser()
            if dock_path is not None
            else self.map_directory / 'dock.yaml'
        )
        self.legacy_path = (
            Path(legacy_path).expanduser()
            if legacy_path is not None
            else self.map_directory / 'waypoints.yaml'
        )

    def ensure_migrated(self):
        """Copy the old single list into Default without altering the source."""
        self.sets_directory.mkdir(parents=True, exist_ok=True)
        if any(self._set_paths()):
            return
        data = self._read_yaml(self.legacy_path)
        waypoints = list(data.get('waypoints', []))
        dock = next(
            (item for item in waypoints
             if str(item.get('name', '')).strip().lower() == 'dock'),
            None,
        )
        route = [item for item in waypoints if item is not dock]
        if dock is not None and not self.dock_path.exists():
            self.save_dock(dock)
        self._write_set('default', 'Default', int(data.get('revision', 0)), route)
        self._write_index('default')

    def list_sets(self):
        self.ensure_migrated()
        result = []
        for path in self._set_paths():
            data = self._read_yaml(path)
            result.append({
                'id': path.stem,
                'name': str(data.get('name') or path.stem),
                'revision': int(data.get('revision', 0)),
                'waypoint_count': len(data.get('waypoints', [])),
            })
        return sorted(result, key=lambda item: item['name'].casefold())

    def active_set_id(self):
        sets = self.list_sets()
        available = {item['id'] for item in sets}
        selected = str(self._read_yaml(self.index_path).get('active_set_id', ''))
        if selected in available:
            return selected
        selected = sets[0]['id'] if sets else 'default'
        self._write_index(selected)
        return selected

    def read_set(self, set_id=''):
        self.ensure_migrated()
        set_id = set_id or self.active_set_id()
        self._validate_id(set_id)
        path = self.sets_directory / f'{set_id}.yaml'
        if not path.exists():
            raise ValueError(f'Waypoint set does not exist: {set_id}')
        data = self._read_yaml(path)
        return int(data.get('revision', 0)), list(data.get('waypoints', []))

    def read_graph(self, set_id=''):
        """Load stops plus preferred route segments for one waypoint set."""
        self.ensure_migrated()
        set_id = set_id or self.active_set_id()
        self._validate_id(set_id)
        path = self.sets_directory / f'{set_id}.yaml'
        if not path.exists():
            raise ValueError(f'Waypoint set does not exist: {set_id}')
        data = self._read_yaml(path)
        return (
            int(data.get('revision', 0)),
            list(data.get('waypoints', [])),
            list(data.get('segments', [])),
            dict(data.get('settings', {})),
        )

    def save_set(self, set_id, expected_revision, waypoints):
        self.ensure_migrated()
        self._validate_id(set_id)
        with self._lock():
            path = self.sets_directory / f'{set_id}.yaml'
            if not path.exists():
                raise ValueError(f'Waypoint set does not exist: {set_id}')
            data = self._read_yaml(path)
            revision = int(data.get('revision', 0))
            if revision != expected_revision:
                raise RuntimeError(f'revision:{revision}')
            revision += 1
            self._write_set(
                set_id,
                str(data.get('name') or set_id),
                revision,
                waypoints,
                data.get('segments', []),
                data.get('settings', {}),
            )
            return revision

    def save_graph(
        self, set_id, expected_revision, waypoints, segments, settings=None
    ):
        """Save stops and route segments with optimistic revision checking."""
        self.ensure_migrated()
        self._validate_id(set_id)
        with self._lock():
            path = self.sets_directory / f'{set_id}.yaml'
            if not path.exists():
                raise ValueError(f'Waypoint set does not exist: {set_id}')
            data = self._read_yaml(path)
            revision = int(data.get('revision', 0))
            if revision != expected_revision:
                raise RuntimeError(f'revision:{revision}')
            revision += 1
            self._write_set(
                set_id,
                str(data.get('name') or set_id),
                revision,
                waypoints,
                segments,
                settings if settings is not None else data.get('settings', {}),
            )
            return revision

    def create_set(self, name):
        self.ensure_migrated()
        base = make_id(name)
        set_id = base
        suffix = 2
        while (self.sets_directory / f'{set_id}.yaml').exists():
            set_id = f'{base[:60]}-{suffix}'
            suffix += 1
        self._write_set(set_id, name.strip() or 'Untitled set', 0, [])
        self._write_index(set_id)
        return set_id

    def rename_set(self, set_id, name):
        revision, waypoints, segments, settings = self.read_graph(set_id)
        self._write_set(
            set_id,
            name.strip() or set_id,
            revision,
            waypoints,
            segments,
            settings,
        )

    def delete_set(self, set_id):
        self.ensure_migrated()
        sets = self.list_sets()
        if len(sets) <= 1:
            raise ValueError('A map must keep at least one waypoint set')
        self._validate_id(set_id)
        path = self.sets_directory / f'{set_id}.yaml'
        if not path.exists():
            raise ValueError(f'Waypoint set does not exist: {set_id}')
        archive = self.sets_directory / 'archive'
        archive.mkdir(exist_ok=True)
        destination = archive / f'{set_id}-{uuid.uuid4().hex[:8]}.yaml'
        shutil.move(str(path), str(destination))
        active = self.active_set_id()
        if active == set_id:
            active = self.list_sets()[0]['id']
            self._write_index(active)
        return active

    def select_set(self, set_id):
        self.read_set(set_id)
        self._write_index(set_id)

    def load_dock(self):
        self.ensure_migrated()
        data = self._read_yaml(self.dock_path)
        return data.get('dock')

    def save_dock(self, dock):
        self.map_directory.mkdir(parents=True, exist_ok=True)
        self.dock_path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'version': 1,
            'dock': {
                'id': str(dock.get('id') or 'dock'),
                'name': 'dock',
                'x': round(float(dock.get('x', 0.0)), 3),
                'y': round(float(dock.get('y', 0.0)), 3),
                'yaw': round(float(dock.get('yaw', 0.0)), 3),
            },
        }
        self._write_yaml(self.dock_path, data)

    def _set_paths(self):
        if not self.sets_directory.exists():
            return []
        return [
            path for path in self.sets_directory.glob('*.yaml')
            if path.name != 'index.yaml'
        ]

    def _write_set(
        self, set_id, name, revision, waypoints, segments=None, settings=None
    ):
        self._validate_id(set_id)
        normalized = [self._normalize_waypoint(item) for item in waypoints]
        data = {
            'name': name,
            'revision': int(revision),
            'waypoints': normalized,
        }
        if segments:
            data['segments'] = [
                self._normalize_segment(item) for item in segments
            ]
        if settings:
            data['settings'] = dict(settings)
        self._write_yaml(self.sets_directory / f'{set_id}.yaml', data)

    @staticmethod
    def _normalize_waypoint(item):
        return {
            'id': str(item.get('id') or uuid.uuid4().hex),
            'name': str(item.get('name') or 'waypoint'),
            'x': round(float(item.get('x', 0.0)), 3),
            'y': round(float(item.get('y', 0.0)), 3),
            'yaw': round(float(item.get('yaw', 0.0)), 3),
            'dwell_seconds': round(float(item.get('dwell_seconds', 0.0)), 2),
            'enabled': bool(item.get('enabled', True)),
        }

    @classmethod
    def _normalize_segment(cls, item):
        via_points = []
        for waypoint in item.get('via_points', []):
            via = cls._normalize_waypoint(waypoint)
            via['dwell_seconds'] = 0.0
            via_points.append(via)
        return {
            'id': str(item.get('id') or uuid.uuid4().hex),
            'name': str(item.get('name') or 'route'),
            'from_waypoint_id': str(item.get('from_waypoint_id') or ''),
            'to_waypoint_id': str(item.get('to_waypoint_id') or ''),
            'bidirectional': bool(item.get('bidirectional', False)),
            'enabled': bool(item.get('enabled', True)),
            'via_points': via_points,
        }

    def _write_index(self, active_set_id):
        self._write_yaml(self.index_path, {'active_set_id': active_set_id})

    @staticmethod
    def _read_yaml(path):
        if not Path(path).exists():
            return {}
        return yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}

    @staticmethod
    def _write_yaml(path, data):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.tmp')
        try:
            temporary.write_text(
                yaml.safe_dump(data, sort_keys=False), encoding='utf-8'
            )
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)

    @contextmanager
    def _lock(self):
        self.sets_directory.mkdir(parents=True, exist_ok=True)
        with (self.sets_directory / '.lock').open('a', encoding='utf-8') as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    @staticmethod
    def _validate_id(value):
        if not VALID_ID.fullmatch(value):
            raise ValueError('Invalid waypoint-set ID')
