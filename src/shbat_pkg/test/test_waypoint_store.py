"""Focused tests for map-scoped waypoint-set persistence."""

import pytest
import yaml

from shbat_pkg.waypoint_store import WaypointStore


def test_migrates_legacy_file_without_modifying_it(tmp_path):
    legacy = tmp_path / 'waypoints.yaml'
    original = {
        'revision': 4,
        'waypoints': [
            {'name': 'dock', 'x': 1.0, 'y': 2.0, 'yaw': 0.5},
            {'name': 'hall', 'x': 3.0, 'y': 4.0, 'yaw': 1.0},
        ],
    }
    legacy.write_text(yaml.safe_dump(original), encoding='utf-8')

    store = WaypointStore(tmp_path)
    revision, waypoints = store.read_set()

    assert revision == 4
    assert [item['name'] for item in waypoints] == ['hall']
    assert store.load_dock()['x'] == 1.0
    assert yaml.safe_load(legacy.read_text(encoding='utf-8')) == original


def test_manages_independent_sets_and_revisions(tmp_path):
    store = WaypointStore(tmp_path)
    store.read_set()
    second = store.create_set('Morning tour')

    assert second == 'morning-tour'
    assert store.active_set_id() == second
    assert store.save_set(second, 0, [{
        'name': 'gallery', 'x': 1, 'y': 2, 'yaw': 0,
    }]) == 1
    assert store.read_set('default')[1] == []
    assert store.read_set(second)[1][0]['name'] == 'gallery'

    with pytest.raises(RuntimeError, match='revision:1'):
        store.save_set(second, 0, [])


def test_saves_route_graph_and_preserves_segments_on_legacy_save(tmp_path):
    store = WaypointStore(tmp_path)
    store.read_set()

    revision = store.save_graph(
        'default',
        0,
        [
            {'id': 'entrance', 'name': 'Entrance', 'x': 0, 'y': 0, 'yaw': 0},
            {'id': 'exhibit-a', 'name': 'Exhibit A', 'x': 2, 'y': 0, 'yaw': 0},
        ],
        [{
            'id': 'entrance-to-exhibit-a',
            'name': 'Entrance to Exhibit A',
            'from_waypoint_id': 'entrance',
            'to_waypoint_id': 'exhibit-a',
            'via_points': [{'name': 'via', 'x': 1, 'y': 0.5, 'yaw': 0}],
        }],
        {'direct_fallback': 'warn'},
    )

    assert revision == 1
    revision, waypoints, segments, settings = store.read_graph('default')
    assert revision == 1
    assert [item['id'] for item in waypoints] == ['entrance', 'exhibit-a']
    assert segments[0]['bidirectional'] is False
    assert segments[0]['enabled'] is True
    assert segments[0]['via_points'][0]['dwell_seconds'] == 0.0
    assert settings['direct_fallback'] == 'warn'

    store.save_set('default', 1, waypoints)
    _revision, _waypoints, preserved_segments, _settings = store.read_graph(
        'default'
    )
    assert preserved_segments[0]['id'] == 'entrance-to-exhibit-a'


def test_archives_deleted_set_and_keeps_one(tmp_path):
    store = WaypointStore(tmp_path)
    store.read_set()
    second = store.create_set('Temporary')

    active = store.delete_set(second)

    assert active == 'default'
    assert list((tmp_path / 'waypoint_sets' / 'archive').glob('temporary-*.yaml'))
    with pytest.raises(ValueError, match='at least one'):
        store.delete_set('default')
