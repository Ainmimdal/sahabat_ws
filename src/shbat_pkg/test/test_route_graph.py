"""Focused tests for deterministic saved route-graph planning."""

import pytest

from shbat_pkg.route_graph import (
    RouteNotFound,
    nearest_waypoint_id,
    plan_route,
)


WAYPOINTS = [
    {'id': 'a', 'x': 0.0, 'y': 0.0, 'enabled': True},
    {'id': 'b', 'x': 2.0, 'y': 0.0, 'enabled': True},
    {'id': 'c', 'x': 4.0, 'y': 0.0, 'enabled': True},
]


def test_chains_segments_for_any_connected_waypoint_pair():
    segments = [
        {
            'id': 'a-b',
            'from_waypoint_id': 'a',
            'to_waypoint_id': 'b',
            'bidirectional': True,
            'enabled': True,
            'via_points': [{'id': 'ab-via', 'x': 1.0, 'y': 0.5}],
        },
        {
            'id': 'b-c',
            'from_waypoint_id': 'b',
            'to_waypoint_id': 'c',
            'bidirectional': True,
            'enabled': True,
            'via_points': [{'id': 'bc-via', 'x': 3.0, 'y': -0.5}],
        },
    ]

    route = plan_route(WAYPOINTS, segments, 'a', 'c')

    assert [item['id'] for item in route] == ['ab-via', 'b', 'bc-via', 'c']


def test_bidirectional_segment_reverses_doorway_approach_points():
    segments = [{
        'id': 'door',
        'from_waypoint_id': 'a',
        'to_waypoint_id': 'b',
        'bidirectional': True,
        'enabled': True,
        'via_points': [
            {'id': 'inside', 'x': 0.5, 'y': 0.0},
            {'id': 'outside', 'x': 1.5, 'y': 0.0},
        ],
    }]

    route = plan_route(WAYPOINTS, segments, 'b', 'a')

    assert [item['id'] for item in route] == ['outside', 'inside', 'a']


def test_chooses_shorter_human_route_and_ignores_disabled_segment():
    segments = [
        {
            'id': 'direct-disabled',
            'from_waypoint_id': 'a',
            'to_waypoint_id': 'c',
            'enabled': False,
            'via_points': [],
        },
        {
            'id': 'a-b',
            'from_waypoint_id': 'a',
            'to_waypoint_id': 'b',
            'bidirectional': True,
            'enabled': True,
            'via_points': [],
        },
        {
            'id': 'b-c',
            'from_waypoint_id': 'b',
            'to_waypoint_id': 'c',
            'bidirectional': True,
            'enabled': True,
            'via_points': [],
        },
    ]

    assert [item['id'] for item in plan_route(
        WAYPOINTS, segments, 'a', 'c'
    )] == ['b', 'c']


def test_reports_disconnected_graph_and_finds_nearest_enabled_origin():
    assert nearest_waypoint_id(WAYPOINTS, 0.2, 0.1, 0.5) == 'a'
    assert nearest_waypoint_id(WAYPOINTS, 0.7, 0.0, 0.5) is None

    with pytest.raises(RouteNotFound, match='No enabled route'):
        plan_route(WAYPOINTS, [], 'a', 'c')


def test_directed_ring_reaches_every_waypoint_in_clockwise_order():
    waypoints = [
        {'id': str(index), 'x': float(index), 'y': 0.0, 'enabled': True}
        for index in range(1, 7)
    ]
    segments = []
    for source in range(1, 7):
        target = source + 1 if source < 6 else 1
        segments.append({
            'id': f'{source}-to-{target}',
            'from_waypoint_id': str(source),
            'to_waypoint_id': str(target),
            'bidirectional': False,
            'enabled': True,
            'via_points': [{
                'id': f'rp-{source:03d}',
                'x': float(source) + 0.5,
                'y': 0.0,
            }],
        })

    waypoint_ids = {item['id'] for item in waypoints}
    full_clockwise_route = plan_route(waypoints, segments, '1', '6')
    return_to_start = plan_route(waypoints, segments, '6', '1')

    assert [
        item['id'] for item in full_clockwise_route
        if item['id'] in waypoint_ids
    ] == ['2', '3', '4', '5', '6']
    assert [item['id'] for item in return_to_start] == ['rp-006', '1']

    for source in waypoint_ids:
        for target in waypoint_ids - {source}:
            assert plan_route(waypoints, segments, source, target)[-1]['id'] == target
