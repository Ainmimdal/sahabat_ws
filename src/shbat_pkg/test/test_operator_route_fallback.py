"""Non-motion tests for SahaBot's direct Nav2 route fallback."""

import pytest

from shbat_pkg.operator_backend import OperatorBackend
from shbat_pkg.route_graph import RouteNotFound


class LoggerHarness:
    """Capture route warnings without creating a ROS node."""

    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def backend_harness():
    """Construct the state used by route-origin selection."""
    operator = object.__new__(OperatorBackend)
    operator._status_pose = lambda: ('map', 2.52, -1.70, 0.0)
    operator.get_logger = lambda: LoggerHarness()
    return operator


WAYPOINTS = [
    {'id': 'start', 'x': -1.24, 'y': 0.94, 'enabled': True},
    {'id': 'target', 'x': 2.59, 'y': 6.30, 'enabled': True},
]
SEGMENTS = [
    {
        'id': 'start-to-target',
        'from_waypoint_id': 'start',
        'to_waypoint_id': 'target',
        'enabled': True,
    },
]
SETTINGS = {'direct_fallback': 'reject', 'route_origin_tolerance': 1.25}


def test_strict_route_still_rejects_when_robot_is_off_graph():
    operator = backend_harness()

    with pytest.raises(RouteNotFound, match='not within 1.25 m'):
        operator._route_points(
            WAYPOINTS,
            SEGMENTS,
            SETTINGS,
            'target',
        )


def test_sahabot_can_fall_back_to_the_same_direct_goal_as_rviz():
    operator = backend_harness()

    points, description = operator._route_points(
        WAYPOINTS,
        SEGMENTS,
        SETTINGS,
        'target',
        allow_direct_fallback=True,
    )

    assert points == [WAYPOINTS[1]]
    assert description.startswith('direct fallback')
