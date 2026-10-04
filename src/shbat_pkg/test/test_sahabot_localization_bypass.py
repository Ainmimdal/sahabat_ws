"""Non-motion tests for SahaBot's scoped localization bypass."""

from types import SimpleNamespace

import shbat_pkg.api_bridge as api_bridge
from shbat_pkg.api_bridge import APIBridgeNode


def node_harness(*, estop=False, scan=True, localization=False, map_ok=False,
                 tf=False):
    """Construct only the status used by the app navigation gate."""
    node = object.__new__(APIBridgeNode)
    node.status = SimpleNamespace(
        emergency_stop_active=estop,
        scan_healthy=scan,
        localization_healthy=localization,
        map_healthy=map_ok,
        tf_healthy=tf,
    )
    return node


def test_normal_api_requests_still_require_localization_map_and_tf():
    node = node_harness()

    assert node._navigation_unhealthy_reasons() == [
        'localization is not healthy',
        'map is not healthy',
        'transform tree is not healthy',
    ]


def test_sahabot_bypass_skips_only_localization_related_checks():
    node = node_harness()

    assert node._navigation_unhealthy_reasons(
        bypass_localization=True
    ) == []


def test_sahabot_bypass_keeps_scan_and_estop_blocks():
    node = node_harness(estop=True, scan=False)

    assert node._navigation_unhealthy_reasons(
        bypass_localization=True
    ) == [
        'emergency stop is active',
        'laser scan is not healthy',
    ]


class RosNodeHarness:
    """Capture the bypass value received by the Flask exhibit route."""

    def __init__(self):
        self.bypass_localization = None

    def operator_patrol_command(self, *_args, **kwargs):
        self.bypass_localization = kwargs.get('bypass_localization')
        return {'success': True}


def test_local_sahabot_header_enables_bypass_for_exhibit_route():
    previous = api_bridge.ros_node
    node = RosNodeHarness()
    api_bridge.ros_node = node
    try:
        client = api_bridge.create_flask_app().test_client()
        response = client.post(
            '/exhibit/goto',
            json={'exhibit': 'waypoint_1'},
            headers={'X-SahaBot-Bypass-Localization': '1'},
        )
    finally:
        api_bridge.ros_node = previous

    assert response.status_code == 200
    assert node.bypass_localization is True


def test_remote_header_cannot_enable_local_sahabot_bypass():
    previous = api_bridge.ros_node
    node = RosNodeHarness()
    api_bridge.ros_node = node
    try:
        client = api_bridge.create_flask_app().test_client()
        response = client.post(
            '/exhibit/goto',
            json={'exhibit': 'waypoint_1'},
            headers={'X-SahaBot-Bypass-Localization': '1'},
            environ_base={'REMOTE_ADDR': '192.0.2.10'},
        )
    finally:
        api_bridge.ros_node = previous

    assert response.status_code == 200
    assert node.bypass_localization is False
