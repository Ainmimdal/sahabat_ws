"""Pure-function tests for the scan mask tuner (no ROS or display needed)."""

import math

from shbat_pkg.scan_mask_tuner import (
    angle_masked,
    find_rear_zone,
    lidar_to_robot_deg,
    rear_edges_to_zone,
    replace_filter_zones,
    split_zones,
    zone_to_rear_edges,
)


def test_upside_down_lidar_bearing_mapping():
    # Matches the scan_filter.yaml note: 0 = rear, 90 = left, 180 = front.
    assert lidar_to_robot_deg(0.0) == -180.0
    assert lidar_to_robot_deg(90.0) == 90.0
    assert lidar_to_robot_deg(-90.0) == -90.0
    assert lidar_to_robot_deg(180.0) == 0.0


def test_current_rear_half_mask_round_trips():
    assert rear_edges_to_zone(90.0, -90.0) == [-90.0, 90.0]
    assert zone_to_rear_edges((-90.0, 90.0)) == (90.0, -90.0)


def test_widening_the_view_shrinks_the_lidar_zone():
    assert rear_edges_to_zone(95.0, -93.0) == [-87.0, 85.0]


def test_mask_matches_scan_filter_inclusive_check():
    zones = [(math.radians(-87.0), math.radians(85.0))]
    assert angle_masked(math.radians(0.0), zones)
    assert angle_masked(math.radians(85.0), zones)
    assert not angle_masked(math.radians(86.0), zones)
    assert not angle_masked(math.radians(180.0), zones)


def test_rear_zone_is_found_among_others():
    zones = split_zones([100.0, 110.0, -90.0, 90.0])
    assert find_rear_zone(zones) == 1


def test_replace_filter_zones_keeps_other_lines():
    text = (
        '# header comment\n'
        'scan_filter:\n'
        '  ros__parameters:\n'
        '    min_range: 0.10\n'
        '    filter_zones: [-90.0, 90.0]\n'
        '    max_range: 8.0\n'
    )
    updated = replace_filter_zones(text, [(-87.0, 85.0)], 'visible 188')
    assert '    filter_zones: [-87.0, 85.0]  # visible 188\n' in updated
    assert updated.replace(
        '    filter_zones: [-87.0, 85.0]  # visible 188\n',
        '    filter_zones: [-90.0, 90.0]\n',
    ) == text
