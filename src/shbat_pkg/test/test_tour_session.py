"""Focused non-motion tests for the map-aware SahaBot tour state."""

import pytest

from shbat_pkg.tour_session import TourConfigurationError, TourSession


def waypoint(number, x=None, enabled=True):
    """Build a compact waypoint fixture."""
    return {
        'id': f'wp-{number}',
        'name': f'waypoint_{number}',
        'x': float(number if x is None else x),
        'y': 0.0,
        'enabled': enabled,
    }


def test_gallerysq4_auto_profile_requires_all_six_exhibits():
    """Treat gallerysq4 as strict production data in automatic mode."""
    session = TourSession(profile='auto')
    session.configure('gallerysq4', 'new-tour', [waypoint(1), waypoint(2)])

    assert session.profile == 'production'
    assert not session.production_ready
    with pytest.raises(TourConfigurationError, match='station-3'):
        session.resolve_station('station-1')


def test_partial_test_map_exposes_only_configured_exhibits():
    """Allow a named subset when the active map is a test map."""
    session = TourSession(profile='auto')
    session.configure(
        'rdlsabtu', 'tests',
        [waypoint(1), waypoint(2), waypoint(3)],
    )

    assert session.profile == 'test'
    assert session.production_ready
    assert session.next_station_id() == 'station-1'
    with pytest.raises(TourConfigurationError, match='not configured'):
        session.resolve_station('station-4')


def test_next_skips_missing_exhibits_and_does_not_wrap():
    """Advance through available stations and stop at the final one."""
    session = TourSession(profile='test', arrival_tolerance_m=0.5)
    session.configure(
        'dummy', 'subset',
        [waypoint(1), waypoint(3), waypoint(6)],
    )
    session.update_pose(1.0, 0.0)

    target = session.resolve_station('station-1')
    session.prepare_navigation(target, now=0.0, touring=True)
    session.goal_accepted(now=0.1)
    arrived = session.observe_navigation_state('complete', now=1.0)
    assert arrived['type'] == 'arrived'
    assert session.current_exhibit == 'station-1'
    assert session.next_station_id() == 'station-3'

    session.update_pose(6.0, 0.0)
    target = session.resolve_station('station-6')
    session.prepare_navigation(target, now=2.0, touring=True)
    session.goal_accepted(now=2.1)
    session.observe_navigation_state('complete', now=3.0)
    assert session.next_station_id() is None


def test_narration_events_wait_for_acceptance_and_confirmed_arrival():
    """Do not announce movement or arrival before their real transitions."""
    session = TourSession(profile='test', arrival_tolerance_m=0.5)
    session.configure('dummy', 'one', [waypoint(1, x=10.0)])
    session.update_pose(0.0, 0.0)
    session.prepare_navigation(
        session.resolve_station('station-1'), now=0.0
    )

    assert session.latest_event is None
    started = session.observe_navigation_state('navigating', now=1.0)
    assert started['type'] == 'navigation_started'

    session.update_pose(9.0, 0.0)
    failed = session.observe_navigation_state('complete', now=2.0)
    assert failed['type'] == 'navigation_failed'
    assert session.current_exhibit == ''


def test_almost_there_is_once_and_short_trips_skip_it():
    """Emit useful progress speech once and omit it on short journeys."""
    session = TourSession(profile='test')
    session.configure('dummy', 'one', [waypoint(1, x=10.0)])
    session.update_pose(0.0, 0.0)
    session.prepare_navigation(session.resolve_station('station-1'), now=0.0)
    session.goal_accepted(now=0.1)
    session.update_pose(8.0, 0.0)

    event = session.tick(now=9.0)
    assert event['type'] == 'almost_there'
    assert session.tick(now=10.0) is None

    short = TourSession(profile='test')
    short.configure('dummy', 'one', [waypoint(1, x=3.0)])
    short.update_pose(0.0, 0.0)
    short.prepare_navigation(short.resolve_station('station-1'), now=0.0)
    short.goal_accepted(now=0.1)
    short.update_pose(2.0, 0.0)
    assert short.tick(now=20.0) is None


def test_blocked_and_cleared_events_do_not_repeat():
    """Speak once when blocked and once when the path becomes clear."""
    session = TourSession(profile='test')
    session.configure('dummy', 'one', [waypoint(1, x=10.0)])
    session.update_pose(0.0, 0.0)
    session.prepare_navigation(session.resolve_station('station-1'), now=0.0)
    session.goal_accepted(now=0.1)

    assert session.tick(now=1.0, is_stuck=True)['type'] == 'path_blocked'
    assert session.tick(now=2.0, is_stuck=True) is None
    assert session.tick(now=3.0, is_stuck=False)['type'] == 'path_cleared'
    assert session.tick(now=4.0, is_stuck=False) is None


def test_location_text_distinguishes_arrived_and_in_transit():
    """Report the target while moving and the exhibit only after arrival."""
    session = TourSession(profile='test', arrival_tolerance_m=0.5)
    session.configure('dummy', 'one', [waypoint(1, x=2.0)])
    session.update_pose(0.0, 0.0)
    session.prepare_navigation(session.resolve_station('station-1'), now=0.0)
    assert 'on the way' in session.location_text()

    session.goal_accepted(now=0.1)
    session.update_pose(2.0, 0.0)
    session.observe_navigation_state('complete', now=1.0)
    assert session.location_text() == 'I am at Section A, Entrance Hall.'


def test_dock_is_optional_and_not_part_of_exhibit_order():
    """Keep dock availability independent from the visitor tour catalog."""
    session = TourSession(profile='test')
    session.configure('dummy', 'one', [waypoint(1)])
    with pytest.raises(TourConfigurationError, match='no dock'):
        session.resolve_dock()

    session.configure(
        'dummy', 'one', [waypoint(1)],
        dock={'id': 'dock', 'x': -1.0, 'y': 2.0},
    )
    dock = session.resolve_dock()
    assert dock.is_dock
    assert session.next_station_id() == 'station-1'
