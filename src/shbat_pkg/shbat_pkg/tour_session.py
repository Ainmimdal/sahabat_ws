"""Map-aware, event-driven state for the SahaBot gallery tour."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Iterable, Optional


CATALOG = (
    {
        'station_id': 'station-1',
        'waypoint_name': 'waypoint_1',
        'display_name': 'Section A, Entrance Hall',
    },
    {
        'station_id': 'station-2',
        'waypoint_name': 'waypoint_2',
        'display_name': 'Section B, Historical Gallery',
    },
    {
        'station_id': 'station-3',
        'waypoint_name': 'waypoint_3',
        'display_name': 'Section C, Governance Hall',
    },
    {
        'station_id': 'station-4',
        'waypoint_name': 'waypoint_4',
        'display_name': 'Section D, Featured Exhibitions',
    },
    {
        'station_id': 'station-5',
        'waypoint_name': 'waypoint_5',
        'display_name': 'Section F, Hall of Fame',
    },
    {
        'station_id': 'station-6',
        'waypoint_name': 'waypoint_6',
        'display_name': 'Section E, Convocation Hall',
    },
)

DOCK_WAYPOINT_ID = '__map_dock__'


class TourConfigurationError(ValueError):
    """Raised when a tour request is incompatible with the active profile."""


@dataclass(frozen=True)
class TourTarget:
    """Resolved destination for one navigation request."""

    station_id: str
    waypoint_name: str
    waypoint_id: str
    display_name: str
    x: float
    y: float
    is_dock: bool = False


class TourSession:
    """Own visitor-facing tour state without sending motion commands."""

    def __init__(
        self,
        profile: str = 'auto',
        production_map_ids: Iterable[str] = ('gallerysq4',),
        arrival_tolerance_m: float = 0.5,
        almost_there_min_trip_m: float = 6.0,
        almost_there_distance_m: float = 2.5,
        almost_there_fraction: float = 0.3,
        almost_there_min_time_s: float = 8.0,
    ):
        """Initialize thresholds and an empty map-scoped tour session."""
        profile = str(profile).strip().lower()
        if profile not in ('auto', 'production', 'test'):
            raise ValueError('tour profile must be auto, production, or test')
        self.requested_profile = profile
        self.production_map_ids = {
            str(item).strip() for item in production_map_ids if str(item).strip()
        }
        self.arrival_tolerance_m = max(0.05, float(arrival_tolerance_m))
        self.almost_there_min_trip_m = max(
            self.arrival_tolerance_m, float(almost_there_min_trip_m)
        )
        self.almost_there_distance_m = max(
            self.arrival_tolerance_m, float(almost_there_distance_m)
        )
        self.almost_there_fraction = max(
            0.05, min(0.9, float(almost_there_fraction))
        )
        self.almost_there_min_time_s = max(
            0.0, float(almost_there_min_time_s)
        )

        self.map_id = ''
        self.set_id = ''
        self.profile = 'test'
        self.exhibits: list[dict[str, Any]] = []
        self.dock: Optional[dict[str, Any]] = None
        self.state = 'idle'
        self.current_exhibit = ''
        self.target: Optional[TourTarget] = None
        self.nearest_exhibit = ''
        self.nearest_distance_m: Optional[float] = None
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_yaw = 0.0
        self.initial_distance_m: Optional[float] = None
        self.remaining_distance_m: Optional[float] = None
        self.navigation_started_at: Optional[float] = None
        self.almost_there_announced = False
        self.blocked_announced = False
        self.touring = False
        self._resume_pending = False
        self._event_id = 0
        self.latest_event: Optional[dict[str, Any]] = None

    def configure(
        self,
        map_id: str,
        set_id: str,
        waypoints: Iterable[dict[str, Any]],
        dock: Optional[dict[str, Any]] = None,
    ) -> None:
        """Resolve logical exhibits against one active map waypoint set."""
        map_id = str(map_id).strip()
        set_id = str(set_id).strip()
        identity_changed = (map_id, set_id) != (self.map_id, self.set_id)
        self.map_id = map_id
        self.set_id = set_id
        self.profile = (
            'production'
            if self.requested_profile == 'production'
            or (
                self.requested_profile == 'auto'
                and map_id in self.production_map_ids
            )
            else 'test'
        )

        enabled_by_name = {
            str(item.get('name', '')).strip(): item
            for item in waypoints
            if bool(item.get('enabled', True))
        }
        exhibits = []
        for order, catalog_item in enumerate(CATALOG, start=1):
            waypoint = enabled_by_name.get(catalog_item['waypoint_name'])
            exhibit = {
                **catalog_item,
                'order': order,
                'available': waypoint is not None,
                'waypoint_id': str(waypoint.get('id', '')) if waypoint else '',
                'x': float(waypoint.get('x', 0.0)) if waypoint else None,
                'y': float(waypoint.get('y', 0.0)) if waypoint else None,
            }
            exhibits.append(exhibit)
        self.exhibits = exhibits
        self.dock = dict(dock) if dock else None

        available_ids = {
            item['station_id'] for item in self.exhibits if item['available']
        }
        if identity_changed:
            self.state = 'idle'
            self.current_exhibit = ''
            self.target = None
            self.touring = False
            self.latest_event = None
            self._reset_navigation_progress()
        elif self.current_exhibit not in available_ids:
            self.current_exhibit = ''
        if (
            self.target is not None
            and not self.target.is_dock
            and self.target.station_id not in available_ids
        ):
            self.target = None
            self.state = 'idle'
            self._reset_navigation_progress()
        self.update_pose(
            self.position_x, self.position_y, self.position_yaw
        )

    @property
    def missing_station_ids(self) -> list[str]:
        """Return catalog stations absent from the active waypoint set."""
        return [
            item['station_id'] for item in self.exhibits
            if not item['available']
        ]

    @property
    def production_ready(self) -> bool:
        """Return whether strict production validation currently passes."""
        return self.profile != 'production' or not self.missing_station_ids

    def ensure_ready(self) -> None:
        """Reject navigation when a strict production catalog is incomplete."""
        if self.production_ready:
            return
        missing = ', '.join(self.missing_station_ids)
        raise TourConfigurationError(
            f'Production tour is missing configured exhibits: {missing}'
        )

    def resolve_station(self, station_or_waypoint: str) -> TourTarget:
        """Resolve a logical station id or expected waypoint name."""
        requested = str(station_or_waypoint).strip()
        for item in self.exhibits:
            if requested not in (item['station_id'], item['waypoint_name']):
                continue
            if not item['available']:
                raise TourConfigurationError(
                    f"{item['display_name']} is not configured on this map"
                )
            self.ensure_ready()
            return TourTarget(
                station_id=item['station_id'],
                waypoint_name=item['waypoint_name'],
                waypoint_id=item['waypoint_id'],
                display_name=item['display_name'],
                x=float(item['x']),
                y=float(item['y']),
            )
        raise TourConfigurationError(f'Unknown exhibit: {requested}')

    def resolve_dock(self) -> TourTarget:
        """Return the configured map dock independently of tour strictness."""
        if not self.dock:
            raise TourConfigurationError('This map has no dock pose')
        return TourTarget(
            station_id='dock',
            waypoint_name='dock',
            waypoint_id=DOCK_WAYPOINT_ID,
            display_name='the dock',
            x=float(self.dock.get('x', 0.0)),
            y=float(self.dock.get('y', 0.0)),
            is_dock=True,
        )

    def prepare_navigation(
        self,
        target: TourTarget,
        *,
        now: Optional[float] = None,
        touring: Optional[bool] = None,
        resumed: bool = False,
    ) -> None:
        """Record a pending target before the operator command is sent."""
        now = time.monotonic() if now is None else float(now)
        self.target = target
        self.state = 'goal_pending'
        self.navigation_started_at = now
        self.initial_distance_m = self._distance_to(target)
        self.remaining_distance_m = self.initial_distance_m
        self.almost_there_announced = False
        self.blocked_announced = False
        self._resume_pending = bool(resumed)
        if touring is not None:
            self.touring = bool(touring)

    def goal_accepted(self, now: Optional[float] = None) -> dict[str, Any]:
        """Enter NAVIGATING and emit the departure or resumed phrase."""
        if self.target is None:
            raise RuntimeError('No tour target is pending')
        now = time.monotonic() if now is None else float(now)
        self.navigation_started_at = now
        self.state = 'navigating'
        if self._resume_pending:
            text = 'Thank you. Let us continue.'
            event_type = 'navigation_resumed'
        else:
            text = f'All right, let us head to {self.target.display_name}.'
            event_type = 'navigation_started'
        self._resume_pending = False
        return self._emit(event_type, text, now)

    def mark_paused(self, now: Optional[float] = None) -> dict[str, Any]:
        """Enter the paused state and emit one visitor acknowledgement."""
        self.state = 'paused'
        return self._emit(
            'navigation_paused',
            'I have paused here.',
            time.monotonic() if now is None else float(now),
        )

    def mark_cancelled(self, now: Optional[float] = None) -> dict[str, Any]:
        """Clear the active target after an explicit cancellation."""
        self.state = 'idle'
        event = self._emit(
            'navigation_cancelled',
            'Okay, I have stopped here.',
            time.monotonic() if now is None else float(now),
        )
        self.target = None
        self.touring = False
        self._reset_navigation_progress()
        return event

    def mark_failed(
        self, message: str = '', now: Optional[float] = None
    ) -> dict[str, Any]:
        """Record a failed navigation result and request operator help."""
        display_name = self.target.display_name if self.target else 'the exhibit'
        self.state = 'failed'
        event = self._emit(
            'navigation_failed',
            f'I could not reach {display_name}. Please ask an operator for help.',
            time.monotonic() if now is None else float(now),
            detail=message,
        )
        self.target = None
        self._reset_navigation_progress()
        return event

    def observe_navigation_state(
        self, raw_state: str, now: Optional[float] = None
    ) -> Optional[dict[str, Any]]:
        """Translate operator/Nav2 state changes into visitor events."""
        now = time.monotonic() if now is None else float(now)
        raw_state = str(raw_state or '').strip().lower()
        if raw_state == 'goal_pending':
            if self.target is not None:
                self.state = 'goal_pending'
            return None
        if raw_state == 'navigating':
            if self.target is not None and self.state == 'goal_pending':
                return self.goal_accepted(now)
            if self.state != 'blocked' and self.target is not None:
                self.state = 'navigating'
            return None
        if raw_state == 'complete' and self.target is not None:
            remaining = self._distance_to(self.target)
            self.remaining_distance_m = remaining
            if remaining > self.arrival_tolerance_m:
                return self.mark_failed(
                    f'Navigation completed {remaining:.2f} m from its target', now
                )
            target = self.target
            if target.is_dock:
                self.current_exhibit = ''
                self.state = 'docked'
                event = self._emit(
                    'docked', 'We have reached the dock.', now
                )
                self.touring = False
            else:
                self.current_exhibit = target.station_id
                self.state = 'at_exhibit'
                event = self._emit(
                    'arrived',
                    f'We have arrived at {target.display_name}.',
                    now,
                    station_id=target.station_id,
                )
            self.target = None
            self._reset_navigation_progress()
            return event
        if raw_state == 'rejected' or raw_state.startswith('failed'):
            if self.target is not None:
                return self.mark_failed(raw_state, now)
        return None

    def update_pose(self, x: float, y: float, yaw: float = 0.0) -> None:
        """Update exact pose, nearest exhibit, and remaining distance."""
        self.position_x = float(x)
        self.position_y = float(y)
        self.position_yaw = float(yaw)
        candidates = [item for item in self.exhibits if item['available']]
        if candidates:
            nearest = min(
                candidates,
                key=lambda item: math.hypot(
                    self.position_x - float(item['x']),
                    self.position_y - float(item['y']),
                ),
            )
            self.nearest_exhibit = nearest['station_id']
            self.nearest_distance_m = math.hypot(
                self.position_x - float(nearest['x']),
                self.position_y - float(nearest['y']),
            )
        else:
            self.nearest_exhibit = ''
            self.nearest_distance_m = None
        if self.target is not None:
            self.remaining_distance_m = self._distance_to(self.target)

    def tick(
        self, *, now: Optional[float] = None, is_stuck: bool = False
    ) -> Optional[dict[str, Any]]:
        """Emit conditional progress events at most once per condition."""
        now = time.monotonic() if now is None else float(now)
        if self.target is None or self.state not in ('navigating', 'blocked'):
            return None

        self.remaining_distance_m = self._distance_to(self.target)
        if is_stuck and not self.blocked_announced:
            self.blocked_announced = True
            self.state = 'blocked'
            return self._emit(
                'path_blocked',
                'The path is blocked. Please give me a little space.',
                now,
            )
        if not is_stuck and self.blocked_announced:
            self.blocked_announced = False
            self.state = 'navigating'
            return self._emit(
                'path_cleared', 'Thank you. Let us continue.', now
            )

        elapsed = (
            now - self.navigation_started_at
            if self.navigation_started_at is not None
            else 0.0
        )
        threshold = min(
            self.almost_there_distance_m,
            (self.initial_distance_m or 0.0) * self.almost_there_fraction,
        )
        if (
            not self.target.is_dock
            and not self.almost_there_announced
            and not is_stuck
            and (self.initial_distance_m or 0.0)
            >= self.almost_there_min_trip_m
            and elapsed >= self.almost_there_min_time_s
            and self.remaining_distance_m <= threshold
            and self.remaining_distance_m > self.arrival_tolerance_m
        ):
            self.almost_there_announced = True
            return self._emit('almost_there', 'We are almost there.', now)
        return None

    def next_station_id(self) -> Optional[str]:
        """Return the next available exhibit without wrapping the tour."""
        available = [
            item['station_id'] for item in self.exhibits if item['available']
        ]
        if not available:
            return None
        anchor = self.current_exhibit
        if not anchor and self.target is not None and not self.target.is_dock:
            anchor = self.target.station_id
        if not anchor or anchor not in available:
            return available[0]
        index = available.index(anchor) + 1
        return available[index] if index < len(available) else None

    def mark_tour_complete(
        self, now: Optional[float] = None
    ) -> dict[str, Any]:
        """End the non-wrapping tour while leaving the robot in place."""
        self.touring = False
        return self._emit(
            'tour_complete',
            'That is the end of the available tour. I will stay here.',
            time.monotonic() if now is None else float(now),
        )

    def location_text(self) -> str:
        """Return an honest, human-readable current-location sentence."""
        current = self._exhibit(self.current_exhibit)
        if (
            current is not None
            and self.nearest_exhibit == self.current_exhibit
            and (self.nearest_distance_m or 0.0) <= self.arrival_tolerance_m
        ):
            return f"I am at {current['display_name']}."
        if self.target is not None:
            return f'I am on the way to {self.target.display_name}.'
        nearest = self._exhibit(self.nearest_exhibit)
        if nearest is not None and self.nearest_distance_m is not None:
            return (
                f"I am {self.nearest_distance_m:.1f} metres from "
                f"{nearest['display_name']}."
            )
        return 'My current gallery location is not available yet.'

    def status_dict(self) -> dict[str, Any]:
        """Return the complete API-facing tour snapshot."""
        available = [item for item in self.exhibits if item['available']]
        progress = 0
        if self.current_exhibit:
            ids = [item['station_id'] for item in available]
            if self.current_exhibit in ids:
                progress = ids.index(self.current_exhibit) + 1
        return {
            'map_id': self.map_id,
            'set_id': self.set_id,
            'profile': self.profile,
            'production_ready': self.production_ready,
            'missing_station_ids': self.missing_station_ids,
            'state': self.state,
            'current_exhibit': self.current_exhibit,
            'target_exhibit': (
                self.target.station_id if self.target is not None else ''
            ),
            'target_waypoint': (
                self.target.waypoint_name if self.target is not None else ''
            ),
            'nearest_exhibit': self.nearest_exhibit,
            'nearest_distance_m': self.nearest_distance_m,
            'remaining_distance_m': self.remaining_distance_m,
            'is_touring': self.touring,
            'tour_progress': f'{progress}/{len(available)}',
            'available_count': len(available),
            'exhibits': [dict(item) for item in self.exhibits],
            'dock_available': self.dock is not None,
            'location_text': self.location_text(),
            'interaction': dict(self.latest_event) if self.latest_event else None,
        }

    def _distance_to(self, target: TourTarget) -> float:
        return math.hypot(
            self.position_x - target.x, self.position_y - target.y
        )

    def _exhibit(self, station_id: str) -> Optional[dict[str, Any]]:
        return next(
            (
                item for item in self.exhibits
                if item['station_id'] == station_id
            ),
            None,
        )

    def _emit(
        self,
        event_type: str,
        text: str,
        now: float,
        **extra: Any,
    ) -> dict[str, Any]:
        self._event_id += 1
        event = {
            'id': self._event_id,
            'type': event_type,
            'text': text,
            'station_id': (
                self.target.station_id
                if self.target is not None and not self.target.is_dock
                else ''
            ),
            'timestamp': float(now),
        }
        event.update(extra)
        self.latest_event = event
        return event

    def _reset_navigation_progress(self) -> None:
        self.initial_distance_m = None
        self.remaining_distance_m = None
        self.navigation_started_at = None
        self.almost_there_announced = False
        self.blocked_announced = False
        self._resume_pending = False
