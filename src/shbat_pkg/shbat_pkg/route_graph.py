"""Plan deterministic waypoint-to-waypoint routes over saved route segments."""

from dataclasses import dataclass
import heapq
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


class RouteNotFound(ValueError):
    """Raised when enabled route segments do not connect two waypoints."""


@dataclass(frozen=True)
class _Edge:
    """One directed traversal of a saved route segment."""

    target_id: str
    weight: float
    points: Tuple[dict, ...]
    segment_id: str


def _distance(first: dict, second: dict) -> float:
    return math.hypot(
        float(second.get('x', 0.0)) - float(first.get('x', 0.0)),
        float(second.get('y', 0.0)) - float(first.get('y', 0.0)),
    )


def _edge(
    source: dict,
    target: dict,
    via_points: Sequence[dict],
    segment_id: str,
) -> _Edge:
    points = [dict(point) for point in via_points]
    points.append(dict(target))
    chain = [source, *points]
    weight = sum(
        _distance(chain[index], chain[index + 1])
        for index in range(len(chain) - 1)
    )
    return _Edge(
        target_id=str(target['id']),
        weight=weight,
        points=tuple(points),
        segment_id=segment_id,
    )


def _adjacency(
    waypoints: Iterable[dict],
    segments: Iterable[dict],
) -> Tuple[Dict[str, dict], Dict[str, List[_Edge]]]:
    nodes = {
        str(item.get('id', '')): dict(item)
        for item in waypoints
        if str(item.get('id', ''))
    }
    graph: Dict[str, List[_Edge]] = {node_id: [] for node_id in nodes}
    for segment in segments:
        if not bool(segment.get('enabled', True)):
            continue
        source_id = str(segment.get('from_waypoint_id', ''))
        target_id = str(segment.get('to_waypoint_id', ''))
        if source_id not in nodes or target_id not in nodes:
            continue
        segment_id = str(segment.get('id') or f'{source_id}-to-{target_id}')
        via_points = list(segment.get('via_points', []))
        graph[source_id].append(
            _edge(nodes[source_id], nodes[target_id], via_points, segment_id)
        )
        if bool(segment.get('bidirectional', False)):
            graph[target_id].append(
                _edge(
                    nodes[target_id],
                    nodes[source_id],
                    list(reversed(via_points)),
                    segment_id,
                )
            )
    for edges in graph.values():
        edges.sort(
            key=lambda item: (
                item.weight,
                item.segment_id,
                item.target_id,
            )
        )
    return nodes, graph


def nearest_waypoint_id(
    waypoints: Iterable[dict],
    x: float,
    y: float,
    maximum_distance: Optional[float] = None,
) -> Optional[str]:
    """Return the closest enabled destination within an optional radius."""
    candidates = []
    for item in waypoints:
        if not bool(item.get('enabled', True)):
            continue
        waypoint_id = str(item.get('id', ''))
        if not waypoint_id:
            continue
        distance = math.hypot(
            float(item.get('x', 0.0)) - x,
            float(item.get('y', 0.0)) - y,
        )
        candidates.append((distance, waypoint_id))
    if not candidates:
        return None
    distance, waypoint_id = min(candidates)
    if maximum_distance is not None and distance > maximum_distance:
        return None
    return waypoint_id


def plan_route(
    waypoints: Iterable[dict],
    segments: Iterable[dict],
    source_id: str,
    target_id: str,
) -> List[dict]:
    """Return ordered route points using Dijkstra shortest-path search."""
    nodes, graph = _adjacency(waypoints, segments)
    if source_id not in nodes:
        raise RouteNotFound(f'Unknown route origin: {source_id}')
    if target_id not in nodes:
        raise RouteNotFound(f'Unknown route destination: {target_id}')
    if source_id == target_id:
        return [dict(nodes[target_id])]

    queue = [(0.0, source_id)]
    distances = {source_id: 0.0}
    previous: Dict[str, Tuple[str, _Edge]] = {}
    while queue:
        distance, node_id = heapq.heappop(queue)
        if distance != distances.get(node_id):
            continue
        if node_id == target_id:
            break
        for edge in graph.get(node_id, []):
            next_distance = distance + edge.weight
            if next_distance >= distances.get(edge.target_id, math.inf):
                continue
            distances[edge.target_id] = next_distance
            previous[edge.target_id] = (node_id, edge)
            heapq.heappush(queue, (next_distance, edge.target_id))

    if target_id not in previous:
        raise RouteNotFound(
            f'No enabled route from {source_id} to {target_id}'
        )

    edges = []
    current = target_id
    while current != source_id:
        prior, edge = previous[current]
        edges.append(edge)
        current = prior
    edges.reverse()

    result: List[dict] = []
    for edge in edges:
        result.extend(dict(point) for point in edge.points)
    return result
