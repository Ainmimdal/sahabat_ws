#!/usr/bin/env python3
"""Lidar scan-to-map matching for guarded startup localization.

Pure numpy, no ROS imports, so it is unit-testable off the robot.

Two uses:

* ``search``   - find the best robot pose inside a bounded window around a
  prior (the dock pose, or a coarse AprilTag pose) by brute-force
  correlative matching of the current scan against a likelihood field built
  from the saved map.
* ``evaluate`` - report how well a single pose explains the scan.

Every result carries an absolute fit quality (fraction of beams that land on a
mapped wall) and an ambiguity ratio (best score of a clearly different pose
divided by the best score) so callers can refuse to publish a pose that the
lidar does not confirm.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class MatchResult:
    """Outcome of a scan-to-map search or evaluation."""

    x: float
    y: float
    yaw: float
    score: float
    inlier_ratio: float
    runner_up_score: float = 0.0
    point_count: int = 0

    @property
    def ambiguity(self) -> float:
        """Runner-up / best score. Near 1.0 means another pose fits as well."""
        if self.score <= 1e-9:
            return 1.0
        return float(self.runner_up_score / self.score)


def normalize_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LikelihoodField:
    """Distance-to-nearest-wall field built from an occupancy grid."""

    def __init__(
        self,
        data: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        occupied_threshold: int = 65,
        max_distance: float = 0.5,
        sigma: float = 0.10,
        inlier_distance: float = 0.10,
    ):
        """Build the field.

        ``data`` is row-major with row 0 at ``origin_y`` (nav_msgs/OccupancyGrid
        convention), values -1 (unknown) or 0..100.
        """
        grid = np.asarray(data)
        if grid.ndim != 2:
            raise ValueError('Occupancy data must be a 2-D array')
        if resolution <= 0.0:
            raise ValueError('Map resolution must be positive')
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = grid.shape
        self.max_distance = float(max_distance)
        self.sigma = float(sigma)
        self.inlier_distance = float(inlier_distance)
        occupied = grid >= occupied_threshold
        if not np.any(occupied):
            raise ValueError('Map contains no occupied cells')
        self.distance = self._distance_field(occupied)

    def _distance_field(self, occupied: np.ndarray) -> np.ndarray:
        """Approximate Euclidean distance with alternating 4/8 dilation."""
        steps = int(math.ceil(self.max_distance / self.resolution))
        distance = np.full(occupied.shape, self.max_distance, dtype=np.float32)
        distance[occupied] = 0.0
        reached = occupied.copy()
        for step in range(1, steps + 1):
            grown = reached.copy()
            grown[1:, :] |= reached[:-1, :]
            grown[:-1, :] |= reached[1:, :]
            grown[:, 1:] |= reached[:, :-1]
            grown[:, :-1] |= reached[:, 1:]
            if step % 2 == 0:
                grown[1:, 1:] |= reached[:-1, :-1]
                grown[1:, :-1] |= reached[:-1, 1:]
                grown[:-1, 1:] |= reached[1:, :-1]
                grown[:-1, :-1] |= reached[1:, 1:]
            new = grown & ~reached
            distance[new] = min(self.max_distance, step * self.resolution)
            reached = grown
        return distance

    def lookup(self, wx: np.ndarray, wy: np.ndarray) -> np.ndarray:
        """Distance to the nearest wall for world points (max outside map)."""
        col = np.floor((wx - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((wy - self.origin_y) / self.resolution).astype(np.int64)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        result = np.full(wx.shape, self.max_distance, dtype=np.float32)
        result[inside] = self.distance[row[inside], col[inside]]
        return result

    def score_poses(
        self, points: np.ndarray, poses: np.ndarray, chunk: int = 1500
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Score (M,3) poses for (N,2) base-frame points.

        Returns (mean gaussian likelihood, inlier ratio) per pose.
        """
        points = np.asarray(points, dtype=np.float64)
        poses = np.atleast_2d(np.asarray(poses, dtype=np.float64))
        scores = np.empty(len(poses))
        inliers = np.empty(len(poses))
        px = points[:, 0][None, :]
        py = points[:, 1][None, :]
        denominator = 2.0 * self.sigma * self.sigma
        for start in range(0, len(poses), chunk):
            block = poses[start:start + chunk]
            cos = np.cos(block[:, 2])[:, None]
            sin = np.sin(block[:, 2])[:, None]
            wx = block[:, 0][:, None] + cos * px - sin * py
            wy = block[:, 1][:, None] + sin * px + cos * py
            distance = self.lookup(wx, wy)
            scores[start:start + chunk] = np.mean(
                np.exp(-(distance * distance) / denominator), axis=1
            )
            inliers[start:start + chunk] = np.mean(
                distance <= self.inlier_distance, axis=1
            )
        return scores, inliers


def scan_to_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    sensor_x: float = 0.0,
    sensor_y: float = 0.0,
    sensor_yaw: float = 0.0,
    max_points: int = 240,
    usable_range: float = 8.0,
) -> np.ndarray:
    """Convert a LaserScan into (N,2) base-frame points, evenly subsampled."""
    values = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + angle_increment * np.arange(len(values))
    upper = min(float(range_max), float(usable_range))
    valid = np.isfinite(values) & (values >= max(range_min, 0.05)) & (values <= upper)
    values = values[valid]
    angles = angles[valid]
    if len(values) > max_points:
        keep = np.linspace(0, len(values) - 1, max_points).round().astype(int)
        values = values[keep]
        angles = angles[keep]
    sx = values * np.cos(angles)
    sy = values * np.sin(angles)
    cos = math.cos(sensor_yaw)
    sin = math.sin(sensor_yaw)
    return np.column_stack((
        sensor_x + cos * sx - sin * sy,
        sensor_y + sin * sx + cos * sy,
    ))


def _pose_grid(
    center: Tuple[float, float, float],
    xy_window: float,
    yaw_window: float,
    xy_step: float,
    yaw_step: float,
) -> np.ndarray:
    xy_count = max(0, int(round(xy_window / xy_step)))
    yaw_count = max(0, int(round(yaw_window / yaw_step)))
    offsets = np.arange(-xy_count, xy_count + 1) * xy_step
    yaw_offsets = np.arange(-yaw_count, yaw_count + 1) * yaw_step
    dx, dy, dyaw = np.meshgrid(offsets, offsets, yaw_offsets, indexing='ij')
    return np.column_stack((
        center[0] + dx.ravel(),
        center[1] + dy.ravel(),
        center[2] + dyaw.ravel(),
    ))


def evaluate(field: LikelihoodField, points: np.ndarray, pose) -> MatchResult:
    """Score one pose without searching."""
    scores, inliers = field.score_poses(points, np.array([pose], dtype=float))
    return MatchResult(
        x=float(pose[0]), y=float(pose[1]), yaw=normalize_angle(float(pose[2])),
        score=float(scores[0]), inlier_ratio=float(inliers[0]),
        point_count=int(len(points)),
    )


def search(
    field: LikelihoodField,
    points: np.ndarray,
    center: Tuple[float, float, float],
    xy_window: float = 0.6,
    yaw_window: float = math.radians(30.0),
    coarse_xy_step: float = 0.05,
    coarse_yaw_step: float = math.radians(2.0),
    fine_xy_step: float = 0.01,
    fine_yaw_step: float = math.radians(0.5),
    separation_xy: float = 0.30,
    separation_yaw: float = math.radians(15.0),
) -> MatchResult:
    """Coarse-to-fine correlative search around ``center``.

    ``runner_up_score`` is the best coarse score among poses at least
    ``separation_xy`` or ``separation_yaw`` away from the winner; it measures
    whether the scan is ambiguous inside the window.
    """
    if len(points) < 20:
        raise ValueError('Too few valid scan points for matching')
    coarse = _pose_grid(center, xy_window, yaw_window, coarse_xy_step, coarse_yaw_step)
    scores, _ = field.score_poses(points, coarse)
    best_index = int(np.argmax(scores))
    best = coarse[best_index]

    separated = (
        np.hypot(coarse[:, 0] - best[0], coarse[:, 1] - best[1]) >= separation_xy
    ) | (
        np.abs(np.arctan2(
            np.sin(coarse[:, 2] - best[2]), np.cos(coarse[:, 2] - best[2])
        )) >= separation_yaw
    )
    runner_up = float(np.max(scores[separated])) if np.any(separated) else 0.0

    fine = _pose_grid(
        tuple(best), coarse_xy_step, coarse_yaw_step, fine_xy_step, fine_yaw_step
    )
    fine_scores, fine_inliers = field.score_poses(points, fine)
    fine_index = int(np.argmax(fine_scores))
    winner = fine[fine_index]
    return MatchResult(
        x=float(winner[0]),
        y=float(winner[1]),
        yaw=normalize_angle(float(winner[2])),
        score=float(fine_scores[fine_index]),
        inlier_ratio=float(fine_inliers[fine_index]),
        runner_up_score=runner_up,
        point_count=int(len(points)),
    )


def accept(
    result: Optional[MatchResult],
    minimum_inlier_ratio: float,
    maximum_ambiguity: float,
) -> Tuple[bool, str]:
    """Apply the production acceptance gate and explain the decision."""
    if result is None:
        return False, 'no scan match result'
    summary = (
        f'inliers {result.inlier_ratio:.0%}, score {result.score:.2f}, '
        f'ambiguity {result.ambiguity:.2f}'
    )
    if result.inlier_ratio < minimum_inlier_ratio:
        return False, f'lidar does not confirm pose ({summary})'
    if result.ambiguity > maximum_ambiguity:
        return False, f'lidar match is ambiguous ({summary})'
    return True, summary


def occupancy_from_message(message) -> Tuple[np.ndarray, float, float, float]:
    """Return (grid, resolution, origin_x, origin_y) from nav_msgs/OccupancyGrid."""
    info = message.info
    q = info.origin.orientation
    origin_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    if abs(origin_yaw) > 1e-3:
        raise ValueError('Rotated map origins are not supported')
    grid = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
    origin = info.origin.position
    return grid, float(info.resolution), float(origin.x), float(origin.y)
