import math
from pathlib import Path

import numpy as np
import pytest

from shbat_pkg.scan_matcher import (
    LikelihoodField, MatchResult, accept, evaluate, scan_to_points, search,
)


def _room_grid():
    # 8 m x 6 m room at 5 cm with an asymmetric pillar and alcove.
    grid = np.zeros((120, 160), dtype=np.int16)
    grid[0, :] = grid[-1, :] = 100
    grid[:, 0] = grid[:, -1] = 100
    grid[30:45, 40:52] = 100       # pillar
    grid[80:120, 110] = 100        # partition
    grid[60, 0:25] = 100           # stub wall
    return grid


def _raycast(grid, resolution, origin, pose, beams=360, max_range=8.0):
    occupied = grid >= 65
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    ranges = np.full(beams, np.inf)
    steps = np.arange(0.05, max_range, resolution / 2.0)
    for index, angle in enumerate(angles):
        heading = pose[2] + angle
        xs = pose[0] + steps * math.cos(heading)
        ys = pose[1] + steps * math.sin(heading)
        cols = np.floor((xs - origin[0]) / resolution).astype(int)
        rows = np.floor((ys - origin[1]) / resolution).astype(int)
        inside = (cols >= 0) & (cols < grid.shape[1]) & (rows >= 0) & (rows < grid.shape[0])
        hit = np.zeros_like(inside)
        hit[inside] = occupied[rows[inside], cols[inside]]
        if np.any(hit):
            ranges[index] = steps[int(np.argmax(hit))]
    return angles, ranges


def _points(grid, resolution, origin, pose, rng, occlude=0.0):
    angles, ranges = _raycast(grid, resolution, origin, pose)
    ranges = ranges + rng.normal(0.0, 0.015, ranges.shape)
    if occlude:
        people = rng.random(ranges.shape) < occlude
        ranges[people] = np.minimum(ranges[people], rng.uniform(0.4, 1.2, people.sum()))
    return scan_to_points(ranges, angles[0], angles[1] - angles[0], 0.1, 12.0)


def test_search_recovers_offset_dock_pose_in_room():
    rng = np.random.default_rng(1)
    grid = _room_grid()
    field = LikelihoodField(grid, 0.05, 0.0, 0.0)
    truth = (2.3, 3.1, 0.35)
    points = _points(grid, 0.05, (0.0, 0.0), truth, rng)
    prior = (truth[0] + 0.32, truth[1] - 0.25, truth[2] - math.radians(18))
    result = search(field, points, prior)
    assert math.hypot(result.x - truth[0], result.y - truth[1]) < 0.05
    assert abs(result.yaw - truth[2]) < math.radians(2)
    ok, message = accept(result, 0.55, 0.9)
    assert ok, message


def test_visitors_occluding_beams_still_match():
    rng = np.random.default_rng(2)
    grid = _room_grid()
    field = LikelihoodField(grid, 0.05, 0.0, 0.0)
    truth = (5.0, 2.0, -1.2)
    points = _points(grid, 0.05, (0.0, 0.0), truth, rng, occlude=0.25)
    result = search(field, points, (5.2, 2.2, -1.0))
    assert math.hypot(result.x - truth[0], result.y - truth[1]) < 0.06
    assert accept(result, 0.55, 0.9)[0]


def test_wrong_pose_is_rejected_by_inlier_gate():
    rng = np.random.default_rng(3)
    grid = _room_grid()
    field = LikelihoodField(grid, 0.05, 0.0, 0.0)
    points = _points(grid, 0.05, (0.0, 0.0), (2.3, 3.1, 0.35), rng)
    wrong = evaluate(field, points, (4.0, 1.5, 2.0))
    assert not accept(wrong, 0.55, 0.9)[0]


def test_symmetric_corridor_is_flagged_ambiguous():
    grid = np.zeros((40, 400), dtype=np.int16)
    grid[0, :] = grid[-1, :] = 100   # long featureless corridor
    field = LikelihoodField(grid, 0.05, 0.0, 0.0)
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    _a, ranges = _raycast(grid, 0.05, (0.0, 0.0), (10.0, 1.0, 0.0), max_range=4.0)
    points = scan_to_points(ranges, angles[0], angles[1] - angles[0], 0.1, 4.0)
    result = search(field, points, (10.0, 1.0, 0.0))
    assert not accept(result, 0.55, 0.9)[0]


def test_ambiguity_ratio_handles_zero_score():
    assert MatchResult(0, 0, 0, 0.0, 0.0).ambiguity == 1.0


GALLERY = next((p for p in (
    Path.home() / 'sahabat_ws' / 'maps' / 'gallerysq4.pgm',
    Path('/mnt/user-data/uploads/sahabat_ws/maps/gallerysq4.pgm'),
) if p.exists()), Path('missing.pgm'))


@pytest.mark.skipif(not GALLERY.exists(), reason='gallery map not available')
def test_real_gallery_map_recovers_poses():
    data = GALLERY.read_bytes()
    header = data.split(b'\n', 4)
    width, height = (int(v) for v in header[2 if header[1].startswith(b'#') else 1].split())
    pixels = np.frombuffer(data[-width * height:], dtype=np.uint8).reshape(height, width)
    occupancy = np.where((255 - pixels) / 255.0 >= 0.65, 100, 0).astype(np.int16)
    occupancy = np.flipud(occupancy)
    origin = (-10.2, -12.8)
    field = LikelihoodField(occupancy, 0.05, *origin)
    rng = np.random.default_rng(5)
    free = np.argwhere(np.flipud(pixels) >= 250)
    checked = 0
    for row, col in free[rng.choice(len(free), 40, replace=False)]:
        truth = (origin[0] + (col + 0.5) * 0.05, origin[1] + (row + 0.5) * 0.05,
                 rng.uniform(-math.pi, math.pi))
        if field.lookup(np.array([truth[0]]), np.array([truth[1]]))[0] < 0.3:
            continue
        points = _points(occupancy, 0.05, origin, truth, rng, occlude=0.15)
        if len(points) < 120:
            continue
        prior = (truth[0] + 0.25, truth[1] - 0.2, truth[2] + math.radians(12))
        result = search(field, points, prior)
        ok, _ = accept(result, 0.55, 0.9)
        if ok:
            assert math.hypot(result.x - truth[0], result.y - truth[1]) < 0.1
            assert abs(math.atan2(math.sin(result.yaw - truth[2]),
                                  math.cos(result.yaw - truth[2]))) < math.radians(3)
        checked += 1
    assert checked >= 10
