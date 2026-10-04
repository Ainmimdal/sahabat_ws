"""Non-motion tests for the browser console helpers."""

import io
import queue

import pytest

from shbat_pkg.web_console import encode_map_png, Hub


def test_map_png_flips_rows_and_colours_cells():
    """Row 0 of the grid is the bottom of the image; unknown is transparent."""
    Image = pytest.importorskip('PIL.Image')
    # 2x2 grid, row-major from the map origin (bottom-left).
    data = [100, 0, -1, 50]
    image = Image.open(io.BytesIO(encode_map_png(2, 2, data))).convert('RGBA')

    assert image.size == (2, 2)
    bottom_left = image.getpixel((0, 1))
    bottom_right = image.getpixel((1, 1))
    top_left = image.getpixel((0, 0))
    top_right = image.getpixel((1, 0))
    assert bottom_left[3] == 255 and bottom_left[0] < 64      # occupied
    assert bottom_right[3] == 255 and bottom_right[0] > 200   # free
    assert top_left[3] == 0                                   # unknown
    assert top_right[3] == 255 and 64 < top_right[0] < 244    # uncertain


def test_hub_replays_latest_state_to_new_clients():
    """A browser that connects late still receives the last map and status."""
    hub = Hub()
    hub.publish('status', {'n': 1})
    hub.publish('status', {'n': 2})
    hub.publish('map', {'version': 1})

    client = hub.subscribe()
    events = []
    while True:
        try:
            events.append(client.get_nowait())
        except queue.Empty:
            break

    assert len(events) == 2
    assert any('"n":2' in event for event in events)
    assert not any('"n":1' in event for event in events)
    hub.unsubscribe(client)
    assert client not in hub.clients


@pytest.mark.parametrize('name', ['gallery_oct', 'lab-2', 'A1'])
def test_map_names_accepted(name):
    """Map names become file stems under maps/."""
    from shbat_pkg.web_console import VALID_MAP_NAME
    assert VALID_MAP_NAME.fullmatch(name)


@pytest.mark.parametrize('name', ['', '../x', 'a b', '_lead', 'x/y', 'm.yaml'])
def test_map_names_rejected(name):
    """Reject names that could escape the maps folder or break file stems."""
    from shbat_pkg.web_console import VALID_MAP_NAME
    assert not VALID_MAP_NAME.fullmatch(name)


def test_upside_down_lidar_is_not_mirrored():
    """rpy (pi, 0, pi) maps laser (x, y) to base (-x, y), not a yaw-only turn."""
    from types import SimpleNamespace

    from shbat_pkg.web_console import planar_projection

    # Live base_link -> lidar_link transform on the robot (xyzw ~ 0, 1, 0, 0).
    transform = SimpleNamespace(
        rotation=SimpleNamespace(x=0.0, y=1.0, z=0.0, w=0.0),
        translation=SimpleNamespace(x=0.07, y=0.0, z=0.25),
    )
    r00, r01, r10, r11, tx, ty = planar_projection(transform)
    project = lambda x, y: (r00 * x + r01 * y + tx, r10 * x + r11 * y + ty)  # noqa: E731

    assert project(1.0, 0.0) == pytest.approx((-0.93, 0.0))
    # Yaw-only handling would have put this point at y = -1 (mirrored).
    assert project(0.0, 1.0) == pytest.approx((0.07, 1.0))


def test_planar_projection_plain_yaw():
    """A level frame rotated 90 degrees behaves like a 2D rotation."""
    import math
    from types import SimpleNamespace

    from shbat_pkg.web_console import planar_projection

    half = math.pi / 4.0
    transform = SimpleNamespace(
        rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
        translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
    )
    r00, r01, r10, r11, tx, ty = planar_projection(transform)
    assert (r00 * 1.0 + tx, r10 * 1.0 + ty) == pytest.approx((1.0, 3.0))


def test_write_map_files_matches_map_saver_format(tmp_path):
    """Trinary PGM (0/205/254, top row = max y) and nav2 map_saver YAML."""
    import yaml
    from nav_msgs.msg import OccupancyGrid

    from shbat_pkg.web_console import write_map_files

    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 3, 2, 0.05
    grid.info.origin.position.x, grid.info.origin.position.y = -1.5, 2.25
    grid.info.origin.orientation.w = 1.0
    grid.data = [100, 0, -1,    # bottom row (y = origin)
                 50, 25, 65]    # top row
    write_map_files(grid, tmp_path / 'room')

    pgm = (tmp_path / 'room.pgm').read_bytes()
    assert pgm == b'P5\n3 2\n255\n' + bytes([205, 254, 0, 0, 254, 205])
    meta = yaml.safe_load((tmp_path / 'room.yaml').read_text())
    assert meta == {
        'image': 'room.pgm', 'mode': 'trinary', 'resolution': 0.05,
        'origin': [-1.5, 2.25, 0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.25,
    }
    assert not list(tmp_path.glob('.*.partial'))
