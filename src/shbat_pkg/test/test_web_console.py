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
