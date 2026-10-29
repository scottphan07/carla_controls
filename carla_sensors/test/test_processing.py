"""Unit tests for processing.py. Run: cd carla_sensors && python3 -m pytest test/
(or `colcon test --packages-select carla_sensors` in the workspace)."""
import numpy as np

from carla_sensors.processing import LidarFilter, Watchdog, bgra_to_rgb, filter_points


# ---------------------------------------------------------------- camera
def test_bgra_to_rgb_reorders_channels_and_drops_alpha():
    # 1 x 2 image: pixel 0 is pure red, pixel 1 is pure blue (stored as B, G, R, A)
    data = bytes([0, 0, 255, 255,   255, 0, 0, 255])
    rgb = bgra_to_rgb(data, height=1, width=2, step=8)
    assert rgb.shape == (1, 2, 3)
    assert rgb.dtype == np.uint8
    assert rgb[0, 0].tolist() == [255, 0, 0]
    assert rgb[0, 1].tolist() == [0, 0, 255]


def test_bgra_to_rgb_ignores_row_padding():
    # 2 x 1 image with 4 bytes of padding at the end of each row (step = 8, width * 4 = 4)
    data = bytes([10, 20, 30, 255, 9, 9, 9, 9,
                  40, 50, 60, 255, 9, 9, 9, 9])
    rgb = bgra_to_rgb(data, height=2, width=1, step=8)
    assert rgb[:, 0].tolist() == [[30, 20, 10], [60, 50, 40]]


def test_bgra_to_rgb_full_carla_frame_size():
    h, w = 600, 800
    rgb = bgra_to_rgb(bytes(h * w * 4), height=h, width=w, step=w * 4)
    assert rgb.shape == (h, w, 3)
    assert rgb.flags["C_CONTIGUOUS"]  # tobytes() gives rows in order


# ---------------------------------------------------------------- lidar
POINTS = np.array([
    [1.0, 0.0, 0.0, 1.0],     # 1 m ahead
    [10.0, 0.0, -2.4, 0.5],   # road, 10 m ahead
    [30.0, 0.0, 0.0, 0.2],    # 30 m ahead
    [5.0, 0.0, 3.0, 0.8],     # overhead
], dtype=np.float32)


def test_default_filter_keeps_everything():
    assert LidarFilter().is_pass_through()
    assert filter_points(POINTS, LidarFilter()) is POINTS


def test_range_filter():
    kept = filter_points(POINTS, LidarFilter(min_range=2.0, max_range=20.0))
    assert kept[:, 0].tolist() == [10.0, 5.0]


def test_ground_removal_with_min_z():
    kept = filter_points(POINTS, LidarFilter(min_z=-2.2))
    assert -2.4 not in kept[:, 2]
    assert len(kept) == 3


def test_max_z():
    kept = filter_points(POINTS, LidarFilter(max_z=2.0))
    assert 3.0 not in kept[:, 2]


def test_empty_cloud():
    empty = np.zeros((0, 4), dtype=np.float32)
    assert filter_points(empty, LidarFilter(min_z=-2.2)).shape == (0, 4)


# ---------------------------------------------------------------- watchdog
def test_watchdog_reports_stale_once_then_recovery():
    w = Watchdog(timeout=2.0)
    assert w.seen("lidar", 0.0) is False
    assert w.check(1.0) == []                  # still fresh
    assert w.check(2.5) == [("lidar", 2.5)]    # went stale
    assert w.check(3.0) == []                  # reported only once
    assert w.seen("lidar", 3.1) is True        # came back
    assert w.check(3.2) == []


def test_watchdog_ignores_sensors_never_seen():
    assert Watchdog(timeout=1.0).check(100.0) == []
