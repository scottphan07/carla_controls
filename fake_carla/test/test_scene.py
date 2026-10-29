"""Tests for the fake world and the output checks (no ROS needed).

    cd fake_carla && python3 -m pytest test/

The last test runs the fake data through carla_sensors' real processing code, so it also
covers the sensors pipeline end to end (skipped if carla_sensors isn't importable).
"""
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from fake_carla.checks import check_camera_info, check_image, check_lidar, check_rate, write_png
from fake_carla.scene import (CAMERA_H, CAMERA_W, LIDAR_RANGE, FakeScene, camera_K,
                              yaw_to_quaternion)


@pytest.fixture
def scene():
    return FakeScene()


# ---------------------------------------------------------------- camera
def test_camera_is_carla_shaped_bgra(scene):
    img = scene.render_camera()
    assert img.shape == (CAMERA_H, CAMERA_W, 4) and img.dtype == np.uint8
    assert (img[:, :, 3] == 255).all()
    assert img.flags["C_CONTIGUOUS"]


def test_camera_shows_sky_on_top_and_road_below(scene):
    img = scene.render_camera()
    b, g, r = (int(v) for v in img[10, 400, :3])     # top center: sky is blue-ish
    assert b > r
    b, g, r = (int(v) for v in img[590, 300, :3])    # bottom: gray asphalt
    assert abs(b - r) < 15


def test_camera_sees_red_car_ahead(scene):
    scene.car.x = scene.obstacle_x - 15.0
    img = scene.render_camera()
    center = img[250:350, 350:450]
    red = (center[:, :, 2] > 120) & (center[:, :, 0] < 60)
    assert red.sum() > 500


def test_camera_intrinsics():
    K = camera_K()
    assert K[0, 0] == pytest.approx(400.0) and K[0, 2] == 400.0 and K[1, 2] == 300.0


# ---------------------------------------------------------------- lidar
def test_lidar_points_are_carla_shaped(scene):
    pts = scene.lidar_points()
    assert pts.dtype == np.float32 and pts.shape[1] == 4
    assert 10_000 < len(pts) <= 16_000
    dist = np.linalg.norm(pts[:, :3], axis=1)
    assert dist.max() <= LIDAR_RANGE + 1e-3
    assert np.allclose(pts[:, 3], np.exp(-0.004 * dist), atol=1e-4)


def test_lidar_ground_is_2_4_m_below_sensor(scene):
    pts = scene.lidar_points()
    assert pts[:, 2].min() == pytest.approx(-2.4, abs=1e-3)


def test_lidar_sees_parked_car_at_right_distance(scene):
    scene.car.x = scene.obstacle_x - 10.0
    pts = scene.lidar_points()
    ahead = pts[(np.abs(pts[:, 1]) < 1.0) & (pts[:, 0] > 0) & (pts[:, 2] > -2.3)]
    assert len(ahead) > 50
    assert ahead[:, 0].min() == pytest.approx(10.0, abs=0.05)


def test_lidar_noise_is_optional_and_seeded(scene):
    a = scene.lidar_points(np.random.default_rng(1), 0.02)
    b = scene.lidar_points(np.random.default_rng(1), 0.02)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, scene.lidar_points())


# ---------------------------------------------------------------- driving
def test_cruise_moves_forward(scene):
    for _ in range(20):
        scene.cruise(0.05, 5.0)
    assert scene.car.x == pytest.approx(5.0)
    assert scene.time == pytest.approx(1.0)


def test_throttle_brake_and_carla_steer_sign(scene):
    for _ in range(40):
        scene.step(0.05, throttle=0.5, steer_carla=0.0, brake=0.0)
    assert scene.car.speed > 3.0
    for _ in range(20):
        scene.step(0.05, throttle=0.0, steer_carla=0.3, brake=0.0)   # CARLA + = right
    assert scene.car.yaw < 0                                         # ROS: turning right = negative yaw
    for _ in range(100):
        scene.step(0.05, throttle=0.0, steer_carla=0.0, brake=1.0)
    assert scene.car.speed == 0.0                                    # brakes never reverse the car


def test_parked_car_respawns_ahead(scene):
    first = scene.obstacle_x
    scene.car.x = first + 10.0
    scene.cruise(0.05, 5.0)
    assert scene.obstacle_x > scene.car.x


def test_quaternion():
    x, y, z, w = yaw_to_quaternion(np.pi / 2)
    assert (x, y) == (0.0, 0.0) and z == pytest.approx(w)


# ---------------------------------------------------------------- checks used by check_sensors
def _image(**kw):
    m = dict(encoding="rgb8", width=800, height=600, step=2400, data=bytes(2400 * 600),
             header=SimpleNamespace(frame_id="ego_vehicle/rgb_front"))
    m.update(kw)
    return SimpleNamespace(**m)


def test_check_image():
    assert check_image(_image()) == []
    assert any("encoding" in p for p in check_image(_image(encoding="bgra8")))
    assert any("bytes" in p for p in check_image(_image(data=b"")))


def test_check_camera_info():
    ok = SimpleNamespace(width=800, height=600, k=camera_K().flatten().tolist())
    assert check_camera_info(ok) == []
    assert check_camera_info(SimpleNamespace(width=800, height=600, k=[0.0] * 9))


def test_check_lidar():
    fields = [SimpleNamespace(name=n, datatype=7) for n in ("x", "y", "z", "intensity")]
    ok = SimpleNamespace(fields=fields, width=10, height=1, point_step=16, row_step=160,
                         data=bytes(160), header=SimpleNamespace(frame_id="ego_vehicle/lidar"))
    assert check_lidar(ok) == []
    ok.fields = fields[:3]
    assert check_lidar(ok) == ["missing field 'intensity'"]


def test_check_rate():
    assert check_rate(100, 5.0, 15.0) == []
    assert check_rate(10, 5.0, 15.0)


def test_write_png(tmp_path):
    path = tmp_path / "x.png"
    write_png(str(path), bytes(range(6)) * 2, width=2, height=2)
    assert path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


# ---------------------------------------------------------------- end to end with carla_sensors
def test_fake_data_through_sensors_processing(scene):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "carla_sensors"))
    processing = pytest.importorskip("carla_sensors.processing")

    bgra = scene.render_camera()
    rgb = processing.bgra_to_rgb(bgra.tobytes(), CAMERA_H, CAMERA_W, CAMERA_W * 4)
    assert rgb.shape == (CAMERA_H, CAMERA_W, 3)
    assert (rgb[:, :, 0] == bgra[:, :, 2]).all()          # red channel moved to the front

    pts = scene.lidar_points()
    no_ground = processing.filter_points(pts, processing.LidarFilter(min_z=-2.2))
    assert len(no_ground) < len(pts)
    assert no_ground[:, 2].min() >= -2.2
