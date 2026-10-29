"""Tests for drive_test's logic (no ROS). Run: cd fake_carla && python3 -m pytest test/"""
import math

import numpy as np
import pytest

from fake_carla.drive_logic import DriveTest, goal_ahead, obstacle_distance
from fake_carla.scene import FakeScene


def test_goal_ahead_follows_heading():
    assert goal_ahead(10.0, 5.0, 0.0, 30.0) == pytest.approx((40.0, 5.0))
    assert goal_ahead(0.0, 0.0, math.pi / 2, 10.0) == pytest.approx((0.0, 10.0))


def test_obstacle_distance_ignores_road_hood_and_other_lanes():
    pts = np.array([
        [10.0, 0.0, -2.4],    # road surface
        [2.0, 0.0, -1.0],     # our own hood
        [8.0, 5.0, -1.0],     # car in the next lane
        [60.0, 0.0, -1.0],    # too far
    ], dtype=np.float32)
    assert obstacle_distance(pts) is None
    assert obstacle_distance(np.vstack([pts, [[12.0, 0.3, -1.5]]])) == pytest.approx(12.0 - 2.4)
    assert obstacle_distance(np.zeros((0, 3))) is None


def test_obstacle_distance_on_fake_lidar():
    scene = FakeScene()
    scene.car.x = scene.obstacle_x - 15.0
    assert obstacle_distance(scene.lidar_points()) == pytest.approx(15.0 - 2.4, abs=0.1)
    scene.car.x = scene.obstacle_x - 70.0
    assert obstacle_distance(scene.lidar_points()) is None


def test_drive_test_asks_for_speed_then_stops_and_passes():
    t = DriveTest(distance=30.0, speed=5.0)
    assert t.update(0.0, 0.0, 0.0, 0.0, 0.0) == (5.0, False)
    assert t.goal == pytest.approx((30.0, 0.0))
    assert t.update(5.0, 28.0, 0.0, 0.0, 3.0) == (0.0, True)    # inside the 3 m radius
    assert t.result(5.0) is None
    t.update(6.0, 28.5, 0.0, 0.0, 0.0)
    assert t.result(7.0) is None
    t.update(8.1, 28.5, 0.0, 0.0, 0.0)
    assert t.result(8.1) is True


def test_drive_test_times_out():
    t = DriveTest(distance=30.0, timeout=10.0)
    t.update(0.0, 0.0, 0.0, 0.0, 0.0)
    t.update(11.0, 1.0, 0.0, 0.0, 0.0)
    assert t.result(11.0) is False
