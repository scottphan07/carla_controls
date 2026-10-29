"""Sensor processing in plain Python + numpy (no ROS), so it can be tested without a simulator.

sensors_node.py unpacks ROS messages into numpy arrays, calls these functions, and packs the
results back into messages.

Conventions (same as the rest of the project): ROS axes (x forward, y left, z up), meters.
Lidar points are relative to the lidar itself, which is mounted 2.4 m above the ground,
so the road is at about z = -2.4.
"""
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np


# ---------------------------------------------------------------- camera
def bgra_to_rgb(data, height: int, width: int, step: int) -> np.ndarray:
    """CARLA camera bytes (bgra8) -> H x W x 3 uint8 RGB array.

    `step` is bytes per row; usually width * 4, but rows may be padded.
    """
    buf = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)
    pixels = buf[:, : width * 4].reshape(height, width, 4)
    return np.ascontiguousarray(pixels[:, :, [2, 1, 0]])  # drop alpha, BGR -> RGB


# ---------------------------------------------------------------- lidar
@dataclass
class LidarFilter:
    """Which points to keep. The defaults keep everything the 50 m lidar can see."""
    min_range: float = 0.0     # [m] drop points closer than this (e.g. hits on our own car)
    max_range: float = 100.0   # [m] drop points farther than this
    min_z: float = -100.0      # [m] drop points below this; about -2.2 removes the road
    max_z: float = 100.0       # [m] drop points above this (overhangs, trees)

    def is_pass_through(self) -> bool:
        return self == LidarFilter()


def filter_points(points: np.ndarray, f: LidarFilter) -> np.ndarray:
    """points: N x 4 float32 array of (x, y, z, intensity). Returns the rows that pass `f`.

    TODO: smarter ground removal (plane fit) instead of a flat z cut, downsampling,
    a region of interest in front of the car.
    """
    if f.is_pass_through() or len(points) == 0:
        return points
    dist = np.linalg.norm(points[:, :3], axis=1)
    keep = (
        (dist >= f.min_range) & (dist <= f.max_range)
        & (points[:, 2] >= f.min_z) & (points[:, 2] <= f.max_z)
    )
    return points[keep]


# ---------------------------------------------------------------- health
class Watchdog:
    """Notices when a sensor stops publishing and when it comes back.

    Call seen(name, now) on every message and check(now) periodically. `now` is any clock in
    seconds; sensors_node uses wall time, because sim time freezes when the bridge stops.
    """

    def __init__(self, timeout: float):
        self.timeout = timeout
        self.last_seen: Dict[str, float] = {}
        self.stale: Dict[str, bool] = {}

    def seen(self, name: str, now: float) -> bool:
        """Record a message. Returns True if this sensor was stale and just came back."""
        self.last_seen[name] = now
        was_stale = self.stale.get(name, False)
        self.stale[name] = False
        return was_stale

    def check(self, now: float) -> List[Tuple[str, float]]:
        """Return (name, seconds since last message) for sensors that just went stale."""
        newly_stale = []
        for name, t in self.last_seen.items():
            age = now - t
            if age > self.timeout and not self.stale[name]:
                self.stale[name] = True
                newly_stale.append((name, age))
        return newly_stale
