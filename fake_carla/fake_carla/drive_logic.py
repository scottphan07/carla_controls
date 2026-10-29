"""Logic for drive_test, in plain Python + numpy (no ROS), so it can be unit-tested.

drive_test stands in for planning and perception so controls can be tried end to end:
  planning   -> a goal some distance ahead of where the car starts; full speed until close,
                then desired_speed 0 so controls brakes and holds
  perception -> distance from our front bumper to the nearest lidar point in our lane ahead
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Lane corridor ahead of the lidar for "is something in front of us" (lidar frame, meters).
# The lidar sits 2.4 m above the road, so z > -2.0 skips the road surface.
CORRIDOR_HALF_WIDTH = 1.2
CORRIDOR_MIN_Z = -2.0
CORRIDOR_MAX_Z = 0.5
CORRIDOR_MAX_X = 50.0
FRONT_BUMPER_X = 2.4       # [m] front bumper ahead of the lidar (Model 3: 4.7 m long, lidar mid-car)


def goal_ahead(x: float, y: float, yaw: float, distance: float) -> Tuple[float, float]:
    """The point `distance` meters straight ahead of a pose."""
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw)


def obstacle_distance(points: np.ndarray) -> Optional[float]:
    """Gap from our front bumper to the nearest point in our lane (N x >=3 array, lidar frame).

    None if the lane is clear. Points behind the bumper (our own hood and roof) are ignored.
    """
    if len(points) == 0:
        return None
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    ahead = ((x > FRONT_BUMPER_X + 0.1) & (x < CORRIDOR_MAX_X) & (np.abs(y) < CORRIDOR_HALF_WIDTH)
             & (z > CORRIDOR_MIN_Z) & (z < CORRIDOR_MAX_Z))
    return float(x[ahead].min() - FRONT_BUMPER_X) if ahead.any() else None


@dataclass
class DriveTest:
    """Tracks one run: where the goal is, what speed to ask for, and whether we passed."""
    distance: float = 30.0       # [m] goal this far ahead of the start
    speed: float = 5.0           # [m/s] requested speed
    stop_radius: float = 3.0     # [m] ask for speed 0 inside this radius of the goal
    timeout: float = 60.0        # [s, sim time]
    goal: Optional[Tuple[float, float]] = None
    start_time: Optional[float] = None
    max_speed: float = 0.0
    stopped_since: Optional[float] = None

    def update(self, t: float, x: float, y: float, yaw: float, speed: float) -> Tuple[float, bool]:
        """Feed the latest odometry. Returns (desired_speed, brake_request) to send to controls."""
        if self.goal is None:
            self.goal = goal_ahead(x, y, yaw, self.distance)
            self.start_time = t
        self.max_speed = max(self.max_speed, speed)
        arrived = self.distance_to_goal(x, y) < self.stop_radius
        if arrived and speed < 0.1:
            self.stopped_since = self.stopped_since if self.stopped_since is not None else t
        else:
            self.stopped_since = None
        return (0.0, True) if arrived else (self.speed, False)

    def distance_to_goal(self, x: float, y: float) -> float:
        return math.hypot(self.goal[0] - x, self.goal[1] - y) if self.goal else float("inf")

    def result(self, t: float) -> Optional[bool]:
        """True = passed (stopped at the goal for 2 s), False = timed out, None = still running."""
        if self.stopped_since is not None and t - self.stopped_since >= 2.0:
            return True
        if self.start_time is not None and t - self.start_time > self.timeout:
            return False
        return None
