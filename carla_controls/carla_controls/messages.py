"""Data passed into and out of the controls logic.

Everything uses the ROS convention, matching what the CARLA ROS bridge publishes:
frame "map", x forward, y to the LEFT, yaw in radians (counter-clockwise positive),
meters and m/s. controls_node.py converts ROS messages into these and back.

TODO: agree on these fields, units, and frames with planning and perception.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Optional


# ---------------------------------------------------------------- Controls input
@dataclass
class VehicleState:
    """Current vehicle state, from the bridge's odometry every tick."""
    timestamp: float  # sim time [s]
    x: float  # [m]
    y: float  # [m]
    yaw: float  # [rad]
    speed: float  # [m/s]
    # TODO: acceleration, yaw rate, ... if the controllers need them


@dataclass
class PlanningInput:
    """What planning asks for (from controls_msgs/ControlTarget)."""
    timestamp: float  # when planning produced this [s]; used to detect stale data
    desired_speed: float  # [m/s]
    target_x: float  # desired position [m]
    target_y: float
    target_yaw: Optional[float] = None  # desired direction [rad]; None = point at the target
    brake_request: bool = False  # planning wants the car stopped
    # TODO: a whole path / trajectory instead of a single target?


@dataclass
class PerceptionInput:
    """What perception tells us (from sensor_msgs/Range)."""
    timestamp: float  # [s]
    obstacle_distance: Optional[float] = None  # nearest obstacle ahead [m]; None = nothing seen


# ---------------------------------------------------------------- Controls output
@dataclass
class Command:
    """Same fields as carla.VehicleControl, so the controller code needs no ROS or CARLA imports."""
    throttle: float = 0.0  # [0, 1]
    steer: float = 0.0  # [-1, 1], positive = LEFT (ROS); controls_node flips it for CARLA
    brake: float = 0.0  # [0, 1]
    hand_brake: bool = False
    reverse: bool = False


class StopReason(str, Enum):
    TARGET_SPEED_ZERO = "target_speed_zero"
    EMERGENCY_STOP = "emergency_stop"
    OBSTACLE = "obstacle"
    MISSING_DATA = "missing_data"


@dataclass
class ControlsOutput:
    """Published on /controls/status after every tick."""
    state: VehicleState
    command: Command  # what was actually sent to the car
    stop_reason: Optional[StopReason] = None  # None = driving normally
