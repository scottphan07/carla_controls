"""Steering control: vehicle state + planning target -> steer.

Return steer in [-1, 1], positive = LEFT (ROS convention).
"""

import math
from .messages import PlanningInput, VehicleState

class SteeringController:
    def __init__(self, dt: float):
        self.dt = dt

        # Initial tuning parameters
        self.kp_steer = 1.0
        self.max_steer_rate = 1.0  # Command units per second

        self.prev_steer = 0.0

    def compute(self, state: VehicleState, plan: PlanningInput) -> float:

        # Direction from vehicle to target
        dx = plan.target_x - state.x
        dy = plan.target_y - state.y

        # Calculate desired heading using theta_target = atan(y_t - y, x_t - x)
        target_yaw = math.atan2(dy, dx)
        # Difference between target heading and current heading
        heading_error = target_yaw - state.yaw
        # Normalize angle to [-pi, pi]
        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )

        # Proportional steering controller
        steer = self.kp_steer * heading_error

        # Clamp steering to valid range
        steer = max(-1.0, min(1.0, steer))

        # Limit sudden steering changes
        max_change = self.max_steer_rate * self.dt

        steer = max(
            self.prev_steer - max_change,
            min(self.prev_steer + max_change, steer)
        )

        self.prev_steer = steer

        return steer