"""Steering control: compare with the target path -> steer.

TODO (from the brainstorm):
- Pick the real algorithm (Pure Pursuit / Stanley / MPC) and compare against the target path
- Limit steering by what is physically valid, and less at higher speed (avoid drifting)
- Prevent sudden steering (rate limit)
"""
import math

from .messages import PlanningInput, VehicleState


class SteeringController:
    def __init__(self, dt: float, kp: float = 1.0, max_steer: float = math.radians(70.0)):
        self.dt = dt
        self.kp = kp
        self.max_steer = max_steer  # wheel angle at steer = 1.0 [rad]; TODO: read from /carla/ego_vehicle/vehicle_info

    def compute(self, state: VehicleState, plan: PlanningInput) -> float:
        """Return steer in [-1, 1] (positive = left)."""
        # Placeholder: point the car at the target position.
        if plan.target_yaw is not None:
            desired_yaw = plan.target_yaw
        else:
            desired_yaw = math.atan2(plan.target_y - state.y, plan.target_x - state.x)
        heading_error = (desired_yaw - state.yaw + math.pi) % (2.0 * math.pi) - math.pi  # wrap to [-pi, pi)
        return self.kp * heading_error / self.max_steer
