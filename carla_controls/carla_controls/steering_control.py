"""Steering control: vehicle state + planning target -> steer.

TODO: implement. Return steer in [-1, 1], positive = LEFT (ROS convention);
controls_node.py flips the sign for CARLA.
"""
from .messages import PlanningInput, VehicleState


class SteeringController:
    def __init__(self, dt: float):
        self.dt = dt  # tick length [s]

    def compute(self, state: VehicleState, plan: PlanningInput) -> float:
        """Return steer in [-1, 1] (positive = left)."""
        return 0.0  # TODO
