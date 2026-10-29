"""Speed control: vehicle state + planning target -> (throttle, brake).

TODO: implement. Return values in [0, 1]; controller.py clamps them and never
sends throttle and brake together.
"""
from .messages import PlanningInput, VehicleState


class SpeedController:
    def __init__(self, dt: float):
        self.dt = dt  # tick length [s]

    def compute(self, state: VehicleState, plan: PlanningInput) -> tuple:
        """Return (throttle, brake)."""
        return 0.0, 0.0  # TODO
