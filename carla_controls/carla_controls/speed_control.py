"""Speed control: speed error -> throttle or brake.

TODO (from the brainstorm):
- Smooth throttle / brake (rate limit, or a proper PID)
- Avoid rapid switching between brake and throttle (deadband / hysteresis)
- Decide the rule if both are ever non-zero (currently: never both)
- Keep values in [0, 1] (also enforced again in controller.py)
"""
from .messages import PlanningInput, VehicleState


class SpeedController:
    def __init__(self, dt: float, kp: float = 0.3):
        self.dt = dt  # tick length [s], for when this becomes a PID
        self.kp = kp

    def compute(self, state: VehicleState, plan: PlanningInput) -> tuple:
        """Return (throttle, brake)."""
        # Placeholder: proportional control on speed error.
        error = plan.desired_speed - state.speed
        effort = self.kp * error
        if effort >= 0:
            return min(effort, 1.0), 0.0
        return 0.0, min(-effort, 1.0)
