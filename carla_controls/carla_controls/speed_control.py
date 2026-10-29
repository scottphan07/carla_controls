"""Speed control: vehicle state + planning target -> (throttle, brake).
"""
from .messages import PlanningInput, VehicleState


class SpeedController:
    def __init__(self, dt: float):
        self.dt = dt
        ''' May have to change based on CARLASim'''
        self.kp_throttle = 0.1
        self.kp_brake = 0.1
        self.deadband = 0.1

        # Throttle rate limiting
        self.max_change_per_second = 1.0
        self.prev_throttle = 0.0

    def reset(self):
        self.prev_throttle = 0.0

    def compute(self, state: VehicleState, plan: PlanningInput) -> tuple:
        error = plan.desired_speed - state.speed

        throttle = 0.0
        brake = 0.0

        if abs(error) >= self.deadband:
            if error > 0:
                throttle = min(1.0, self.kp_throttle * error)
            else:
                brake = min(1.0, self.kp_brake * abs(error))

        # Limit how quickly throttle increases
        max_change = self.max_change_per_second * self.dt

        throttle = min(
            throttle,
            self.prev_throttle + max_change
        )

        self.prev_throttle = throttle

        # Always return both values
        return throttle, brake

