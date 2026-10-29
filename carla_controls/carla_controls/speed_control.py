"""Speed control: vehicle state + planning target -> (throttle, brake).

TODO: implement. Return values in [0, 1]; controller.py clamps them and never
sends throttle and brake together.
"""
from .messages import PlanningInput, VehicleState


class SpeedController:
    def __init__(self, dt: float):
        self.dt = dt  # Tick length [s]

        '''***Have to experiment values o kp_throttle, 
        kp_brake, and deadband in Carla***'''
        # Proportional gains
        self.kp_throttle = 0.3
        self.kp_brake = 0.4
        # Speed tolerance in m/s
        self.deadband = 0.1

    def compute(self, state, plan):
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
