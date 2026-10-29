"""Ties the pieces together: inputs -> stop check -> speed + steering -> validated command.

No CARLA imports here, so this can be tested (or wrapped in ROS 2) without a simulator.
"""
from typing import Optional

from .messages import Command, ControlsOutput, PerceptionInput, PlanningInput, VehicleState
from .speed_control import SpeedController
from .steering_control import SteeringController
from .stop_behavior import check_stop
import math


def clamp(value, low, high):
        if not math.isinfinite(value):
            raise ValueError("Invalid control command")
        return max(low, min(high, value))  

class Controller:
    def __init__(self, dt: float):
        self.speed = SpeedController(dt)
        self.steering = SteeringController(dt)
        self.emergency_stop = False  # set from /controls/emergency_stop

    def step(
        self,
        state: VehicleState,
        plan: Optional[PlanningInput],
        perception: Optional[PerceptionInput],
    ) -> ControlsOutput:
        stop_reason = check_stop(state, plan, perception, self.emergency_stop)

        if stop_reason is not None:
            self.speed.reset()
            self.steering.reset()
            command = Command(throttle=0.0, steer=0.0, brake=1.0)
        else:
            throttle, brake = self.speed.compute(state, plan)
            steer = self.steering.compute(state, plan)
            command = Command(throttle=throttle, steer=steer, brake=brake)

        # Make sure everything is valid before it goes to the car.
        command.throttle = min(max(command.throttle, 0.0), 1.0)
        command.brake = min(max(command.brake, 0.0), 1.0)
        command.steer = min(max(command.steer, -1.0), 1.0)
        if command.brake > 0.0:
            command.throttle = 0.0  # never throttle and brake together

        return ControlsOutput(state=state, command=command, stop_reason=stop_reason)

