"""Stop behavior: decide whether the car must stop, and why."""

import math
from typing import Optional
from .messages import (PerceptionInput, PlanningInput, StopReason, VehicleState,)

# Initial values to tune in CARLA
MAX_PLAN_AGE = 0.5          # seconds
MAX_PERCEPTION_AGE = 0.5    # seconds
MIN_OBSTACLE_DISTANCE = 5.0 # meters
STOP_SPEED_THRESHOLD = 0.1  # m/s


def check_stop(state: VehicleState, plan: Optional[PlanningInput], perception: Optional[PerceptionInput], emergency_stop: bool,
) -> Optional[StopReason]:

    # Emergency stop has highest priority
    if emergency_stop:
        return StopReason.EMERGENCY_STOP

    # No planning target received
    if plan is None:
        return StopReason.MISSING_DATA

    # Planning data is stale or invalid
    plan_age = state.timestamp - plan.timestamp

    if (not math.isfinite(plan_age) or plan_age < 0 or plan_age > MAX_PLAN_AGE):
        return StopReason.MISSING_DATA

    # Perception data is missing or stale
    if perception is None:
        return StopReason.MISSING_DATA

    perception_age = state.timestamp - perception.timestamp

    if (not math.isfinite(perception_age) or perception_age < 0 or perception_age > MAX_PERCEPTION_AGE):
        return StopReason.MISSING_DATA

    # Obstacle detected ahead
    distance = perception.obstacle_distance

    if distance is not None:
        if math.isnan(distance) or distance < 0:
            return StopReason.MISSING_DATA
        if distance < MIN_OBSTACLE_DISTANCE:
            return StopReason.OBSTACLE

    # Planning requests a stop
    if plan.brake_request or plan.desired_speed <= STOP_SPEED_THRESHOLD:
        return StopReason.TARGET_SPEED_ZERO

    return None
