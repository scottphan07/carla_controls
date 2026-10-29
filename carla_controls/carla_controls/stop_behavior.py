"""Stop behavior: decide whether the car must stop, and why.

Checked in priority order. Returns None when it's OK to drive.

TODO:
- How hard to brake for each reason (gentle for target speed 0, full for the rest?)
- Tune the thresholds below; maybe scale obstacle distance with speed
- How an emergency stop gets triggered and released
"""
from typing import Optional

from .messages import PerceptionInput, PlanningInput, StopReason, VehicleState

DATA_TIMEOUT = 0.5  # [s] planning data older than this counts as missing
MIN_OBSTACLE_DISTANCE = 5.0  # [m]


def check_stop(
    state: VehicleState,
    plan: Optional[PlanningInput],
    perception: Optional[PerceptionInput],
    emergency_stop: bool,
) -> Optional[StopReason]:
    if emergency_stop:
        return StopReason.EMERGENCY_STOP
    if plan is None or state.timestamp - plan.timestamp > DATA_TIMEOUT:
        return StopReason.MISSING_DATA
    if perception is not None and perception.obstacle_distance is not None \
            and perception.obstacle_distance < MIN_OBSTACLE_DISTANCE:
        return StopReason.OBSTACLE
    if plan.brake_request or plan.desired_speed <= 0.0:
        return StopReason.TARGET_SPEED_ZERO
    return None
