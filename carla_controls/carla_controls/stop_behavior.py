"""Stop behavior: decide whether the car must stop, and why.

Return a StopReason to make controller.py brake, or None when it's OK to drive.

TODO: implement the remaining checks (stale planning data, obstacles,
target speed zero / brake request).
"""
from typing import Optional

from .messages import PerceptionInput, PlanningInput, StopReason, VehicleState


def check_stop(
    state: VehicleState,
    plan: Optional[PlanningInput],
    perception: Optional[PerceptionInput],
    emergency_stop: bool,
) -> Optional[StopReason]:
    if emergency_stop:
        return StopReason.EMERGENCY_STOP
    if plan is None:  # no target yet: speed/steering controllers always get a plan
        return StopReason.MISSING_DATA
    return None  # TODO
