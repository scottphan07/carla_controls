import pytest

from carla_controls.stop_behavior import check_stop
from carla_controls.messages import (
    VehicleState,
    PlanningInput,
    PerceptionInput,
    StopReason,
)


@pytest.fixture
def state():
    return VehicleState(
        timestamp=10.0,
        x=0.0,
        y=0.0,
        yaw=0.0,
        speed=5.0,
    )


@pytest.fixture
def plan():
    return PlanningInput(
        timestamp=10.0,
        desired_speed=10.0,
        target_x=50.0,
        target_y=0.0,
    )


@pytest.fixture
def perception():
    return PerceptionInput(
        timestamp=10.0,
        obstacle_distance=20.0,
    )


def test_normal_driving(state, plan, perception):
    """Valid inputs with no hazard should allow driving."""
    reason = check_stop(state, plan, perception, False)

    assert reason is None


def test_emergency_stop(state, plan, perception):
    """Emergency stop must override normal driving."""
    reason = check_stop(state, plan, perception, True)

    assert reason == StopReason.EMERGENCY_STOP


def test_missing_planning(state, perception):
    """Missing planning data should trigger a safe stop."""
    reason = check_stop(state, None, perception, False)

    assert reason == StopReason.MISSING_DATA


def test_missing_perception(state, plan):
    """Missing perception data should trigger a safe stop."""
    reason = check_stop(state, plan, None, False)

    assert reason == StopReason.MISSING_DATA


def test_stale_planning(state, plan, perception):
    """Planning data older than the timeout should trigger a stop."""
    plan.timestamp = 9.0

    reason = check_stop(state, plan, perception, False)

    assert reason == StopReason.MISSING_DATA


def test_stale_perception(state, plan, perception):
    """Perception data older than the timeout should trigger a stop."""
    perception.timestamp = 9.0

    reason = check_stop(state, plan, perception, False)

    assert reason == StopReason.MISSING_DATA


def test_obstacle_detected(state, plan, perception):
    """An obstacle inside the stop threshold should trigger braking."""
    perception.obstacle_distance = 2.0

    reason = check_stop(state, plan, perception, False)

    assert reason == StopReason.OBSTACLE


def test_obstacle_far_away(state, plan, perception):
    """A distant obstacle should not trigger an immediate stop."""
    perception.obstacle_distance = 20.0

    reason = check_stop(state, plan, perception, False)

    assert reason is None


def test_zero_target_speed(state, plan, perception):
    """A target speed of zero should trigger a stop."""
    plan.desired_speed = 0.0

    reason = check_stop(state, plan, perception, False)

    assert reason == StopReason.TARGET_SPEED_ZERO


def test_brake_request(state, plan, perception):
    """An explicit planning brake request should trigger a stop."""
    plan.brake_request = True

    reason = check_stop(state, plan, perception, False)

    assert reason == StopReason.TARGET_SPEED_ZERO


def test_emergency_stop_priority(state, plan, perception):
    """Emergency stop should take priority over an obstacle."""
    perception.obstacle_distance = 1.0

    reason = check_stop(state, plan, perception, True)

    assert reason == StopReason.EMERGENCY_STOP


def test_obstacle_priority_over_zero_speed(state, plan, perception):
    """Obstacle detection should take priority over zero target speed."""
    perception.obstacle_distance = 1.0
    plan.desired_speed = 0.0

    reason = check_stop(state, plan, perception, False)

    assert reason == StopReason.OBSTACLE
