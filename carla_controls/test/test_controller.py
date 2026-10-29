import pytest

from carla_controls.controller import Controller
from carla_controls.messages import (
    VehicleState,
    PlanningInput,
    PerceptionInput,
    StopReason,
    ControlsOutput,
)


@pytest.fixture
def controller():
    return Controller(dt=0.05)


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


def test_output_type(controller, state, plan, perception):
    """Controller should return a ControlsOutput object."""
    output = controller.step(state, plan, perception)

    assert isinstance(output, ControlsOutput)


def test_normal_acceleration(controller, state, plan, perception):
    """Vehicle below target speed should accelerate."""
    output = controller.step(state, plan, perception)

    assert output.command.throttle > 0.0
    assert output.command.brake == 0.0
    assert output.stop_reason is None


def test_normal_braking(controller, state, plan, perception):
    """Vehicle above target speed should brake."""
    state.speed = 15.0

    output = controller.step(state, plan, perception)

    assert output.command.throttle == 0.0
    assert output.command.brake > 0.0


def test_turn_left(controller, state, plan, perception):
    """Target to the left should produce positive steering."""
    plan.target_x = 10.0
    plan.target_y = 10.0

    output = controller.step(state, plan, perception)

    assert output.command.steer > 0.0


def test_turn_right(controller, state, plan, perception):
    """Target to the right should produce negative steering."""
    plan.target_x = 10.0
    plan.target_y = -10.0

    output = controller.step(state, plan, perception)

    assert output.command.steer < 0.0


def test_emergency_stop(controller, state, plan, perception):
    """Emergency stop should apply full braking."""
    controller.emergency_stop = True

    output = controller.step(state, plan, perception)

    assert output.command.throttle == 0.0
    assert output.command.brake == 1.0
    assert output.stop_reason == StopReason.EMERGENCY_STOP


def test_obstacle_stop(controller, state, plan, perception):
    """Nearby obstacle should trigger full braking."""
    perception.obstacle_distance = 2.0

    output = controller.step(state, plan, perception)

    assert output.command.throttle == 0.0
    assert output.command.brake == 1.0
    assert output.stop_reason == StopReason.OBSTACLE


def test_missing_planning(controller, state, perception):
    """Missing planning data should trigger a safe stop."""
    output = controller.step(state, None, perception)

    assert output.command.throttle == 0.0
    assert output.command.brake == 1.0
    assert output.stop_reason == StopReason.MISSING_DATA


def test_missing_perception(controller, state, plan):
    """Missing perception data should trigger a safe stop."""
    output = controller.step(state, plan, None)

    assert output.command.throttle == 0.0
    assert output.command.brake == 1.0
    assert output.stop_reason == StopReason.MISSING_DATA


def test_stale_planning(controller, state, plan, perception):
    """Outdated planning data should trigger a safe stop."""
    plan.timestamp = 9.0

    output = controller.step(state, plan, perception)

    assert output.command.throttle == 0.0
    assert output.command.brake == 1.0
    assert output.stop_reason == StopReason.MISSING_DATA


def test_zero_target_speed(controller, state, plan, perception):
    """Target speed of zero should trigger full braking."""
    plan.desired_speed = 0.0

    output = controller.step(state, plan, perception)

    assert output.command.throttle == 0.0
    assert output.command.brake == 1.0
    assert output.stop_reason == StopReason.TARGET_SPEED_ZERO


def test_brake_request(controller, state, plan, perception):
    """Planning brake request should trigger full braking."""
    plan.brake_request = True

    output = controller.step(state, plan, perception)

    assert output.command.throttle == 0.0
    assert output.command.brake == 1.0
    assert output.stop_reason == StopReason.TARGET_SPEED_ZERO


def test_throttle_brake_exclusivity(controller, state, plan, perception):
    """Throttle and brake should never be active together."""
    output = controller.step(state, plan, perception)

    assert not (
        output.command.throttle > 0.0
        and output.command.brake > 0.0
    )


def test_command_limits(controller, state, plan, perception):
    """All control outputs should remain within CARLA limits."""
    plan.desired_speed = 100.0
    plan.target_x = 0.0
    plan.target_y = 100.0

    output = controller.step(state, plan, perception)

    assert 0.0 <= output.command.throttle <= 1.0
    assert 0.0 <= output.command.brake <= 1.0
    assert -1.0 <= output.command.steer <= 1.0


def test_stop_resets_controllers(controller, state, plan, perception):
    """Stopping should clear previous speed and steering commands."""
    plan.target_x = 10.0
    plan.target_y = 10.0

    controller.step(state, plan, perception)

    assert controller.speed.prev_throttle > 0.0
    assert controller.steering.prev_steer > 0.0

    controller.emergency_stop = True
    controller.step(state, plan, perception)

    assert controller.speed.prev_throttle == 0.0
    assert controller.steering.prev_steer == 0.0


def test_state_preserved(controller, state, plan, perception):
    """Output should contain the input vehicle state."""
    output = controller.step(state, plan, perception)

    assert output.state == state
