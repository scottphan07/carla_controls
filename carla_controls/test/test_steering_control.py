import math
import pytest

from carla_controls.steering_control import SteeringController
from carla_controls.messages import VehicleState, PlanningInput


@pytest.fixture
def controller():
    return SteeringController(dt=0.05)


def make_state(yaw=0.0):
    return VehicleState(
        timestamp=1.0,
        x=0.0,
        y=0.0,
        yaw=yaw,
        speed=5.0
    )


def make_plan(x=50.0, y=0.0):
    return PlanningInput(
        timestamp=1.0,
        desired_speed=10.0,
        target_x=x,
        target_y=y
    )


def test_straight_ahead(controller):
    """No steering when the target is directly ahead."""
    steer = controller.compute(
        make_state(yaw=0.0),
        make_plan(x=50.0, y=0.0)
    )

    assert steer == pytest.approx(0.0)


def test_turn_left(controller):
    """Target to the left should produce positive steering."""
    steer = controller.compute(
        make_state(),
        make_plan(x=10.0, y=10.0)
    )

    assert steer > 0.0


def test_turn_right(controller):
    """Target to the right should produce negative steering."""
    steer = controller.compute(
        make_state(),
        make_plan(x=10.0, y=-10.0)
    )

    assert steer < 0.0


def test_steering_limits(controller):
    """Steering must remain between -1 and 1."""
    targets = [
        (0.0, 50.0),
        (0.0, -50.0),
        (-50.0, 0.0),
        (50.0, 0.0)
    ]

    for x, y in targets:
        steer = controller.compute(
            make_state(),
            make_plan(x=x, y=y)
        )

        assert -1.0 <= steer <= 1.0


def test_angle_wrapping(controller):
    """Heading error should use the shortest angular difference."""
    state = make_state(yaw=math.radians(179))
    plan = make_plan(
        x=math.cos(math.radians(-179)),
        y=math.sin(math.radians(-179))
    )

    steer = controller.compute(state, plan)

    assert steer > 0.0


def test_steering_rate_limit(controller):
    """Steering should not change too quickly between ticks."""
    first = controller.compute(
        make_state(),
        make_plan(x=10.0, y=10.0)
    )

    second = controller.compute(
        make_state(),
        make_plan(x=10.0, y=-10.0)
    )

    max_change = controller.max_steer_rate * controller.dt

    assert abs(second - first) <= max_change + 1e-9


def test_reset(controller):
    """Reset should clear previous steering state."""
    controller.compute(
        make_state(),
        make_plan(x=10.0, y=10.0)
    )

    controller.reset()

    assert controller.prev_steer == 0.0
