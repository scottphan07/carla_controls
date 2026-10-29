import pytest

from carla_controls.speed_control import SpeedController
from carla_controls.messages import VehicleState, PlanningInput


@pytest.fixture
def controller():
    return SpeedController(dt=0.05)


def make_state(speed):
    return VehicleState(
        timestamp=1.0,
        x=0.0,
        y=0.0,
        yaw=0.0,
        speed=speed
    )


def make_plan(desired_speed):
    return PlanningInput(
        timestamp=1.0,
        desired_speed=desired_speed,
        target_x=50.0,
        target_y=0.0
    )


def test_acceleration(controller):
    """Vehicle should accelerate when below target speed."""
    state = make_state(speed=5.0)
    plan = make_plan(desired_speed=10.0)

    throttle, brake = controller.compute(state, plan)

    assert throttle > 0.0
    assert brake == 0.0


def test_braking(controller):
    """Vehicle should brake when above target speed."""
    state = make_state(speed=10.0)
    plan = make_plan(desired_speed=5.0)

    throttle, brake = controller.compute(state, plan)

    assert throttle == 0.0
    assert brake > 0.0


def test_deadband(controller):
    """Small speed errors should produce no correction."""
    state = make_state(speed=9.95)
    plan = make_plan(desired_speed=10.0)

    throttle, brake = controller.compute(state, plan)

    assert throttle == 0.0
    assert brake == 0.0


def test_throttle_limits(controller):
    """Throttle must stay between 0 and 1."""
    state = make_state(speed=0.0)
    plan = make_plan(desired_speed=100.0)

    throttle, brake = controller.compute(state, plan)

    assert 0.0 <= throttle <= 1.0
    assert brake == 0.0


def test_brake_limits(controller):
    """Brake must stay between 0 and 1."""
    state = make_state(speed=100.0)
    plan = make_plan(desired_speed=0.0)

    throttle, brake = controller.compute(state, plan)

    assert throttle == 0.0
    assert 0.0 <= brake <= 1.0


def test_equal_speeds(controller):
    """No correction when current speed equals target."""
    state = make_state(speed=10.0)
    plan = make_plan(desired_speed=10.0)

    throttle, brake = controller.compute(state, plan)

    assert throttle == 0.0
    assert brake == 0.0
