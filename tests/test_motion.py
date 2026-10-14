import math

import pytest

from rf_sim.config import RoomConfig, TargetConfig, VehicleConfig
from rf_sim.motion import Control, Vehicle


def car():
    return Vehicle(TargetConfig(initial_x=0.5, initial_y=0.5), VehicleConfig(), RoomConfig())


def test_stationary():
    vehicle = car()
    for _ in range(100):
        vehicle.step(Control(), 0.02)
    assert (vehicle.state.x, vehicle.state.y, vehicle.state.speed) == (0.5, 0.5, 0)


def test_straight_motion():
    vehicle = car()
    vehicle.state.speed = 1
    for _ in range(50):
        vehicle.step(Control(), 0.02)
    assert vehicle.state.x == pytest.approx(1.5)
    assert vehicle.state.y == 0.5


def test_steering():
    vehicle = car()
    vehicle.state.speed = 1
    vehicle.step(Control(steering_angle=0.2), 0.02)
    assert vehicle.state.heading == pytest.approx(1 / 0.25 * math.tan(0.2) * 0.02)


def test_limits_and_wall():
    vehicle = car()
    for _ in range(40):
        vehicle.step(Control(acceleration=100, steering_angle=100), 0.02)
    assert vehicle.state.speed <= 2
    assert vehicle.state.acceleration <= 2.5
    assert vehicle.state.steering_angle == pytest.approx(math.radians(35))
    vehicle.state.x = 1.87
    vehicle.state.heading = 0
    vehicle.state.speed = 2
    vehicle.step(Control(), 0.02)
    assert vehicle.state.x == 1.88
    assert vehicle.state.speed == 0


def test_reverse_limits_and_stop():
    vehicle = car()
    vehicle.state.x = 1.5
    for _ in range(40):
        vehicle.step(Control(acceleration=-100), 0.02)
    assert vehicle.state.speed == -1
    vehicle.step(Control(emergency_stop=True), 0.02)
    assert vehicle.state.speed == 0


@pytest.mark.parametrize(
    "heading,axis,limit",
    [
        (0, "x", 1.88),
        (math.pi, "x", 0.12),
        (math.pi / 2, "y", 1.88),
        (-math.pi / 2, "y", 0.12),
    ],
)
def test_default_arena_clamps_all_walls_without_wrapping(heading, axis, limit):
    vehicle = car()
    vehicle.state.heading = heading
    for _ in range(500):
        vehicle.step(Control(acceleration=2.5), 0.02)
        assert 0.12 <= vehicle.state.x <= 1.88
        assert 0.12 <= vehicle.state.y <= 1.88
    assert getattr(vehicle.state, axis) == pytest.approx(limit)
