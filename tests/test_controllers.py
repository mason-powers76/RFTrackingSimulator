from rf_sim.config import AppConfig, RoomConfig, TargetConfig, VehicleConfig
from rf_sim.controllers import WaypointController
from rf_sim.motion import CarState, Vehicle
from rf_sim.scenarios import make_controller


def test_switch_and_finish():
    controller = WaypointController([(1, 1), (2, 1)], loop=False)
    state = CarState(1, 1, 1)
    controller.command(state, VehicleConfig())
    assert controller.index == 1
    state.x = 2
    assert controller.command(state, VehicleConfig()).emergency_stop


def test_rectangle_completes_lap_without_wall_collision():
    # Use a larger explicit room to isolate waypoint steering from tight-arena walls.
    config = AppConfig(room=RoomConfig(width_m=6, height_m=4))
    controller = make_controller("rectangle", config)
    vehicle = Vehicle(TargetConfig(), config.vehicle, config.room)
    visited = set()
    for _ in range(3000):
        vehicle.step(controller.command(vehicle.state, vehicle.parameters), 0.02)
        visited.add(controller.index)
        assert (
            config.vehicle.radius_m
            < vehicle.state.x
            < config.room.width_m - config.vehicle.radius_m
        )
        assert (
            config.vehicle.radius_m
            < vehicle.state.y
            < config.room.height_m - config.vehicle.radius_m
        )
    assert visited == {0, 1, 2, 3}


def test_all_scripted_scenarios_run():
    config = AppConfig()
    for name in ["straight_line", "circle", "figure_eight", "stationary"]:
        controller = make_controller(name, config)
        vehicle = Vehicle(TargetConfig(), VehicleConfig(), RoomConfig())
        for _ in range(1000):
            vehicle.step(controller.command(vehicle.state, vehicle.parameters), 0.02)
        assert 0.12 <= vehicle.state.x <= 1.88
        assert 0.12 <= vehicle.state.y <= 1.88
