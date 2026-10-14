from rf_sim.config import AppConfig
from rf_sim.controllers import ManualController
from rf_sim.simulator import Simulator


def test_manual_preset_scales_with_dimensions_and_radius():
    config = AppConfig(room={"width_m": 4, "height_m": 3})
    sim = Simulator(config, "two_targets_manual", epoch=0)
    assert [(v.state.x, v.state.y) for v in sim.vehicles] == [(1, 0.75), (3, 2.25)]
    config = AppConfig(vehicle={"radius_m": 0.6}, targets=[{"initial_x": 1, "initial_y": 1}])
    sim = Simulator(config, "two_targets_manual", epoch=0)
    assert [(v.state.x, v.state.y) for v in sim.vehicles] == [(0.6, 0.6), (1.4, 1.4)]


def test_repeatable_core_and_separate_rates():
    config = AppConfig()
    a = Simulator(config, "rectangle", epoch=100)
    b = Simulator(config, "rectangle", epoch=100)
    frames = []
    for _ in range(500):
        left, right = a.step(), b.step()
        assert left == right
        frames.extend(left)
    assert a.simulation_time == 10
    assert 198 <= len(frames) <= 201
    assert all(a.epoch + f.simulation_time == f.timestamp for f in frames)
    assert set(frames[0].model_dump()) == {
        "device_frame_number",
        "device_time_value",
        "device_time_unit",
        "detections",
        "associations",
        "presence",
        "target_heights",
        "diagnostics",
        "source_metadata",
        "schema_version",
        "source",
        "frame_id",
        "timestamp",
        "simulation_time",
        "targets",
    }


def test_multiple_targets_and_reset_timeline():
    config = AppConfig.model_validate(
        {
            "targets": [{"target_id": 1}, {"target_id": 2, "initial_x": 1.5}],
            "radar": {"detection_probability": 1, "latency_ms": 0},
        }
    )
    sim = Simulator(config, "stationary", epoch=0)
    frame = sim.step()[0]
    assert len(frame.targets) == 2
    sim.reset_vehicles()
    assert sim.tick == 1


def test_two_targets_preset_motion_measurements_and_reset():
    config = AppConfig(radar={"detection_probability": 1, "latency_ms": 0})
    sim = Simulator(config, "two_targets", epoch=0)
    starts = [(v.state.x, v.state.y) for v in sim.vehicles]
    assert starts == [(1 / 3, 0.5), (5 / 3, 1.5)]
    frames = []
    for _ in range(100):
        frames.extend(sim.step())
    assert all({t.target_id for t in frame.targets} == {1, 2} for frame in frames)
    assert all(
        (v.state.x, v.state.y) != start for v, start in zip(sim.vehicles, starts, strict=True)
    )
    sim.reset_vehicles()
    assert [(v.state.x, v.state.y) for v in sim.vehicles] == starts
    assert len(config.targets) == 2


def test_two_targets_manual_preset_and_radar():
    config = AppConfig(radar={"detection_probability": 1, "latency_ms": 0, "snr_noise_std_db": 0})
    sim = Simulator(config, "two_targets_manual", epoch=0)
    assert len(sim.vehicles) == 2
    assert [v.state.target_id for v in sim.vehicles] == [1, 2]
    assert [v.target.reflector_enabled for v in sim.vehicles] == [True, False]
    assert [(v.state.x, v.state.y) for v in sim.vehicles] == [(0.5, 0.5), (1.5, 1.5)]
    assert (sim.config.room.width_m, sim.config.room.height_m) == (2, 2)
    assert sim.source.sensor.room == sim.config.room
    assert (sim.config.radar.sensor_x_m, sim.config.radar.sensor_y_m) == (1, 1)
    assert sim.config.radar.sensor_height_m == 3
    for v in sim.vehicles:
        assert v.parameters.radius_m <= v.state.x <= 2 - v.parameters.radius_m
        assert v.parameters.radius_m <= v.state.y <= 2 - v.parameters.radius_m
    assert all(v.parameters == config.vehicle for v in sim.vehicles)
    assert all(isinstance(c, ManualController) for c in sim.controllers)
    assert sim.controllers[0] is not sim.controllers[1]
    frame = sim.step()[0]
    assert {t.target_id for t in frame.targets} == {1, 2}
    for measurement, vehicle in zip(frame.targets, sim.vehicles, strict=True):
        gain = vehicle.target.reflector_gain_db if vehicle.target.reflector_enabled else 0
        assert measurement.snr_db == 30 - 2 * measurement.range_m + gain
    assert len(config.targets) == 2
