import math

import pytest

from rf_sim.config import RadarConfig, RoomConfig, TargetConfig, VehicleConfig
from rf_sim.motion import Vehicle
from rf_sim.radar import RadarSensor, SimulatedRadarSource


def vehicle():
    return Vehicle(TargetConfig(initial_x=1.5, initial_y=1.5), VehicleConfig(), RoomConfig())


def sensor(**kwargs):
    values = {
        "position_noise_std_m": 0,
        "velocity_noise_std_mps": 0,
        "snr_noise_std_db": 0,
        "detection_probability": 1,
    }
    return RadarSensor(RadarConfig(**(values | kwargs)), RoomConfig(), 42)


def test_zero_noise_geometry_and_velocity():
    car = vehicle()
    car.state.speed = 1
    measured = sensor().measure([car])[0]
    assert (measured.x, measured.y) == (1.5, 1.5)
    assert (measured.vx, measured.vy, measured.speed) == (1, 0, 1)
    assert measured.range_m == pytest.approx(math.sqrt(9.5))
    assert measured.horizontal_range_m == pytest.approx(math.sqrt(0.5))
    assert measured.slant_range_m == measured.range_m
    assert measured.azimuth_rad == pytest.approx(math.atan2(0.5, 0.5))


def test_height_and_legacy_range_are_independent_of_floor_geometry():
    car = vehicle()
    low = sensor(sensor_height_m=2).measure([car])[0]
    high = sensor(sensor_height_m=4).measure([car])[0]
    assert (low.x, low.y) == (high.x, high.y) == (1.5, 1.5)
    assert low.horizontal_range_m == high.horizontal_range_m
    assert low.slant_range_m == pytest.approx(math.sqrt(4.5))
    assert high.slant_range_m == pytest.approx(math.sqrt(16.5))
    legacy = sensor(legacy_horizontal_range=True).measure([car])[0]
    assert legacy.range_m == legacy.horizontal_range_m
    assert legacy.slant_range_m == pytest.approx(math.sqrt(9.5))
    car.state.x = car.state.y = 1
    assert sensor().measure([car])[0].range_m == 3
    assert sensor(maximum_range_m=2.9).measure([car]) == ()


def test_detection_and_range():
    for _ in range(50):
        assert sensor(detection_probability=0).measure([vehicle()]) == ()
        assert len(sensor().measure([vehicle()])) == 1
    assert sensor(maximum_range_m=1).measure([vehicle()]) == ()


def test_seeded_sequence_and_confidence():
    config = RadarConfig(false_alarm_probability=0.3)
    left = RadarSensor(config, RoomConfig(), 42)
    right = RadarSensor(config, RoomConfig(), 42)
    for _ in range(100):
        a, b = left.measure([vehicle()]), right.measure([vehicle()])
        assert a == b
        assert all(0 <= target.confidence <= 1 for target in a)


def test_false_alarm_and_multiple_targets():
    source = sensor(false_alarm_probability=1)
    second = Vehicle(TargetConfig(target_id=2), VehicleConfig(), RoomConfig())
    targets = source.measure([vehicle(), second])
    assert len(targets) == 3
    assert {t.target_id for t in targets} == {1, 2, 1_000_000}
    assert all("reflector_enabled" not in t.model_dump() for t in targets)


def test_reflector_gain():
    car = vehicle()
    baseline = sensor().measure([car])[0]
    car.target.reflector_enabled = True
    boosted = sensor().measure([car])[0]
    assert boosted.snr_db - baseline.snr_db == pytest.approx(6)


def test_latency_keeps_acquisition_snapshot():
    car = vehicle()
    source = SimulatedRadarSource(sensor(latency_ms=50), epoch=100)
    source.acquire([car], 4)
    car.state.x = 1.7
    assert source.poll(4.049) == []
    frame = source.poll(4.05)[0]
    assert frame.targets[0].x == 1.5
    assert frame.simulation_time == 4
    assert frame.timestamp == 104
    assert source.poll(5) == []
