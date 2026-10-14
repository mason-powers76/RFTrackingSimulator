from pathlib import Path

import pytest
from pydantic import ValidationError

from rf_sim.config import AppConfig, load_config
from rf_sim.models import RadarTargetMeasurement, TargetFrame


def test_python_and_checked_in_yaml_defaults_agree():
    config = load_config()
    assert load_config(Path(__file__).resolve().parents[1] / "config/default.yaml") == config
    assert (config.room.width_m, config.room.height_m) == (2.0, 2.0)
    assert (config.radar.sensor_x_m, config.radar.sensor_y_m, config.radar.sensor_height_m) == (
        1,
        1,
        3,
    )
    with pytest.raises(ValidationError, match="wall clearance"):
        AppConfig(targets=[{"initial_x": 1.9}])


def test_frame_round_trip():
    frame = TargetFrame(frame_id=1, timestamp=1, simulation_time=0)
    assert TargetFrame.model_validate_json(frame.model_dump_json()) == frame
    assert "ground_truth" not in frame.model_dump()


def test_confidence_and_extra_fields_rejected():
    values = {
        "target_id": 1,
        "x": 0,
        "y": 0,
        "vx": 0,
        "vy": 0,
        "speed": 0,
        "range_m": 1,
        "azimuth_rad": 0,
        "confidence": 1,
        "snr_db": 20,
    }
    for confidence in [-0.1, 1.1, float("nan")]:
        with pytest.raises(ValidationError):
            RadarTargetMeasurement(**(values | {"confidence": confidence}))
    with pytest.raises(ValidationError):
        TargetFrame(frame_id=0, timestamp=1, simulation_time=0, ground_truth=[])


def test_yaml_defaults(tmp_path):
    path = tmp_path / "settings.yaml"
    path.write_text("radar:\n  position_noise_std_m: 0.2\n", encoding="utf-8")
    config = load_config(path)
    assert config.radar.position_noise_std_m == 0.2
    assert config.vehicle.wheelbase_m == 0.25
    assert load_config().room.width_m == 2
    with pytest.raises(ValidationError):
        AppConfig.model_validate({"targets": [{"target_id": 1}, {"target_id": 1}]})
