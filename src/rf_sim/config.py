"""Validated configuration with built-in defaults and optional YAML overrides."""

from pathlib import Path
from typing import Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class Settings(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, validate_assignment=True)


class RoomConfig(Settings):
    width_m: float = Field(default=2.0, gt=0)
    height_m: float = Field(default=2.0, gt=0)


class SimulationConfig(Settings):
    physics_rate_hz: float = Field(default=50.0, gt=0)
    gui_rate_hz: int = Field(default=60, gt=0)
    random_seed: int = Field(default=42, ge=0)


class VehicleConfig(Settings):
    wheelbase_m: float = Field(default=0.25, gt=0)
    radius_m: float = Field(default=0.12, gt=0)
    max_forward_speed_mps: float = Field(default=2.0, gt=0)
    max_reverse_speed_mps: float = Field(default=1.0, gt=0)
    max_acceleration_mps2: float = Field(default=2.5, gt=0)
    max_braking_mps2: float = Field(default=3.0, gt=0)
    max_steering_deg: float = Field(default=35.0, gt=0, lt=89)


class TargetConfig(Settings):
    target_id: int = Field(default=1, ge=0, lt=1_000_000)
    name: str = "RC car"
    initial_x: float = 1.0
    initial_y: float = 1.0
    initial_heading: float = 0.0
    vehicle: VehicleConfig | None = None
    reflector_enabled: bool = False
    reflector_gain_db: float = 6.0
    length_m: float = Field(default=0.35, gt=0)
    width_m: float = Field(default=0.20, gt=0)
    height_m: float = Field(default=0.12, ge=0)
    scattering_gain_db: float = 0.0


class RadarConfig(Settings):
    point_cloud_enabled: bool = True
    points_per_object: float = Field(default=12, ge=0, le=1000)
    point_detection_probability: float = Field(default=0.9, ge=0, le=1)
    sensor_height_m: float = Field(default=3.0, ge=0)
    range_noise_std_m: float = Field(default=0.04, ge=0)
    angle_noise_std_rad: float = Field(default=0.01, ge=0)
    doppler_noise_std_mps: float = Field(default=0.05, ge=0)
    snr_reference_db: float = 30.0
    snr_range_loss_db_per_m: float = Field(default=2.0, ge=0)
    measurement_rate_hz: float = Field(default=20.0, gt=0)
    position_noise_std_m: float = Field(default=0.04, ge=0)
    velocity_noise_std_mps: float = Field(default=0.05, ge=0)
    detection_probability: float = Field(default=0.98, ge=0, le=1)
    false_alarm_probability: float = Field(default=0.0, ge=0, le=1)
    latency_ms: float = Field(default=30.0, ge=0)
    minimum_range_m: float = Field(default=0.1, ge=0)
    maximum_range_m: float = Field(default=10.0, gt=0)
    sensor_x_m: float = 1.0
    sensor_y_m: float = 1.0
    legacy_horizontal_range: bool = False
    snr_noise_std_db: float = Field(default=1.0, ge=0)

    @model_validator(mode="after")
    def valid_range(self) -> Self:
        if self.minimum_range_m >= self.maximum_range_m:
            raise ValueError("minimum_range_m must be below maximum_range_m")
        return self


class NetworkConfig(Settings):
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = Field(default=5005, ge=1, le=65535)


class ScenarioConfig(Settings):
    cruise_speed_mps: float = Field(default=0.75, gt=0)
    waypoint_tolerance_m: float = Field(default=0.22, gt=0)
    loop: bool = True
    waypoints: list[tuple[float, float]] | None = None


class AppConfig(Settings):
    room: RoomConfig = Field(default_factory=RoomConfig)
    simulation: SimulationConfig = Field(default_factory=SimulationConfig)
    vehicle: VehicleConfig = Field(default_factory=VehicleConfig)
    radar: RadarConfig = Field(default_factory=RadarConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    scenario: ScenarioConfig = Field(default_factory=ScenarioConfig)
    targets: list[TargetConfig] = Field(
        default_factory=lambda: [
            TargetConfig(name="RC car 1", initial_x=0.5, initial_y=0.5),
            TargetConfig(
                target_id=2,
                name="RC car 2",
                initial_x=1.5,
                initial_y=1.5,
                initial_heading=3.141592653589793,
            ),
        ]
    )

    @model_validator(mode="after")
    def valid_world(self) -> Self:
        ids = [target.target_id for target in self.targets]
        if len(ids) != len(set(ids)) or not ids:
            raise ValueError("targets must be nonempty and have unique IDs")
        if self.radar.measurement_rate_hz > self.simulation.physics_rate_hz:
            raise ValueError("radar rate must not exceed physics rate")
        for target in self.targets:
            radius = (target.vehicle or self.vehicle).radius_m
            if 2 * radius >= min(self.room.width_m, self.room.height_m):
                raise ValueError("vehicle radius does not fit inside room")
            if not (
                radius <= target.initial_x <= self.room.width_m - radius
                and radius <= target.initial_y <= self.room.height_m - radius
            ):
                raise ValueError("target initial position must be inside room with wall clearance")
        if self.scenario.waypoints is not None:
            if not self.scenario.waypoints:
                raise ValueError("custom waypoints must not be empty")
            for x, y in self.scenario.waypoints:
                if not (0 < x < self.room.width_m and 0 < y < self.room.height_m):
                    raise ValueError("waypoints must be inside room")
        return self


def load_config(path: Path | None = None) -> AppConfig:
    """Missing custom sections inherit defaults; unknown settings are errors."""
    if path is None:
        return AppConfig()
    with path.open(encoding="utf-8") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError(f"Invalid YAML in {path}: {error}") from error
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise TypeError("configuration must be a YAML mapping")
    return AppConfig.model_validate(data)
