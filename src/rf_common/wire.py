"""Normalized measurement contract; never contains simulator ground truth."""

from typing import Literal

from pydantic import ConfigDict, Field, model_validator

from rf_common.frame import Detection, RadarFrame


class RadarTargetMeasurement(Detection):
    """Floor XY track; range_m is slant unless legacy mode is explicit.

    azimuth_rad retains the legacy floor bearing from +x, not TI azimuth.
    Optional explicit ranges allow old recorded frames to remain readable.
    """

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True, strict=False)
    target_id: int = Field(ge=0)
    x: float
    y: float
    vx: float
    vy: float
    speed: float = Field(ge=0)
    heading_rad: float | None = None
    range_m: float | None = Field(default=None, ge=0)
    azimuth_rad: float | None = None
    confidence: float | None = Field(default=None, ge=0, le=1)
    snr_db: float | None = None


class TargetFrame(RadarFrame):
    """Acquisition-time frame shared by simulated and future hardware sources."""

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True, strict=False)
    schema_version: Literal["1.0"] = "1.0"
    source: str = "rf_simulator"
    frame_id: int = Field(ge=0)
    timestamp: float = Field(ge=0)
    simulation_time: float | None = Field(default=None, ge=0)
    targets: tuple[RadarTargetMeasurement, ...] = ()

    @model_validator(mode="after")
    def synthetic_time(self):
        if self.source == "rf_simulator" and self.simulation_time is None:
            raise ValueError("synthetic frames require simulation_time")
        return self
