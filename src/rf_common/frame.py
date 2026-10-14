"""Backend contract: room coordinates, metres, seconds, and radians."""

from typing import Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

from rf_common.models import FrameDetails


class Detection(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True, strict=True)
    target_id: int = Field(ge=0, le=2**63 - 1)
    x: float
    y: float
    vx: float | None = None
    vy: float | None = None
    speed: float | None = Field(default=None, ge=0)
    heading_rad: float | None = None
    range_m: float | None = Field(default=None, ge=0)
    horizontal_range_m: float | None = Field(default=None, ge=0)
    slant_range_m: float | None = Field(default=None, ge=0)
    azimuth_rad: float | None = None
    confidence: float | None = Field(default=None, ge=0, le=1)
    snr_db: float | None = None


class RadarFrame(FrameDetails):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True, strict=True)
    source: str = Field(min_length=1, max_length=128)
    frame_id: int = Field(ge=0, le=2**63 - 1)
    timestamp: float = Field(ge=0)
    simulation_time: float | None = Field(default=None, ge=0)
    targets: tuple[Detection, ...] = ()

    @model_validator(mode="after")
    def unique_targets(self) -> Self:
        ids = [target.target_id for target in self.targets]
        if len(ids) != len(set(ids)):
            raise ValueError("target IDs must be unique within a frame")
        return self
