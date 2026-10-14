"""SI units; floor tracks and spherical sensor detections are distinct."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class RadarValue(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True)


class RadarPoint(RadarValue):
    range_m: float = Field(ge=0)
    azimuth_rad: float
    elevation_rad: float
    radial_velocity_mps: float
    signal_quality: float
    signal_quality_kind: Literal["synthetic_db", "ti_native_snr"]


class PointAssociations(RadarValue):
    # Explicit frame reference: TI indices belong to the PREVIOUS device frame.
    point_frame_id: int | None = Field(default=None, ge=0)
    device_point_frame_number: int | None = Field(default=None, ge=0)
    track_ids: tuple[int | None, ...] = ()
    native_indices: tuple[int, ...] | None = None


class TargetHeight(RadarValue):
    target_id: int = Field(ge=0)
    maximum_m: float
    minimum_m: float


class FrameDetails(RadarValue):
    device_frame_number: int | None = Field(default=None, ge=0)
    device_time_value: int | None = Field(default=None, ge=0)
    device_time_unit: str | None = None
    detections: tuple[RadarPoint, ...] = ()
    associations: PointAssociations | None = None
    presence: bool | None = None
    target_heights: tuple[TargetHeight, ...] | None = None
    diagnostics: dict[str, str | int | float | bool] = Field(default_factory=dict)
    source_metadata: dict = Field(default_factory=dict)
