"""Processed TI UART encoding. No serial, simulator, or GUI dependencies."""

import math
import struct
from collections.abc import Iterable, Iterator
from pathlib import Path

from pydantic import Field

from rf_common.frame import RadarFrame
from rf_common.models import RadarPoint, RadarValue, TargetHeight
from rf_common.ti import MAGIC, TITarget

UINT32_MASK = 2**32 - 1
# Row-major range/azimuth/elevation/Doppler covariance: synthetic placeholders.
SYNTHETIC_EC = (0.04, 0, 0, 0, 0, 0.0004, 0, 0, 0, 0, 0.0004, 0, 0, 0, 0, 0.01)


class PointUnits(RadarValue):
    elevation: float = Field(default=0.01, gt=0)
    azimuth: float = Field(default=0.01, gt=0)
    doppler: float = Field(default=0.001, gt=0)
    range: float = Field(default=0.001, gt=0)
    snr: float = Field(default=0.1, gt=0)

    def wire_values(self) -> tuple[float, ...]:
        """Use transmitted float32 scales for quantization, not Python float64."""
        try:
            values = struct.unpack("<5f", struct.pack("<5f", *self.model_dump().values()))
        except (OverflowError, struct.error) as error:
            raise ValueError("point units must fit float32") from error
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("point units must be positive finite float32")
        return values


class SensorPose(RadarValue):
    """Unrotated overhead mount: world (x,y,z) -> sensor (x-x0,z0-z,y-y0)."""

    x_m: float = 1.0
    y_m: float = 1.0
    height_m: float = Field(default=3.0, ge=0)


DEFAULT_UNITS = PointUnits()
DEFAULT_POSE = SensorPose()


class TIOutput(RadarValue):
    """Native output intent; byte counts are computed only by the encoder."""

    version: int = Field(default=0x03050004, ge=0, le=UINT32_MASK)
    platform: int = Field(default=0xA6843, ge=0, le=UINT32_MASK)
    frame_number: int = Field(ge=0, le=UINT32_MASK)
    time_cpu_cycles: int = Field(ge=0, le=UINT32_MASK)
    subframe_number: int = Field(default=0, ge=0, le=UINT32_MASK)
    points: tuple[RadarPoint, ...] = ()
    targets: tuple[TITarget, ...] = ()
    target_indices: tuple[int, ...] | None = None
    heights: tuple[TargetHeight, ...] | None = None
    # TI bytes carry no association frame number. Retain it until serialization.
    association_point_frame_number: int | None = Field(default=None, ge=0, le=UINT32_MASK)


def frame_to_ti(
    frame: RadarFrame,
    *,
    frame_number: int,
    time_cpu_cycles: int,
    sensor_pose: SensorPose = DEFAULT_POSE,
    previous_frame: RadarFrame | None = None,
    previous_frame_number: int | None = None,
    version: int = 0x03050004,
    platform: int = 0xA6843,
) -> TIOutput:
    """Pure common -> native adapter. Supplied indices must name previous points.

    Native target metadata from adapt_packet is authoritative. Otherwise tracks
    are planar synthetic surrogates: zero acceleration, deterministic covariance,
    g=1, confidenceLevel=0 (unavailable), never the common probability score.
    """
    pose_data = frame.source_metadata.get("sensor_pose")
    pose = SensorPose.model_validate(pose_data) if pose_data is not None else sensor_pose
    native = frame.source_metadata.get("ti_targets")
    if native is not None:
        targets = tuple(TITarget.model_validate(t) for t in native)
        if tuple(t.tid for t in targets) != tuple(t.target_id for t in frame.targets):
            raise ValueError("native TI target IDs disagree with common tracks")
    else:
        floor_z = float(frame.source_metadata.get("track_target_z_m", 0.0))
        targets = tuple(
            TITarget(
                tid=t.target_id,
                posX=t.x - pose.x_m,
                posY=pose.height_m - floor_z,
                posZ=t.y - pose.y_m,
                velX=t.vx if t.vx is not None else 0.0,
                velY=0.0,
                velZ=t.vy if t.vy is not None else 0.0,
                accX=0.0,
                accY=0.0,
                accZ=0.0,
                ec=SYNTHETIC_EC,
                g=1.0,
                confidenceLevel=0.0,
            )
            for t in frame.targets
        )
    indices = None
    reference = None
    if (assoc := frame.associations) is not None:
        if previous_frame is None or previous_frame_number is None:
            raise ValueError("TI associations require the previous emitted point frame")
        if assoc.point_frame_id is None and assoc.device_point_frame_number is None:
            raise ValueError("TI association point frame is unknown")
        if assoc.point_frame_id is not None and assoc.point_frame_id != previous_frame.frame_id:
            raise ValueError("TI associations must refer to the previous point frame")
        if (
            assoc.device_point_frame_number is not None
            and assoc.device_point_frame_number != previous_frame_number
        ):
            raise ValueError("TI association device frame reference mismatch")
        if assoc.native_indices is not None:
            indices = assoc.native_indices
            if assoc.track_ids and assoc.track_ids != tuple(
                i if i < 250 else None for i in indices
            ):
                raise ValueError("native indices disagree with normalized track IDs")
        else:
            # None does not distinguish TI's three reason codes: do not guess.
            if any(t is None or not 0 <= t < 250 for t in assoc.track_ids):
                raise ValueError("unassociated entries require explicit TI native reason codes")
            indices = assoc.track_ids
        if len(indices) != len(previous_frame.detections):
            raise ValueError("TI index count must match previous point count")
        reference = previous_frame_number
    heights = frame.target_heights
    if heights is not None and frame.source_metadata.get("height_coordinates") != "native_sensor_Z":
        raise ValueError("TI heights require explicit native_sensor_Z coordinates")
    header = frame.source_metadata.get("ti_header", {})
    return TIOutput(
        version=header.get("version", version),
        platform=header.get("platform", platform),
        frame_number=frame_number,
        time_cpu_cycles=time_cpu_cycles,
        subframe_number=header.get("subFrameNumber", 0),
        points=frame.detections,
        targets=targets,
        target_indices=indices,
        association_point_frame_number=reference,
        heights=heights,
    )


def _quantize(value: float, unit: float, low: int, high: int, field: str) -> int:
    scaled = value / unit
    if not math.isfinite(scaled) or not low <= scaled <= high:
        raise ValueError(f"{field} outside compressed TI range [{low * unit}, {high * unit}]")
    # Python round: nearest integer, ties to even. No silent saturation.
    return round(scaled)


def encode_packet(output: TIOutput, units: PointUnits = DEFAULT_UNITS) -> bytes:
    """Encode only 1020/1010/1011/1012, with payload-only lengths and zero padding."""
    scales = units.wire_values()
    cloud = bytearray(struct.pack("<5f", *scales))
    for p in output.points:
        values = (
            p.elevation_rad,
            p.azimuth_rad,
            p.radial_velocity_mps,
            p.range_m,
            p.signal_quality,
        )
        limits = ((-128, 127), (-128, 127), (-32768, 32767), (0, 65535), (0, 65535))
        names = ("elevation", "azimuth", "doppler", "range", "snr")
        cloud.extend(
            struct.pack(
                "<bbhHH",
                *(
                    _quantize(v, u, lo, hi, name)
                    for v, u, (lo, hi), name in zip(values, scales, limits, names, strict=True)
                ),
            )
        )
    tlvs = [(1020, bytes(cloud))]
    target_payload = bytearray()
    for t in output.targets:
        try:
            record = struct.pack(
                "<I27f",
                t.tid,
                t.posX,
                t.posY,
                t.posZ,
                t.velX,
                t.velY,
                t.velZ,
                t.accX,
                t.accY,
                t.accZ,
                *t.ec,
                t.g,
                t.confidenceLevel,
            )
        except (OverflowError, struct.error) as error:
            raise ValueError("TI target fields must fit float32") from error
        target_payload.extend(record)
    if output.targets:
        tlvs.append((1010, bytes(target_payload)))
    if output.target_indices is not None:
        if output.association_point_frame_number is None:
            raise ValueError("TI index payload requires a previous-frame reference")
        if any(not 0 <= i <= 255 for i in output.target_indices):
            raise ValueError("TI associations must fit unsigned bytes")
        tlvs.append((1011, bytes(output.target_indices)))
    if output.heights:
        heights = bytearray()
        ids = {t.tid for t in output.targets}
        for h in output.heights:
            if h.target_id not in ids or h.maximum_m < h.minimum_m:
                raise ValueError("TI height requires a matching target and ordered native Z bounds")
            try:
                heights.extend(struct.pack("<I2f", h.target_id, h.maximum_m, h.minimum_m))
            except (OverflowError, struct.error) as error:
                raise ValueError("TI height fields must fit native record") from error
        tlvs.append((1012, bytes(heights)))
    payload = b"".join(struct.pack("<II", kind, len(body)) + body for kind, body in tlvs)
    size = (40 + len(payload) + 31) // 32 * 32
    header = MAGIC + struct.pack(
        "<8I",
        output.version,
        size,
        output.platform,
        output.frame_number,
        output.time_cpu_cycles,
        len(output.points),
        len(tlvs),
        output.subframe_number,
    )
    return header + payload + bytes(size - 40 - len(payload))


class TIFrameEncoder:
    """Sequence adapter: uint32 clock/sequence and previous-cloud validation.

    CPU clock defaults to a synthetic 200 MHz. Simulation acquisition time is
    used directly; other sources without device cycles use elapsed host time
    from the first frame. Unix epoch is never encoded as device cycles.
    State advances only after successful encoding.
    """

    def __init__(
        self,
        *,
        units: PointUnits = DEFAULT_UNITS,
        sensor_pose: SensorPose = DEFAULT_POSE,
        cpu_clock_hz: int = 200_000_000,
        start_frame_number: int = 0,
    ) -> None:
        if cpu_clock_hz <= 0 or not 0 <= start_frame_number <= UINT32_MASK:
            raise ValueError("invalid synthetic clock or starting frame number")
        self.units = units
        self.sensor_pose = sensor_pose
        self.cpu_clock_hz = cpu_clock_hz
        self.next_frame_number = start_frame_number
        self.previous_frame: RadarFrame | None = None
        self.previous_number: int | None = None
        self.first_timestamp: float | None = None
        self.previous_elapsed: float | None = None

    def encode(self, frame: RadarFrame) -> bytes:
        number = frame.device_frame_number
        if number is None:
            number = self.next_frame_number
        if not 0 <= number <= UINT32_MASK:
            raise ValueError("TI frame number must fit uint32")
        if self.previous_number is not None:
            delta = (number - self.previous_number) & UINT32_MASK
            if not 0 < delta < 2**31:
                raise ValueError("TI frame numbers must advance modulo uint32")
        origin = frame.timestamp if self.first_timestamp is None else self.first_timestamp
        elapsed = frame.simulation_time
        if elapsed is None:
            elapsed = frame.timestamp - origin
        if elapsed < 0 or (self.previous_elapsed is not None and elapsed < self.previous_elapsed):
            raise ValueError("frame acquisition time moved backwards")
        if frame.device_time_value is not None:
            if frame.device_time_unit != "cpu_cycles" or frame.device_time_value > UINT32_MASK:
                raise ValueError("TI device time must be uint32 cpu_cycles")
            cycles = frame.device_time_value
        else:
            cycles = round(elapsed * self.cpu_clock_hz) & UINT32_MASK
        output = frame_to_ti(
            frame,
            frame_number=number,
            time_cpu_cycles=cycles,
            sensor_pose=self.sensor_pose,
            previous_frame=self.previous_frame,
            previous_frame_number=self.previous_number,
        )
        data = encode_packet(output, self.units)
        self.previous_frame, self.previous_number = frame, number
        self.previous_elapsed, self.first_timestamp = elapsed, origin
        self.next_frame_number = (number + 1) & UINT32_MASK
        return data


def encode_frames(
    frames: Iterable[RadarFrame], encoder: TIFrameEncoder | None = None
) -> Iterator[bytes]:
    """Lazy encoding usable with live, recorded, or test frames."""
    encoder = encoder or TIFrameEncoder()
    for frame in frames:
        yield encoder.encode(frame)


def write_capture(
    path: Path, frames: Iterable[RadarFrame], encoder: TIFrameEncoder | None = None
) -> int:
    """Exclusive creation preserves existing captures. Partial output survives errors."""
    path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with path.open("xb") as stream:
        for packet in encode_frames(frames, encoder):
            stream.write(packet)
            count += 1
    return count
