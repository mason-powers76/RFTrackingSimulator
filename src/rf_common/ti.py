"""IWR6843 overhead demo packet decoding; no serial-port dependency.

Reference: Radar Toolbox 4.00.00.05 Applications Visualizer and demo firmware.
Native covariance is range/azimuth/elevation/Doppler, never XYZ covariance.
"""

import math
import struct

from pydantic import Field

from rf_common.frame import Detection, RadarFrame
from rf_common.models import PointAssociations, RadarPoint, RadarValue, TargetHeight

MAGIC = bytes.fromhex("02 01 04 03 06 05 08 07")


class TITarget(RadarValue):
    tid: int = Field(ge=0, le=2**32 - 1)
    posX: float
    posY: float
    posZ: float
    velX: float
    velY: float
    velZ: float
    accX: float
    accY: float
    accZ: float
    ec: tuple[float, ...] = Field(min_length=16, max_length=16)
    g: float
    confidenceLevel: float


class TIHeader(RadarValue):
    version: int = Field(ge=0)
    totalPacketLen: int = Field(ge=40)
    platform: int = Field(ge=0)
    frameNumber: int = Field(ge=0)
    timeCPUCycles: int = Field(ge=0)
    numDetectedObj: int = Field(ge=0)
    numTLVs: int = Field(ge=0)
    subFrameNumber: int = Field(ge=0)


class TIPacket(RadarValue):
    header: TIHeader
    targets: tuple[TITarget, ...] = ()
    points: tuple[RadarPoint, ...] = ()
    target_indices: tuple[int, ...] | None = None
    presence: bool | None = None
    heights: tuple[TargetHeight, ...] | None = None
    unknown_tlv_types: tuple[int, ...] = ()


def decode_packet(data: bytes) -> TIPacket:
    """Decode a complete little-endian packet; TLV length excludes its header.

    Firmware/Visualizer use unsigned range/SNR and uint32 height IDs despite
    conflicting signedness/height-ID descriptions in the HTML user guide.
    """
    if len(data) < 40 or data[:8] != MAGIC:
        raise ValueError("invalid TI frame header")
    header = TIHeader(**dict(zip(TIHeader.model_fields, struct.unpack_from("<8I", data, 8))))
    if len(data) != header.totalPacketLen:
        raise ValueError("TI packet length mismatch")
    cursor, targets, points, unknown = 40, [], [], []
    indices = presence = heights = None
    seen = set()
    for _ in range(header.numTLVs):
        if cursor + 8 > len(data):
            raise ValueError("truncated TI TLV header")
        kind, length = struct.unpack_from("<2I", data, cursor)
        cursor += 8
        if cursor + length > len(data):
            raise ValueError("truncated TI TLV payload")
        payload = data[cursor : cursor + length]
        cursor += length
        if kind in seen:
            raise ValueError("duplicate TI TLV")
        seen.add(kind)
        if kind == 1010:
            if length % 112:
                raise ValueError("invalid TI target list size")
            for values in struct.iter_unpack("<I27f", payload):
                fields = dict(zip(list(TITarget.model_fields)[:10], values[:10]))
                targets.append(
                    TITarget(**fields, ec=values[10:26], g=values[26], confidenceLevel=values[27])
                )
        elif kind == 1020:
            if length < 20 or (length - 20) % 8:
                raise ValueError("invalid TI point cloud size")
            units = struct.unpack_from("<5f", payload)
            if any(not math.isfinite(unit) or unit <= 0 for unit in units):
                raise ValueError("invalid TI point units")
            for elev, az, doppler, distance, snr in struct.iter_unpack("<bbhHH", payload[20:]):
                points.append(
                    RadarPoint(
                        elevation_rad=elev * units[0],
                        azimuth_rad=az * units[1],
                        radial_velocity_mps=doppler * units[2],
                        range_m=distance * units[3],
                        signal_quality=snr * units[4],
                        signal_quality_kind="ti_native_snr",
                    )
                )
        elif kind == 1011:
            indices = tuple(payload)
        elif kind == 1021:
            if length != 4 or int.from_bytes(payload, "little") not in (0, 1):
                raise ValueError("invalid TI presence payload")
            presence = bool(int.from_bytes(payload, "little"))
        elif kind == 1012:
            if length % 12:
                raise ValueError("invalid TI height payload")
            heights = tuple(
                TargetHeight(target_id=tid, maximum_m=high, minimum_m=low)
                for tid, high, low in struct.iter_unpack("<I2f", payload)
            )
        else:
            unknown.append(kind)
    if len(points) != header.numDetectedObj:
        raise ValueError("TI detected point count mismatch")
    if len(data) - cursor >= 32:
        raise ValueError("excess TI frame padding")
    return TIPacket(
        header=header,
        targets=tuple(targets),
        points=tuple(points),
        target_indices=indices,
        presence=presence,
        heights=heights,
        unknown_tlv_types=tuple(unknown),
    )


def adapt_packet(
    packet: TIPacket,
    *,
    frame_id: int,
    received_at: float,
    floor_origin: tuple[float, float] = (0, 0),
    previous_device_frame_number: int | None = None,
) -> RadarFrame:
    """Unrotated overhead mount: sensor X/Z -> floor x/y, origin configurable.

    The caller supplies the actual previous device frame number; do not infer
    n-1 across packet loss, rollover or restart. Heights retain native Z values.
    """
    tracks = []
    for target in packet.targets:
        speed = math.hypot(target.velX, target.velZ)
        tracks.append(
            Detection(
                target_id=target.tid,
                x=target.posX + floor_origin[0],
                y=target.posZ + floor_origin[1],
                vx=target.velX,
                vy=target.velZ,
                speed=speed,
                heading_rad=math.atan2(target.velZ, target.velX) if speed > 1e-9 else None,
            )
        )
    associations = None
    if packet.target_indices is not None:
        associations = PointAssociations(
            device_point_frame_number=previous_device_frame_number,
            native_indices=packet.target_indices,
            track_ids=tuple(i if i < 250 else None for i in packet.target_indices),
        )
    return RadarFrame(
        source="ti_iwr6843isk_ods",
        frame_id=frame_id,
        timestamp=received_at,
        targets=tuple(tracks),
        detections=packet.points,
        associations=associations,
        presence=packet.presence,
        target_heights=packet.heights,
        device_frame_number=packet.header.frameNumber,
        device_time_value=packet.header.timeCPUCycles,
        device_time_unit="cpu_cycles",
        diagnostics={"unknown_tlv_count": len(packet.unknown_tlv_types)},
        source_metadata={
            "ti_header": packet.header.model_dump(),
            "ti_targets": [t.model_dump(mode="json") for t in packet.targets],
            "unknown_tlv_types": list(packet.unknown_tlv_types),
            "floor_transform": {"axes": "X,Z", "origin": list(floor_origin)},
            "height_coordinates": "native_sensor_Z",
        },
    )
