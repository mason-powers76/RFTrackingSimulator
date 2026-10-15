import json
import math
import os
import random
import struct
import subprocess
import sys
from itertools import islice
from pathlib import Path

import pytest

from rf_common.frame import Detection, RadarFrame
from rf_common.models import PointAssociations, RadarPoint, TargetHeight
from rf_common.ti import MAGIC, TITarget, adapt_packet, decode_packet
from rf_common.ti_encoder import (
    SYNTHETIC_EC,
    PointUnits,
    SensorPose,
    TIFrameEncoder,
    TIOutput,
    encode_packet,
    frame_to_ti,
    write_capture,
)
from rf_common.ti_stream import TIStreamDecoder
from rf_sim.config import AppConfig
from rf_sim.ti_capture import recorded_frames, simulated_frames

ROOT = Path(__file__).resolve().parents[1]


def point(**overrides):
    return RadarPoint(
        **{
            "range_m": 3.14159,
            "azimuth_rad": -0.234,
            "elevation_rad": 0.125,
            "radial_velocity_mps": -0.2718,
            "signal_quality": 24.36,
            "signal_quality_kind": "synthetic_db",
        }
        | overrides
    )


def frame(fid=10, points=(), tracks=()):
    return RadarFrame(
        source="rf_simulator",
        frame_id=fid,
        timestamp=1700000000.0 + fid / 20,
        simulation_time=fid / 20,
        detections=tuple(points),
        targets=tuple(tracks),
    )


def track(tid=7):
    return Detection(target_id=tid, x=0.5, y=1.5, vx=1.2, vy=-0.4, confidence=0.9)


def complete_frames():
    native = TITarget(
        tid=7,
        posX=-0.4,
        posY=2.9,
        posZ=0.2,
        velX=1,
        velY=2,
        velZ=3,
        accX=4,
        accY=5,
        accZ=6,
        ec=tuple(i / 10 for i in range(16)),
        g=2.5,
        confidenceLevel=42,
    )
    previous = frame(10, [point()] * 4, [track()])
    current = frame(12, [point(), point(azimuth_rad=0.215, elevation_rad=-0.147)], [track()])
    current = current.model_copy(
        update={
            "associations": PointAssociations(
                point_frame_id=10,
                native_indices=(7, 253, 254, 255),
                track_ids=(7, None, None, None),
            ),
            "target_heights": (TargetHeight(target_id=7, maximum_m=0.35, minimum_m=0.05),),
            "source_metadata": {
                "ti_targets": [native.model_dump()],
                "height_coordinates": "native_sensor_Z",
            },
        }
    )
    return previous, current, native


def tlvs(data):
    cursor = 40
    result = []
    for _ in range(struct.unpack_from("<I", data, 32)[0]):
        kind, length = struct.unpack_from("<II", data, cursor)
        cursor += 8
        result.append((kind, data[cursor : cursor + length]))
        cursor += length
    return result, cursor


def test_full_common_native_roundtrip():
    previous, current, native = complete_frames()
    units = PointUnits()
    encoder = TIFrameEncoder(start_frame_number=100, units=units)
    encoder.encode(previous)
    data = encoder.encode(current)
    parsed = decode_packet(data)
    assert data[:8] == MAGIC
    assert len(data) % 32 == 0
    assert struct.unpack_from("<8I", data, 8) == (
        0x03050004,
        len(data),
        0xA6843,
        101,
        120000000,
        2,
        4,
        0,
    )
    assert parsed.header.totalPacketLen == len(data)
    bodies, cursor = tlvs(data)
    assert [t for t, _ in bodies] == [1020, 1010, 1011, 1012]
    assert len(data) - cursor < 32
    assert data[cursor:] == bytes(len(data) - cursor)
    scales = units.wire_values()
    for before, after in zip(current.detections, parsed.points, strict=True):
        for attr, scale in zip(
            ("elevation_rad", "azimuth_rad", "radial_velocity_mps", "range_m", "signal_quality"),
            scales,
            strict=True,
        ):
            assert getattr(after, attr) == pytest.approx(getattr(before, attr), abs=scale / 2)
    result = parsed.targets[0]
    assert result.tid == native.tid
    for field in TITarget.model_fields:
        assert getattr(result, field) == pytest.approx(getattr(native, field))
    assert len(result.ec) == 16
    assert result.confidenceLevel == 42
    assert parsed.target_indices == (7, 253, 254, 255)
    assert parsed.heights[0].target_id == 7
    assert parsed.heights[0].maximum_m == pytest.approx(0.35)
    assert parsed.heights[0].minimum_m == pytest.approx(0.05)
    normalized = adapt_packet(
        parsed, frame_id=20, received_at=1234, previous_device_frame_number=100
    )
    assert normalized.associations.device_point_frame_number == 100
    assert normalized.timestamp == 1234
    assert normalized.targets[0].confidence is None


@pytest.mark.parametrize("point_count,track_count", [(0, 0), (0, 1), (1, 0), (8, 1), (20, 2)])
def test_empty_single_two_tracks_and_many_points(point_count, track_count):
    common = frame(
        points=[point()] * point_count, tracks=[track(i + 1) for i in range(track_count)]
    )
    parsed = decode_packet(TIFrameEncoder().encode(common))
    assert len(parsed.points) == parsed.header.numDetectedObj == point_count
    assert len(parsed.targets) == track_count
    assert parsed.header.numTLVs == (2 if track_count else 1)
    assert parsed.target_indices is None
    assert parsed.heights is None
    for target in parsed.targets:
        assert (target.posX, target.posY, target.posZ) == (-0.5, 3, 0.5)
        assert (target.velX, target.velY, target.velZ) == pytest.approx((1.2, 0, -0.4))
        assert (target.accX, target.accY, target.accZ) == (0, 0, 0)
        assert target.ec == pytest.approx(SYNTHETIC_EC)
        assert target.g == 1 and target.confidenceLevel == 0


@pytest.mark.parametrize("az,el", [(-0.5, -0.3), (0.5, 0.3), (0, 0)])
def test_signed_angles_and_radial_doppler(az, el):
    common = frame(
        points=[point(azimuth_rad=az, elevation_rad=el, radial_velocity_mps=-2.4)], tracks=[track()]
    )
    parsed = decode_packet(TIFrameEncoder().encode(common))
    assert parsed.points[0].azimuth_rad == pytest.approx(az, abs=0.005)
    assert parsed.points[0].elevation_rad == pytest.approx(el, abs=0.005)
    assert parsed.points[0].radial_velocity_mps == pytest.approx(-2.4, abs=0.0005)
    assert parsed.points[0].radial_velocity_mps != parsed.targets[0].velX


@pytest.mark.parametrize(
    "field,index,low,high",
    [
        ("elevation_rad", 0, -128, 127),
        ("azimuth_rad", 1, -128, 127),
        ("radial_velocity_mps", 2, -32768, 32767),
        ("range_m", 3, 0, 65535),
        ("signal_quality", 4, 0, 65535),
    ],
)
def test_quantization_limits_and_overflow(field, index, low, high):
    units = PointUnits()
    unit = units.wire_values()[index]
    for value in (low, low + 0.49, high - 0.49, high):
        common = frame(points=[point(**{field: value * unit})])
        parsed = decode_packet(TIFrameEncoder(units=units).encode(common))
        assert getattr(parsed.points[0], field) == round(value) * unit
    for value in (low - 0.01, high + 0.01):
        if field == "range_m" and value < 0:
            continue  # Common model already rejects negative range.
        with pytest.raises(ValueError, match="outside compressed"):
            TIFrameEncoder().encode(frame(points=[point(**{field: value * unit})]))


@pytest.mark.parametrize("unit", [1e-100, 1e100])
def test_units_must_fit_float32(unit):
    with pytest.raises(ValueError, match="float32"):
        encode_packet(TIOutput(frame_number=0, time_cpu_cycles=0), PointUnits(range=unit))


def test_coordinate_pose_replay_and_native_preservation():
    common = frame(points=[point()], tracks=[track()]).model_copy(
        update={
            "source_metadata": {
                "sensor_pose": {"x_m": 0.2, "y_m": 0.3, "height_m": 4},
                "track_target_z_m": 0.1,
            }
        }
    )
    native = frame_to_ti(common, frame_number=25, time_cpu_cycles=30)
    assert (
        native.targets[0].posX,
        native.targets[0].posY,
        native.targets[0].posZ,
    ) == pytest.approx((0.3, 3.9, 1.2))
    assert native.points == common.detections
    parsed = decode_packet(encode_packet(native))
    normalized = adapt_packet(parsed, frame_id=99, received_at=222, floor_origin=(0.2, 0.3))
    restored = decode_packet(TIFrameEncoder().encode(normalized))
    assert restored.targets == parsed.targets
    assert restored.header.frameNumber == 25
    assert restored.header.timeCPUCycles == 30
    explicit_pose = frame_to_ti(
        frame(tracks=[track()]),
        frame_number=0,
        time_cpu_cycles=0,
        sensor_pose=SensorPose(height_m=5),
    )
    assert explicit_pose.targets[0].posY == 5


def test_previous_cloud_count_not_current_and_no_inferred_ids():
    previous, current, _ = complete_frames()
    encoder = TIFrameEncoder(start_frame_number=2**32 - 1)
    encoder.encode(previous)
    parsed = decode_packet(encoder.encode(current))
    assert parsed.header.frameNumber == 0
    assert len(parsed.target_indices) == 4 and len(parsed.points) == 2
    with pytest.raises(ValueError, match="previous emitted"):
        TIFrameEncoder().encode(current)
    for changes, match in [
        ({"point_frame_id": 12}, "previous point"),
        ({"point_frame_id": None}, "unknown"),
        ({"device_point_frame_number": 5}, "reference mismatch"),
        ({"native_indices": (7,), "track_ids": ()}, "count"),
        ({"native_indices": None}, "reason codes"),
        ({"native_indices": (7, 253, 254, 256), "track_ids": ()}, "unsigned bytes"),
    ]:
        broken = current.model_copy(
            update={"associations": current.associations.model_copy(update=changes)}
        )
        encoder = TIFrameEncoder()
        encoder.encode(previous)
        with pytest.raises(ValueError, match=match):
            encoder.encode(broken)
        # Failure does not consume a sequence number or previous-cloud context.
        assert decode_packet(encoder.encode(current)).header.frameNumber == 1


def test_valid_ids_without_native_indices():
    previous = frame(points=[point()])
    current = frame(12).model_copy(
        update={"associations": PointAssociations(point_frame_id=10, track_ids=(7,))}
    )
    encoder = TIFrameEncoder()
    encoder.encode(previous)
    assert decode_packet(encoder.encode(current)).target_indices == (7,)


def test_cycles_host_timestamp_and_uint32_rollover():
    encoder = TIFrameEncoder(start_frame_number=2**32 - 1)
    assert decode_packet(encoder.encode(frame(0))).header.timeCPUCycles == 0
    parsed = decode_packet(encoder.encode(frame(500)))
    assert parsed.header.timeCPUCycles == 5_000_000_000 % 2**32
    assert parsed.header.frameNumber == 0
    assert parsed.header.frameNumber != 500
    host = TIFrameEncoder()
    common = frame(0).model_copy(update={"simulation_time": None})
    assert decode_packet(host.encode(common)).header.timeCPUCycles == 0
    common = common.model_copy(update={"timestamp": common.timestamp + 0.25})
    assert decode_packet(host.encode(common)).header.timeCPUCycles == 50_000_000
    with pytest.raises(ValueError, match="moved backwards"):
        encoder.encode(frame(1))
    with pytest.raises(ValueError, match="cpu_cycles"):
        TIFrameEncoder().encode(
            frame().model_copy(
                update={"device_time_value": 123, "device_time_unit": "unix_seconds"}
            )
        )
    with pytest.raises(ValueError, match="advance"):
        host.encode(common.model_copy(update={"device_frame_number": 1}))


def test_heights_require_coherent_native_bounds():
    _, current, _ = complete_frames()
    with pytest.raises(ValueError, match="native_sensor_Z"):
        frame_to_ti(
            current.model_copy(update={"associations": None, "source_metadata": {}}),
            frame_number=0,
            time_cpu_cycles=0,
        )
    for height in (
        TargetHeight(target_id=8, maximum_m=1, minimum_m=0),
        TargetHeight(target_id=7, maximum_m=0, minimum_m=1),
    ):
        output = TIOutput(
            frame_number=0, time_cpu_cycles=0, targets=(complete_frames()[2],), heights=(height,)
        )
        with pytest.raises(ValueError, match="ordered native"):
            encode_packet(output)


def test_malformed_packets_and_unknown_tlv():
    data = TIFrameEncoder().encode(frame(points=[point()], tracks=[track()]))
    damaged_magic = b"X" + data[1:]
    damaged_length = bytearray(data)
    struct.pack_into("<I", damaged_length, 44, len(data))
    damaged_count = bytearray(data)
    struct.pack_into("<I", damaged_count, 28, 42)
    for malformed in (data[:-1], damaged_magic, bytes(damaged_length), bytes(damaged_count)):
        with pytest.raises(ValueError):
            decode_packet(malformed)
    # Unknown payloads are recorded, not interpreted or generated by the encoder.
    unknown = bytearray(data)
    struct.pack_into("<I", unknown, 68 + 8, 9999)  # point TLV: 8 + 20 + 8, next TLV at 76
    parsed = decode_packet(bytes(unknown))
    assert parsed.unknown_tlv_types == (9999,)
    assert parsed.targets == ()


@pytest.mark.parametrize("chunks", [1, 2, 3, 7, 31, 64, 999])
def test_stream_garbage_concatenation_and_arbitrary_chunks(chunks):
    data = b"garbage\x02\x01rubbish" + TIFrameEncoder().encode(frame())
    data += TIFrameEncoder(start_frame_number=1).encode(frame(points=[point()]))
    decoder = TIStreamDecoder()
    parsed = []
    for start in range(0, len(data), chunks):
        parsed.extend(decoder.feed(data[start : start + chunks]))
    decoder.finish()
    assert [p.header.frameNumber for p in parsed] == [0, 1]
    assert [len(p.points) for p in parsed] == [0, 1]


def test_stream_random_chunks_and_truncation():
    data = TIFrameEncoder().encode(frame(points=[point()]))
    decoder = TIStreamDecoder()
    parsed = []
    rng = random.Random(42)
    offset = 0
    while offset < len(data):
        size = rng.randint(1, 19)
        parsed.extend(decoder.feed(data[offset : offset + size]))
        offset += size
    assert len(parsed) == 1
    decoder.finish()
    decoder.feed(data[:-1])
    with pytest.raises(ValueError, match="truncated"):
        decoder.finish()
    bad = bytearray(data)
    struct.pack_into("<I", bad, 12, 2**32 - 1)
    with pytest.raises(ValueError, match="length"):
        TIStreamDecoder().feed(bytes(bad))


def test_aligned_payload_does_not_add_extra_block():
    # 40 header + 28 cloud TLV + 8*12 points + 124? use one 112-byte target:
    # 40 + 28 + 8*n + 120 -> n=0 mod4 with total 188 (not aligned).
    # One index TLV with 4 bytes makes total 200 + 8*n; n=3 gives 224.
    output = TIOutput(
        frame_number=1,
        time_cpu_cycles=0,
        points=(point(),) * 3,
        targets=(complete_frames()[2],),
        target_indices=(7,) * 4,
        association_point_frame_number=0,
    )
    data = encode_packet(output)
    assert len(data) == 224
    assert tlvs(data)[1] == len(data)


def test_capture_and_simulated_points(tmp_path):
    path = tmp_path / "test.bin"
    frames = list(islice(simulated_frames(AppConfig(), "two_targets"), 3))
    assert all(len(f.detections) > 4 for f in frames)
    assert all(len(f.targets) == 2 for f in frames)
    assert all(f.source_metadata["sensor_pose"]["height_m"] == 3 for f in frames)
    assert write_capture(path, frames) == 3
    decoder = TIStreamDecoder()
    parsed = decoder.feed(path.read_bytes())
    decoder.finish()
    assert len(parsed) == 3
    with pytest.raises(FileExistsError):
        write_capture(path, frames)


@pytest.mark.parametrize("arguments", [["--frames", "2"], ["--seconds", "0.11"]])
def test_capture_cli(tmp_path, arguments):
    path = tmp_path / "capture.bin"
    result = subprocess.run(
        [sys.executable, "-m", "rf_sim.ti_capture", str(path), *arguments],
        check=False,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stderr
    parsed = TIStreamDecoder().feed(path.read_bytes())
    assert len(parsed) == (2 if arguments[0] == "--frames" else 3)


def test_local_ti_reference_parser_all_four_tlvs(tmp_path):
    common = Path(
        os.environ.get(
            "TI_VISUALIZER_COMMON",
            "C:/ti/radar_toolbox_4_00_00_05/tools/visualizers/Applications_Visualizer/common",
        )
    )
    if not (common / "parseFrame.py").is_file():
        pytest.skip("local TI reference parser not installed; set TI_VISUALIZER_COMMON")
    previous, current, _ = complete_frames()
    encoder = TIFrameEncoder()
    encoder.encode(previous)
    path = tmp_path / "reference.bin"
    path.write_bytes(encoder.encode(current))
    result = subprocess.run(
        [
            sys.executable,
            "-B",
            str(ROOT / "tools/validate_ti_reference.py"),
            str(path),
            "--common-dir",
            str(common),
        ],
        check=False,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["ti_reference_parser_verified"] is True
    assert report["hardware_capture_verified"] is False
    assert report["tlv_types"] == [1020, 1010, 1011, 1012]
    assert report["covariance_layout_checks"] == 1
    assert report["points"] == 2 and report["tracks"] == 1
    assert math.isclose(report["header"]["timeCPUCycles"], 120000000)


def test_capture_replays_common_and_existing_jsonl(tmp_path):
    frames = list(islice(simulated_frames(AppConfig(), "two_targets"), 2))
    path = tmp_path / "measurements.jsonl"
    path.write_text("\n".join(f.model_dump_json() for f in frames), encoding="utf-8")
    common = list(recorded_frames(path))
    assert common[0].detections == frames[0].detections
    output = tmp_path / "replay.bin"
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "rf_sim.ti_capture",
            str(output),
            "--input",
            str(path),
            "--frames",
            "2",
        ],
        check=False,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stderr
    assert len(TIStreamDecoder().feed(output.read_bytes())) == 2
    path.write_text(frame().model_dump_json(), encoding="utf-8")
    assert list(recorded_frames(path)) == [frame()]
    path.write_text('{"schema_version":"2.0"}', encoding="utf-8")
    with pytest.raises(ValueError, match="line 1"):
        list(recorded_frames(path))


def test_native_header_metadata_and_unsupported_presence():
    common = frame(points=[point()]).model_copy(
        update={
            "presence": True,
            "source_metadata": {
                "ti_header": {"version": 123, "platform": 456, "subFrameNumber": 2}
            },
        }
    )
    parsed = decode_packet(TIFrameEncoder().encode(common))
    assert (parsed.header.version, parsed.header.platform, parsed.header.subFrameNumber) == (
        123,
        456,
        2,
    )
    assert parsed.presence is None
    assert [t for t, _ in tlvs(TIFrameEncoder().encode(common))[0]] == [1020]
