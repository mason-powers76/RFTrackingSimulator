import math
import struct

import pytest

from rf_backend.adapter import parse_packet
from rf_backend.store import RadarStore
from rf_common.ti import MAGIC, adapt_packet, decode_packet
from rf_sim.config import AppConfig, RadarConfig, RoomConfig, TargetConfig, VehicleConfig
from rf_sim.motion import Vehicle
from rf_sim.radar import RadarSensor
from rf_sim.transport import serialize_frame


def packet_bytes():
    # Independent fixture with signed angles/Doppler, unsigned range/SNR,
    # native confidence > 1, all 16 covariance entries, and previous-frame indices.
    target = struct.pack("<I27f", 7, 1, 2, 3, 4, 5, 6, 7, 8, 9, *range(16), 2.5, 42)
    points = struct.pack("<5fbbhHH", 0.01, 0.02, 0.1, 0.01, 0.1, -3, 4, -5, 40000, 50000)
    tlvs = [
        (1010, target),
        (1020, points),
        (1011, bytes([7, 253, 254, 255])),
        (1021, struct.pack("<I", 1)),
        (1012, struct.pack("<I2f", 7, 3.5, 0.5)),
        (9999, b"unknown"),
    ]
    payload = b"".join(struct.pack("<II", kind, len(body)) + body for kind, body in tlvs)
    size = math.ceil((40 + len(payload)) / 32) * 32
    return (
        MAGIC
        + struct.pack("<8I", 1, size, 0xA6843, 100, 123456, 1, len(tlvs), 0)
        + payload
        + bytes(size - 40 - len(payload))
    )


def test_native_ti_fields_and_floor_adapter(tmp_path):
    packet = decode_packet(packet_bytes())
    assert packet.targets[0].ec == tuple(range(16))
    assert packet.targets[0].confidenceLevel == 42
    assert packet.points[0].range_m == pytest.approx(400)
    assert packet.points[0].signal_quality == pytest.approx(5000)
    assert packet.points[0].radial_velocity_mps == pytest.approx(-0.5)
    frame = adapt_packet(
        packet,
        frame_id=3,
        received_at=1700000000,
        floor_origin=(10, 20),
        previous_device_frame_number=98,
    )
    assert (frame.targets[0].x, frame.targets[0].y) == (11, 23)
    assert frame.targets[0].speed == pytest.approx(math.hypot(4, 6))
    assert frame.targets[0].confidence is None
    assert frame.timestamp != frame.device_time_value
    assert frame.device_time_unit == "cpu_cycles"
    assert frame.associations.device_point_frame_number == 98
    assert frame.associations.track_ids == (7, None, None, None)
    assert frame.presence is True
    assert frame.target_heights[0].maximum_m == 3.5
    parsed = parse_packet(serialize_frame(frame))
    assert parsed == frame
    with_store = RadarStore(tmp_path / "test.sqlite", "hardware")
    try:
        assert with_store.ingest(parsed)
        assert with_store.latest()["frame"] == frame.model_dump(mode="json")
        assert not with_store.ingest(parsed)
    finally:
        with_store.close()


@pytest.mark.parametrize("data", [b"", packet_bytes()[:-1], packet_bytes()[:40] + bytes(32)])
def test_reject_truncated_packets(data):
    with pytest.raises(ValueError):
        decode_packet(data)


def test_points_have_extent_and_only_radial_velocity():
    cfg = RadarConfig(
        detection_probability=1,
        point_detection_probability=1,
        points_per_object=100,
        range_noise_std_m=0,
        angle_noise_std_rad=0,
        doppler_noise_std_mps=0,
    )
    car = Vehicle(TargetConfig(initial_x=1.5, initial_y=1.5), VehicleConfig(), RoomConfig())
    car.state.speed = 1
    left = RadarSensor(cfg, RoomConfig(), 42)
    right = RadarSensor(cfg, RoomConfig(), 42)
    points = left.detect_points([car])
    assert points == right.detect_points([car])
    assert len(points) > 2
    positions = []
    for p in points:
        dx = p.range_m * math.cos(p.elevation_rad) * math.sin(p.azimuth_rad)
        dy = p.range_m * math.cos(p.elevation_rad) * math.cos(p.azimuth_rad)
        dz = p.range_m * math.sin(p.elevation_rad)
        assert 1.5 - car.target.length_m / 2 <= dx + cfg.sensor_x_m <= 1.5 + car.target.length_m / 2
        assert 1.5 - car.target.width_m / 2 <= dz + cfg.sensor_y_m <= 1.5 + car.target.width_m / 2
        assert 0 <= cfg.sensor_height_m - dy <= car.target.height_m
        assert p.radial_velocity_mps == pytest.approx(dx / p.range_m)
        assert "vx" not in p.model_dump() and "target_id" not in p.model_dump()
        positions.append((dx, dz))
    assert len(set(positions)) == len(points)
    assert len(AppConfig().targets) == 2
    cfg.point_cloud_enabled = False
    assert left.detect_points([car]) == ()


def test_points_dropout_and_track_rng_are_independent():
    car = Vehicle(TargetConfig(), VehicleConfig(), RoomConfig())
    left = RadarSensor(RadarConfig(), RoomConfig(), 42)
    right = RadarSensor(RadarConfig(point_cloud_enabled=False), RoomConfig(), 42)
    for _ in range(10):
        assert left.measure([car]) == right.measure([car])
        left.detect_points([car])
    left.config.point_detection_probability = 0
    assert left.detect_points([car]) == ()
