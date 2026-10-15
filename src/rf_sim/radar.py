"""Synthetic target measurements, acquisition scheduling, and delayed delivery.

SNR = 30 - 2*range + reflector gain + Gaussian noise (dB). Confidence
is a bounded logistic SNR score times a mild range penalty. Neither formula
is a radar equation or a prediction of TI performance.
"""

import math
from collections import deque
from collections.abc import Sequence
from typing import Protocol

import numpy as np

from rf_common.frame import RadarFrame
from rf_common.models import RadarPoint
from rf_sim.config import RadarConfig, RoomConfig
from rf_sim.models import RadarTargetMeasurement, TargetFrame
from rf_sim.motion import Vehicle


class RadarSource(Protocol):
    """Downstream boundary: return ready normalized frames; no truth arguments.

    A future UART adapter can implement poll() and normalize TI measurements.
    This protocol imposes no simulation-specific acquisition method on hardware.
    """

    def poll(self, current_time: float) -> list[RadarFrame]: ...


class RadarSensor:
    """Measurement model using only its own seeded NumPy generator."""

    def __init__(self, config: RadarConfig, room: RoomConfig, seed: int) -> None:
        self.config = config
        self.room = room
        self.rng = np.random.default_rng(seed)
        self.false_id = 1_000_000
        # Independent random stream preserves the existing track/control behavior.
        self.point_rng = np.random.default_rng(np.random.SeedSequence([seed, 1]))

    def detect_points(self, vehicles: Sequence[Vehicle]) -> tuple[RadarPoint, ...]:
        """Uniform volume scatterers; TI spherical axes X, boresight Y, Z.

        Room floor (x,y,height) maps to sensor (x, mounting-height-height, y).
        Only line-of-sight velocity is measured. No truth IDs leave this stage.
        """
        cfg, rng = self.config, self.point_rng
        if not cfg.point_cloud_enabled:
            return ()
        points = []
        for vehicle in vehicles:
            state, target = vehicle.state, vehicle.target
            if rng.random() >= cfg.detection_probability:
                continue
            for _ in range(rng.poisson(cfg.points_per_object)):
                if rng.random() >= cfg.point_detection_probability:
                    continue
                forward = rng.uniform(-target.length_m / 2, target.length_m / 2)
                lateral = rng.uniform(-target.width_m / 2, target.width_m / 2)
                c, s = math.cos(state.heading), math.sin(state.heading)
                dx = state.x + c * forward - s * lateral - cfg.sensor_x_m
                dz = state.y + s * forward + c * lateral - cfg.sensor_y_m
                dy = cfg.sensor_height_m - rng.uniform(0, target.height_m)
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                if not cfg.minimum_range_m <= distance <= cfg.maximum_range_m:
                    continue
                gain = target.scattering_gain_db
                if target.reflector_enabled:
                    gain += target.reflector_gain_db
                points.append(
                    RadarPoint(
                        range_m=max(0, distance + rng.normal(0, cfg.range_noise_std_m)),
                        azimuth_rad=math.atan2(dx, dy) + rng.normal(0, cfg.angle_noise_std_rad),
                        elevation_rad=math.atan2(dz, math.hypot(dx, dy))
                        + rng.normal(0, cfg.angle_noise_std_rad),
                        radial_velocity_mps=(dx * state.vx + dz * state.vy) / max(distance, 1e-9)
                        + rng.normal(0, cfg.doppler_noise_std_mps),
                        signal_quality=cfg.snr_reference_db
                        - cfg.snr_range_loss_db_per_m * distance
                        + gain
                        + rng.normal(0, cfg.snr_noise_std_db),
                        signal_quality_kind="synthetic_db",
                    )
                )
        return tuple(points)

    def _measurement(
        self, target_id: int, x: float, y: float, vx: float, vy: float, reflector_gain: float = 0.0
    ) -> RadarTargetMeasurement:
        cfg = self.config
        x, y = np.array([x, y]) + self.rng.normal(0, cfg.position_noise_std_m, 2)
        vx, vy = np.array([vx, vy]) + self.rng.normal(0, cfg.velocity_noise_std_mps, 2)
        dx, dy = x - cfg.sensor_x_m, y - cfg.sensor_y_m
        horizontal_range_m = math.hypot(dx, dy)
        slant_range_m = math.hypot(horizontal_range_m, cfg.sensor_height_m)
        distance = slant_range_m
        snr = float(
            cfg.snr_reference_db
            - cfg.snr_range_loss_db_per_m * distance
            + reflector_gain
            + self.rng.normal(0, cfg.snr_noise_std_db)
        )
        quality = 1 / (1 + math.exp(-max(-60, min(60, (snr - 8) / 5))))
        confidence = max(
            0.0, min(1.0, quality * (1 - 0.15 * min(1.0, distance / cfg.maximum_range_m)))
        )
        speed = math.hypot(vx, vy)
        return RadarTargetMeasurement(
            target_id=target_id,
            x=float(x),
            y=float(y),
            vx=float(vx),
            vy=float(vy),
            speed=speed,
            heading_rad=math.atan2(vy, vx) if speed > 1e-9 else None,
            range_m=horizontal_range_m if cfg.legacy_horizontal_range else slant_range_m,
            horizontal_range_m=horizontal_range_m,
            slant_range_m=slant_range_m,
            azimuth_rad=math.atan2(dy, dx),
            confidence=confidence,
            snr_db=snr,
        )

    def measure(self, vehicles: Sequence[Vehicle]) -> tuple[RadarTargetMeasurement, ...]:
        measurements = []
        cfg = self.config
        for vehicle in vehicles:
            state = vehicle.state
            distance = math.hypot(
                state.x - cfg.sensor_x_m, state.y - cfg.sensor_y_m, cfg.sensor_height_m
            )
            if not cfg.minimum_range_m <= distance <= cfg.maximum_range_m:
                continue
            # Reflector affects strength only, preserving a clear per-frame
            # detection_probability independent of synthetic classification cues.
            if self.rng.random() >= cfg.detection_probability:
                continue
            gain = vehicle.target.reflector_gain_db if vehicle.target.reflector_enabled else 0
            measurements.append(
                self._measurement(state.target_id, state.x, state.y, state.vx, state.vy, gain)
            )
        if self.rng.random() < cfg.false_alarm_probability:
            # At most one clutter target per opportunity. Candidate must be in
            # sensor range. No simulator-only false-alarm flag is transmitted.
            x = float(self.rng.uniform(0, self.room.width_m))
            y = float(self.rng.uniform(0, self.room.height_m))
            distance = math.hypot(x - cfg.sensor_x_m, y - cfg.sensor_y_m, cfg.sensor_height_m)
            if cfg.minimum_range_m <= distance <= cfg.maximum_range_m:
                measurements.append(self._measurement(self.false_id, x, y, 0, 0))
                self.false_id += 1
        return tuple(measurements)


class SimulatedRadarSource:
    """Accept truth internally; downstream consumers only call poll()."""

    def __init__(self, sensor: RadarSensor, epoch: float) -> None:
        self.sensor = sensor
        self.epoch = epoch
        self.next_frame_id = 0
        self.pending: deque[tuple[float, TargetFrame]] = deque()

    def acquire(self, vehicles: Sequence[Vehicle], simulation_time: float) -> None:
        frame = TargetFrame(
            frame_id=self.next_frame_id,
            timestamp=self.epoch + simulation_time,
            simulation_time=simulation_time,
            targets=self.sensor.measure(vehicles),
            detections=self.sensor.detect_points(vehicles),
            source_metadata={
                "point_coordinates": "sensor_X_downY_Z",
                "sensor_pose": {
                    "x_m": self.sensor.config.sensor_x_m,
                    "y_m": self.sensor.config.sensor_y_m,
                    "height_m": self.sensor.config.sensor_height_m,
                },
                "track_coordinates": "room_floor_xy",
                "track_range": (
                    "legacy_horizontal" if self.sensor.config.legacy_horizontal_range else "slant"
                ),
                "track_azimuth": "legacy_floor_bearing_from_positive_x",
                "track_target_z_m": 0.0,
                "tracker": "synthetic_noisy_truth_surrogate",
                "associations": "unavailable_independent_track_surrogate",
            },
        )
        self.next_frame_id += 1
        self.pending.append((simulation_time + self.sensor.config.latency_ms / 1000, frame))

    def poll(self, current_time: float) -> list[TargetFrame]:
        frames = []
        while self.pending and self.pending[0][0] <= current_time + 1e-10:
            frames.append(self.pending.popleft()[1])
        return frames
