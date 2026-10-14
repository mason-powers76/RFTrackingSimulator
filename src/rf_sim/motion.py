"""Deterministic kinematic bicycle integration, independent of GUI and radar."""

import math
from dataclasses import dataclass

from rf_sim.config import RoomConfig, TargetConfig, VehicleConfig


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class CarState:
    target_id: int
    x: float
    y: float
    speed: float = 0.0
    heading: float = 0.0
    steering_angle: float = 0.0
    acceleration: float = 0.0

    @property
    def vx(self) -> float:
        return self.speed * math.cos(self.heading)

    @property
    def vy(self) -> float:
        return self.speed * math.sin(self.heading)


@dataclass(frozen=True)
class Control:
    acceleration: float = 0.0
    steering_angle: float = 0.0
    emergency_stop: bool = False


class Vehicle:
    """One vehicle and its footprint; a wall collision stops signed speed."""

    def __init__(self, target: TargetConfig, defaults: VehicleConfig, room: RoomConfig) -> None:
        self.target = target
        self.parameters = target.vehicle or defaults
        self.room = room
        self.state = CarState(
            target.target_id, target.initial_x, target.initial_y, heading=target.initial_heading
        )

    def step(self, control: Control, dt: float) -> None:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        state, params = self.state, self.parameters
        limit = math.radians(params.max_steering_deg)
        state.steering_angle = max(-limit, min(limit, control.steering_angle))
        # Braking can use max_braking; increasing speed in either direction uses
        # max_acceleration. This avoids treating reverse acceleration as braking.
        lower = -params.max_braking_mps2 if state.speed > 0 else -params.max_acceleration_mps2
        upper = params.max_braking_mps2 if state.speed < 0 else params.max_acceleration_mps2
        state.acceleration = max(lower, min(upper, control.acceleration))
        if control.emergency_stop:
            state.speed = state.acceleration = 0.0
            return
        old_speed = state.speed
        state.x += state.vx * dt
        state.y += state.vy * dt
        state.heading = wrap_angle(
            state.heading + old_speed / params.wheelbase_m * math.tan(state.steering_angle) * dt
        )
        new_speed = old_speed + state.acceleration * dt
        if old_speed * new_speed < 0:
            # Stop first; the next step may accelerate in the opposite direction.
            new_speed = 0.0
        state.speed = max(
            -params.max_reverse_speed_mps, min(params.max_forward_speed_mps, new_speed)
        )
        radius = params.radius_m
        x = max(radius, min(self.room.width_m - radius, state.x))
        y = max(radius, min(self.room.height_m - radius, state.y))
        if x != state.x or y != state.y:
            state.x, state.y = x, y
            state.speed = state.acceleration = 0.0
