"""Simple waypoint and keyboard-command controllers."""

import math
from dataclasses import dataclass
from typing import Protocol

from rf_sim.config import VehicleConfig
from rf_sim.motion import CarState, Control, wrap_angle


class Controller(Protocol):
    def command(self, state: CarState, parameters: VehicleConfig) -> Control: ...


class StationaryController:
    def command(self, state: CarState, parameters: VehicleConfig) -> Control:
        return Control(emergency_stop=True)


class WaypointController:
    """Proportional heading/speed control; no path-planning machinery."""

    def __init__(
        self,
        waypoints: list[tuple[float, float]],
        cruise_speed: float = 0.75,
        tolerance: float = 0.22,
        loop: bool = True,
    ) -> None:
        if not waypoints:
            raise ValueError("waypoints must not be empty")
        self.waypoints = waypoints
        self.cruise_speed = cruise_speed
        self.tolerance = tolerance
        self.loop = loop
        self.index = 0
        self.finished = False

    def command(self, state: CarState, parameters: VehicleConfig) -> Control:
        if self.finished:
            return Control(emergency_stop=True)
        x, y = self.waypoints[self.index]
        distance = math.hypot(x - state.x, y - state.y)
        if distance <= self.tolerance:
            if self.index == len(self.waypoints) - 1 and not self.loop:
                self.finished = True
                return Control(emergency_stop=True)
            self.index = (self.index + 1) % len(self.waypoints)
            x, y = self.waypoints[self.index]
            distance = math.hypot(x - state.x, y - state.y)
        error = wrap_angle(math.atan2(y - state.y, x - state.x) - state.heading)
        steering = 1.2 * error
        desired_speed = self.cruise_speed * max(0.25, math.cos(error))
        desired_speed *= min(1.0, max(0.4, distance / 0.6))
        return Control(acceleration=3.0 * (desired_speed - state.speed), steering_angle=steering)


@dataclass
class ManualController:
    """GUI supplies normalized throttle/steering; controller knows no Pygame."""

    throttle: float = 0.0
    steering: float = 0.0
    stop: bool = False

    def command(self, state: CarState, parameters: VehicleConfig) -> Control:
        if self.throttle * state.speed < 0:
            acceleration = self.throttle * parameters.max_braking_mps2
        else:
            acceleration = self.throttle * parameters.max_acceleration_mps2
        return Control(
            acceleration, self.steering * math.radians(parameters.max_steering_deg), self.stop
        )
