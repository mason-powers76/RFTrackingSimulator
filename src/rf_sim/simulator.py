"""Fixed-step orchestration; no Pygame, files, or sockets."""

import time

from rf_sim.config import AppConfig
from rf_sim.controllers import ManualController
from rf_sim.models import TargetFrame
from rf_sim.motion import Control, Vehicle
from rf_sim.radar import RadarSensor, SimulatedRadarSource
from rf_sim.scenarios import make_controller, scenario_config


class Simulator:
    """Step advances exactly one physics tick regardless of rendering speed.

    Radar opportunities are scheduled at their own rate and acquired on the
    first physics tick at/after each deadline. Timing jitter is < one tick;
    delivery latency is rounded up to a physics tick as well.
    """

    def __init__(self, config: AppConfig, scenario: str, epoch: float | None = None) -> None:
        config = scenario_config(scenario, config)
        self.config = config
        self.scenario = scenario
        self.dt = 1 / config.simulation.physics_rate_hz
        self.tick = 0
        self.simulation_time = 0.0
        self.epoch = time.time() if epoch is None else epoch
        self.source = SimulatedRadarSource(
            RadarSensor(config.radar, config.room, config.simulation.random_seed), self.epoch
        )
        self.radar_opportunity = 0
        self.latest_frame: TargetFrame | None = None
        self.emergency_stop = False
        self.reset_vehicles()

    def reset_vehicles(self) -> None:
        """Reset motion/controller states; preserve frame IDs and timeline."""
        self.vehicles = [
            Vehicle(t, self.config.vehicle, self.config.room) for t in self.config.targets
        ]
        self.controllers = [
            make_controller(self.scenario, self.config, index)
            for index in range(len(self.vehicles))
        ]

    def set_manual_input(
        self, throttle: float, steering: float, stop: bool, *, target_index: int | None = None
    ) -> None:
        """Set one target's input, or broadcast for the existing manual scenario.

        Emergency stop always applies to every vehicle.
        """
        self.emergency_stop = stop
        controllers = self.controllers if target_index is None else [self.controllers[target_index]]
        for controller in controllers:
            if isinstance(controller, ManualController):
                controller.throttle = throttle
                controller.steering = steering
                controller.stop = stop

    def step(self) -> list[TargetFrame]:
        for vehicle, controller in zip(self.vehicles, self.controllers, strict=True):
            control = (
                Control(emergency_stop=True)
                if self.emergency_stop
                else controller.command(vehicle.state, vehicle.parameters)
            )
            vehicle.step(control, self.dt)
        self.tick += 1
        self.simulation_time = self.tick * self.dt
        deadline = self.radar_opportunity / self.config.radar.measurement_rate_hz
        if self.simulation_time + 1e-10 >= deadline:
            self.source.acquire(self.vehicles, self.simulation_time)
            self.radar_opportunity += 1
        frames = self.source.poll(self.simulation_time)
        if frames:
            self.latest_frame = frames[-1]
        return frames
