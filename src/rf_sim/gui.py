"""Pygame debug drawing and input only; simulation remains in SI coordinates."""

import math
from collections import defaultdict, deque
from dataclasses import dataclass

import pygame

from rf_sim.models import TargetFrame
from rf_sim.simulator import Simulator

TRUTH_COLOR = (80, 220, 140)
MEASUREMENT_COLOR = (255, 165, 65)


@dataclass(frozen=True)
class KeyboardMapping:
    forward: int
    reverse: int
    left: int
    right: int

    def read(self, keys) -> tuple[float, float]:
        return (
            float(keys[self.forward]) - float(keys[self.reverse]),
            float(keys[self.left]) - float(keys[self.right]),
        )


WASD = KeyboardMapping(pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d)
ARROWS = KeyboardMapping(pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT)


@dataclass(frozen=True)
class CoordinateTransform:
    left: int
    top: int
    scale: float
    height_m: float

    def to_screen(self, x: float, y: float) -> tuple[int, int]:
        return (
            round(self.left + x * self.scale),
            round(self.top + (self.height_m - y) * self.scale),
        )


class DebugGUI:
    """Green truth vehicle vs orange measured rings; ten-second bounded trails."""

    def __init__(self, simulator: Simulator) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((1180, 760))
        pygame.display.set_caption("RF Target Simulator — engineering debug view")
        self.font = pygame.font.Font(None, 23)
        self.config = simulator.config
        self.key_mappings = (
            (WASD, ARROWS) if simulator.scenario == "two_targets_manual" else (WASD,)
        )
        room = self.config.room
        self.transform = CoordinateTransform(
            40, 60, min(730 / room.width_m, 580 / room.height_m), room.height_m
        )
        self.running = True
        self.paused = False
        self.truth_trails: dict[int, deque[tuple[float, float, float]]] = defaultdict(deque)
        self.measurement_trails: dict[int, deque[tuple[float, float, float]]] = defaultdict(deque)
        self.last_render = float("-inf")

    def handle_input(self, simulator: Simulator) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    simulator.reset_vehicles()
                    self.truth_trails.clear()
                    self.measurement_trails.clear()
        keys = pygame.key.get_pressed()
        stop = bool(keys[pygame.K_SPACE])
        if simulator.scenario == "two_targets_manual":
            for index, mapping in enumerate(self.key_mappings):
                throttle, steering = mapping.read(keys)
                simulator.set_manual_input(throttle, steering, stop, target_index=index)
        else:
            throttle, steering = WASD.read(keys)
            simulator.set_manual_input(throttle, steering, stop)

    def observe(self, simulator: Simulator, frames: list[TargetFrame]) -> None:
        now = simulator.simulation_time
        for vehicle in simulator.vehicles:
            state = vehicle.state
            self.truth_trails[state.target_id].append((now, state.x, state.y))
        for frame in frames:
            for target in frame.targets:
                self.measurement_trails[target.target_id].append((now, target.x, target.y))
        for histories in [self.truth_trails, self.measurement_trails]:
            for target_id in list(histories):
                history = histories[target_id]
                while history and history[0][0] < now - 10:
                    history.popleft()
                if not history:
                    del histories[target_id]

    def _text(
        self, text: str, x: int, y: int, color: tuple[int, int, int] = (220, 225, 235)
    ) -> None:
        self.screen.blit(self.font.render(text, True, color), (x, y))

    def render(self, simulator: Simulator) -> None:
        self.screen.fill((22, 27, 36))
        room = self.config.room
        transform = self.transform
        for x in range(math.floor(room.width_m) + 1):
            pygame.draw.line(
                self.screen,
                (50, 60, 70),
                transform.to_screen(x, 0),
                transform.to_screen(x, room.height_m),
            )
            px, py = transform.to_screen(x, 0)
            self._text(f"{x} m", px - 10, py + 10)
        for y in range(math.floor(room.height_m) + 1):
            pygame.draw.line(
                self.screen,
                (50, 60, 70),
                transform.to_screen(0, y),
                transform.to_screen(room.width_m, y),
            )
            px, py = transform.to_screen(0, y)
            self._text(str(y), px - 22, py - 8)
        upper_left = transform.to_screen(0, room.height_m)
        pygame.draw.rect(
            self.screen,
            (175, 185, 195),
            (
                *upper_left,
                round(room.width_m * transform.scale),
                round(room.height_m * transform.scale),
            ),
            2,
        )
        for histories, color in [
            (self.truth_trails, TRUTH_COLOR),
            (self.measurement_trails, MEASUREMENT_COLOR),
        ]:
            for history in histories.values():
                points = [transform.to_screen(x, y) for _, x, y in history]
                if len(points) > 1:
                    pygame.draw.lines(self.screen, color, False, points, 1)
        radar = self.config.radar
        sensor = transform.to_screen(radar.sensor_x_m, radar.sensor_y_m)
        pygame.draw.circle(self.screen, (100, 165, 255), sensor, 10)
        self._text(
            f"RADAR ({radar.sensor_height_m:.1f} m overhead)",
            sensor[0] + 12,
            sensor[1] - 15,
            (100, 165, 255),
        )
        self._text(f"X: 0–{room.width_m:g} m | Y: 0–{room.height_m:g} m", 40, 660)
        for vehicle in simulator.vehicles:
            state = vehicle.state
            point = transform.to_screen(state.x, state.y)
            pygame.draw.circle(
                self.screen,
                TRUTH_COLOR,
                point,
                max(5, round(vehicle.parameters.radius_m * transform.scale)),
                2,
            )
            nose = transform.to_screen(
                state.x + 0.25 * math.cos(state.heading), state.y + 0.25 * math.sin(state.heading)
            )
            pygame.draw.line(self.screen, TRUTH_COLOR, point, nose, 3)
            self._text(str(state.target_id), point[0] + 16, point[1], TRUTH_COLOR)
        frame = simulator.latest_frame
        if frame:
            for target in frame.targets:
                x, y = transform.to_screen(target.x, target.y)
                pygame.draw.circle(self.screen, MEASUREMENT_COLOR, (x, y), 8, 2)
                pygame.draw.line(self.screen, MEASUREMENT_COLOR, (x - 11, y), (x + 11, y), 2)
                pygame.draw.line(self.screen, MEASUREMENT_COLOR, (x, y - 11), (x, y + 11), 2)
        self._text("Green: ground truth    Orange: radar measurement", 40, 25)
        state = simulator.vehicles[0].state
        measurement = (
            next((t for t in frame.targets if t.target_id == state.target_id), None)
            if frame
            else None
        )
        network = self.config.network
        lines = [
            "SIMULATION",
            f"Scenario: {simulator.scenario}",
            f"Simulation time: {simulator.simulation_time:.2f} s",
            f"Physics: {self.config.simulation.physics_rate_hz:g} Hz",
            f"Radar: {radar.measurement_rate_hz:g} Hz",
            f"GUI target: {self.config.simulation.gui_rate_hz} FPS",
            f"Pause: {'PAUSED' if self.paused else 'running'}",
            "",
            "GROUND TRUTH",
            f"Target {state.target_id} ({len(simulator.vehicles)} total)",
            f"XY: {state.x:.3f}, {state.y:.3f} m",
            f"Speed: {state.speed:.2f} m/s",
            f"Heading: {math.degrees(state.heading):.1f} deg",
            "",
            "RADAR",
            f"Position noise: {radar.position_noise_std_m:g} m",
            f"Detection probability: {radar.detection_probability:.2f}",
            f"Latency: {radar.latency_ms:g} ms",
            f"Frame: {frame.frame_id if frame else 'waiting'}",
            f"Target {state.target_id}: {'detected' if measurement else 'not detected'}",
        ]
        if measurement:
            lines.extend(
                [
                    f"XY: {measurement.x:.3f}, {measurement.y:.3f} m",
                    f"Range: {measurement.range_m:.2f} m",
                    f"Floor bearing: {math.degrees(measurement.azimuth_rad):.1f} deg",
                    f"SNR: {measurement.snr_db:.1f} dB",
                    f"Confidence: {measurement.confidence:.3f}",
                ]
            )
        lines.extend(
            [
                "",
                "NETWORK",
                f"UDP: {'ENABLED' if network.enabled else 'disabled'}",
                f"Destination: {network.host}:{network.port}",
            ]
        )
        if simulator.scenario == "two_targets_manual":
            # Keep both target summaries and existing status within the side panel.
            lines = lines[:8] + ["GROUND TRUTH"]
            for vehicle in simulator.vehicles:
                state = vehicle.state
                lines.extend(
                    [
                        (
                            f"Target {state.target_id}: reflector "
                            f"{'on' if vehicle.target.reflector_enabled else 'off'}"
                        ),
                        f"XY: {state.x:.3f}, {state.y:.3f} m",
                        (
                            f"Speed: {state.speed:.2f} m/s | "
                            f"Heading: {math.degrees(state.heading):.1f} deg"
                        ),
                    ]
                )
            lines.extend(
                [
                    "",
                    "RADAR",
                    f"Position noise: {radar.position_noise_std_m:g} m",
                    f"Detection probability: {radar.detection_probability:.2f}",
                    f"Latency: {radar.latency_ms:g} ms",
                    f"Frame: {frame.frame_id if frame else 'waiting'}",
                ]
            )
            for vehicle in simulator.vehicles:
                measurement = (
                    next((t for t in frame.targets if t.target_id == vehicle.state.target_id), None)
                    if frame
                    else None
                )
                lines.append(
                    f"Target {vehicle.state.target_id}: "
                    f"{'detected' if measurement else 'not detected'}"
                )
                if measurement:
                    lines.extend(
                        [
                            f"XY: {measurement.x:.3f}, {measurement.y:.3f} m",
                            (
                                f"Range: {measurement.range_m:.2f} m | "
                                f"Floor bearing: {math.degrees(measurement.azimuth_rad):.1f} deg"
                            ),
                            (
                                f"SNR: {measurement.snr_db:.1f} dB | "
                                f"Confidence: {measurement.confidence:.3f}"
                            ),
                        ]
                    )
            lines.extend(
                [
                    "",
                    "NETWORK",
                    f"UDP: {'ENABLED' if network.enabled else 'disabled'}",
                    f"Destination: {network.host}:{network.port}",
                ]
            )
        for index, line in enumerate(lines):
            spacing = 20 if simulator.scenario == "two_targets_manual" else 23
            self._text(line, 815, 25 + index * spacing)
        help_text = (
            "Target 1: W/S/A/D | Target 2: Arrow Keys | SPACE stop"
            if simulator.scenario == "two_targets_manual"
            else "W forward | S brake/reverse | A left | D right | SPACE stop"
        )
        self._text(help_text, 40, 690)
        self._text("R reset vehicles | P pause | ESC exit | Trails: last 10 seconds", 40, 720)
        pygame.display.flip()

    def close(self) -> None:
        pygame.quit()
