from collections import defaultdict

import numpy as np
import pygame
import pytest

from rf_sim.config import AppConfig
from rf_sim.gui import MEASUREMENT_COLOR, TRUTH_COLOR, CoordinateTransform, DebugGUI
from rf_sim.simulator import Simulator


def test_physical_coordinates_to_pixels():
    transform = CoordinateTransform(40, 60, 100, 2)
    assert transform.to_screen(0, 0) == (40, 260)
    assert transform.to_screen(2, 2) == (240, 60)
    assert transform.to_screen(1, 1) == (140, 160)


def test_gui_controls_and_distinct_markers(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    sim = Simulator(AppConfig(), "manual", epoch=100)
    gui = DebugGUI(sim)
    try:
        assert gui.transform.height_m == 2
        assert gui.transform.scale == 290
        assert gui.transform.to_screen(0, 0) == (40, 640)
        assert gui.transform.to_screen(2, 2) == (620, 60)
        assert gui.transform.to_screen(1, 1) == (330, 350)
        keys = defaultdict(int, {pygame.K_w: 1, pygame.K_a: 1})
        monkeypatch.setattr(pygame.key, "get_pressed", lambda: keys)
        gui.handle_input(sim)
        for _ in range(10):
            gui.observe(sim, sim.step())
        assert sim.vehicles[0].state.speed > 0
        assert sim.vehicles[0].state.heading > 0
        gui.render(sim)
        pixels = pygame.surfarray.array3d(gui.screen)
        assert np.any(np.all(pixels == TRUTH_COLOR, axis=2))
        assert np.any(np.all(pixels == MEASUREMENT_COLOR, axis=2))
        keys[pygame.K_SPACE] = 1
        gui.handle_input(sim)
        sim.step()
        assert sim.vehicles[0].state.speed == 0
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p))
        gui.handle_input(sim)
        assert gui.paused
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_r))
        gui.handle_input(sim)
        assert sim.vehicles[0].state.x == 0.5
        assert sim.tick == 11
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE))
        gui.handle_input(sim)
        assert not gui.running
    finally:
        gui.close()


@pytest.mark.parametrize(
    "pressed,expected",
    [
        ((pygame.K_w, pygame.K_a), ((1, 1), (0, 0))),
        ((pygame.K_UP, pygame.K_LEFT), ((0, 0), (1, 1))),
        ((pygame.K_s, pygame.K_d, pygame.K_DOWN, pygame.K_RIGHT), ((-1, -1), (-1, -1))),
        ((pygame.K_w, pygame.K_a, pygame.K_UP, pygame.K_LEFT), ((1, 1), (1, 1))),
    ],
)
def test_two_manual_keyboard_inputs(monkeypatch, pressed, expected):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    sim = Simulator(AppConfig(), "two_targets_manual", epoch=100)
    gui = DebugGUI(sim)
    try:
        keys = defaultdict(int, {key: 1 for key in pressed})
        monkeypatch.setattr(pygame.key, "get_pressed", lambda: keys)
        assert gui.key_mappings[0] != gui.key_mappings[1]
        gui.handle_input(sim)
        assert [(c.throttle, c.steering) for c in sim.controllers] == list(expected)
        for _ in range(10):
            gui.observe(sim, sim.step())
        for vehicle, (throttle, steering) in zip(sim.vehicles, expected, strict=True):
            assert vehicle.state.speed * throttle > 0 if throttle else vehicle.state.speed == 0
            assert (
                vehicle.state.steering_angle * steering > 0
                if steering
                else vehicle.state.steering_angle == 0
            )
        if expected == ((1, 1), (1, 1)):
            keys[pygame.K_w] = keys[pygame.K_a] = 0
            gui.handle_input(sim)
            assert [(c.throttle, c.steering) for c in sim.controllers] == [(0, 0), (1, 1)]
        texts = []
        monkeypatch.setattr(gui, "_text", lambda text, *args: texts.append(text))
        gui.render(sim)
        assert "Target 1: detected" in texts and "Target 2: detected" in texts
        assert "RADAR (3.0 m overhead)" in texts
        assert "X: 0–2 m | Y: 0–2 m" in texts
        keys[pygame.K_SPACE] = 1
        gui.handle_input(sim)
        sim.step()
        assert all(v.state.speed == 0 for v in sim.vehicles)
        keys.clear()
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_r))
        gui.handle_input(sim)
        assert [(v.state.x, v.state.y, v.state.heading) for v in sim.vehicles] == [
            (0.5, 0.5, 0),
            (1.5, 1.5, np.pi),
        ]
        assert all(c.throttle == c.steering == 0 for c in sim.controllers)
        for paused in (True, False):
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p))
            gui.handle_input(sim)
            assert gui.paused == paused
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE))
        gui.handle_input(sim)
        assert not gui.running
    finally:
        gui.close()
