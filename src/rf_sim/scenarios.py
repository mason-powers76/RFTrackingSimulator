"""Predefined paths scaled to room dimensions; custom waypoints can override them."""

import math

from rf_sim.config import AppConfig, TargetConfig
from rf_sim.controllers import (
    Controller,
    ManualController,
    StationaryController,
    WaypointController,
)

SCENARIOS = (
    "straight_line",
    "rectangle",
    "circle",
    "figure_eight",
    "manual",
    "stationary",
    "two_targets",
    "two_targets_manual",
)


def scenario_config(name: str, config: AppConfig) -> AppConfig:
    """The two-car preset supplies targets while retaining other configuration settings."""
    if name not in ("two_targets", "two_targets_manual"):
        return config
    width, height = config.room.width_m, config.room.height_m
    radius = config.vehicle.radius_m
    x = max(width / (4 if name == "two_targets_manual" else 6), radius)
    y = max(height / 4, radius)
    values = config.model_dump()
    values["targets"] = [
        TargetConfig(target_id=1, name="RC car 1", initial_x=x, initial_y=y).model_dump(),
        TargetConfig(
            target_id=2,
            name="RC car 2",
            initial_x=width - x,
            initial_y=height - y,
            initial_heading=math.pi,
        ).model_dump(),
    ]
    if name == "two_targets_manual":
        values["targets"][0]["reflector_enabled"] = True
    return AppConfig.model_validate(values)


def make_controller(name: str, config: AppConfig, target_index: int = 0) -> Controller:
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {name}")
    if name in ("manual", "two_targets_manual"):
        return ManualController()
    if name == "stationary":
        return StationaryController()
    width, height = config.room.width_m, config.room.height_m
    loop = config.scenario.loop
    if config.scenario.waypoints is not None:
        points = config.scenario.waypoints
    elif name == "straight_line":
        points = [(width * 5 / 6, height / 4)]
        loop = False
    elif name in ("rectangle", "two_targets"):
        points = [
            (width / 6, height / 4),
            (width * 5 / 6, height / 4),
            (width * 5 / 6, height * 3 / 4),
            (width / 6, height * 3 / 4),
        ]
    elif name == "circle":
        points = [
            (
                width / 2 + min(width, height) * 0.3 * math.cos(i * math.tau / 48),
                height / 2 + min(width, height) * 0.3 * math.sin(i * math.tau / 48),
            )
            for i in range(48)
        ]
    else:
        points = [
            (
                width / 2 + width * 0.3 * math.sin(i * math.tau / 64),
                height / 2 + height * 0.25 * math.sin(2 * i * math.tau / 64),
            )
            for i in range(64)
        ]
    if name == "two_targets" and target_index == 1:
        offset = len(points) // 2
        points = points[offset:] + points[:offset]
    return WaypointController(
        points, config.scenario.cruise_speed_mps, config.scenario.waypoint_tolerance_m, loop
    )
