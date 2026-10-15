"""Generate processed TI UART captures from simulation or measurement JSONL."""

import argparse
import json
import math
from collections.abc import Iterable, Iterator
from itertools import islice
from pathlib import Path

from rf_common.frame import RadarFrame
from rf_common.ti_encoder import SensorPose, TIFrameEncoder, write_capture
from rf_sim.config import AppConfig, load_config
from rf_sim.scenarios import SCENARIOS
from rf_sim.simulator import Simulator


def simulated_frames(config: AppConfig, scenario: str) -> Iterator[RadarFrame]:
    simulator = Simulator(config, scenario, epoch=0.0)
    while True:
        yield from simulator.step()


def recorded_frames(path: Path) -> Iterator[RadarFrame]:
    """Read common JSONL and existing schema-1.0 simulator recordings."""
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise TypeError("frame must be an object")
                schema = value.pop("schema_version", "1.0")
                if schema != "1.0":
                    raise ValueError("unsupported recording schema")
                yield RadarFrame.model_validate_json(json.dumps(value))
            except (TypeError, ValueError) as error:
                raise ValueError(f"{path}: line {number}: invalid RadarFrame: {error}") from error


def limit_seconds(frames: Iterable[RadarFrame], seconds: float) -> Iterator[RadarFrame]:
    first = None
    for frame in frames:
        value = frame.simulation_time if frame.simulation_time is not None else frame.timestamp
        if first is None:
            first = value
        if value - first >= seconds:
            return
        yield frame


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--input", type=Path, help="Recorded measurement JSONL instead of simulation"
    )
    parser.add_argument("--config", type=Path)
    parser.add_argument("--scenario", choices=SCENARIOS, default="two_targets")
    parser.add_argument("--seed", type=int, default=42)
    limit = parser.add_mutually_exclusive_group(required=True)
    limit.add_argument("--frames", type=int)
    limit.add_argument("--seconds", type=float, help="Acquisition interval from the first frame")
    args = parser.parse_args()
    if args.frames is not None and args.frames <= 0:
        parser.error("--frames must be positive")
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    if args.seed < 0:
        parser.error("--seed must be nonnegative")
    try:
        config = load_config(args.config)
        config.simulation.random_seed = args.seed
        frames = (
            recorded_frames(args.input) if args.input else simulated_frames(config, args.scenario)
        )
        selected = (
            islice(frames, args.frames) if args.frames else limit_seconds(frames, args.seconds)
        )
        pose = SensorPose(
            x_m=config.radar.sensor_x_m,
            y_m=config.radar.sensor_y_m,
            height_m=config.radar.sensor_height_m,
        )
        count = write_capture(args.output, selected, TIFrameEncoder(sensor_pose=pose))
        print(
            f"Wrote {count} processed TI frames to {args.output} ({args.output.stat().st_size} bytes)"
        )
    except (OSError, ValueError) as error:
        parser.exit(2, f"Error: {error}\n")


if __name__ == "__main__":
    main()
