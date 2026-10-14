"""Replay measurement JSONL through exactly the live TargetFrame/UDP interface."""

import argparse
import logging
import math
import time
from collections.abc import Callable, Iterable, Iterator
from pathlib import Path

from pydantic import ValidationError

from rf_sim.models import TargetFrame
from rf_sim.transport import UDPPublisher

logger = logging.getLogger(__name__)


def read_frames(path: Path) -> Iterator[TargetFrame]:
    """Stream validated frames; reject corrupt or out-of-order recordings."""
    previous_time = -1.0
    with path.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                frame = TargetFrame.model_validate_json(line)
            except ValidationError as error:
                raise ValueError(f"{path}: line {number}: invalid TargetFrame") from error
            frame_time = frame.timestamp if frame.simulation_time is None else frame.simulation_time
            if frame_time < previous_time:
                raise ValueError(f"{path}: line {number}: simulation_time moved backwards")
            previous_time = frame_time
            yield frame


def replay_frames(
    frames: Iterable[TargetFrame],
    publish: Callable[[TargetFrame], None],
    speed: float = 1.0,
    realtime: bool = True,
    clock: Callable[[], float] = time.perf_counter,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Use absolute deadlines to avoid cumulative drift; preserve recorded payloads."""
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("speed must be finite and positive")
    start = clock()
    first_time = None
    previous_time = -1.0
    count = 0
    for frame in frames:
        frame_time = frame.timestamp if frame.simulation_time is None else frame.simulation_time
        if frame_time < previous_time:
            raise ValueError("replay frames must be in acquisition order")
        previous_time = frame_time
        if first_time is None:
            first_time = frame_time
        if realtime:
            deadline = start + (frame_time - first_time) / speed
            while (remaining := deadline - clock()) > 0:
                sleep(min(remaining, 0.05))
        publish(frame)
        count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded TargetFrame JSONL over UDP")
    parser.add_argument("path", type=Path, help="Measurement JSONL (truth file not required)")
    parser.add_argument("--speed", type=float, default=1.0, help="Timing multiplier, e.g. 2.0")
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--no-realtime", action="store_true", help="Send as fast as possible")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    if not 1 <= args.udp_port <= 65535:
        parser.error("UDP port must be between 1 and 65535")
    try:
        with UDPPublisher(args.udp_host, args.udp_port) as publisher:
            count = replay_frames(
                read_frames(args.path), publisher.publish, args.speed, not args.no_realtime
            )
        logger.info("Replayed %d frames", count)
    except KeyboardInterrupt:
        logger.info("Replay stopped")
    except (OSError, ValueError) as error:
        parser.exit(2, f"Error: {error}\n")


if __name__ == "__main__":
    main()
