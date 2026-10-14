"""Standalone receiver; run after editable installation, no PYTHONPATH needed."""

import argparse
import logging
import socket

from pydantic import ValidationError

from rf_sim.models import TargetFrame

logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate and display radar UDP frames")
    parser.add_argument("--host", default="127.0.0.1", help="Local bind address")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--count", type=int, help="Exit after this many valid frames")
    parser.add_argument("--timeout", type=float, help="Receive timeout in seconds")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind((args.host, args.port))
        receiver.settimeout(args.timeout)
        logger.info("Listening on %s:%s", args.host, args.port)
        count = 0
        try:
            while args.count is None or count < args.count:
                payload, _ = receiver.recvfrom(65535)
                try:
                    frame = TargetFrame.model_validate_json(payload)
                except ValidationError as error:
                    logger.warning("Invalid frame: %s", error)
                    continue
                count += 1
                logger.info(
                    "Frame %d timestamp=%.3f sim=%.3f targets=%d",
                    frame.frame_id,
                    frame.timestamp,
                    frame.simulation_time,
                    len(frame.targets),
                )
                for target in frame.targets:
                    logger.info(
                        "  ID %d xy=(%.3f, %.3f) v=(%.3f, %.3f) range=%.3f "
                        "azimuth=%.3f rad confidence=%.3f SNR=%.2f dB",
                        target.target_id,
                        target.x,
                        target.y,
                        target.vx,
                        target.vy,
                        target.range_m,
                        target.azimuth_rad,
                        target.confidence,
                        target.snr_db,
                    )
        except (KeyboardInterrupt, TimeoutError):
            logger.info("Receiver stopped after %d valid frames", count)


if __name__ == "__main__":
    main()
