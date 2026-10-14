"""Command-line entry point for the target simulator."""

import argparse
import logging
import time
from contextlib import ExitStack
from pathlib import Path

from rf_sim.config import AppConfig, load_config
from rf_sim.recorder import Recorder
from rf_sim.scenarios import SCENARIOS
from rf_sim.simulator import Simulator
from rf_sim.transport import UDPPublisher

logger = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Target-level radar simulator (SI units)")
    parser.add_argument(
        "--scenario",
        default="rectangle",
        choices=SCENARIOS,
    )
    parser.add_argument("--config", help="Optional YAML configuration")
    parser.add_argument("--seed", type=int, help="Override deterministic radar seed")
    parser.add_argument("--gui", action="store_true", help="Open Pygame debug view")
    parser.add_argument(
        "--udp",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable/disable UDP (otherwise use config)",
    )
    parser.add_argument("--udp-host", help="UDP destination host")
    parser.add_argument("--udp-port", type=int, help="UDP destination port")
    parser.add_argument("--record", help="Recording name, e.g. test01")
    parser.add_argument("--duration", type=float, help="Stop after this many simulation seconds")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    try:
        config = load_config(Path(args.config) if args.config else None)
        values = config.model_dump()
        if args.seed is not None:
            values["simulation"]["random_seed"] = args.seed
        for key, value in [("enabled", args.udp), ("host", args.udp_host), ("port", args.udp_port)]:
            if value is not None:
                values["network"][key] = value
        config = AppConfig.model_validate(values)
        if args.duration is not None and not 0 < args.duration < float("inf"):
            raise ValueError("duration must be finite and positive")
        if args.scenario in ("manual", "two_targets_manual") and not args.gui:
            raise ValueError(f"{args.scenario} scenario requires --gui")
        run(config, args.scenario, args.gui, args.duration, args.record)
    except (OSError, ValueError, TypeError) as error:
        parser.exit(2, f"Error: {error}\n")


def run(
    config: AppConfig, scenario: str, use_gui: bool, duration: float | None, recording: str | None
) -> None:
    """Wall-clock accumulator; no fixed physics steps are discarded on slow renders."""
    simulator = Simulator(config, scenario)
    with ExitStack() as stack:
        recorder = stack.enter_context(Recorder(recording)) if recording else None
        publisher = (
            stack.enter_context(UDPPublisher(config.network.host, config.network.port))
            if config.network.enabled
            else None
        )
        gui = None
        if use_gui:
            from rf_sim.gui import DebugGUI

            gui = DebugGUI(simulator)
            stack.callback(gui.close)
        logger.info(
            "Scenario=%s seed=%d physics=%g Hz radar=%g Hz UDP=%s",
            scenario,
            config.simulation.random_seed,
            config.simulation.physics_rate_hz,
            config.radar.measurement_rate_hz,
            config.network.enabled,
        )
        previous = time.perf_counter()
        accumulator = 0.0
        try:
            while duration is None or simulator.simulation_time + 1e-10 < duration:
                now = time.perf_counter()
                elapsed, previous = now - previous, now
                if gui:
                    gui.handle_input(simulator)
                    if not gui.running:
                        break
                if not gui or not gui.paused:
                    accumulator += elapsed
                else:
                    accumulator = 0.0
                while accumulator + 1e-10 >= simulator.dt:
                    frames = simulator.step()
                    accumulator -= simulator.dt
                    for frame in frames:
                        if publisher:
                            publisher.publish(frame)
                        if recorder:
                            recorder.record_frame(frame)
                    if recorder:
                        recorder.record_truth(simulator)
                    if gui:
                        gui.observe(simulator, frames)
                    if duration is not None and simulator.simulation_time + 1e-10 >= duration:
                        break
                if gui and now - gui.last_render >= 1 / config.simulation.gui_rate_hz:
                    gui.render(simulator)
                    gui.last_render = now
                time.sleep(0.001)
        except KeyboardInterrupt:
            logger.info("Stopped by user")
        logger.info(
            "Finished at %.2f simulation seconds; acquired %d radar frames",
            simulator.simulation_time,
            simulator.source.next_frame_id,
        )


if __name__ == "__main__":
    main()
