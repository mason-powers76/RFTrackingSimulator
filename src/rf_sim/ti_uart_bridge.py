"""Host software UART emulator for TI xWR6843 3D People Tracking."""

import argparse
import logging
import math
import time
from contextlib import ExitStack
from pathlib import Path

from rf_common.ti_encoder import SensorPose, TIFrameEncoder
from rf_common.ti_uart import CLI_BAUD, DATA_BAUD, TIDataUART, TIUARTSession, open_serial
from rf_sim.config import AppConfig, load_config
from rf_sim.scenarios import SCENARIOS
from rf_sim.simulator import Simulator

log = logging.getLogger(__name__)


def run(
    config: AppConfig,
    scenario: str,
    cli_port: str,
    data_port: str,
    *,
    cli_baud: int = CLI_BAUD,
    data_baud: int = DATA_BAUD,
    use_gui: bool = True,
    duration: float | None = None,
    serial_factory=open_serial,
    clock=time.monotonic,
    sleep=time.sleep,
) -> int:
    """One Simulator and its existing manual controls; bounded synchronous I/O.

    Physics and controls continue while the sensor is stopped. Backpressure
    slows simulation; missed wall time is not accumulated into catch-up bursts.
    Duration, when supplied, counts monotonic wall time, including stopped time.
    """
    if cli_port.casefold() == data_port.casefold():
        raise ValueError("CLI and DATA must use different endpoints of different pairs")
    if cli_baud <= 0 or data_baud <= 0:
        raise ValueError("baud rates must be positive")
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise ValueError("duration must be finite and positive")
    simulator = Simulator(config, scenario)
    config = simulator.config
    with ExitStack() as stack:
        cli = serial_factory(cli_port, cli_baud)
        stack.callback(cli.close)
        raw_data = serial_factory(data_port, data_baud)
        stack.callback(raw_data.close)
        radar = config.radar
        data = TIDataUART(
            raw_data,
            radar.measurement_rate_hz,
            encoder=TIFrameEncoder(
                sensor_pose=SensorPose(
                    x_m=radar.sensor_x_m, y_m=radar.sensor_y_m, height_m=radar.sensor_height_m
                )
            ),
            clock=clock,
            sleep=sleep,
        )
        session = TIUARTSession(cli, data)
        gui = None
        if use_gui:
            from rf_sim.gui import DebugGUI

            gui = DebugGUI(simulator)
            stack.callback(gui.close)
        log.info(
            "TI UART emulator (HOST SOFTWARE EMULATION ONLY)\nScenario: %s\n"
            "CLI endpoint: %s @ %d\nDATA endpoint: %s @ %d\nRadar rate: %g Hz\n"
            "Sensor state: STOPPED",
            scenario,
            cli_port,
            cli_baud,
            data_port,
            data_baud,
            radar.measurement_rate_hz,
        )
        started = clock()
        next_tick = started
        next_report = started + 5
        try:
            while duration is None or clock() - started < duration:
                session.poll_cli()
                if gui:
                    gui.handle_input(simulator)
                    if not gui.running:
                        break
                now = clock()
                if now >= next_tick:
                    if not gui or not gui.paused:
                        frames = simulator.step()
                        for frame in frames:
                            session.publish(frame)
                        if gui:
                            gui.observe(simulator, frames)
                    next_tick = max(next_tick + simulator.dt, clock())
                if gui and now - gui.last_render >= 1 / config.simulation.gui_rate_hz:
                    gui.render(simulator)
                    gui.last_render = now
                if now >= next_report:
                    log.info("DATA frames transmitted: %d", data.frame_count)
                    next_report = now + 5
                sleep(0.001)
        except KeyboardInterrupt:
            log.info("Stopped by user")
        log.info("UART emulator closed; DATA frames transmitted: %d", data.frame_count)
        return data.frame_count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli-port", required=True)
    parser.add_argument("--data-port", required=True)
    parser.add_argument("--cli-baud", type=int, default=CLI_BAUD)
    parser.add_argument("--data-baud", type=int, default=DATA_BAUD)
    parser.add_argument("--scenario", choices=SCENARIOS, default="two_targets_manual")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--gui", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--duration", type=float, help="Optional wall-clock seconds")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    try:
        config = load_config(args.config)
        config.simulation.random_seed = args.seed
        manual = args.scenario in ("manual", "two_targets_manual")
        use_gui = manual if args.gui is None else args.gui
        if manual and not use_gui:
            raise ValueError("manual scenarios require the GUI for keyboard controls")
        run(
            config,
            args.scenario,
            args.cli_port,
            args.data_port,
            cli_baud=args.cli_baud,
            data_baud=args.data_baud,
            use_gui=use_gui,
            duration=args.duration,
        )
    except KeyboardInterrupt:
        log.info("Stopped by user during startup")
    except (OSError, ValueError, TypeError) as error:
        log.error("UART emulator stopped: %s", error)
        parser.exit(2)


if __name__ == "__main__":
    main()
