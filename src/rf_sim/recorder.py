"""Separate JSONL streams for downstream measurements and validation truth."""

import json
import re
from dataclasses import asdict
from pathlib import Path
from types import TracebackType
from typing import Self

from rf_sim.models import TargetFrame
from rf_sim.simulator import Simulator
from rf_sim.transport import serialize_frame


class Recorder:
    """Create new recordings without overwriting existing experiment data."""

    def __init__(self, name: str, directory: Path = Path("recordings")) -> None:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
            raise ValueError("recording name must contain only letters, digits, '_' or '-'")
        directory.mkdir(parents=True, exist_ok=True)
        self.measurement_path = directory / f"{name}_measurements.jsonl"
        self.truth_path = directory / f"{name}_ground_truth.jsonl"
        if self.measurement_path.exists() or self.truth_path.exists():
            raise FileExistsError(f"Recording {name!r} already exists; use a new name")
        self.measurements = self.measurement_path.open("x", encoding="utf-8")
        try:
            self.truth = self.truth_path.open("x", encoding="utf-8")
        except OSError:
            self.measurements.close()
            raise

    def record_frame(self, frame: TargetFrame) -> None:
        self.measurements.write(serialize_frame(frame).decode("utf-8") + "\n")
        self.measurements.flush()

    def record_truth(self, simulator: Simulator) -> None:
        payload = {
            "source": "simulator_ground_truth",
            "timestamp": simulator.epoch + simulator.simulation_time,
            "simulation_time": simulator.simulation_time,
            "targets": [asdict(vehicle.state) for vehicle in simulator.vehicles],
        }
        self.truth.write(json.dumps(payload, allow_nan=False) + "\n")
        self.truth.flush()

    def close(self) -> None:
        self.measurements.close()
        self.truth.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
