"""In-memory UART byte/coordinate audit against unmodified local TI source.

No serial endpoints or GUI are opened. Run with python -B; imported TI source
is read only. This never advances Applications Visualizer or hardware status.
"""

import argparse
import hashlib
import importlib
import io
import json
import sys

import numpy as np
from validate_ti_reference import DEFAULT_COMMON, validate_reference

from rf_common.ti_encoder import TIFrameEncoder
from rf_common.ti_uart import TIDataUART, TIUARTSession
from rf_sim.config import AppConfig
from rf_sim.simulator import Simulator


class MemoryPort(io.BytesIO):
    """Write-only serial test double; never calls pyserial."""

    @property
    def is_open(self):
        return not self.closed


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--common-dir", type=type(DEFAULT_COMMON), default=DEFAULT_COMMON)
    args = parser.parse_args()
    sys.dont_write_bytecode = True
    config = AppConfig()
    sim = Simulator(config, "two_targets_manual", epoch=1700000000)
    frames = []
    while len(frames) < 4:
        frames.extend(sim.step())
    clock, port = Clock(), MemoryPort()
    transport = TIDataUART(port, 20, clock=clock, sleep=clock.sleep)
    session = TIUARTSession(MemoryPort(), transport)
    assert session.cli.feed(b"sensorStop\n") == [b"sensorStop\r\nDone\r\n"]
    assert not session.publish(frames[0]) and port.getvalue() == b""
    session.cli.feed(b"sensorStart\n")
    direct = TIFrameEncoder()
    expected = []
    reports = []
    for frame in frames:
        expected.append(direct.encode(frame))
        assert session.publish(frame)
        reports.append(validate_reference(expected[-1], args.common_dir))
    captured = port.getvalue()
    assert captured == b"".join(expected)
    session.cli.feed(b"sensorStop\n")
    assert not session.publish(frames[-1]) and port.getvalue() == captured

    # Use TI's actual rotation implementation, not an independently guessed sign.
    graph = importlib.import_module("graph_utilities")
    display = []
    for x, y in ((1, 1), (0.5, 0.5), (1.5, 1.5)):
        native = (x - 1, 3, y - 1)
        rx, ry, rz = graph.eulerRot(*native, 90, 0)
        shown = (float(rx), float(ry), float(rz) + 3)
        np.testing.assert_allclose(shown, (x - 1, y - 1, 0), atol=1e-12)
        display.append({"world_floor_xy": [x, y], "native_xyz": native, "display_xyz": shown})
    transport.close()
    print(
        json.dumps(
            {
                "serial_test_kind": "in_memory_only",
                "frames": len(frames),
                "captured_bytes": len(captured),
                "byte_for_byte_identical": True,
                "sha256": hashlib.sha256(captured).hexdigest(),
                "paced_start_span_seconds": clock.now,
                "ti_reference_packet_checks": len(reports),
                "coordinate_checks_using_ti_eulerRot": display,
                "synthetic_only": True,
                "published_spec_verified": True,
                "ti_reference_parser_verified": True,
                "applications_visualizer_verified": False,
                "hardware_capture_verified": False,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
