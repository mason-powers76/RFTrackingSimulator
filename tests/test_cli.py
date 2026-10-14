"""Exercise installed module entry points and the standalone receiver as processes."""

import os
import socket
import subprocess
import sys
from pathlib import Path

import pytest

from rf_sim import main as entrypoint
from rf_sim.models import TargetFrame
from rf_sim.replay import read_frames
from rf_sim.simulator import Simulator

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("config_args", [[], ["--config", str(ROOT / "config/default.yaml")]])
def test_cli_geometry_reaches_simulator(monkeypatch, config_args):
    observed = []

    def capture(config, scenario, use_gui, duration, recording):
        sim = Simulator(config, scenario, epoch=0)
        observed.append(sim)
        assert use_gui
        assert (sim.config.room.width_m, sim.config.room.height_m) == (2, 2)
        assert (sim.config.radar.sensor_x_m, sim.config.radar.sensor_y_m) == (1, 1)
        assert sim.config.radar.sensor_height_m == 3
        assert [(v.state.x, v.state.y) for v in sim.vehicles] == [(0.5, 0.5), (1.5, 1.5)]

    monkeypatch.setattr(entrypoint, "run", capture)
    monkeypatch.setattr(
        sys, "argv", ["rf_sim", "--scenario", "two_targets_manual", "--gui", *config_args]
    )
    entrypoint.main()
    assert len(observed) == 1


def command(*arguments: str, cwd: Path, gui: bool = False) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    if gui:
        environment["SDL_VIDEODRIVER"] = "dummy"
        environment["SDL_AUDIODRIVER"] = "dummy"
    return subprocess.run(
        [sys.executable, *arguments],
        cwd=cwd,
        env=environment,
        capture_output=True,
        text=True,
        check=True,
        timeout=30,
    )


def test_cli_record_receiver_and_replay(tmp_path):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        port = str(sock.getsockname()[1])
    with subprocess.Popen(
        [
            sys.executable,
            str(ROOT / "tools" / "udp_receiver.py"),
            "--port",
            port,
            "--count",
            "3",
            "--timeout",
            "10",
        ],
        cwd=tmp_path,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    ) as receiver:
        assert "Listening" in receiver.stdout.readline()
        result = command(
            "-m",
            "rf_sim.main",
            "--scenario",
            "rectangle",
            "--udp",
            "--udp-port",
            port,
            "--duration",
            "0.5",
            "--seed",
            "42",
            "--record",
            "cli",
            cwd=tmp_path,
        )
        output, _ = receiver.communicate(timeout=15)
        assert receiver.returncode == 0
        assert "confidence=" in output and "SNR=" in output
        assert "Finished" in result.stderr
    path = tmp_path / "recordings" / "cli_measurements.jsonl"
    frames = list(read_frames(path))
    assert frames
    result = command(
        "-m", "rf_sim.replay", str(path), "--speed", "2", "--udp-port", port, cwd=tmp_path
    )
    assert f"Replayed {len(frames)} frames" in result.stderr
    command(
        "-m",
        "rf_sim.main",
        "--scenario",
        "stationary",
        "--duration",
        "0.2",
        "--no-udp",
        cwd=tmp_path,
    )
    command(
        "-m",
        "rf_sim.main",
        "--scenario",
        "manual",
        "--gui",
        "--duration",
        "0.2",
        "--no-udp",
        cwd=tmp_path,
        gui=True,
    )


def test_cli_two_targets(tmp_path):
    command(
        "-m",
        "rf_sim.main",
        "--scenario",
        "two_targets",
        "--duration",
        "0.2",
        "--no-udp",
        "--record",
        "two",
        cwd=tmp_path,
    )
    frames = list(read_frames(tmp_path / "recordings" / "two_measurements.jsonl"))
    assert any({target.target_id for target in frame.targets} == {1, 2} for frame in frames)


def test_cli_two_targets_manual_udp(tmp_path):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        sock.settimeout(2)
        command(
            "-m",
            "rf_sim.main",
            "--scenario",
            "two_targets_manual",
            "--gui",
            "--udp",
            "--udp-port",
            str(sock.getsockname()[1]),
            "--seed",
            "42",
            "--duration",
            "0.2",
            "--record",
            "manual_two",
            cwd=tmp_path,
            gui=True,
        )
        frame = TargetFrame.model_validate_json(sock.recv(65535))
        assert {target.target_id for target in frame.targets} == {1, 2}
    frames = list(read_frames(tmp_path / "recordings" / "manual_two_measurements.jsonl"))
    assert any({target.target_id for target in frame.targets} == {1, 2} for frame in frames)


def test_cli_two_targets_manual_requires_gui(tmp_path):
    result = subprocess.run(
        [sys.executable, "-m", "rf_sim.main", "--scenario", "two_targets_manual"],
        check=False,
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 2
    assert "two_targets_manual scenario requires --gui" in result.stderr
