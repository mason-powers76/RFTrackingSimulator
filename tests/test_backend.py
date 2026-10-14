import json
import os
import socket
import subprocess
import sys
import threading
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from rf_backend.adapter import parse_packet
from rf_backend.api import create_server
from rf_backend.models import Detection, RadarFrame
from rf_backend.store import RadarStore
from rf_backend.udp import UDPReceiver
from rf_sim.config import AppConfig
from rf_sim.simulator import Simulator
from rf_sim.transport import serialize_frame


def frame(number=0, timestamp=100.0, ids=(1, 2)):
    return RadarFrame(
        source="test",
        frame_id=number,
        timestamp=timestamp,
        targets=tuple(Detection(target_id=i, x=float(i), y=2.0) for i in ids),
    )


@pytest.fixture
def store(tmp_path):
    database = RadarStore(tmp_path / "radar.sqlite3", "run-1")
    yield database
    database.close()


def test_wire_adapter_exact_fields():
    simulator = Simulator(
        AppConfig(radar={"latency_ms": 0, "detection_probability": 1}),
        "two_targets_manual",
        epoch=100,
    )
    wire = simulator.step()[0]
    parsed = parse_packet(serialize_frame(wire))
    assert {t.target_id for t in parsed.targets} == {1, 2}
    assert parsed.timestamp == wire.timestamp
    assert parsed.simulation_time == wire.simulation_time
    assert parsed.targets[0].model_dump() == wire.targets[0].model_dump()


@pytest.mark.parametrize(
    "change",
    [
        {"timestamp": -1},
        {"timestamp": float("nan")},
        {"frame_id": "1"},
        {"schema_version": "2.0"},
        {"truth": []},
        {"simulation_time": None},
        {"source": ""},
        {"frame_id": 2**63},
    ],
)
def test_invalid_wire(change):
    values = {"frame_id": 1, "timestamp": 100, "simulation_time": 0, "targets": []}
    with pytest.raises(ValueError):
        parse_packet(json.dumps(values | change).encode())


@pytest.mark.parametrize("payload", [b"garbage", b"\xff", b"[]"])
def test_invalid_json(payload):
    with pytest.raises(ValueError):
        parse_packet(payload)


def test_duplicate_target_ids():
    with pytest.raises(ValueError, match="unique"):
        frame(ids=(1, 1))


def test_history_latest_missing_order_and_duplicate(store):
    first = frame()
    assert store.latest() is None
    assert store.ingest(first, received_at=200)
    assert not store.ingest(first, received_at=201)
    assert store.ingest(frame(3, 103.0, ()), received_at=204)
    assert store.ingest(frame(1, 101.0, (2,)), received_at=205)
    assert store.latest()["frame"]["targets"] == []
    assert store.latest()["frame"]["frame_id"] == 3
    assert [f["frame"]["frame_id"] for f in store.history(100, 103)] == [0, 1, 3]
    assert len(store.history(100, 103, limit=1)) == 1
    assert store.last_seen(1)["timestamp"] == 100
    assert store.last_seen(2)["timestamp"] == 101
    assert not store.last_seen(2)["detected_in_latest_frame"]
    assert store.last_seen(2, at=100.5)["timestamp"] == 100
    assert store.last_seen(2, at=99) is None
    assert store.last_seen(999) is None
    with pytest.raises(ValueError, match="conflicting"):
        store.ingest(frame(0, 105.0))
    assert len(store.history(0, 1000)) == 3
    with pytest.raises(ValueError):
        store.history(103, 100)
    with pytest.raises(ValueError):
        store.history(0, float("inf"))
    with pytest.raises(ValueError):
        store.history(0, 100, limit=0)


def test_restart_stream_isolation_and_optional_fields(tmp_path):
    path = tmp_path / "history.sqlite3"
    database = RadarStore(path, "a")
    database.ingest(frame())
    database.close()
    database = RadarStore(path, "a")
    assert database.latest()["frame"]["targets"][0]["snr_db"] is None
    assert not database.ingest(frame())
    database.close()
    database = RadarStore(path, "b")
    assert database.latest() is None
    database.ingest(frame(0, 200.0))
    assert database.connection.execute("SELECT count(*) FROM frames").fetchone()[0] == 2
    database.close()


def test_udp_invalid_then_valid(store):
    receiver = UDPReceiver(store, port=0)
    try:
        destination = receiver.socket.getsockname()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b"bad", destination)
            assert not receiver.receive_once()
            simulator = Simulator(AppConfig(), "two_targets", epoch=100)
            wire = next(f for _ in range(10) for f in simulator.step())
            sender.sendto(serialize_frame(wire), destination)
            assert receiver.receive_once()
            assert store.latest()["frame"]["timestamp"] == wire.timestamp
            assert len(store.latest()["frame"]["targets"]) == 2
    finally:
        receiver.close()


def test_http_queries(store):
    server = create_server(store, "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with pytest.raises(HTTPError) as error:
            urlopen(base + "/api/latest", timeout=2)
        assert error.value.code == 404
        store.ingest(frame())
        for endpoint in [
            "/api/latest",
            "/api/targets/1",
            "/api/targets/1?at=100",
            "/api/frames?start=99&end=101",
        ]:
            with urlopen(base + endpoint, timeout=2) as response:
                assert json.load(response)
        for endpoint in [
            "/api/frames",
            "/api/frames?start=nan&end=101",
            "/api/targets/-1",
            "/api/targets/1?at=inf",
        ]:
            with pytest.raises(HTTPError) as error:
                urlopen(base + endpoint, timeout=2)
            assert error.value.code == 400
    finally:
        server.shutdown()
        thread.join()
        server.server_close()


def test_existing_simulator_process_to_database(store, tmp_path):
    receiver = UDPReceiver(store, port=0)
    port = receiver.socket.getsockname()[1]
    try:
        with subprocess.Popen(
            [
                sys.executable,
                "-m",
                "rf_sim.main",
                "--scenario",
                "two_targets",
                "--udp",
                "--udp-port",
                str(port),
                "--duration",
                "0.5",
                "--seed",
                "42",
            ],
            cwd=tmp_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        ) as process:
            while process.poll() is None:
                receiver.receive_once()
            output, errors = process.communicate(timeout=10)
            assert process.returncode == 0, output + errors
        assert len(store.history(0, 1e12)) >= 5
        assert {t["target_id"] for t in store.latest()["frame"]["targets"]} == {1, 2}
    finally:
        receiver.close()


def test_backend_cli_manual_gui_and_replay(tmp_path):
    ports = []
    sockets = []
    for kind in (socket.SOCK_DGRAM, socket.SOCK_STREAM):
        sock = socket.socket(socket.AF_INET, kind)
        sock.bind(("127.0.0.1", 0))
        ports.append(str(sock.getsockname()[1]))
        sockets.append(sock)
    for sock in sockets:
        sock.close()
    path = tmp_path / "radar.sqlite3"
    environment = os.environ.copy()
    environment["SDL_VIDEODRIVER"] = "dummy"
    environment["SDL_AUDIODRIVER"] = "dummy"
    with subprocess.Popen(
        [
            sys.executable,
            "-m",
            "rf_backend.main",
            "--stream-id",
            "manual-test",
            "--db",
            str(path),
            "--udp-port",
            ports[0],
            "--http-port",
            ports[1],
        ],
        cwd=tmp_path,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    ) as backend:
        try:
            assert "Listening UDP" in backend.stdout.readline()
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "rf_sim.main",
                    "--scenario",
                    "two_targets_manual",
                    "--gui",
                    "--udp",
                    "--udp-port",
                    ports[0],
                    "--duration",
                    "0.5",
                    "--seed",
                    "42",
                    "--record",
                    "manual",
                ],
                cwd=tmp_path,
                env=environment,
                check=True,
                capture_output=True,
                timeout=15,
            )
            with urlopen(f"http://127.0.0.1:{ports[1]}/api/latest", timeout=2) as response:
                latest = json.load(response)
            assert {t["target_id"] for t in latest["frame"]["targets"]} == {1, 2}
            recording = Path(tmp_path / "recordings/manual_measurements.jsonl")
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "rf_sim.replay",
                    str(recording),
                    "--udp-port",
                    ports[0],
                    "--no-realtime",
                ],
                cwd=tmp_path,
                check=True,
                capture_output=True,
                timeout=15,
            )
        finally:
            backend.terminate()
            backend.communicate(timeout=10)
    database = RadarStore(path, "manual-test")
    try:
        assert database.latest()["frame"] == latest["frame"]
        assert len(database.history(0, 1e12)) >= 5
    finally:
        database.close()
