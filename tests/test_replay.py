import json
import socket

import pytest

from rf_sim.config import AppConfig
from rf_sim.recorder import Recorder
from rf_sim.replay import read_frames, replay_frames
from rf_sim.simulator import Simulator
from rf_sim.transport import UDPPublisher


def test_recording_separation_and_replay_udp(tmp_path):
    sim = Simulator(AppConfig(), "stationary", epoch=100)
    original = []
    with Recorder("test", tmp_path) as recorder:
        for _ in range(20):
            frames = sim.step()
            for frame in frames:
                recorder.record_frame(frame)
                original.append(frame)
            recorder.record_truth(sim)
    loaded = list(read_frames(tmp_path / "test_measurements.jsonl"))
    assert loaded == original
    truth = json.loads((tmp_path / "test_ground_truth.jsonl").read_text().splitlines()[0])
    assert truth["source"] == "simulator_ground_truth"
    assert "steering_angle" in truth["targets"][0]
    assert all("steering_angle" not in t.model_dump() for f in loaded for t in f.targets)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2)
        with UDPPublisher(port=receiver.getsockname()[1]) as publisher:
            replay_frames(loaded, publisher.publish, realtime=False)
        received = [receiver.recvfrom(65535)[0] for _ in loaded]
    assert received == [frame.model_dump_json().encode() for frame in loaded]
    with pytest.raises(FileExistsError):
        Recorder("test", tmp_path)


def test_replay_speed_without_real_sleep(tmp_path):
    sim = Simulator(AppConfig(), "stationary", epoch=100)
    frames = [frame for _ in range(20) for frame in sim.step()]
    now = [0.0]
    sent = []

    def sleep(seconds):
        now[0] += seconds

    replay_frames(
        frames,
        lambda frame: sent.append((now[0], frame)),
        speed=2,
        clock=lambda: now[0],
        sleep=sleep,
    )
    assert len(sent) == len(frames)
    assert sent[-1][0] == pytest.approx(
        (frames[-1].simulation_time - frames[0].simulation_time) / 2
    )
    with pytest.raises(ValueError):
        replay_frames(frames, lambda frame: None, speed=0)


def test_corrupt_recording(tmp_path):
    path = tmp_path / "bad.jsonl"
    path.write_text('{"ground_truth": []}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="line 1"):
        list(read_frames(path))
