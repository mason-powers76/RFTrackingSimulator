import json
import socket

from rf_sim.models import TargetFrame
from rf_sim.transport import UDPPublisher, serialize_frame


def test_serialization_and_actual_loopback():
    frame = TargetFrame(frame_id=4, timestamp=100, simulation_time=2)
    payload = serialize_frame(frame)
    assert json.loads(payload)["frame_id"] == 4
    assert TargetFrame.model_validate_json(payload) == frame
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2)
        with UDPPublisher(port=receiver.getsockname()[1]) as publisher:
            publisher.publish(frame)
            received, _ = receiver.recvfrom(65535)
        assert received == payload
