"""One validated measurement frame per UDP datagram."""

import socket
from types import TracebackType
from typing import Self

from rf_common.frame import RadarFrame
from rf_sim.models import TargetFrame

MAX_UDP_PAYLOAD = 65507


def serialize_frame(frame: TargetFrame | RadarFrame) -> bytes:
    # Validate again at the external boundary (including model_construct misuse).
    validated = TargetFrame.model_validate(frame.model_dump())
    payload = validated.model_dump_json().encode("utf-8")
    if len(payload) > MAX_UDP_PAYLOAD:
        raise ValueError("TargetFrame exceeds the IPv4 UDP payload limit")
    return payload


class UDPPublisher:
    def __init__(self, host: str = "127.0.0.1", port: int = 5005) -> None:
        self.destination = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, frame: TargetFrame) -> None:
        self.socket.sendto(serialize_frame(frame), self.destination)

    def close(self) -> None:
        self.socket.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
