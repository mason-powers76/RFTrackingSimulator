"""Receive one datagram at a time; malformed packets never stop ingestion."""

import logging
import socket
from collections.abc import Callable

from rf_backend.adapter import parse_packet
from rf_backend.models import RadarFrame
from rf_backend.store import RadarStore

logger = logging.getLogger(__name__)


class UDPReceiver:
    def __init__(
        self,
        store: RadarStore,
        host: str = "127.0.0.1",
        port: int = 5005,
        parser: Callable[[bytes], RadarFrame] = parse_packet,
    ) -> None:
        self.store = store
        self.parser = parser
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((host, port))
            self.socket.settimeout(0.2)
        except OSError:
            self.socket.close()
            raise

    def receive_once(self) -> bool:
        try:
            payload, _ = self.socket.recvfrom(65535)
        except TimeoutError:
            return False
        try:
            return self.store.ingest(self.parser(payload))
        except ValueError as error:
            logger.warning("Rejected radar packet: %s", error)
            return False

    def close(self) -> None:
        self.socket.close()
