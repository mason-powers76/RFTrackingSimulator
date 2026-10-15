"""Incremental processed-UART framing, without a UART transport."""

import struct

from rf_common.ti import MAGIC, TIPacket, decode_packet


class TIStreamDecoder:
    def __init__(self, max_packet_length: int = 1_048_576) -> None:
        if max_packet_length < 64:
            raise ValueError("maximum packet length must be at least 64")
        self.max_packet_length = max_packet_length
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[TIPacket]:
        """Skip preamble garbage; reject malformed complete frames strictly."""
        self.buffer.extend(data)
        packets = []
        while True:
            offset = self.buffer.find(MAGIC)
            if offset < 0:
                # Retain only a suffix that could be a split magic word.
                keep = 0
                for n in range(1, len(MAGIC)):
                    if self.buffer[-n:] == MAGIC[:n]:
                        keep = n
                if keep:
                    del self.buffer[:-keep]
                else:
                    self.buffer.clear()
                return packets
            del self.buffer[:offset]
            if len(self.buffer) < 40:
                return packets
            size = struct.unpack_from("<I", self.buffer, 12)[0]
            if size < 40 or size > self.max_packet_length:
                raise ValueError("invalid TI stream packet length")
            if len(self.buffer) < size:
                return packets
            packets.append(decode_packet(bytes(self.buffer[:size])))
            del self.buffer[:size]

    def finish(self) -> None:
        if self.buffer:
            raise ValueError("truncated TI stream frame")
