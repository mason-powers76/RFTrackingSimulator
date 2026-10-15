"""Bounded synchronous TI UART I/O; independent of physics and Pygame.

No application frame queue: publish blocks for pacing and a bounded write.
Serial failures are fatal because continuing a partially written packet would
leave the receiver with a corrupt stream. Never flush() on shutdown (can hang).
"""

import logging
import math
import time
from collections.abc import Callable
from typing import Protocol

from rf_common.frame import RadarFrame
from rf_common.ti_encoder import TIFrameEncoder

log = logging.getLogger(__name__)
CLI_BAUD = 115200
DATA_BAUD = 921600


class UARTError(OSError):
    """Actionable serial connection or I/O error."""


class SerialPort(Protocol):
    is_open: bool
    in_waiting: int

    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def close(self) -> None: ...


def open_serial(port: str, baud: int, *, read_timeout: float = 0) -> SerialPort:
    """Open arbitrary port names with explicit 8N1 and bounded I/O."""
    try:
        import serial
    except ImportError as error:
        raise UARTError("pyserial is required; install the project requirements") from error
    try:
        return serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=read_timeout,
            write_timeout=0.1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except (OSError, ValueError) as error:
        raise UARTError(
            f"Cannot open {port} @ {baud}: {error}. Check the port exists and is not "
            "already in use; assign opposite endpoints of each virtual pair."
        ) from error


def write_all(
    port: SerialPort,
    data: bytes,
    *,
    clock: Callable[[], float] = time.monotonic,
    timeout: float = 0.25,
) -> None:
    """Handle short writes; the injected port must also bound each write call."""
    end = clock() + timeout
    offset = 0
    try:
        while offset < len(data):
            if not port.is_open:
                raise UARTError("Serial port disconnected or closed")
            if clock() >= end:
                raise UARTError("Serial write deadline exceeded")
            written = port.write(data[offset:])
            if written is None or not 0 < written <= len(data) - offset:
                raise UARTError("Serial write made no progress or returned an invalid count")
            offset += written
    except OSError as error:
        raise UARTError(f"Serial write failed: {error}") from error


class TIDataUART:
    """Common frame -> existing encoder -> monotonic pacing -> exact bytes.

    One packet in flight, at most max_packet_bytes. Slow writers block the
    producer, preserve order and slow simulation time. No catch-up bursts.
    A supplied port must use finite write timeouts, as open_serial does.
    """

    def __init__(
        self,
        port: SerialPort,
        rate_hz: float,
        *,
        encoder: TIFrameEncoder | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        max_packet_bytes: int = 1024 * 1024,
    ) -> None:
        if not math.isfinite(rate_hz) or rate_hz <= 0 or max_packet_bytes <= 0:
            raise ValueError("rate and maximum packet size must be positive and finite")
        self.port = port
        self.encoder = encoder or TIFrameEncoder()
        self.period = 1 / rate_hz
        self.clock, self.sleep = clock, sleep
        self.max_packet_bytes = max_packet_bytes
        self.next_send: float | None = None
        self.frame_count = 0
        self.closed = False

    def publish(self, frame: RadarFrame) -> None:
        if self.closed:
            raise UARTError("DATA transport is closed")
        data = self.encoder.encode(frame)
        if len(data) > self.max_packet_bytes:
            raise UARTError("Encoded packet exceeds bounded DATA packet size")
        if self.next_send is not None:
            while (remaining := self.next_send - self.clock()) > 0:
                self.sleep(remaining)
        started = self.clock()
        write_all(self.port, data, clock=self.clock)
        self.next_send = max(started + self.period, self.clock())
        self.frame_count += 1

    def pause(self) -> None:
        self.next_send = None

    def close(self) -> None:
        self.closed = True
        self.port.close()


# Commands present in installed TI ODS overhead configs, including low-bandwidth,
# static-retention and 8m variants. These acknowledge host configuration only.
CONFIG_COMMANDS = frozenset(
    [
        "flushCfg",
        "dfeDataOutputMode",
        "channelCfg",
        "adcCfg",
        "adcbufCfg",
        "lowPower",
        "profileCfg",
        "chirpCfg",
        "frameCfg",
        "dynamicRACfarCfg",
        "staticRACfarCfg",
        "dynamicRangeAngleCfg",
        "dynamic2DAngleCfg",
        "staticRangeAngleCfg",
        "antGeometry0",
        "antGeometry1",
        "antPhaseRot",
        "fovCfg",
        "compRangeBiasAndRxChanPhase",
        "staticBoundaryBox",
        "boundaryBox",
        "sensorPosition",
        "gatingParam",
        "stateParam",
        "allocationParam",
        "maxAcceleration",
        "trackingCfg",
        "presenceBoundaryBox",
        "fineMotionCfg",
    ]
)


class TICLIEmulator:
    """Two-line responses for UARTParser.sendCfg; no unsolicited banner/prompt.

    RF/config arguments are deliberately not interpreted. Input beyond the line
    limit is discarded through the next delimiter, never executed as a suffix.
    """

    def __init__(self, *, max_line_bytes: int = 4096) -> None:
        if max_line_bytes <= 0:
            raise ValueError("CLI line limit must be positive")
        self.running = False
        self.max_line_bytes = max_line_bytes
        self.pending = bytearray()
        self.discarding = False

    def feed(self, data: bytes) -> list[bytes]:
        responses = []
        for byte in data:
            if byte in (10, 13):
                if self.discarding:
                    responses.append(b"<invalid command>\r\nError: command too long\r\n")
                elif self.pending:
                    responses.append(self._command(bytes(self.pending)))
                self.pending.clear()
                self.discarding = False
            elif not self.discarding:
                if len(self.pending) >= self.max_line_bytes:
                    self.pending.clear()
                    self.discarding = True
                else:
                    self.pending.append(byte)
        return responses

    def _command(self, raw: bytes) -> bytes:
        command = raw.decode("ascii", errors="replace").strip()
        log.info("CLI received: %s", command)
        parts = command.split()
        status = "Done"
        if not parts:
            status = "Error: empty command"
        elif parts[0] in ("sensorStart", "sensorStop"):
            if len(parts) > 2 or (len(parts) == 2 and parts[1] not in ("0", "1")):
                status = "Error: invalid sensor command arguments"
            else:
                self.running = parts[0] == "sensorStart"
                log.info("Sensor state: %s", "RUNNING" if self.running else "STOPPED")
        elif parts[0] in CONFIG_COMMANDS:
            log.info("Compatibility acknowledgement only; no RF hardware settings applied")
        else:
            status = "Error: unsupported command"
        log.info("CLI response: echo + %s", status)
        return command.encode("ascii", errors="replace") + b"\r\n" + status.encode() + b"\r\n"


class TIUARTSession:
    """Single-threaded CLI service and start/stop gate around the DATA transport."""

    def __init__(self, cli_port: SerialPort, data: TIDataUART) -> None:
        self.cli_port, self.data = cli_port, data
        self.cli = TICLIEmulator()

    def poll_cli(self) -> None:
        try:
            if not self.cli_port.is_open:
                raise UARTError("CLI port disconnected or closed")
            incoming = self.cli_port.read(min(self.cli_port.in_waiting, 4096))
            for response in self.cli.feed(incoming):
                write_all(self.cli_port, response)
            if not self.cli.running:
                self.data.pause()
        except OSError as error:
            raise UARTError(f"CLI I/O failed: {error}") from error

    def publish(self, frame: RadarFrame) -> bool:
        if not self.cli.running:
            return False
        self.data.publish(frame)
        return True
