"""UART contracts use deterministic byte streams; no COM endpoints are opened."""

import sys
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import pytest

from rf_common.ti import decode_packet
from rf_common.ti_encoder import TIFrameEncoder
from rf_common.ti_uart import (
    CLI_BAUD,
    CONFIG_COMMANDS,
    DATA_BAUD,
    TICLIEmulator,
    TIDataUART,
    TIUARTSession,
    UARTError,
    open_serial,
    write_all,
)
from rf_sim.config import AppConfig
from rf_sim.simulator import Simulator
from rf_sim.ti_uart_bridge import run


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0
        self.sleeps.append(seconds)
        self.now += seconds


class FakeSerial:
    def __init__(self, incoming=b"", *, short_write=None):
        self.incoming = bytearray(incoming)
        self.output = bytearray()
        self.writes = []
        self.is_open = True
        self.short_write = short_write
        self.failure = None

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, size=1):
        if self.failure:
            raise self.failure
        data = bytes(self.incoming[:size])
        del self.incoming[:size]
        return data

    def write(self, data):
        if self.failure:
            raise self.failure
        count = len(data) if self.short_write is None else min(len(data), self.short_write)
        self.writes.append(bytes(data[:count]))
        self.output.extend(data[:count])
        return count

    def close(self):
        self.is_open = False


def frames(count=4):
    sim = Simulator(AppConfig(), "two_targets_manual", epoch=1700000000)
    result = []
    while len(result) < count:
        result.extend(sim.step())
    return result[:count]


@pytest.mark.parametrize("delimiter", [b"\r", b"\n", b"\r\n"])
def test_cli_partial_full_and_line_endings(delimiter):
    cli = TICLIEmulator()
    assert cli.feed(b"sensorSta") == []
    assert not cli.running
    assert cli.feed(b"rt" + delimiter) == [b"sensorStart\r\nDone\r\n"]
    assert cli.running
    assert cli.feed(b"sensorStop" + delimiter) == [b"sensorStop\r\nDone\r\n"]
    assert not cli.running


def test_cli_multiple_commands_and_exact_two_line_handshake():
    cli = TICLIEmulator()
    replies = cli.feed(b"sensorStop\r\nflushCfg\nsensorStart 0\r")
    assert replies == [
        b"sensorStop\r\nDone\r\n",
        b"flushCfg\r\nDone\r\n",
        b"sensorStart 0\r\nDone\r\n",
    ]
    assert cli.running
    assert all(len(r.splitlines()) == 2 for r in replies)
    assert cli.feed(b"sensorStop 0\n") == [b"sensorStop 0\r\nDone\r\n"]
    assert not cli.running


@pytest.mark.parametrize("command", sorted(CONFIG_COMMANDS))
def test_config_compatibility_acknowledgements(command, caplog):
    cli = TICLIEmulator()
    with caplog.at_level("INFO"):
        assert cli.feed(command.encode() + b" 1 2 3\n") == [
            command.encode() + b" 1 2 3\r\nDone\r\n"
        ]
    assert not cli.running
    assert "Compatibility acknowledgement only" in caplog.text


def test_unsupported_and_invalid_command_dont_start_sensor():
    cli = TICLIEmulator()
    assert cli.feed(b"unknown\n") == [b"unknown\r\nError: unsupported command\r\n"]
    assert cli.feed(b"sensorStart invalid\n") == [
        b"sensorStart invalid\r\nError: invalid sensor command arguments\r\n"
    ]
    assert not cli.running
    assert cli.feed(b"\xff\n")[0].endswith(b"Error: unsupported command\r\n")


def test_cli_bounded_input_discards_whole_overlong_command():
    cli = TICLIEmulator(max_line_bytes=12)
    assert cli.feed(b"x" * 10000 + b"sensorStart") == []
    assert len(cli.pending) == 0
    assert cli.feed(b"\r\nsensorStop\n") == [
        b"<invalid command>\r\nError: command too long\r\n",
        b"sensorStop\r\nDone\r\n",
    ]
    assert not cli.running


def test_exact_encoded_bytes_order_pacing_and_close():
    clock, port = FakeClock(), FakeSerial(short_write=11)
    data = TIDataUART(port, 20, clock=clock, sleep=clock.sleep)
    encoder = TIFrameEncoder()
    expected = bytearray()
    for frame in frames():
        expected.extend(encoder.encode(frame))
        data.publish(frame)
    assert port.output == expected
    assert clock.now == pytest.approx(0.15)
    assert clock.sleeps == pytest.approx([0.05] * 3)
    assert data.frame_count == 4
    cursor, numbers = 0, []
    while cursor < len(port.output):
        size = int.from_bytes(port.output[cursor + 12 : cursor + 16], "little")
        packet = decode_packet(bytes(port.output[cursor : cursor + size]))
        numbers.append(packet.header.frameNumber)
        cursor += size
    assert numbers == [0, 1, 2, 3]
    data.close()
    assert not port.is_open
    with pytest.raises(UARTError, match="closed"):
        data.publish(frames(1)[0])


def test_late_writer_preserves_order_without_catchup_queue():
    clock, port = FakeClock(), FakeSerial()
    data = TIDataUART(port, 20, clock=clock, sleep=clock.sleep)
    acquired = frames(3)
    data.publish(acquired[0])
    clock.now = 1  # Producer was delayed; don't replay missed wall-clock slots.
    data.publish(acquired[1])
    data.publish(acquired[2])
    assert clock.sleeps == pytest.approx([0.05])
    assert len(port.writes) == 3
    assert data.frame_count == 3
    # No submission queue exists: each publish has completed before returning.
    assert b"".join(port.writes) == port.output


def test_stop_start_stop_in_memory_integration():
    clock, cli, port = FakeClock(), FakeSerial(), FakeSerial()
    session = TIUARTSession(cli, TIDataUART(port, 20, clock=clock, sleep=clock.sleep))
    acquired = frames(4)
    cli.incoming.extend(b"sensorStop\n")
    session.poll_cli()
    assert not session.publish(acquired[0])
    assert port.output == b""
    cli.incoming.extend(b"sensorSta")
    session.poll_cli()
    assert not session.cli.running
    cli.incoming.extend(b"rt\r\n")
    session.poll_cli()
    assert session.publish(acquired[1])
    assert session.publish(acquired[2])
    before = bytes(port.output)
    cli.incoming.extend(b"sensorStop\n")
    session.poll_cli()
    assert not session.publish(acquired[3])
    assert port.output == before
    assert session.data.next_send is None
    cli.incoming.extend(b"sensorStart\n")
    session.poll_cli()
    assert session.publish(acquired[3])
    assert session.data.frame_count == 3


def test_data_limits_disconnect_and_write_failure():
    frame = frames(1)[0]
    port = FakeSerial()
    data = TIDataUART(port, 20, max_packet_bytes=40)
    with pytest.raises(UARTError, match="packet size"):
        data.publish(frame)
    assert port.output == b""
    port.close()
    with pytest.raises(UARTError, match="disconnected"):
        TIDataUART(port, 20).publish(frame)
    port = FakeSerial()
    port.failure = OSError("device removed")
    with pytest.raises(UARTError, match="device removed"):
        TIDataUART(port, 20).publish(frame)


def test_zero_progress_and_total_write_deadline():
    with pytest.raises(UARTError, match="no progress"):
        write_all(FakeSerial(short_write=0), b"abc")
    clock = FakeClock()

    class SlowPort(FakeSerial):
        def write(self, data):
            clock.now += 0.1
            return super().write(data)

    with pytest.raises(UARTError, match="deadline"):
        write_all(SlowPort(short_write=1), b"abcdef", clock=clock)
    assert clock.now == pytest.approx(0.3)


def test_cli_read_and_response_write_failures():
    cli = FakeSerial()
    session = TIUARTSession(cli, TIDataUART(FakeSerial(), 20))
    cli.close()
    with pytest.raises(UARTError, match="CLI.*disconnected"):
        session.poll_cli()
    cli.is_open = True
    cli.failure = OSError("read failed")
    with pytest.raises(UARTError, match="read failed"):
        session.poll_cli()
    cli.failure = None
    cli.short_write = 0
    cli.incoming.extend(b"sensorStart\n")
    with pytest.raises(UARTError, match="no progress"):
        session.poll_cli()


def test_open_serial_settings_and_actionable_errors(monkeypatch):
    calls = []

    def factory(**kwargs):
        calls.append(kwargs)
        return FakeSerial()

    fake_module = SimpleNamespace(Serial=factory, EIGHTBITS=8, PARITY_NONE="N", STOPBITS_ONE=1)
    monkeypatch.setitem(sys.modules, "serial", fake_module)
    open_serial("arbitrary-endpoint", DATA_BAUD)
    assert calls == [
        {
            "port": "arbitrary-endpoint",
            "baudrate": 921600,
            "bytesize": 8,
            "parity": "N",
            "stopbits": 1,
            "timeout": 0,
            "write_timeout": 0.1,
            "xonxoff": False,
            "rtscts": False,
            "dsrdtr": False,
        }
    ]
    for message in ("File not found", "Access denied; already open"):

        def fail(error_message=message, **kwargs):
            raise OSError(error_message)

        fake_module.Serial = fail
        with pytest.raises(UARTError, match="Check the port exists.*already in use"):
            open_serial("arbitrary-endpoint", CLI_BAUD)


def test_bridge_real_physics_fake_ports_and_shutdown():
    clock = FakeClock()
    ports = {"CLI-X": FakeSerial(b"sensorStart\n"), "DATA-Y": FakeSerial()}
    count = run(
        AppConfig(),
        "two_targets",
        "CLI-X",
        "DATA-Y",
        use_gui=False,
        duration=0.3,
        serial_factory=lambda name, baud: ports[name],
        clock=clock,
        sleep=clock.sleep,
    )
    assert count >= 4
    assert ports["DATA-Y"].output.startswith(b"\x02\x01\x04\x03\x06\x05\x08\x07")
    assert all(not p.is_open for p in ports.values())


def test_bridge_closes_cli_when_data_open_fails():
    cli = FakeSerial()

    def factory(name, baud):
        if name == "DATA":
            raise UARTError("DATA port unavailable")
        return cli

    with pytest.raises(UARTError, match="unavailable"):
        run(AppConfig(), "two_targets", "CLI", "DATA", serial_factory=factory, use_gui=False)
    assert not cli.is_open


@pytest.mark.parametrize("failure", [KeyboardInterrupt(), OSError("removed")])
def test_bridge_ctrl_c_and_io_error_close_both_ports(failure):
    clock = FakeClock()
    ports = [FakeSerial(b"sensorStart\n"), FakeSerial()]

    def sleep(seconds):
        raise failure

    kwargs = {
        "use_gui": False,
        "serial_factory": lambda name, baud: ports[name == "DATA"],
        "clock": clock,
        "sleep": sleep,
    }
    if isinstance(failure, KeyboardInterrupt):
        assert run(AppConfig(), "two_targets", "CLI", "DATA", **kwargs) == 0
    else:
        with pytest.raises(OSError, match="removed"):
            run(AppConfig(), "two_targets", "CLI", "DATA", **kwargs)
    assert all(not p.is_open for p in ports)


def test_bridge_manual_controls_share_simulator(monkeypatch):
    import pygame

    from rf_sim.gui import DebugGUI

    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    keys = defaultdict(int, {pygame.K_w: 1, pygame.K_UP: 1})
    monkeypatch.setattr(pygame.key, "get_pressed", lambda: keys)
    observed = []
    original = DebugGUI.observe

    def observe(self, simulator, ready):
        observed.append(tuple(v.state.speed for v in simulator.vehicles))
        original(self, simulator, ready)

    monkeypatch.setattr(DebugGUI, "observe", observe)
    clock = FakeClock()
    cli, port = FakeSerial(b"sensorStart\n"), FakeSerial()
    assert (
        run(
            AppConfig(),
            "two_targets_manual",
            "CLI",
            "DATA",
            duration=0.12,
            serial_factory=lambda name, baud: cli if name == "CLI" else port,
            clock=clock,
            sleep=clock.sleep,
        )
        > 0
    )
    assert observed[-1][0] > 0 and observed[-1][1] > 0
    assert not pygame.get_init()


def test_host_config_all_commands_acknowledged_and_stop_config():
    cli = TICLIEmulator()
    root = Path(__file__).resolve().parents[1]
    for name in ("ti_uart_host_only.cfg", "ti_uart_stop_only.cfg"):
        lines = (root / "configs" / name).read_bytes().splitlines()
        assert b"HOST SOFTWARE EMULATION ONLY" in lines[0]
        replies = cli.feed(b"\n".join(line for line in lines if not line.startswith(b"%")) + b"\n")
        assert all(r.endswith(b"\r\nDone\r\n") for r in replies)
        assert cli.running == (name == "ti_uart_host_only.cfg")


@pytest.mark.parametrize("rate", [0, -1, float("inf"), float("nan")])
def test_invalid_pacing_rate(rate):
    with pytest.raises(ValueError):
        TIDataUART(FakeSerial(), rate)


def test_same_endpoint_rejected_before_open():
    with pytest.raises(ValueError, match="different endpoints"):
        run(AppConfig(), "two_targets", "COM-X", "com-x", use_gui=False)
