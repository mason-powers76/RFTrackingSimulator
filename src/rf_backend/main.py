"""Run local UDP ingestion, SQLite persistence, and the dashboard HTTP API."""

import argparse
import logging
import sqlite3
import threading
from pathlib import Path

from rf_backend.api import create_server
from rf_backend.store import RadarStore
from rf_backend.udp import UDPReceiver

logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=Path, default=Path("data/radar.sqlite3"))
    parser.add_argument("--stream-id", required=True, help="Unique name per acquisition run/replay")
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8000)
    args = parser.parse_args()
    if not all(1 <= port <= 65535 for port in (args.udp_port, args.http_port)):
        parser.error("ports must be between 1 and 65535")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    store = receiver = server = thread = None
    try:
        store = RadarStore(args.db, args.stream_id)
        receiver = UDPReceiver(store, args.udp_host, args.udp_port)
        server = create_server(store, args.http_host, args.http_port)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        logger.info(
            "Listening UDP %s:%d; HTTP http://%s:%d; DB %s; stream %s",
            args.udp_host,
            args.udp_port,
            args.http_host,
            args.http_port,
            args.db.resolve(),
            args.stream_id,
        )
        while True:
            receiver.receive_once()
    except KeyboardInterrupt:
        logger.info("Backend stopped")
    except (OSError, ValueError, sqlite3.Error) as error:
        parser.exit(2, f"Error: {error}\n")
    finally:
        if server:
            if thread:
                server.shutdown()
                thread.join()
            server.server_close()
        if receiver:
            receiver.close()
        if store:
            store.close()


if __name__ == "__main__":
    main()
