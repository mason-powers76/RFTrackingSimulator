"""Transactional SQLite state and history; no prediction or target association."""

import json
import math
import sqlite3
import threading
import time
from pathlib import Path

from rf_backend.models import Detection, RadarFrame

SCHEMA = """
CREATE TABLE IF NOT EXISTS frames (
    id INTEGER PRIMARY KEY,
    stream_id TEXT NOT NULL,
    source TEXT NOT NULL,
    frame_id INTEGER NOT NULL,
    timestamp REAL NOT NULL,
    received_at REAL NOT NULL,
    simulation_time REAL,
    UNIQUE(stream_id, frame_id)
);
CREATE INDEX IF NOT EXISTS frame_time ON frames(stream_id, timestamp, frame_id);
CREATE TABLE IF NOT EXISTS detections (
    frame_pk INTEGER NOT NULL REFERENCES frames(id),
    target_id INTEGER NOT NULL,
    x REAL NOT NULL, y REAL NOT NULL,
    vx REAL, vy REAL, speed REAL, heading_rad REAL,
    range_m REAL, azimuth_rad REAL, confidence REAL, snr_db REAL,
    PRIMARY KEY(frame_pk, target_id)
);
CREATE INDEX IF NOT EXISTS target_history ON detections(target_id, frame_pk);
CREATE TABLE IF NOT EXISTS frame_details (
    frame_pk INTEGER PRIMARY KEY REFERENCES frames(id),
    payload TEXT NOT NULL
);
"""


class RadarStore:
    def __init__(self, path: Path | str, stream_id: str) -> None:
        if not stream_id or len(stream_id) > 128:
            raise ValueError("stream_id must contain 1 to 128 characters")
        self.stream_id = stream_id
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.connection = sqlite3.connect(path, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        self.connection.execute("PRAGMA foreign_keys=ON")
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.executescript(SCHEMA)

    def ingest(self, frame: RadarFrame, received_at: float | None = None) -> bool:
        frame = RadarFrame.model_validate(frame.model_dump())
        frame = frame.model_copy(
            update={"targets": tuple(sorted(frame.targets, key=lambda t: t.target_id))}
        )
        received_at = time.time() if received_at is None else received_at
        if not math.isfinite(received_at) or received_at < 0:
            raise ValueError("received_at must be finite and nonnegative")
        with self.lock, self.connection:
            existing = self.connection.execute(
                "SELECT id FROM frames WHERE stream_id=? AND frame_id=?",
                (self.stream_id, frame.frame_id),
            ).fetchone()
            if existing:
                if self._frame(existing["id"])["frame"] != frame.model_dump(mode="json"):
                    raise ValueError(
                        "conflicting duplicate frame; use a new stream ID for a new run"
                    )
                return False
            source = self.connection.execute(
                "SELECT source FROM frames WHERE stream_id=? LIMIT 1", (self.stream_id,)
            ).fetchone()
            if source and source["source"] != frame.source:
                raise ValueError("a stream must have one source")
            cursor = self.connection.execute(
                "INSERT INTO frames(stream_id,source,frame_id,timestamp,received_at,simulation_time) "
                "VALUES(?,?,?,?,?,?)",
                (
                    self.stream_id,
                    frame.source,
                    frame.frame_id,
                    frame.timestamp,
                    received_at,
                    frame.simulation_time,
                ),
            )
            for target in frame.targets:
                self.connection.execute(
                    "INSERT INTO detections VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                    (
                        cursor.lastrowid,
                        target.target_id,
                        target.x,
                        target.y,
                        target.vx,
                        target.vy,
                        target.speed,
                        target.heading_rad,
                        target.range_m,
                        target.azimuth_rad,
                        target.confidence,
                        target.snr_db,
                    ),
                )
            self.connection.execute(
                "INSERT INTO frame_details VALUES(?,?)",
                (cursor.lastrowid, frame.model_dump_json()),
            )
        return True

    def _frame(self, frame_pk: int) -> dict:
        row = dict(
            self.connection.execute("SELECT * FROM frames WHERE id=?", (frame_pk,)).fetchone()
        )
        details = self.connection.execute(
            "SELECT payload FROM frame_details WHERE frame_pk=?", (frame_pk,)
        ).fetchone()
        if details is not None:
            return {
                "stream_id": row["stream_id"],
                "received_at": row["received_at"],
                "frame": json.loads(details["payload"]),
            }
        targets = []
        for target in self.connection.execute(
            "SELECT * FROM detections WHERE frame_pk=? ORDER BY target_id", (frame_pk,)
        ):
            values = dict(target)
            values.pop("frame_pk")
            targets.append(Detection.model_validate(values))
        frame = RadarFrame(
            source=row["source"],
            frame_id=row["frame_id"],
            timestamp=row["timestamp"],
            simulation_time=row["simulation_time"],
            targets=tuple(targets),
        )
        return {
            "stream_id": row["stream_id"],
            "received_at": row["received_at"],
            "frame": frame.model_dump(mode="json"),
        }

    def latest(self) -> dict | None:
        with self.lock:
            row = self.connection.execute(
                "SELECT id FROM frames WHERE stream_id=? ORDER BY timestamp DESC, frame_id DESC LIMIT 1",
                (self.stream_id,),
            ).fetchone()
            return self._frame(row["id"]) if row else None

    def history(self, start: float, end: float, limit: int = 1000) -> list[dict]:
        if not (math.isfinite(start) and math.isfinite(end) and 0 <= start <= end):
            raise ValueError("require finite 0 <= start <= end (Unix seconds)")
        if not 1 <= limit <= 10000:
            raise ValueError("limit must be between 1 and 10000")
        with self.lock:
            rows = self.connection.execute(
                "SELECT id FROM frames WHERE stream_id=? AND timestamp BETWEEN ? AND ? "
                "ORDER BY timestamp, frame_id LIMIT ?",
                (self.stream_id, start, end, limit),
            ).fetchall()
            return [self._frame(row["id"]) for row in rows]

    def last_seen(self, target_id: int, at: float | None = None) -> dict | None:
        if target_id < 0 or target_id > 2**63 - 1:
            raise ValueError("invalid target ID")
        if at is not None and (not math.isfinite(at) or at < 0):
            raise ValueError("at must be finite nonnegative Unix seconds")
        with self.lock:
            row = self.connection.execute(
                "SELECT f.id FROM frames f JOIN detections d ON d.frame_pk=f.id "
                "WHERE f.stream_id=? AND d.target_id=? AND f.timestamp<=? "
                "ORDER BY f.timestamp DESC, f.frame_id DESC LIMIT 1",
                (self.stream_id, target_id, at if at is not None else float("inf")),
            ).fetchone()
            if not row:
                return None
            result = self._frame(row["id"])
            frame = result.pop("frame")
            result.update(
                timestamp=frame["timestamp"],
                frame_id=frame["frame_id"],
                target=next(t for t in frame["targets"] if t["target_id"] == target_id),
            )
            latest = self.latest()
            result["detected_in_latest_frame"] = bool(
                latest and any(t["target_id"] == target_id for t in latest["frame"]["targets"])
            )
            return result

    def close(self) -> None:
        with self.lock:
            self.connection.close()
