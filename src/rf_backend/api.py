"""Small local read-only JSON HTTP interface for a future dashboard."""

import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlsplit

from rf_backend.store import RadarStore


def create_server(store: RadarStore, host: str, port: int) -> HTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            url = urlsplit(self.path)
            query = parse_qs(url.query, keep_blank_values=True)
            try:
                if any(len(values) != 1 for values in query.values()):
                    raise ValueError("query parameters must occur once")
                if url.path == "/api/latest":
                    result = store.latest()
                elif url.path == "/api/frames":
                    result = store.history(
                        float(query["start"][0]),
                        float(query["end"][0]),
                        int(query.get("limit", ["1000"])[0]),
                    )
                elif url.path.startswith("/api/targets/"):
                    target_id = int(url.path.removeprefix("/api/targets/"))
                    at = float(query["at"][0]) if "at" in query else None
                    result = store.last_seen(target_id, at)
                else:
                    self.respond(404, {"error": "unknown endpoint"})
                    return
                self.respond(200 if result is not None else 404, result)
            except (KeyError, ValueError, OverflowError) as error:
                self.respond(400, {"error": str(error)})

        def respond(self, status: int, result: object) -> None:
            payload = json.dumps(result, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format: str, *args: object) -> None:
            pass

    return HTTPServer((host, port), Handler)
