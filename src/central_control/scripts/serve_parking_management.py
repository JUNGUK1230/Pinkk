#!/usr/bin/env python3
"""Serve the parking management UI and stop its launcher on request."""

from __future__ import annotations

import argparse
import json
import os
import signal
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import yaml


class ParkingManagementHandler(SimpleHTTPRequestHandler):
    vehicle_config: str

    def do_GET(self) -> None:
        if self.path == "/api/vehicles":
            with open(self.vehicle_config, encoding="utf-8") as file:
                vehicles = yaml.safe_load(file)["vehicles"]
            body = json.dumps({"vehicles": vehicles}, ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def do_POST(self) -> None:
        if self.path != "/api/shutdown":
            self.send_error(404)
            return
        if not self._is_same_origin_request():
            self.send_error(403, "Cross-origin shutdown request rejected")
            return

        body = json.dumps({"ok": True, "message": "Services are stopping."}).encode()
        self.send_response(202)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()

        # run_parking_management.sh traps TERM and cleans up every process group.
        threading.Timer(0.2, os.kill, args=(os.getppid(), signal.SIGTERM)).start()

    def _is_same_origin_request(self) -> bool:
        host = self.headers.get("Host")
        origin = self.headers.get("Origin")
        if not host or not origin:
            return False
        parsed = urlsplit(origin)
        return parsed.scheme in {"http", "https"} and parsed.netloc == host


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--directory", required=True)
    parser.add_argument("--vehicle-config", required=True)
    args = parser.parse_args()

    ParkingManagementHandler.vehicle_config = args.vehicle_config
    handler = partial(ParkingManagementHandler, directory=args.directory)
    ThreadingHTTPServer((args.bind, args.port), handler).serve_forever()


if __name__ == "__main__":
    main()
