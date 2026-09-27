"""TCP NDJSON client for the official vision feed (port 2026).  Last-value-wins (contract s6.3):
`latest()` drains everything available and returns only the newest parsed frame."""
from __future__ import annotations

import json
import socket
import time

from ..config import Config, DEFAULT
from ..world import Frame
from .parser import TelemetryError, parse_message

DEFAULT_PORT = 2026


class VisionClient:
    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT, cfg: Config = DEFAULT):
        self.addr = (host, port)
        self.cfg = cfg
        self.sock: socket.socket | None = None
        self.buf = b""
        self.dropped_invalid = 0
        self.last_seq: int | None = None
        self.seq_gaps = 0

    def connect(self, timeout: float = 2.0) -> None:
        self.sock = socket.create_connection(self.addr, timeout=timeout)
        self.sock.setblocking(False)
        self.buf = b""

    def close(self) -> None:
        if self.sock:
            self.sock.close()
        self.sock = None

    def latest(self) -> Frame | None:
        """Non-blocking. Reconnect is the caller's job (returns None and closes on EOF)."""
        if self.sock is None:
            return None
        try:
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    self.close()
                    break
                self.buf += chunk
        except BlockingIOError:
            pass
        *lines, self.buf = self.buf.split(b"\n")
        for line in reversed(lines):
            if not line.strip():
                continue
            try:
                frame = parse_message(json.loads(line), time.time(), self.cfg)
            except (json.JSONDecodeError, TelemetryError, KeyError, TypeError, ValueError):
                self.dropped_invalid += 1
                continue
            if self.last_seq is not None and frame.seq > self.last_seq + 1:
                self.seq_gaps += 1
            self.last_seq = frame.seq
            return frame
        return None
