from __future__ import annotations

import json
import socket
import subprocess
import time
from pathlib import Path

import pytest

from rover_strategy.vision.client import VisionClient
from rover_strategy.vision.parser import parse_message


ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_CONTRACT = (
    ROOT
    / "_source"
    / "Rover Vision Artificial"
    / "Vision-Rover-Challenge-main_unz"
    / "Vision-Rover-Challenge-main"
    / "vision-system"
    / "contrato"
)
MOCK_PUBLISHER = OFFICIAL_CONTRACT / "mock_publisher.py"


def _message(seq: int, version: int = 1) -> dict:
    return {
        "v": version,
        "seq": seq,
        "ts_ms": 1_000,
        "phase": "IDLE",
        "grid": {"cols": 4, "rows": 5, "cell_mm": 20.0},
        "rovers": [],
        "cubes": [],
        "obstacles": [],
        "depots": [],
    }


def test_unknown_versions_are_counted_and_a_custom_adapter_can_be_enabled():
    peer, server = socket.socketpair()
    client = VisionClient(accepted_versions={1, 2})
    client.sock = server
    server.setblocking(False)
    try:
        peer.sendall((json.dumps(_message(1, version=2)) + "\n").encode())
        assert client.latest() is None
        assert client.dropped_invalid == 1
        assert client.dropped_unknown_versions == 1

        def v2_adapter(msg: dict, received: float, cfg):
            # Test-only adapter: the v2 wire format is not defined here.  This
            # hook deliberately delegates a fixture with the known v1 shape.
            return parse_message({**msg, "v": 1}, received, cfg)

        client.adapters = {2: v2_adapter}
        peer.sendall((json.dumps(_message(2, version=2)) + "\n").encode())
        frame = client.latest()
        assert frame is not None and frame.seq == 2
        assert client.dropped_unknown_versions == 1
    finally:
        peer.close()
        client.close()


def test_client_handles_partial_lines_and_counts_a_sequence_gap():
    peer, server = socket.socketpair()
    client = VisionClient()
    client.sock = server
    server.setblocking(False)
    line_1 = (json.dumps(_message(10)) + "\n").encode()
    line_3 = (json.dumps(_message(12)) + "\n").encode()
    try:
        peer.sendall(line_1[:7])
        assert client.latest() is None
        peer.sendall(line_1[7:] + line_3[:11])
        first = client.latest()
        assert first is not None and first.seq == 10
        peer.sendall(line_3[11:])
        second = client.latest()
        assert second is not None and second.seq == 12
        assert client.seq_gaps == 1
    finally:
        peer.close()
        client.close()


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _wait_for_frame(client: VisionClient, phase: str | None = None, timeout: float = 4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = client.latest()
        if frame is not None and (phase is None or frame.phase == phase):
            return frame
        time.sleep(0.01)
    return None


class _ChunkedSocket:
    """Limit reads without changing the official publisher or wire format."""

    def __init__(self, sock: socket.socket, chunk_size: int = 7):
        self._sock = sock
        self._chunk_size = chunk_size

    def recv(self, size: int) -> bytes:
        return self._sock.recv(min(size, self._chunk_size))

    def close(self) -> None:
        self._sock.close()


def test_official_mock_publisher_end_to_end() -> None:
    """Consume official NDJSON, including phase transitions and row-down conversion."""
    if not MOCK_PUBLISHER.is_file():
        pytest.skip("official mock publisher source is unavailable")

    try:
        port = _free_port()
    except OSError as exc:
        pytest.skip(f"cannot reserve a free TCP port: {exc}")

    process = subprocess.Popen(
        [
            str(ROOT / ".venv" / "bin" / "python"),
            str(MOCK_PUBLISHER),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        cwd=OFFICIAL_CONTRACT,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    client = VisionClient(port=port)
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            try:
                client.connect(timeout=0.1)
                break
            except OSError:
                if process.poll() is not None:
                    error = process.stderr.read() if process.stderr is not None else ""
                    if "Address already in use" in error or "OSError" in error and "bind" in error:
                        pytest.skip(f"official mock publisher could not bind port {port}")
                    raise RuntimeError(f"mock publisher exited during startup: {error}")
                time.sleep(0.02)
        else:
            pytest.fail("official mock publisher did not accept a TCP connection")
        assert client.sock is not None
        client.sock = _ChunkedSocket(client.sock)

        idle = _wait_for_frame(client, "IDLE")
        assert idle is not None
        assert set(idle.rovers) == {10, 11}
        assert set(idle.cubes) == {"green", "blue", "red"}
        assert idle.board_w == pytest.approx(860.0)
        assert idle.board_h == pytest.approx(860.0)
        # Official config starts rover 10 at (col,row)=(4,4), theta=45°.
        # Parser output is millimetres, radians, and y-up.
        rover = idle.rovers[10]
        assert rover.pose.x == pytest.approx(80.0, abs=2.0)
        assert rover.pose.y == pytest.approx((43.0 - 4.0) * 20.0, abs=2.0)
        assert rover.pose.theta == pytest.approx(0.785, abs=0.08)

        assert process.stdin is not None
        process.stdin.write("ready\n")
        process.stdin.flush()
        assert _wait_for_frame(client, "READY") is not None
        process.stdin.write("start\n")
        process.stdin.flush()
        running = _wait_for_frame(client, "RUNNING")
        assert running is not None

        # The official publisher keeps one pending message per client.  Let it
        # overwrite that slot, then drain once: a sequence jump is expected.
        time.sleep(0.30)
        latest = client.latest()
        assert latest is not None
        assert client.seq_gaps > 0
        assert set(latest.rovers) == {10, 11}
        assert set(latest.cubes) == {"green", "blue", "red"}
    finally:
        client.close()
        if process.stdin is not None:
            try:
                process.stdin.write("quit\n")
                process.stdin.flush()
            except OSError:
                pass
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=3.0)
