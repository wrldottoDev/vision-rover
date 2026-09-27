"""Official telemetry v1 (CONTRATO.md) -> world.Frame.  The only place official units enter the system.

Validation policy (contract s6): unknown `v` -> drop; look things up by id / colour, never by index;
malformed entries are skipped individually rather than dropping the whole frame.
"""
from __future__ import annotations

import json
import math

from ..config import Config, DEFAULT
from ..frames import Grid, marker_to_body
from ..geometry.zones import DepotZone
from ..world import CubeObs, Frame, Pose, RoverObs

PROTOCOL_VERSION = 1


class TelemetryError(ValueError):
    pass


def _num(d: dict, k: str) -> float:
    v = d[k]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise TelemetryError(f"bad number {k}={v!r}")
    return float(v)


def parse_message(msg: dict | str | bytes, t_received: float, cfg: Config = DEFAULT) -> Frame:
    if isinstance(msg, (str, bytes)):
        msg = json.loads(msg)
    if msg.get("v") != PROTOCOL_VERSION:
        raise TelemetryError(f"unsupported protocol version {msg.get('v')!r}")
    g = msg["grid"]
    grid = Grid(int(g["cols"]), int(g["rows"]), _num(g, "cell_mm"))
    t_cap = _num(msg, "ts_ms") / 1000.0
    rg = cfg.rover

    rovers: dict[int, RoverObs] = {}
    for r in msg.get("rovers", []):
        try:
            x, y, th = grid.to_internal_pose(_num(r, "col"), _num(r, "row"), _num(r, "theta"))
            age = max(0.0, _num(r, "age_ms") / 1000.0)
            bx, by, bth = marker_to_body(x, y, th, rg.marker_offset_fwd, rg.marker_offset_left, rg.marker_angle_offset)
            rovers[int(r["id"])] = RoverObs(int(r["id"]), Pose(bx, by, bth), t_cap - age, age)
        except (KeyError, TypeError, TelemetryError):
            continue

    cubes: dict[str, CubeObs] = {}
    for c in msg.get("cubes", []):
        try:
            x, y = grid.to_internal_xy(_num(c, "col"), _num(c, "row"))
            age = max(0.0, _num(c, "age_ms") / 1000.0)
            cubes[str(c["color"])] = CubeObs(str(c["color"]), x, y, t_cap - age, age)
        except (KeyError, TypeError, TelemetryError):
            continue

    depots: dict[str, DepotZone] = {}
    for d in msg.get("depots", []):
        x, y = grid.to_internal_xy(_num(d, "col"), _num(d, "row"))
        depots[str(d["color"])] = DepotZone(str(d["color"]), x, y, cfg.depot.half_size)

    obstacles = tuple(grid.to_internal_xy(_num(o, "col"), _num(o, "row")) for o in msg.get("obstacles", []))
    return Frame(seq=int(msg["seq"]), t_capture=t_cap, t_received=t_received, phase=str(msg["phase"]),
                 board_w=grid.cols * grid.cell_mm, board_h=grid.rows * grid.cell_mm,
                 rovers=rovers, cubes=cubes, depots=depots, obstacles=obstacles)
