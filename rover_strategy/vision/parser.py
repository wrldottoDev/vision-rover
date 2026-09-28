"""Official telemetry v1 (CONTRATO.md) -> world.Frame.  The only place official units enter the system.

Validation policy (contract s6): unknown `v` -> drop; look things up by id / colour, never by index;
malformed entries are skipped individually rather than dropping the whole frame.
"""
from __future__ import annotations

import json
import math
from collections.abc import Collection, Mapping
from typing import Callable

from ..config import Config, DEFAULT
from ..frames import Grid, marker_to_body
from ..geometry.zones import DepotZone
from ..world import CubeObs, Frame, Pose, RoverObs

PROTOCOL_VERSION = 1


class TelemetryError(ValueError):
    pass


class UnsupportedVersion(TelemetryError):
    """Raised when no enabled adapter can interpret a telemetry version."""

    def __init__(self, version: object):
        self.version = version
        super().__init__(f"unsupported protocol version {version!r}")


VersionAdapter = Callable[[dict, float, Config], Frame]


def _num(d: dict, k: str) -> float:
    v = d[k]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise TelemetryError(f"bad number {k}={v!r}")
    return float(v)


def _parse_v1(msg: dict, t_received: float, cfg: Config) -> Frame:
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


# Deliberately only v1 is registered here.  A future protocol can add its
# adapter without changing the v1 parser or pretending that its wire format is
# already known.  Callers may provide an additional adapter explicitly and must
# also opt that version into ``accepted_versions``.
VERSION_ADAPTERS: Mapping[int, VersionAdapter] = {PROTOCOL_VERSION: _parse_v1}
ACCEPTED_VERSIONS = frozenset(VERSION_ADAPTERS)


def parse_message(
    msg: dict | str | bytes,
    t_received: float,
    cfg: Config = DEFAULT,
    *,
    accepted_versions: Collection[int] | None = None,
    adapters: Mapping[int, VersionAdapter] | None = None,
) -> Frame:
    """Parse one telemetry message using explicitly enabled version adapters.

    The default accepts only the official v1 format.  ``adapters`` is an
    extension hook for a future wire version; it is intentionally not supplied
    by this project until that version's contract exists.
    """
    if isinstance(msg, (str, bytes)):
        msg = json.loads(msg)
    if not isinstance(msg, dict):
        raise TelemetryError("telemetry message must be a JSON object")

    version = msg.get("v")
    enabled = ACCEPTED_VERSIONS if accepted_versions is None else frozenset(accepted_versions)
    registry = VERSION_ADAPTERS if adapters is None else {**VERSION_ADAPTERS, **adapters}
    try:
        adapter = registry.get(version) if version in enabled else None
    except TypeError:
        # JSON permits arrays/objects here, but they cannot be protocol
        # version keys.  Treat them as unknown rather than leaking a raw
        # unhashable-key exception through the client.
        adapter = None
    if adapter is None:
        raise UnsupportedVersion(version)
    return adapter(msg, t_received, cfg)
