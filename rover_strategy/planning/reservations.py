"""Spatial (not time-indexed) corridor reservations between the two rovers, plus the hard
safety layer (imminent_collision) and deadlock resolution.

Deliberately NOT time-indexed: execution timing is unreliable (motor + vision latency), so
"who is where when" is tracked as *space currently claimed*, trimmed as a rover progresses,
rather than as a schedule that would drift out of sync with reality.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import Config, DEFAULT
from ..geometry import shapes as S
from ..geometry.footprint import Footprint
from ..world import RoverEstimate


# --------------------------------------------------------------------------- priority
# Strongly favour a rover mid PUSH/CAPTURE/RETREAT-with-cube (can't back out without dropping
# or fouling the cube) over one that is merely NAVIGATE/IDLE/WAIT (can yield cheaply).
HIGH_PRIORITY_STATES = frozenset({"PUSH", "CAPTURE", "RETREAT_WITH_CUBE"})


def priority(state_name: str, rover_id: int) -> int:
    """Deterministic priority ordinal: state dominates, ties broken in favour of the lower
    rover id.  Higher = more important = the other rover yields to it."""
    base = 100 if state_name in HIGH_PRIORITY_STATES else 10
    return base * 1000 - rover_id


def resolve_deadlock(state_a: str, id_a: int, waiting_s_a: float,
                      state_b: str, id_b: int, waiting_s_b: float, timeout_s: float) -> int | None:
    """Both rovers WAITING on each other (mutually blocked) past `timeout_s` -> the lower-priority
    one is told to yield.  Returns the rover id that must yield, or None if no deadlock is declared
    yet (either side hasn't been waiting long enough)."""
    if waiting_s_a < timeout_s or waiting_s_b < timeout_s:
        return None
    pa, pb = priority(state_a, id_a), priority(state_b, id_b)
    return id_b if pa >= pb else id_a


# --------------------------------------------------------------------------- reservation table
@dataclass
class _Reservation:
    polygons: list[np.ndarray]
    priority: int
    t: float


class ReservationTable:
    """Each rover holds at most one reservation: the list of convex polygons (corridor / prepush /
    depot footprints) it currently claims.  `reserve` replaces it wholesale; callers are expected to
    re-reserve (with a shrunk polygon list) as a plan is re-legged, and `trim` releases the part
    already physically traversed."""

    def __init__(self, cfg: Config = DEFAULT):
        self.cfg = cfg
        self._res: dict[int, _Reservation] = {}

    def reserve(self, rover_id: int, polygons: list[np.ndarray], priority: int, t: float) -> None:
        self._res[rover_id] = _Reservation(list(polygons), priority, t)

    def release(self, rover_id: int) -> None:
        self._res.pop(rover_id, None)

    def polygons_of(self, rover_id: int) -> list[np.ndarray]:
        res = self._res.get(rover_id)
        return list(res.polygons) if res else []

    def conflicts(self, rover_id: int, polygons: list[np.ndarray]) -> list[int]:
        """Other rover ids whose reservation comes within cfg.margins.rover_rover of `polygons`
        (distance < margin is exactly "would intersect once both are inflated by the margin")."""
        margin = self.cfg.margins.rover_rover
        hits: set[int] = set()
        for other_id, res in self._res.items():
            if other_id == rover_id:
                continue
            for p in polygons:
                for q in res.polygons:
                    if S.distance(p, q) < margin:
                        hits.add(other_id)
                        break
                else:
                    continue
                break
        return sorted(hits)

    def as_obstacles(self, for_rover: int) -> list[np.ndarray]:
        """The other rover's reserved area, inflated by cfg.margins.rover_rover, as static obstacle
        polygons a path planner for `for_rover` can route around."""
        margin = self.cfg.margins.rover_rover
        return [_inflate_poly(p, margin) for other_id, res in self._res.items() if other_id != for_rover
                for p in res.polygons]

    def trim(self, rover_id: int, progress_polygons: list[np.ndarray]) -> None:
        """Drop whichever of `rover_id`'s reserved polygons overlap the area already traversed."""
        res = self._res.get(rover_id)
        if res is None:
            return
        res.polygons = [p for p in res.polygons if not any(S.overlap(p, g) for g in progress_polygons)]


def _inflate_poly(P: np.ndarray, margin: float, n_dirs: int = 12) -> np.ndarray:
    """Conservative outward offset of a convex polygon by `margin`, via the exact Minkowski sum with
    a regular n_dirs-gon whose INradius equals margin (so real offset >= margin everywhere).
    ponytail: n-gon approximation of a disc offset; swap for an exact circular offset if margins get tight."""
    if margin <= 0:
        return P
    r = margin / math.cos(math.pi / n_dirs)
    dirs = np.array([[math.cos(2 * math.pi * k / n_dirs), math.sin(2 * math.pi * k / n_dirs)] for k in range(n_dirs)])
    pts = np.vstack([P + d * r for d in dirs])
    return S.hull(pts)


# --------------------------------------------------------------------------- hard safety layer
def _rollout(est: RoverEstimate, t: float) -> tuple[float, float, float]:
    """Constant-twist (v, omega) rollout of a rover pose forward by `t` seconds."""
    x0, y0, th0 = est.pose.x, est.pose.y, est.pose.theta
    w = est.omega
    if abs(w) < 1e-6:
        return x0 + est.v * math.cos(th0) * t, y0 + est.v * math.sin(th0) * t, th0
    th1 = th0 + w * t
    x1 = x0 + (est.v / w) * (math.sin(th1) - math.sin(th0))
    y1 = y0 - (est.v / w) * (math.cos(th1) - math.cos(th0))
    return x1, y1, th1


def imminent_collision(est_a: RoverEstimate, est_b: RoverEstimate, horizon_s: float,
                        cfg: Config = DEFAULT, dt: float = 0.1) -> bool:
    """Predict whether the two rovers' footprints (inflated by cfg.margins.rover_rover_estop, split
    evenly so the required clearance between raw footprints is the full margin) would overlap at any
    sampled instant within `horizon_s`, assuming each holds its current (v, omega)."""
    fp = Footprint(cfg.rover, inflate=cfg.margins.rover_rover_estop / 2.0)
    steps = max(1, int(round(horizon_s / dt)))
    for i in range(steps + 1):
        t = i * dt
        pa = _rollout(est_a, t)
        pb = _rollout(est_b, t)
        if S.overlap(fp.envelope(*pa), fp.envelope(*pb)):
            return True
    return False
