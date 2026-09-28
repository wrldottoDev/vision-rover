"""Collision-free navigation planner for a single rover with an asymmetric footprint.

Design choice (Hybrid A* over a discrete {rotate in place, straight fwd/rev} lattice, plus an
analytic rotate-straight-rotate (RTR) shortcut and post-hoc shortcutting), evaluated against the
alternatives, tied to THIS rover's actual motion model (in-place rotation about the axle midpoint;
straight forward/reverse; no strafing, no continuous-curvature driving in the executor):

* Visibility graph + rotate-translate-rotate: cheap and near-optimal for *point* robots, but this
  rover's footprint is asymmetric (nose sticks 102 mm out front, only 47 mm behind, paddles narrow
  the front) and the in-place rotation sweeps a 113.5 mm circle around the axle -- much bigger than
  the straight-line footprint. A visibility graph built on inflated obstacles checks the *disc*
  clearance implicitly if you inflate by the sweep radius everywhere, but that is needlessly
  conservative near the goal (a straight-only corridor narrower than 2x sweep radius but wider than
  the footprint becomes "unreachable" even though the rover can drive straight through it without
  ever rotating there). It also doesn't reason about the case in this brief where the rover must
  translate to a clear spot *before* it can rotate to the goal heading (edge-of-board pre-push
  poses) -- that requires searching translate/rotate order, not just connecting visibility vertices.
* Pure state-lattice A* (fixed step + fixed heading bins only, no continuous state): simple and
  predictable, but landing exactly on the goal pose (position tolerance ~mm, heading tolerance for
  the push direction) needs either a very fine lattice (expensive) or a final analytic connector
  anyway. Since we need the connector regardless, keep the lattice coarse and continuous-valued.
* Hybrid A* (continuous (x, y, theta) state, primitives = rotate +-one heading bin / rotate exactly
  to the goal heading / straight forward / straight reverse, closed set discretised onto
  (nav_cell_mm, 360/nav_heading_bins) purely to bound re-expansion) with an RTR analytic expansion
  tried at every popped node, plus post-search shortcutting: this is what's picked. It matches the
  primitive set the executor can actually run, keeps the goal pose exact (no lattice-resolution
  goal error), and the RTR shortcut turns almost every open-field query into an O(1) success instead
  of a full search, which is what keeps typical queries fast (see report for measured timings).

Cost model: cost_s estimates execution time from config.limits (v_nav, w_nav): straight time =
length / v_nav (reverse segments penalised by NavParams.reverse_penalty, TUNED -- reversing is
slower/riskier to line up than driving forward, no measured number exists yet); rotate time =
angle / w_nav + NavParams.rotate_settle_s (TUNED fixed settle/accel overhead per rotation, since a
sequence of many small rotations should cost more than one big one).

Units throughout: millimetres, radians, seconds (see config.py).
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from typing import Sequence, Union

import numpy as np

from ..config import Config, DEFAULT
from ..frames import angle_diff, wrap
from ..geometry import shapes as S
from ..geometry.footprint import Footprint
from ..world import Path, Pose, SegKind, Segment

# ---------------------------------------------------------------------------------------- obstacles


@dataclass(frozen=True)
class DiscObstacle:
    x: float
    y: float
    r: float


@dataclass(frozen=True)
class PolyObstacle:
    poly: np.ndarray  # (n, 2) convex, CCW


Obstacle = Union[DiscObstacle, PolyObstacle]

# Rotation-sweep collision sampling: intermediate headings no coarser than this (spec: <=5 deg).
_ROTATE_SAMPLE_RAD = math.radians(5.0)
# swept_polygons() samples a bit finer than the collision check purely for a tighter (still
# conservative) reservation polygon; not load-bearing for correctness.
_SWEEP_SAMPLE_RAD = math.radians(3.0)


def _poly_out_of_board(poly: np.ndarray, board_w: float, board_h: float, board_margin: float) -> bool:
    return not S.inside_rect(poly, board_margin, board_w - board_margin, board_margin, board_h - board_margin)


def _poly_hits_obstacle(poly: np.ndarray, obstacle: Obstacle, pad: float = 0.0) -> bool:
    """True if `poly` is within `pad` of touching `obstacle` (pad=0 -> plain touch/overlap).

    `pad` is how the continuous-rotation-sweep conservatism (see `_arc_pad`) is applied: instead of
    inflating the polygon geometrically, we shrink the "safe" gap by `pad`, which is equivalent and
    cheaper with the primitives `shapes.py` already provides. For pad=0 this is exactly the original
    check (`S.distance(...) <= 0.0` <=> `S.overlap(...)`, since `distance` returns 0 iff they overlap).
    """
    if isinstance(obstacle, DiscObstacle):
        return S.disc_distance(poly, (obstacle.x, obstacle.y), obstacle.r) <= pad
    return S.distance(poly, obstacle.poly) <= pad


def _fp_sweep_radius(fp) -> float:
    """Max distance from the rotation pivot to any footprint point, for a `fp` that only guarantees
    `.envelope(x, y, theta)` (an explicit `.sweep_radius` attribute, e.g. on `Footprint`, is used
    when present -- it's the same number, just precomputed)."""
    r = getattr(fp, "sweep_radius", None)
    if r is not None:
        return float(r)
    poly = fp.envelope(0.0, 0.0, 0.0)
    return float(np.max(np.hypot(poly[:, 0], poly[:, 1])))


def _arc_pad(radius: float, dtheta: float) -> float:
    """Sagitta: max distance a point at `radius` from a rotation's pivot can stray from the straight
    chord joining its positions at the two ends of a rotation spanning `dtheta` radians. Padding a
    per-interval swept hull by this (conservatively, using the footprint's max corner radius) turns a
    finite-sample rotation check into a proof that covers the whole continuous sweep in that interval."""
    return abs(radius) * (1.0 - math.cos(dtheta / 2.0))


def _inflate_hull(poly: np.ndarray, pad: float, n: int = 8) -> np.ndarray:
    """Conservative (superset) convex approximation of the Minkowski sum of `poly` with a disc of
    radius `pad`: circumscribe each vertex with a coarse n-gon that contains that disc, then re-hull.
    ponytail: an n-gon per vertex, not an exact rounded offset -- fine for the sub-mm/few-mm pads used
    here; revisit with true polygon offsetting if `pad` is ever large relative to `poly`."""
    if pad <= 0.0:
        return poly
    r = pad / math.cos(math.pi / n)
    extra = [(vx + r * math.cos(2 * math.pi * k / n), vy + r * math.sin(2 * math.pi * k / n))
             for vx, vy in poly for k in range(n)]
    return S.hull(np.vstack([poly, np.array(extra)]))


def pose_collides(fp, pose: Pose, obstacles: Sequence[Obstacle], board_w: float, board_h: float,
                   board_margin: float) -> bool:
    """True if the rover footprint at `pose` leaves the field (minus margin) or touches an obstacle.

    `fp` only needs an `.envelope(x, y, theta) -> Poly` method, so a `Footprint` or the internal
    per-heading cache both work here.
    """
    poly = fp.envelope(pose.x, pose.y, pose.theta)
    if _poly_out_of_board(poly, board_w, board_h, board_margin):
        return True
    return any(_poly_hits_obstacle(poly, o) for o in obstacles)


def _swept_hull(fp, start: Pose, end: Pose) -> np.ndarray:
    a = fp.envelope(start.x, start.y, start.theta)
    b = fp.envelope(end.x, end.y, end.theta)
    return S.hull(np.vstack([a, b]))


def segment_collides(fp, seg: Segment, obstacles: Sequence[Obstacle], board_w: float, board_h: float,
                      board_margin: float) -> bool:
    """Checks the whole swept motion of `seg`, not just its endpoints.

    ROTATE: split into <=5 deg intervals (module constant `_ROTATE_SAMPLE_RAD`); each interval's
    endpoint envelopes are hulled together (like a straight sweep) and the hit test is padded by that
    interval's arc/chord error (`_arc_pad`), so the check is a conservative proof for the *continuous*
    sweep, not just the sampled endpoints (a coarse sample can straddle a real contact -- see gate 5).
    STRAIGHT: exact -- the convex hull of the start and end envelope IS the swept region for a
    translation of a convex shape, no sampling needed.
    """
    if seg.kind is SegKind.ROTATE:
        dtheta = angle_diff(seg.end.theta, seg.start.theta)
        n = max(1, int(math.ceil(abs(dtheta) / _ROTATE_SAMPLE_RAD)))
        step = dtheta / n
        pad = _arc_pad(_fp_sweep_radius(fp), step)
        eff_margin = board_margin + pad
        prev = fp.envelope(seg.start.x, seg.start.y, seg.start.theta)
        for k in range(1, n + 1):
            th = wrap(seg.start.theta + step * k)
            cur = fp.envelope(seg.start.x, seg.start.y, th)
            poly = S.hull(np.vstack([prev, cur]))
            if _poly_out_of_board(poly, board_w, board_h, eff_margin):
                return True
            if any(_poly_hits_obstacle(poly, o, pad) for o in obstacles):
                return True
            prev = cur
        return False
    poly = _swept_hull(fp, seg.start, seg.end)
    if _poly_out_of_board(poly, board_w, board_h, board_margin):
        return True
    return any(_poly_hits_obstacle(poly, o) for o in obstacles)


def path_collides(fp, path: Path, obstacles: Sequence[Obstacle], board_w: float, board_h: float,
                   board_margin: float) -> bool:
    return any(segment_collides(fp, s, obstacles, board_w, board_h, board_margin) for s in path.segments)


def swept_polygons(fp, path: Path) -> list[np.ndarray]:
    """Conservative convex polygons covering the whole motion, one per segment (for reservations).

    Each returned polygon is a guaranteed SUPERSET of the true swept footprint -- required since other
    rovers' reservations are checked against it. ROTATE: hull of many fine-angle samples still misses
    the arc bulging past the chord between samples (see `_arc_pad`), so the sampled hull is additionally
    padded (`_inflate_hull`) by that interval's arc error before being returned.
    """
    polys = []
    for seg in path.segments:
        if seg.kind is SegKind.ROTATE:
            dtheta = angle_diff(seg.end.theta, seg.start.theta)
            n = max(1, int(math.ceil(abs(dtheta) / _SWEEP_SAMPLE_RAD)))
            step = dtheta / n
            pts = [fp.envelope(seg.start.x, seg.start.y, wrap(seg.start.theta + step * k))
                   for k in range(n + 1)]
            raw = S.hull(np.vstack(pts))
            pad = _arc_pad(_fp_sweep_radius(fp), step)
            polys.append(_inflate_hull(raw, pad))
        else:
            polys.append(_swept_hull(fp, seg.start, seg.end))
    return polys


# ---------------------------------------------------------------------------------- fast internal path
#
# The search loop calls the collision check ~1e4-1e5 times per query; numpy's per-call overhead on
# 4-point polygons dominates at that rate (measured: ~1s for a few hundred expansions). `pose_collides`
# / `segment_collides` above stay the simple, obviously-correct numpy version (used for the public API
# and to validate the final path). Internally NavPlanner instead uses this pure-python mirror, which a
# fuzz-equivalence test checks against the public functions. ponytail: code duplication between the two
# paths; justified only because it's a measured hot loop, not applied speculatively elsewhere.


def _fast_point_in(pts: list[tuple[float, float]], px: float, py: float) -> bool:
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) < -1e-9:
            return False
    return True


def _fast_seg_point_dist(x1: float, y1: float, x2: float, y2: float, px: float, py: float) -> float:
    abx, aby = x2 - x1, y2 - y1
    denom = abx * abx + aby * aby
    t = 0.0 if denom < 1e-15 else min(1.0, max(0.0, ((px - x1) * abx + (py - y1) * aby) / denom))
    return math.hypot(x1 + t * abx - px, y1 + t * aby - py)


def _fast_disc_hit(pts: list[tuple[float, float]], cx: float, cy: float, r: float, pad: float = 0.0) -> bool:
    if _fast_point_in(pts, cx, cy):
        return True
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if _fast_seg_point_dist(x1, y1, x2, y2, cx, cy) <= r + pad:
            return True
    return False


def _fast_overlap(A: list[tuple[float, float]], B: list[tuple[float, float]], pad: float = 0.0) -> bool:
    """True if convex A, B are within `pad` of touching (pad=0 -> plain SAT overlap)."""
    for P, Q in ((A, B), (B, A)):
        n = len(P)
        for i in range(n):
            x1, y1 = P[i]
            x2, y2 = P[(i + 1) % n]
            nx, ny = -(y2 - y1), x2 - x1
            L = math.hypot(nx, ny)
            if L < 1e-12:
                continue
            nx, ny = nx / L, ny / L
            amin = amax = P[0][0] * nx + P[0][1] * ny
            for (px, py) in P[1:]:
                d = px * nx + py * ny
                amin, amax = min(amin, d), max(amax, d)
            bmin = bmax = Q[0][0] * nx + Q[0][1] * ny
            for (px, py) in Q[1:]:
                d = px * nx + py * ny
                bmin, bmax = min(bmin, d), max(bmax, d)
            if amax < bmin - pad or bmax < amin - pad:
                return False
    return True


def _fast_hull(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Pure-python monotone-chain convex hull (mirrors `shapes.hull`), CCW. Kept python-native (no
    numpy round-trip) since it runs in the search's hot loop for every rotation interval."""
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo: list[tuple[float, float]] = []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    hi: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


class _FastCache:
    """Pure-python per-heading rotated envelope cache (see module note above)."""

    def __init__(self, fp: Footprint):
        self._local = [(float(p[0]), float(p[1])) for p in fp.local_envelope()]
        self.sweep_radius = max(math.hypot(lx, ly) for lx, ly in self._local)
        self._cache: dict[float, list[tuple[float, float]]] = {}

    def poly(self, x: float, y: float, th: float) -> list[tuple[float, float]]:
        # Exact float key: a rounded/quantized key can silently share one rotated polygon between two
        # headings that are actually distinct (e.g. 0 and 4e-6 rad), hiding a real sub-mm-scale contact
        # (gate 5 finding 9). Lattice headings repeat via identical arithmetic, so this still caches well.
        key = th
        rot = self._cache.get(key)
        if rot is None:
            c, s = math.cos(th), math.sin(th)
            rot = [(c * lx - s * ly, s * lx + c * ly) for lx, ly in self._local]
            self._cache[key] = rot
        return [(px + x, py + y) for px, py in rot]


def _fast_obstacles(obstacles: Sequence[Obstacle]):
    out = []
    for o in obstacles:
        if isinstance(o, DiscObstacle):
            out.append(("d", o.x, o.y, o.r))
        else:
            out.append(("p", [(float(p[0]), float(p[1])) for p in o.poly]))
    return out


def _fast_pose_collides(cache: _FastCache, pose: Pose, obstacles_fast, board_w: float, board_h: float,
                         board_margin: float) -> bool:
    poly = cache.poly(pose.x, pose.y, pose.theta)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    if min(xs) < board_margin or max(xs) > board_w - board_margin:
        return True
    if min(ys) < board_margin or max(ys) > board_h - board_margin:
        return True
    for o in obstacles_fast:
        if o[0] == "d":
            if _fast_disc_hit(poly, o[1], o[2], o[3]):
                return True
        elif _fast_overlap(poly, o[1]):
            return True
    return False


def _fast_segment_collides(cache: _FastCache, seg: Segment, obstacles_fast, board_w: float, board_h: float,
                            board_margin: float) -> bool:
    if seg.kind is SegKind.ROTATE:
        # Same conservative interval-hull + arc-pad treatment as the public `segment_collides` (kept in
        # sync deliberately -- both must accept/reject the same continuous sweep, see the fuzz-equivalence
        # test), just built on the pure-python primitives this hot path uses.
        dtheta = angle_diff(seg.end.theta, seg.start.theta)
        n = max(1, int(math.ceil(abs(dtheta) / _ROTATE_SAMPLE_RAD)))
        step = dtheta / n
        pad = _arc_pad(cache.sweep_radius, step)
        eff_margin = board_margin + pad
        prev = cache.poly(seg.start.x, seg.start.y, seg.start.theta)
        for k in range(1, n + 1):
            th = wrap(seg.start.theta + step * k)
            cur = cache.poly(seg.start.x, seg.start.y, th)
            hull_pts = _fast_hull(prev + cur)
            xs = [p[0] for p in hull_pts]
            ys = [p[1] for p in hull_pts]
            if min(xs) < eff_margin or max(xs) > board_w - eff_margin:
                return True
            if min(ys) < eff_margin or max(ys) > board_h - eff_margin:
                return True
            for o in obstacles_fast:
                if o[0] == "d":
                    if _fast_disc_hit(hull_pts, o[1], o[2], o[3], pad):
                        return True
                elif _fast_overlap(hull_pts, o[1], pad):
                    return True
            prev = cur
        return False
    a = cache.poly(seg.start.x, seg.start.y, seg.start.theta)
    b = cache.poly(seg.end.x, seg.end.y, seg.end.theta)
    poly = S.hull(np.array(a + b))
    if _poly_out_of_board(poly, board_w, board_h, board_margin):
        return True
    for o in obstacles_fast:
        if o[0] == "d":
            if S.disc_distance(poly, (o[1], o[2]), o[3]) <= 0.0:
                return True
        elif S.overlap(poly, np.array(o[1])):
            return True
    return False


def _clearance(fp0: Footprint, pose: Pose, obstacles: Sequence[Obstacle], board_w: float, board_h: float) -> float:
    """Escape-only metric (zero-inflate footprint): >0 free, 0 touching/at the edge. Not used for
    the main search, only to make the start-escape maneuver monotonically safer."""
    poly = fp0.envelope(pose.x, pose.y, pose.theta)
    edge = min(poly[:, 0].min(), board_w - poly[:, 0].max(), poly[:, 1].min(), board_h - poly[:, 1].max())
    obs = math.inf
    for o in obstacles:
        if isinstance(o, DiscObstacle):
            obs = min(obs, S.disc_distance(poly, (o.x, o.y), o.r))
        else:
            obs = min(obs, S.distance(poly, o.poly))
    return min(edge, obs)


# ---------------------------------------------------------------------------------------- planner


@dataclass(frozen=True)
class NavParams:
    """Module-local tunables (centralise on request; see final report)."""
    step_mm: float = 40.0                          # TUNED: 2x nav_cell_mm straight-primitive length
    reverse_penalty: float = 1.6                   # TUNED: time multiplier for reverse segments
    rotate_settle_s: float = 0.35                  # TUNED: fixed accel/decel+settle overhead / rotation
    goal_pos_tol_mm: float = 5.0                   # ASSUMED: capture needs precision (Gate 5 3/4): <=5mm
    goal_theta_tol_rad: float = math.radians(3.0)  # ASSUMED: capture needs precision (Gate 5 3/4): <=3deg
    escape_step_mm: float = 15.0                   # TUNED: step size for the start-in-collision escape
    escape_rotate_rad: float = math.radians(12.0)  # TUNED
    escape_max_steps: int = 12
    max_expansions: int | None = None              # None -> cfg.planner.nav_max_expansions
    analytic_every: int = 8                        # TUNED: try the (expensive) RTR connector every Nth
                                                    # expansion when far from the goal (always tried when
                                                    # close, see `_search`) -- most attempts far away fail
                                                    # (something is in the way), so trying every node just
                                                    # burns time without helping search progress.


@dataclass
class _Node:
    pose: Pose
    g: float
    parent: int
    segs: list[Segment]


def _segs_cost(segs: Sequence[Segment], cfg: Config, params: NavParams) -> float:
    total = 0.0
    for s in segs:
        if s.kind is SegKind.ROTATE:
            total += abs(angle_diff(s.end.theta, s.start.theta)) / cfg.limits.w_nav + params.rotate_settle_s
        else:
            c = s.length / cfg.limits.v_nav
            if s.reverse:
                c *= params.reverse_penalty
            total += c
    return total


class NavPlanner:
    """Hybrid A* planner from a start Pose to a goal Pose, honouring a wall-clock deadline."""

    def __init__(self, cfg: Config = DEFAULT, params: NavParams | None = None):
        self.cfg = cfg
        self.params = params or NavParams()
        self.last_failure: str = ""

    # -- public -------------------------------------------------------------------------------
    def plan(self, start: Pose, goal: Pose, obstacles: Sequence[Obstacle], board_w: float, board_h: float,
             inflate: float | None = None, deadline_s: float = 1.0) -> Path | None:
        t0 = time.monotonic()
        cfg, params = self.cfg, self.params
        self.last_failure = ""

        inflate = cfg.margins.pose_uncertainty if inflate is None else inflate
        board_margin = cfg.margins.board
        fp = Footprint(cfg.rover, inflate=inflate)
        fp0 = Footprint(cfg.rover, inflate=0.0)
        cache = _FastCache(fp)
        cache0 = _FastCache(fp0)
        obstacles_fast = _fast_obstacles(obstacles)

        if _fast_pose_collides(cache, goal, obstacles_fast, board_w, board_h, board_margin):
            self.last_failure = "goal pose in collision"
            return None

        prefix: list[Segment] = []
        search_start = start
        if _fast_pose_collides(cache, start, obstacles_fast, board_w, board_h, board_margin):
            if _fast_pose_collides(cache0, start, obstacles_fast, board_w, board_h, board_margin):
                self.last_failure = "start pose in collision (uninflated)"
                return None
            # One absolute deadline covers every phase (Gate 5 finding 7): check it before starting the
            # (potentially expensive) escape search rather than only after, so an already-expired budget
            # never even begins work it cannot finish.
            if time.monotonic() - t0 > deadline_s:
                self.last_failure = "deadline exceeded"
                return None
            escaped = self._escape(cache, cache0, fp0, start, obstacles, board_w, board_h, board_margin,
                                    deadline_s, t0)
            if escaped is None:
                if not self.last_failure:
                    self.last_failure = "start pose in inflated collision; escape failed"
                return None
            search_start, prefix = escaped

        if time.monotonic() - t0 > deadline_s:
            self.last_failure = "deadline exceeded"
            return None

        result = self._search(cache, search_start, goal, obstacles_fast, board_w, board_h, board_margin,
                               deadline_s, t0)
        if result is None:
            return None  # last_failure already set by _search

        segs = prefix + result
        if not segs:
            # Start already at goal (within tolerance): a well-formed, trivially-done path (`.goal`
            # reads the last segment's end), not a STRAIGHT segment carrying some arbitrary heading a
            # follower would try to track (Gate 5 finding 4) -- a zero-angle ROTATE is a no-op for any
            # follower/FSM that watches for start==end.
            segs = [Segment(SegKind.ROTATE, search_start, search_start)]
        path = Path(segments=segs, cost_s=_segs_cost(segs, cfg, params))
        path = self._smooth(cache, path, obstacles_fast, board_w, board_h, board_margin, deadline_s, t0)

        # Final validation with the PUBLIC (numpy) collision checker (Gate 5 finding 9): catches any
        # internal fast-path bug before a colliding path is ever handed back to a caller.
        if path_collides(fp, path, obstacles, board_w, board_h, board_margin):
            self.last_failure = "internal error: planned path failed public re-validation"
            return None
        return path

    # -- start-in-collision escape -------------------------------------------------------------
    def _escape_moves(self, p: Pose) -> list[tuple[Pose, Segment]]:
        step, ang = self.params.escape_step_mm, self.params.escape_rotate_rad
        fwd, rev = p.moved(step), p.moved(-step)
        rot_p, rot_m = p.rotated_to(wrap(p.theta + ang)), p.rotated_to(wrap(p.theta - ang))
        return [
            (fwd, Segment(SegKind.STRAIGHT, p, fwd, reverse=False)),
            (rev, Segment(SegKind.STRAIGHT, p, rev, reverse=True)),
            (rot_p, Segment(SegKind.ROTATE, p, rot_p)),
            (rot_m, Segment(SegKind.ROTATE, p, rot_m)),
        ]

    def _escape(self, cache: "_FastCache", cache0: "_FastCache", fp0: Footprint, pose: Pose,
                obstacles: Sequence[Obstacle], board_w: float, board_h: float, board_margin: float,
                deadline_s: float, t0: float) -> tuple[Pose, list[Segment]] | None:
        """Only reached when `pose` is free of real obstacles/board at zero inflation but still
        collides once inflated -- i.e. the collision is purely a safety-margin artifact (e.g. just
        retreated from a cube). Each candidate move must be genuinely collision-free at zero
        inflation (checked as a full swept segment, `cache0` -- the escape must never let the rover
        actually touch anything, only cross its own safety margin) and must strictly increase
        clearance, until the normal inflated check passes again."""
        obstacles_fast = _fast_obstacles(obstacles)
        start_clear = _clearance(fp0, pose, obstacles, board_w, board_h)
        frontier = [(pose, start_clear, [])]
        seen = {pose}
        for _ in range(self.params.escape_max_steps):
            if time.monotonic() - t0 > deadline_s:
                self.last_failure = "deadline exceeded"
                return None
            nxt = []
            for p, c, segs in frontier:
                for cand, seg in self._escape_moves(p):
                    if cand in seen:
                        continue
                    seen.add(cand)
                    if _fast_segment_collides(cache0, seg, obstacles_fast, board_w, board_h, board_margin):
                        continue
                    cc = _clearance(fp0, cand, obstacles, board_w, board_h)
                    if cc <= c + 1e-9:
                        continue
                    path_segs = segs + [seg]
                    if not _fast_pose_collides(cache, cand, obstacles_fast, board_w, board_h, board_margin):
                        return cand, path_segs
                    nxt.append((cand, cc, path_segs))
            if not nxt:
                return None
            frontier = nxt
        return None

    # -- analytic rotate-straight-rotate connector ---------------------------------------------
    def _analytic_connect(self, cache: "_FastCache", pose: Pose, goal: Pose, obstacles_fast, board_w: float,
                           board_h: float, board_margin: float) -> list[Segment] | None:
        """Try to connect `pose` directly to `goal` with rotate/straight/rotate primitives. Returns
        the cheapest collision-free candidate, or None: rotate to face `goal`, drive straight to it,
        rotate to `goal.theta` there (classic RTR; tried in both a forward-driving and a
        reverse-driving flavour). A straight segment is only ever produced between two poses that
        share a heading collinear with the displacement (each RTR variant's straight leg has both
        endpoints at the *same* theta, exactly along that heading -- never a shortcut that lets a
        differential-drive rover "strafe" a small lateral offset away, see Gate 5 finding 1).
        """
        eps = 1e-6
        dx, dy = goal.x - pose.x, goal.y - pose.y
        dist = math.hypot(dx, dy)
        candidates: list[list[Segment]] = []

        if dist <= eps:
            if abs(angle_diff(goal.theta, pose.theta)) > eps:
                candidates.append([Segment(SegKind.ROTATE, pose, goal)])
            else:
                candidates.append([])
        else:
            phi = math.atan2(dy, dx)
            for target_theta, rev in ((phi, False), (wrap(phi + math.pi), True)):
                segs: list[Segment] = []
                r1_end = pose
                if abs(angle_diff(target_theta, pose.theta)) > eps:
                    r1_end = pose.rotated_to(target_theta)
                    segs.append(Segment(SegKind.ROTATE, pose, r1_end))
                mid = Pose(goal.x, goal.y, target_theta)
                segs.append(Segment(SegKind.STRAIGHT, r1_end, mid, reverse=rev))
                if abs(angle_diff(goal.theta, target_theta)) > eps:
                    segs.append(Segment(SegKind.ROTATE, mid, goal))
                candidates.append(segs)

        best, best_cost = None, math.inf
        for segs in candidates:
            if any(_fast_segment_collides(cache, s, obstacles_fast, board_w, board_h, board_margin) for s in segs):
                continue
            c = _segs_cost(segs, self.cfg, self.params)
            if c < best_cost:
                best, best_cost = segs, c
        return best

    # -- lattice successors ---------------------------------------------------------------------
    def _successors(self, pose: Pose, goal: Pose, dtheta_bin: float) -> list[tuple[Pose, Segment]]:
        step = self.params.step_mm
        out = []
        for sign in (1.0, -1.0):
            end = pose.rotated_to(wrap(pose.theta + sign * dtheta_bin))
            out.append((end, Segment(SegKind.ROTATE, pose, end)))
        for sign in (1.0, -1.0):
            end = pose.moved(sign * step)
            out.append((end, Segment(SegKind.STRAIGHT, pose, end, reverse=sign < 0)))
        if abs(angle_diff(goal.theta, pose.theta)) > 1e-3:
            end = pose.rotated_to(goal.theta)
            out.append((end, Segment(SegKind.ROTATE, pose, end)))
        return out

    # -- Hybrid A* search -------------------------------------------------------------------------
    def _search(self, cache: "_FastCache", start: Pose, goal: Pose, obstacles_fast, board_w: float,
                board_h: float, board_margin: float, deadline_s: float, t0: float) -> list[Segment] | None:
        cfg, params = self.cfg, self.params
        dtheta_bin = 2.0 * math.pi / cfg.planner.nav_heading_bins
        cell = cfg.planner.nav_cell_mm
        max_exp = params.max_expansions or cfg.planner.nav_max_expansions

        def h(pose: Pose) -> float:
            # Tolerance-aware (Gate 5 finding 8): an accepted goal only needs to land within
            # goal_pos_tol_mm / goal_theta_tol_rad, so the remaining distance/angle still owed is
            # whatever exceeds that tolerance (floored at 0), not the distance to the exact goal pose --
            # otherwise h can overestimate the true cost of a legally-accepted (tolerance) goal, which
            # breaks A*'s admissibility guarantee.
            d = max(0.0, math.hypot(goal.x - pose.x, goal.y - pose.y) - params.goal_pos_tol_mm)
            dth = max(0.0, abs(angle_diff(goal.theta, pose.theta)) - params.goal_theta_tol_rad)
            return d / cfg.limits.v_nav + dth / cfg.limits.w_nav

        def key(pose: Pose) -> tuple[int, int, int]:
            return (round(pose.x / cell), round(pose.y / cell), round(pose.theta / dtheta_bin))

        def is_goal(pose: Pose) -> bool:
            return (math.hypot(goal.x - pose.x, goal.y - pose.y) <= params.goal_pos_tol_mm
                    and abs(angle_diff(goal.theta, pose.theta)) <= params.goal_theta_tol_rad)

        nodes: list[_Node] = [_Node(start, 0.0, -1, [])]
        openh: list[tuple[float, int, int]] = []
        seq = 0
        heapq.heappush(openh, (h(start), seq, 0))
        best_g = {key(start): 0.0}
        expansions = 0

        # Gate 5 finding 8: the first successful analytic (RTR) connection is kept as an INCUMBENT
        # rather than returned immediately -- a cheaper lattice path may still exist. Search continues
        # (bounded: it stops as soon as the open set's lower bound `f` can no longer beat the incumbent,
        # the standard admissible-heuristic proof of optimality) instead of accepting the first hit.
        incumbent: list[Segment] | None = None
        incumbent_cost = math.inf

        def consider(segs: list[Segment], cost: float) -> None:
            nonlocal incumbent, incumbent_cost
            if cost < incumbent_cost - 1e-9:
                incumbent, incumbent_cost = segs, cost

        if is_goal(start):
            consider([], 0.0)

        reason = "no path found"
        while openh:
            # One absolute deadline (Gate 5 finding 7): an expired budget is a hard failure, even with a
            # valid incumbent in hand -- the caller needed an answer (or a clear "no") by this time.
            if time.monotonic() - t0 > deadline_s:
                self.last_failure = "deadline exceeded"
                return None
            f, _, ni = heapq.heappop(openh)
            if incumbent is not None and f >= incumbent_cost - 1e-9:
                break
            node = nodes[ni]
            if node.g > best_g.get(key(node.pose), math.inf) + 1e-9:
                continue
            if is_goal(node.pose):
                consider(self._reconstruct(nodes, ni), node.g)
                continue

            expansions += 1
            if expansions > max_exp:
                reason = "max expansions exceeded"
                break

            near_goal = math.hypot(goal.x - node.pose.x, goal.y - node.pose.y) <= 3.0 * params.step_mm
            if near_goal or expansions == 1 or expansions % params.analytic_every == 0:
                connect = self._analytic_connect(cache, node.pose, goal, obstacles_fast, board_w, board_h,
                                                  board_margin)
                if connect is not None:
                    consider(self._reconstruct(nodes, ni) + connect, node.g + _segs_cost(connect, cfg, params))

            for succ_pose, seg in self._successors(node.pose, goal, dtheta_bin):
                if _fast_segment_collides(cache, seg, obstacles_fast, board_w, board_h, board_margin):
                    continue
                if seg.kind is SegKind.ROTATE:
                    cost = abs(angle_diff(seg.end.theta, seg.start.theta)) / cfg.limits.w_nav + params.rotate_settle_s
                else:
                    cost = seg.length / cfg.limits.v_nav
                    if seg.reverse:
                        cost *= params.reverse_penalty
                gg = node.g + cost
                kk = key(succ_pose)
                if gg + 1e-9 >= best_g.get(kk, math.inf):
                    continue
                if incumbent is not None and gg + h(succ_pose) >= incumbent_cost - 1e-9:
                    continue
                best_g[kk] = gg
                nodes.append(_Node(succ_pose, gg, ni, [seg]))
                seq += 1
                heapq.heappush(openh, (gg + h(succ_pose), seq, len(nodes) - 1))

        if incumbent is not None:
            return incumbent
        self.last_failure = reason
        return None

    def _reconstruct(self, nodes: list[_Node], idx: int) -> list[Segment]:
        segs: list[Segment] = []
        while idx != -1:
            node = nodes[idx]
            segs = list(node.segs) + segs
            idx = node.parent
        return segs

    # -- post-search shortcutting -----------------------------------------------------------------
    def _smooth(self, cache: "_FastCache", path: Path, obstacles_fast, board_w: float, board_h: float,
                board_margin: float, deadline_s: float, t0: float) -> Path:
        segs = path.segments
        i = 0
        while i < len(segs):
            if time.monotonic() - t0 > deadline_s:
                # Shortcutting is a bounded, optional optimisation over an already-valid path (Gate 5
                # finding 7): stop improving rather than blow the deadline, don't fail the whole plan.
                break
            improved = False
            j = len(segs) - 1
            while j > i + 1:
                connect = self._analytic_connect(cache, segs[i].start, segs[j].end, obstacles_fast, board_w,
                                                  board_h, board_margin)
                if connect is not None:
                    old_cost = _segs_cost(segs[i:j + 1], self.cfg, self.params)
                    new_cost = _segs_cost(connect, self.cfg, self.params)
                    if new_cost < old_cost - 1e-6:
                        segs = segs[:i] + connect + segs[j + 1:]
                        improved = True
                        break
                j -= 1
            if not improved:
                i += 1
        return Path(segments=segs, cost_s=_segs_cost(segs, self.cfg, self.params))
