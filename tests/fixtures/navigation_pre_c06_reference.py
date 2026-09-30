"""Collision-free navigation planner for a single rover with an asymmetric footprint.

Design choice: Hybrid A* over rotate, straight, and exact constant-curvature ARC primitives,
plus analytic terminal connectors and post-hoc shortcutting. It is tied to THIS rover's actual
motion model (in-place rotation about the axle midpoint, straight forward/reverse, and ARC wheel
speeds):

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
* Hybrid A* (continuous (x, y, theta) state, closed set discretised onto
  (nav_cell_mm, 360/nav_heading_bins) purely to bound re-expansion) with analytic terminal
  connectors and post-search shortcutting: this matches the primitive set executed by the rover
  and keeps the goal pose exact.

Cost model: cost_s is expected execution time from config.limits (v_nav, w_nav): STRAIGHT and ARC
time is length / v_nav, and ROTATE time is angle / w_nav plus NavParams.rotate_settle_s for every
nonzero rotation.

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
# ARC samples use the same angular resolution.  The padding below is a proof bound, not
# an empirical clearance fudge: between two samples every footprint point follows a circle.
_ARC_SAMPLE_RAD = math.radians(5.0)
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


def _arc_pose(seg: Segment, distance: float) -> Pose:
    """Pose after ``distance`` mm on an ARC (including reverse arcs).

    ``curvature`` is signed with respect to the direction of travel.  The body heading
    therefore changes by ``curvature * distance`` for both forward and reverse motion;
    only the translational tangent is shifted by pi for reverse motion.
    """
    k = seg.curvature
    if abs(k) <= 1e-12:
        return seg.start.moved(-distance if seg.reverse else distance)
    q0 = seg.start.theta + (math.pi if seg.reverse else 0.0)
    a = k * distance
    q = q0 + a
    x = seg.start.x + (math.sin(q) - math.sin(q0)) / k
    y = seg.start.y + (-math.cos(q) + math.cos(q0)) / k
    return Pose(x, y, wrap(seg.start.theta + a))


def _arc_center(seg: Segment) -> tuple[float, float]:
    """Return the fixed centre of the rotation of an ARC, in world coordinates."""
    q0 = seg.start.theta + (math.pi if seg.reverse else 0.0)
    k = seg.curvature
    return (seg.start.x - math.sin(q0) / k,
            seg.start.y + math.cos(q0) / k)


def _arc_sweep_radius(fp, seg: Segment) -> float:
    """Bound the radius of every footprint point about an ARC's rotation centre."""
    return abs(1.0 / seg.curvature) + _fp_sweep_radius(fp)


def _make_arc(start: Pose, curvature: float, length: float, reverse: bool = False) -> Segment:
    """Construct an exact ARC segment from its signed curvature and positive length."""
    probe = Segment(SegKind.ARC, start, start, reverse=reverse, curvature=curvature)
    end = _arc_pose(probe, length)
    return Segment(SegKind.ARC, start, end, reverse=reverse, curvature=curvature)


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
    if seg.kind is SegKind.ARC:
        dtheta = angle_diff(seg.end.theta, seg.start.theta)
        if abs(seg.curvature) <= 1e-12 or abs(dtheta) <= 1e-12:
            return pose_collides(fp, seg.start, obstacles, board_w, board_h, board_margin)
        n = max(1, int(math.ceil(abs(dtheta) / _ARC_SAMPLE_RAD)))
        step = dtheta / n
        pad = _arc_pad(_arc_sweep_radius(fp, seg), step)
        prev = fp.envelope(seg.start.x, seg.start.y, seg.start.theta)
        for i in range(1, n + 1):
            cur = _arc_pose(seg, seg.length * i / n)
            cur_poly = fp.envelope(cur.x, cur.y, cur.theta)
            poly = S.hull(np.vstack([prev, cur_poly]))
            if _poly_out_of_board(poly, board_w, board_h, board_margin + pad):
                return True
            if any(_poly_hits_obstacle(poly, o, pad) for o in obstacles):
                return True
            prev = cur_poly
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
        elif seg.kind is SegKind.ARC:
            dtheta = angle_diff(seg.end.theta, seg.start.theta)
            if abs(seg.curvature) <= 1e-12 or abs(dtheta) <= 1e-12:
                polys.append(fp.envelope(seg.start.x, seg.start.y, seg.start.theta))
                continue
            n = max(1, int(math.ceil(abs(dtheta) / _ARC_SAMPLE_RAD)))
            step = dtheta / n
            poses = [_arc_pose(seg, seg.length * k / n) for k in range(n + 1)]
            raw = S.hull(np.vstack([fp.envelope(p.x, p.y, p.theta) for p in poses]))
            # Each interval's true sweep lies within its endpoint hull plus this sagitta.
            # Taking the maximum pad makes the single returned reservation polygon a superset.
            pad = _arc_pad(_arc_sweep_radius(fp, seg), step)
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


def _poly_axes(P: list[tuple[float, float]]) -> list[tuple[float, float]]:
    axes = []
    n = len(P)
    for i in range(n):
        x1, y1 = P[i]
        x2, y2 = P[(i + 1) % n]
        nx, ny = -(y2 - y1), x2 - x1
        L = math.hypot(nx, ny)
        if L > 1e-12:
            axes.append((nx / L, ny / L))
    return axes


def _fast_overlap(A: list[tuple[float, float]], B: list[tuple[float, float]], pad: float = 0.0,
                   axes_b: list[tuple[float, float]] | None = None) -> bool:
    """True if convex A, B are within `pad` of touching (pad=0 -> plain SAT overlap). `axes_b`, when
    given, is B's precomputed edge-normal axes (obstacles are static within a `plan()` call -- no need
    to re-derive and re-normalise the same normals on every one of the ~1e4-1e5 hot-loop checks)."""
    axes = _poly_axes(A) + (list(axes_b) if axes_b is not None else _poly_axes(B))
    for nx, ny in axes:
        amin = amax = A[0][0] * nx + A[0][1] * ny
        for (px, py) in A[1:]:
            d = px * nx + py * ny
            amin, amax = min(amin, d), max(amax, d)
        bmin = bmax = B[0][0] * nx + B[0][1] * ny
        for (px, py) in B[1:]:
            d = px * nx + py * ny
            bmin, bmax = min(bmin, d), max(bmax, d)
        if amax < bmin - pad or bmax < amin - pad:
            return False
    return True


def _fast_hull(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Pure-python monotone-chain convex hull (mirrors `shapes.hull`), CCW. Kept python-native (no
    numpy round-trip) since it runs in the search's hot loop for every rotation interval."""
    pts.sort()
    unique: list[tuple[float, float]] = []
    for point in pts:
        if not unique or point != unique[-1]:
            unique.append(point)
    if len(unique) <= 2:
        return unique

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo: list[tuple[float, float]] = []
    for p in unique:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    hi: list[tuple[float, float]] = []
    for p in reversed(unique):
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
    """Precompute what stays constant across the ~1e4-1e5 hot-loop checks in one `plan()` call: for a
    poly obstacle, its bounding circle (cheap reject before a full SAT test, see `_obstacle_far`) and
    its edge-normal axes (so `_fast_overlap` never re-derives them)."""
    out = []
    for o in obstacles:
        if isinstance(o, DiscObstacle):
            out.append(("d", o.x, o.y, o.r))
        else:
            pts = [(float(p[0]), float(p[1])) for p in o.poly]
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            rb = max(math.hypot(px - cx, py - cy) for px, py in pts)
            out.append(("p", pts, cx, cy, rb, _poly_axes(pts)))
    return out


def _obstacle_far(cx: float, cy: float, reach: float, o, pad: float = 0.0) -> bool:
    """Cheap bounding-circle reject: True if `o` cannot possibly be within `pad` of a shape all of
    whose points are within `reach` of (cx, cy). Skips the full disc/SAT test for obstacles nowhere
    near the current sweep -- the common case with many other-rover reservation polygons."""
    ocx, ocy, orb = (o[1], o[2], o[3]) if o[0] == "d" else (o[2], o[3], o[4])
    return math.hypot(cx - ocx, cy - ocy) > reach + orb + pad


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
        if _obstacle_far(pose.x, pose.y, cache.sweep_radius, o):
            continue
        if o[0] == "d":
            if _fast_disc_hit(poly, o[1], o[2], o[3]):
                return True
        elif _fast_overlap(poly, o[1], axes_b=o[5]):
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
        # Whole-rotation early-out: every swept point lies within sweep_radius of the centre.
        r_all = cache.sweep_radius + pad
        cx0, cy0 = seg.start.x, seg.start.y
        near = [o for o in obstacles_fast if not _obstacle_far(cx0, cy0, cache.sweep_radius, o, pad)]
        if not near and (cx0 - r_all >= board_margin and cx0 + r_all <= board_w - board_margin
                         and cy0 - r_all >= board_margin and cy0 + r_all <= board_h - board_margin):
            return False
        obstacles_fast = near
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
                if _obstacle_far(seg.start.x, seg.start.y, cache.sweep_radius, o, pad):
                    continue
                if o[0] == "d":
                    if _fast_disc_hit(hull_pts, o[1], o[2], o[3], pad):
                        return True
                elif _fast_overlap(hull_pts, o[1], pad, axes_b=o[5]):
                    return True
            prev = cur
        return False
    if seg.kind is SegKind.ARC:
        dtheta = angle_diff(seg.end.theta, seg.start.theta)
        if abs(seg.curvature) <= 1e-12 or abs(dtheta) <= 1e-12:
            return _fast_pose_collides(cache, seg.start, obstacles_fast, board_w, board_h, board_margin)
        n = max(1, int(math.ceil(abs(dtheta) / _ARC_SAMPLE_RAD)))
        step = dtheta / n
        pad = _arc_pad(_arc_sweep_radius(cache, seg), step)
        cx, cy = _arc_center(seg)
        reach = _arc_sweep_radius(cache, seg) + pad
        if (not obstacles_fast and cx - reach >= board_margin and cx + reach <= board_w - board_margin
                and cy - reach >= board_margin and cy + reach <= board_h - board_margin):
            return False
        near = [o for o in obstacles_fast if not _obstacle_far(cx, cy, reach, o)]
        prev_pose = seg.start
        prev = cache.poly(prev_pose.x, prev_pose.y, prev_pose.theta)
        for i in range(1, n + 1):
            cur_pose = _arc_pose(seg, seg.length * i / n)
            cur = cache.poly(cur_pose.x, cur_pose.y, cur_pose.theta)
            hull_pts = _fast_hull(prev + cur)
            xs = [p[0] for p in hull_pts]
            ys = [p[1] for p in hull_pts]
            if (min(xs) < board_margin + pad or max(xs) > board_w - board_margin - pad
                    or min(ys) < board_margin + pad or max(ys) > board_h - board_margin - pad):
                return True
            for o in near:
                if o[0] == "d":
                    if _fast_disc_hit(hull_pts, o[1], o[2], o[3], pad):
                        return True
                elif _fast_overlap(hull_pts, o[1], pad, axes_b=o[5]):
                    return True
            prev = cur
        return False
    a = cache.poly(seg.start.x, seg.start.y, seg.start.theta)
    b = cache.poly(seg.end.x, seg.end.y, seg.end.theta)
    poly = _fast_hull(a + b)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    if min(xs) < board_margin or max(xs) > board_w - board_margin:
        return True
    if min(ys) < board_margin or max(ys) > board_h - board_margin:
        return True
    mx, my = (seg.start.x + seg.end.x) / 2.0, (seg.start.y + seg.end.y) / 2.0
    reach = cache.sweep_radius + seg.length / 2.0
    for o in obstacles_fast:
        if _obstacle_far(mx, my, reach, o):
            continue
        if o[0] == "d":
            if _fast_disc_hit(poly, o[1], o[2], o[3]):
                return True
        elif _fast_overlap(poly, o[1], axes_b=o[5]):
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
    analytic_every: int = 32                       # TUNED: terminal connectors are costlier with ARC candidates
                                                    # expansion when far from the goal (always tried when
                                                    # close, see `_search`) -- most attempts far away fail
                                                    # (something is in the way), so trying every node just
                                                    # burns time without helping search progress.
    arc_radii_mm: tuple[float, ...] = (120.0, 200.0, 350.0)
    arc_step_mm: float = 40.0                      # bounded translational lattice increment


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
            dtheta = abs(angle_diff(s.end.theta, s.start.theta))
            # A zero-angle ROTATE is the "already at goal" no-op marker (Gate 5 finding 4): it costs
            # nothing to execute, so it must not carry the per-rotation settle overhead either.
            settle = 0.0 if dtheta <= 1e-9 else params.rotate_settle_s
            total += dtheta / cfg.limits.w_nav + settle
        else:
            # Both signs are commanded at the same navigation speed.  In particular an ARC's
            # execution time is its analytic arc length / v_nav, never its chord or a micro-segment
            # approximation.
            total += s.length / cfg.limits.v_nav
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
        # Reserve a small return/validation tail so a hard deadline is not exceeded by the
        # caller-visible cleanup after a hot-loop collision check.
        work_deadline_s = max(0.0, deadline_s - 0.01)
        cfg, params = self.cfg, self.params
        self.last_failure = ""

        explicit_inflate = inflate is not None
        inflate = cfg.margins.pose_uncertainty if inflate is None else inflate
        # An explicit zero-inflation query is the geometry/debug contract used by the
        # physical swept-footprint checks: enforce the actual board boundary, while the
        # normal production query retains the configured planning margin.
        board_margin = 0.0 if explicit_inflate and inflate <= 0.0 else cfg.margins.board
        fp = Footprint(cfg.rover, inflate=inflate)
        fp0 = Footprint(cfg.rover, inflate=0.0)
        cache = _FastCache(fp)
        cache0 = _FastCache(fp0)
        obstacles_fast = _fast_obstacles(obstacles)

        # A valid pose can sit inside the safety band while neither small in-place turn is
        # valid there (the common approach to a low board edge).  Let the curved lattice move
        # out of that band, but only for an obstacle-free query and only when the physical,
        # zero-margin turns are genuinely available.
        if (board_margin > 0.0 and not obstacles_fast
                and not _fast_pose_collides(cache0, start, [], board_w, board_h, 0.0)):
            dth = 2.0 * math.pi / cfg.planner.nav_heading_bins
            turns_blocked = any(
                _fast_segment_collides(cache, Segment(SegKind.ROTATE, start,
                                                       start.rotated_to(start.theta + sign * dth)),
                                       [], board_w, board_h, board_margin)
                for sign in (-1.0, 1.0))
            if turns_blocked:
                board_margin = 0.0

        if _fast_pose_collides(cache, goal, obstacles_fast, board_w, board_h, board_margin):
            self.last_failure = "goal pose in collision"
            return None

        prefix: list[Segment] = []
        search_start = start
        if _fast_pose_collides(cache, start, obstacles_fast, board_w, board_h, board_margin):
            # Zero board margin here too (not the safety `board_margin`): a start pose within the
            # margin of the physical edge, but still on the actual field, is a safety-margin artifact
            # like any inflated-obstacle-only collision, not a real collision escape can't fix.
            if _fast_pose_collides(cache0, start, obstacles_fast, board_w, board_h, 0.0):
                self.last_failure = "start pose in collision (uninflated)"
                return None
            # One absolute deadline covers every phase (Gate 5 finding 7): check it before starting the
            # (potentially expensive) escape search rather than only after, so an already-expired budget
            # never even begins work it cannot finish.
            if time.monotonic() - t0 > work_deadline_s:
                self.last_failure = "deadline exceeded"
                return None
            escaped = self._escape(cache, cache0, fp0, start, obstacles, board_w, board_h, board_margin,
                                    work_deadline_s, t0)
            if escaped is None:
                if not self.last_failure:
                    self.last_failure = "start pose in inflated collision; escape failed"
                return None
            search_start, prefix = escaped

        if time.monotonic() - t0 > work_deadline_s:
            self.last_failure = "deadline exceeded"
            return None

        result = self._search(cache, search_start, goal, obstacles_fast, board_w, board_h, board_margin,
                               work_deadline_s, t0)
        if result is None:
            return None  # last_failure already set by _search

        prefix_len = len(prefix)
        segs = prefix + result
        if not segs:
            # Start already at goal (within tolerance): a well-formed, trivially-done path (`.goal`
            # reads the last segment's end), not a STRAIGHT segment carrying some arbitrary heading a
            # follower would try to track (Gate 5 finding 4) -- a zero-angle ROTATE is a no-op for any
            # follower/FSM that watches for start==end.
            segs = [Segment(SegKind.ROTATE, search_start, search_start)]
        path = Path(segments=segs, cost_s=_segs_cost(segs, cfg, params))
        path = self._smooth(cache, path, obstacles_fast, board_w, board_h, board_margin, work_deadline_s, t0,
                             prefix_len)

        # Final validation with the PUBLIC (numpy) collision checker (Gate 5 finding 9): catches any
        # internal fast-path bug before a colliding path is ever handed back to a caller. The escape
        # prefix (if any, always the leading `prefix_len` segments -- smoothing never reaches into it,
        # see `_smooth`) only ever promised zero-inflation/zero-board-margin safety, not the normal
        # safety-margined one, so it's re-validated against that same, weaker contract.
        prefix_segs, rest_segs = path.segments[:prefix_len], path.segments[prefix_len:]
        if any(segment_collides(fp0, s, obstacles, board_w, board_h, 0.0) for s in prefix_segs):
            self.last_failure = "internal error: escape path failed public re-validation"
            return None
        if any(segment_collides(fp, s, obstacles, board_w, board_h, board_margin) for s in rest_segs):
            self.last_failure = "internal error: planned path failed public re-validation"
            return None
        return path

    # -- start-in-collision escape -------------------------------------------------------------
    # TUNED: a real obstacle can sit close enough that only a sub-degree/sub-mm move is genuinely
    # collision-free and clearance-improving (a full-size move's continuous sweep grazes it even
    # though both endpoints look fine) -- see Gate 5 "escape drives into a real cube". Offering a
    # ladder of shrinking magnitudes lets the escape fall back to a fine step when the coarse one is
    # blocked, while still preferring the coarse (faster-converging) one when it's actually safe.
    _ESCAPE_SCALES = (1.0, 1.0 / 4.0, 1.0 / 16.0, 1.0 / 64.0)

    def _escape_moves(self, p: Pose) -> list[tuple[Pose, Segment]]:
        out = []
        for scale in self._ESCAPE_SCALES:
            step, ang = self.params.escape_step_mm * scale, self.params.escape_rotate_rad * scale
            fwd, rev = p.moved(step), p.moved(-step)
            rot_p, rot_m = p.rotated_to(wrap(p.theta + ang)), p.rotated_to(wrap(p.theta - ang))
            out += [
                (fwd, Segment(SegKind.STRAIGHT, p, fwd, reverse=False)),
                (rev, Segment(SegKind.STRAIGHT, p, rev, reverse=True)),
                (rot_p, Segment(SegKind.ROTATE, p, rot_p)),
                (rot_m, Segment(SegKind.ROTATE, p, rot_m)),
            ]
        return out

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
                    # Zero board margin (not `board_margin`): the escape only has to never truly cross
                    # the physical edge, exactly like it only has to never truly touch an obstacle.
                    if _fast_segment_collides(cache0, seg, obstacles_fast, board_w, board_h, 0.0):
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

    def _arc_terminal_connect(self, cache: "_FastCache", pose: Pose, goal: Pose, obstacles_fast,
                              board_w: float, board_h: float, board_margin: float) -> list[list[Segment]]:
        """Enumerate exact ARC-to-goal-heading connectors.

        The arc ends on the infinite line through the goal with the requested terminal
        heading.  A rotate at that clear point sets the heading, followed by a collinear
        STRAIGHT leg.  This is the useful edge case where rotating at the goal would sweep
        outside the board.
        """
        candidates: list[list[Segment]] = []
        for reverse in (False, True):
            q0 = pose.theta + (math.pi if reverse else 0.0)
            for radius in self.params.arc_radii_mm:
                for turn in (-1.0, 1.0):
                    k = turn / radius
                    a_limit = turn * math.pi

                    def line_error(a: float) -> float:
                        probe = _make_arc(pose, k, abs(a / k), reverse)
                        dx, dy = goal.x - probe.end.x, goal.y - probe.end.y
                        return dx * math.sin(goal.theta) - dy * math.cos(goal.theta)

                    # Find all sign-changing roots.  This fixed small scan is deterministic;
                    # collision checking, not numerical resolution, remains the safety gate.
                    samples = 32
                    roots: list[float] = []
                    prev_a = 0.0
                    prev_f = line_error(prev_a)
                    for i in range(1, samples + 1):
                        a = a_limit * i / samples
                        f = line_error(a)
                        if abs(f) <= 1e-7 and abs(a) > 1e-6:
                            roots.append(a)
                        elif prev_f * f < 0.0:
                            lo, hi = prev_a, a
                            flo = prev_f
                            for _ in range(48):
                                mid = (lo + hi) / 2.0
                                fm = line_error(mid)
                                if flo * fm <= 0.0:
                                    hi = mid
                                else:
                                    lo, flo = mid, fm
                            roots.append((lo + hi) / 2.0)
                        prev_a, prev_f = a, f

                    for alpha in roots:
                        length = abs(alpha / k)
                        if length < 1e-3:
                            continue
                        arc = _make_arc(pose, k, length, reverse)
                        tx, ty = goal.x - arc.end.x, goal.y - arc.end.y
                        along = tx * math.cos(goal.theta) + ty * math.sin(goal.theta)
                        straight_start = arc.end.rotated_to(goal.theta)
                        straight_end = Pose(goal.x, goal.y, goal.theta)
                        assert abs(straight_start.to_local(straight_end.x, straight_end.y)[1]) <= 1e-4
                        segs: list[Segment] = [arc, Segment(SegKind.ROTATE, arc.end, straight_start)]
                        if abs(along) > 1e-5:
                            segs.append(Segment(SegKind.STRAIGHT, straight_start, straight_end,
                                                reverse=along < 0.0))
                        if all(not _fast_segment_collides(cache, s, obstacles_fast, board_w, board_h,
                                                          board_margin) for s in segs):
                            candidates.append(segs)
        return candidates

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
        # The arc terminal enumeration is intentionally lazy: an ordinary RTR connector is
        # both cheaper to compute and sufficient in open space.
        if best is None and not obstacles_fast:
            arc_candidates = self._arc_terminal_connect(cache, pose, goal, obstacles_fast,
                                                        board_w, board_h, board_margin)
            for segs in arc_candidates:
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
        # Constant-curvature lattice moves are emitted as one ARC each.  Radius choices provide
        # useful manoeuvrability without coordinate/layout special cases.
        for reverse in (False, True):
            for radius in self.params.arc_radii_mm:
                for turn in (-1.0, 1.0):
                    curvature = turn / radius
                    arc = _make_arc(pose, curvature, self.params.arc_step_mm, reverse)
                    out.append((arc.end, arc))
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

            near_goal = math.hypot(goal.x - node.pose.x, goal.y - node.pose.y) <= 2.0 * params.step_mm
            if expansions == 1 or expansions % params.analytic_every == 0 or (near_goal and expansions % 8 == 0):
                connect = self._analytic_connect(cache, node.pose, goal, obstacles_fast, board_w, board_h,
                                                  board_margin)
                if connect is not None:
                    consider(self._reconstruct(nodes, ni) + connect, node.g + _segs_cost(connect, cfg, params))

            successors = self._successors(node.pose, goal, dtheta_bin)
            regular = [(p, s) for p, s in successors if s.kind is not SegKind.ARC]
            arcs = [(p, s) for p, s in successors if s.kind is SegKind.ARC]
            regular_free = [(p, s) for p, s in regular
                            if not _fast_segment_collides(cache, s, obstacles_fast, board_w, board_h,
                                                          board_margin)]
            # In open space and around ordinary obstacles the rotate/straight lattice is much
            # cheaper and remains the preferred route.  Curved successors are activated when no
            # in-place rotation is feasible at this state (the board-edge case that motivated
            # ARC support), while `_successors` still exposes the complete forward/reverse set.
            has_free_rotation = any(s.kind is SegKind.ROTATE for _, s in regular_free)
            selected = regular_free if has_free_rotation else regular_free + arcs
            for succ_pose, seg in selected:
                if seg.kind is SegKind.ARC and _fast_segment_collides(cache, seg, obstacles_fast, board_w,
                                                                       board_h, board_margin):
                    continue
                if seg.kind is SegKind.ROTATE:
                    cost = abs(angle_diff(seg.end.theta, seg.start.theta)) / cfg.limits.w_nav + params.rotate_settle_s
                else:
                    cost = seg.length / cfg.limits.v_nav
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
                board_margin: float, deadline_s: float, t0: float, prefix_len: int = 0) -> Path:
        segs = path.segments
        # Never shortcut starting from within the escape prefix (`prefix_len` leading segments): those
        # poses are only promised zero-inflation-safe, but `_analytic_connect` checks candidates against
        # the normal inflated `cache` -- mixing the two contracts would let a still-margin-violating
        # escape pose get silently absorbed into a "safety-margin-clean" shortcut.
        i = prefix_len
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
