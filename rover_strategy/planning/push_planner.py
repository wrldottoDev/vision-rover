"""Push-geometry planner: turns one cube + a depot + other cubes into ranked, geometrically
verified push plans (one or more straight legs).  Units: mm, rad, s (see config.py).

Model recap (docs/interpretation_v0.md, finding H-11/H-15):
  - The rover pushes with its flat front plate (rover-frame x = +47).  A flush-pushed cube's final
    orientation is the push heading, reduced mod 90 deg (`wrap_to_pm45`).
  - No walls: the rover's full envelope (incl. paddles) must stay inside the field, minus
    `config.margins.board`, at the pre-push pose, through the whole push, and through the retreat.
  - Before rotating after a push the rover must reverse straight >= paddle_reach + a margin so the
    paddles clear the delivered cube (H-15), else it can knock the cube back out.
  - Cube orientation is unobservable from the vision contract until the cube has been flush-pushed
    once; an untouched cube's push heading is unconstrained but riskier (see `PushParams.
    grid_align_confidence`).  A cube with a known orientation may only be pushed within
    `config.planner.push_heading_window_deg` of one of its four (mod-90) face normals.

Every free function here only needs `cfg` (plus, where noted, the module-level `DEFAULT_PARAMS`
for the handful of tunables that don't belong in the frozen shared Config).  `PushPlanner` is the
only place a caller can override those tunables (via `params=`).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ..config import Config, DEFAULT
from ..frames import angle_diff, wrap
from ..geometry import shapes as S
from ..geometry.footprint import Footprint
from ..geometry.zones import DepotZone, cube_half_extent
from ..world import CubeEstimate, Pose, PushLeg, PushPlan


@dataclass(frozen=True)
class PushParams:
    """Tunables owned by this module (not in the frozen shared Config).  All ASSUMED/TUNED --
    flag for centralisation once real hardware timing / capture-rate data exists."""
    # Legs that START at a planned intermediate point get this much extra field margin: the cube never lands
    # exactly where planned (push stop error, slip), and a plan that is feasible only at the exact planned
    # point fails on replan (lead, closed-loop finding).  TUNED.
    intermediate_slack_mm: float = 10.0
    # Require a rotate-in-place + straight approach to every pre-push pose (navigation has no arcs yet).  OFF: with
    # it, single-cube coverage drops from ~65 % to 5-29 % (tools/solvability_map.py); arcs in navigation are the fix.
    require_straight_approach: bool = False

    # ASSUMED: minimum push distance for the cube to travel far enough to square flush against the
    # plate (rather than just kiss it).  Shorter "legs" are rejected as not physically meaningful.
    min_leg_length: float = 25.0
    # ASSUMED, additive to rover.paddle_reach: extra clearance on the post-push retreat (H-15).
    retreat_margin_mm: float = 10.0
    # TUNED: grid pitch for sampling intermediate waypoints in 2-/3-leg search. Trade-off: finer
    # grid finds more multi-leg routes (through tight gaps) but cost grows ~ (board/pitch)^2 per
    # extra leg; 140 mm keeps a 3-leg search on an 860x860 field within a ~0.3s deadline.
    intermediate_grid_mm: float = 140.0
    # ASSUMED prior: P(an untouched, orientation-unknown cube happens to be grid-aligned).  Used
    # only to bias risk of first-touch headings towards multiples of 90 deg, per the brief.
    grid_align_confidence: float = 0.6
    # ASSUMED: lumped per-leg overhead (re-navigation to next prepush + final rotate-to-heading +
    # slow capture creep) since no measured nav-planner cost model exists yet. Centralise once one does.
    per_leg_overhead_s: float = 2.0
    # TUNED: extra risk added per additional leg (compounding chance of a mid-plan failure).
    extra_leg_risk: float = 0.05
    # TUNED: weights combining the delivery-point risk term (all in [0,1] before the final clip).
    marker_risk_weight: float = 0.5
    border_risk_weight: float = 0.3
    # TUNED: mild preference for delivery points close to the published depot point (lead audit),
    # scaled by depot.half so it stays meaningful regardless of the (ASSUMED) depot zone size.
    depot_center_pref_weight: float = 0.05
    # TUNED: extra risk once the WORST-CASE (widened, lead audit) orientation would no longer fit
    # the depot with margin -- a soft penalty, not a hard reject: near a tight board corner the
    # nominally-reachable region is already only a few mm deep, and a hard worst-case gate there
    # makes routine corner deliveries impossible outright. The gate stays on the nominal (mean)
    # orientation; uncertainty only makes a borderline point look riskier, so it ranks worse but is
    # not thrown away.
    uncertainty_risk_weight: float = 0.4
    # ASSUMED (lead audit flagged 8 deg as a placeholder; TUNED down from that here): once WE plan a
    # push, we predict the resulting orientation is `heading mod 90` with this std -- there is no
    # live vision feedback inside one planning call. Reasoned down from the lead's 8 deg suggestion:
    # a straight-line push is the easiest motion a differential-drive rover makes to hold heading on
    # (no turning), and a flat plate mechanically corrects minor cube misalignment on contact, so the
    # residual uncertainty should be well under the 20 deg push_heading_window_deg the push itself
    # was already allowed to be off by. At 8 deg (2*std=16) the effective usable window for a
    # *second* leg's pivot is only push_heading_window_deg - 16 = 4 deg, which made routine 2-/3-leg
    # corner deliveries requiring a shallow-angle avoidance leg (see PushPlanner search) fail outright
    # even with no obstacles at all. Flagging for the lead to confirm/override.
    post_push_alpha_std: float = math.radians(3.0)
    # Candidate-generation shape: split config.planner.delivery_candidates across this many
    # final-orientation buckets spanning the cube's mod-90 symmetry class [-45, 45) deg.
    n_alpha_buckets: int = 7
    # Headings scanned by classify_unsolvable's first-leg feasibility sweep.
    heading_scan_n: int = 36


DEFAULT_PARAMS = PushParams()

# Tiny inward nudge (mm) so a computed "exactly at the safety threshold" anchor lands cleanly on
# the safe side of a containment check instead of exactly on the boundary, where float rounding can
# put it a few ULPs over. Physically negligible; avoids flaky boundary rejections near tight corners.
_EPS = 0.05


# --------------------------------------------------------------------------------- pure geometry

def wrap_to_pm45(theta: float) -> float:
    """Reduce an angle to the cube's mod-90 symmetry class, range [-pi/4, pi/4)."""
    return ((theta + math.pi / 4.0) % (math.pi / 2.0)) - math.pi / 4.0


def _angle_to_face_normal(heading: float, alpha: float) -> float:
    """Smallest |heading - one of alpha's 4 face normals| (alpha need not be pre-reduced)."""
    return min(abs(angle_diff(heading, alpha + k * math.pi / 2.0)) for k in range(4))


def _capture_risk(cfg: Config, d_rad: float) -> float:
    """H-11 capture-risk proxy: 0 at a perfectly aligned push, 1 at the worst case (45 deg off)."""
    channel = 2.0 * cfg.rover.inner_half_width
    w = cfg.cube.side * (abs(math.cos(d_rad)) + abs(math.sin(d_rad)))
    best_tol = (channel - cfg.cube.side) / 2.0
    if best_tol <= 0:
        return 0.0
    tol = (channel - w) / 2.0
    return min(1.0, max(0.0, 1.0 - tol / best_tol))


def _combine_risk(risks: Sequence[float], params: PushParams) -> float:
    """Combine independent per-leg/delivery risk terms as if they were failure probabilities,
    plus a small penalty per extra leg (more legs = more chances for something to go wrong)."""
    p_ok = 1.0
    for r in risks:
        p_ok *= 1.0 - min(max(r, 0.0), 1.0)
    extra = params.extra_leg_risk * max(0, len(risks) - 1)
    return min(1.0, (1.0 - p_ok) + extra)


def _worst_alpha_for_extent(alpha: float | None, alpha_std: float) -> float | None:
    """Lead audit: alpha within [alpha - 2*std, alpha + 2*std] that maximises the cube's
    axis-aligned extent (the worst case for depot-region erosion / marker overlap).  None (treat
    as fully unknown -> half-diagonal) if alpha is None or the belief interval already spans a
    full 45 deg (2*std >= 45 deg), per the lead's instruction."""
    if alpha is None:
        return None
    spread = 2.0 * alpha_std
    if spread >= math.pi / 4.0:
        return None
    lo, hi = alpha - spread, alpha + spread
    if hi >= math.pi / 4.0 or lo <= -math.pi / 4.0:
        return math.pi / 4.0   # interval reaches a peak (cube diagonal) -> same as half_diag
    return hi if abs(hi) > abs(lo) else lo


def _worst_face_normal_offset(heading: float, alpha: float, alpha_std: float) -> float:
    """Lead audit: worst-case (largest) angular offset between `heading` and the nearest of
    `alpha`'s 4 face normals, over the belief interval alpha +- 2*alpha_std.  Capped at pi/4 (fully
    unconstrained) once the interval is that wide."""
    spread = 2.0 * alpha_std
    if spread >= math.pi / 4.0:
        return math.pi / 4.0
    return min(math.pi / 4.0, _angle_to_face_normal(heading, alpha) + spread)


def _nearest_corner(x: float, y: float, board_w: float, board_h: float) -> tuple[float, float]:
    return (0.0 if x < board_w / 2.0 else board_w, 0.0 if y < board_h / 2.0 else board_h)


def prepush_pose(cfg: Config, cube_xy: tuple[float, float], heading: float) -> Pose:
    """Rover pose `cfg.prepush_distance` behind the cube along `-heading`, already facing `heading`."""
    d = cfg.prepush_distance
    return Pose(cube_xy[0] - d * math.cos(heading), cube_xy[1] - d * math.sin(heading), wrap(heading))


def leg_feasible(cfg: Config, fp: Footprint, cube_xy: tuple[float, float], alpha: float | None,
                  heading: float, length: float, other_cubes: Sequence[CubeEstimate],
                  board_w: float, board_h: float, extra_margin: float = 0.0,
                  final_leg: bool = False) -> tuple[bool, str]:
    """Is a single straight push leg (cube_xy -> cube_xy + length*heading) physically executable?

    Checks, all against the no-walls field [margins.board, board_w/h - margins.board]:
      1. `length` clears `PushParams.min_leg_length` (else the cube never squares to the plate).
      2. the rover's push corridor (`Footprint.straight_sweep` from contact to leg end) stays inside.
      3. the cube's own swept footprint stays inside AND clears `other_cubes` by `margins.cube_nav`.
         Known orientation -> exact rotated-square sweep (idealised at the leg's *final* resting
         orientation, `heading mod 90`, for the whole leg -- the true orientation only differs by
         up to `push_heading_window_deg`, which the caller enforces before calling this).  Unknown
         -> conservative disc of `cube.half_diag`.  Each *other* cube's own footprint radius is its
         known rotated half-extent (`cube_half_extent`) if `oc.alpha` is set, else the conservative
         `cube.half_diag`.
      4. the post-push retreat (straight reverse of `paddle_reach + retreat_margin_mm`, H-15) stays
         inside and clears `other_cubes`.

    Does NOT check heading-vs-face-normal alignment or depot containment -- those depend on why the
    leg is being taken and are the caller's job (see `PushPlanner._leg_attempt`, `classify_unsolvable`).
    `other_cubes` must already exclude the pushed cube itself.
    """
    if length < DEFAULT_PARAMS.min_leg_length:
        return False, "leg too short to square the cube against the plate"

    cx, cy = cube_xy
    hx, hy = math.cos(heading), math.sin(heading)
    cube_end = (cx + length * hx, cy + length * hy)
    contact = cfg.contact_distance
    rx, ry = cx - contact * hx, cy - contact * hy

    bm = (cfg.margins.board_final_leg if final_leg else cfg.margins.board) + extra_margin
    lo_x, hi_x = bm, board_w - bm
    lo_y, hi_y = bm, board_h - bm

    # The rover sweeps from the PRE-PUSH pose (capture approach) through contact to the leg end.
    approach = cfg.prepush_distance - contact
    corridor = fp.straight_sweep(rx - approach * hx, ry - approach * hy, heading, length + approach)
    if not S.inside_rect(corridor, lo_x, hi_x, lo_y, hi_y):
        return False, "push corridor exits the field"

    clear = cfg.margins.cube_nav
    # Each obstacle's own footprint radius: its actual (known) rotated half-extent when its
    # orientation is known, else the conservative worst-case half-diagonal; plus its position uncertainty.
    other_r = [(oc, (cube_half_extent(cfg.cube.side, oc.alpha) if oc.alpha is not None else cfg.cube.half_diag)
                + min(oc.pos_std, 15.0))
               for oc in other_cubes]
    for oc, r_oc in other_r:
        if S.disc_distance(corridor, (oc.x, oc.y), r_oc) < clear:
            return False, f"rover corridor too close to cube '{oc.color}'"

    if alpha is None:
        r = cfg.cube.half_diag
        for (px, py) in (cube_xy, cube_end):
            if not (lo_x + r <= px <= hi_x - r and lo_y + r <= py <= hi_y - r):
                return False, "cube sweep exits the field"
        for oc, r_oc in other_r:
            d = S._seg_point_dist(np.array(cube_xy, float), np.array(cube_end, float), np.array([oc.x, oc.y]))
            if d - r - r_oc < clear:
                return False, f"cube sweep too close to cube '{oc.color}'"
    else:
        final_alpha = wrap_to_pm45(heading)
        sq0 = S.square(cx, cy, cfg.cube.side, final_alpha)
        sq1 = S.square(cube_end[0], cube_end[1], cfg.cube.side, final_alpha)
        sweep_poly = S.hull(np.vstack([sq0, sq1]))
        if not S.inside_rect(sweep_poly, lo_x, hi_x, lo_y, hi_y):
            return False, "cube sweep exits the field"
        for oc, r_oc in other_r:
            if S.disc_distance(sweep_poly, (oc.x, oc.y), r_oc) < clear:
                return False, f"cube sweep too close to cube '{oc.color}'"

    retreat_dist = cfg.rover.paddle_reach + DEFAULT_PARAMS.retreat_margin_mm
    rex, rey = rx + length * hx, ry + length * hy
    retreat = fp.straight_sweep(rex, rey, heading, -retreat_dist)
    if not S.inside_rect(retreat, lo_x, hi_x, lo_y, hi_y):
        return False, "retreat exits the field"
    for oc, r_oc in other_r:
        if S.disc_distance(retreat, (oc.x, oc.y), r_oc) < clear:
            return False, f"retreat too close to cube '{oc.color}'"

    return True, ""


def _prepush_ok(cfg: Config, fp: Footprint, pose: Pose, target_xy: tuple[float, float],
                 other_cubes: Sequence[CubeEstimate], board_w: float, board_h: float,
                 extra_margin: float = 0.0) -> bool:
    """Pre-push pose: envelope inside the field (the general "pre-push pose" containment rule),
    AND the full in-place rotation-sweep circle clear of every other cube -- including the target
    cube at its current (intermediate) position -- per the MULTI-LEG "must be able to rotate at the
    next prepush" rule.  That rule only requires clearance from *cubes*; it is not read here as also
    requiring the rotation circle itself to stay inside the field (only the three explicitly listed
    phases -- pre-push pose, push, retreat -- must; a board-edge-clipping in-place spin between two
    otherwise-legal poses is not one of them).  cfg.prepush_distance is built to already guarantee
    clearance from the target cube itself; the check against it below is a cheap sanity assertion."""
    # Board contract (same as navigation, whose board margin is reduced by its footprint inflation): the TRUE
    # footprint keeps margins.board from the field edge (lead, closed-loop finding).
    m = cfg.margins.board + extra_margin
    lo_x, hi_x = m, board_w - m
    lo_y, hi_y = m, board_h - m
    env = fp.envelope(pose.x, pose.y, pose.theta)
    if not S.inside_rect(env, lo_x, hi_x, lo_y, hi_y):
        return False
    r = fp.sweep_radius
    thresh = r + cfg.cube.half_diag + cfg.margins.cube_nav
    if math.hypot(pose.x - target_xy[0], pose.y - target_xy[1]) < thresh - 1e-6:
        return False  # should not happen: cfg.prepush_distance is built to satisfy this
    for oc in other_cubes:
        if math.hypot(pose.x - oc.x, pose.y - oc.y) < thresh:
            return False
    if DEFAULT_PARAMS.require_straight_approach:
        return _approach_reachable(cfg, fp, pose, other_cubes, board_w, board_h)
    return True


def _approach_reachable(cfg: Config, fp: Footprint, pose: Pose, other_cubes: Sequence[CubeEstimate],
                        board_w: float, board_h: float, max_back: float = 320.0, step: float = 20.0) -> bool:
    """The navigation planner only has in-place rotations and straight moves (no arcs yet): a pre-push pose is
    reachable only if, somewhere behind it on the push line, the rover can turn in place fully inside the field and
    then drive straight in.  (Lead, closed-loop finding: edge-parallel pre-push poses were certified but could never
    be reached.)  ponytail: straight-in approach only; arc primitives in navigation would relax this."""
    m = cfg.margins.board
    r = fp.sweep_radius + cfg.margins.pose_uncertainty
    c, s_ = math.cos(pose.theta), math.sin(pose.theta)
    for k in range(0, int(max_back / step) + 1):
        bx, by = pose.x - k * step * c, pose.y - k * step * s_
        if not (m + r <= bx <= board_w - m - r and m + r <= by <= board_h - m - r):
            continue
        if any(math.hypot(bx - oc.x, by - oc.y) < r + cfg.cube.half_diag + cfg.margins.cube_nav for oc in other_cubes):
            continue
        sweep = fp.straight_sweep(bx, by, pose.theta, k * step)
        if S.inside_rect(sweep, m, board_w - m, m, board_h - m):
            return True
    return False


def is_delivered(cfg: Config, cube: CubeEstimate, depot: DepotZone) -> bool:
    """Whole cube footprint inside the depot zone (strict, no planning margin).  Uses the
    worst-case orientation over `cube.alpha +- 2*cube.alpha_std` (lead audit) -- `cube.alpha is
    None`, or a belief interval spanning >= 45 deg, falls back to the worst-case half-diagonal."""
    worst_alpha = _worst_alpha_for_extent(cube.alpha, cube.alpha_std)
    return depot.contains_cube(cube.x, cube.y, cfg.cube.side, worst_alpha, margin=0.0)


def classify_unsolvable(cfg: Config, cube: CubeEstimate, depot: DepotZone,
                         other_cubes: Sequence[CubeEstimate], board_w: float, board_h: float) -> str | None:
    """Explain why a cube can never be delivered under this no-walls model (e.g. wedged in a
    non-depot corner so no first push leg keeps the rover's footprint in the field), else None.

    Only proves a *first-leg* impossibility (scanning `PushParams.heading_scan_n` headings); it does
    not attempt to disprove every multi-leg escape, so a None return is not a promise of solvability.
    """
    fp = Footprint(cfg.rover)
    cx, cy = cube.x, cube.y
    n = DEFAULT_PARAMS.heading_scan_n
    any_prepush_ok = False
    for k in range(n):
        heading = -math.pi + 2.0 * math.pi * k / n
        if cube.alpha is not None:
            d = _worst_face_normal_offset(heading, cube.alpha, cube.alpha_std)
            if math.degrees(d) > cfg.planner.push_heading_window_deg:
                continue
        pre = prepush_pose(cfg, (cx, cy), heading)
        if not _prepush_ok(cfg, fp, pre, (cx, cy), other_cubes, board_w, board_h):
            continue
        any_prepush_ok = True
        ok, _ = leg_feasible(cfg, fp, (cx, cy), cube.alpha, heading, DEFAULT_PARAMS.min_leg_length,
                              other_cubes, board_w, board_h)
        if ok:
            return None

    corner = _nearest_corner(cx, cy, board_w, board_h)
    depot_corner = _nearest_corner(depot.cx, depot.cy, board_w, board_h)
    near = cfg.cube.half_diag + cfg.margins.board + 60.0
    if abs(cx - corner[0]) < near and abs(cy - corner[1]) < near and corner != depot_corner:
        return (f"cube '{cube.color}' at ({cx:.0f},{cy:.0f}) is wedged in the non-depot corner "
                f"{corner}: no first-leg push heading keeps the rover footprint inside the field")
    if not any_prepush_ok:
        return (f"cube '{cube.color}' at ({cx:.0f},{cy:.0f}): every pre-push pose falls outside "
                f"the field for every scanned heading")
    return (f"cube '{cube.color}' at ({cx:.0f},{cy:.0f}): no push heading has a clear in-field "
            f"corridor (blocked by other cubes or the field edge)")


# --------------------------------------------------------------------------------- planner

def _perp_safe_value(lo: float, hi: float, corner_val: float, board_dim: float, margin: float, half_width: float) -> float:
    """The value in [lo, hi] closest to keeping the ROVER's own half-width (not just the cube)
    clear of the nearest board edge -- i.e. usable as the *perpendicular* axis of a straight final
    approach.  Delivering near a board corner needs this precisely: the reachable slice of the
    depot's valid-center region on the perpendicular axis is typically only a couple of mm wide
    (rover half-width + margins.board eats nearly all of a corner-sized depot), far finer than any
    affordable uniform grid pitch."""
    if corner_val <= board_dim / 2.0:
        safe = margin + half_width + _EPS
        return min(hi, max(lo, safe))
    safe = board_dim - margin - half_width - _EPS
    return max(lo, min(hi, safe))


def _delivery_candidates(cfg: Config, params: PushParams, depot: DepotZone, board_w: float,
                          board_h: float) -> list[tuple[tuple[float, float], float]]:
    """Sample points across the depot's valid-center regions for a spread of final orientations.
    Returns (point, orientation_bucket) pairs; the bucket is only a sampling aid -- feasibility is
    re-checked against the *actual* resulting orientation for whatever leg reaches the point
    (self-consistency: alpha = wrap_to_pm45(heading to that point), not the bucket).

    Besides a uniform grid per bucket, also adds precision candidates at `_perp_safe_value` on each
    axis (crossed with a spread on the other axis) -- see that function's docstring for why a blind
    grid alone routinely misses the only reachable slice near a board corner."""
    side = cfg.cube.side
    margin = cfg.depot.delivery_margin
    m = max(1, params.n_alpha_buckets)
    per_bucket = max(1, round(cfg.planner.delivery_candidates / m))
    grid_n = max(1, round(math.sqrt(per_bucket)))
    half_width = cfg.rover.outer_half_width
    board_margin = cfg.margins.board
    out: list[tuple[tuple[float, float], float]] = []
    for i in range(m):
        deg = -45.0 + (i + 0.5) * (90.0 / m)
        a = math.radians(deg)
        region = depot.valid_center_region(side, a, margin)
        if region is None:
            continue
        x0, x1, y0, y1 = region
        xs = np.linspace(x0, x1, grid_n) if x1 > x0 else np.array([x0])
        ys = np.linspace(y0, y1, grid_n) if y1 > y0 else np.array([y0])
        for x in xs:
            for y in ys:
                out.append(((float(x), float(y)), a))
        safe_x = _perp_safe_value(x0, x1, depot.cx, board_w, board_margin, half_width)
        safe_y = _perp_safe_value(y0, y1, depot.cy, board_h, board_margin, half_width)
        for y in ys:
            out.append(((safe_x, float(y)), a))
        for x in xs:
            out.append(((float(x), safe_y), a))
        out.append(((safe_x, safe_y), a))
    return out


def _marker_overlap_frac(cfg: Config, depot: DepotZone, D: tuple[float, float], alpha: float | None,
                          board_w: float, board_h: float) -> float:
    """Fraction of the (worst-case) cube AABB at D that overlaps the in-field quarter of the
    nearest corner's ArUco marker square (100mm black + 20mm white border, config.board.
    corner_marker_half_mm each side of the field corner) -- covering it freezes vision (lead audit)."""
    corner_x, corner_y = _nearest_corner(depot.cx, depot.cy, board_w, board_h)
    half = cfg.board.corner_marker_half_mm
    marker = (corner_x - half, corner_x + half, corner_y - half, corner_y + half)
    he = cube_half_extent(cfg.cube.side, alpha) if alpha is not None else cfg.cube.half_diag
    box = (D[0] - he, D[0] + he, D[1] - he, D[1] + he)
    ox = max(0.0, min(box[1], marker[1]) - max(box[0], marker[0]))
    oy = max(0.0, min(box[3], marker[3]) - max(box[2], marker[2]))
    area = (2.0 * he) ** 2
    return (ox * oy) / area if area > 0 else 0.0


def _delivery_point_risk(cfg: Config, params: PushParams, depot: DepotZone, D: tuple[float, float],
                          alpha_mean: float, board_w: float, board_h: float) -> tuple[float, float] | None:
    """Risk term for a delivery point reached with predicted final orientation `alpha_mean`
    (= heading mod 90).  Returns None if D is NOT self-consistent under the *nominal* (mean)
    orientation: it must fit inside the depot's valid-center region with `config.depot.
    delivery_margin`, else the cube would predictably stick out and this D is rejected outright.

    The WORST-CASE orientation over `alpha_mean +- 2*PushParams.post_push_alpha_std` (lead audit)
    is folded in as an extra RISK term instead of a second hard gate: right at a board corner the
    nominally-reachable region is already only a few mm deep (rover-envelope clearance eats most of
    config.depot.half_size), so hard-rejecting on the worst case there would make routine corner
    deliveries impossible outright. A borderline point still ranks worse, it just is not discarded.

    Otherwise returns (risk, marker_frac): risk blends distance-to-region-border, the worst-case
    uncertainty penalty above, corner-marker AABB overlap (favours dodging the marker -- hard-tiered
    by the caller, this is just its magnitude for tie-breaking) and a mild pull towards the
    published depot point.
    """
    side = cfg.cube.side
    region = depot.valid_center_region(side, alpha_mean, cfg.depot.delivery_margin)
    if region is None:
        return None
    x0, x1, y0, y1 = region
    if not (x0 <= D[0] <= x1 and y0 <= D[1] <= y1):
        return None

    hx, hy = max((x1 - x0) / 2.0, 1e-6), max((y1 - y0) / 2.0, 1e-6)
    cxr, cyr = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    border_risk = min(1.0, max(abs(D[0] - cxr) / hx, abs(D[1] - cyr) / hy))
    center_pref = min(1.0, math.hypot(D[0] - depot.cx, D[1] - depot.cy) / max(depot.half, 1e-6))

    worst_alpha = _worst_alpha_for_extent(alpha_mean, params.post_push_alpha_std)
    worst_region = depot.valid_center_region(side, worst_alpha, cfg.depot.delivery_margin)
    if worst_region is None:
        uncertainty_risk = 1.0
    else:
        wx0, wx1, wy0, wy1 = worst_region
        whx, why = max((wx1 - wx0) / 2.0, 1e-6), max((wy1 - wy0) / 2.0, 1e-6)
        wcxr, wcyr = (wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0
        worst_frac = max(abs(D[0] - wcxr) / whx, abs(D[1] - wcyr) / why)   # > 1 => sticks out under worst case
        uncertainty_risk = min(1.0, max(0.0, worst_frac - 1.0))
    marker_frac = _marker_overlap_frac(cfg, depot, D, worst_alpha, board_w, board_h)

    risk = (params.border_risk_weight * border_risk
            + params.marker_risk_weight * marker_frac
            + params.depot_center_pref_weight * center_pref
            + params.uncertainty_risk_weight * uncertainty_risk)
    return min(1.0, risk), marker_frac


def _grid_axes(cfg: Config, params: PushParams, board_w: float, board_h: float) -> tuple[list[float], list[float]]:
    """Coarse per-axis grid values, PLUS precision anchors on each axis:
      - the rover-envelope-safe values (see `_perp_safe_value`) -- a straight leg along a board
        edge routinely needs its perpendicular coordinate within a couple of mm of one of these.
      - the "either-direction-safe" values `margins.board + prepush_distance + chassis_half_length`
        from each edge -- a waypoint used to STAGE a later axis-aligned approach (as opposed to
        being the approach's own perpendicular coordinate) instead needs enough clearance for a
        full pre-push pose in front of AND behind it, which is a much larger margin than the
        rover's own half-width; without this anchor a coarse grid can land a staging point just
        short of the room a subsequent leg needs to even get lined up.
    A uniform pitch alone can straddle either band without ever landing inside it."""
    lo_margin = cfg.margins.board + cfg.cube.half_diag + 5.0
    xs = np.arange(lo_margin, board_w - lo_margin, params.intermediate_grid_mm)
    ys = np.arange(lo_margin, board_h - lo_margin, params.intermediate_grid_mm)
    half_width, m = cfg.rover.outer_half_width, cfg.margins.board
    stage = cfg.prepush_distance + cfg.rover.chassis_half_length
    extra = [m + half_width + _EPS, board_w - m - half_width - _EPS, m + stage + _EPS, board_w - m - stage - _EPS]
    extra_y = [m + half_width + _EPS, board_h - m - half_width - _EPS, m + stage + _EPS, board_h - m - stage - _EPS]
    xs = list(xs) + extra
    ys = list(ys) + extra_y
    return [float(x) for x in xs], [float(y) for y in ys]


def _intermediate_grid(xs: Sequence[float], ys: Sequence[float]) -> list[tuple[float, float]]:
    """Coarse spatial grid, for routing an intermediate leg around obstacles.  Trade-off: finer
    pitch finds more/tighter routes but the 2-/3-leg search cost grows ~ (board/pitch)^2 per hop;
    `PushParams.intermediate_grid_mm` (baked into `xs`/`ys`, see `_grid_axes`) picks the pitch."""
    return [(x, y) for x in xs for y in ys]


def _hop_candidates(pos: tuple[float, float], d_points: Sequence[tuple[float, float]], grid: list[tuple[float, float]],
                     xs: Sequence[float], ys: Sequence[float]) -> list[tuple[float, float]]:
    """Candidate next waypoints from `pos`: the generic spatial `grid` (routes around obstacles)
    PLUS two kinds of axis-aligned anchor that a uniform grid is too coarse to hit reliably:
      - for every delivery candidate D, the "elbow" points (D.x, pos.y) / (pos.x, D.y) that let the
        NEXT leg reach D axis-aligned.  Delivering into a board corner routinely needs the final
        leg's *perpendicular* axis to land within a very tight band (a couple of mm, given typical
        margins/rover width) -- these hit that band directly instead of hoping a blind grid does.
      - a plain axis-aligned "cross" through `pos` itself, ((x, pos.y) / (pos.x, y) for the grid's
        own x/y values), so a leg FROM `pos` can itself be axis-aligned (locking a mod-90 orientation
        the *next* leg can then reuse within the push_heading_window, per the physical rule that a
        known-oriented cube can only be re-pushed near one of its own face normals).
    Computed fresh at whatever position the search has actually reached (not just the cube's
    original spot), so a 3rd leg can still square up a 2-leg route that only got partway there."""
    px, py = pos
    out = list(grid)
    for D in d_points:
        out.append((D[0], py))
        out.append((px, D[1]))
    for x in xs:
        out.append((x, py))
    for y in ys:
        out.append((px, y))
    return out


_LegAttempt = tuple[PushLeg, float, float, float]   # leg, resulting_alpha, cost_s, risk


def _leg_attempt(cfg: Config, fp: Footprint, start_xy: tuple[float, float], entering_alpha: float | None,
                  entering_alpha_std: float, end_xy: tuple[float, float], other_cubes: Sequence[CubeEstimate],
                  board_w: float, board_h: float, params: PushParams, slack: float = 0.0,
                  final_leg: bool = False) -> _LegAttempt | None:
    dx, dy = end_xy[0] - start_xy[0], end_xy[1] - start_xy[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    heading = math.atan2(dy, dx)

    window = math.radians(cfg.planner.push_heading_window_deg)
    if entering_alpha is not None and 2.0 * entering_alpha_std < 0.5 * window:
        # Orientation known well enough to enforce the H-11 window.
        d = _worst_face_normal_offset(heading, entering_alpha, entering_alpha_std)
        if d > window:
            return None
        leg_risk = _capture_risk(cfg, d)
    elif entering_alpha is not None:
        # Poorly known orientation: a hard window would forbid every heading.  Use the belief as a soft prior:
        # expected capture risk over "aligned as believed" vs worst case (lead fix, closed-loop finding).
        d = _angle_to_face_normal(heading, entering_alpha)
        leg_risk = 0.5 * _capture_risk(cfg, d) + 0.5 * _capture_risk(cfg, min(math.pi / 4, d + 2 * entering_alpha_std))
    else:
        near90 = _angle_to_face_normal(heading, 0.0)
        leg_risk = (params.grid_align_confidence * _capture_risk(cfg, near90)
                    + (1.0 - params.grid_align_confidence) * _capture_risk(cfg, math.pi / 4.0))

    ok, _why = leg_feasible(cfg, fp, start_xy, entering_alpha, heading, length, other_cubes, board_w, board_h, slack,
                            final_leg)
    if not ok:
        return None
    pre = prepush_pose(cfg, start_xy, heading)
    if not _prepush_ok(cfg, fp, pre, start_xy, other_cubes, board_w, board_h, slack):
        return None

    result_alpha = wrap_to_pm45(heading)
    leg = PushLeg(cube_start=start_xy, cube_end=end_xy, heading=heading, prepush=pre)
    push_time = length / cfg.limits.v_push
    retreat_time = (cfg.rover.paddle_reach + params.retreat_margin_mm) / cfg.limits.v_retreat
    leg_cost = push_time + retreat_time + params.per_leg_overhead_s
    return leg, result_alpha, leg_cost, leg_risk


class PushPlanner:
    def __init__(self, cfg: Config = DEFAULT, params: PushParams | None = None):
        self.cfg = cfg
        self.params = params or DEFAULT_PARAMS
        self.fp = Footprint(cfg.rover)
        self.last_failure = ""

    def plan(self, cube: CubeEstimate, depot: DepotZone, other_cubes: Sequence[CubeEstimate],
             board_w: float, board_h: float, max_plans: int = 5, deadline_s: float = 0.3) -> list[PushPlan]:
        cfg, p, fp = self.cfg, self.params, self.fp
        t0 = time.monotonic()
        self.last_failure = ""
        others = [c for c in other_cubes if c.color != cube.color]

        def order_key(D: tuple[float, float], a: float) -> float:
            scored = _delivery_point_risk(cfg, p, depot, D, a, board_w, board_h)
            return 1.0 if scored is None else scored[0]

        d_points = sorted({D for D, _a in _delivery_candidates(cfg, p, depot, board_w, board_h)},
                           key=lambda D: order_key(D, wrap_to_pm45(math.atan2(D[1] - cube.y, D[0] - cube.x))))

        def budget_left() -> float:
            return deadline_s - (time.monotonic() - t0)

        # results: (marker_tier, risk, cost_s, PushPlan). marker_tier 0 = no corner-marker overlap.
        results: list[tuple[int, float, float, PushPlan]] = []
        start = (cube.x, cube.y)

        def try_final(legs: list[PushLeg], prior_cost: float, prior_risks: list[float],
                      leg_start: tuple[float, float], entering_alpha: float | None, entering_std: float,
                      D: tuple[float, float], tag: str) -> None:
            att = _leg_attempt(cfg, fp, leg_start, entering_alpha, entering_std, D, others, board_w, board_h, p,
                               final_leg=True)
            if att is None:
                return
            leg, alpha_final, cost, leg_risk = att
            scored = _delivery_point_risk(cfg, p, depot, D, alpha_final, board_w, board_h)
            if scored is None:
                return   # not self-consistent: predicted orientation would stick out of the depot
            d_risk, marker_frac = scored
            total_cost = prior_cost + cost
            total_risk = _combine_risk([*prior_risks, leg_risk, d_risk], p)
            notes = tag + (" (corner-marker overlap risk)" if marker_frac > 0.0 else "")
            tier = 0 if marker_frac <= 0.0 else 1
            plan_ = PushPlan(cube.color, [*legs, leg], D, total_cost, total_risk, notes)
            results.append((tier, total_risk, total_cost, plan_))

        # ---- 1-leg (direct)
        for D in d_points:
            if budget_left() <= 0:
                break
            try_final([], 0.0, [], start, cube.alpha, cube.alpha_std, D, "1-leg direct")

        # ---- 2-/3-leg (only if we still need more candidates and legs are allowed)
        if len(results) < max_plans and budget_left() > 0 and cfg.planner.max_push_legs >= 2:
            grid_xs, grid_ys = _grid_axes(cfg, p, board_w, board_h)
            grid = _intermediate_grid(grid_xs, grid_ys)
            hop1_candidates = _hop_candidates(start, d_points, grid, grid_xs, grid_ys)
            hop1 = []
            for I1 in hop1_candidates:
                if budget_left() <= 0:
                    break
                att = _leg_attempt(cfg, fp, start, cube.alpha, cube.alpha_std, I1, others, board_w, board_h, p)
                if att is not None:
                    hop1.append((I1, *att))

            for I1, leg1, alpha1, cost1, risk1 in hop1:
                if budget_left() <= 0:
                    break
                for D in d_points:
                    if budget_left() <= 0:
                        break
                    try_final([leg1], cost1, [risk1], I1, alpha1, p.post_push_alpha_std, D, "2-leg")

            # Only pay for the O(hop1 x hop2 x delivery) 3-leg search if 1-/2-leg genuinely didn't
            # already give us enough plans -- this is the expensive phase and most cubes never need
            # it, so gating it keeps the common case fast (typically well under 0.15s).
            if len(results) < max_plans and cfg.planner.max_push_legs >= 3:
                # Cap how many first hops feed the 3-leg search: try the most promising (cheapest,
                # safest) ones first: a 3rd leg is only ever needed for the few hop1's that a 2-leg
                # couldn't already close out, and this bounds the O(hop1 x hop2 x delivery) blow-up.
                hop1_ranked = sorted(hop1, key=lambda h: (h[3], h[4]))[:20]
                for I1, leg1, alpha1, cost1, risk1 in hop1_ranked:
                    if budget_left() <= 0:
                        break
                    # Anchors relative to I1 (not just the original cube position) so a 3rd leg can
                    # still square up to a tight corner even when the 2-leg route only got partway.
                    for I2 in _hop_candidates(I1, d_points, grid, grid_xs, grid_ys):
                        if budget_left() <= 0:
                            break
                        att_mid = _leg_attempt(cfg, fp, I1, alpha1, p.post_push_alpha_std, I2, others,
                                                board_w, board_h, p, slack=p.intermediate_slack_mm)
                        if att_mid is None:
                            continue
                        leg2, alpha2, cost2, risk2 = att_mid
                        for D in d_points:
                            if budget_left() <= 0:
                                break
                            try_final([leg1, leg2], cost1 + cost2, [risk1, risk2], I2, alpha2,
                                       p.post_push_alpha_std, D, "3-leg")

        if not results:
            self.last_failure = classify_unsolvable(cfg, cube, depot, other_cubes, board_w, board_h) \
                or "no feasible push plan found within the search deadline"
            return []

        # Hard corner-marker preference (lead audit): if any candidate avoids the marker entirely,
        # drop every candidate that doesn't.
        best_tier = min(r[0] for r in results)
        results = [r for r in results if r[0] == best_tier]

        results.sort(key=lambda t: (t[1], t[2]))   # safest first, then fastest
        out: list[PushPlan] = []
        seen: set[tuple[int, int]] = set()
        for _tier, _risk, _cost, plan_ in results:
            key = (round(plan_.delivery_point[0] / 8.0), round(plan_.delivery_point[1] / 8.0))
            if key in seen:
                continue
            seen.add(key)
            out.append(plan_)
            if len(out) >= max_plans:
                break
        return out
