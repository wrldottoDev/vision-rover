"""Two-rover task allocation: which cube goes to which rover, in what order, with which
candidate PushPlan.  Enumeration-based (small n: <=3 cubes) so we can just search all of it
instead of building a real solver.

Execution timing is unreliable (H: motors/vision latency) so nothing here depends on a precise
time-indexed schedule -- times are only ever used *relatively*, to rank candidate allocations and
to decide "who must go first" (precedence), never to hand out a wall-clock schedule to execution.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Callable, Iterator

import numpy as np

from ..config import Config, DEFAULT
from ..frames import angle_diff
from ..geometry import shapes as S
from ..geometry.footprint import Footprint
from ..world import Pose, PushPlan, RoverEstimate


# --------------------------------------------------------------------------- tunables
@dataclass(frozen=True)
class AllocatorParams:
    """Every constant here is local to allocation-time *estimation*; none of it feeds control.
    Provenance tags per worker_common.md; centralise on request."""

    top_k_plans: int = 3                 # TUNED: candidate plans considered per cube
    detour_factor: float = 1.3           # ASSUMED: straight-line nav underestimates real path (cubes/other rover in the way)
    retreat_distance_mm: float = 80.0    # ASSUMED: how far the rover backs off after a push, for end-pose estimation only
    capture_overhead_s: float = 0.5      # ASSUMED: align + grip time, not included in PushPlan.cost_s
    retreat_overhead_s: float = 1.0      # ASSUMED: retreat maneuver time, not included in PushPlan.cost_s ("push-phase")
    lambda_total: float = 0.05           # TUNED: small weight on sum(T) to break makespan ties toward less total work
    risk_weight: float = 5.0             # TUNED
    conflict_weight: float = 8.0         # TUNED
    depot_conflict_radius_mm: float = 150.0   # ASSUMED: "heading to the same depot corner" proximity threshold
    precedence_penalty: float = 1.0e6    # effectively-infeasible penalty for an unresolved precedence hazard
    require_both_rovers: bool = True     # HARD RULE (reglamento.md s12): both rovers must deliver >=1 cube each
                                          # when both are healthy and >=2 cubes are up for grabs this round.


# --------------------------------------------------------------------------- result types
@dataclass(frozen=True)
class RoverTask:
    color: str
    plan: PushPlan


@dataclass(frozen=True)
class ObjectiveBreakdown:
    makespan: float
    total_time: float      # T1 + T2 (or just T1 if one rover)
    risk: float
    conflict: float
    total: float


@dataclass(frozen=True)
class Allocation:
    assignment: dict[int, list[RoverTask]]
    times: dict[int, float]
    makespan: float
    objective: ObjectiveBreakdown
    skipped: tuple[str, ...] = ()          # cubes with no usable plan this round


# --------------------------------------------------------------------------- geometry helpers
def _end_pose(plan: PushPlan, cfg: Config, params: AllocatorParams) -> Pose:
    """Rover pose after finishing `plan`: contact pose at the final leg's cube_end, then backed off
    by contact_distance + an assumed retreat distance along the (unchanged) push heading."""
    last = plan.legs[-1]
    heading = last.heading
    back = cfg.contact_distance + params.retreat_distance_mm
    cx, cy = last.cube_end
    return Pose(cx - back * math.cos(heading), cy - back * math.sin(heading), heading)


def default_nav_time(a: Pose, b: Pose, cfg: Config = DEFAULT, params: AllocatorParams | None = None) -> float:
    """Rotate to face b, drive the straight-line distance (inflated by a detour factor to account for
    obstacles not modelled here), rotate to b's heading."""
    params = params or AllocatorParams()
    dx, dy = b.x - a.x, b.y - a.y
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return abs(angle_diff(b.theta, a.theta)) / cfg.limits.w_nav
    heading = math.atan2(dy, dx)
    rot1 = abs(angle_diff(heading, a.theta))
    rot2 = abs(angle_diff(b.theta, heading))
    return (rot1 + rot2) / cfg.limits.w_nav + d * params.detour_factor / cfg.limits.v_nav


def _work_region(plan: PushPlan, cfg: Config) -> S.Poly:
    """Convex hull of everything the rover physically occupies while running `plan`: its footprint at
    every leg's prepush pose plus the cube's start/end points on each leg."""
    fp = Footprint(cfg.rover)
    pts = [fp.envelope(plan.legs[0].prepush.x, plan.legs[0].prepush.y, plan.legs[0].prepush.theta)]
    for leg in plan.legs:
        pts.append(fp.envelope(leg.prepush.x, leg.prepush.y, leg.prepush.theta))
        pts.append(np.array([leg.cube_start]))
        pts.append(np.array([leg.cube_end]))
    return S.hull(np.vstack(pts))


def _seg_point_dist(a: np.ndarray, b: np.ndarray, p: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    t = 0.0 if denom < 1e-12 else float(np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0))
    return float(np.linalg.norm(a + t * ab - p))


def _corridor_hits(plan: PushPlan, point: tuple[float, float], threshold: float) -> bool:
    p = np.asarray(point, float)
    for leg in plan.legs:
        d = _seg_point_dist(np.asarray(leg.cube_start, float), np.asarray(leg.cube_end, float), p)
        if d < threshold:
            return True
    return False


# --------------------------------------------------------------------------- enumeration
def distribute_cubes(cubes: list[str], rover_ids: list[int]) -> Iterator[dict[int, list[str]]]:
    """Every way to split `cubes` (order matters, order = execution sequence) across `rover_ids`
    (1 or 2 of them).  For n cubes and 2 rovers this yields n! * (n+1) distributions."""
    if len(rover_ids) == 1:
        rid = rover_ids[0]
        for perm in itertools.permutations(cubes):
            yield {rid: list(perm)}
        return
    r0, r1 = rover_ids
    n = len(cubes)
    for k in range(n + 1):
        for combo in itertools.combinations(cubes, k):
            rest = [c for c in cubes if c not in combo]
            for perm0 in itertools.permutations(combo):
                for perm1 in itertools.permutations(rest):
                    yield {r0: list(perm0), r1: list(perm1)}


@dataclass(frozen=True)
class _Timing:
    color: str
    rover_id: int
    plan: PushPlan
    task_start: float       # cumulative time on this rover's timeline before this task began
    push_start: float       # after nav
    completion: float       # after cost_s + capture + retreat overhead


def _sequence_timing(rover_id: int, seq: list[tuple[str, PushPlan]], start_pose: Pose,
                      cfg: Config, params: AllocatorParams,
                      nav_time: Callable[[Pose, Pose], float]) -> tuple[float, list[_Timing]]:
    t = 0.0
    pose = start_pose
    out: list[_Timing] = []
    for color, plan in seq:
        task_start = t
        push_start = t + nav_time(pose, plan.legs[0].prepush)
        completion = push_start + plan.cost_s + params.capture_overhead_s + params.retreat_overhead_s
        out.append(_Timing(color, rover_id, plan, task_start, push_start, completion))
        t = completion
        pose = _end_pose(plan, cfg, params)
    return t, out


def _intervals_overlap(a0: float, a1: float, b0: float, b1: float) -> bool:
    return a0 < b1 and b0 < a1


def _conflict_and_precedence(timings: list[_Timing], cfg: Config, params: AllocatorParams) -> float:
    """Precedence: if cube A's push corridor passes over cube B's (still current) position, B must
    finish before A starts -- checked across BOTH rovers' timelines, since B might be the other
    rover's job.  Note: a cube that was skipped for lack of any plan can't be checked here (there is
    no PushPlan to read its position from); see task_allocator module docstring / final report."""
    penalty = 0.0
    threshold = cfg.cube.half + cfg.margins.cube_nav

    for tA in timings:
        for tB in timings:
            if tA.color == tB.color:
                continue
            b_pos = tB.plan.legs[0].cube_start
            if _corridor_hits(tA.plan, b_pos, threshold) and tB.completion > tA.push_start:
                penalty += params.precedence_penalty

    for tA, tB in itertools.combinations(timings, 2):
        if tA.rover_id == tB.rover_id:
            continue
        if not _intervals_overlap(tA.task_start, tA.completion, tB.task_start, tB.completion):
            continue
        d = S.distance(_work_region(tA.plan, cfg), _work_region(tB.plan, cfg))
        if d < cfg.margins.rover_rover:
            penalty += params.conflict_weight * (1.0 + (cfg.margins.rover_rover - d) / cfg.margins.rover_rover)
        dd = math.hypot(tA.plan.delivery_point[0] - tB.plan.delivery_point[0],
                         tA.plan.delivery_point[1] - tB.plan.delivery_point[1])
        if dd < params.depot_conflict_radius_mm:
            penalty += params.conflict_weight * 0.5
    return penalty


# --------------------------------------------------------------------------- allocator
class TaskAllocator:
    def __init__(self, cfg: Config = DEFAULT, params: AllocatorParams | None = None):
        self.cfg = cfg
        self.params = params or AllocatorParams()

    def allocate(self, rovers: dict[int, RoverEstimate], plans: dict[str, list[PushPlan]],
                 locked: dict[int, str | None], nav_time: Callable[[Pose, Pose], float] | None = None) -> Allocation:
        cfg, params = self.cfg, self.params
        nav_fn = nav_time or (lambda a, b: default_nav_time(a, b, cfg, params))
        rover_ids = sorted(rovers.keys())

        if not rover_ids:
            return Allocation(assignment={}, times={}, makespan=0.0,
                               objective=ObjectiveBreakdown(0.0, 0.0, 0.0, 0.0, 0.0),
                               skipped=tuple(sorted(plans.keys())))

        skipped = tuple(sorted(c for c, cands in plans.items() if not cands))
        usable_plans = {c: cands[:params.top_k_plans] for c, cands in plans.items() if cands}

        # locked cube stays with its rover as a fixed first task; a lock on an unavailable
        # (missing from `rovers`) rover is dropped -- that rover is down, the cube is free again.
        locked_colors: dict[int, str] = {r: c for r, c in locked.items()
                                          if c and r in rovers and c in usable_plans}
        locked_set = set(locked_colors.values())
        free_cubes = sorted(c for c in usable_plans if c not in locked_set)

        n_assignable = len(free_cubes) + len(locked_colors)
        enforce_both = params.require_both_rovers and len(rover_ids) == 2 and n_assignable >= 2

        best: Allocation | None = None
        best_key: tuple[float, ...] | None = None

        for dist in distribute_cubes(free_cubes, rover_ids):
            full: dict[int, list[str]] = {}
            for r in rover_ids:
                prefix = [locked_colors[r]] if r in locked_colors else []
                full[r] = prefix + dist.get(r, [])

            if enforce_both and any(len(full[r]) == 0 for r in rover_ids):
                continue

            plan_choice_lists = []
            slots: list[tuple[int, str]] = []
            for r in rover_ids:
                for c in full[r]:
                    slots.append((r, c))
                    plan_choice_lists.append(usable_plans[c])

            if not slots:
                # nothing to do at all (no cubes usable this round)
                alloc = Allocation(assignment={r: [] for r in rover_ids}, times={r: 0.0 for r in rover_ids},
                                    makespan=0.0,
                                    objective=ObjectiveBreakdown(0.0, 0.0, 0.0, 0.0, 0.0), skipped=skipped)
                return alloc

            for combo in itertools.product(*plan_choice_lists):
                chosen: dict[str, PushPlan] = {c: plan for (r, c), plan in zip(slots, combo)}
                seqs = {r: [(c, chosen[c]) for c in full[r]] for r in rover_ids}

                times: dict[int, float] = {}
                all_timings: list[_Timing] = []
                for r in rover_ids:
                    t, timings = _sequence_timing(r, seqs[r], rovers[r].pose, cfg, params, nav_fn)
                    times[r] = t
                    all_timings.extend(timings)

                makespan = max(times.values()) if times else 0.0
                total_time = sum(times.values())
                risk = sum(chosen[c].risk for c in chosen)
                conflict = _conflict_and_precedence(all_timings, cfg, params)
                objective = makespan + params.lambda_total * total_time + params.risk_weight * risk + conflict

                if best_key is None or objective < best_key[0]:
                    assignment = {r: [RoverTask(c, chosen[c]) for c in full[r]] for r in rover_ids}
                    best = Allocation(assignment=assignment, times=dict(times), makespan=makespan,
                                       objective=ObjectiveBreakdown(makespan, total_time, risk, conflict, objective),
                                       skipped=skipped)
                    best_key = (objective,)

        assert best is not None
        return best


def should_reallocate(old: Allocation, new: Allocation, hysteresis: float) -> bool:
    """Avoid thrashing: only switch plans if the new allocation is meaningfully (> hysteresis) better,
    or the assignment is actually different from what's already running."""
    if old.assignment == new.assignment:
        return False
    return new.objective.total < old.objective.total - hysteresis
