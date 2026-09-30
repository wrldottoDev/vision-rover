"""Closed-loop simulation of one scenario: physics truth -> official-format telemetry -> parser -> Supervisor ->
wheel commands -> physics.  Outcome judged from GROUND TRUTH, never from the strategy's own beliefs."""
from __future__ import annotations

import math
import inspect
from dataclasses import dataclass, field

import numpy as np

from ..config import Config, DEFAULT
from ..coordination.supervisor import Supervisor
from ..geometry.zones import DepotZone
from ..vision.parser import TelemetryError, parse_message
from ..world import STOP
from . import scenarios as SC

PHYS_DT = 0.01
CONTROL_DT = 0.05
SETTLING_INTERVAL_S = 1.0
WHEEL_STOP_TOLERANCE_MM_S = 1.0
MIN_ENGAGED_PUSH_MM = 30.0


@dataclass
class RunResult:
    seed: int
    family: str
    outcome: str = "fail"                 # success | fail | crash
    failure_class: str = "none"
    detail: str = ""
    n_cubes: int = 0
    delivered: int = 0
    delivery_time: float | None = None       # first time every cube is truth-valid
    completion_time: float | None = None       # settled completion, for compatibility
    settled_completion_time: float | None = None
    sim_time: float = 0.0
    rover_rover_collisions: int = 0
    non_target_contacts: int = 0
    rotations_with_cube: int = 0
    rover_exits: int = 0
    falls: int = 0
    cube_exits: int = 0
    deadlocks: int = 0
    rover_transport_counts: dict[int, int] = field(default_factory=dict)
    rover_delivery_counts: dict[int, int] = field(default_factory=dict)
    participating_rovers: list[int] = field(default_factory=list)
    delivery_undone: int = 0
    replans: int = 0
    recoveries: int = 0
    estops: int = 0
    capture_failures: int = 0
    est_pos_rmse: float | None = None
    est_heading_rmse_deg: float | None = None
    push_cross_track_p95: float | None = None
    max_occlusion_s: float | None = None
    collision_duration_s: float = 0.0
    non_target_contact_duration_s: float = 0.0
    max_collision_duration_s: float = 0.0
    max_non_target_contact_duration_s: float = 0.0
    planner_rejection: str = ""
    physical_infeasibility: str = ""
    counters: dict = field(default_factory=dict)
    last_events: list = field(default_factory=list)


def truth_delivered(world, depots: dict[str, DepotZone], cfg: Config) -> dict[str, bool]:
    out = {}
    for color, c in world.cubes.items():
        d = depots.get(color)
        out[color] = bool(c.in_play and d and d.contains_cube(c.x, c.y, cfg.cube.side, c.alpha, 0.0))
    return out


def _participation(world, delivered: dict[str, bool]) -> tuple[dict[int, int], dict[int, int]]:
    """Attribute meaningful transport from ground-truth engaged displacement.

    Contact history is retained for diagnostics, but it is intentionally not
    sufficient for participation.  Each credit requires at least 30 mm of
    direct cube displacement while that rover's task target was the cube; the
    delivery side requires the same threshold and a currently valid delivery.
    """
    transported: dict[int, int] = {rid: 0 for rid in world.rovers}
    deposited: dict[int, int] = {rid: 0 for rid in world.rovers}
    for color, is_delivered in delivered.items():
        cube = world.cubes[color]
        engaged = {rid for rid, distance in cube.engaged_displacement.items()
                   if rid in world.rovers and distance >= MIN_ENGAGED_PUSH_MM}
        for rid in engaged:
            transported[rid] += 1
            if is_delivered and cube.in_play:
                deposited[rid] += 1
    return transported, deposited


def _all_rovers_stopped(world) -> bool:
    """Truth-side stop condition: actual wheel speeds and command queue are idle."""
    return all(
        abs(r.wl) <= WHEEL_STOP_TOLERANCE_MM_S and
        abs(r.wr) <= WHEEL_STOP_TOLERANCE_MM_S and
        not r.pending and abs(r.cmd_left) <= WHEEL_STOP_TOLERANCE_MM_S and
        abs(r.cmd_right) <= WHEEL_STOP_TOLERANCE_MM_S
        for r in world.rovers.values()
    )


def _request_truth_stop(world) -> None:
    """Queue STOP once per rover while allowing motor latency/coast-down to elapse."""
    for rid, rover in world.rovers.items():
        if rover.pending or abs(rover.cmd_left) > WHEEL_STOP_TOLERANCE_MM_S \
                or abs(rover.cmd_right) > WHEEL_STOP_TOLERANCE_MM_S \
                or abs(rover.wl) > WHEEL_STOP_TOLERANCE_MM_S \
                or abs(rover.wr) > WHEEL_STOP_TOLERANCE_MM_S:
            world.set_command(rid, STOP, world.t)


def run_scenario(sc: SC.Scenario, cfg: Config = DEFAULT, max_time: float = 300.0,
                 stall_s: float = 300.0, record: list | None = None) -> RunResult:
    world, emu = SC.instantiate(sc, cfg)
    from ..coordination.supervisor import SupervisorParams
    # deterministic planning in simulation: expansion caps, not wall clock, bound the planners
    ids = list(world.rovers)
    params = SupervisorParams(nav_deadline_s=30.0, push_deadline_s=3.0)
    # The production supervisor accepts deterministic simulation parameters;
    # keep the narrow test double API (ids, cfg) compatible as well.
    try:
        inspect.signature(Supervisor).bind(ids, cfg, params)
    except TypeError:
        sup = Supervisor(ids, cfg)
    else:
        sup = Supervisor(ids, cfg, params)
    grid_rows = sc.board_rows
    depots = {}
    for color, (col, row) in sc.depots.items():
        depots[color] = DepotZone(color, col * sc.cell_mm, (grid_rows - row) * sc.cell_mm, cfg.depot.half_size)
    res = RunResult(seed=sc.seed, family=sc.family, n_cubes=len(world.cubes))
    physical_reason = infeasibility(world, depots, cfg)
    planner_reason = planner_rejection(world, depots, cfg)
    res.physical_infeasibility = physical_reason
    res.planner_rejection = planner_reason
    emu.set_phase("RUNNING")
    next_ctrl = 0.0
    err_p, err_h = [], []
    was_in: dict[str, bool] = {c: False for c in world.cubes}
    last_progress_t = 0.0
    previous_cube_pose = {c: world.cube_pose(c) for c in world.cubes}
    delivery_valid_since: float | None = None
    delivery_participation: tuple[dict[int, int], dict[int, int]] | None = None
    stop_requested = False
    duration = {"collision": 0.0, "non_target_contact": 0.0}
    total_duration = {"collision": 0.0, "non_target_contact": 0.0}
    max_duration = {"collision": 0.0, "non_target_contact": 0.0}
    cube_last_seen = {c: 0.0 for c in world.cubes}
    max_occ = 0.0
    t = 0.0
    while t < max_time:
        world.step(PHYS_DT)
        t = world.t
        active_collision = bool(getattr(world, "_collision_active", ()))
        active_non_target = any(
            world.targets.get(rid) != color
            for rid, color in getattr(world, "_contact_active", ())
        )
        for key, active in (("collision", active_collision), ("non_target_contact", active_non_target)):
            if active:
                duration[key] += PHYS_DT
                total_duration[key] += PHYS_DT
            else:
                duration[key] = 0.0
            max_duration[key] = max(max_duration[key], duration[key])
        truth_now = truth_delivered(world, depots, cfg)
        for color, ok in truth_now.items():
            if was_in[color] and not ok:
                res.delivery_undone += 1
            was_in[color] = ok
        if all(truth_now.values()) and truth_now:
            if delivery_valid_since is None:
                delivery_valid_since = t
                if res.delivery_time is None:
                    res.delivery_time = t
                delivery_participation = _participation(world, truth_now)
            if not stop_requested:
                _request_truth_stop(world)
                stop_requested = True
        else:
            delivery_valid_since = None
        emu.capture(world, t)
        for msg in emu.poll(t):
            try:
                frame = parse_message(msg, t, cfg)
            except (TelemetryError, KeyError, ValueError):
                continue
            sup.ingest(frame, t)
            for color, co in frame.cubes.items():
                if co.age_s <= 0.06:
                    cube_last_seen[color] = t
        if t + 1e-9 >= next_ctrl:
            next_ctrl += CONTROL_DT
            cmds = {} if stop_requested else sup.tick(t)
            for rid, wc in cmds.items():
                world.set_command(rid, wc, t)
                ag = sup.agents[rid]
                # the task's own cube is never a NON-target contact (approach/alignment may brush it)
                world.set_target(rid, ag.task.color if ag.task else None)
            for rid in world.rovers:
                if rid in sup.est:
                    e = sup.est[rid].estimate(t)
                    x, y, th = world.rover_pose(rid)
                    err_p.append(math.hypot(e.pose.x - x, e.pose.y - y))
                    err_h.append(abs(math.remainder(e.pose.theta - th, 2 * math.pi)))
            for c in world.cubes:
                max_occ = max(max_occ, t - cube_last_seen[c])
            dl = truth_now
            n_ok = sum(dl.values())

            # Count actual cube motion as useful task progress.  A mission can need
            # intermediate pushes away from a depot, so depot-count increments alone
            # are not a valid stall signal.
            task_colors = {
                a.task.color for a in sup.agents.values()
                if a.task is not None and a.engaged
            }
            moved = False
            for color, pose in ((c, world.cube_pose(c)) for c in world.cubes):
                old = previous_cube_pose[color]
                if color in task_colors and (
                        math.hypot(pose[0] - old[0], pose[1] - old[1]) > 0.5 \
                        or abs(pose[2] - old[2]) > math.radians(0.5)
                ):
                    moved = True
                previous_cube_pose[color] = pose
            if moved or n_ok == len(world.cubes):
                last_progress_t = t
            if record is not None:
                record.append((t, {r: world.rover_pose(r) for r in world.rovers},
                               {c: world.cube_pose(c) for c in world.cubes},
                               {r: sup.agents[r].state.value for r in world.rovers}))
            if n_ok == len(world.cubes) and delivery_valid_since is not None \
                    and t - delivery_valid_since >= SETTLING_INTERVAL_S \
                    and _all_rovers_stopped(world):
                res.settled_completion_time = t
                break
            if world.counts.get("rover_fell", 0):
                break
            if t - last_progress_t > stall_s:
                break
    dl = truth_delivered(world, depots, cfg)
    res.delivered = sum(dl.values())
    transported, deposited = delivery_participation or _participation(world, dl)
    res.rover_transport_counts = transported
    res.rover_delivery_counts = deposited
    res.participating_rovers = sorted(
        rid for rid in world.rovers
        if transported.get(rid, 0) >= 1 and deposited.get(rid, 0) >= 1
    )
    res.sim_time = round(t, 2)
    wc = world.counts
    res.rover_rover_collisions = wc.get("collision", 0)
    res.non_target_contacts = wc.get("non_target_contact", 0)
    # Diagnostic only: rotations with a cube do not make the run a failure.
    res.rotations_with_cube = wc.get("rotations_with_cube", 0)
    res.rover_exits = wc.get("rover_exit", 0)
    res.falls = wc.get("rover_fell", 0)
    res.cube_exits = wc.get("cube_exit", 0)
    for a in sup.agents.values():
        res.replans += a.stats.replans
        res.recoveries += a.stats.recoveries
        res.estops += a.stats.estops
        res.capture_failures += a.stats.capture_failures
    ct = [x for a in sup.agents.values() for x in a.stats.push_cross_track]
    res.push_cross_track_p95 = float(np.percentile(ct, 95)) if ct else None
    res.est_pos_rmse = float(np.sqrt(np.mean(np.square(err_p)))) if err_p else None
    res.est_heading_rmse_deg = float(np.degrees(np.sqrt(np.mean(np.square(err_h))))) if err_h else None
    res.max_occlusion_s = round(max_occ, 2)
    # `duration` is the final active episode; max_duration captures the complete
    # episode severity while the runner was advancing truth.
    res.collision_duration_s = round(total_duration["collision"], 2)
    res.non_target_contact_duration_s = round(total_duration["non_target_contact"], 2)
    res.max_collision_duration_s = round(max_duration["collision"], 2)
    res.max_non_target_contact_duration_s = round(max_duration["non_target_contact"], 2)
    res.counters = dict(sup.counters)
    res.last_events = [str(e) for e in sup.events[-25:]]
    safety = res.rover_rover_collisions or res.falls or res.rover_exits or res.cube_exits
    participation_ok = res.n_cubes < 2 or len(res.participating_rovers) == len(world.rovers)
    if res.settled_completion_time is not None and participation_ok and not safety and not res.non_target_contacts:
        res.completion_time = res.settled_completion_time
        res.outcome = "success"
    else:
        res.outcome = "fail"
        res.failure_class = classify(res, sup, t, max_time, last_progress_t, stall_s,
                                     planner_reason, physical_reason)
        if res.failure_class == "coordination_deadlock":
            res.deadlocks = max(1, res.deadlocks)
        if physical_reason and res.failure_class not in ("coordination_collision", "board_exit",
                                                         "non_target_contact", "cube_exit"):
            res.failure_class = "physically_infeasible"
            res.detail = physical_reason
        elif res.failure_class == "planner_rejected":
            res.detail = planner_reason or res.detail
    return res


def infeasibility(world, depots: dict[str, DepotZone], cfg: Config) -> str:
    """Return only independently proven physical impossibility.

    Planner clearance/corridor checks are conservative model decisions, not proofs
    about the simulator's ground-truth contact dynamics, and therefore belong in
    :func:`planner_rejection` instead.
    """
    for color, cube in world.cubes.items():
        if color not in depots:
            return f"cube '{color}' has no depot"
        if not cube.in_play:
            return f"cube '{color}' starts out of play"
        if depots[color].half < cfg.cube.side / 2.0:
            return (f"depot '{color}' half-size {depots[color].half:.1f} mm is smaller than "
                    f"the cube half-side {cfg.cube.side / 2.0:.1f} mm")
    return ""


def planner_rejection(world, depots: dict[str, DepotZone], cfg: Config) -> str:
    """Return the planner's conservative rejection reason, without calling it physical."""
    from ..planning.push_planner import classify_unsolvable
    from ..world import CubeEstimate

    cubes = {c: CubeEstimate(c, cb.x, cb.y, pos_std=0.5)
             for c, cb in world.cubes.items() if cb.in_play and c in depots}
    W, H = cfg.board.width, cfg.board.height
    for color, cube in cubes.items():
        why = classify_unsolvable(cfg, cube, depots[color],
                                  [o for k, o in cubes.items() if k != color], W, H)
        if why:
            return why
    return ""


def classify(res: RunResult, sup: Supervisor, t: float, max_time: float, last_progress_t: float,
             stall_s: float, planner_reason: str = "", physical_reason: str = "") -> str:
    if res.rover_rover_collisions:
        return "coordination_collision"
    if res.falls or res.rover_exits:
        return "board_exit"
    if res.cube_exits:
        return "cube_exit"
    if res.non_target_contacts:
        return "non_target_contact"
    if planner_reason:
        return "planner_rejected"
    evs = [e[2] for e in sup.events]
    if res.delivered < res.n_cubes:
        if "no_plan" in evs and res.replans == 0 and res.delivered == 0:
            return "planner_rejected"
        if sup.counters.get("waiting_all_rovers", 0) > 100 or (res.est_pos_rmse or 0) > 15:
            return "perception_estimation"
        if res.capture_failures >= 3:
            return "controller_capture"
        if sup.counters.get("yield_failed", 0) > 3 or sup.counters.get("commit_denied", 0) > 200:
            return "coordination_deadlock"
        if t - last_progress_t > stall_s:
            return "stall"
        return "timeout"
    if res.n_cubes >= 2 and len(res.participating_rovers) < len(res.rover_transport_counts):
        return "participation"
    if res.non_target_contacts:
        return "non_target_contact"
    return "unclassified"


def run_seed(seed: int, family: str, max_time: float = 300.0, cfg: Config = DEFAULT) -> RunResult:
    return run_scenario(SC.generate(seed, family, cfg), cfg, max_time)
