"""Closed-loop simulation of one scenario: physics truth -> official-format telemetry -> parser -> Supervisor ->
wheel commands -> physics.  Outcome judged from GROUND TRUTH, never from the strategy's own beliefs."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..config import Config, DEFAULT
from ..coordination.supervisor import Supervisor
from ..geometry.zones import DepotZone
from ..vision.parser import TelemetryError, parse_message
from . import scenarios as SC

PHYS_DT = 0.01
CONTROL_DT = 0.05


@dataclass
class RunResult:
    seed: int
    family: str
    outcome: str = "fail"                 # success | fail | crash
    failure_class: str = "none"
    detail: str = ""
    n_cubes: int = 0
    delivered: int = 0
    completion_time: float | None = None
    sim_time: float = 0.0
    rover_rover_collisions: int = 0
    non_target_contacts: int = 0
    rover_exits: int = 0
    falls: int = 0
    cube_exits: int = 0
    deadlocks: int = 0
    delivery_undone: int = 0
    replans: int = 0
    recoveries: int = 0
    estops: int = 0
    capture_failures: int = 0
    est_pos_rmse: float | None = None
    est_heading_rmse_deg: float | None = None
    push_cross_track_p95: float | None = None
    max_occlusion_s: float | None = None
    counters: dict = field(default_factory=dict)
    last_events: list = field(default_factory=list)


def truth_delivered(world, depots: dict[str, DepotZone], cfg: Config) -> dict[str, bool]:
    out = {}
    for color, c in world.cubes.items():
        d = depots.get(color)
        out[color] = bool(d and d.contains_cube(c.x, c.y, cfg.cube.side, c.alpha, 0.0))
    return out


def run_scenario(sc: SC.Scenario, cfg: Config = DEFAULT, max_time: float = 300.0,
                 stall_s: float = 300.0, record: list | None = None) -> RunResult:
    world, emu = SC.instantiate(sc, cfg)
    from ..coordination.supervisor import SupervisorParams
    # deterministic planning in simulation: expansion caps, not wall clock, bound the planners
    sup = Supervisor(list(world.rovers), cfg, SupervisorParams(nav_deadline_s=30.0, push_deadline_s=3.0))
    grid_rows = sc.board_rows
    depots = {}
    for color, (col, row) in sc.depots.items():
        depots[color] = DepotZone(color, col * sc.cell_mm, (grid_rows - row) * sc.cell_mm, cfg.depot.half_size)
    res = RunResult(seed=sc.seed, family=sc.family, n_cubes=len(world.cubes))
    infeasible = infeasibility(world, depots, cfg)
    emu.set_phase("RUNNING")
    next_ctrl = 0.0
    err_p, err_h = [], []
    was_in: dict[str, bool] = {c: False for c in world.cubes}
    last_progress_t, best = 0.0, -1
    cube_last_seen = {c: 0.0 for c in world.cubes}
    max_occ = 0.0
    t = 0.0
    while t < max_time:
        world.step(PHYS_DT)
        t = world.t
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
            cmds = sup.tick(t)
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
            dl = truth_delivered(world, depots, cfg)
            for c, ok in dl.items():
                if was_in[c] and not ok:
                    res.delivery_undone += 1
                was_in[c] = ok
            n_ok = sum(dl.values())
            if n_ok > best:
                best, last_progress_t = n_ok, t
            if record is not None:
                record.append((t, {r: world.rover_pose(r) for r in world.rovers},
                               {c: world.cube_pose(c) for c in world.cubes},
                               {r: sup.agents[r].state.value for r in world.rovers}))
            if n_ok == len(world.cubes) and all(a.task is None for a in sup.agents.values()) \
                    and all(not a.engaged for a in sup.agents.values()):
                res.completion_time = t
                break
            if world.counts.get("rover_fell", 0):
                break
            if t - last_progress_t > stall_s:
                break
    dl = truth_delivered(world, depots, cfg)
    res.delivered = sum(dl.values())
    res.sim_time = round(t, 2)
    wc = world.counts
    res.rover_rover_collisions = wc.get("collision", 0)
    res.non_target_contacts = wc.get("non_target_contact", 0)
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
    res.counters = dict(sup.counters)
    res.last_events = [str(e) for e in sup.events[-25:]]
    safety = res.rover_rover_collisions or res.falls or res.rover_exits
    if res.completion_time is not None and not safety and not res.non_target_contacts:
        res.outcome = "success"
    else:
        res.outcome = "fail"
        res.failure_class = classify(res, sup, t, max_time, last_progress_t, stall_s)
        if infeasible and res.failure_class not in ("coordination_collision", "board_exit", "non_target_contact"):
            res.failure_class = "physically_infeasible"
            res.detail = infeasible
    return res


def infeasibility(world, depots: dict[str, DepotZone], cfg: Config) -> str:
    """Ground-truth check (true cube poses) with the push model's necessary conditions."""
    from ..planning.push_planner import classify_unsolvable
    from ..world import CubeEstimate
    cubes = {c: CubeEstimate(c, cb.x, cb.y, pos_std=0.5) for c, cb in world.cubes.items()}
    W, H = cfg.board.width, cfg.board.height
    for c, ce in cubes.items():
        why = classify_unsolvable(cfg, ce, depots[c], [o for k, o in cubes.items() if k != c], W, H)
        if why:
            return why
    return ""


def classify(res: RunResult, sup: Supervisor, t: float, max_time: float, last_progress_t: float,
             stall_s: float) -> str:
    if res.rover_rover_collisions:
        return "coordination_collision"
    if res.falls or res.rover_exits:
        return "board_exit"
    if res.non_target_contacts and res.completion_time is not None:
        return "non_target_contact"
    evs = [e[2] for e in sup.events]
    if res.delivered < res.n_cubes:
        if "no_plan" in evs and res.replans == 0 and res.delivered == 0:
            return "planner_no_solution"
        if sup.counters.get("waiting_all_rovers", 0) > 100 or (res.est_pos_rmse or 0) > 15:
            return "perception_estimation"
        if res.capture_failures >= 3:
            return "controller_capture"
        if sup.counters.get("yield_failed", 0) > 3 or sup.counters.get("commit_denied", 0) > 200:
            return "coordination_deadlock"
        if t - last_progress_t > stall_s:
            return "stall"
        return "timeout"
    if res.non_target_contacts:
        return "non_target_contact"
    return "unclassified"


def run_seed(seed: int, family: str, max_time: float = 300.0, cfg: Config = DEFAULT) -> RunResult:
    return run_scenario(SC.generate(seed, family, cfg), cfg, max_time)
