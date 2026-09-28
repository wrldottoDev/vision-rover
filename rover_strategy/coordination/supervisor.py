"""Mission supervisor (v2, after Astra Gate 3): the PC-side brain.  Owns the world model (estimators), planning
services, task allocation, spatial reservations, the reactive safety layer and deadlock resolution.

Coordination invariant: a rover moves only inside the region it has COMMITTED; commits are granted only when
disjoint (with margin) from the other rover's reservation, which always contains that rover's current envelope and,
while it is moving / engaged / interrupted mid-manoeuvre, its whole committed region.  Reservations shrink only for
rovers that are passive (IDLE/DONE/WAIT-not-engaged).  A rover without a trusted estimate keeps its last reservation
plus an uncertainty disc.  An execution guard stops any command whose braking sweep leaves the field, enters the
other rover's reservation, or meets a non-target cube; imminent-collision is an independent last layer.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from ..config import DEFAULT, ROVER_MOTORS, Config
from ..estimation.cube_tracker import CubeTracker
from ..estimation.pose_estimator import PoseEstimator
from ..frames import wrap
from ..geometry import shapes
from ..geometry.footprint import Footprint
from ..geometry.zones import DepotZone, cube_half_extent
from ..planning.navigation import (DiscObstacle, NavPlanner, PolyObstacle, path_collides, pose_collides,
                                   swept_polygons)
from ..planning.push_planner import PushPlanner
from ..planning.reservations import ReservationTable, imminent_collision, priority
from ..planning.task_allocator import TaskAllocator
from ..rover.fsm import PASSIVE, RoverAgent, S, Task
from ..world import STOP, CubeEstimate, Frame, Path, Pose, RoverEstimate, TrackQuality, WheelCommand

OBSTACLE_HALF_DIAG = 100.0 / math.sqrt(2.0)      # official yellow blocks: 10 cm


def _disc_poly(x: float, y: float, r: float, n: int = 16) -> np.ndarray:
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    rr = r / math.cos(math.pi / n)                 # circumscribed polygon => conservative
    return np.stack([x + rr * np.cos(a), y + rr * np.sin(a)], axis=1)


def _inflate(poly: np.ndarray, pad: float, n: int = 12) -> np.ndarray:
    """Conservative Minkowski sum of a convex polygon with a disc (circumscribed n-gon)."""
    if pad <= 0.0:
        return poly
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    rr = pad / math.cos(math.pi / n)
    ring = np.stack([rr * np.cos(a), rr * np.sin(a)], axis=1)
    return shapes.hull((poly[:, None, :] + ring[None, :, :]).reshape(-1, 2))


@dataclass(frozen=True)
class SupervisorParams:
    estop_horizon_s: float = 0.45           # control tick + command latency + braking (not a planning horizon)
    estop_clear_s: float = 0.5
    deadlock_no_progress_s: float = 3.0
    yield_hold_s: float = 25.0
    failed_cube_cooldown_s: float = 6.0
    fresh_obs_age_s: float = 0.06
    stale_feed_s: float = 0.5
    future_tolerance_s: float = 0.05
    clock_window_s: float = 20.0
    guard_lookahead_s: float = 0.35
    guard_brake_mm: float = 12.0
    lost_rover_growth: float = 60.0          # mm added to a lost rover's reservation disc
    mission_min_cubes: int = 2
    parking_candidates: int = 20


def _min_sep(fp: Footprint, A: list[RoverEstimate], B: list[RoverEstimate], horizon: float) -> float:
    """Minimum envelope separation over constant-twist rollouts of every pair in A x B."""
    best = math.inf
    for a in A:
        for b in B:
            for k in range(1, 9):
                s = horizon * k / 8
                pa = [(e.pose.x + e.v * s * math.cos(e.pose.theta + e.omega * s / 2),
                       e.pose.y + e.v * s * math.sin(e.pose.theta + e.omega * s / 2), e.pose.theta + e.omega * s)
                      for e in (a, b)]
                best = min(best, shapes.distance(fp.envelope(*pa[0]), fp.envelope(*pa[1])))
    return best


class Supervisor:
    def __init__(self, rover_ids: list[int], cfg: Config = DEFAULT, params: SupervisorParams | None = None):
        self.cfg = cfg
        self.p = params or SupervisorParams()
        self.ids = sorted(rover_ids)
        self.fp = Footprint(cfg.rover)
        # Navigation inflates the footprint by pose_uncertainty for EVERYTHING incl. the field edge; reduce its board
        # margin accordingly so the board contract is identical to the push planner's: true footprint >= margins.board.
        from dataclasses import replace as _r
        self.nav = NavPlanner(_r(cfg, margins=_r(cfg.margins, board=cfg.margins.board - cfg.margins.pose_uncertainty)))
        self.pusher = PushPlanner(cfg)
        self.alloc = TaskAllocator(cfg)
        self.res = ReservationTable(cfg)
        self.est: dict[int, PoseEstimator] = {}
        self.cubes: dict[str, CubeTracker] = {}
        self.fresh: dict[str, deque] = {}
        self.depots: dict[str, DepotZone] = {}
        self.obstacles: tuple[tuple[float, float], ...] = ()
        self.agents = {i: RoverAgent(i, cfg) for i in self.ids}
        self.board_w, self.board_h = cfg.board.width, cfg.board.height
        self.phase = "IDLE"
        self.newest_capture = -math.inf
        self._offsets: deque = deque()
        self.delivered: set[str] = set()
        self.delivered_by: dict[int, int] = {i: 0 for i in self.ids}
        self.claimed: dict[str, int] = {}
        self.cooldown: dict[str, float] = {}
        self.estop_since: dict[int, float] = {}
        self.committed: dict[int, list[np.ndarray]] = {i: [] for i in self.ids}
        self.last_region: dict[int, list[np.ndarray]] = {i: [] for i in self.ids}
        self.cmd: dict[int, tuple[float, float]] = {i: (0.0, 0.0) for i in self.ids}
        self.nav_blocked: dict[int, bool] = {i: False for i in self.ids}
        self.yield_pairs: dict[int, tuple[int, str | None]] = {}     # yielder -> (keeper, keeper task colour)
        self.events: list[tuple] = []
        self.counters: dict[str, int] = {}
        self.t = 0.0
        self.t_running: float | None = None

    # ================================================================== ingest
    def _clock_shift(self, frame: Frame) -> float:
        """Map capture clock -> local clock.  shift = min over a window of (received - capture); this bounds
        the unknown offset + minimum latency.  It under-estimates ages by at most the true minimum latency."""
        self._offsets.append((frame.t_received, frame.t_received - frame.t_capture))
        while self._offsets and self._offsets[0][0] < frame.t_received - self.p.clock_window_s:
            self._offsets.popleft()
        return min(o for _, o in self._offsets)

    def ingest(self, frame: Frame, t_now: float) -> None:
        shift = self._clock_shift(frame)
        self.phase = frame.phase
        self.board_w, self.board_h = frame.board_w, frame.board_h
        self.depots = frame.depots
        self.obstacles = frame.obstacles
        tcap = frame.t_capture + shift
        if tcap > t_now + self.p.future_tolerance_s or tcap <= self.newest_capture:
            self._count("frame_ignored")
            return                                   # future-dated / duplicate / frozen / out-of-order
        self.newest_capture = tcap
        for rid, obs in frame.rovers.items():
            if rid not in self.agents:
                continue
            tc = obs.t_capture + shift
            e = self.est.get(rid)
            if e is None:
                if obs.age_s > self.cfg.telemetry.good_age_s:
                    continue
                e = self.est[rid] = PoseEstimator(self.cfg.estimator, self.cfg.telemetry, rid)
                e.initialize(obs.pose, tc)
            else:
                e.update_vision(obs.pose, tc, t_now)
        for color, obs in frame.cubes.items():
            tr = self.cubes.get(color)
            if tr is None:
                tr = self.cubes[color] = CubeTracker(color)
                self.fresh[color] = deque(maxlen=60)
            tc = obs.t_capture + shift
            tr.update(obs.x, obs.y, tc, obs.age_s)
            q = self.fresh[color]
            if obs.age_s <= self.p.fresh_obs_age_s and (not q or tc > q[-1][0]):
                q.append((tc, obs.x, obs.y))

    # ================================================================== context API used by agents
    def cube(self, color: str) -> CubeEstimate | None:
        tr = self.cubes.get(color)
        return tr.estimate(self.t) if tr else None

    def fresh_cube(self, color: str, since: float):
        """Mean of DISTINCT fresh captures of the cube since `since`: (xy, n, newest capture time) or None."""
        q = self.fresh.get(color)
        if not q:
            return None
        pts = [(tc, x, y) for (tc, x, y) in q if tc >= since]
        if not pts:
            return None
        a = np.array([(x, y) for _, x, y in pts])
        return a.mean(axis=0), len(pts), pts[-1][0]

    def _cube_obstacles(self, exclude: str | None = None, target: str | None = None) -> list[DiscObstacle]:
        """Cubes (and official obstacles) as discs.  The TARGET cube of a navigation query gets a tighter
        margin: its pre-push pose is designed to sit only prepush_clearance (+pose_uncertainty) away from it."""
        m = self.cfg.margins
        out = []
        for color, tr in self.cubes.items():
            if color == exclude:
                continue
            c = tr.estimate(self.t)
            alpha = c.alpha if (c.alpha is not None and c.alpha_std < math.radians(15)) else None
            if color == target:
                r = cube_half_extent(self.cfg.cube.side, alpha) + m.prepush_clearance - 2.0 + min(c.pos_std, 2.0)
            else:
                r = cube_half_extent(self.cfg.cube.side, alpha) + m.cube_nav + min(c.pos_std, 15.0)
            out.append(DiscObstacle(c.x, c.y, r))
        for (x, y) in self.obstacles:
            out.append(DiscObstacle(x, y, OBSTACLE_HALF_DIAG + m.cube_nav))
        return out

    def _other_region(self, rid: int) -> list[PolyObstacle]:
        """The other rover's reservation polygons, each inflated by the rover-rover margin -- or, where my
        envelope is already closer than that to a polygon (start poses, adjacent reservations), by the current
        distance minus a little: never plan closer to anything than I already am."""
        mine = self._env_now(rid) if rid in self.est else None
        out = []
        for other in self.ids:
            if other == rid:
                continue
            for q in self.res.polygons_of(other):
                if mine is None or shapes.bbox_gap(mine, q) > 400.0:
                    out.append(PolyObstacle(_inflate(q, self.cfg.margins.rover_rover)))
                    continue
                pad = min(self.cfg.margins.rover_rover, max(0.0, shapes.distance(mine, q) - 3.0))
                out.append(PolyObstacle(_inflate(q, pad)))
        return out

    def plan_nav(self, rid: int, goal: Pose, ignore_other: bool = False, deadline_s: float = 1.5):
        """ponytail: planning runs synchronously inside tick(); fine in simulation (sim time is frozen while we
        compute) but a real deployment must run it in a worker thread and re-check freshness before commanding."""
        start = self.est[rid].estimate(self.t).pose
        task = self.agents[rid].task
        cubes = self._cube_obstacles(target=task.color if task else None)
        obs = cubes + ([] if ignore_other else self._other_region(rid))
        path = self.nav.plan(start, goal, obs, self.board_w, self.board_h, deadline_s=deadline_s)
        self.nav_blocked[rid] = False
        if path is None:
            self.log(rid, "nav_fail", why=self.nav.last_failure)
            if not ignore_other and self.nav.plan(start, goal, cubes, self.board_w, self.board_h,
                                                  deadline_s=deadline_s) is not None:
                self.nav_blocked[rid] = True        # only the other rover is in the way: that is WAIT, not failure
        return path

    def _cube_list(self, exclude: str | None) -> list[CubeEstimate]:
        out = [self.cube(c) for c in self.cubes if c != exclude]
        for (x, y) in self.obstacles:                 # obstacles as pseudo-cubes of unknown orientation
            out.append(CubeEstimate("obstacle", x, y, pos_std=5.0))
        return out

    def plan_push(self, rid: int, color: str):
        cube = self.cube(color)
        depot = self.depots.get(color)
        if cube is None or depot is None:
            return None
        plans = self.pusher.plan(cube, depot, self._cube_list(color), self.board_w, self.board_h, max_plans=3)
        if not plans:
            self.log(rid, "push_plan_fail", color=color, why=self.pusher.last_failure)
            return None
        return plans[0]

    def manipulation_region(self, rid: int, start: Pose, heading: float, travel: float) -> list[np.ndarray]:
        """Everything a capture/push/abort from `start` along `heading` can sweep: an in-place rotation disc at
        the start (alignment + nudges), the straight advance, and a full retreat from any point of it."""
        s = start.rotated_to(heading)
        back = self.cfg.rover.paddle_reach + self.agents[rid].p.retreat_extra
        end = s.moved(travel)
        return [_disc_poly(s.x, s.y, self.fp.sweep_radius + 25.0),
                self.fp.straight_sweep(s.x, s.y, s.theta, travel),
                self.fp.straight_sweep(s.x, s.y, s.theta, -back),
                self.fp.straight_sweep(end.x, end.y, end.theta, -back)]

    def _env_now(self, rid: int) -> np.ndarray:
        p = self.est[rid].estimate(self.t).pose
        return self.fp.envelope(p.x, p.y, p.theta)

    def _conflict(self, rid: int, polys: list[np.ndarray]) -> bool:
        """Does `polys` come within the rover-rover margin of the other rover's reservation?  Where my current
        envelope is already closer than the margin to one of its polygons, the threshold for that polygon is the
        current distance (minus the polygon-approximation tolerance): motion may not bring us closer."""
        margin = self.cfg.margins.rover_rover
        mine = self._env_now(rid)
        for other in self.ids:
            if other == rid or other not in self.est:
                continue
            for q in self.res.polygons_of(other):
                d_now = shapes.distance(mine, q)
                thr = max(0.0, min(margin, max(0.0, d_now - 3.0)) - 4.0)
                if thr <= 0.0 and d_now > 0.0:
                    thr = 0.0
                for i_p, p in enumerate(polys):
                    if shapes.bbox_gap(p, q) >= max(thr, 1e-9):
                        continue
                    dpq = shapes.distance(p, q)
                    if dpq < thr or (thr == 0.0 and shapes.overlap(p, q) and d_now > 0.0):
                        self.last_conflict = (i_p, len(polys),
                                              round(dpq, 1), round(thr, 1), round(d_now, 1))
                        return True
        return False

    def _commit(self, rid: int, polygons: list[np.ndarray]) -> bool:
        region = list(polygons) + [self._env_now(rid)]
        if self._conflict(rid, region):
            self._count("commit_denied")
            self.log(rid, "commit_denied", why=getattr(self, "last_conflict", None))
            return False
        self.committed[rid] = region
        self.res.reserve(rid, region, priority(self.agents[rid].priority_state(), rid), self.t)
        return True

    def commit_path(self, rid: int, path: Path, extra: list[np.ndarray]) -> bool:
        """The path itself was planned in this same tick with the other rover's reservation as obstacles (exact
        planner collision checks), so only the extra (manipulation) regions need the reservation conflict test;
        re-testing the conservatively padded sweep polygons would reject every move of rovers starting < margin
        apart."""
        if self._conflict(rid, list(extra)):
            self._count("commit_denied")
            self.log(rid, "commit_denied", why=getattr(self, "last_conflict", None))
            return False
        region = swept_polygons(self.fp, path) + list(extra) + [self._env_now(rid)]
        self.committed[rid] = region
        self.res.reserve(rid, region, priority(self.agents[rid].priority_state(), rid), self.t)
        return True

    def shrink_region(self, rid: int, remaining: Path, extra: list[np.ndarray]) -> None:
        """Replace the committed region by the sweep of the REMAINING path (+extra) and the current envelope.
        Only ever shrinks the claim, so no conflict check is needed."""
        region = swept_polygons(self.fp, remaining) + list(extra) + [self._env_now(rid)]
        self.committed[rid] = region
        self.res.reserve(rid, region, priority(self.agents[rid].priority_state(), rid), self.t)

    def commit_region(self, rid: int, polygons: list[np.ndarray]) -> bool:
        return self._commit(rid, self.committed.get(rid, []) + list(polygons))

    def path_still_clear(self, rid: int, path: Path, from_seg: int) -> bool:
        """Revalidate the rest of a committed path in the planning context (cubes incl. the tighter target
        margin, obstacles, other rover's reservation, footprint inflation, board margin).  If the rover is
        currently inside a margin (e.g. it is escaping), the segment being executed is only checked against hard
        contact; everything after it gets the full planning contract."""
        task = self.agents[rid].task if rid in self.agents else None
        cubes = self._cube_obstacles(target=task.color if task else None)
        full = cubes + self._other_region(rid)
        fpi = Footprint(self.cfg.rover, inflate=self.cfg.margins.pose_uncertainty)
        cur = Path(path.segments[from_seg:from_seg + 1], 0.0)
        rest = Path(path.segments[from_seg + 1:], 0.0)
        pose = self.est[rid].estimate(self.t).pose if rid in self.est else None
        bm = self.cfg.margins.board - self.cfg.margins.pose_uncertainty     # same contract as self.nav
        inside_margins = pose is None or not pose_collides(fpi, pose, full, self.board_w, self.board_h, bm)
        if cur.segments:
            if inside_margins:
                if path_collides(fpi, cur, full, self.board_w, self.board_h, bm):
                    return False
            else:
                hard = [DiscObstacle(c.x, c.y, c.r - self.cfg.margins.cube_nav) for c in cubes]
                if path_collides(self.fp, cur, hard, self.board_w, self.board_h, 0.0):
                    return False
        return not (rest.segments and path_collides(fpi, rest, full, self.board_w, self.board_h, bm))

    def retreat_clear(self, rid: int, pose: Pose, dist: float, target: str | None) -> bool:
        sweep = self.fp.straight_sweep(pose.x, pose.y, pose.theta, -dist)
        if not shapes.inside_rect(sweep, 0.0, self.board_w, 0.0, self.board_h):
            return False
        for c in self._cube_obstacles(exclude=target):
            if shapes.disc_distance(sweep, (c.x, c.y), c.r - self.cfg.margins.cube_nav) <= 0.0:
                return False
        return not self._conflict(rid, [sweep])

    def _confirm_frames(self, color: str, since: float, t: float):
        q = [(tc, x, y) for (tc, x, y) in self.fresh.get(color, ()) if tc >= since]
        n = self.cfg.depot.confirm_frames
        if len(q) < n or t - q[-1][0] > 0.25:
            return None
        q = q[-n:]
        a = np.array([(x, y) for _, x, y in q])
        if np.ptp(a[:, 0]) > 4.0 or np.ptp(a[:, 1]) > 4.0:     # not stationary
            return None
        return a

    def _worst_half(self, color: str) -> float:
        c = self.cube(color)
        side = self.cfg.cube.side
        if c is None or c.alpha is None or c.alpha_std >= math.radians(20):
            return cube_half_extent(side, None)
        lo, hi = c.alpha - 2 * c.alpha_std, c.alpha + 2 * c.alpha_std
        cands = [lo, hi] + [k * math.pi / 4 for k in range(-8, 9) if lo <= k * math.pi / 4 <= hi]
        return max(cube_half_extent(side, a) for a in cands)

    def is_delivered(self, color: str, since: float, t: float) -> bool:
        """ENGINEERING placement confirmation under the ASSUMED depot model (config.depot); not an official
        verdict.  Needs n distinct, fresh, stationary captures after `since`, whole footprint (worst case over
        the orientation belief) inside the zone."""
        depot = self.depots.get(color)
        a = self._confirm_frames(color, since, t)
        if depot is None or a is None:
            return False
        h = self._worst_half(color)
        x0, x1, y0, y1 = depot.bounds
        return bool(np.all((a[:, 0] - h >= x0) & (a[:, 0] + h <= x1) & (a[:, 1] - h >= y0) & (a[:, 1] + h <= y1)))

    def on_delivered(self, rid: int, color: str, t: float) -> None:
        self.delivered.add(color)
        self.delivered_by[rid] = self.delivered_by.get(rid, 0) + 1
        self.log(rid, "delivered", color=color)

    def release_task(self, rid: int, color: str, failed: bool, t: float) -> None:
        self.claimed.pop(color, None)
        if failed:
            self.cooldown[color] = t + self.p.failed_cube_cooldown_s
            self._count("task_released_failed")

    def set_cube_orientation(self, color: str, alpha, std: float) -> None:
        if color in self.cubes:
            self.cubes[color].set_orientation(alpha, std)

    def set_cube_pushed(self, color: str, flag: bool) -> None:
        if color in self.cubes:
            self.cubes[color].set_pushed(flag)

    def log(self, rid: int, event: str, **kw) -> None:
        self.events.append((round(self.t, 3), rid, event, kw))

    def _count(self, k: str) -> None:
        self.counters[k] = self.counters.get(k, 0) + 1

    # ================================================================== main tick
    def tick(self, t: float) -> dict[int, WheelCommand]:
        self.t = t
        ests: dict[int, RoverEstimate] = {}
        for rid in self.ids:
            if rid in self.est:
                self.est[rid].predict(t)
                ests[rid] = self.est[rid].estimate(t)
        feed_stale = (t - self.newest_capture) > self.p.stale_feed_s
        if self.phase != "RUNNING" or feed_stale or len(ests) < len(self.ids):
            # Every rover must be localised before anything moves: an unseen rover has no reservation.
            if self.phase == "RUNNING" and len(ests) < len(self.ids):
                self._count("waiting_all_rovers")
            return self._stop_all(t)
        if self.t_running is None:
            self.t_running = t
        safe = {rid: self.est[rid].is_safe_to_drive(t) for rid in self.ids}
        self.lost_reasons = {rid: getattr(self.est[rid], "lost_reasons", []) for rid in self.ids}

        self._refresh_reservations(ests, safe)
        self._monitor_deliveries(t)
        self._allocate(t, ests, safe)

        for rid in self.ids:
            self.cmd[rid] = self.agents[rid].step(t, ests[rid], safe[rid], self)

        self._execution_guard(t, ests)
        self._imminent_collision(t, ests)
        self._resolve_deadlock(t, ests)
        out = {}
        for rid in self.ids:
            v, w = self.cmd[rid]
            if ests[rid].quality == TrackQuality.DEGRADED:        # stale-ish vision: keep going, slowly
                k = self.cfg.telemetry.degraded_speed_scale
                v, w = v * k, w * k
            wc = WheelCommand.from_unicycle(v, w, self.cfg.rover.track_width, self.cfg.limits.wheel_speed_max)
            v2, w2 = wc.unicycle(self.cfg.rover.track_width)
            self.est[rid].set_command(t, v2, w2)                 # the estimator models the INTENDED twist
            mc = ROVER_MOTORS.get(rid)
            if mc is not None:                                    # per-rover feed-forward (hardware asymmetry)
                wc = WheelCommand(wc.v_left * mc.left_scale, wc.v_right * mc.right_scale)
            out[rid] = wc
        return out

    def _stop_all(self, t: float) -> dict[int, WheelCommand]:
        for rid in self.ids:
            self.cmd[rid] = (0.0, 0.0)
            if rid in self.est:
                self.est[rid].set_command(t, 0.0, 0.0)
        return {rid: STOP for rid in self.ids}

    # ================================================================== reservations
    def _refresh_reservations(self, ests: dict[int, RoverEstimate], safe: dict[int, bool]) -> None:
        for rid, e in ests.items():
            a = self.agents[rid]
            env = self.fp.envelope(e.pose.x, e.pose.y, e.pose.theta)
            if not safe[rid]:
                # untrusted estimate: keep the last reservation, grown by an uncertainty disc
                region = self.last_region[rid] + [_disc_poly(e.pose.x, e.pose.y,
                                                             self.fp.sweep_radius + self.p.lost_rover_growth)]
            elif a.state in PASSIVE or (a.state == S.WAIT and not a.engaged):
                region = [env]
                self.committed[rid] = []
            else:
                region = self.committed[rid] + [env]
            self.last_region[rid] = region[-8:] if not safe[rid] else region
            self.res.reserve(rid, region, priority(a.priority_state(), rid), self.t)
        if len(self.ids) == 2:
            a, b = self.ids
            if self._conflict(a, self.res.polygons_of(a)):
                self._count("reservation_overlap")

    # ================================================================== deliveries / allocation
    def _monitor_deliveries(self, t: float) -> None:
        for c in list(self.delivered):
            a = self._confirm_frames(c, t - 1.0, t)
            d = self.depots.get(c)
            if a is None or d is None:
                continue
            x, y = a.mean(axis=0)
            if not d.contains_cube(float(x), float(y), self.cfg.cube.side, 0.0, -4.0):
                self.delivered.discard(c)
                self._count("delivery_undone")
                self.log(-1, "delivery_undone", color=c)

    def mission_complete(self) -> bool:
        seen = [c for c in self.cubes if c in self.depots]
        return len(seen) >= self.p.mission_min_cubes and all(c in self.delivered for c in seen) and not self.claimed

    def _allocate(self, t: float, ests: dict[int, RoverEstimate], safe: dict[int, bool]) -> None:
        idle = [rid for rid in self.ids if self.agents[rid].state in PASSIVE and safe[rid]]
        if not idle:
            return
        if self.mission_complete():
            for rid in idle:
                self.agents[rid].state = S.DONE
            return
        for rid in idle:
            if self.agents[rid].state == S.DONE:
                self.agents[rid].state = S.IDLE
        free = [c for c in self.cubes if c in self.depots and c not in self.delivered and c not in self.claimed
                and self.cooldown.get(c, -1) <= t]
        if not free:
            return
        plans = {}
        for c in free:
            ps = self.pusher.plan(self.cube(c), self.depots[c], self._cube_list(c), self.board_w, self.board_h,
                                  max_plans=3)
            if ps:
                plans[c] = ps
            else:
                self.cooldown[c] = t + self.p.failed_cube_cooldown_s
                self.log(-1, "no_plan", color=c, why=self.pusher.last_failure)
        if not plans:
            return
        rovers = {rid: ests[rid] for rid in self.ids if safe[rid]}
        locked = {}
        for rid in rovers:
            task = self.agents[rid].task
            locked[rid] = task.color if task else None
            if task:
                plans.setdefault(task.color, [task.plan])
        alloc = self.alloc.allocate(rovers, plans, locked)
        undelivered_free = len(free)
        for rid in idle:
            seq = [rt for rt in alloc.assignment.get(rid, []) if rt.color in free and rt.color not in self.claimed]
            if not seq:
                continue
            other = [o for o in self.ids if o != rid]
            # Mission-level participation rule: do not take the last free cube from a healthy rover that has not
            # participated yet (reglamento s12: both robots must transport and deposit).
            if undelivered_free == 1 and other:
                o = other[0]
                if safe.get(o) and self.delivered_by.get(o, 0) == 0 and self.agents[o].task is None \
                        and self.delivered_by.get(rid, 0) > 0:
                    continue
            rt = seq[0]
            self.claimed[rt.color] = rid
            self.agents[rid].assign(Task(rt.color, rt.plan), t, self)
            undelivered_free -= 1

    # ================================================================== safety layers
    def _execution_guard(self, t: float, ests: dict[int, RoverEstimate]) -> None:
        """Stop any command whose short braking sweep leaves the field, enters the other rover's reservation or
        touches a non-target cube.  Samples the whole sweep (rotation included)."""
        for rid, e in ests.items():
            v, w = self.cmd[rid]
            if v == 0.0 and w == 0.0:
                continue
            a = self.agents[rid]
            h = self.p.guard_lookahead_s
            vv = v + math.copysign(self.p.guard_brake_mm, v) / h if v != 0.0 else 0.0
            polys = []
            for k in range(1, 6):
                s = h * k / 5
                th = e.pose.theta + w * s
                x = e.pose.x + vv * s * math.cos(e.pose.theta + w * s / 2)
                y = e.pose.y + vv * s * math.sin(e.pose.theta + w * s / 2)
                polys.append(self.fp.envelope(x, y, th))
            why = None
            now_env = self.fp.envelope(e.pose.x, e.pose.y, e.pose.theta)
            now_in = shapes.inside_rect(now_env, 0.0, self.board_w, 0.0, self.board_h)
            if now_in and not all(shapes.inside_rect(p, 0.0, self.board_w, 0.0, self.board_h) for p in polys):
                why = "board"
            elif self._conflict(rid, polys):
                why = "reservation"
            else:
                # Physical non-contact only (planning margins are the planners' job): each cube as a disc of its
                # worst-case half extent + 2 mm; the engaged target cube is excluded.
                target = a.task.color if (a.task and (a.engaged or a.state == S.ALIGN)) else None
                for color, tr in self.cubes.items():
                    if color == target:
                        continue
                    c = tr.estimate(self.t)
                    alpha = c.alpha if (c.alpha is not None and c.alpha_std < math.radians(15)) else None
                    rr = cube_half_extent(self.cfg.cube.side, alpha) + 2.0
                    if any(shapes.disc_distance(p, (c.x, c.y), rr) <= 0.0 for p in polys) and \
                            shapes.disc_distance(now_env, (c.x, c.y), rr) > 0.0:
                        why = "cube"
                        break
            if why:
                self.cmd[rid] = (0.0, 0.0)
                self._count("guard_" + why)
                self.log(rid, "guard", why=why, state=a.state.value)
                if a.state in (S.NAVIGATE, S.YIELD) and not a.engaged:
                    a.path = None                        # force a fresh plan from the actual state
                    a.resume = a.state
                    a._go(S.WAIT, t, self, "guard " + why)

    def _imminent_collision(self, t: float, ests: dict[int, RoverEstimate]) -> None:
        if len(self.ids) != 2:
            return
        a_id, b_id = self.ids
        ea, eb = ests[a_id], ests[b_id]

        d_now = shapes.distance(self.fp.envelope(ea.pose.x, ea.pose.y, ea.pose.theta),
                                self.fp.envelope(eb.pose.x, eb.pose.y, eb.pose.theta))

        def hit() -> bool:
            # Last-resort layer: roll out both the estimated current motion and the commanded motion.  Trigger when
            # the predicted envelope separation drops below the estop margin -- or, for rovers already closer than
            # that (start poses), when it would shrink materially below the current separation.
            A = [ea, self._with_cmd(ea, self.cmd[a_id])]
            B = [eb, self._with_cmd(eb, self.cmd[b_id])]
            thr = min(self.cfg.margins.rover_rover_estop, max(0.0, d_now - 8.0))
            return _min_sep(self.fp, A, B, self.p.estop_horizon_s) < thr

        if (self.cmd[a_id] != (0.0, 0.0) or self.cmd[b_id] != (0.0, 0.0)) and hit():
            pa = priority(self.agents[a_id].priority_state(), a_id)
            pb = priority(self.agents[b_id].priority_state(), b_id)
            low, high = (a_id, b_id) if pa < pb else (b_id, a_id)
            self._estop(low, t)
            if self.cmd[high] != (0.0, 0.0) and hit():
                self._estop(high, t)
        else:
            for rid in (a_id, b_id):
                if self.agents[rid].state == S.EMERGENCY_STOP:
                    since = self.estop_since.setdefault(rid, t)
                    if t - since >= self.p.estop_clear_s:
                        self.estop_since.pop(rid, None)
                        self.agents[rid].clear_estop(t, self)

    def _with_cmd(self, e: RoverEstimate, cmd: tuple[float, float]) -> RoverEstimate:
        return RoverEstimate(e.id, e.pose, cmd[0], cmd[1], e.cov, e.age_s, e.quality)

    def _estop(self, rid: int, t: float) -> None:
        self.cmd[rid] = (0.0, 0.0)
        self.estop_since.pop(rid, None)
        if self.agents[rid].state != S.EMERGENCY_STOP:
            self._count("estop")
            self.log(rid, "estop")
            self.agents[rid].estop(t, self)

    # ================================================================== deadlock / yield
    def _resolve_deadlock(self, t: float, ests: dict[int, RoverEstimate]) -> None:
        if len(self.ids) != 2:
            return
        # release yielders whose keeper finished its task (or switched task)
        for y_id, (k_id, k_color) in list(self.yield_pairs.items()):
            k = self.agents[k_id]
            if k.task is None or k.task.color != k_color or self.agents[y_id].state != S.YIELD:
                self.agents[y_id].release_yield(t)
                del self.yield_pairs[y_id]
        a, b = (self.agents[i] for i in self.ids)
        for waiter, other in ((a, b), (b, a)):
            if waiter.task is None or waiter.engaged or waiter.id in self.yield_pairs:
                continue
            if waiter.state not in (S.WAIT, S.NAVIGATE, S.EMERGENCY_STOP):
                continue
            if t - waiter.last_progress_t < self.p.deadlock_no_progress_s:
                continue
            if other.engaged or other.state == S.YIELD:
                continue                                  # the other is doing real work: keep waiting
            other_stuck = other.task is None or t - other.last_progress_t >= self.p.deadlock_no_progress_s
            if not other_stuck:
                continue
            if other.task is None:
                yielder, keeper = other, waiter
            else:
                yielder, keeper = (other, waiter) if other.id > waiter.id else (waiter, other)
            goal = self._parking_pose(yielder.id, keeper, ests)
            if goal is not None and yielder.order_yield(goal, self.p.yield_hold_s, t, self):
                self._count("yield")
                self.log(yielder.id, "yield", for_rover=keeper.id)
                self.yield_pairs[yielder.id] = (keeper.id, keeper.task.color if keeper.task else None)
                keeper.last_progress_t = t
                yielder.last_progress_t = t
            else:
                self._count("yield_failed")
                waiter.last_progress_t = t                # back off before trying again

    def _keeper_region(self, keeper: RoverAgent, ests) -> list[np.ndarray]:
        e = ests[keeper.id]
        region = [self.fp.envelope(e.pose.x, e.pose.y, e.pose.theta)]
        if keeper.task:
            path = self.plan_nav(keeper.id, keeper.leg.prepush, ignore_other=True)
            if path is not None:
                region += swept_polygons(self.fp, path)
            region += keeper._leg_region(self)
        return region

    def _parking_pose(self, rid: int, keeper: RoverAgent, ests) -> Pose | None:
        region = self._keeper_region(keeper, ests)
        me = ests[rid].pose
        m = self.cfg.margins
        cubes = self._cube_obstacles()
        cands = []
        for x in np.arange(80.0, self.board_w - 60.0, 50.0):
            for y in np.arange(80.0, self.board_h - 60.0, 50.0):
                for th in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    env = self.fp.envelope(x, y, th)
                    if not shapes.inside_rect(env, m.board, self.board_w - m.board, m.board, self.board_h - m.board):
                        continue
                    if any(shapes.disc_distance(env, (c.x, c.y), c.r) <= 0 for c in cubes):
                        continue
                    if any(shapes.distance(env, r) < m.rover_rover for r in region):
                        continue
                    cands.append((math.hypot(x - me.x, y - me.y) + 20 * abs(wrap(th - me.theta)), x, y, th))
        cands.sort()
        keeper_obs = [PolyObstacle(p) for p in region]
        for _, x, y, th in cands[:self.p.parking_candidates]:
            goal = Pose(float(x), float(y), th)
            if self.nav.plan(me, goal, cubes + keeper_obs, self.board_w, self.board_h, deadline_s=0.15) is not None:
                return goal
        return None
