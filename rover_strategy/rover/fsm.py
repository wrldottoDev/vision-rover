r"""Per-rover mission FSM (v2, after Astra Gate 3).  One `RoverAgent` per rover; the Supervisor owns planning
services (`AgentContext`), reservations and the safety layers.

Task = deliver one cube colour.  A task executes its PushPlan one leg at a time and REPLANS from the observed
cube state after every leg (a stale plan is never executed).

    IDLE -> NAVIGATE -> ALIGN -> CAPTURE -> VERIFY_CAPTURE -> PUSH -> MEASURE --(more legs)--> RETREAT -> replan
                                                                              \--(final)--> VERIFY_DELIVERY -> RETREAT
                                                                                            -> CONFIRM -> IDLE
    interrupts: WAIT (blocked) | RELOCALIZE (estimate unsafe) | EMERGENCY_STOP (safety layer) | YIELD (parking)

Invariants
- `engaged` (the channel may hold a cube) is independent of the state and survives every interrupt.  While
  engaged the only permitted motions are the manipulation itself (capture/push along the committed line) or a
  STRAIGHT retreat.  It clears only after a retreat whose measured backward displacement reached the distance.
- Every failure path increments a bounded per-task budget; every state has a timeout; no state waits forever.
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from ..config import Config
from ..control.controllers import (AlignController, CaptureController, PushController, RetreatController,
                                   SegmentFollower)
from ..frames import angle_diff
from ..world import CubeEstimate, Path, Pose, PushPlan, RoverEstimate, SegKind, TrackQuality


class S(enum.Enum):
    IDLE = "IDLE"
    NAVIGATE = "NAVIGATE"
    ALIGN = "ALIGN"
    CAPTURE = "CAPTURE"
    VERIFY_CAPTURE = "VERIFY_CAPTURE"
    PUSH = "PUSH"
    MEASURE = "MEASURE"
    VERIFY_DELIVERY = "VERIFY_DELIVERY"
    RETREAT = "RETREAT"
    CONFIRM = "CONFIRM"
    WAIT = "WAIT"
    RELOCALIZE = "RELOCALIZE"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    YIELD = "YIELD"
    DONE = "DONE"


INTERRUPTS = {S.WAIT, S.RELOCALIZE, S.EMERGENCY_STOP}
PASSIVE = {S.IDLE, S.DONE}


@dataclass(frozen=True)
class FsmParams:
    nav_timeout_factor: float = 3.0          # x planned time + 8 s
    align_timeout_s: float = 15.0
    capture_timeout_s: float = 10.0
    verify_s: float = 0.35                   # dwell before judging (motion settles)
    verify_timeout_s: float = 3.0
    min_frames: int = 4                      # distinct fresh captures needed to judge contact
    push_timeout_factor: float = 2.5
    push_blind_hold_s: float = 0.6           # cube unseen this long while pushing -> stop and wait
    push_blind_fail_s: float = 3.0           # ... this long -> abort (retreat)
    retreat_timeout_s: float = 8.0
    retreat_blocked_fail_s: float = 6.0
    wait_retry_s: float = 0.5
    relocalize_good_s: float = 0.4
    task_budget: float = 8.0                 # cumulative failure weight per task before the cube is released
    task_time_budget_s: float = 150.0
    cube_moved_tol: float = 25.0             # mm: cube displaced from the planned leg start -> replan
    align_reposition_tol: float = 25.0
    capture_overdrive: float = 6.0
    contact_tol: float = 5.0                 # mm geometric tolerance on channel-fit check
    depth_tol: float = 10.0                  # mm tolerance on contact depth
    flush_tol: float = 3.0                   # depth <= half side + this  => flush (orientation inferable)
    flush_alpha_std: float = math.radians(10.0)
    retreat_extra: float = 25.0              # mm beyond paddle reach
    delivery_retries: int = 2
    nav_track_cross_tol: float = 35.0
    nav_track_heading_tol: float = math.radians(30.0)
    push_line_recommit_deg: float = 3.0
    push_line_max_change_deg: float = 15.0   # beyond this an engaged rover retreats and re-approaches
    retreat_omega_max: float = 0.35          # rad/s heading-hold authority while retreating engaged


class AgentContext(Protocol):
    cfg: Config

    def cube(self, color: str) -> CubeEstimate | None: ...
    def plan_push(self, rover_id: int, color: str) -> PushPlan | None: ...
    def plan_nav(self, rover_id: int, goal: Pose) -> Path | None: ...
    def commit_path(self, rover_id: int, path: Path, extra: list[np.ndarray]) -> bool: ...
    def commit_region(self, rover_id: int, polygons: list[np.ndarray]) -> bool: ...
    def shrink_region(self, rover_id: int, remaining: Path, extra: list[np.ndarray]) -> None: ...
    def manipulation_region(self, rover_id: int, start: Pose, heading: float, travel: float) -> list[np.ndarray]: ...
    def path_still_clear(self, rover_id: int, path: Path, from_seg: int) -> bool: ...
    def retreat_clear(self, rover_id: int, pose: Pose, dist: float, target: str | None) -> bool: ...
    def fresh_cube(self, color: str, since: float) -> tuple[np.ndarray, int, float] | None: ...
    def is_delivered(self, color: str, since: float, t: float) -> bool: ...
    def on_delivered(self, rover_id: int, color: str, t: float) -> None: ...
    def release_task(self, rover_id: int, color: str, failed: bool, t: float) -> None: ...
    def set_cube_orientation(self, color: str, alpha: float | None, std: float) -> None: ...
    def set_cube_pushed(self, color: str, flag: bool) -> None: ...
    def log(self, rover_id: int, event: str, **kw) -> None: ...


@dataclass
class Task:
    color: str
    plan: PushPlan
    t_start: float = 0.0
    leg: int = 0
    budget_used: float = 0.0
    delivery_attempts: int = 0


@dataclass
class AgentStats:
    replans: int = 0
    recoveries: int = 0
    capture_failures: int = 0
    nav_failures: int = 0
    estops: int = 0
    waits_s: float = 0.0
    push_cross_track: list[float] = field(default_factory=list)


def wrap4(theta: float) -> float:
    """Orientation modulo 90 deg in [-pi/4, pi/4)."""
    return (theta + math.pi / 4) % (math.pi / 2) - math.pi / 4


class RoverAgent:
    def __init__(self, rover_id: int, cfg: Config, params: FsmParams | None = None):
        self.id = rover_id
        self.cfg = cfg
        self.p = params or FsmParams()
        self.state = S.IDLE
        self.t_enter = 0.0
        self.task: Task | None = None
        self.engaged = False
        self.path: Path | None = None
        self.seg_i = 0
        self.nav_t0 = 0.0
        self.resume: S | None = None
        self.after_retreat = "idle"
        self.retreat_start: Pose | None = None
        self.retreat_blocked_since: float | None = None
        self.push_line: tuple[np.ndarray, np.ndarray] | None = None
        self.push_budget = 10.0
        self.last_seen_push = 0.0
        self.yield_goal: Pose | None = None
        self.yield_hold_until = 0.0
        self.align_started = False
        self.last_progress_t = 0.0
        self._progress_xy: np.ndarray | None = None
        self.stats = AgentStats()
        self.follow = SegmentFollower(cfg)
        self.align = AlignController(cfg)
        self.capture = CaptureController(cfg)
        self.pusher = PushController(cfg)
        self.retreat = RetreatController(cfg)
        self._last_t = 0.0

    # ------------------------------------------------------------------ helpers
    def _go(self, s: S, t: float, ctx: AgentContext, why: str = "") -> None:
        if s != self.state:
            ctx.log(self.id, "state", frm=self.state.value, to=s.value, why=why)
        self.state = s
        self.t_enter = t

    def dwell(self, t: float) -> float:
        return t - self.t_enter

    @property
    def leg(self):
        return self.task.plan.legs[self.task.leg] if self.task else None

    @property
    def retreat_distance(self) -> float:
        return self.cfg.rover.paddle_reach + self.p.retreat_extra

    def priority_state(self) -> str:
        """State name for reservation priority: an engaged rover is always treated as PUSH."""
        return "PUSH" if self.engaged else self.state.value

    def assign(self, task: Task, t: float, ctx: AgentContext) -> None:
        task.t_start = t
        self.task = task
        ctx.log(self.id, "assign", color=task.color, legs=len(task.plan.legs), cost=round(task.plan.cost_s, 1))
        self._start_nav(t, ctx)

    def _start_nav(self, t: float, ctx: AgentContext) -> None:
        self.path = None
        self._go(S.NAVIGATE, t, ctx)

    def _charge(self, w: float, t: float, ctx: AgentContext, why: str) -> bool:
        """Charge the task failure budget. Returns True if the task was released (budget exhausted)."""
        if not self.task:
            return False
        self.task.budget_used += w
        ctx.log(self.id, "budget", used=self.task.budget_used, why=why)
        if self.task.budget_used >= self.p.task_budget or t - self.task.t_start > self.p.task_time_budget_s:
            if self.engaged:
                return False                     # must retreat first; the release happens after the retreat
            ctx.release_task(self.id, self.task.color, failed=True, t=t)
            self.task = None
            self.path = None
            self._go(S.IDLE, t, ctx, "task budget exhausted: " + why)
            return True
        return False

    def _fail_task(self, t: float, ctx: AgentContext, why: str, weight: float = 1.0) -> None:
        """Abort current attempt: retreat first if the channel may hold the cube, then replan."""
        self.stats.recoveries += 1
        ctx.log(self.id, "task_fail", why=why, state=self.state.value)
        if self.engaged:
            if self.task:
                self.task.budget_used += weight
            self._begin_retreat(t, ctx, "replan")
            return
        if not self._charge(weight, t, ctx, why):
            self._replan(t, ctx)

    def _replan(self, t: float, ctx: AgentContext) -> None:
        if not self.task:
            self._go(S.IDLE, t, ctx)
            return
        if self.task.budget_used >= self.p.task_budget:
            ctx.release_task(self.id, self.task.color, failed=True, t=t)
            self.task = None
            self._go(S.IDLE, t, ctx, "task budget exhausted")
            return
        self.stats.replans += 1
        plan = ctx.plan_push(self.id, self.task.color)
        if plan is None:
            ctx.release_task(self.id, self.task.color, failed=True, t=t)
            self.task = None
            self._go(S.IDLE, t, ctx, "no push plan")
            return
        self.task.plan, self.task.leg = plan, 0
        self._start_nav(t, ctx)

    def _begin_retreat(self, t: float, ctx: AgentContext, after: str) -> None:
        self.after_retreat = after
        self.retreat_start = None
        self.retreat_blocked_since = None
        if self.task:
            ctx.set_cube_pushed(self.task.color, False)
        self.retreat.reset(self.retreat_distance)
        self._go(S.RETREAT, t, ctx, after)

    def _progress(self, t: float, est: RoverEstimate) -> None:
        xy = est.pose.xy()
        if self._progress_xy is None or np.linalg.norm(xy - self._progress_xy) > 15.0:
            self._progress_xy = xy
            self.last_progress_t = t

    # ------------------------------------------------------------------ external interrupts
    def estop(self, t: float, ctx: AgentContext) -> None:
        if self.state in INTERRUPTS or self.state in PASSIVE:
            return
        self.resume = self.state
        self.stats.estops += 1
        self._go(S.EMERGENCY_STOP, t, ctx)

    def clear_estop(self, t: float, ctx: AgentContext) -> None:
        if self.state == S.EMERGENCY_STOP:
            self._resume(t, ctx)

    def order_yield(self, goal: Pose, hold_s: float, t: float, ctx: AgentContext) -> bool:
        if self.engaged or self.state not in (S.IDLE, S.DONE, S.WAIT, S.NAVIGATE):
            return False
        self.resume = S.NAVIGATE if self.task else S.IDLE
        self.yield_goal = goal
        self.yield_hold_until = t + hold_s
        self.path = None
        self._go(S.YIELD, t, ctx, "ordered")
        return True

    # ------------------------------------------------------------------ main step
    def step(self, t: float, est: RoverEstimate, safe: bool, ctx: AgentContext) -> tuple[float, float]:
        dt = max(1e-3, t - self._last_t)
        self._last_t = t
        self._progress(t, est)
        if not safe and self.state not in (S.RELOCALIZE,) and self.state not in PASSIVE:
            if self.state not in INTERRUPTS:
                self.resume = self.state
            self._go(S.RELOCALIZE, t, ctx, "estimate unsafe: " + ",".join(getattr(ctx, "lost_reasons", {}).get(self.id, [])))
        v, w = getattr(self, "_s_" + self.state.value.lower())(t, est, safe, ctx)
        if self.state == S.WAIT:
            self.stats.waits_s += dt
        if self.engaged and w != 0.0 and self.state not in (S.CAPTURE, S.PUSH):
            # hard invariant: no free rotation with a cube in the channel; a straight retreat may only apply
            # bounded heading-hold corrections (unequal wheels), never a turn.
            lim = self.p.retreat_omega_max if self.state == S.RETREAT else 0.0
            w = max(-lim, min(lim, w))
        return v, w

    # ------------------------------------------------------------------ passive / interrupt states
    def _s_idle(self, t, est, safe, ctx):
        return 0.0, 0.0

    _s_done = _s_idle

    def _s_emergency_stop(self, t, est, safe, ctx):
        return 0.0, 0.0

    def _s_relocalize(self, t, est, safe, ctx):
        if not safe:
            self.t_enter = t
            return 0.0, 0.0
        if self.dwell(t) >= self.p.relocalize_good_s:
            self._resume(t, ctx)
        return 0.0, 0.0

    def _resume(self, t, ctx):
        r, self.resume = self.resume, None
        if self.engaged:
            # Never resume blind manipulation: re-verify contact (region is still committed), or keep retreating.
            if r == S.RETREAT:
                self._go(S.RETREAT, t, ctx, "resume")
            elif r in (S.VERIFY_DELIVERY, S.MEASURE):
                self._go(r, t, ctx, "resume")
            else:
                self._go(S.VERIFY_CAPTURE, t, ctx, "resume")
            return
        if r == S.CONFIRM:
            self._go(S.CONFIRM, t, ctx, "resume")
        elif r == S.YIELD and self.yield_goal is not None:
            self.path = None
            self._go(S.YIELD, t, ctx, "resume")
        elif self.task:
            self._start_nav(t, ctx)                  # re-plan navigation from where we actually are
        else:
            self._go(S.IDLE, t, ctx, "resume")

    def _s_wait(self, t, est, safe, ctx):
        # Back off while blocked by the other rover (each retry is a full planner query): 0.5, 0.75, ... <= 2 s.
        retry = min(2.0, self.p.wait_retry_s * (1.0 + 0.5 * getattr(self, "blocked_streak", 0)))
        if self.dwell(t) >= retry:
            r, self.resume = self.resume, None
            if self.engaged:
                self._go(r if r in (S.RETREAT, S.VERIFY_CAPTURE) else S.VERIFY_CAPTURE, t, ctx, "retry")
            elif r == S.YIELD and self.yield_goal is not None:
                self.path = None
                self._go(S.YIELD, t, ctx, "retry")
            elif self.task:
                self._start_nav(t, ctx)
            else:
                self._go(S.IDLE, t, ctx)
        return 0.0, 0.0

    def _wait(self, t, ctx, resume: S, why: str):
        self.resume = resume
        self._go(S.WAIT, t, ctx, why)
        return 0.0, 0.0

    # ------------------------------------------------------------------ navigation (also used by YIELD)
    def _navigate_to(self, t, est, ctx, goal: Pose, extra: list[np.ndarray]):
        """-> (v, w, status) status in {'moving','arrived','blocked','noplan','deviated','timeout'}."""
        if self.path is None:
            path = ctx.plan_nav(self.id, goal)
            if path is None:
                return 0.0, 0.0, ("blocked" if getattr(ctx, "nav_blocked", {}).get(self.id) else "noplan")
            if not ctx.commit_path(self.id, path, extra):
                return 0.0, 0.0, "blocked"
            self.path, self.seg_i, self.nav_t0 = path, 0, t
            self.follow.reset(self.path.segments[0])
            self._revalidate_t = t
        if self.seg_i >= len(self.path.segments):
            return 0.0, 0.0, "arrived"
        if t - self.nav_t0 > self.p.nav_timeout_factor * self.path.cost_s + 8.0:
            return 0.0, 0.0, "timeout"
        if t - self._revalidate_t > 0.5:             # the world may have changed (cubes moved)
            self._revalidate_t = t
            if not ctx.path_still_clear(self.id, self.path, self.seg_i):
                return 0.0, 0.0, "deviated"
        seg = self.path.segments[self.seg_i]
        v, w, done = self.follow.step(est, t)
        if seg.kind == SegKind.STRAIGHT:
            _, cross, head = self.follow.tracking_error()
            if abs(cross) > self.p.nav_track_cross_tol or abs(head) > self.p.nav_track_heading_tol:
                ctx.log(self.id, "nav_deviation", cross=round(cross, 1), head=round(head, 3))
                return 0.0, 0.0, "deviated"
        if done:
            self.seg_i += 1
            if self.seg_i >= len(self.path.segments):
                return 0.0, 0.0, "arrived"
            self.follow.reset(self.path.segments[self.seg_i])
            # release the part of the reservation already traversed (shrink only: always safe)
            ctx.shrink_region(self.id, Path(self.path.segments[self.seg_i:], 0.0), extra)
        return v, w, "moving"

    def _leg_region(self, ctx: AgentContext) -> list[np.ndarray]:
        leg = self.leg
        pre = leg.prepush
        travel = math.hypot(leg.cube_end[0] - pre.x, leg.cube_end[1] - pre.y) - self.cfg.contact_distance
        return ctx.manipulation_region(self.id, pre, leg.heading, max(0.0, travel))

    _NAV_WEIGHT = {"noplan": 1.0, "blocked": 0.15, "deviated": 0.5, "timeout": 1.0}

    def _s_navigate(self, t, est, safe, ctx):
        cube = ctx.cube(self.task.color)
        leg = self.leg
        if cube is None:
            return self._wait(t, ctx, S.NAVIGATE, "cube unknown")
        if math.hypot(cube.x - leg.cube_start[0], cube.y - leg.cube_start[1]) > self.p.cube_moved_tol:
            ctx.log(self.id, "cube_moved_before_capture", color=self.task.color)
            self.path = None
            if not self._charge(0.5, t, ctx, "cube moved"):
                self._replan(t, ctx)
            return 0.0, 0.0
        v, w, st = self._navigate_to(t, est, ctx, leg.prepush, self._leg_region(ctx))
        if st == "moving":
            self.blocked_streak = 0
        if st == "arrived":
            self.path = None
            self.align_started = False
            self._go(S.ALIGN, t, ctx)
        elif st != "moving":
            self.path = None
            self.blocked_streak = getattr(self, "blocked_streak", 0) + 1 if st == "blocked" else 0
            if st == "noplan":
                self.stats.nav_failures += 1
            if not self._charge(self._NAV_WEIGHT[st], t, ctx, "nav " + st):
                return self._wait(t, ctx, S.NAVIGATE, "nav " + st)
        return v, w

    def _s_yield(self, t, est, safe, ctx):
        if self.yield_goal is None:
            self._go(S.IDLE, t, ctx)
            return 0.0, 0.0
        v, w, st = self._navigate_to(t, est, ctx, self.yield_goal, [])
        if st == "arrived":
            if t < self.yield_hold_until:
                return 0.0, 0.0                      # hold the parking spot until released / timeout
            self.path, self.yield_goal = None, None
            if self.task:
                return self._wait(t, ctx, S.NAVIGATE, "yield done")
            self._go(S.IDLE, t, ctx, "yield done")
        elif st != "moving":
            self.path = None
            if t > self.yield_hold_until:
                self.yield_goal = None
                self._go(S.IDLE if not self.task else S.WAIT, t, ctx, "yield " + st)
                self.resume = S.NAVIGATE if self.task else None
                return 0.0, 0.0
            return self._wait(t, ctx, S.YIELD, "yield " + st)
        return v, w

    def release_yield(self, t: float) -> None:
        self.yield_hold_until = min(self.yield_hold_until, t)

    # ------------------------------------------------------------------ manipulation
    def _s_align(self, t, est, safe, ctx):
        if self.dwell(t) > self.p.align_timeout_s:
            self._fail_task(t, ctx, "align timeout")
            return 0.0, 0.0
        obs = ctx.fresh_cube(self.task.color, t - 0.6)
        cube = ctx.cube(self.task.color)
        if obs is None or cube is None:
            return 0.0, 0.0                          # wait for a fresh look at the (static) cube
        p0 = obs[0]
        p1 = np.array(self.leg.cube_end, float)
        d = p1 - p0
        phi = math.atan2(d[1], d[0]) if np.linalg.norm(d) > 10.0 else self.leg.heading
        if abs(angle_diff(phi, self.leg.heading)) > math.radians(10):
            if not self._charge(0.5, t, ctx, "cube moved at align"):
                self._replan(t, ctx)
            return 0.0, 0.0
        lat = -math.sin(phi) * (est.pose.x - p0[0]) + math.cos(phi) * (est.pose.y - p0[1])
        if abs(lat) > self.p.align_reposition_tol:
            if not self._charge(0.5, t, ctx, "align lateral too large"):
                self._start_nav(t, ctx)
            return 0.0, 0.0
        if est.quality != TrackQuality.GOOD:
            return 0.0, 0.0                          # never start a capture on degraded localization
        if not self.align_started:
            self.align.reset((float(p0[0]), float(p0[1])), phi)
            self.align_started = True
            self.align_phi = phi
        v, w, done, status = self.align.step(est, t)
        if done:
            fw = np.array([math.cos(phi), math.sin(phi)])
            dist = float(np.dot(p0 - est.pose.xy(), fw))
            travel = dist - self.cfg.contact_distance + self.p.capture_overdrive
            start_on_line = p0 - dist * fw           # point of the cube line abeam of the rover: along = 0 here
            region = ctx.manipulation_region(self.id, est.pose.rotated_to(phi), phi,
                                             max(0.0, float(np.linalg.norm(p1 - est.pose.xy())) - self.cfg.contact_distance))
            if not ctx.commit_region(self.id, region):
                return self._wait(t, ctx, S.NAVIGATE, "manipulation region denied")
            self.capture.reset((float(start_on_line[0]), float(start_on_line[1])), phi, max(0.0, travel))
            self.push_line = (p0, p1)
            self.engaged = True
            ctx.set_cube_pushed(self.task.color, True)
            self._go(S.CAPTURE, t, ctx)
        return v, w

    def _s_capture(self, t, est, safe, ctx):
        if self.dwell(t) > self.p.capture_timeout_s:
            self._fail_task(t, ctx, "capture timeout")
            return 0.0, 0.0
        v, w, done = self.capture.step(est, t)
        if done:
            self._go(S.VERIFY_CAPTURE, t, ctx)
            return 0.0, 0.0
        return v, w

    def _contact(self, est: RoverEstimate, since: float, ctx: AgentContext):
        """Relative cube geometry from distinct fresh captures since `since`: (depth, lateral, n) or None.
        depth = cube-centre distance ahead of the front plate."""
        obs = ctx.fresh_cube(self.task.color, since)
        if obs is None or obs[1] < self.p.min_frames:
            return None
        xl, yl = est.pose.to_local(float(obs[0][0]), float(obs[0][1]))
        return xl - self.cfg.rover.x_front_plate, yl, obs[1]

    def _fits_channel(self, depth: float, lat: float) -> bool:
        """Is there an orientation a for which a cube touching the plate has projected half-depth h(a) = depth
        and fits laterally: |lat| + h(a) <= inner half width?  (square: same h for depth and width)"""
        c, g = self.cfg.cube, self.cfg.rover
        if not (c.half - self.p.depth_tol <= depth <= c.half_diag + self.p.depth_tol):
            return False
        h = min(max(depth, c.half), c.half_diag)
        return abs(lat) + h <= g.inner_half_width + self.p.contact_tol

    def _infer_orientation(self, depth: float, heading: float, ctx, pushed_mm: float = 0.0) -> None:
        """Orientation from contact depth (cube-centre distance ahead of the plate): 30 mm flush vs 42.4 mm at 45 deg.
        Measured depths of flush cubes in closed loop span ~26-37 mm (unknown marker offset biases them), so:
        depth <= 36 after a real push (>= 60 mm of flat-plate pushing, which squares the cube) -> strong flush
        evidence (std 6 deg); depth <= 39 -> flush but less certain (std 10 deg); otherwise unknown."""
        half = self.cfg.cube.half
        if depth <= half + 6.0 and pushed_mm >= 60.0:
            ctx.set_cube_orientation(self.task.color, wrap4(heading), math.radians(6.0))
        elif depth <= half + 9.0:
            ctx.set_cube_orientation(self.task.color, wrap4(heading), self.p.flush_alpha_std)
        else:
            ctx.set_cube_orientation(self.task.color, None, math.pi / 4)

    def _s_verify_capture(self, t, est, safe, ctx):
        if self.dwell(t) < self.p.verify_s:
            return 0.0, 0.0
        if self.dwell(t) > self.p.verify_timeout_s + 3.0:      # e.g. corridor re-commit never granted
            self._fail_task(t, ctx, "verify capture timeout")
            return 0.0, 0.0
        m = self._contact(est, self.t_enter + self.p.verify_s * 0.5, ctx)
        if m is None:
            if self.dwell(t) > self.p.verify_timeout_s:
                self.stats.capture_failures += 1
                self._fail_task(t, ctx, "cube not observed at capture")
            return 0.0, 0.0
        depth, lat, n = m
        ok = self._fits_channel(depth, lat)
        ctx.log(self.id, "verify_capture", ok=ok, depth=round(depth, 1), lat=round(lat, 1), n=n)
        if not ok:
            self.stats.capture_failures += 1
            self._fail_task(t, ctx, "capture verification failed")
            return 0.0, 0.0
        self._infer_orientation(depth, est.pose.theta, ctx)
        cube = ctx.cube(self.task.color)
        p0 = cube.xy()
        p1 = np.array(self.leg.cube_end, float)
        if np.linalg.norm(p1 - p0) < 5.0:
            self._go(S.MEASURE, t, ctx, "leg already complete")
            return 0.0, 0.0
        phi_new = math.atan2(*(p1 - p0)[::-1])
        dphi = abs(angle_diff(phi_new, est.pose.theta))
        if dphi > math.radians(self.p.push_line_max_change_deg):
            self._fail_task(t, ctx, "push line changed too much")
            return 0.0, 0.0
        if dphi > math.radians(self.p.push_line_recommit_deg):
            # The actual push line differs from the reserved one: re-validate the corridor along the NEW line.
            region = ctx.manipulation_region(self.id, est.pose, phi_new,
                                             float(np.linalg.norm(p1 - est.pose.xy())))
            if not ctx.commit_region(self.id, region):
                return 0.0, 0.0                      # hold (engaged) until granted; timeout above
        self.push_line = (p0, p1)
        self.pusher.reset((float(p0[0]), float(p0[1])), (float(p1[0]), float(p1[1])))
        self.push_budget = self.p.push_timeout_factor * (np.linalg.norm(p1 - p0) / self.cfg.limits.v_push) + 6.0
        self.last_seen_push = t
        self.push_start_xy = est.pose.xy()
        ctx.set_cube_pushed(self.task.color, True)
        self._go(S.PUSH, t, ctx)
        return 0.0, 0.0

    def _s_push(self, t, est, safe, ctx):
        if self.dwell(t) > self.push_budget:
            self._fail_task(t, ctx, "push timeout")
            return 0.0, 0.0
        obs = ctx.fresh_cube(self.task.color, max(self.last_seen_push, t - 0.25) + 1e-6)
        if obs is not None:
            self.last_seen_push = obs[2]             # newest distinct capture time, not "now"
        blind = t - self.last_seen_push
        if blind > self.p.push_blind_fail_s:
            self._fail_task(t, ctx, "cube unseen during push")
            return 0.0, 0.0
        if blind > self.p.push_blind_hold_s:
            return 0.0, 0.0                          # hold still until the cube is seen again
        cube_xy = (float(obs[0][0]), float(obs[0][1])) if obs else None
        age = (t - obs[2]) if obs else 99.0
        v, w, done, status = self.pusher.step(est, cube_xy, age, t)
        self.stats.push_cross_track.append(abs(status.cross_mm))
        if status.lost_cube:
            if obs is not None:
                xl, yl = est.pose.to_local(float(obs[0][0]), float(obs[0][1]))
                ctx.log(self.id, "push_lost", depth=round(xl - self.cfg.rover.x_front_plate, 1), lat=round(yl, 1),
                        age=round(age, 3), n=obs[1], cross=round(status.cross_mm, 1))
            self.stats.capture_failures += 1
            self._fail_task(t, ctx, "cube lost during push")
            return 0.0, 0.0
        if done:
            self._go(S.MEASURE, t, ctx)
            return 0.0, 0.0
        return v, w

    def _s_measure(self, t, est, safe, ctx):
        """End of a leg: measure contact geometry (orientation belief), then continue."""
        if self.dwell(t) < self.p.verify_s:
            return 0.0, 0.0
        m = self._contact(est, self.t_enter + self.p.verify_s * 0.5, ctx)
        if m is None and self.dwell(t) < self.p.verify_timeout_s:
            return 0.0, 0.0
        if m is not None and self._fits_channel(m[0], m[1]):
            pushed = float(np.linalg.norm(est.pose.xy() - self.push_start_xy)) if getattr(self, "push_start_xy", None) is not None else 0.0
            self._infer_orientation(m[0], est.pose.theta, ctx, pushed)
        else:
            ctx.set_cube_orientation(self.task.color, None, math.pi / 4)
        final = self.task.leg == len(self.task.plan.legs) - 1
        if final:
            # Verify delivery only with a clear view: while the rover presses the cube into a corner its body
            # partly covers the cube and the published position is noisy/biased (closed-loop finding: 27 mm
            # spread).  Retreat straight, then CONFIRM; a miss is handled by replanning from the observed state.
            self._begin_retreat(t, ctx, "delivered")
        else:
            self._begin_retreat(t, ctx, "next_leg")
        return 0.0, 0.0

    def _s_verify_delivery(self, t, est, safe, ctx):
        if self.dwell(t) < self.p.verify_s:
            return 0.0, 0.0
        if ctx.is_delivered(self.task.color, self.t_enter, t):
            self._begin_retreat(t, ctx, "delivered")
            return 0.0, 0.0
        if self.dwell(t) < self.p.verify_timeout_s:
            return 0.0, 0.0
        self.task.delivery_attempts += 1
        ctx.log(self.id, "delivery_check_failed", **getattr(ctx, "last_confirm_diag", {}))
        obs = ctx.fresh_cube(self.task.color, t - 0.6)
        if obs is not None and self.task.delivery_attempts <= self.p.delivery_retries:
            dp = np.array(self.task.plan.delivery_point) - obs[0]
            ahead = float(np.dot(dp, est.pose.forward()))
            lateral = abs(float(np.dot(dp, [-math.sin(est.pose.theta), math.cos(est.pose.theta)])))
            xl, yl = est.pose.to_local(float(obs[0][0]), float(obs[0][1]))
            if 2.0 < ahead < 40.0 and lateral < 8.0 and self._fits_channel(xl - self.cfg.rover.x_front_plate, yl):
                # corrective push straight ahead (inside the committed corridor), no new line
                target = obs[0] + ahead * est.pose.forward()
                self.pusher.reset((float(obs[0][0]), float(obs[0][1])), (float(target[0]), float(target[1])))
                self.push_budget = 8.0
                self.last_seen_push = t
                self._go(S.PUSH, t, ctx, "corrective push")
                return 0.0, 0.0
        self._fail_task(t, ctx, "delivery not confirmed")
        return 0.0, 0.0

    def _s_retreat(self, t, est, safe, ctx):
        if self.retreat_start is None:
            self.retreat_start = est.pose
        back = -float(np.dot(est.pose.xy() - self.retreat_start.xy(), self.retreat_start.forward()))
        remaining = self.retreat_distance - back
        if remaining <= 3.0:
            return self._after_retreat(t, ctx)
        target = self.task.color if self.task else None
        if not ctx.retreat_clear(self.id, est.pose, remaining, target):
            self.retreat_blocked_since = self.retreat_blocked_since or t
            if t - self.retreat_blocked_since > self.p.retreat_blocked_fail_s:
                ctx.log(self.id, "retreat_blocked")
                self.retreat_blocked_since = t
                if self.task:
                    self.task.budget_used += 1.0
            return 0.0, 0.0                          # hold (still engaged); supervisor may move the other rover
        self.retreat_blocked_since = None
        if self.dwell(t) > self.p.retreat_timeout_s:
            ctx.log(self.id, "retreat_slow", back=round(back, 1))
            self.t_enter = t                          # keep trying; engagement is NOT cleared by a timeout
        v, w, done = self.retreat.step(est, t)
        return (v, w) if not done else (-self.cfg.limits.v_capture, 0.0)

    def _after_retreat(self, t, ctx):
        self.engaged = False
        a = self.after_retreat
        if a == "delivered" and self.task:
            self._go(S.CONFIRM, t, ctx)
        elif a in ("next_leg", "replan") and self.task:
            if self.task.budget_used >= self.p.task_budget or t - self.task.t_start > self.p.task_time_budget_s:
                ctx.release_task(self.id, self.task.color, failed=True, t=t)
                self.task = None
                self._go(S.IDLE, t, ctx, "task budget exhausted")
            else:
                if a == "next_leg":
                    self.task.budget_used = max(0.0, self.task.budget_used - 1.0)   # verified progress
                self._replan(t, ctx)
        else:
            self._go(S.IDLE, t, ctx)
        return 0.0, 0.0

    def _s_confirm(self, t, est, safe, ctx):
        """Post-retreat confirmation that the cube is still (completely) in the depot."""
        if self.dwell(t) < self.p.verify_s:
            return 0.0, 0.0
        if ctx.is_delivered(self.task.color, self.t_enter, t, nominal_alpha=wrap4(self.task.plan.legs[-1].heading)):
            if getattr(ctx, "last_confirm_diag", {}).get("marginal"):
                ctx.log(self.id, "delivery_marginal", **ctx.last_confirm_diag)
            ctx.on_delivered(self.id, self.task.color, t)
            ctx.release_task(self.id, self.task.color, failed=False, t=t)
            self.task = None
            self._go(S.IDLE, t, ctx, "delivered")
        elif self.dwell(t) > self.p.verify_timeout_s:
            ctx.log(self.id, "delivery_check_failed", **getattr(ctx, "last_confirm_diag", {}))
            # The cube has not been touched since the flush push ended: keep that orientation belief so a short
            # corrective push can be planned (the tracker may have reset it on occlusion noise).
            ctx.set_cube_orientation(self.task.color, wrap4(self.task.plan.legs[-1].heading), self.p.flush_alpha_std)
            if not self._charge(1.0, t, ctx, "delivery not confirmed after retreat"):
                self._replan(t, ctx)
        return 0.0, 0.0
