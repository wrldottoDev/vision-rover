"""Ground-truth 2D physics for the Monte Carlo simulator.

Internal frame: mm, rad, s (see config.py). Rover pose = rotation centre (axle midpoint).

Contact model (the core of this module): the rover is kinematically driven (heavy,
motor-driven) and pushes cubes, which are quasi-static sliding squares. Penetration
between a rover PART polygon (body rect or one of the two thin paddle rails -- the
channel between them is empty, see geometry/footprint.py `parts()`) and a cube is
resolved by moving the CUBE with an ellipsoidal-limit-surface quasi-static pushing
law (Mason-style): for the deepest contact point p, contact normal n (unit, pointing
from the pusher into the cube) and cube centre c, with r = p - c and characteristic
length c_ls,

    k = depth / (1 + (r x n)^2 / c_ls^2)
    dc = k * n                       (cube centre translation)
    dtheta = k * (r x n) / c_ls^2    (cube rotation)

is chosen so the contact point's normal displacement exactly cancels `depth` given
the *current* geometry (see derivation in the module docstring history / PR notes).
A flat-plate contact yields a near-central contact point (mostly translation); a
corner/tip strike yields a large offset (mostly rotation) -- this is what lets a
paddle TIP induce spin instead of only "containing" the cube (a known limitation of
the prior research simulator, `04_Banco_Pruebas/simulador.py`, whose paddles never
rotate the cube). Multiple simultaneous contacts (e.g. a flat plate = one contact
polygon per part, but body + 2 paddles = up to 3) are relaxed with a few Gauss-Seidel
style iterations per step so they converge to a consistent resolved state.

Cube-cube contact reuses the exact same law: whichever cube was touched by a rover
THIS step is the "driver" (does not move under this specific interaction) and moves
the other cube (the "driven" one) away from it.
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field

import numpy as np

from ..config import Config
from ..frames import wrap
from ..geometry import shapes as S
from ..geometry.footprint import Footprint
from ..world import STOP, WheelCommand

# ---------------------------------------------------------------------------
# plain-python polygon helpers (deliberately NOT numpy: at 2-8 vertices per
# polygon, numpy's per-call overhead dominates; the >=20x-real-time budget
# needs these on the hot path to be cheap plain-float loops).
# ---------------------------------------------------------------------------

Pt = tuple[float, float]


def _pts(P) -> list[Pt]:
    return [(float(p[0]), float(p[1])) for p in P]


def _square_pts(cx: float, cy: float, side: float, alpha: float) -> list[Pt]:
    h = side / 2.0
    c, s = math.cos(alpha), math.sin(alpha)
    local = ((-h, -h), (h, -h), (h, h), (-h, h))
    return [(cx + c * lx - s * ly, cy + s * lx + c * ly) for lx, ly in local]


def _edges(P: list[Pt]) -> list[Pt]:
    n = len(P)
    return [(P[(i + 1) % n][0] - P[i][0], P[(i + 1) % n][1] - P[i][1]) for i in range(n)]


def _proj(P: list[Pt], nx: float, ny: float) -> tuple[float, float]:
    lo = hi = P[0][0] * nx + P[0][1] * ny
    for px, py in P[1:]:
        d = px * nx + py * ny
        if d < lo:
            lo = d
        if d > hi:
            hi = d
    return lo, hi


def _mean(P: list[Pt]) -> Pt:
    n = len(P)
    return (sum(p[0] for p in P) / n, sum(p[1] for p in P) / n)


def _poly_area(P: list[Pt]) -> float:
    n = len(P)
    if n < 3:
        return 0.0
    a = 0.0
    for i in range(n):
        x1, y1 = P[i]
        x2, y2 = P[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) * 0.5


def _poly_centroid(P: list[Pt]) -> Pt:
    n = len(P)
    if n < 3:
        return _mean(P)
    a = 0.0
    cx = 0.0
    cy = 0.0
    for i in range(n):
        x1, y1 = P[i]
        x2, y2 = P[(i + 1) % n]
        cross = x1 * y2 - x2 * y1
        a += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    a *= 0.5
    if abs(a) < 1e-9:
        return _mean(P)
    return (cx / (6.0 * a), cy / (6.0 * a))


def _clip_convex(subject: list[Pt], window: list[Pt]) -> list[Pt]:
    """Sutherland-Hodgman intersection of two CCW convex polygons."""
    out = list(subject)
    wn = len(window)
    for i in range(wn):
        ax, ay = window[i]
        bx, by = window[(i + 1) % wn]
        ex, ey = bx - ax, by - ay
        inp, out = out, []
        if not inp:
            break
        m = len(inp)
        for j in range(m):
            px, py = inp[j]
            qx, qy = inp[(j + 1) % m]
            sp = ex * (py - ay) - ey * (px - ax)
            sq = ex * (qy - ay) - ey * (qx - ax)
            if sp >= 0:
                out.append((px, py))
            if (sp > 0) != (sq > 0):
                denom = sp - sq
                if abs(denom) > 1e-12:
                    t = sp / denom
                    out.append((px + t * (qx - px), py + t * (qy - py)))
    return out


def _sat_contact(A: list[Pt], B: list[Pt]):
    """SAT between CCW convex polygons A (driver, stationary this call) and B (driven).

    Returns (depth, normal, contact_point) with `normal` a unit vector pointing from
    A towards B (the direction B must move to resolve penetration), or
    (None, None, None) if disjoint. `contact_point` is the centroid of the overlap
    region (or, degenerately, the deepest vertex of B) -- this is what lets a corner
    strike differ from a flush push.
    """
    best_depth = math.inf
    best_n = None
    for ex, ey in _edges(A) + _edges(B):
        length = math.hypot(ex, ey)
        if length < 1e-12:
            continue
        nx, ny = -ey / length, ex / length
        amin, amax = _proj(A, nx, ny)
        bmin, bmax = _proj(B, nx, ny)
        if amax < bmin or bmax < amin:
            return None, None, None
        d = min(amax - bmin, bmax - amin)
        if d < best_depth:
            acx, acy = _mean(A)
            bcx, bcy = _mean(B)
            if (bcx - acx) * nx + (bcy - acy) * ny < 0:
                nx, ny = -nx, -ny
            best_depth, best_n = d, (nx, ny)
    if best_n is None:
        return None, None, None
    inter = _clip_convex(B, A)
    if len(inter) >= 3:
        p = _poly_centroid(inter)
    else:
        p = min(B, key=lambda v: v[0] * best_n[0] + v[1] * best_n[1])
    return best_depth, best_n, p


# ---------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MotorParams:
    """Per-rover open-loop motor model. Units mm/s, s. All ASSUMED pending calibration."""

    gain_left: float = 1.0        # wheel asymmetry, TYPICAL range 0.9-1.1
    gain_right: float = 1.0
    deadband_mm_s: float = 4.0    # commands below this (after gain) produce zero wheel speed
    tau_s: float = 0.08           # first-order response time constant
    max_wheel_speed_mm_s: float = 180.0
    speed_noise_std: float = 0.03  # multiplicative, resampled every physics step
    latency_s: float = 0.05       # command -> applied delay


@dataclass(frozen=True)
class ContactParams:
    """Contact-resolution tuning. `c_ls_frac` is randomisable per scenario (spec: ~0.38)."""

    c_ls_frac: float = 0.38       # characteristic length / cube side (uniform-pressure disc approx)
    friction_mu: float = 0.5      # pusher-cube friction coefficient (documents the assumption the
                                  # ellipsoid law already encodes; not separately enforced -- ponytail:
                                  # add an explicit friction-cone clamp on tangential k if slip matters)
    iterations: int = 4           # Gauss-Seidel sweeps per step for multi-point convergence
    floor_perturb_std: float = 0.0  # extra tangential jitter fraction (floor non-uniformity)
    max_correction_mm: float = 20.0
    # ponytail: SAT-MTV depth on a containment axis (e.g. a paddle rail fully spanned
    # by a cube along one axis) can legitimately be large -- that's the correct MTV,
    # not a bug -- but such deep interpenetration should never arise from gradual
    # per-step motion at a sane dt. Clamping guards the "no explosion" requirement
    # against adversarial initial conditions / bad hand-built states; raise this if a
    # scenario needs genuinely large one-step corrections.


@dataclass
class SimRover:
    id: int
    x: float
    y: float
    theta: float
    motor: MotorParams
    wl: float = 0.0                # actual (lagged) wheel speeds, mm/s
    wr: float = 0.0
    cmd_left: float = 0.0          # last applied command target (post-latency)
    cmd_right: float = 0.0
    pending: list = field(default_factory=list)   # [(t_apply, WheelCommand)]


@dataclass
class SimCube:
    color: str
    x: float
    y: float
    alpha: float                    # true orientation, full unwrapped angle, radians
    in_play: bool = True
    touched_by: dict = field(default_factory=dict)   # rover_id -> last-touch sim time


@dataclass(frozen=True)
class Event:
    kind: str
    t: float
    data: dict


class PhysicsWorld:
    def __init__(
        self,
        cfg: Config,
        rovers: list[SimRover],
        cubes: list[SimCube],
        contact: ContactParams,
        rng: np.random.Generator,
    ):
        self.cfg = cfg
        self.rovers: dict[int, SimRover] = {r.id: r for r in rovers}
        self.cubes: dict[str, SimCube] = {c.color: c for c in cubes}
        self.contact = contact
        self.rng = rng
        self.footprint = Footprint(cfg.rover)
        self.track = cfg.rover.track_width
        self.t = 0.0
        self.events: list[Event] = []
        self.counts: Counter = Counter()
        self.targets: dict[int, str | None] = {r.id: None for r in rovers}
        self._rover_out = {r.id: False for r in rovers}
        self._rover_fell = set()
        self._cube_out = {c.color: False for c in cubes}
        self._collision_active: set[tuple[int, int]] = set()
        self._contact_active: set[tuple[int, str]] = set()

    # ------------------------------------------------------------ commands --
    def set_command(self, rover_id: int, cmd: WheelCommand, t: float) -> None:
        r = self.rovers[rover_id]
        r.pending.append((t + r.motor.latency_s, cmd))
        r.pending.sort(key=lambda e: e[0])

    def set_target(self, rover_id: int, color: str | None) -> None:
        self.targets[rover_id] = color

    # -------------------------------------------------------------- access --
    def rover_pose(self, rover_id: int) -> tuple[float, float, float]:
        r = self.rovers[rover_id]
        return r.x, r.y, r.theta

    def cube_pose(self, color: str) -> tuple[float, float, float]:
        c = self.cubes[color]
        return c.x, c.y, c.alpha

    # ---------------------------------------------------------------- step --
    def step(self, dt: float) -> None:
        self.t += dt
        prev = {rid: (r.x, r.y, r.theta) for rid, r in self.rovers.items()}
        for r in self.rovers.values():
            self._advance_rover(r, dt)
        self._resolve_rover_rover(prev)
        touched = self._resolve_rover_cube()
        self._resolve_cube_cube(touched)
        self._update_contact_records(touched)
        self._check_board_bounds()

    # --------------------------------------------------------- rover motion --
    def _advance_rover(self, r: SimRover, dt: float) -> None:
        m = r.motor
        while r.pending and r.pending[0][0] <= self.t:
            _, cmd = r.pending.pop(0)
            r.cmd_left, r.cmd_right = cmd.v_left, cmd.v_right

        def _target(cmd_v: float, gain: float) -> float:
            v = gain * cmd_v
            if abs(v) < m.deadband_mm_s:
                v = 0.0
            return max(-m.max_wheel_speed_mm_s, min(m.max_wheel_speed_mm_s, v))

        tl = _target(r.cmd_left, m.gain_left)
        tr = _target(r.cmd_right, m.gain_right)
        a = min(1.0, dt / max(m.tau_s, 1e-6))
        r.wl += (tl - r.wl) * a
        r.wr += (tr - r.wr) * a
        nl = 1.0 + self.rng.normal(0.0, m.speed_noise_std)
        nr = 1.0 + self.rng.normal(0.0, m.speed_noise_std)
        wl_eff, wr_eff = r.wl * nl, r.wr * nr
        v = (wl_eff + wr_eff) / 2.0
        w = (wr_eff - wl_eff) / self.track
        r.x += v * math.cos(r.theta) * dt
        r.y += v * math.sin(r.theta) * dt
        r.theta = wrap(r.theta + w * dt)

    def _resolve_rover_rover(self, prev: dict[int, tuple[float, float, float]]) -> None:
        ids = list(self.rovers.keys())
        new_active: set[tuple[int, int]] = set()
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = self.rovers[ids[i]], self.rovers[ids[j]]
                Ea = self.footprint.envelope(a.x, a.y, a.theta)
                Eb = self.footprint.envelope(b.x, b.y, b.theta)
                if not S.overlap(Ea, Eb):
                    continue
                pair = (ids[i], ids[j])
                new_active.add(pair)
                if pair not in self._collision_active:
                    self.events.append(Event("collision", self.t, {"rovers": pair}))
                    self.counts["collision"] += 1
                a.x, a.y, a.theta = prev[a.id]
                b.x, b.y, b.theta = prev[b.id]
                a.wl = a.wr = 0.0
                b.wl = b.wr = 0.0
        self._collision_active = new_active

    # --------------------------------------------------------- cube contact --
    def _apply_twist(self, cube: SimCube, p: Pt, n: Pt, depth: float) -> None:
        depth = min(depth, self.contact.max_correction_mm)
        rx, ry = p[0] - cube.x, p[1] - cube.y
        cross_rn = rx * n[1] - ry * n[0]
        c_ls = self.contact.c_ls_frac * self.cfg.cube.side
        k = depth / (1.0 + (cross_rn * cross_rn) / (c_ls * c_ls))
        if self.contact.floor_perturb_std > 0.0:
            jitter = self.rng.normal(0.0, self.contact.floor_perturb_std * max(abs(k), 1e-6))
            cube.x += -n[1] * jitter
            cube.y += n[0] * jitter
        cube.x += k * n[0]
        cube.y += k * n[1]
        cube.alpha += k * cross_rn / (c_ls * c_ls)

    def _resolve_rover_cube(self) -> set[tuple[int, str]]:
        side = self.cfg.cube.side
        rover_parts: dict[int, list[list[Pt]]] = {}
        for rid, r in self.rovers.items():
            rover_parts[rid] = [_pts(part) for part in self.footprint.parts(r.x, r.y, r.theta)]

        touched: set[tuple[int, str]] = set()
        for _ in range(self.contact.iterations):
            for color, cube in self.cubes.items():
                if not cube.in_play:
                    continue
                for rid, parts in rover_parts.items():
                    for part in parts:
                        cube_poly = _square_pts(cube.x, cube.y, side, cube.alpha)
                        depth, n, p = _sat_contact(part, cube_poly)
                        if depth is None or depth < 1e-9:
                            continue
                        touched.add((rid, color))
                        self._apply_twist(cube, p, n, depth)
        return touched

    def _resolve_cube_cube(self, touched: set[tuple[int, str]]) -> None:
        side = self.cfg.cube.side
        driven_by_rover = {color for (_, color) in touched}
        colors = list(self.cubes.keys())
        for _ in range(self.contact.iterations):
            for i in range(len(colors)):
                for j in range(i + 1, len(colors)):
                    ca, cb = self.cubes[colors[i]], self.cubes[colors[j]]
                    if not ca.in_play or not cb.in_play:
                        continue
                    a_drv = colors[i] in driven_by_rover
                    b_drv = colors[j] in driven_by_rover
                    if a_drv and not b_drv:
                        driver, driven = ca, cb
                    elif b_drv and not a_drv:
                        driver, driven = cb, ca
                    else:
                        # ponytail: neither (or both) cube touched by a rover this step
                        # -- no clear pusher, so just split the overlap symmetrically with
                        # no induced spin. Upgrade to per-pair priority if this matters.
                        A = _square_pts(ca.x, ca.y, side, ca.alpha)
                        B = _square_pts(cb.x, cb.y, side, cb.alpha)
                        depth, n, _p = _sat_contact(A, B)
                        if depth is None or depth < 1e-9:
                            continue
                        ca.x -= 0.5 * depth * n[0]
                        ca.y -= 0.5 * depth * n[1]
                        cb.x += 0.5 * depth * n[0]
                        cb.y += 0.5 * depth * n[1]
                        continue
                    A = _square_pts(driver.x, driver.y, side, driver.alpha)
                    B = _square_pts(driven.x, driven.y, side, driven.alpha)
                    depth, n, p = _sat_contact(A, B)
                    if depth is None or depth < 1e-9:
                        continue
                    self._apply_twist(driven, p, n, depth)

    def _update_contact_records(self, touched: set[tuple[int, str]]) -> None:
        for rid, color in touched:
            self.cubes[color].touched_by[rid] = self.t
            pair = (rid, color)
            if pair not in self._contact_active:
                if self.targets.get(rid) != color:
                    self.events.append(Event("non_target_contact", self.t, {"rover": rid, "cube": color}))
                    self.counts["non_target_contact"] += 1
        self._contact_active = touched

    # ------------------------------------------------------------- bounds --
    def _check_board_bounds(self) -> None:
        b = self.cfg.board
        W, H = b.width, b.height
        over = b.overhang_allowance_mm
        phys = b.physical_margin_mm
        for rid, r in self.rovers.items():
            env = _pts(self.footprint.envelope(r.x, r.y, r.theta))
            inside = S.inside_rect(np.array(env), -over, W + over, -over, H + over)
            if not inside and not self._rover_out[rid]:
                self.events.append(Event("rover_exit", self.t, {"rover": rid}))
                self.counts["rover_exit"] += 1
            self._rover_out[rid] = not inside
            fell = not (-phys <= r.x <= W + phys and -phys <= r.y <= H + phys)
            if fell and rid not in self._rover_fell:
                self.events.append(Event("rover_fell", self.t, {"rover": rid}))
                self.counts["rover_fell"] += 1
                self._rover_fell.add(rid)
        for color, c in self.cubes.items():
            if not c.in_play:
                continue
            poly = np.array(_square_pts(c.x, c.y, self.cfg.cube.side, c.alpha))
            inside = S.inside_rect(poly, -over, W + over, -over, H + over)
            if not inside and not self._cube_out[color]:
                self.events.append(Event("cube_exit", self.t, {"cube": color}))
                self.counts["cube_exit"] += 1
            self._cube_out[color] = not inside
