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

ROTATION_WITH_CUBE_OMEGA_RAD_S = 0.2
CHANNEL_PLATE_TOLERANCE_MM = 5.0
MIN_CAUSAL_PUSH_SPEED_MM_S = 5.0


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


def _bbox_overlap(A: list[Pt], B: list[Pt]) -> bool:
    """Cheap conservative broad phase for the small convex polygons here."""
    aminx = amaxx = A[0][0]
    aminy = amaxy = A[0][1]
    for x, y in A[1:]:
        if x < aminx: aminx = x
        if x > amaxx: amaxx = x
        if y < aminy: aminy = y
        if y > amaxy: amaxy = y
    bminx = bmaxx = B[0][0]
    bminy = bmaxy = B[0][1]
    for x, y in B[1:]:
        if x < bminx: bminx = x
        if x > bmaxx: bmaxx = x
        if y < bminy: bminy = y
        if y > bmaxy: bmaxy = y
    return not (amaxx < bminx or bmaxx < aminx or amaxy < bminy or bmaxy < aminy)


def _bbox(P: list[Pt]) -> tuple[float, float, float, float]:
    min_x = max_x = P[0][0]
    min_y = max_y = P[0][1]
    for x, y in P[1:]:
        if x < min_x: min_x = x
        if x > max_x: max_x = x
        if y < min_y: min_y = y
        if y > max_y: max_y = y
    return min_x, max_x, min_y, max_y


def _bbox_overlap_cached(a: tuple[float, float, float, float],
                        b: tuple[float, float, float, float]) -> bool:
    return not (a[1] < b[0] or b[1] < a[0] or a[3] < b[2] or b[3] < a[2])


def _inside_rect(P: list[Pt], x0: float, x1: float, y0: float, y1: float) -> bool:
    return all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in P)


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


def _support_point(P: list[Pt], n: Pt) -> Pt:
    """Midpoint of the support face nearest the opposite body along ``n``."""
    values = [p[0] * n[0] + p[1] * n[1] for p in P]
    near = min(values)
    tol = max(1e-8, (max(values) - near) * 1e-7)
    return _mean([p for p, value in zip(P, values) if value <= near + tol])


def _contact_point(A: list[Pt], B: list[Pt], n: Pt) -> Pt:
    """Return B's support point inside the tangential overlap with A.

    Using the midpoint of the entire support face erases the moment arm when a
    thin paddle rail clips a cube corner.  Restricting the face to A's
    tangential projection retains the actual corner/edge contact while keeping
    a stable point for a flat plate.
    """
    p = _support_point(B, n)
    tx, ty = -n[1], n[0]
    face_values = [v[0] * tx + v[1] * ty for v in B
                   if abs(v[0] * n[0] + v[1] * n[1] -
                          min(q[0] * n[0] + q[1] * n[1] for q in B)) < 1e-6]
    amin, amax = _proj(A, tx, ty)
    if not face_values:
        return p
    lo = max(min(face_values), amin)
    hi = min(max(face_values), amax)
    if lo > hi:
        return p
    target = 0.5 * (lo + hi)
    current = p[0] * tx + p[1] * ty
    return (p[0] + (target - current) * tx, p[1] + (target - current) * ty)


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
    (None, None, None) if disjoint. `contact_point` is on B's support face nearest
    A; this is what lets a corner strike differ from a flush push.
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
        # Keep the first axis on an exact tie.  Polygon vertex order is stable,
        # so this makes the contact graph independent of dict insertion order.
        if d < best_depth - 1e-10:
            acx, acy = _mean(A)
            bcx, bcy = _mean(B)
            if (bcx - acx) * nx + (bcy - acy) * ny < 0:
                nx, ny = -nx, -ny
            best_depth, best_n = d, (nx, ny)
    if best_n is None:
        return None, None, None
    # The contact point belongs to the driven body's support face, rather than
    # the centroid of its penetration volume.  The latter can be far inside a
    # cube and loses the moment arm for a paddle-tip strike.
    p = _contact_point(A, B, best_n)
    return best_depth, best_n, p


# ---------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------


# The experiment recorded distance, not speed: its fixed interval duration is
# unknown.  These are therefore dimensionless distance/speed fractions and the
# simulator keeps the existing 180 mm/s scale for the 100% point.  The zero
# knot is the commanded deadband endpoint; the measured knots are 30/50/70/100
# percent throttle.  R11 has no 30% measurement, so its low-throttle knot uses
# the measured R10 shape until that experiment is repeated.
THROTTLE_KNOTS: tuple[float, ...] = (0.0, 0.30, 0.50, 0.70, 1.0)
R10_LEFT_CURVE: tuple[float, ...] = (
    0.0, 39.6 / 57.5, 49.5 / 57.5, 54.0 / 57.5, 1.0,
)
R10_RIGHT_CURVE: tuple[float, ...] = (
    0.0, 40.0 / 58.0, 50.0 / 58.0, 54.2 / 58.0, 1.0,
)
R11_LEFT_CURVE: tuple[float, ...] = (
    0.0, R10_LEFT_CURVE[1], 45.33 / 54.0, 50.0 / 54.0, 1.0,
)
R11_RIGHT_CURVE: tuple[float, ...] = (
    0.0, R10_RIGHT_CURVE[1], 50.0 / 58.0, 54.0 / 58.0, 1.0,
)

# The planner emits wheel speeds, not throttles.  It has no knowledge of the
# true rover curves, so inversion uses this balanced nominal curve only.  An
# average of the two measured R10 wheels avoids baking a tiny left/right bias
# into the command interface while retaining the measured saturation shape.
NOMINAL_CURVE: tuple[float, ...] = tuple(
    (left + right) / 2.0 for left, right in zip(R10_LEFT_CURVE, R10_RIGHT_CURVE)
)


def _piecewise_linear(x: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    """Evaluate a monotone piecewise-linear curve, clamped at its endpoints."""
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            span = xs[i] - xs[i - 1]
            return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / span
    return ys[-1]


def _inverse_piecewise_linear(y: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    """Invert a monotone piecewise-linear curve, clamped at its endpoints."""
    if y <= ys[0]:
        return xs[0]
    for i in range(1, len(ys)):
        if y <= ys[i]:
            span = ys[i] - ys[i - 1]
            if span <= 1e-12:
                return xs[i]
            return xs[i - 1] + (xs[i] - xs[i - 1]) * (y - ys[i - 1]) / span
    return xs[-1]


_MOTOR_TABLE_SIZE = 1000


def _lookup_table(curve: tuple[float, ...], scale: float = 1.0) -> tuple[float, ...]:
    """Compile a normalized piecewise curve for constant-time hot-path lookup."""
    return tuple(scale * _piecewise_linear(i / _MOTOR_TABLE_SIZE, THROTTLE_KNOTS, curve)
                 for i in range(_MOTOR_TABLE_SIZE + 1))


def _inverse_lookup_table(curve: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(_inverse_piecewise_linear(i / _MOTOR_TABLE_SIZE, THROTTLE_KNOTS, curve)
                 for i in range(_MOTOR_TABLE_SIZE + 1))


_NOMINAL_INVERSE_TABLE = _inverse_lookup_table(NOMINAL_CURVE)


def _table_value(x: float, table: tuple[float, ...]) -> float:
    """Linearly interpolate a normalized lookup table value in [0, 1]."""
    scaled = min(1.0, max(0.0, x)) * _MOTOR_TABLE_SIZE
    index = min(_MOTOR_TABLE_SIZE - 1, int(scaled))
    return table[index] + (table[index + 1] - table[index]) * (scaled - index)



@dataclass(frozen=True)
class MotorParams:
    """Per-rover open-loop model with the measured saturating motor curves.

    ``curve_left`` and ``curve_right`` are normalized speed at the throttle
    knots above.  ``full_speed_*_factor`` carries the measured wheel-to-wheel
    full-throttle difference; multiplying by ``max_wheel_speed_mm_s`` keeps
    the absolute simulator scale at the former 180 mm/s nominal maximum.
    Commands are still wheel speeds in mm/s.  The physics uses the inverse of
    :data:`NOMINAL_CURVE`, never these true curves, before applying them.
    """

    # Retained as a compatibility multiplier for hand-authored fixtures.  The
    # scenario generator no longer randomizes these legacy gains: measured
    # asymmetry lives in the curves and full-speed factors below.
    gain_left: float = 1.0
    gain_right: float = 1.0
    # A direct MotorParams() fixture is the balanced nominal model.  Scenarios
    # always replace these with the measured R10 or R11 model explicitly.
    curve_left: tuple[float, ...] = NOMINAL_CURVE
    curve_right: tuple[float, ...] = NOMINAL_CURVE
    full_speed_left_factor: float = 1.0
    full_speed_right_factor: float = 1.0
    deadband_mm_s: float = 4.0    # command magnitude below this produces zero wheel speed
    tau_s: float = 0.08           # first-order response time constant
    max_wheel_speed_mm_s: float = 180.0
    speed_noise_std: float = 0.03  # multiplicative, resampled every physics step
    latency_s: float = 0.05       # command -> applied delay
    max_accel_mm_s2: float = 300.0  # planner's conservative measured assumption
    # USER-REPORTED: pushing a cube costs about 20 mm over a 500 mm run.
    pushing_speed_loss: float = 0.04

    def __post_init__(self) -> None:
        # Private caches are deliberately not dataclass fields: scenario JSON
        # must contain physical parameters only.
        object.__setattr__(self, "_true_left_table",
                           _lookup_table(self.curve_left, self.full_speed_left_factor * self.gain_left))
        object.__setattr__(self, "_true_right_table",
                           _lookup_table(self.curve_right, self.full_speed_right_factor * self.gain_right))

    def throttle_for_command(self, command_mm_s: float) -> float:
        """Map a signed wheel-speed command (mm/s) to nominal throttle."""
        if abs(command_mm_s) < self.deadband_mm_s:
            return 0.0
        fraction = min(1.0, abs(command_mm_s) / max(self.max_wheel_speed_mm_s, 1e-9))
        # Preserve the measured knots exactly (important for calibration
        # assertions); all other commands use the compiled interpolation.
        if abs(fraction - NOMINAL_CURVE[1]) < 1e-12:
            throttle = THROTTLE_KNOTS[1]
        elif abs(fraction - NOMINAL_CURVE[2]) < 1e-12:
            throttle = THROTTLE_KNOTS[2]
        elif abs(fraction - NOMINAL_CURVE[3]) < 1e-12:
            throttle = THROTTLE_KNOTS[3]
        else:
            throttle = _table_value(fraction, _NOMINAL_INVERSE_TABLE)
        return math.copysign(throttle, command_mm_s)

    def speed_at_throttle(self, throttle: float, wheel: str) -> float:
        """Return true signed wheel speed (mm/s) for ``wheel`` at throttle."""
        if wheel not in ("left", "right"):
            raise ValueError(f"wheel must be 'left' or 'right', got {wheel!r}")
        sign = -1.0 if throttle < 0.0 else 1.0
        magnitude = min(1.0, abs(throttle))
        table = self._true_left_table if wheel == "left" else self._true_right_table
        return sign * self.max_wheel_speed_mm_s * _table_value(magnitude, table)


@dataclass(frozen=True)
class ContactParams:
    """Contact-resolution tuning. `c_ls_frac` is randomisable per scenario (spec: ~0.38)."""

    c_ls_frac: float = 0.38       # characteristic length / cube side (uniform-pressure disc approx)
    friction_mu: float = 0.5      # pusher-cube friction coefficient
    iterations: int = 4           # Gauss-Seidel sweeps per step for multi-point convergence
    floor_perturb_std: float = 0.0  # extra tangential jitter fraction (floor non-uniformity)
    max_correction_mm: float = 20.0
    # A cap limits pathological initial-state impulses.  The solver keeps
    # iterating after a cap and reports activation; it never treats the capped
    # correction as resolved penetration.


@dataclass
class SimRover:
    id: int
    x: float
    y: float
    theta: float
    motor: MotorParams
    wl: float = 0.0                # lag-filtered TARGET wheel speeds (pre-noise), mm/s
    wr: float = 0.0
    wl_realized: float = 0.0       # REALIZED wheel speeds this step (post multiplicative noise), mm/s --
    wr_realized: float = 0.0       # use these, not wl/wr, for anything judging actual motion (A07).
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
    # Cumulative cube-centre displacement (mm) while each rover's front plate
    # contacts its assigned cube and actual forward speed exceeds 5 mm/s.
    # Paddle touches, stationary contact and reverse motion never earn credit.
    engaged_displacement: dict = field(default_factory=dict)


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
        self._direct_contact_active: set[tuple[int, str]] = set()
        self._plate_contact_active: set[tuple[int, str]] = set()
        self._rotation_cube_active: set[tuple[int, str]] = set()
        self._rover_load: dict[int, int] = {r.id: 0 for r in rovers}

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
        if dt <= 0.0:
            return
        end = self.t + dt
        # 20 ms / 3.6 mm at the default maximum wheel speed prevents a rover
        # from crossing a 60 mm cube between collision queries.  The distance
        # bound below also handles custom high-speed motor parameters.
        max_h = 0.02
        while self.t < end - 1e-12:
            for r in self.rovers.values():
                self._apply_pending(r)
            next_due = end
            for r in self.rovers.values():
                if r.pending and r.pending[0][0] > self.t + 1e-12:
                    next_due = min(next_due, r.pending[0][0])
            speed_bound = max(
                [1.0]
                + [abs(v) for r in self.rovers.values() for v in
                   (r.wl, r.wr, r.cmd_left, r.cmd_right)]
            )
            segment_h = min(max_h, 4.0 / speed_bound)
            segment_end = min(end, self.t + segment_h, next_due)
            h = segment_end - self.t
            if h <= 1e-12:
                # Only possible at an exact command boundary; the next loop
                # applies it before selecting a positive integration interval.
                self.t = segment_end
                continue
            prev = {rid: (r.x, r.y, r.theta) for rid, r in self.rovers.items()}
            for r in self.rovers.values():
                self._advance_rover(r, h)
            self.t = segment_end
            self._resolve_rover_rover(prev)
            self._record_rotation_with_cube()
            cube_before = {color: (c.x, c.y) for color, c in self.cubes.items()}
            touched = self._resolve_contacts()
            self._update_contact_records(touched, cube_before)
            self._check_board_bounds()

    # --------------------------------------------------------- rover motion --
    def _apply_pending(self, r: SimRover) -> None:
        while r.pending and r.pending[0][0] <= self.t + 1e-12:
            _, cmd = r.pending.pop(0)
            r.cmd_left, r.cmd_right = cmd.v_left, cmd.v_right

    def _advance_rover(self, r: SimRover, dt: float) -> None:
        m = r.motor
        self._apply_pending(r)

        # Wheel commands are the public simulator interface.  Convert them to
        # nominal throttle first, then apply the true per-wheel curve.  This is
        # deliberately open-loop: the strategy cannot compensate for the
        # measured R10/R11 differences because it never sees the true throttle.
        tl = m.speed_at_throttle(m.throttle_for_command(r.cmd_left), "left")
        tr = m.speed_at_throttle(m.throttle_for_command(r.cmd_right), "right")
        # The response time constant shapes the target approach, while the
        # acceleration limit is a hard physical bound shared with planning.
        a = min(1.0, dt / max(m.tau_s, 1e-6))
        load = self._rover_load.get(r.id, 0)
        mu_scale = max(0.2, min(1.0, self.contact.friction_mu / 0.5))
        load_factor = 1.0 - min(0.9, m.pushing_speed_loss * load / mu_scale)
        tl *= load_factor
        tr *= load_factor
        dl = (tl - r.wl) * a
        dr = (tr - r.wr) * a
        max_delta = max(0.0, m.max_accel_mm_s2) * dt
        r.wl += max(-max_delta, min(max_delta, dl))
        r.wr += max(-max_delta, min(max_delta, dr))
        nl = 1.0 + self.rng.normal(0.0, m.speed_noise_std)
        nr = 1.0 + self.rng.normal(0.0, m.speed_noise_std)
        wl_eff, wr_eff = r.wl * nl, r.wr * nr
        r.wl_realized, r.wr_realized = wl_eff, wr_eff
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
                pair = (ids[i], ids[j])
                Ea = self.footprint.envelope(a.x, a.y, a.theta)
                Eb = self.footprint.envelope(b.x, b.y, b.theta)
                if (Ea[:, 0].max() < Eb[:, 0].min() or Eb[:, 0].max() < Ea[:, 0].min()
                        or Ea[:, 1].max() < Eb[:, 1].min() or Eb[:, 1].max() < Ea[:, 1].min()):
                    if pair in self._collision_active and S.distance(Ea, Eb) < 1.0:
                        new_active.add(pair)
                    continue
                if not S.overlap(Ea, Eb):
                    # Rejected opposing commands can leave a sub-millimetre
                    # gap for one integration step.  Keep one collision
                    # episode active through that numerical release band;
                    # otherwise acceleration-limited braking is reported as
                    # repeated collisions every other step.
                    if pair in self._collision_active and S.distance(Ea, Eb) < 1.0:
                        new_active.add(pair)
                    continue
                new_active.add(pair)
                if pair not in self._collision_active:
                    self.events.append(Event("collision", self.t, {"rovers": pair}))
                    self.counts["collision"] += 1
                a.x, a.y, a.theta = prev[a.id]
                b.x, b.y, b.theta = prev[b.id]
                a.wl = a.wr = a.wl_realized = a.wr_realized = 0.0
                b.wl = b.wr = b.wl_realized = b.wr_realized = 0.0
        self._collision_active = new_active

    # --------------------------------------------------------- cube contact --
    def _apply_twist(self, cube: SimCube, p: Pt, n: Pt, depth: float,
                     *, separate: bool = False) -> None:
        if self.contact.max_correction_mm > 0.0 and depth > self.contact.max_correction_mm:
            self.counts["contact_correction_clamped"] += 1
            self.events.append(Event("contact_correction_clamped", self.t,
                                     {"cube": cube.color, "depth_mm": depth,
                                      "applied_mm": self.contact.max_correction_mm}))
            depth = self.contact.max_correction_mm
        if separate:
            # Shapes.overlap treats touching as contact.  Leave a microscopic
            # geometric gap after a resolved SAT contact so a departing rover
            # cannot remain classified as overlapping forever.
            depth += 1e-6
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

    def _resolve_contacts(self) -> set[tuple[int, str]]:
        """Relax the complete rover/cube contact graph to a small residual.

        A rover is kinematic and therefore remains fixed during a substep.  A
        cube touched by a rover is the driver for the next cube in a chain;
        rover attribution is propagated with that chain for safety metrics.
        Rechecking rover contacts after cube propagation prevents a chain from
        being pushed through a stationary rover.
        """
        side = self.cfg.cube.side
        rover_parts: dict[int, list[tuple[list[Pt], tuple[float, float, float, float]]]] = {}
        for rid in sorted(self.rovers):
            r = self.rovers[rid]
            rover_parts[rid] = []
            for part in self.footprint.parts(r.x, r.y, r.theta):
                points = _pts(part)
                rover_parts[rid].append((points, _bbox(points)))

        colors = sorted(self.cubes)
        sources: dict[str, set[int]] = {color: set() for color in colors}
        direct_contacts: set[tuple[int, str]] = set()
        plate_contacts: set[tuple[int, str]] = set()
        # Four default Gauss-Seidel iterations are enough for ordinary
        # contact chains; retain a bounded extra pass budget for rotated
        # corners without paying 32 sweeps on every sustained contact.
        max_sweeps = max(8, self.contact.iterations * 4)
        for _ in range(max_sweeps):
            max_depth = 0.0
            cube_polys = {
                color: _square_pts(self.cubes[color].x, self.cubes[color].y, side,
                                   self.cubes[color].alpha)
                for color in colors if self.cubes[color].in_play
            }
            cube_boxes = {color: _bbox(poly) for color, poly in cube_polys.items()}
            for rid in sorted(rover_parts):
                parts = rover_parts[rid]
                for color in colors:
                    cube = self.cubes[color]
                    if not cube.in_play:
                        continue
                    cube_poly = cube_polys[color]
                    cube_box = cube_boxes[color]
                    for part_index, (part, part_box) in enumerate(parts):
                        if not _bbox_overlap_cached(part_box, cube_box):
                            continue
                        depth, n, p = _sat_contact(part, cube_poly)
                        if depth is None or depth < 1e-9:
                            continue
                        actual_v = 0.5 * (self.rovers[rid].wl_realized + self.rovers[rid].wr_realized)
                        along = math.cos(self.rovers[rid].theta) * n[0] + math.sin(self.rovers[rid].theta) * n[1]
                        # A rover braking/reversing away from a cube should
                        # not drag it by the residual motor response.  A
                        # stationary rover remains an immovable obstacle.
                        if actual_v * along < 0.0:
                            continue
                        max_depth = max(max_depth, depth)
                        sources[color].add(rid)
                        direct_contacts.add((rid, color))
                        if part_index == 0 and along > 0.5:
                            plate_contacts.add((rid, color))
                        self._apply_twist(cube, p, n, depth, separate=True)
                        cube_poly = _square_pts(cube.x, cube.y, side, cube.alpha)
                        cube_box = _bbox(cube_poly)
                        cube_polys[color] = cube_poly
                        cube_boxes[color] = cube_box

            # Stable colour order is intentional: insertion order must not
            # decide which member of a cube chain gets left penetrating.
            for i, ca_color in enumerate(colors):
                ca = self.cubes[ca_color]
                if not ca.in_play:
                    continue
                for cb_color in colors[i + 1:]:
                    cb = self.cubes[cb_color]
                    if not cb.in_play:
                        continue
                    A = _square_pts(ca.x, ca.y, side, ca.alpha)
                    B = _square_pts(cb.x, cb.y, side, cb.alpha)
                    if not _bbox_overlap(A, B):
                        continue
                    depth, n, p = _sat_contact(A, B)
                    if depth is None or depth < 1e-9:
                        continue
                    max_depth = max(max_depth, depth)
                    a_src, b_src = sources[ca_color], sources[cb_color]
                    if a_src and not b_src:
                        self._apply_twist(cb, p, n, depth, separate=True)
                        b_src.update(a_src)
                    elif b_src and not a_src:
                        reverse_depth, reverse_n, reverse_p = _sat_contact(B, A)
                        self._apply_twist(ca, reverse_p, reverse_n, reverse_depth,
                                          separate=True)
                        a_src.update(b_src)
                    elif a_src and b_src and not (b_src - a_src):
                        # Both bodies are in the same pushing component.
                        self._apply_twist(cb, p, n, depth, separate=True)
                    elif a_src and b_src:
                        # A chain cannot push its driven cube through a rover
                        # that is holding it from the opposite side.  Leave
                        # that edge constrained; the next rover contact sweep
                        # moves the chain away from the blocker.
                        continue
                    else:
                        # No unique driver (or both are independently driven):
                        # separate symmetrically, with no artificial spin.
                        ca.x -= 0.5 * depth * n[0]
                        ca.y -= 0.5 * depth * n[1]
                        cb.x += 0.5 * depth * n[0]
                        cb.y += 0.5 * depth * n[1]
                        if a_src or b_src:
                            union = a_src | b_src
                            a_src.update(union)
                            b_src.update(union)

            if max_depth < 1e-4:
                break

        self._rover_load = {
            rid: sum(rid in source for source in sources.values())
            for rid in self.rovers
        }
        self._direct_contact_active = direct_contacts
        self._plate_contact_active = plate_contacts
        return {(rid, color) for color, rids in sources.items() for rid in rids}

    # Kept as a narrow compatibility alias for diagnostics that used the old
    # private helper; all production stepping uses the complete solver above.
    def _resolve_rover_cube(self) -> set[tuple[int, str]]:
        return self._resolve_contacts()

    def _update_contact_records(self, touched: set[tuple[int, str]],
                                cube_before: dict[str, tuple[float, float]]) -> None:
        for rid, color in touched:
            self.cubes[color].touched_by[rid] = self.t
            pair = (rid, color)
            if pair not in self._contact_active:
                if self.targets.get(rid) != color:
                    self.events.append(Event("non_target_contact", self.t, {"rover": rid, "cube": color}))
                    self.counts["non_target_contact"] += 1
        for rid, color in self._plate_contact_active:
            if self.targets.get(rid) != color:
                continue
            actual_v = 0.5 * (self.rovers[rid].wl_realized + self.rovers[rid].wr_realized)
            if actual_v <= MIN_CAUSAL_PUSH_SPEED_MM_S:
                continue
            before_x, before_y = cube_before[color]
            cube = self.cubes[color]
            displacement = math.hypot(cube.x - before_x, cube.y - before_y)
            if displacement > 0.0:
                cube.engaged_displacement[rid] = (
                    cube.engaged_displacement.get(rid, 0.0) + displacement
                )
        self._contact_active = touched

    def _cube_in_channel(self, rover: SimRover, cube: SimCube) -> bool:
        """Return whether a cube is between the paddles and within 5 mm of the plate.

        Coordinates are evaluated in the rover frame.  The lateral test uses all
        cube vertices, so a rotated cube only qualifies when its complete footprint
        is inside the physical channel.  ``plate_gap`` is the distance from the
        plate to the cube's nearest point along +x; a small negative value allows
        the contact solver's finite penetration tolerance.
        """
        c, s = math.cos(rover.theta), math.sin(rover.theta)
        local = []
        for px, py in _square_pts(cube.x, cube.y, self.cfg.cube.side, cube.alpha):
            dx, dy = px - rover.x, py - rover.y
            local.append((c * dx + s * dy, -s * dx + c * dy))
        if max(abs(y) for _, y in local) > self.cfg.rover.inner_half_width + 1e-6:
            return False
        plate_gap = min(x for x, _ in local) - self.cfg.rover.x_front_plate
        return (
            -CHANNEL_PLATE_TOLERANCE_MM - 1e-6 <= plate_gap
            <= CHANNEL_PLATE_TOLERANCE_MM + 1e-6
            and max(x for x, _ in local) >= self.cfg.rover.x_front_plate
        )

    def _record_rotation_with_cube(self) -> None:
        """Record one edge-triggered event per rover/cube rotation episode.

        This is diagnostic scoring data only.  It deliberately does not stop or
        fail a run; the judge can inspect how often a rover turned while carrying
        a cube in its channel.
        """
        active: set[tuple[int, str]] = set()
        for rid, rover in self.rovers.items():
            omega = (rover.wr_realized - rover.wl_realized) / self.track
            if abs(omega) <= ROTATION_WITH_CUBE_OMEGA_RAD_S:
                continue
            for color, cube in self.cubes.items():
                if not cube.in_play or not self._cube_in_channel(rover, cube):
                    continue
                pair = (rid, color)
                active.add(pair)
                if pair not in self._rotation_cube_active:
                    self.events.append(Event("rotation_with_cube", self.t,
                                             {"rover": rid, "cube": color, "omega": omega}))
                    self.counts["rotations_with_cube"] += 1
        self._rotation_cube_active = active

    # ------------------------------------------------------------- bounds --
    def _check_board_bounds(self) -> None:
        b = self.cfg.board
        W, H = b.width, b.height
        over = b.overhang_allowance_mm
        phys = b.physical_margin_mm
        for rid, r in self.rovers.items():
            env = _pts(self.footprint.envelope(r.x, r.y, r.theta))
            inside = _inside_rect(env, -over, W + over, -over, H + over)
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
                self._cube_out[color] = True
                # The field boundary is irreversible for cubes.  Do not use
                # the larger physical-board margin as a recoverable play area.
                c.in_play = False
