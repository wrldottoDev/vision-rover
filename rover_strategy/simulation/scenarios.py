"""Deterministic scenario generation for the Monte Carlo runner.

A `Scenario` is a JSON-serialisable snapshot: rover start poses + per-rover motor
model, cube start poses, a random depot-corner-per-colour assignment, and the
sensor/contact noise params for that run. `generate(seed, family)` is a pure
function of its inputs (one `np.random.default_rng(seed)` consumed in a fixed
order) -- same seed + family always yields the same scenario.

Depot / start points follow the official convention (CONTRATO.md s3 example:
(40.5,2.5)/(2.5,40.5)/(40.5,40.5) cells on a 43-cell field, start (2.5,2.5)):
2.5 cells in from each edge of the corner, derived from `cfg.board.cols/rows` (not
hard-coded), so a differently-sized board still gets sane points.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, replace

import numpy as np

from ..config import DEFAULT, Config
from ..frames import Grid
from ..geometry import shapes as S
from ..geometry.footprint import Footprint
from ..geometry.zones import DepotZone
from .physics import (
    ContactParams,
    MotorParams,
    PhysicsWorld,
    R10_LEFT_CURVE,
    R10_RIGHT_CURVE,
    R11_LEFT_CURVE,
    R11_RIGHT_CURVE,
    SimCube,
    SimRover,
)
from .sensors import SensorParams, VisionEmulator

FAMILIES = ("start_zone", "arbitrary", "hard", "official_like", "official_like_slow")
# official_like: rovers in the start zone, cubes in the interior band like the official example layout
# (config_simulador.json cubes at internal (520,660),(300,280),(660,340)): [160, 700] mm on both axes.
# official_like_slow: same geometry, with the opt-in user-reported heavy-tailed
# transport latency profile (p95 ~= 470 ms, capped at ~= 1420 ms).
OFFICIAL_BAND = (160.0, 700.0)
COLORS = ("green", "blue", "red")

#: ASSUMED (matches the contract example): every fixed point (start, depots) sits
#: this many cells in from the two board edges that form its corner.
_CORNER_INSET_CELLS = 2.5

#: ASSUMED (interpretation_v0 R7): the two official ArUco ids used by the sim config.
ROVER_IDS = (10, 11)


@dataclass(frozen=True)
class GenParams:
    """Scenario-generation tunables (not physics/sensor params -- just how spread out
    and how noisy a generated scenario is)."""

    min_clearance_mm: float = 8.0        # minimum gap between any two generated bodies
    max_attempts: int = 4000
    edge_band_cells: float = 3.0         # "hard": cube considered near-edge within this many cells
    corner_bias_prob: float = 0.5        # "hard": prob a cube is placed near an edge/depot vs anywhere
    near_other_cube_prob: float = 0.35   # "hard": prob a cube is placed close to an already-placed cube
    rotation_min_deg: float = 12.0       # "hard": keep rotated cubes away from the 0/90 axes
    rotation_max_deg: float = 78.0


GEN = GenParams()


def _depot_point(corner: tuple[int, int], cols: int, rows: int) -> tuple[float, float]:
    ccol, crow = corner
    col = _CORNER_INSET_CELLS if ccol == 0 else cols - _CORNER_INSET_CELLS
    row = _CORNER_INSET_CELLS if crow == 0 else rows - _CORNER_INSET_CELLS
    return (col, row)


@dataclass
class RoverInit:
    id: int
    x: float
    y: float
    theta: float
    motor: MotorParams


@dataclass
class CubeInit:
    color: str
    x: float
    y: float
    alpha: float


@dataclass
class Scenario:
    seed: int
    family: str
    rovers: list[RoverInit]
    cubes: list[CubeInit]
    depots: dict[str, tuple[float, float]]   # official cell units, static
    start: tuple[float, float]               # official cell units, static
    sensor: SensorParams
    contact: ContactParams
    board_cols: int
    board_rows: int
    cell_mm: float

    def to_json(self) -> str:
        return json.dumps(
            {
                "seed": self.seed,
                "family": self.family,
                "rovers": [asdict(r) for r in self.rovers],
                "cubes": [asdict(c) for c in self.cubes],
                "depots": self.depots,
                "start": self.start,
                "sensor": asdict(self.sensor),
                "contact": asdict(self.contact),
                "board_cols": self.board_cols,
                "board_rows": self.board_rows,
                "cell_mm": self.cell_mm,
            }
        )

    @staticmethod
    def from_json(text: str) -> "Scenario":
        d = json.loads(text)
        return Scenario(
            seed=d["seed"],
            family=d["family"],
            rovers=[RoverInit(id=r["id"], x=r["x"], y=r["y"], theta=r["theta"], motor=MotorParams(**r["motor"]))
                    for r in d["rovers"]],
            cubes=[CubeInit(**c) for c in d["cubes"]],
            depots={k: tuple(v) for k, v in d["depots"].items()},
            start=tuple(d["start"]),
            sensor=SensorParams(**d["sensor"]),
            contact=ContactParams(**d["contact"]),
            board_cols=d["board_cols"],
            board_rows=d["board_rows"],
            cell_mm=d["cell_mm"],
        )


# --------------------------------------------------------------------------- #
# validity
# --------------------------------------------------------------------------- #


def _rover_polygon(fp: Footprint, x: float, y: float, theta: float):
    return fp.envelope(x, y, theta)


def _cube_polygon(x: float, y: float, side: float, alpha: float):
    return S.square(x, y, side, alpha)


def validate(scenario: Scenario, cfg: Config = DEFAULT) -> list[str]:
    """Returns a list of violated-invariant descriptions (empty = valid)."""
    problems: list[str] = []
    fp = Footprint(cfg.rover)
    W, H = cfg.board.width, cfg.board.height
    side = cfg.cube.side

    rover_polys = []
    for r in scenario.rovers:
        poly = _rover_polygon(fp, r.x, r.y, r.theta)
        rover_polys.append(poly)
        if not S.inside_rect(poly, 0.0, W, 0.0, H):
            problems.append(f"rover {r.id} envelope outside field")
    for i in range(len(rover_polys)):
        for j in range(i + 1, len(rover_polys)):
            if S.distance(rover_polys[i], rover_polys[j]) < GEN.min_clearance_mm:
                problems.append(f"rovers {scenario.rovers[i].id}/{scenario.rovers[j].id} too close")

    cube_polys = []
    for c in scenario.cubes:
        poly = _cube_polygon(c.x, c.y, side, c.alpha)
        cube_polys.append(poly)
        if not S.inside_rect(poly, 0.0, W, 0.0, H):
            problems.append(f"cube {c.color} outside field")
        for rp, r in zip(rover_polys, scenario.rovers):
            if S.distance(rp, poly) < GEN.min_clearance_mm:
                problems.append(f"rover {r.id} too close to cube {c.color}")
        dcol, drow = scenario.depots[c.color]
        dx, dy = Grid(scenario.board_cols, scenario.board_rows, scenario.cell_mm).to_internal_xy(dcol, drow)
        zone = DepotZone(c.color, dx, dy, cfg.depot.half_size)
        if zone.contains_cube(c.x, c.y, side, c.alpha):
            problems.append(f"cube {c.color} already in its depot")
    for i in range(len(cube_polys)):
        for j in range(i + 1, len(cube_polys)):
            if S.distance(cube_polys[i], cube_polys[j]) < GEN.min_clearance_mm:
                problems.append(f"cubes {scenario.cubes[i].color}/{scenario.cubes[j].color} too close")
    return problems


# --------------------------------------------------------------------------- #
# generation
# --------------------------------------------------------------------------- #


def _jitter_curve(
    rng: np.random.Generator,
    curve: tuple[float, ...],
    std: tuple[float, ...],
    hard: bool,
) -> tuple[float, ...]:
    """Perturb measured knots while retaining a valid saturating curve."""
    width = 1.75 if hard else 1.0
    values = list(curve)
    for i in range(1, len(values) - 1):
        values[i] = min(1.0, max(values[i - 1], values[i] + rng.normal(0.0, std[i] * width)))
    values[0] = 0.0
    values[-1] = 1.0
    return tuple(float(v) for v in values)


def _motor_params(rng: np.random.Generator, rover_id: int, hard: bool) -> MotorParams:
    """Build a measured motor model for rover 10 or 11.

    R11's 50% knot uses the observed repeated-run spread (48/44/44 cm and
    52/48/50 cm).  R10 has no repeated-run sample, so its smaller variation is
    an explicit uncertainty around the measured wheel curves.  The hard family
    widens both measured distributions, but never changes rover identity.
    """
    if rover_id == 10:
        curve_left, curve_right = R10_LEFT_CURVE, R10_RIGHT_CURVE
        curve_std_left = (0.0, 0.008, 0.008, 0.006, 0.0)
        curve_std_right = (0.0, 0.008, 0.008, 0.006, 0.0)
        full_left, full_right = 57.5 / 58.0, 1.0
    elif rover_id == 11:
        curve_left, curve_right = R11_LEFT_CURVE, R11_RIGHT_CURVE
        # Population standard deviations of the repeated 50% normalized
        # distances: (48,44,44)/54 and (52,48,50)/58 respectively.
        curve_std_left = (0.0, 0.010, 0.0349, 0.015, 0.0)
        curve_std_right = (0.0, 0.010, 0.0282, 0.015, 0.0)
        full_left, full_right = 54.0 / 58.0, 1.0
    else:
        raise ValueError(f"measured motor model is only defined for rover 10/11, got {rover_id}")

    curve_left = _jitter_curve(rng, curve_left, curve_std_left, hard)
    curve_right = _jitter_curve(rng, curve_right, curve_std_right, hard)
    tau = rng.uniform(0.06, 0.15) if not hard else rng.uniform(0.08, 0.20)
    latency = rng.uniform(0.02, 0.06) if not hard else rng.uniform(0.03, 0.10)
    deadband = rng.uniform(2.0, 6.0)
    noise = rng.uniform(0.01, 0.04) if not hard else rng.uniform(0.02, 0.07)
    return MotorParams(
        curve_left=curve_left, curve_right=curve_right,
        full_speed_left_factor=full_left, full_speed_right_factor=full_right,
        deadband_mm_s=float(deadband),
        tau_s=float(tau), speed_noise_std=float(noise), latency_s=float(latency),
    )


def _sensor_params(rng: np.random.Generator, hard: bool, family: str = "") -> SensorParams:
    base = SensorParams()
    if family == "official_like_slow":
        return replace(base, latency_profile="realistic_latency")
    if not hard:
        return base
    scale = rng.uniform(1.3, 2.2)
    return SensorParams(
        frame_drop_prob=min(0.3, base.frame_drop_prob * scale),
        rover_loss_prob=min(0.3, base.rover_loss_prob * scale),
        rover_loss_burst_prob=min(0.05, base.rover_loss_burst_prob * scale),
        pos_noise_std_mm=base.pos_noise_std_mm * scale,
        heading_noise_std_rad=base.heading_noise_std_rad * scale,
        outlier_prob=min(0.05, base.outlier_prob * scale),
        cube_pos_noise_std_mm=base.cube_pos_noise_std_mm * scale,
        cube_pos_bias_mm=base.cube_pos_bias_mm * scale,
    )


def _contact_params(rng: np.random.Generator, hard: bool) -> ContactParams:
    c_ls_frac = float(rng.uniform(0.34, 0.42))
    friction_mu = float(rng.uniform(0.3, 0.7))
    floor = float(rng.uniform(0.02, 0.08)) if hard else 0.0
    return ContactParams(c_ls_frac=c_ls_frac, friction_mu=friction_mu, iterations=4, floor_perturb_std=floor)


def _clamp_inside_field(fp: Footprint, x: float, y: float, theta: float, W: float, H: float) -> tuple[float, float]:
    poly = fp.envelope(x, y, theta)
    lo_x, hi_x = float(poly[:, 0].min()), float(poly[:, 0].max())
    lo_y, hi_y = float(poly[:, 1].min()), float(poly[:, 1].max())
    dx = -lo_x if lo_x < 0.0 else (W - hi_x if hi_x > W else 0.0)
    dy = -lo_y if lo_y < 0.0 else (H - hi_y if hi_y > H else 0.0)
    return x + dx, y + dy


def _place_rovers(rng: np.random.Generator, cfg: Config, family: str) -> list[RoverInit]:
    fp = Footprint(cfg.rover)
    W, H = cfg.board.width, cfg.board.height
    grid = Grid(cfg.board.cols, cfg.board.rows, cfg.board.cell_mm)
    hard = family == "hard"

    for _ in range(GEN.max_attempts):
        polys = []
        rovers: list[tuple[int, float, float, float]] = []
        ok = True
        # NB: the official (4,4)/(4,8) reference cells (interpretation_v0) are only
        # ~80mm apart while our measured rover (149x99.5mm incl. paddles, sweep
        # radius 113.5mm) cannot fit two side by side that close at 45deg -- spread
        # the second one further down the start edge so both are still "near the
        # start corner" but geometrically feasible.
        for rid, base_cell in zip(ROVER_IDS, ((4.0, 4.0), (4.0, 12.0))):
            if family in ("start_zone", "official_like", "official_like_slow"):
                col = base_cell[0] + rng.normal(0.0, 0.3)
                row = base_cell[1] + rng.normal(0.0, 0.3)
                x, y, _ = grid.to_internal_pose(col, row, 45.0)
                theta = grid.to_internal_heading(45.0 + rng.normal(0.0, 5.0))
                # The official (4,4)/(4,8) anchor is close enough to the corner that a
                # 45deg-rotated envelope (paddle reach makes the sweep radius 113.5mm)
                # can poke past the field edge. Push the centre inward by the overhang
                # rather than reject-sampling forever (ponytail: cheap, deterministic).
                mb = cfg.margins.board + 1.0          # start inside the planning margin, not on the edge
                x, y = _clamp_inside_field(fp, x - mb, y - mb, theta, W - 2 * mb, H - 2 * mb)
                x, y = x + mb, y + mb
            else:
                margin = fp.sweep_radius + 2.0
                x = rng.uniform(margin, W - margin)
                y = rng.uniform(margin, H - margin)
                theta = rng.uniform(-math.pi, math.pi)
            poly = _rover_polygon(fp, x, y, theta)
            if not S.inside_rect(poly, 0.0, W, 0.0, H):
                ok = False
                break
            for other in polys:
                if S.distance(poly, other) < GEN.min_clearance_mm:
                    ok = False
                    break
            if not ok:
                break
            polys.append(poly)
            rovers.append((rid, x, y, theta))
        if ok:
            return [RoverInit(id=rid, x=x, y=y, theta=th, motor=_motor_params(rng, rid, hard))
                    for rid, x, y, th in rovers]
    raise RuntimeError("failed to place rovers within max_attempts")


def _sample_cube_center(rng: np.random.Generator, cfg: Config, family: str, depots: dict[str, tuple[float, float]],
                         color: str, grid: Grid) -> tuple[float, float]:
    W, H = cfg.board.width, cfg.board.height
    m = cfg.cube.half_diag + 2.0
    if family in ("official_like", "official_like_slow"):
        return rng.uniform(*OFFICIAL_BAND), rng.uniform(*OFFICIAL_BAND)
    if family != "hard" or rng.random() > GEN.corner_bias_prob:
        return rng.uniform(m, W - m), rng.uniform(m, H - m)
    # hard: bias near a board edge or near this cube's own depot
    if rng.random() < 0.5:
        dcol, drow = depots[color]
        dx, dy = grid.to_internal_xy(dcol, drow)
        band = GEN.edge_band_cells * cfg.board.cell_mm
        return (
            min(max(dx + rng.normal(0.0, band), m), W - m),
            min(max(dy + rng.normal(0.0, band), m), H - m),
        )
    band = GEN.edge_band_cells * cfg.board.cell_mm
    edge = rng.integers(0, 4)
    if edge == 0:
        return rng.uniform(m, W - m), rng.uniform(m, m + band)
    if edge == 1:
        return rng.uniform(m, W - m), rng.uniform(H - m - band, H - m)
    if edge == 2:
        return rng.uniform(m, m + band), rng.uniform(m, H - m)
    return rng.uniform(W - m - band, W - m), rng.uniform(m, H - m)


def _sample_alpha(rng: np.random.Generator, family: str) -> float:
    if family != "hard":
        return float(rng.uniform(0.0, 2.0 * math.pi))
    # keep it genuinely rotated (away from the 0/90 axes) to exercise re-alignment
    quadrant = rng.integers(0, 4)
    deg = rng.uniform(GEN.rotation_min_deg, GEN.rotation_max_deg)
    return math.radians(float(quadrant) * 90.0 + deg)


def _place_cubes(rng: np.random.Generator, cfg: Config, family: str, rovers: list[RoverInit],
                  depots: dict[str, tuple[float, float]]) -> list[CubeInit]:
    grid = Grid(cfg.board.cols, cfg.board.rows, cfg.board.cell_mm)
    side = cfg.cube.side
    n = int(rng.integers(2, 4))  # 2 or 3, per contract (R9)
    if family in ("official_like", "official_like_slow"):
        n = 3                    # the mission under study: three cubes
    colors = [str(c) for c in rng.choice(np.array(COLORS), size=n, replace=False)]

    rover_polys = [_rover_polygon(Footprint(cfg.rover), r.x, r.y, r.theta) for r in rovers]
    placed: list = []
    result: list[CubeInit] = []
    for color in colors:
        for _ in range(GEN.max_attempts):
            x, y = _sample_cube_center(rng, cfg, family, depots, color, grid)
            alpha = _sample_alpha(rng, family)
            poly = _cube_polygon(x, y, side, alpha)
            if not S.inside_rect(poly, 0.0, cfg.board.width, 0.0, cfg.board.height):
                continue
            if any(S.distance(poly, rp) < GEN.min_clearance_mm for rp in rover_polys):
                continue
            if family == "hard" and placed and rng.random() < GEN.near_other_cube_prob:
                # nudge toward an already-placed cube (still must clear min_clearance)
                ox, oy = placed[-1].x, placed[-1].y
                x = ox + rng.normal(0.0, side * 1.2)
                y = oy + rng.normal(0.0, side * 1.2)
                poly = _cube_polygon(x, y, side, alpha)
                if not S.inside_rect(poly, 0.0, cfg.board.width, 0.0, cfg.board.height):
                    continue
                if any(S.distance(poly, rp) < GEN.min_clearance_mm for rp in rover_polys):
                    continue
            if any(S.distance(poly, pp) < GEN.min_clearance_mm for pp in (_cube_polygon(p.x, p.y, side, p.alpha) for p in placed)):
                continue
            dcol, drow = depots[color]
            dx, dy = grid.to_internal_xy(dcol, drow)
            if DepotZone(color, dx, dy, cfg.depot.half_size).contains_cube(x, y, side, alpha):
                continue
            cube = CubeInit(color=color, x=float(x), y=float(y), alpha=float(alpha))
            placed.append(cube)
            result.append(cube)
            break
        else:
            raise RuntimeError(f"failed to place cube {color} within max_attempts")
    return result


def generate(seed: int, family: str, cfg: Config = DEFAULT) -> Scenario:
    if family not in FAMILIES:
        raise ValueError(f"unknown family {family!r}, expected one of {FAMILIES}")
    rng = np.random.default_rng(seed)
    hard = family == "hard"

    corners = [(cfg.board.cols, 0), (0, cfg.board.rows), (cfg.board.cols, cfg.board.rows)]
    corner_pts = [_depot_point(c, cfg.board.cols, cfg.board.rows) for c in corners]
    shuffled_colors = [str(c) for c in rng.permutation(np.array(COLORS))]
    depots = {color: corner_pts[i] for i, color in enumerate(shuffled_colors)}
    start = _depot_point((0, 0), cfg.board.cols, cfg.board.rows)

    rovers = _place_rovers(rng, cfg, family)
    cubes = _place_cubes(rng, cfg, family, rovers, depots)
    sensor = _sensor_params(rng, hard, family)
    contact = _contact_params(rng, hard)

    scenario = Scenario(
        seed=seed, family=family, rovers=rovers, cubes=cubes, depots=depots, start=start,
        sensor=sensor, contact=contact,
        board_cols=cfg.board.cols, board_rows=cfg.board.rows, cell_mm=cfg.board.cell_mm,
    )
    problems = validate(scenario, cfg)
    if problems:
        raise RuntimeError(f"generated invalid scenario seed={seed} family={family}: {problems}")
    return scenario


def instantiate(scenario: Scenario, cfg: Config = DEFAULT) -> tuple[PhysicsWorld, VisionEmulator]:
    """Build the two runnable ground-truth objects (`PhysicsWorld`, `VisionEmulator`)
    from a `Scenario`. The runner drives `world` and reads telemetry through `emu`
    (see physics.PhysicsWorld / sensors.VisionEmulator docstrings for the step/poll
    API). Physics and sensor noise get independent RNG streams spawned from the same
    seed (`np.random.SeedSequence.spawn`), so the pair is still fully determined by
    `scenario.seed` alone.
    """
    # A serialized scenario is authoritative for board geometry. Rebuild only the
    # board portion of the supplied config so restoring a 50x50 snapshot cannot
    # silently emit 43x43 telemetry or simulate on a different field.
    scenario_cfg = replace(
        cfg,
        board=replace(cfg.board, cols=scenario.board_cols, rows=scenario.board_rows,
                      cell_mm=scenario.cell_mm),
    )
    rng_phys, rng_sensor = (np.random.default_rng(s) for s in np.random.SeedSequence(scenario.seed).spawn(2))
    rovers = [SimRover(id=r.id, x=r.x, y=r.y, theta=r.theta, motor=r.motor) for r in scenario.rovers]
    cubes = [SimCube(color=c.color, x=c.x, y=c.y, alpha=c.alpha) for c in scenario.cubes]
    world = PhysicsWorld(scenario_cfg, rovers, cubes, scenario.contact, rng_phys)
    emu = VisionEmulator(scenario.sensor, rng_sensor, scenario.depots, scenario.start)
    return world, emu
