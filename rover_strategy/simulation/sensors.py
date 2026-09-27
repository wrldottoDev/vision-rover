"""Vision emulator: publishes official contract-v1 telemetry dicts from a PhysicsWorld.

Modelled pathologies (per CONTRATO.md s2/s6 and the lead brief): capture->delivery
latency (mean + jitter, sampled per published frame, monotonic so a single ordered
TCP-like channel never reorders messages), whole dropped frames (creates the seq
gaps the contract says are normal), per-rover detection loss (steady-state prob +
occasional ~1 s bursts) with last-known pose + growing `age_ms`, a fixed (unknown to
the strategy) marker->rotation-centre offset per rover, Gaussian rover pose noise
with rare heavy-tailed outliers, cube centre noise + a small fixed per-cube bias, and
cube occlusion.

Occlusion model (lead correction): the camera looks straight down. The paddle rails
are 3 mm thin and only 14 mm tall with an OPEN channel between them, so a cube being
pushed stays visible from above -- only the chassis BODY casts a meaningful shadow.
Coverage is therefore computed from `Footprint.parts()[0]` (the body rect,
`x in [-47, 47]`), not the full envelope, optionally enlarged outward from the field
nadir by the camera-height parallax factor kappa = H/(H-h). Above ~70% covered the
official detector drops the reading (frozen last position, growing age); ~15-70%
covered it still publishes but with extra error, biased toward the UNCOVERED side
(the visible remainder pulls the estimated centroid that way).

Cube orientation is NEVER published (matches the contract; there is no field for it).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from ..frames import Grid, body_to_marker, wrap
from .physics import PhysicsWorld, SimRover, _clip_convex, _pts, _poly_area, _poly_centroid, _square_pts

PHASES = ("IDLE", "READY", "RUNNING", "FINISHED")


@dataclass(frozen=True)
class SensorParams:
    publish_hz: float = 20.0
    jitter_std_s: float = 0.004          # ASSUMED: per-frame capture-interval jitter
    latency_mean_s: float = 0.05
    latency_jitter_s: float = 0.02
    frame_drop_prob: float = 0.02        # whole published frame lost (creates seq gaps)

    rover_loss_prob: float = 0.01        # steady-state per-frame detection loss
    rover_loss_burst_prob: float = 0.002
    rover_loss_burst_s: float = 1.0

    marker_offset_mm: float = 4.0        # true |marker->rotation-centre| offset magnitude, sampled once/rover
    marker_angle_offset_rad: float = math.radians(1.0)

    pos_noise_std_mm: float = 2.0        # MEASURED-ish: sim publisher sigma ~1.2mm (interpretation_v0 G8); widened
    heading_noise_std_rad: float = math.radians(1.5)
    outlier_prob: float = 0.005
    outlier_scale: float = 8.0

    cube_pos_noise_std_mm: float = 2.0
    cube_pos_bias_mm: float = 1.5        # per-cube fixed bias magnitude, sampled once/cube

    camera_height_mm: float = 2100.0     # MEASURED-ish (interpretation_v0 default rig height)
    rover_height_mm: float = 90.0        # chassis top height above the floor
    parallax_enabled: bool = True

    full_occlusion_frac: float = 0.70    # ASSUMED (lead brief): detector drops the reading above this
    partial_occlusion_frac: float = 0.15  # ASSUMED: below this, no extra error
    partial_extra_noise_std_mm: float = 2.0
    partial_bias_mm: float = 4.0         # extra bias toward the uncovered side at frac == full threshold


def _kappa(p: SensorParams) -> float:
    h = p.rover_height_mm
    H = p.camera_height_mm
    return H / (H - h) if 0.0 < h < H else 1.0


class VisionEmulator:
    """params/rng are the tunables; `depots`/`start_xy` are the scenario's static,
    never-changing corners (official cell units) -- passed once at construction, same
    reason the contract keeps them out of the per-frame dynamic lists."""

    def __init__(
        self,
        params: SensorParams,
        rng: np.random.Generator,
        depots: dict[str, tuple[float, float]],
        start_xy: tuple[float, float],
    ):
        self.params = params
        self.rng = rng
        self.depots = dict(depots)
        self.start_xy = start_xy
        self.phase = "IDLE"
        self._seq = 0
        self._next_capture_t = 0.0
        self._last_delivery_t = -1.0
        self._queue: deque[dict] = deque()
        self._rover_state: dict[int, dict] = {}
        self._cube_state: dict[str, dict] = {}

    def set_phase(self, phase: str) -> None:
        assert phase in PHASES, phase
        self.phase = phase

    # ------------------------------------------------------------- capture --
    def capture(self, world: PhysicsWorld, t_now: float) -> None:
        """Call every physics step; internally paces itself to ~publish_hz (+jitter)."""
        period = 1.0 / self.params.publish_hz
        guard = 0
        while t_now >= self._next_capture_t and guard < 1000:
            self._emit(world, self._next_capture_t)
            jitter = self.rng.normal(0.0, self.params.jitter_std_s)
            self._next_capture_t += max(0.2 * period, period + jitter)
            guard += 1

    def _emit(self, world: PhysicsWorld, t_capture: float) -> None:
        self._seq += 1
        if self.rng.random() < self.params.frame_drop_prob:
            return  # whole frame lost: the consumer will see a seq gap
        grid = Grid(world.cfg.board.cols, world.cfg.board.rows, world.cfg.board.cell_mm)
        rovers = [self._capture_rover(rid, r, t_capture, grid) for rid, r in world.rovers.items()]
        cubes = [
            self._capture_cube(color, c, t_capture, world, grid)
            for color, c in world.cubes.items()
        ]
        msg = {
            "v": 1,
            "seq": self._seq,
            "ts_ms": int(round(t_capture * 1000.0)),
            "phase": self.phase,
            "grid": {"cols": grid.cols, "rows": grid.rows, "cell_mm": grid.cell_mm},
            "rovers": rovers,
            "cubes": cubes,
            "obstacles": [],
            "start": {"col": self.start_xy[0], "row": self.start_xy[1]},
            "depots": [{"color": k, "col": v[0], "row": v[1]} for k, v in self.depots.items()],
        }
        latency = max(0.0, self.rng.normal(self.params.latency_mean_s, self.params.latency_jitter_s))
        delivery_t = max(t_capture + latency, self._last_delivery_t + 1e-6)
        self._last_delivery_t = delivery_t
        self._queue.append((delivery_t, msg))

    def _sample_marker_offset(self) -> tuple[float, float, float]:
        ang = self.rng.uniform(0.0, 2.0 * math.pi)
        mag = self.params.marker_offset_mm
        fwd, left = mag * math.cos(ang), mag * math.sin(ang)
        dth = self.rng.normal(0.0, self.params.marker_angle_offset_rad)
        return fwd, left, dth

    def _capture_rover(self, rid: int, r: SimRover, t_capture: float, grid: Grid) -> dict:
        st = self._rover_state.setdefault(
            rid,
            {
                "last_seen": t_capture,
                "last_pub": (r.x, r.y, r.theta),
                "burst_until": -1.0,
                "offset": self._sample_marker_offset(),
            },
        )
        lost = False
        if t_capture < st["burst_until"]:
            lost = True
        elif self.rng.random() < self.params.rover_loss_burst_prob:
            st["burst_until"] = t_capture + self.rng.uniform(0.3, self.params.rover_loss_burst_s)
            lost = True
        elif self.rng.random() < self.params.rover_loss_prob:
            lost = True

        if not lost:
            fwd, left, dth = st["offset"]
            mx, my, mth = body_to_marker(r.x, r.y, r.theta, fwd, left, dth)
            std = self.params.pos_noise_std_mm
            if self.rng.random() < self.params.outlier_prob:
                std *= self.params.outlier_scale
            mx += self.rng.normal(0.0, std)
            my += self.rng.normal(0.0, std)
            mth = wrap(mth + self.rng.normal(0.0, self.params.heading_noise_std_rad))
            st["last_pub"] = (mx, my, mth)
            st["last_seen"] = t_capture

        x, y, th = st["last_pub"]
        age_ms = max(0, int(round((t_capture - st["last_seen"]) * 1000.0)))
        col, row = grid.to_official_xy(x, y)
        theta_deg = Grid.to_official_heading(th)
        return {"id": rid, "col": col, "row": row, "theta": theta_deg, "age_ms": age_ms}

    def _coverage(self, c, world: PhysicsWorld) -> tuple[float, tuple[float, float] | None]:
        """Fraction of the cube's top area covered by a rover BODY (not envelope,
        not paddles -- see module docstring), plus the covered region's centroid."""
        side = world.cfg.cube.side
        cube_poly = _square_pts(c.x, c.y, side, c.alpha)
        area = side * side
        k = _kappa(self.params)
        nadir = (world.cfg.board.width / 2.0, world.cfg.board.height / 2.0)
        best_frac, best_centroid = 0.0, None
        for r in world.rovers.values():
            body = world.footprint.parts(r.x, r.y, r.theta)[0]
            body_pts = _pts(body)
            if self.params.parallax_enabled and k != 1.0:
                body_pts = [(nadir[0] + k * (x - nadir[0]), nadir[1] + k * (y - nadir[1])) for x, y in body_pts]
            inter = _clip_convex(cube_poly, body_pts)
            a = _poly_area(inter)
            frac = a / area if area > 0 else 0.0
            if frac > best_frac:
                best_frac = frac
                best_centroid = _poly_centroid(inter) if len(inter) >= 3 else None
        return best_frac, best_centroid

    def _capture_cube(self, color: str, c, t_capture: float, world: PhysicsWorld, grid: Grid) -> dict:
        st = self._cube_state.setdefault(
            color,
            {
                "last_seen": t_capture,
                "last_pub": (c.x, c.y),
                "bias": tuple(self.rng.normal(0.0, self.params.cube_pos_bias_mm, size=2)),
            },
        )
        frac, cov_centroid = self._coverage(c, world)
        if frac < self.params.full_occlusion_frac:
            std = self.params.cube_pos_noise_std_mm
            bx = by = 0.0
            if frac > self.params.partial_occlusion_frac and cov_centroid is not None:
                span = max(1e-6, self.params.full_occlusion_frac - self.params.partial_occlusion_frac)
                t = min(1.0, max(0.0, (frac - self.params.partial_occlusion_frac) / span))
                std += t * self.params.partial_extra_noise_std_mm
                dx, dy = c.x - cov_centroid[0], c.y - cov_centroid[1]
                L = math.hypot(dx, dy)
                if L > 1e-6:
                    bx, by = dx / L * t * self.params.partial_bias_mm, dy / L * t * self.params.partial_bias_mm
            if self.rng.random() < self.params.outlier_prob:
                std *= self.params.outlier_scale
            nx = c.x + st["bias"][0] + bx + self.rng.normal(0.0, std)
            ny = c.y + st["bias"][1] + by + self.rng.normal(0.0, std)
            st["last_pub"] = (nx, ny)
            st["last_seen"] = t_capture

        x, y = st["last_pub"]
        age_ms = max(0, int(round((t_capture - st["last_seen"]) * 1000.0)))
        col, row = grid.to_official_xy(x, y)
        return {"color": color, "col": col, "row": row, "age_ms": age_ms}

    # --------------------------------------------------------------- poll --
    def poll(self, t_now: float) -> list[dict]:
        out = []
        while self._queue and self._queue[0][0] <= t_now:
            out.append(self._queue.popleft()[1])
        return out
