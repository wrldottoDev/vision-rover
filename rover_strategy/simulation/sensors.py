"""Vision emulator: publishes official contract-v1 telemetry dicts from a PhysicsWorld.

Modelled pathologies (per CONTRATO.md s2/s6 and the lead brief): independent
capture/publication clocks and capture->delivery latency (mean + jitter or an optional
heavy-tailed profile; the ordered TCP-like channel never reorders messages), whole dropped frames (creates the seq
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
from dataclasses import dataclass

import numpy as np

from ..frames import Grid, body_to_marker, wrap
from .physics import PhysicsWorld, SimRover, _clip_convex, _pts, _poly_area, _poly_centroid, _square_pts

PHASES = ("IDLE", "READY", "RUNNING", "FINISHED")


@dataclass(frozen=True)
class SensorParams:
    publish_hz: float = 20.0
    capture_hz: float = 30.0       # official camera/processing clock
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
    cube_height_mm: float = 60.0         # ASSUMED: cube top plane for rover-shadow projection
    parallax_enabled: bool = True

    full_occlusion_frac: float = 0.70    # ASSUMED (lead brief): detector drops the reading above this
    partial_occlusion_frac: float = 0.15  # ASSUMED: below this, no extra error
    partial_extra_noise_std_mm: float = 2.0
    partial_bias_mm: float = 4.0         # extra bias toward the uncovered side at frac == full threshold

    # ASSUMED detector threshold. The black 100 mm marker plus 20 mm white border
    # is centred on a field corner; the configured 70 mm in-field quarter is used
    # for coverage. One covered quarter is tolerated by the saved-homography path;
    # two or more freeze the official state.
    corner_marker_coverage_threshold: float = 0.25

    # `publish_hz` is the publication timer. `capture_hz` is the independent
    # camera/processing timer used by the official engine.
    latency_profile: str = "default"


def _projection_factor(p: SensorParams, source_height_mm: float, target_height_mm: float) -> float:
    """Project a top silhouette onto a lower horizontal plane, from a nadir camera."""
    if not p.parallax_enabled:
        return 1.0
    H = p.camera_height_mm
    if not 0.0 < source_height_mm < H or target_height_mm >= H:
        return 1.0
    return (H - target_height_mm) / (H - source_height_mm)


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
        self._next_publication_t = 0.0
        self._last_delivery_t = -1.0
        self._queue: deque[tuple[float, dict]] = deque()
        self._rover_state: dict[int, dict] = {}
        self._cube_state: dict[str, dict] = {}
        self._latest_state: dict | None = None
        self._homography_valid = False

    def set_phase(self, phase: str) -> None:
        assert phase in PHASES, phase
        self.phase = phase

    # ------------------------------------------------------------- capture --
    def capture(self, world: PhysicsWorld, t_now: float) -> None:
        """Advance camera processing to ``t_now``.

        Publication is driven separately by its own timer. A publication tick due
        before a capture uses the previous slot; the tick at the capture timestamp
        sees the new slot, matching the official processing/publication split.
        """
        capture_hz = self.params.capture_hz
        period = 1.0 / capture_hz
        guard = 0
        while t_now >= self._next_capture_t and guard < 1000:
            t_capture = self._next_capture_t
            self._advance_publication(t_capture, inclusive=False)
            self._capture(world, t_capture)
            self._advance_publication(t_capture, inclusive=True)
            jitter = self.rng.normal(0.0, self.params.jitter_std_s)
            self._next_capture_t += max(0.2 * period, period + jitter)
            guard += 1
        self._advance_publication(t_now, inclusive=True)

    def _capture(self, world: PhysicsWorld, t_capture: float) -> None:
        """Process one camera frame and replace the latest-state slot if valid."""
        grid = Grid(world.cfg.board.cols, world.cfg.board.rows, world.cfg.board.cell_mm)
        coverages = self._corner_marker_coverages(world)
        covered = sum(frac >= self.params.corner_marker_coverage_threshold for frac in coverages)
        if not self._homography_valid:
            # A saved homography cannot be inferred from one or two visible
            # corners.  Withhold all telemetry until a clean four-marker
            # capture establishes the first valid geometry.
            if covered != 0:
                return
            self._homography_valid = True
        elif covered >= 2:
            # Official freeze: keep the complete last good state, including its
            # capture timestamp and cached object ages.
            return

        rovers = [obs for rid, r in world.rovers.items()
                  if (obs := self._capture_rover(rid, r, t_capture, grid)) is not None]
        cubes = [obs for color, c in world.cubes.items() if c.in_play
                 and (obs := self._capture_cube(color, c, t_capture, world, grid)) is not None]
        self._latest_state = {
            "v": 1,
            "ts_ms": int(round(t_capture * 1000.0)),
            "phase": self.phase,
            "grid": {"cols": grid.cols, "rows": grid.rows, "cell_mm": grid.cell_mm},
            "rovers": rovers,
            "cubes": cubes,
            "obstacles": [],
            "start": {"col": self.start_xy[0], "row": self.start_xy[1]},
            "depots": [{"color": k, "col": v[0], "row": v[1]} for k, v in self.depots.items()],
        }

    def _advance_publication(self, t_now: float, *, inclusive: bool) -> None:
        """Run the independent publication timer up to ``t_now``."""
        period = 1.0 / self.params.publish_hz
        while (self._next_publication_t < t_now or
               (inclusive and self._next_publication_t <= t_now)):
            self._publish(self._next_publication_t)
            self._next_publication_t += period

    def _sample_latency(self) -> float:
        if self.params.latency_profile == "realistic_latency":
            # Lognormal(mu=ln(.16), sigma=.65) has p95 ~= .47 s and a bounded
            # worst case matching the user-reported ~1.42 s tail.
            return min(1.42, float(self.rng.lognormal(math.log(0.16), 0.65)))
        if self.params.latency_profile != "default":
            raise ValueError(f"unknown latency profile {self.params.latency_profile!r}")
        return max(0.0, self.rng.normal(self.params.latency_mean_s, self.params.latency_jitter_s))

    def _publish(self, t_publication: float) -> None:
        """Publish the current slot; seq advances even for a dropped message."""
        self._seq += 1
        if self.rng.random() < self.params.frame_drop_prob:
            return  # whole frame lost: the consumer will see a seq gap
        state = self._latest_state
        if state is None:
            # No placeholder grid/objects: the official client must not infer
            # a homography or object absence before the first valid capture.
            return
        else:
            msg = dict(state)
            msg["seq"] = self._seq
        if len(self._queue) >= 2:
            # Preserve the in-flight item and replace only the pending slot.
            # Its delivery reservation remains unchanged, so a slow consumer
            # still receives the newest state at the next available arrival.
            self._queue[-1] = (self._queue[-1][0], msg)
        else:
            delivery_t = max(t_publication + self._sample_latency(), self._last_delivery_t + 1e-6)
            self._last_delivery_t = delivery_t
            self._queue.append((delivery_t, msg))

    def _sample_marker_offset(self) -> tuple[float, float, float]:
        ang = self.rng.uniform(0.0, 2.0 * math.pi)
        mag = self.params.marker_offset_mm
        fwd, left = mag * math.cos(ang), mag * math.sin(ang)
        dth = self.rng.normal(0.0, self.params.marker_angle_offset_rad)
        return fwd, left, dth

    def _capture_rover(self, rid: int, r: SimRover, t_capture: float, grid: Grid) -> dict:
        st = self._rover_state.get(rid)
        if st is None:
            st = {"last_seen": t_capture, "last_pub": None,
                  "burst_until": -1.0, "offset": self._sample_marker_offset()}
            self._rover_state[rid] = st
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

        if st["last_pub"] is None:
            return None

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
        k = _projection_factor(self.params, self.params.rover_height_mm, self.params.cube_height_mm)
        nadir = (world.cfg.board.width / 2.0, world.cfg.board.height / 2.0)
        occluders = []
        for r in world.rovers.values():
            body = world.footprint.parts(r.x, r.y, r.theta)[0]
            body_pts = _pts(body)
            occluders.append(self._project(body_pts, nadir, k))
        inters = [_clip_convex(cube_poly, p) for p in occluders]
        covered, centroid = _union_area_and_centroid(inters)
        return covered / area if area > 0 else 0.0, centroid

    def _capture_cube(self, color: str, c, t_capture: float, world: PhysicsWorld, grid: Grid) -> dict:
        st = self._cube_state.setdefault(
            color,
            {
                "last_seen": t_capture,
                "last_pub": None,
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

        if st["last_pub"] is None:
            return None

        x, y = st["last_pub"]
        age_ms = max(0, int(round((t_capture - st["last_seen"]) * 1000.0)))
        col, row = grid.to_official_xy(x, y)
        return {"color": color, "col": col, "row": row, "age_ms": age_ms}

    # --------------------------------------------------------------- poll --
    def poll(self, t_now: float) -> list[dict]:
        self._advance_publication(t_now, inclusive=True)
        ready = []
        while self._queue and self._queue[0][0] <= t_now:
            ready.append(self._queue.popleft()[1])
        # The official transport has one bounded latest-value slot per client.
        # Keep future deliveries (which preserve channel order), but collapse all
        # currently ready backlog to the newest message.
        return [ready[-1]] if ready else []

    def _project(self, points, nadir: tuple[float, float], factor: float):
        nx, ny = nadir
        return [(nx + factor * (x - nx), ny + factor * (y - ny)) for x, y in points]

    def _corner_marker_coverages(self, world: PhysicsWorld) -> list[float]:
        """Return union-covered fractions for the four in-field marker quarters."""
        half = world.cfg.board.corner_marker_half_mm
        W, H = world.cfg.board.width, world.cfg.board.height
        corners = ((0.0, 0.0), (W, 0.0), (0.0, H), (W, H))
        occluders = []
        for c in world.cubes.values():
            if c.in_play:
                poly = self._project(
                    _square_pts(c.x, c.y, world.cfg.cube.side, c.alpha),
                    (W / 2.0, H / 2.0),
                    _projection_factor(self.params, self.params.cube_height_mm, 0.0),
                )
                occluders.append(poly)
        for r in world.rovers.values():
            for part in world.footprint.parts(r.x, r.y, r.theta):
                poly = self._project(
                    _pts(part), (W / 2.0, H / 2.0),
                    _projection_factor(self.params, self.params.rover_height_mm, 0.0),
                )
                occluders.append(poly)

        coverages = []
        for cx, cy in corners:
            x0, x1 = (cx, cx + half) if cx == 0.0 else (cx - half, cx)
            y0, y1 = (cy, cy + half) if cy == 0.0 else (cy - half, cy)
            region = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            intersections = [_clip_convex(region, poly) for poly in occluders]
            covered, _ = _union_area_and_centroid(intersections)
            coverages.append(min(1.0, max(0.0, covered / (half * half))))
        return coverages


def _union_area_and_centroid(polygons: list[list[tuple[float, float]]]) -> tuple[float, tuple[float, float] | None]:
    """Exact union for the small convex-polygon sets used by the sensor model."""
    polygons = [p for p in polygons if len(p) >= 3 and _poly_area(p) > 1e-9]
    if not polygons:
        return 0.0, None
    area = 0.0
    cx = cy = 0.0
    # Inclusion-exclusion is compact and robust here: at most three cubes and
    # three U-shaped rover parts are present, and all intersections stay convex.
    n = len(polygons)
    for mask in range(1, 1 << n):
        inter = polygons[(mask & -mask).bit_length() - 1]
        bits = mask & (mask - 1)
        while bits:
            bit = bits & -bits
            inter = _clip_convex(inter, polygons[bit.bit_length() - 1])
            bits -= bit
            if len(inter) < 3:
                break
        a = _poly_area(inter)
        if a <= 1e-9:
            continue
        sign = 1.0 if mask.bit_count() % 2 else -1.0
        c = _poly_centroid(inter)
        area += sign * a
        cx += sign * a * c[0]
        cy += sign * a * c[1]
    if area <= 1e-9:
        return 0.0, None
    return area, (cx / area, cy / area)
