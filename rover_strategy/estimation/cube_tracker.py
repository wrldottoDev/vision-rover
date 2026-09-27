"""Position filter for one cube: static-with-jumps model.

A cube does not move on its own -- position is constant except while a rover is
actively pushing it (`set_pushed(True)`), so the process model is "random walk with a
process-noise rate that depends on whether we are currently pushing": tiny while
static (just enough that the filter never fully stops listening to new evidence),
large while pushed (so the estimate can track genuine motion quickly).

Two independent scalar filters (x, y) are used rather than a full 2x2 covariance:
vision gives an isotropic centroid estimate with no cross-axis correlation modelled,
so a diagonal covariance is the right amount of machinery.

Vision feed quirks handled here (mirrors pose_estimator.py):
  * The feed repeats the last-known position with growing `age_ms` while occluded;
    `age_s > 0` on a call means "not a fresh observation" and must not be treated as
    new evidence (no Kalman correction, no jump/orientation-reset check).
  * When >= 2 corner markers are occluded, the feed can additionally FREEZE: same
    t_capture, unchanged age, repeated verbatim. Such a repeat is detected by
    `t_capture` not having advanced past the last call and is dropped before it can
    reset anything -- age_s alone is not a reliable freshness signal for this case.
  * Because a frozen feed cannot itself carry "time is passing" information, the
    growing uncertainty while stale is evaluated lazily in `estimate(t)` against the
    caller's own clock `t`, not against the (possibly frozen) feed timestamps. The
    running covariance is only advanced by elapsed feed time at the moment a genuinely
    fresh observation arrives (predict-then-update).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..world import CubeEstimate


@dataclass(frozen=True)
class CubeTrackerConfig:
    # ASSUMED mm, fresh vision centroid noise. docs/interpretation_v0.md quotes a 5 mm
    # worst-case *occluded* sigma; a fresh (unoccluded) centroid is assumed tighter,
    # in line with the rover position noise (1-3 mm sigma) given for the same camera.
    meas_pos_std: float = 3.0
    # TUNED mm/sqrt(s) random-walk while NOT pushed: keeps the Kalman gain from decaying
    # to exactly zero after many static frames (a cube nudged without triggering the
    # jump gate must still be trackable).
    static_process_std_per_s: float = 0.3
    # TUNED mm/sqrt(s) while being pushed: lets the estimate follow genuine motion
    # instead of lagging behind a moving cube.
    pushed_process_std_per_s: float = 120.0
    # ASSUMED mm floor on reported std: systematic vision bias (lens distortion,
    # colour-segmentation centroid offset) does not average out with more frames.
    bias_floor_mm: float = 1.5
    # TUNED mm: a fresh observation vs. current estimate distance beyond this, while
    # NOT pushed, is flagged as an unexpected move. Combined 1-sigma of a fresh
    # measurement (3 mm) and a converged static estimate (floor 1.5 mm) is ~3.4 mm;
    # this sits at ~4.5 sigma above that -- comfortably above sensor jitter, well below
    # any real nudge of a 60 mm cube.
    jump_threshold_mm: float = 15.0
    # ASSUMED mm: uncertainty before the first observation is ever seen.
    init_pos_std: float = 10.0


class CubeTracker:
    """Position (+ optional orientation) filter for one cube."""

    def __init__(self, color: str, cfg: CubeTrackerConfig = CubeTrackerConfig()):
        self.color = color
        self.cfg = cfg
        self._initialized = False
        self._x = 0.0
        self._y = 0.0
        self._Pxx = cfg.init_pos_std ** 2
        self._Pyy = cfg.init_pos_std ** 2
        self._pushed = False
        self._alpha: float | None = None
        self._alpha_std = math.pi / 4
        self._unexpected = False
        self._last_fresh_t = 0.0
        self._last_seen_capture_t = -math.inf

    def _process_var_rate(self) -> float:
        std_rate = self.cfg.pushed_process_std_per_s if self._pushed else self.cfg.static_process_std_per_s
        return std_rate ** 2

    def update(self, x: float, y: float, t_capture: float, age_s: float) -> None:
        if not self._initialized:
            self._x, self._y = x, y
            self._Pxx = self._Pyy = self.cfg.init_pos_std ** 2
            self._last_fresh_t = t_capture
            self._last_seen_capture_t = t_capture
            self._initialized = True
            return

        if t_capture <= self._last_seen_capture_t:
            # Frozen/duplicate frame (or out-of-order): no new information.
            return
        self._last_seen_capture_t = t_capture

        if age_s > 0.0:
            # Feed is repeating a stale detection (partial occlusion with advancing
            # age): not new evidence. Uncertainty growth is handled lazily in estimate().
            return

        # Fresh observation: predict the running covariance to t_capture, then correct.
        elapsed = max(0.0, t_capture - self._last_fresh_t)
        q_rate = self._process_var_rate()
        self._Pxx += q_rate * elapsed
        self._Pyy += q_rate * elapsed

        dist = math.hypot(x - self._x, y - self._y)
        self._unexpected = (not self._pushed) and (dist > self.cfg.jump_threshold_mm)
        if self._unexpected:
            self.set_orientation(None, math.pi / 4)

        r = self.cfg.meas_pos_std ** 2
        kx = self._Pxx / (self._Pxx + r)
        ky = self._Pyy / (self._Pyy + r)
        self._x += kx * (x - self._x)
        self._y += ky * (y - self._y)
        self._Pxx = max((1.0 - kx) * self._Pxx, self.cfg.bias_floor_mm ** 2)
        self._Pyy = max((1.0 - ky) * self._Pyy, self.cfg.bias_floor_mm ** 2)
        self._last_fresh_t = t_capture

    def estimate(self, t: float) -> CubeEstimate:
        elapsed = max(0.0, t - self._last_fresh_t) if self._initialized else math.inf
        q_rate = self._process_var_rate()
        proj_pxx = self._Pxx + q_rate * elapsed
        proj_pyy = self._Pyy + q_rate * elapsed
        pos_std = max(math.sqrt(max(proj_pxx, proj_pyy)), self.cfg.bias_floor_mm)
        return CubeEstimate(
            color=self.color,
            x=self._x,
            y=self._y,
            pos_std=pos_std,
            age_s=elapsed,
            alpha=self._alpha,
            alpha_std=self._alpha_std,
            delivered=False,
        )

    def set_pushed(self, flag: bool) -> None:
        self._pushed = flag

    def set_orientation(self, alpha: float | None, std: float) -> None:
        if alpha is None:
            self._alpha = None
        else:
            r = alpha % (math.pi / 2.0)
            if r >= math.pi / 4.0:
                r -= math.pi / 2.0
            self._alpha = r
        self._alpha_std = std

    def moved_unexpectedly(self) -> bool:
        """True if the most recent fresh observation jumped beyond the gate while the
        cube was not being pushed. Level-triggered: reflects the last fresh update
        until the next fresh update re-evaluates it."""
        return self._unexpected
