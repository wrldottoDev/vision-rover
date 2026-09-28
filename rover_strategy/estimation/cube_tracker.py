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
    caller's own clock (`_last_known_t`, advanced by every `estimate()`/`update()`
    call), not against the (possibly frozen) feed timestamps.

Mode transitions while stale (`set_pushed` toggled with no fresh observation in
between): `set_pushed` has no clock argument, so the best available "now" is
`_last_known_t`. Toggling bakes the variance accrued so far under the OLD mode
directly into `_Pxx`/`_Pyy` and advances a separate `_var_baseline_t` checkpoint to
that instant, before flipping the mode -- so stopping a push mid-occlusion cannot
retroactively erase the uncertainty that push accumulated. `_var_baseline_t` is
deliberately kept separate from `_last_fresh_t` (which drives the reported `age_s`):
baking in variance is bookkeeping, not new evidence, and must not make the estimate
look fresher than it is.

Jump / reacquisition: a fresh observation that lands far from the current estimate
(beyond `jump_threshold_mm`, and not flagged as an expected push) still needs the
filter to actually MOVE to the new position, not just flag it and keep reporting the
old (over-confident) one. Each fresh update inflates `_Pxx`/`_Pyy` by a term
proportional to its own residual squared (`_JUMP_ADAPT_K * residual**2`) before the
Kalman gain is computed -- a large, isolated residual (a real jump, or a lag that has
built up while smoothly tracking an unannounced push) raises the gain enough to
actually catch up, while typical few-mm sensor noise contributes a negligible amount
and the static-tracking floor/shrink behaviour is unaffected.
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
    # TUNED (dimensionless): innovation-adaptive inflation added to Pxx/Pyy as
    # k*residual**2 before each fresh update's Kalman gain is computed. At the
    # converged static floor (residual ~ a few mm of sensor noise) this adds a
    # fraction of a mm**2, invisible next to the floor; at a real jump/lag residual
    # (tens of mm) it dominates, driving the gain towards 1 so the estimate actually
    # catches up instead of crawling towards it over many frames.
    jump_adapt_k: float = 0.2


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
        self._last_fresh_t = 0.0        # last genuinely new evidence -> drives age_s
        self._var_baseline_t = 0.0      # last instant Pxx/Pyy fully account for -> growth checkpoint
        self._last_known_t = 0.0        # latest time we have ANY signal about (query or capture)
        self._last_seen_capture_t = -math.inf

    def _process_var_rate(self) -> float:
        std_rate = self.cfg.pushed_process_std_per_s if self._pushed else self.cfg.static_process_std_per_s
        return std_rate ** 2

    def _accumulate_var_to(self, t: float) -> None:
        """Grow Pxx/Pyy by the CURRENT mode's rate over [_var_baseline_t, t], then
        advance the checkpoint. Must be called with the mode that was actually in
        effect during that interval (i.e. before flipping `_pushed`)."""
        elapsed = max(0.0, t - self._var_baseline_t)
        if elapsed > 0.0:
            q_rate = self._process_var_rate()
            self._Pxx += q_rate * elapsed
            self._Pyy += q_rate * elapsed
            self._var_baseline_t = t

    def update(self, x: float, y: float, t_capture: float, age_s: float) -> None:
        if not self._initialized:
            self._x, self._y = x, y
            self._Pxx = self._Pyy = self.cfg.init_pos_std ** 2
            self._last_fresh_t = t_capture
            self._var_baseline_t = t_capture
            self._last_known_t = t_capture
            self._last_seen_capture_t = t_capture
            self._initialized = True
            return

        if t_capture <= self._last_seen_capture_t:
            # Frozen/duplicate frame (or out-of-order): no new information.
            return
        self._last_seen_capture_t = t_capture
        self._last_known_t = max(self._last_known_t, t_capture)

        if age_s > 0.0:
            # Feed is repeating a stale detection (partial occlusion with advancing
            # age): not new evidence. Uncertainty growth is handled lazily in estimate().
            return

        # Fresh observation: predict the running covariance to t_capture, then correct.
        self._accumulate_var_to(t_capture)

        dist = math.hypot(x - self._x, y - self._y)
        self._unexpected = (not self._pushed) and (dist > self.cfg.jump_threshold_mm)
        if self._unexpected:
            self.set_orientation(None, math.pi / 4)

        # Innovation-adaptive inflation: a residual far beyond sensor noise (a real
        # jump, or lag accrued while smoothly tracking an unannounced push) widens P
        # before the gain is computed, so the correction actually catches up instead
        # of creeping towards the true position over many frames.
        dx2 = (x - self._x) ** 2
        dy2 = (y - self._y) ** 2
        Pxx_eff = self._Pxx + self.cfg.jump_adapt_k * dx2
        Pyy_eff = self._Pyy + self.cfg.jump_adapt_k * dy2

        r = self.cfg.meas_pos_std ** 2
        kx = Pxx_eff / (Pxx_eff + r)
        ky = Pyy_eff / (Pyy_eff + r)
        self._x += kx * (x - self._x)
        self._y += ky * (y - self._y)
        self._Pxx = max((1.0 - kx) * Pxx_eff, self.cfg.bias_floor_mm ** 2)
        self._Pyy = max((1.0 - ky) * Pyy_eff, self.cfg.bias_floor_mm ** 2)
        self._last_fresh_t = t_capture
        self._var_baseline_t = t_capture

    def estimate(self, t: float) -> CubeEstimate:
        if not self._initialized:
            return CubeEstimate(color=self.color, x=self._x, y=self._y,
                                 pos_std=self.cfg.init_pos_std, age_s=math.inf,
                                 alpha=self._alpha, alpha_std=self._alpha_std, delivered=False)
        self._last_known_t = max(self._last_known_t, t)
        age = max(0.0, t - self._last_fresh_t)
        q_rate = self._process_var_rate()
        grow = max(0.0, t - self._var_baseline_t)
        proj_pxx = self._Pxx + q_rate * grow
        proj_pyy = self._Pyy + q_rate * grow
        pos_std = max(math.sqrt(max(proj_pxx, proj_pyy)), self.cfg.bias_floor_mm)
        return CubeEstimate(
            color=self.color,
            x=self._x,
            y=self._y,
            pos_std=pos_std,
            age_s=age,
            alpha=self._alpha,
            alpha_std=self._alpha_std,
            delivered=False,
        )

    def set_pushed(self, flag: bool) -> None:
        if flag == self._pushed:
            return
        # Bake in whatever growth already accrued under the OLD mode up to the best
        # "now" we have, before switching rates -- otherwise the new (possibly much
        # smaller) rate would retroactively apply to the interval that just elapsed.
        self._accumulate_var_to(self._last_known_t)
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
