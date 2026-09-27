"""EKF that tracks one rover's planar pose plus body-frame speed.

State model
-----------
    s = [x, y, theta, v, omega]        (mm, mm, rad, mm/s, rad/s)

    dx/dt     = v * cos(theta)
    dy/dt     = v * sin(theta)
    dtheta/dt = omega
    dv/dt     = (v_cmd - v) / tau + w_v          (w_v: random-walk accel noise)
    domega/dt = (omega_cmd - omega) / tau + w_om  (w_om: random-walk alpha noise)

This is the lead's candidate (unicycle + first-order-lag speed states) and it is
kept after evaluating the alternative.

Why not [x, y, vx, vy, theta, omega] (a free planar rigid body)?
A differential-drive rover cannot slide sideways: the wheel contact constraint pins
the BODY-FRAME lateral velocity to (approximately) zero at all times -- it is not a
quantity that varies and needs its own filter state, it is a constraint. Estimating
world-frame [vx, vy] is, after rotating into the body frame, equivalent to estimating
[v_forward, v_lateral] where v_lateral's true value is always ~0. But nothing in the
process model or in the measurements (vision gives position + heading only, never a
velocity) ties v_lateral back to zero: process noise injects independent uncertainty
into vx and vy every step, so the lateral component is a pure random walk with no
corrective term. Its variance grows unboundedly between vision fixes and, because
theta itself is uncertain, that spurious lateral energy rotates into the x/y estimate
-- an unobservable mode contaminating the very quantity we care about. This is exactly
the "redundant, unobservable-lateral-velocity" problem flagged by the lead. The
unicycle state sidesteps it structurally: v and omega are the ONLY velocity states and
the nonholonomic constraint is baked into the kinematics (dx/dt, dy/dt both scale off
the single scalar v), so there is no lateral-velocity component to mis-estimate.

Bonus, and the reason this also satisfies the "fuse IMU/odometry later without a
redesign" requirement: v and omega already being explicit states means a gyro
(measures omega directly, H = [0,0,0,0,1]) or wheel odometry (measures v and omega via
wheel speeds, H = [[0,0,0,1,0],[0,0,0,0,1]]) are both simple *linear* observations of
existing states -- addable as new update_* methods with no change to the state vector
or the process model.

Numerics
--------
predict() and the vision-replay path share one integrator (`_advance`) that walks the
recorded command history and, within each constant-command segment, sub-steps at
`_MAX_SUBSTEP_S` using a midpoint (explicit RK2) integration of the mean plus its
analytic Jacobian for the covariance (`P' = F P F^T + Q`). Process noise (accel_noise,
alpha_noise) is injected on the v/omega diagonal each sub-step and is carried into
x/y/theta by F over subsequent steps -- the standard, widely-used approximation for
random-walk-driven unicycle EKFs (it skips the exact double integral of noise over the
step, which is negligible at the sub-step sizes used here).

Latency handling: measurements arrive with capture time t_capture <= t_now. A rolling
list of (t, x, P) checkpoints (spanning `cfg.history_s`) lets update_vision rewind to
t_capture, apply the correction there, and replay forward through the recorded command
history to t_now -- rather than smearing a stale correction onto the current state.

Vision feed quirk (frozen frames): when >= 2 corner markers are occluded, the official
feed keeps publishing (increasing seq) but repeats the SAME t_capture and unchanged
object age. Such a repeat carries no new information and must not reset the "age since
last accepted fix" clock or count as an outlier rejection. `update_vision` therefore
compares t_capture against the last *processed* capture time (not just gates it through
Mahalanobis) and drops exact duplicates / anything out of order before doing any work.
"""
from __future__ import annotations

import bisect
import math

import numpy as np

from ..config import EstimatorConfig, TelemetryPolicy
from ..frames import wrap
from ..world import Pose, RoverEstimate, TrackQuality

# TUNED: sub-step ceiling for the mean/covariance integrator. At the fastest planned
# speeds (v_nav=110 mm/s, w_nav=1.4 rad/s) a 20 ms step turns theta by <=0.028 rad
# within the step -- small compared to the per-frame heading noise (1.5 deg = 0.026
# rad) -- so the midpoint method's O(h^3) linearization error stays negligible next to
# sensor noise without needing a stiffer/rk4 integrator.
_MAX_SUBSTEP_S = 0.02

# ASSUMED: velocity-state uncertainty injected on initialize()/re-init, when v/omega
# are not actually known (start-of-match, or after a kidnap re-init from a bare pose
# measurement). Large enough that a few cmd_tau time-constants of predict+update pull
# it down to the steady-state uncertainty quickly, small enough not to blow up the
# Mahalanobis gate on the very next vision fix.
_INIT_V_STD = 80.0           # mm/s
_INIT_OMEGA_STD = 1.5         # rad/s

_H_POSE = np.array([
    [1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0],
])


def _trim_keep_anchor(history: list, cutoff: float) -> list:
    """Keep entries with t >= cutoff, plus the single entry immediately before cutoff
    (so a later rewind to a time right at the edge still has something to start from)."""
    if not history:
        return history
    keep_from = len(history) - 1
    for i, e in enumerate(history):
        if e[0] >= cutoff:
            keep_from = max(0, i - 1)
            break
    return history[keep_from:]


class PoseEstimator:
    """EKF pose tracker for a single rover. Call `initialize` once, then feed
    `set_command` / `update_vision` as they occur and `predict` / `estimate` on demand.
    """

    def __init__(self, cfg: EstimatorConfig, policy: TelemetryPolicy, rover_id: int):
        self.cfg = cfg
        self.policy = policy
        self.rover_id = rover_id
        self._initialized = False
        # exposed counters for the lead to log
        self.consecutive_rejects = 0
        self.reinit_count = 0

    # ------------------------------------------------------------------ lifecycle
    def initialize(self, pose: Pose, t: float) -> None:
        self._x = np.array([pose.x, pose.y, wrap(pose.theta), 0.0, 0.0])
        self._P = np.diag([
            self.cfg.meas_pos_std ** 2, self.cfg.meas_pos_std ** 2,
            self.cfg.meas_heading_std ** 2, _INIT_V_STD ** 2, _INIT_OMEGA_STD ** 2,
        ])
        self._t = t
        self._cmd_hist: list[tuple[float, float, float]] = [(t, 0.0, 0.0)]
        self._checkpoints: list[tuple[float, np.ndarray, np.ndarray]] = [(t, self._x.copy(), self._P.copy())]
        self._last_accepted_t = t          # for age_s (capture-time based)
        self._last_processed_capture_t = t  # dedup guard (capture-time based)
        self._anchor_xy = (pose.x, pose.y)  # position at last accepted fix, for blind-travel
        self.consecutive_rejects = 0
        self._initialized = True

    def set_command(self, t: float, v_cmd: float, w_cmd: float) -> None:
        # Appended, not inserted: callers (a live control loop) issue commands in
        # non-decreasing t order. bisect_right in _cmd_active then resolves same-
        # timestamp duplicates as "last write wins", matching append order.
        self._cmd_hist.append((t, v_cmd, w_cmd))
        cutoff = self._t - self.cfg.history_s
        self._cmd_hist = _trim_keep_anchor(self._cmd_hist, cutoff)

    # ------------------------------------------------------------------ propagation
    def predict(self, t: float) -> None:
        if t <= self._t:
            return
        self._x, self._P = self._advance(self._x, self._P, self._t, t)
        self._t = t
        self._checkpoints.append((t, self._x.copy(), self._P.copy()))
        self._trim_history()

    def _trim_history(self) -> None:
        cutoff = self._t - self.cfg.history_s
        self._checkpoints = _trim_keep_anchor(self._checkpoints, cutoff)
        self._cmd_hist = _trim_keep_anchor(self._cmd_hist, cutoff)

    def _cmd_active(self, t: float) -> tuple[float, float]:
        times = [c[0] for c in self._cmd_hist]
        idx = bisect.bisect_right(times, t) - 1
        if idx < 0:
            return 0.0, 0.0
        _, v_cmd, w_cmd = self._cmd_hist[idx]
        return v_cmd, w_cmd

    def _advance(self, x: np.ndarray, P: np.ndarray, t0: float, t1: float) -> tuple[np.ndarray, np.ndarray]:
        if t1 <= t0:
            return x, P
        x = x.copy()
        P = P.copy()
        breakpoints = [c[0] for c in self._cmd_hist if t0 < c[0] < t1]
        bounds = [t0] + sorted(breakpoints) + [t1]
        for a, b in zip(bounds[:-1], bounds[1:]):
            dur = b - a
            if dur <= 0.0:
                continue
            v_cmd, w_cmd = self._cmd_active(a)
            n = max(1, math.ceil(dur / _MAX_SUBSTEP_S))
            h = dur / n
            for _ in range(n):
                x, P = self._step(x, P, h, v_cmd, w_cmd)
        return x, P

    def _step(self, x: np.ndarray, P: np.ndarray, h: float, v_cmd: float, w_cmd: float) -> tuple[np.ndarray, np.ndarray]:
        xx, yy, th, v, om = x
        tau = self.cfg.cmd_tau
        a = math.exp(-h / tau) if tau > 0 else 0.0

        v1 = v_cmd + (v - v_cmd) * a
        om1 = w_cmd + (om - w_cmd) * a
        v_avg = 0.5 * (v + v1)
        om_avg = 0.5 * (om + om1)
        d_avg_dself = 0.5 * (1.0 + a)     # d(v_avg)/dv == d(om_avg)/domega
        th_mid = th + 0.5 * om_avg * h
        dthmid_domega = 0.25 * h * (1.0 + a)

        c, s = math.cos(th_mid), math.sin(th_mid)
        x1 = xx + v_avg * c * h
        y1 = yy + v_avg * s * h
        th1 = wrap(th + om_avg * h)

        F = np.zeros((5, 5))
        F[0, 0] = 1.0
        F[0, 2] = -v_avg * s * h
        F[0, 3] = d_avg_dself * c * h
        F[0, 4] = -v_avg * s * h * dthmid_domega
        F[1, 1] = 1.0
        F[1, 2] = v_avg * c * h
        F[1, 3] = d_avg_dself * s * h
        F[1, 4] = v_avg * c * h * dthmid_domega
        F[2, 2] = 1.0
        F[2, 4] = h * d_avg_dself
        F[3, 3] = a
        F[4, 4] = a

        # accel_noise/alpha_noise are documented (config.py) with plain acceleration
        # units (mm/s^2, rad/s^2): a std on the unmodeled acceleration/angular-accel
        # acting over the sub-step, not a spectral density. That is the discrete
        # constant-acceleration ("DWPA") process-noise model: Var(delta v) = sigma_a^2 *
        # h^2 (not sigma_a^2 * h, which would silently reinterpret the documented units
        # as a spectral density and inflate the noise by ~1/h -- huge at h=0.02s).
        Q = np.zeros((5, 5))
        Q[3, 3] = self.cfg.accel_noise ** 2 * h * h
        Q[4, 4] = self.cfg.alpha_noise ** 2 * h * h

        x_new = np.array([x1, y1, th1, v1, om1])
        P_new = F @ P @ F.T + Q
        P_new = 0.5 * (P_new + P_new.T)
        return x_new, P_new

    # ------------------------------------------------------------------ vision update
    def update_vision(self, pose_meas: Pose, t_capture: float, t_now: float) -> bool:
        """Rewind to t_capture using stored history, gate + correct there, then replay
        forward to t_now with the recorded commands. Returns True iff the state was
        modified (accepted correction, or a kidnap re-init)."""
        if t_capture <= self._last_processed_capture_t:
            # Duplicate (frozen ts_ms while occluded) or out-of-order/older-than-history.
            # Not new evidence: no reject, no age reset, no state change.
            return False
        if not self._checkpoints or t_capture < self._checkpoints[0][0]:
            # Older than our replay buffer: cannot safely rewind. Ignore.
            return False

        self._last_processed_capture_t = t_capture

        times = [c[0] for c in self._checkpoints]
        idx = bisect.bisect_right(times, t_capture) - 1
        if idx < 0:
            return False
        t0, x0, P0 = self._checkpoints[idx]
        x_tc, P_tc = self._advance(x0, P0, t0, t_capture)

        innov = np.array([
            pose_meas.x - x_tc[0],
            pose_meas.y - x_tc[1],
            wrap(pose_meas.theta - x_tc[2]),
        ])
        R = np.diag([self.cfg.meas_pos_std ** 2, self.cfg.meas_pos_std ** 2, self.cfg.meas_heading_std ** 2])
        S = _H_POSE @ P_tc @ _H_POSE.T + R
        d2 = float(innov @ np.linalg.solve(S, innov))

        reinit = False
        if d2 <= self.cfg.gate_chi2:
            self.consecutive_rejects = 0
        else:
            self.consecutive_rejects += 1
            if self.consecutive_rejects >= self.cfg.max_consecutive_rejects:
                reinit = True
                self.reinit_count += 1
                self.consecutive_rejects = 0
            else:
                return False

        if reinit:
            x_new = np.array([pose_meas.x, pose_meas.y, wrap(pose_meas.theta), 0.0, 0.0])
            P_new = np.diag([
                self.cfg.meas_pos_std ** 2, self.cfg.meas_pos_std ** 2,
                self.cfg.meas_heading_std ** 2, _INIT_V_STD ** 2, _INIT_OMEGA_STD ** 2,
            ])
        else:
            K = P_tc @ _H_POSE.T @ np.linalg.inv(S)
            x_new = x_tc + K @ innov
            x_new[2] = wrap(x_new[2])
            I_KH = np.eye(5) - K @ _H_POSE
            P_new = I_KH @ P_tc @ I_KH.T + K @ R @ K.T
            P_new = 0.5 * (P_new + P_new.T)

        # Discard invalidated future checkpoints, splice in the corrected one, replay to t_now.
        self._checkpoints = [c for c in self._checkpoints if c[0] <= t0] + [(t_capture, x_new.copy(), P_new.copy())]
        x_final, P_final = self._advance(x_new, P_new, t_capture, t_now)
        self._x, self._P, self._t = x_final, P_final, t_now
        self._checkpoints.append((t_now, x_final.copy(), P_final.copy()))
        self._last_accepted_t = t_capture
        self._anchor_xy = (x_new[0], x_new[1])
        self._trim_history()
        return True

    # ------------------------------------------------------------------ future sensors (no redesign needed)
    def update_gyro(self, omega_meas: float, std: float, t: float) -> None:
        """Intended model: direct linear observation of the omega state,
        H = [0, 0, 0, 0, 1], R = std**2. Requires the same rewind/replay treatment as
        update_vision once IMU samples carry a capture time on this filter's clock."""
        raise NotImplementedError

    def update_odometry(self, v_meas: float, omega_meas: float, std_v: float, std_omega: float, t: float) -> None:
        """Intended model: direct linear observation of [v, omega],
        H = [[0,0,0,1,0], [0,0,0,0,1]], R = diag(std_v**2, std_omega**2), with
        v = (v_r + v_l)/2 and omega = (v_r - v_l)/track from wheel encoder speeds."""
        raise NotImplementedError

    # ------------------------------------------------------------------ read-out
    def estimate(self, t: float) -> RoverEstimate:
        if t > self._t:
            self.predict(t)
        age = self._t - self._last_accepted_t
        pos_std = math.sqrt(max(self._P[0, 0], self._P[1, 1]))
        heading_std = math.sqrt(self._P[2, 2])
        blind_travel = float(np.hypot(self._x[0] - self._anchor_xy[0], self._x[1] - self._anchor_xy[1]))

        if (age > self.policy.lost_age_s or pos_std > self.policy.pos_std_stop_mm
                or heading_std > self.policy.heading_std_stop_rad
                or blind_travel > self.policy.max_blind_travel_mm):
            quality = TrackQuality.LOST
        elif age > self.policy.good_age_s:
            quality = TrackQuality.DEGRADED
        else:
            quality = TrackQuality.GOOD

        return RoverEstimate(
            id=self.rover_id,
            pose=Pose(float(self._x[0]), float(self._x[1]), wrap(float(self._x[2]))),
            v=float(self._x[3]),
            omega=float(self._x[4]),
            cov=self._P[:3, :3].copy(),
            age_s=age,
            quality=quality,
        )

    def is_safe_to_drive(self, t: float) -> bool:
        return self.estimate(t).quality != TrackQuality.LOST
