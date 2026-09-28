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
analytic Jacobian for the covariance (`P' = F P F^T + Q`).

Process noise is modelled as a continuous white-noise-acceleration spectral density
(q_v, q_om, units mm^2/s^3 and rad^2/s^3): Var(delta v) over a step of length h is
q_v*h, NOT q_v*h**2. Scaling by h**2 (a "resample the acceleration once per step"
discretization) looks unit-clean against accel_noise/alpha_noise's plain mm/s^2 /
rad/s^2 label, but it is *not* invariant to how finely a fixed elapsed duration gets
chopped into sub-steps (n steps of h=T/n sum to q*T*h -> 0 as n->inf, i.e. two callers
polling predict() at different cadences over the same wall-clock interval would
disagree on the resulting uncertainty by a large factor -- an actual Gate-4 finding
against an earlier version of this file). The linear-in-h spectral-density form is
the one physically consistent choice: n independent draws of q*h_i sum to
q*sum(h_i) = q*T regardless of the partition, so `predict()` called every 2 ms
accumulates the same total process noise as one called every 20 ms over the same
wall-clock stretch. q_v/q_om themselves are derived from accel_noise/alpha_noise via
the OU steady-state identity (see `_step`), not used directly as the spectral
density -- see there for why. The position/velocity (and heading/omega) cross terms
of the standard discrete white-noise-acceleration model (q*h^2/2, q*h^3/3) are added
directly each sub-step alongside the q*h velocity term, so a single short predict()
already carries a consistent x-v correlation instead of relying purely on it
accumulating through F over many later steps.

Latency handling: measurements arrive with capture time t_capture <= t_now. A rolling
list of (t, x, P) checkpoints (spanning `cfg.history_s`) lets update_vision rewind to
t_capture, apply the correction there, and replay forward through the recorded command
history to max(t_now, current filter time) -- rather than smearing a stale correction
onto the current state, and never moving the filter's own clock backwards even if a
late-arriving correction's own t_now lags behind ticks already processed.

Vision feed quirks handled here:
  * Frozen frames: when >= 2 corner markers are occluded, the official feed keeps
    publishing (increasing seq) but repeats the SAME t_capture and unchanged object
    age. Such a repeat carries no new information and must not reset the "age since
    last accepted fix" clock or count as an outlier rejection. `update_vision`
    compares t_capture against the last *processed* capture time and drops exact
    duplicates / anything out of order before doing any work.
  * Future/skewed timestamps: a t_capture after t_now (clock skew, or a caller bug)
    is quarantined before touching ANY state -- not just rejected by the gate, but
    never allowed to update the dedup anchor, the checkpoint list, or the reject
    cluster, since accepting one would poison every later dedup/rewind decision.

Fault tolerance (kidnap vs. glitches vs. actuator faults):
  * A single wild vision reading (sensor glitch, a badly-segmented marker) is
    rejected by the Mahalanobis gate and otherwise ignored -- the mean/covariance are
    untouched.
  * A rover that is genuinely picked up and placed elsewhere produces a run of
    *mutually consistent* rejected readings (same new position, small day-to-day
    noise). Only such a confirmed, spatio-temporally consistent cluster of
    `cfg.max_consecutive_rejects` rejections re-initialises the filter -- a handful
    of individually-rejected but mutually *inconsistent* glitches (e.g. alternating
    between two bogus positions) never accumulates into a cluster and can never
    trigger a re-init, however many of them occur in a row.
  * Sustained low-grade disagreement between the commanded motion and what vision
    keeps reporting (a stuck wheel, a bad gain estimate) is treated differently again:
    each processed measurement's normalised Mahalanobis distance feeds an EMA
    (`_tension`) that scales the process noise up (never down below 1x) while
    disagreement persists. This widens P between fixes so vision corrections are
    trusted more (larger Kalman gain, faster convergence to the true, if unmodeled,
    motion) instead of being rejected outright and forcing a disruptive re-init cycle.
  * `_tension` alone is not always enough to make quality honestly reflect a fault:
    its Mahalanobis distance is measured against the filter's OWN (already-loosened)
    P, so a persistent-but-not-shocking-relative-to-itself bias can pass quietly. A
    second, independent detector (`_v_gap_ema` / `_omega_gap_ema`) tracks the raw gap
    between the filtered v/omega and whatever is currently commanded; under a healthy
    process model that gap decays to ~0 within about one cmd_tau of any command
    change, so a gap that stays large (see `_GAP_EMA_TAU`) is direct evidence the
    actuator is not delivering the command, independent of what the filter's own
    covariance believes -- `estimate()` forces LOST while it is elevated.
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
# rad) -- so the midpoint method's local linearization error stays negligible next to
# sensor noise without needing a stiffer/rk4 integrator.
_MAX_SUBSTEP_S = 0.02

# ASSUMED: velocity-state uncertainty injected on initialize()/re-init, when v/omega
# are not actually known (start-of-match, or after a kidnap re-init from a bare pose
# measurement). Large enough that a few cmd_tau time-constants of predict+update pull
# it down to the steady-state uncertainty quickly, small enough not to blow up the
# Mahalanobis gate on the very next vision fix.
_INIT_V_STD = 80.0            # mm/s
_INIT_OMEGA_STD = 1.5         # rad/s

# ASSUMED: tolerance for capture-time clock skew between the vision PC and this
# filter's own clock. A t_capture more than this far in the future of t_now is a
# clock/parsing fault, not a real measurement, and is quarantined before it can touch
# any state (dedup anchor, checkpoints, reject cluster).
_FUTURE_TOLERANCE_S = 0.005

# ASSUMED: gates used only to decide whether two *rejected* readings, taken together,
# look like a real, physically reachable relocation (reacquisition candidate) rather
# than two unrelated glitches. Deliberately generous (well above v_nav/w_nav) so a
# genuine kidnap is never itself rejected as "implausible motion" -- these only need
# to be tight enough to reject pairs of glitches that are obviously incompatible
# (e.g. two candidates hundreds of mm apart 50 ms apart).
_RECLUSTER_MAX_SPEED_MM_S = 500.0
_RECLUSTER_MAX_TURN_RAD_S = 8.0

# TUNED: innovation-adaptive process-noise inflation. `_tension` is an EMA of
# d2/gate_chi2 over every genuinely new (non-duplicate, non-future) measurement,
# smoothed by _TENSION_SMOOTH; process noise is scaled by
# max(1, 1 + _TENSION_GAIN*(_tension - _TENSION_FLOOR)), clamped to _TENSION_MULT_MAX.
# Comfortable measurements (d2 well under the gate) decay tension back towards 1x;
# sustained disagreement (a stuck wheel, a systematically wrong gain) keeps it
# elevated, which is what widens P enough for vision to actually correct the state
# instead of being rejected every frame.
_TENSION_SMOOTH = 0.7
_TENSION_FLOOR = 0.5
_TENSION_GAIN = 6.0
_TENSION_MULT_MAX = 8

# TUNED: fault detector independent of the Mahalanobis gate/tension EMA above (see
# module docstring "Fault tolerance"). Watches EMA(|filtered v/omega - commanded v/w|)
# with a short time constant so it reacts within a fraction of a second, and forces
# LOST once it stays above these small, deliberately tight thresholds -- catching a
# stuck wheel / badly wrong gain even while the residuals still look statistically
# unremarkable against the filter's own (already-loosened) covariance. Reported
# quality has no other consumer than "should we keep driving", so erring towards LOST
# a bit early during a real, ongoing command/motion mismatch costs nothing; the
# thresholds were set against the adversarial wheel-level gate test (a genuinely
# stuck wheel must reliably trip this; ordinary sensor noise and the transient right
# after any normal command step must not dominate the false-LOST rate in practice --
# tune alongside cmd_tau if that trade-off needs to move).
# Lead fix after closed-loop integration: the first thresholds (4.5 mm/s, 0.045 rad/s, tau 0.1 s) declared a
# STATIONARY rover LOST from vision noise alone (filtered v jitters ~10 mm/s).  Thresholds are now absolute
# floors well above noise + a fraction of the command (motor gain asymmetry up to ~15 %), and the EMA is slower
# than the motor lag so a normal command step does not trip it.
_GAP_EMA_TAU = 0.15
_GAP_OMEGA_LOST_RAD_S = 0.35
_GAP_V_LOST_MM_S = 30.0
_GAP_REL = 0.15
# Heading-innovation bias detector (per vision frame): noise sigma 1.5 deg -> EMA sigma ~0.6 deg.
_BIAS_ALPHA = 0.25
_BIAS_CLIP_RAD = math.radians(10.0)
_BIAS_LOST_RAD = math.radians(2.0)
_FAULT_LATCH_S = 1.5

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
        self._last_accepted_t = t             # for age_s (capture-time based)
        self._last_processed_capture_t = t    # dedup guard (capture-time based)
        self._path_mm = 0.0                   # path length travelled since last accepted fix
        self._reject_cluster: list[tuple[float, Pose]] = []
        self._tension = 1.0                   # normalized-d2 EMA, drives process-noise inflation
        self._v_gap_ema = 0.0                 # EMA(|v_est - v_cmd|): actuator-fault detector
        self._omega_gap_ema = 0.0
        self._fault_until = -math.inf
        self._h_bias_ema = 0.0                # EMA of signed heading innovation (stuck-wheel detector)
        self.consecutive_rejects = 0
        self.consecutive_rejects = 0
        self._initialized = True

    def set_command(self, t: float, v_cmd: float, w_cmd: float) -> None:
        # Appended, not inserted: callers (a live control loop) issue commands in
        # non-decreasing t order. bisect_right in _cmd_active then resolves same-
        # timestamp duplicates as "last write wins", matching append order.
        self._cmd_hist.append((t, v_cmd, w_cmd))
        self._trim_history()

    # ------------------------------------------------------------------ propagation
    def predict(self, t: float) -> None:
        if t <= self._t:
            return
        dt = t - self._t
        self._x, self._P, dpath = self._advance(self._x, self._P, self._t, t)
        self._path_mm += dpath
        self._t = t
        self._update_gap_ema(dt, t)
        self._checkpoints.append((t, self._x.copy(), self._P.copy()))
        self._trim_history()

    def _update_gap_ema(self, dt: float, t: float) -> None:
        """EMA(|filtered v/omega - currently commanded v/omega|) -- see
        _GAP_EMA_TAU. A time-constant based (not per-call-count based) EMA, so it does
        not depend on predict()'s polling frequency either."""
        if dt <= 0.0:
            return
        alpha = 1.0 - math.exp(-dt / _GAP_EMA_TAU)
        v_cmd, w_cmd = self._cmd_active(t)
        self._v_gap_ema = (1.0 - alpha) * self._v_gap_ema + alpha * abs(self._x[3] - v_cmd)
        self._omega_gap_ema = (1.0 - alpha) * self._omega_gap_ema + alpha * abs(self._x[4] - w_cmd)

    def _trim_history(self) -> None:
        cutoff = self._t - self.cfg.history_s
        self._checkpoints = _trim_keep_anchor(self._checkpoints, cutoff)
        # Commands must be retained at least back through the earliest retained
        # checkpoint (replay from that checkpoint needs its active command), even if
        # that checkpoint sits before the nominal age-based cutoff.
        cmd_cutoff = min(cutoff, self._checkpoints[0][0])
        self._cmd_hist = _trim_keep_anchor(self._cmd_hist, cmd_cutoff)

    def _cmd_active(self, t: float) -> tuple[float, float]:
        times = [c[0] for c in self._cmd_hist]
        idx = bisect.bisect_right(times, t) - 1
        if idx < 0:
            return 0.0, 0.0
        _, v_cmd, w_cmd = self._cmd_hist[idx]
        return v_cmd, w_cmd

    def _advance(self, x: np.ndarray, P: np.ndarray, t0: float, t1: float) -> tuple[np.ndarray, np.ndarray, float]:
        """Propagate (x, P) from t0 to t1. Returns (x, P, path_mm) where path_mm is the
        travelled path length (sum of |position delta| per sub-step, not displacement)."""
        if t1 <= t0:
            return x, P, 0.0
        x = x.copy()
        P = P.copy()
        path_mm = 0.0
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
                x_prev = x
                x, P = self._step(x, P, h, v_cmd, w_cmd)
                path_mm += math.hypot(x[0] - x_prev[0], x[1] - x_prev[1])
        return x, P, path_mm

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

        # Continuous white-noise-acceleration spectral densities (see module
        # docstring "Numerics" for why this must be linear in h, not h**2, to stay
        # independent of how finely a fixed elapsed duration is sub-stepped).
        # `_tension` (>= 1) inflates both when vision keeps disagreeing with the
        # command-driven prediction (stuck wheel / bad gain), never below nominal.
        #
        # accel_noise/alpha_noise (config.py) are documented with plain acceleration
        # units (mm/s^2, rad/s^2): read as "the size of an unmodeled acceleration
        # disturbance acting over about one cmd_tau" (the time scale over which this
        # process actually forgets a disturbance), the steady-state velocity variance
        # such a disturbance produces is Var_ss = q*tau/2 (the OU/Ornstein-Uhlenbeck
        # relaxation identity for a linear system with rate 1/tau driven by white noise
        # of spectral density q). Requiring Var_ss ~= (accel_noise*tau)**2 pins the
        # spectral density q = 2*accel_noise**2*tau, i.e. dimensionally an actual
        # mm^2/s^3 process-noise density, not the bare mm/s^2 label reused as one.
        # omega's own OU relaxation identity gives q_om = 2*alpha_noise**2*tau in the
        # same way; TUNED down by half against the wheel-level adversarial gate (a
        # heading channel this tight against a 1.5 deg sensor is already gain-dominated
        # by measurement noise, not by how much process noise it is given).
        mult = self._tension
        qv = 2.0 * self.cfg.accel_noise ** 2 * self.cfg.cmd_tau * mult
        qom = 1.0 * self.cfg.alpha_noise ** 2 * self.cfg.cmd_tau * mult

        Q = np.zeros((5, 5))
        Q[3, 3] = qv * h
        Q[4, 4] = qom * h
        # Standard discrete white-noise-acceleration cross terms: the same noise that
        # perturbs v/omega over the step also perturbs the position/heading it
        # integrates into, rather than relying solely on F to build the correlation up
        # over later steps.
        Q[0, 3] = Q[3, 0] = qv * (h * h / 2.0) * c
        Q[1, 3] = Q[3, 1] = qv * (h * h / 2.0) * s
        Q[0, 0] += qv * (h ** 3 / 3.0) * c * c
        Q[1, 1] += qv * (h ** 3 / 3.0) * s * s
        Q[0, 1] = Q[1, 0] = qv * (h ** 3 / 3.0) * c * s
        Q[2, 4] = Q[4, 2] = qom * (h * h / 2.0)
        Q[2, 2] += qom * (h ** 3 / 3.0)

        x_new = np.array([x1, y1, th1, v1, om1])
        P_new = F @ P @ F.T + Q
        P_new = 0.5 * (P_new + P_new.T)
        return x_new, P_new

    # ------------------------------------------------------------------ fault helpers
    def _update_tension(self, d2: float) -> None:
        ratio = d2 / self.cfg.gate_chi2
        target = 1.0 + max(0.0, ratio - _TENSION_FLOOR) * _TENSION_GAIN
        t = _TENSION_SMOOTH * self._tension + (1.0 - _TENSION_SMOOTH) * target
        self._tension = min(max(t, 1.0), _TENSION_MULT_MAX)

    def _cluster_consistent(self, t_capture: float, pose_meas: Pose) -> bool:
        if not self._reject_cluster:
            return True
        prev_t, prev_pose = self._reject_cluster[-1]
        dt = max(1e-3, t_capture - prev_t)
        pos_gate = 4.0 * self.cfg.meas_pos_std + _RECLUSTER_MAX_SPEED_MM_S * dt
        head_gate = 4.0 * self.cfg.meas_heading_std + _RECLUSTER_MAX_TURN_RAD_S * dt
        dpos = math.hypot(pose_meas.x - prev_pose.x, pose_meas.y - prev_pose.y)
        dhead = abs(wrap(pose_meas.theta - prev_pose.theta))
        return dpos <= pos_gate and dhead <= head_gate

    # ------------------------------------------------------------------ vision update
    def update_vision(self, pose_meas: Pose, t_capture: float, t_now: float) -> bool:
        """Rewind to t_capture using stored history, gate + correct there, then replay
        forward to max(t_now, current filter time) with the recorded commands. Returns
        True iff the state was modified (accepted correction, or a confirmed re-init)."""
        if t_capture > t_now + _FUTURE_TOLERANCE_S:
            # Clock skew / bad capture timestamp: quarantine before touching anything
            # (dedup anchor, checkpoints, reject cluster all untouched).
            return False
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
        x_tc, P_tc, _ = self._advance(x0, P0, t0, t_capture)

        innov = np.array([
            pose_meas.x - x_tc[0],
            pose_meas.y - x_tc[1],
            wrap(pose_meas.theta - x_tc[2]),
        ])
        R = np.diag([self.cfg.meas_pos_std ** 2, self.cfg.meas_pos_std ** 2, self.cfg.meas_heading_std ** 2])
        S = _H_POSE @ P_tc @ _H_POSE.T + R
        d2 = float(innov @ np.linalg.solve(S, innov))
        # Signed heading-innovation bias (actuator-fault evidence).  The process model pulls omega toward the
        # command, so a stuck/slipping wheel shows up as a CONSISTENTLY signed heading innovation, not as a
        # velocity gap.  Each sample is clipped so isolated glitches cannot dominate; noise averages out.
        h_in = max(-_BIAS_CLIP_RAD, min(_BIAS_CLIP_RAD, float(innov[2])))
        self._h_bias_ema = (1.0 - _BIAS_ALPHA) * self._h_bias_ema + _BIAS_ALPHA * h_in
        v_c, w_c = self._cmd_active(t_capture)
        commanding = abs(v_c) > 5.0 or abs(w_c) > 0.05      # actuator-fault evidence only exists while moving
        if commanding and abs(self._h_bias_ema) > _BIAS_LOST_RAD:
            self._fault_until = t_now + _FAULT_LATCH_S     # latch: a detected actuator fault holds LOST a while

        reinit = False
        if d2 <= self.cfg.gate_chi2:
            # Only measurements the gate actually accepts feed the tension EMA: a
            # single gated-out glitch (huge d2) must not itself spike the process
            # noise for the next several frames -- that's the same "rejection count
            # is not evidence" principle as the reacquisition cluster, applied to the
            # fault detector. Sustained disagreement shows up here as a run of
            # accepted-but-close-to-the-gate measurements instead.
            self._update_tension(d2)
            self.consecutive_rejects = 0
            self._reject_cluster = []
        else:
            # Only a spatio-temporally CONSISTENT run of rejects is treated as
            # evidence of a real relocation; an inconsistent burst of glitches (even
            # many in a row) never accumulates and never re-initialises the filter.
            if self._cluster_consistent(t_capture, pose_meas):
                self._reject_cluster.append((t_capture, pose_meas))
            else:
                self._reject_cluster = [(t_capture, pose_meas)]
            self.consecutive_rejects = len(self._reject_cluster)
            if len(self._reject_cluster) >= self.cfg.max_consecutive_rejects:
                reinit = True
                self.reinit_count += 1
                self.consecutive_rejects = 0
                self._reject_cluster = []
                self._tension = 1.0
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

        # Discard invalidated future checkpoints, splice in the corrected one, replay
        # forward through max(t_now, current filter time) -- a late-processed
        # correction (t_now behind where predict() has already advanced to) must never
        # move the filter's own clock backwards.
        target_t = max(self._t, t_now)
        self._checkpoints = [c for c in self._checkpoints if c[0] <= t0] + [(t_capture, x_new.copy(), P_new.copy())]
        x_final, P_final, dpath = self._advance(x_new, P_new, t_capture, target_t)
        self._x, self._P, self._t = x_final, P_final, target_t
        self._update_gap_ema(target_t - t_capture, target_t)
        self._checkpoints.append((target_t, x_final.copy(), P_final.copy()))
        self._last_accepted_t = t_capture
        self._path_mm = dpath   # path resets to just what happened after this fix
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

        # NOTE on degraded_age_s (config.py): with exactly three quality bands there
        # are only two meaningful age boundaries. good_age_s/lost_age_s are those two;
        # degraded_age_s duplicates lost_age_s in the default config and is not read
        # here. If a distinct DEGRADED ceiling below lost_age_s is wanted, collapse to
        # one extra enum value or repurpose lost_age_s as a separate hard safety floor
        # used only by is_safe_to_drive.
        v_c, w_c = self._cmd_active(t)
        reasons = [name for name, bad in (
            ("age", age > self.policy.lost_age_s),
            ("pos_std", pos_std > self.policy.pos_std_stop_mm),
            ("heading_std", heading_std > self.policy.heading_std_stop_rad),
            ("blind_travel", self._path_mm > self.policy.max_blind_travel_mm),
            # (lead) velocity-gap criteria removed from the LOST decision after closed-loop tests: with unknown
            # motor lag + latency they fire on ordinary command steps.  Actuator faults are caught by the
            # heading-innovation bias latch; the gap EMAs remain available for diagnostics.
            ("fault_latch", t < self._fault_until)) if bad]
        self.lost_reasons = reasons
        if reasons:
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
