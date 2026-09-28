"""Tests for PoseEstimator and CubeTracker.

Truth simulator: unicycle with the same first-order-lag speed response the filter
assumes. This validates the EKF *mechanics* (propagation, gating, replay, wrapping,
quality/blind-driving logic) rather than robustness to process-model mismatch, which
is out of scope for this module.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.frames import wrap, angle_diff
from rover_strategy.world import Pose, TrackQuality
from rover_strategy.estimation.pose_estimator import PoseEstimator
from rover_strategy.estimation.cube_tracker import CubeTracker, CubeTrackerConfig

TAU = C.estimator.cmd_tau
POS_STD = C.estimator.meas_pos_std          # 2.0 mm
HEAD_STD = C.estimator.meas_heading_std     # ~1.5 deg


class Truth:
    """Unicycle-with-lag ground truth, integrated at a fine fixed sub-step."""

    SIM_DT = 0.002

    def __init__(self):
        self.x = self.y = self.theta = self.v = self.omega = 0.0

    def step(self, dt: float, v_cmd: float, w_cmd: float) -> None:
        n = max(1, round(dt / self.SIM_DT))
        h = dt / n
        for _ in range(n):
            a = math.exp(-h / TAU)
            v1 = v_cmd + (self.v - v_cmd) * a
            w1 = w_cmd + (self.omega - w_cmd) * a
            v_avg = 0.5 * (self.v + v1)
            w_avg = 0.5 * (self.omega + w1)
            th_mid = self.theta + 0.5 * w_avg * h
            self.x += v_avg * math.cos(th_mid) * h
            self.y += v_avg * math.sin(th_mid) * h
            self.theta = wrap(self.theta + w_avg * h)
            self.v, self.omega = v1, w1


def make_est(rover_id=0, pose=Pose(0.0, 0.0, 0.0), t0=0.0):
    est = PoseEstimator(C.estimator, C.telemetry, rover_id)
    est.initialize(pose, t0)
    return est


def make_frames(rng, duration, cmd_profile, *, frame_hz=20.0, drop_prob=0.0,
                 latency_range=(0.03, 0.3), pos_std=POS_STD, heading_std=HEAD_STD,
                 outlier_ts=frozenset(), control_dt=0.01):
    """Simulate truth + a noisy, delayed, occasionally-dropped vision feed.

    Returns (frames, truth_log). frames: list of dict(capture_t, deliver_t, pose).
    truth_log: list of (t, x, y, theta) sampled every control tick.
    """
    truth = Truth()
    frame_dt = 1.0 / frame_hz
    t = 0.0
    next_frame_t = 0.0
    frames = []
    truth_log = [(0.0, 0.0, 0.0, 0.0)]
    n_ticks = round(duration / control_dt)
    for _ in range(n_ticks):
        v_cmd, w_cmd = cmd_profile(t)
        truth.step(control_dt, v_cmd, w_cmd)
        t = round(t + control_dt, 6)
        truth_log.append((t, truth.x, truth.y, truth.theta))
        while next_frame_t <= t + 1e-9:
            capture_t = next_frame_t
            if rng.random() >= drop_prob:
                nx = truth.x + rng.normal(0, pos_std)
                ny = truth.y + rng.normal(0, pos_std)
                nth = wrap(truth.theta + rng.normal(0, heading_std))
                if capture_t in outlier_ts:
                    ang = rng.uniform(0, 2 * math.pi)
                    nx += 300.0 * math.cos(ang)
                    ny += 300.0 * math.sin(ang)
                lat = rng.uniform(*latency_range)
                frames.append(dict(capture_t=capture_t, deliver_t=round(capture_t + lat, 6), pose=Pose(nx, ny, nth)))
            next_frame_t = round(next_frame_t + frame_dt, 6)
    frames.sort(key=lambda f: f["deliver_t"])
    return frames, truth_log


def drive(est, frames, cmd_profile, duration, control_dt=0.01, naive=False):
    """Feed cmd_profile + frames through est, return list of (t, RoverEstimate)."""
    fi = 0
    t = 0.0
    last_cmd = None
    log = []
    n_ticks = round(duration / control_dt)
    for _ in range(n_ticks):
        v_cmd, w_cmd = cmd_profile(t)
        if (v_cmd, w_cmd) != last_cmd:
            est.set_command(t, v_cmd, w_cmd)
            last_cmd = (v_cmd, w_cmd)
        t = round(t + control_dt, 6)
        est.predict(t)
        while fi < len(frames) and frames[fi]["deliver_t"] <= t + 1e-9:
            f = frames[fi]
            t_cap = t if naive else f["capture_t"]
            est.update_vision(f["pose"], t_cap, t)
            fi += 1
        log.append((t, est.estimate(t)))
    return log


def truth_at(truth_log, t):
    times = [r[0] for r in truth_log]
    idx = min(range(len(times)), key=lambda i: abs(times[i] - t))
    return truth_log[idx]


def rmse_pos(log, truth_log):
    errs = []
    for t, e in log:
        _, tx, ty, _ = truth_at(truth_log, t)
        errs.append((e.pose.x - tx) ** 2 + (e.pose.y - ty) ** 2)
    return math.sqrt(np.mean(errs))


def rmse_heading(log, truth_log):
    errs = []
    for t, e in log:
        _, _, _, tth = truth_at(truth_log, t)
        errs.append(angle_diff(e.pose.theta, tth) ** 2)
    return math.sqrt(np.mean(errs))


# ---------------------------------------------------------------------- trajectories

def test_straight_line_rmse_and_consistency():
    rng = np.random.default_rng(1)
    profile = lambda t: (80.0, 0.0)
    frames, truth_log = make_frames(rng, 5.0, profile, drop_prob=0.0)
    est = make_est()
    log = drive(est, frames, profile, 5.0)

    # 20 Hz vision at 2 mm sigma fused over ~1 s of history should beat a single raw
    # measurement by a solid margin; 3 mm is a generous cap (empirically ~2.8 mm here,
    # dominated by the startup transient before v settles from 0 to the command) that
    # would fail if the update/replay math were broken (e.g. innovation sign flipped,
    # wrong gain). Gate-4 audit findings pushed the process-noise model towards
    # honestly wider (not narrower) uncertainty during transients, which loosened this
    # from an earlier, over-tight 2 mm bound.
    assert rmse_pos(log, truth_log) < 3.0
    assert rmse_heading(log, truth_log) < math.radians(1.5)

    # NEES (3 dof): chi2 mean is 3. Average over the whole run should land in a wide
    # but meaningful band -- this is what would catch an inconsistent (over/under-
    # confident) covariance, e.g. a missing Q term or a Joseph-form bug.
    nees = []
    for t, e in log:
        _, tx, ty, tth = truth_at(truth_log, t)
        err = np.array([e.pose.x - tx, e.pose.y - ty, angle_diff(e.pose.theta, tth)])
        nees.append(err @ np.linalg.solve(e.cov, err))
    assert 0.5 < np.mean(nees) < 9.0


def test_turning_trajectory_rmse():
    rng = np.random.default_rng(2)
    profile = lambda t: (60.0, 0.8 * math.sin(0.5 * t))
    frames, truth_log = make_frames(rng, 6.0, profile)
    est = make_est()
    log = drive(est, frames, profile, 6.0)
    assert rmse_pos(log, truth_log) < 3.0
    assert rmse_heading(log, truth_log) < math.radians(2.5)


def test_heading_wraparound_no_jump():
    rng = np.random.default_rng(3)
    # Spin fast enough to cross +-pi repeatedly within a few seconds.
    profile = lambda t: (0.0, 2.5)
    frames, truth_log = make_frames(rng, 4.0, profile)
    est = make_est()
    log = drive(est, frames, profile, 4.0)
    thetas = [e.pose.theta for _, e in log]
    for a, b in zip(thetas, thetas[1:]):
        assert abs(angle_diff(b, a)) < 0.3   # no representation jump between consecutive ticks
    assert rmse_heading(log, truth_log) < math.radians(3.0)


# ---------------------------------------------------------------------- latency

def test_replay_beats_naive_apply_under_latency():
    rng = np.random.default_rng(4)
    profile = lambda t: (70.0, 0.9)   # turning -> position moves meaningfully during latency
    frames, truth_log = make_frames(rng, 6.0, profile, latency_range=(0.1, 0.3))

    est_replay = make_est()
    log_replay = drive(est_replay, frames, profile, 6.0, naive=False)

    est_naive = make_est()
    log_naive = drive(est_naive, frames, profile, 6.0, naive=True)

    assert rmse_pos(log_replay, truth_log) < 0.7 * rmse_pos(log_naive, truth_log)


# ---------------------------------------------------------------------- dropped frames / quality

def test_dropped_frames_degrade_then_lost_then_recover():
    rng = np.random.default_rng(5)
    profile = lambda t: (50.0, 0.0)
    est = make_est()
    last_cmd = None
    t = 0.0
    control_dt = 0.01
    est.set_command(0.0, 50.0, 0.0)

    def send_vision(t_now):
        est.update_vision(Pose(50.0 * t_now, 0.0, 0.0), t_now, t_now)

    # Feed good frames for 1s -> should be GOOD.
    for i in range(100):
        t = round(t + control_dt, 6)
        est.predict(t)
        if i % 2 == 0:
            send_vision(t)
    assert est.estimate(t).quality == TrackQuality.GOOD
    assert est.is_safe_to_drive(t)

    # Now stop all vision for a long stretch -> DEGRADED then LOST.
    saw_degraded = False
    for _ in range(150):
        t = round(t + control_dt, 6)
        est.predict(t)
        q = est.estimate(t).quality
        if q == TrackQuality.DEGRADED:
            saw_degraded = True
        if q == TrackQuality.LOST:
            break
    assert saw_degraded
    assert est.estimate(t).quality == TrackQuality.LOST
    assert not est.is_safe_to_drive(t)

    # Frames resume -> should recover to GOOD.
    for i in range(60):
        t = round(t + control_dt, 6)
        est.predict(t)
        send_vision(t)
    assert est.estimate(t).quality == TrackQuality.GOOD
    assert est.is_safe_to_drive(t)


def test_frozen_ts_ms_duplicate_does_not_reset_age_and_goes_lost():
    """Lead's audit finding: >=2 markers occluded => feed repeats the SAME t_capture
    forever. Repeated calls with an unchanged t_capture must not be treated as new
    evidence: quality must still degrade to LOST and stay LOST until t_capture moves."""
    est = make_est()
    est.set_command(0.0, 40.0, 0.0)
    t = 0.0
    control_dt = 0.01
    frozen_capture_t = 0.5
    frozen_pose = Pose(20.0, 0.0, 0.0)

    accepted_after_freeze = 0
    for _ in range(200):
        t = round(t + control_dt, 6)
        est.predict(t)
        ok = est.update_vision(frozen_pose, frozen_capture_t, t)
        if ok:
            accepted_after_freeze += 1

    # The first delivery (t_capture=0.5 arriving once t has passed it) may or may not
    # be accepted depending on gate/geometry, but subsequent identical repeats of the
    # SAME t_capture must all be rejected as duplicates -> at most one accept total.
    assert accepted_after_freeze <= 1
    assert est.consecutive_rejects == 0   # duplicates must never count as rejects
    assert est.estimate(t).quality == TrackQuality.LOST
    assert not est.is_safe_to_drive(t)


# ---------------------------------------------------------------------- outliers / kidnap

def test_sudden_outlier_rejected():
    rng = np.random.default_rng(6)
    profile = lambda t: (50.0, 0.0)
    frames, truth_log = make_frames(rng, 3.0, profile, outlier_ts={1.0})
    est = make_est()
    log = drive(est, frames, profile, 3.0)
    # The outlier at t=1.0 must not have derailed the estimate: RMSE stays tight
    # (dominated by the startup transient, same 3 mm bound as the straight-line test).
    assert rmse_pos(log, truth_log) < 3.0
    assert est.reinit_count == 0
    assert est.consecutive_rejects == 0   # good frames after the outlier reset the streak


def test_genuine_kidnap_triggers_reinit_after_n_rejects():
    est = make_est()
    est.set_command(0.0, 0.0, 0.0)
    t = 0.0
    control_dt = 0.05
    kidnapped_pose = Pose(500.0, 500.0, 1.0)
    accepted_t = None
    for i in range(10):
        t = round(t + control_dt, 6)
        est.predict(t)
        ok = est.update_vision(kidnapped_pose, t, t)
        if ok:
            accepted_t = i
            break
    assert accepted_t is not None
    assert est.reinit_count == 1
    e = est.estimate(t)
    assert math.hypot(e.pose.x - 500.0, e.pose.y - 500.0) < 5.0
    assert abs(angle_diff(e.pose.theta, 1.0)) < 0.1


# ---------------------------------------------------------------------- varying dt / out-of-order

def test_varying_dt_predict():
    est = make_est()
    est.set_command(0.0, 100.0, 0.5)
    dts = [0.003, 0.2, 0.01, 0.5, 0.001, 0.08]
    t = 0.0
    for dt in dts:
        t += dt
        est.predict(t)
    e = est.estimate(t)
    # Should match a fine-grained truth run to good precision (same process model).
    truth = Truth()
    truth.step(t, 100.0, 0.5)
    assert math.hypot(e.pose.x - truth.x, e.pose.y - truth.y) < 1.0
    assert abs(angle_diff(e.pose.theta, truth.theta)) < 0.02


def test_out_of_order_measurement_ignored_safely():
    est = make_est()
    est.set_command(0.0, 60.0, 0.0)
    t = 0.0
    for _ in range(20):
        t = round(t + 0.02, 6)
        est.predict(t)
    ok1 = est.update_vision(Pose(60.0 * t, 0.0, 0.0), t, t)
    assert ok1
    before = (est._x.copy(), est._P.copy())
    # A measurement captured well before anything we've retained -> must be ignored,
    # not crash, not corrupt state.
    stale_t = t - 5 * C.estimator.history_s
    ok2 = est.update_vision(Pose(0.0, 0.0, 0.0), stale_t, t)
    assert not ok2
    assert np.allclose(before[0], est._x)
    assert np.allclose(before[1], est._P)

    # A duplicate of the exact same capture time as the last processed one is ignored.
    ok3 = est.update_vision(Pose(9999.0, 9999.0, 2.0), t, t)
    assert not ok3
    assert np.allclose(before[0], est._x)


# ---------------------------------------------------------------------- cube tracker

def test_cube_occlusion_semantics_age_and_std_floor():
    rng = np.random.default_rng(7)
    cfg = CubeTrackerConfig()
    ct = CubeTracker("red", cfg)
    t = 0.0
    true_x, true_y = 400.0, 300.0
    stds = []
    for i in range(40):
        t = round(t + 0.05, 6)
        nx = true_x + rng.normal(0, cfg.meas_pos_std)
        ny = true_y + rng.normal(0, cfg.meas_pos_std)
        ct.update(nx, ny, t, 0.0)
        stds.append(ct.estimate(t).pos_std)

    # std shrinks with more fresh frames but floors at bias_floor, never below it.
    assert stds[0] > stds[-1] >= cfg.bias_floor_mm - 1e-9
    assert ct.estimate(t).age_s < 1e-6

    # Occlusion: feed repeats last known position with growing age_ms -> not new evidence.
    last_estimate = ct.estimate(t)
    for k in range(20):
        t = round(t + 0.05, 6)
        ct.update(true_x, true_y, t, age_s=0.05 * (k + 1))
    occluded = ct.estimate(t)
    assert occluded.x == last_estimate.x and occluded.y == last_estimate.y
    # uncertainty must GROW while stale, driven by query time (not the frozen feed).
    assert occluded.pos_std > last_estimate.pos_std
    assert occluded.age_s > 0.9


def test_cube_frozen_duplicate_capture_ignored():
    ct = CubeTracker("blue")
    ct.update(10.0, 20.0, 0.0, 0.0)
    ct.update(10.0, 20.0, 0.1, 0.0)
    # Same t_capture repeated (>=2 markers occluded bug): must not be treated as fresh.
    for _ in range(5):
        ct.update(999.0, 999.0, 0.1, 0.0)
    e = ct.estimate(0.1)
    assert e.x == 10.0 and e.y == 20.0


def test_cube_pushed_faster_uncertainty_growth_when_unseen():
    cfg = CubeTrackerConfig()
    a = CubeTracker("green", cfg)
    b = CubeTracker("green", cfg)
    a.update(0.0, 0.0, 0.0, 0.0)
    b.update(0.0, 0.0, 0.0, 0.0)
    a.set_pushed(False)
    b.set_pushed(True)
    ea = a.estimate(2.0)
    eb = b.estimate(2.0)
    assert eb.pos_std > ea.pos_std


def test_cube_unexpected_move_detection_and_orientation_reset():
    ct = CubeTracker("red")
    ct.update(0.0, 0.0, 0.0, 0.0)
    ct.set_orientation(math.radians(10), 0.05)
    assert ct.moved_unexpectedly() is False

    # Not pushed, big jump -> flagged, orientation reset to unknown.
    ct.update(200.0, 0.0, 0.1, 0.0)
    assert ct.moved_unexpectedly() is True
    assert ct.estimate(0.1).alpha is None

    # While pushed, a big jump is expected motion -> not flagged.
    ct.set_pushed(True)
    ct.update(400.0, 0.0, 0.2, 0.0)
    assert ct.moved_unexpectedly() is False


def test_cube_orientation_wraps_modulo_90deg():
    ct = CubeTracker("blue")
    ct.update(0.0, 0.0, 0.0, 0.0)
    ct.set_orientation(math.radians(100), 0.05)
    assert -math.pi / 4 <= ct.estimate(0.0).alpha < math.pi / 4
    assert abs(angle_diff(ct.estimate(0.0).alpha, math.radians(10))) < 1e-9
