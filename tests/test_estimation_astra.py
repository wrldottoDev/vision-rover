"""Gate 4 adversarial contracts, deliberately not xfailed for known defects.

Truth uses independent wheel ODEs and RK4, with unequal gains, a deadband,
and unequal motor time constants. Capture and arrival times are separate.
Bounds are operational (6 mm nominal RMS / 15 mm peak, 6 deg heading),
not comparisons to a second copy of the estimator. Failed contracts are
intended to block the gate until the implementation is corrected.
"""
from dataclasses import dataclass, replace
import math

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.estimation.cube_tracker import CubeTracker
from rover_strategy.estimation.pose_estimator import PoseEstimator
from rover_strategy.frames import wrap
from rover_strategy.world import Pose, TrackQuality


def estimator(*, cfg=C.estimator, policy=C.telemetry, pose=Pose(0., 0., 0.)):
    est = PoseEstimator(cfg, policy, 0)
    est.initialize(pose, 0.)
    return est


class WheelTruth:
    """RK4 integration of x,y,unwrapped heading,left speed,right speed."""

    def __init__(self, gain):
        self.s = np.array([300., 300., math.radians(175.), 0., 0.])
        self.gain = gain

    def advance(self, v, w, stuck=False):
        track = 89.0
        targets = np.array([v - w * track / 2, v + w * track / 2])
        targets = np.sign(targets) * np.maximum(abs(targets) - 3., 0.)
        targets *= [0. if stuck else self.gain, 1.]

        def derivative(s):
            speed = (s[3] + s[4]) / 2
            return np.array([speed * math.cos(s[2]), speed * math.sin(s[2]),
                             (s[4] - s[3]) / track,
                             (targets[0] - s[3]) / .10,
                             (targets[1] - s[4]) / .13])

        h = .001
        for _ in range(10):
            s = self.s
            a = derivative(s)
            b = derivative(s + h * a / 2)
            c = derivative(s + h * b / 2)
            d = derivative(s + h * c)
            self.s = s + h * (a + 2*b + 2*c + d) / 6


@dataclass
class Run:
    position: list
    heading: list
    accepted_good: int
    total_good: int
    reinitializations: int
    wrapped: bool
    lost: list


def simulate(*, gain=.85, burst_frames=2, stuck=False, seed=419):
    rng = np.random.default_rng(seed)
    truth = WheelTruth(gain)
    est = estimator(pose=Pose(*truth.s[:3]))
    pending = {}
    errors, headings, lost = [], [], []
    accepted_good = total_good = 0
    wrapped = False
    previous_heading = wrap(truth.s[2])
    # Exactly one dropped frame in each ten-frame block, offset by seed.
    drop_slot = seed % 10
    for tick in range(1, 1001):
        t = tick / 100
        phase = ((tick - 1) // 200) % 3
        v, w = [(100., .55), (0., 1.4), (85., -.65)][phase]
        est.set_command((tick - 1) / 100, v, w)
        truth.advance(v, w, stuck=stuck and t >= 2.)
        current_heading = wrap(truth.s[2])
        wrapped |= abs(current_heading - previous_heading) > math.pi
        previous_heading = current_heading
        if tick % 5 == 0:
            frame = tick // 5
            bad = frame >= 20 and frame % 20 < burst_frames
            if frame % 10 != drop_slot:
                noise = rng.normal(size=3) * [2., 2., math.radians(1.5)]
                z = truth.s[:3] + noise
                if bad:
                    # 1 Hz bursts of mutually inconsistent vision glitches.
                    z += [180. * (-1)**frame, 120., .7 * (-1)**frame]
                pending[tick + 15] = (t, Pose(z[0], z[1], wrap(z[2])), bad)
        est.predict(t)
        if tick in pending:
            capture, z, bad = pending.pop(tick)
            accepted = est.update_vision(z, capture, t)
            if not bad:
                total_good += 1
                accepted_good += accepted
        e = est.estimate(t)
        assert np.all(np.isfinite(est._P))
        assert np.max(abs(est._P - est._P.T)) < 1e-9
        assert np.linalg.eigvalsh(est._P).min() > 0.
        assert -math.pi <= est._x[2] < math.pi
        if t >= 1.:
            errors.append(math.hypot(e.pose.x-truth.s[0], e.pose.y-truth.s[1]))
            headings.append(abs(wrap(e.pose.theta-truth.s[2])))
            lost.append(e.quality == TrackQuality.LOST)
    return Run(errors, headings, accepted_good, total_good,
               est.reinit_count, wrapped, lost)


@pytest.mark.parametrize("gain", [.90, .85])
def test_wheel_mismatch_latency_drops_wrap_and_short_outlier_bursts(gain):
    run = simulate(gain=gain)
    metrics = {"rms_mm": float(np.sqrt(np.mean(np.square(run.position)))),
               "peak_mm": max(run.position),
               "peak_heading_deg": math.degrees(max(run.heading)),
               "accepted_good_fraction": run.accepted_good / run.total_good,
               "reinits": run.reinitializations}
    assert run.wrapped, "The truth trajectory must actually cross the heading seam"
    assert metrics["rms_mm"] < 6., metrics
    assert metrics["peak_mm"] < 15., metrics
    assert metrics["peak_heading_deg"] < 6., metrics
    assert metrics["accepted_good_fraction"] > .95, metrics
    assert metrics["reinits"] == 0, metrics


def test_five_frame_bursts_do_not_teleport_the_rover():
    run = simulate(burst_frames=5)
    assert max(run.position) < 30., (
        f"peak error {max(run.position):.1f} mm, reinits={run.reinitializations}")
    assert run.reinitializations == 0


def test_stuck_wheel_does_not_leave_a_confident_bad_pose():
    run = simulate(burst_frames=0, stuck=True)
    unsafe = [(p, math.degrees(h)) for p, h, lost in
              zip(run.position, run.heading, run.lost)
              if not lost and (p > 15. or h > math.radians(6.))]
    assert not unsafe, f"{len(unsafe)} unsafe ticks; first five: {unsafe[:5]}"


def test_process_covariance_does_not_depend_on_control_poll_frequency():
    variances = []
    for dt in [.02, .002]:
        est = estimator()
        for i in range(1, round(2/dt) + 1):
            est.predict(i * dt)
        variances.append(np.diag(est._P)[3:])
    ratio = variances[0] / variances[1]
    assert np.all(abs(ratio - 1.) < .1), f"20 ms / 2 ms variances: {ratio}"


def test_step_covariance_jacobian_matches_actual_mean_derivative():
    est = estimator(cfg=replace(C.estimator, accel_noise=0., alpha_noise=0.))
    x = np.array([10., 20., 2.4, 85., 1.2])
    p = np.diag([2., 3., .004, 25., .02])
    jac = np.empty((5, 5))
    for j in range(5):
        delta = np.eye(5)[j] * 1e-5
        plus = est._step(x + delta, p, .02, 100., -.8)[0]
        minus = est._step(x - delta, p, .02, 100., -.8)[0]
        jac[:, j] = (plus-minus) / 2e-5
    actual = est._step(x, p, .02, 100., -.8)[1]
    np.testing.assert_allclose(actual, jac @ p @ jac.T, atol=1e-8, rtol=1e-7)


def test_future_capture_is_not_accepted_or_allowed_to_poison_dedup():
    est = estimator()
    assert not est.update_vision(Pose(0., 0., 0.), 1., .1)
    assert est.update_vision(Pose(0., 0., 0.), .15, .2)
    assert est.estimate(.2).age_s >= 0.


def test_clock_skew_cannot_postpone_lost_after_a_frozen_frame():
    est = estimator()
    est.update_vision(Pose(0., 0., 0.), 1., .1)
    for tick in range(2, 7):
        t = tick / 10
        est.update_vision(Pose(0., 0., 0.), 1., t)
        e = est.estimate(t)
    assert e.quality == TrackQuality.LOST, (e.age_s, e.quality)


def test_late_processing_never_moves_filter_time_backwards():
    est = estimator()
    est.predict(.3)
    est.update_vision(Pose(0., 0., 0.), .1, .2)
    assert est._t >= .3


def test_truly_older_than_history_is_ignored_without_mutation():
    est = estimator()
    for tick in range(1, 61):
        est.predict(tick * .05)
    before = est._x.copy(), est._P.copy()
    # Newer than the last processed capture, but older than replay history.
    assert not est.update_vision(Pose(999., 999., 2.), .5, 3.)
    np.testing.assert_array_equal(est._x, before[0])
    np.testing.assert_array_equal(est._P, before[1])
    assert est.consecutive_rejects == 0


def test_delayed_corrections_invalidate_future_checkpoints():
    delayed, reference = estimator(), estimator()
    for est in (delayed, reference):
        est.set_command(0., 0., 0.)
    # Precomputed uncorrected checkpoints must not survive a correction at .1.
    for t in [.05, .1, .15, .2, .25, .3]:
        delayed.predict(t)
    for t, x in [(.1, 3.), (.2, 4.), (.25, 3.5)]:
        assert reference.update_vision(Pose(x, 0., 0.), t, t)
        assert delayed.update_vision(Pose(x, 0., 0.), t, .3)
    reference.predict(.3)
    # Different numerical partitions can perturb Q; stale checkpoints would lose
    # much of the preceding corrections, not just cause sub-millimetre differences.
    np.testing.assert_allclose(delayed._x[:3], reference._x[:3], atol=.1)
    assert all(a[0] <= b[0] for a, b in
               zip(delayed._checkpoints, delayed._checkpoints[1:]))


def test_duplicate_outlier_does_not_increment_rejection_streak():
    est = estimator()
    assert not est.update_vision(Pose(500., 500., 1.), .1, .15)
    for _ in range(20):
        assert not est.update_vision(Pose(500., 500., 1.), .1, .2)
    assert est.consecutive_rejects == 1
    assert est.reinit_count == 0
    assert est.estimate(.5).quality == TrackQuality.LOST


def test_retained_checkpoint_keeps_all_commands_needed_for_replay():
    short = estimator()
    long = estimator(cfg=replace(C.estimator, history_s=10.))
    oracle = estimator()
    for tick in range(21):
        t = tick / 10
        v = 100. if tick < 10 else 60.
        for est in (short, long, oracle):
            est.set_command(t, v, 0.)
    oracle.predict(1.1)
    z = oracle.estimate(1.1).pose
    results = []
    for est in (short, long):
        # Sparse checkpoints, but fine command history; capture is within history_s.
        est.predict(2.)
        results.append(est.update_vision(z, 1.1, 2.))
    assert results == [True, True], results
    np.testing.assert_allclose(short._x, long._x, atol=1e-8)


def test_blind_travel_counts_distance_when_the_rover_reverses():
    # Isolate the distance guard from the independent age/covariance guards.
    policy = replace(C.telemetry, lost_age_s=100., pos_std_stop_mm=1e6,
                     heading_std_stop_rad=1e6, max_blind_travel_mm=30.)
    est = estimator(policy=policy)
    est.set_command(0., 100., 0.)
    est.predict(.3)  # ~19 mm out.
    est.set_command(.3, -100., 0.)
    est.predict(.75)  # ~24 mm back: >40 mm path, <10 mm displacement.
    assert est.estimate(.75).quality == TrackQuality.LOST


def test_inconsistent_rejections_cannot_reinitialize_to_the_fifth_glitch():
    est = estimator()
    for i in range(1, 6):
        t = i * .05
        est.update_vision(Pose((-1)**i * 400., 300., (-1)**i), t, t)
    e = est.estimate(.25)
    assert math.hypot(e.pose.x, e.pose.y) < 30., (e.pose, e.quality)
    assert e.quality != TrackQuality.GOOD


def settled_cube():
    ct = CubeTracker("red")
    for i in range(41):
        ct.update(100., 200., i * .05, 0.)
    return ct


def test_cube_push_stop_during_occlusion_cannot_erase_uncertainty():
    ct = settled_cube()
    ct.set_pushed(True)
    before = ct.estimate(2.5)
    ct.set_pushed(False)
    after = ct.estimate(2.5)
    assert after.pos_std >= before.pos_std, (before.pos_std, after.pos_std)


def test_cube_genuine_jump_reacquires_or_reports_honest_uncertainty():
    ct = settled_cube()
    ct.update(140., 200., 2.05, 0.)
    e = ct.estimate(2.05)
    assert ct.moved_unexpectedly()
    assert abs(e.x - 140.) <= 3 * e.pos_std, (e.x, e.pos_std)


def test_neighbour_push_is_tracked_without_persistent_false_jump_flags():
    ct = settled_cube()
    flags = []
    for i in range(1, 21):
        # No local set_pushed: another rover makes a smooth 70 mm/s push.
        ct.update(100. + 3.5*i, 200., 2. + .05*i, 0.)
        flags.append(ct.moved_unexpectedly())
    e = ct.estimate(3.)
    assert abs(e.x - 170.) <= 3 * e.pos_std, (e.x, e.pos_std)
    assert sum(flags) <= 2, f"Smooth movement flagged as jumps {sum(flags)} times"


def test_cube_stale_initial_observation_uses_last_seen_capture_time():
    ct = CubeTracker("blue")
    # world.CubeObs / parser pass the last observation time, already age-adjusted.
    ct.update(100., 200., 8., age_s=2.)
    assert ct.estimate(10.).age_s == 2.
    ct.update(100., 200., 8., age_s=3.)
    assert ct.estimate(11.).age_s == 3.


@pytest.mark.parametrize("degrees", [-765., -135., -45., 45., 135., 765.])
def test_cube_orientation_wraps_at_every_square_symmetry_boundary(degrees):
    ct = settled_cube()
    ct.set_orientation(math.radians(degrees), .05)
    alpha = ct.estimate(2.).alpha
    assert -math.pi/4 <= alpha < math.pi/4
    assert math.isclose(math.cos(4*alpha), math.cos(4*math.radians(degrees)), abs_tol=1e-12)
    assert math.isclose(math.sin(4*alpha), math.sin(4*math.radians(degrees)), abs_tol=1e-12)
