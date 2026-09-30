"""Tests for rover_strategy.control.{pid,controllers}.

A small truth model (`TruthRover`) stands in for the real rover: unicycle kinematics driven by
per-wheel commanded speeds passed through a motor deadband, an L/R gain asymmetry, and a first-order
speed lag, with `estimate()` handing the controller a Gaussian-noised pose delayed ~100 ms (as if from
an EKF fed by 100-150 ms latency overhead vision) -- close enough to the real failure modes (deadband
stall, asymmetric turning, stale feedback) to make the assertions below meaningful rather than trivial
kinematic-sim smoke tests.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.control.controllers import (
    AlignController, CaptureController, NavParams, PushController, RetreatController, SegmentFollower,
    _Line,
)
from rover_strategy.control.pid import PID
from rover_strategy.frames import angle_diff, wrap
from rover_strategy.world import Pose, RoverEstimate, Segment, SegKind, WheelCommand

TRACK = C.rover.track_width      # 89 mm
DT = 0.02                        # 50 Hz control loop
LATENCY_S = 0.10                 # s, overhead-vision + EKF latency
LAG_TAU = 0.10                   # s, motor first-order speed lag
DEADBAND = 15.0                  # mm/s, per-wheel motor deadband
GAIN_L, GAIN_R = 0.9, 1.0        # 10% L/R gain asymmetry
POS_NOISE_STD = 1.5              # mm, 1-sigma pose noise
HEAD_NOISE_STD = math.radians(1.0)


class TruthRover:
    def __init__(self, x=0.0, y=0.0, theta=0.0, rng=None, vmax=220.0):
        self.x, self.y, self.theta = x, y, theta
        self.vl = self.vr = 0.0
        self.v = self.omega = 0.0
        self.vmax = vmax
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.t = 0.0
        self._hist: list[tuple] = [(0.0, x, y, theta, 0.0, 0.0)]

    def command(self, v: float, omega: float, dt: float) -> None:
        wc = WheelCommand.from_unicycle(v, omega, TRACK, self.vmax)
        tl, tr = wc.v_left, wc.v_right
        if abs(tl) < DEADBAND:
            tl = 0.0
        if abs(tr) < DEADBAND:
            tr = 0.0
        tl *= GAIN_L
        tr *= GAIN_R
        a = min(1.0, dt / LAG_TAU)
        self.vl += (tl - self.vl) * a
        self.vr += (tr - self.vr) * a
        self.v = (self.vl + self.vr) / 2.0
        self.omega = (self.vr - self.vl) / TRACK
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta = wrap(self.theta + self.omega * dt)
        self.t += dt
        self._hist.append((self.t, self.x, self.y, self.theta, self.v, self.omega))
        while len(self._hist) > 2 and self._hist[1][0] < self.t - 1.0:
            self._hist.pop(0)

    def estimate(self) -> RoverEstimate:
        t_query = self.t - LATENCY_S
        row = self._hist[0]
        for h in self._hist:
            if h[0] >= t_query:
                row = h
                break
            row = h
        _, x, y, th, v, om = row
        x += self.rng.normal(0, POS_NOISE_STD)
        y += self.rng.normal(0, POS_NOISE_STD)
        th = wrap(th + self.rng.normal(0, HEAD_NOISE_STD))
        return RoverEstimate(id=0, pose=Pose(x, y, th), v=v, omega=om)


def run_segment(seg: Segment, rover: TruthRover, max_ticks: int = 4000, dt: float = DT):
    sf = SegmentFollower()
    sf.reset(seg)
    log = []
    for i in range(max_ticks):
        est = rover.estimate()
        v, w, done = sf.step(est, rover.t)
        log.append((rover.t, rover.x, rover.y, rover.theta, v, w, sf.tracking_error()))
        if done:
            break
        rover.command(v, w, dt)
    else:
        pytest.fail("segment did not finish within max_ticks")
    return sf, log


# --------------------------------------------------------------------------------------- PID unit tests
def test_pid_saturation_respected():
    pid = PID(kp=5.0, ki=0.0, kd=0.0, out_min=-1.0, out_max=1.0)
    for _ in range(50):
        u = pid.step(setpoint=100.0, measurement=0.0, dt=0.02)
        assert -1.0 <= u <= 1.0


def test_pid_anti_windup_no_divergence():
    """With a permanently unreachable setpoint (error stays huge and positive the whole run), naive
    integration would grow `_integral` without bound; conditional-integration anti-windup must freeze
    it as soon as the output saturates in the same direction the error is pushing."""
    pid = PID(kp=1.0, ki=2.0, kd=0.0, out_min=-1.0, out_max=1.0)
    x = 0.0
    for _ in range(1000):
        u = pid.step(setpoint=1000.0, measurement=x, dt=0.02)
        assert -1.0 <= u <= 1.0
        x += u * 0.02 * 0.1     # plant barely responds -> setpoint permanently unreachable
    assert abs(pid._integral) < 1e-9, f"integral wound up despite permanent saturation: {pid._integral}"


def test_pid_reset_clears_state():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    pid.step(1.0, 0.0, 0.02)
    pid.step(1.0, 0.2, 0.02)
    pid.reset()
    assert pid._integral == 0.0 and pid._prev_meas is None and pid._d_filt == 0.0


# --------------------------------------------------------------------------------------- ROTATE
@pytest.mark.parametrize("theta0,theta1", [
    (0.0, math.radians(90)),
    (math.radians(170), math.radians(-170)),   # short way is 20 deg across the +-pi seam
    (math.radians(-30), math.radians(200)),    # equivalent to a -190 deg request -> wraps to +170
])
def test_rotate_settles_no_large_overshoot(theta0, theta1):
    rover = TruthRover(theta=theta0, rng=np.random.default_rng(1))
    seg = Segment(SegKind.ROTATE, Pose(0, 0, theta0), Pose(0, 0, theta1))
    sf, log = run_segment(seg, rover, max_ticks=3000)

    errs = [angle_diff(theta1, th) for _, _, _, th, _, _, _ in log]
    final_err = abs(errs[-1])
    assert final_err < math.radians(3.0), f"final heading error {math.degrees(final_err):.2f} deg"

    # Overshoot: once the error first crosses zero, it must not swing back out past a small bound.
    crossed = False
    overshoot = 0.0
    for e in errs:
        if not crossed and abs(e) < math.radians(2.0):
            crossed = True
        if crossed:
            overshoot = max(overshoot, abs(e))
    assert overshoot < math.radians(6.0), f"overshoot {math.degrees(overshoot):.2f} deg"

    # Must take the short way across the wrap: total unwrapped travel should be close to |angle_diff|.
    thetas = [th for _, _, _, th, _, _, _ in log]
    total_travel = sum(abs(angle_diff(thetas[i + 1], thetas[i])) for i in range(len(thetas) - 1))
    assert total_travel < abs(angle_diff(theta1, theta0)) + math.radians(60)


def test_rotate_respects_omega_saturation():
    rover = TruthRover(theta=0.0, rng=np.random.default_rng(2))
    seg = Segment(SegKind.ROTATE, Pose(0, 0, 0.0), Pose(0, 0, math.pi))
    _, log = run_segment(seg, rover, max_ticks=3000)
    omegas = [row[5] for row in log]
    assert max(abs(w) for w in omegas) <= C.limits.w_nav + 1e-9


# --------------------------------------------------------------------------------------- STRAIGHT
def test_straight_forward_converges_from_offset():
    rover = TruthRover(x=0.0, y=20.0, theta=math.radians(10), rng=np.random.default_rng(3))
    seg = Segment(SegKind.STRAIGHT, Pose(0, 0, 0.0), Pose(500, 0, 0.0), reverse=False)
    sf, log = run_segment(seg, rover, max_ticks=4000)

    # Bound: P(D)-only line tracking (no integral, matching the spec's control law) leaves a small
    # steady-state cross-track offset under a persistent L/R gain asymmetry (analytically ~3.6 mm here:
    # disturbance omega_bias = (GAIN_R-GAIN_L)*v/track / ((GAIN_R+GAIN_L)/2), nulled via k_y*e_y_ss),
    # plus vision-noise jitter (pos sigma 1.5 mm) riding on top -- 9 mm is a realistic, not a chased-zero,
    # bound and is well inside the ~16 mm worst-case capture slack (G5).
    crosses = [row[6][1] for row in log]
    tail = crosses[len(crosses) // 2:]     # after the first half of the run, should be well converged
    assert max(abs(c) for c in tail) < 9.0, f"cross-track not converged: {max(abs(c) for c in tail)} mm"
    assert abs(rover.y) < 10.0
    assert rover.x > 490.0    # actually reached the end (no premature stop / stall)


def test_straight_reverse_converges_from_offset_and_correct_direction():
    rover = TruthRover(x=500.0, y=20.0, theta=math.radians(10), rng=np.random.default_rng(4))
    seg = Segment(SegKind.STRAIGHT, Pose(500, 0, 0.0), Pose(0, 0, 0.0), reverse=True)
    sf, log = run_segment(seg, rover, max_ticks=4000)

    crosses = [row[6][1] for row in log]
    tail = crosses[len(crosses) // 2:]
    assert max(abs(c) for c in tail) < 9.0
    assert abs(rover.y) < 10.0
    assert rover.x < 10.0
    vs = [row[4] for row in log]
    assert all(v <= 1e-6 for v in vs[1:-1]), "reverse segment must command v <= 0 throughout"


# --------------------------------------------------------------------------------------- sign conventions
def test_sign_forward_left_of_line_turns_right():
    """Rover 20mm LEFT of an eastward line (+y), heading aligned -> driving forward must turn right
    (negative omega, i.e. clockwise) to come back to the line."""
    sf = SegmentFollower()
    seg = Segment(SegKind.STRAIGHT, Pose(0, 0, 0.0), Pose(500, 0, 0.0), reverse=False)
    sf.reset(seg)
    est = RoverEstimate(id=0, pose=Pose(100.0, 20.0, 0.0), v=80.0, omega=0.0)
    v, w, done = sf.step(est, 0.0)
    assert v > 0
    assert w < 0, f"expected a right turn (omega<0), got {w}"


def test_sign_reverse_left_of_line_turns_correct_way():
    """Backing up from (0,0) to (500,0) (travel direction = +x/east), nose facing -x/west (pi) so the
    rear leads the motion: a rover 20mm to the world-left (+y, north) of the eastward line must still
    turn right (omega<0), same sign as the forward case -- see controllers.py module docstring for the
    kinematic derivation of why the sign does not flip between forward and reverse."""
    sf = SegmentFollower()
    seg = Segment(SegKind.STRAIGHT, Pose(0, 0, math.pi), Pose(500, 0, math.pi), reverse=True)
    sf.reset(seg)
    est = RoverEstimate(id=0, pose=Pose(300.0, 20.0, math.pi), v=-60.0, omega=0.0)
    v, w, done = sf.step(est, 0.0)
    assert v < 0
    assert w < 0, f"expected omega<0, got {w}"


def test_sign_right_of_line_is_mirror_image():
    sf = SegmentFollower()
    seg = Segment(SegKind.STRAIGHT, Pose(0, 0, 0.0), Pose(500, 0, 0.0), reverse=False)
    sf.reset(seg)
    est = RoverEstimate(id=0, pose=Pose(100.0, -20.0, 0.0), v=80.0, omega=0.0)
    v, w, done = sf.step(est, 0.0)
    assert w > 0, f"rover right of line driving forward must turn left (omega>0), got {w}"


# --------------------------------------------------------------------------------------- AlignController
def test_align_converges_heading_and_lateral():
    rover = TruthRover(x=0.0, y=8.0, theta=math.radians(15), rng=np.random.default_rng(5))
    ac = AlignController()
    phi = 0.0
    ac.reset((0.0, 0.0), phi)
    for i in range(4000):
        est = rover.estimate()
        v, w, done, status = ac.step(est, rover.t)
        if done:
            break
        rover.command(v, w, DT)
    else:
        pytest.fail("align did not converge")

    line = _Line((0.0, 0.0), phi, reverse=False)
    _, cross, e_h = line.errors(rover.x, rover.y, rover.theta)
    assert abs(cross) < 6.0, f"final lateral error {cross} mm"
    assert abs(e_h) < math.radians(2.0), f"final heading error {math.degrees(e_h)} deg"


# --------------------------------------------------------------------------------------- CaptureController
def test_capture_reaches_travel_distance_with_line_tracking():
    rover = TruthRover(x=0.0, y=6.0, theta=math.radians(-8), rng=np.random.default_rng(6))
    cc = CaptureController()
    travel = C.prepush_distance - C.contact_distance + 5.0
    cc.reset((0.0, 0.0), 0.0, travel)
    max_cross = 0.0
    for i in range(4000):
        est = rover.estimate()
        v, w, done = cc.step(est, rover.t)
        along, cross, _ = cc.tracking_error()
        if i > 20:
            max_cross = max(max_cross, abs(cross))
        if done:
            break
        assert abs(v) <= C.limits.v_capture + 1e-6
        rover.command(v, w, DT)
    else:
        pytest.fail("capture never finished")
    assert abs(rover.x - travel) < 8.0
    assert max_cross < 10.0


# --------------------------------------------------------------------------------------- PushController
def test_push_tracks_line_and_stops_near_goal_under_asymmetry():
    # Small residual entry error, as CaptureController's own line tracking would realistically hand off
    # (this test is about the asymmetry disturbance during the push itself, not re-testing capture).
    p0, p1 = (0.0, 0.0), (400.0, 0.0)
    rover = TruthRover(x=0.0, y=3.0, theta=math.radians(1.5), rng=np.random.default_rng(7))
    pc = PushController()
    pc.reset(p0, p1)
    max_omega = 0.0
    max_cross_mid = 0.0
    for i in range(6000):
        est = rover.estimate()
        # PushController now requires FRESH cube evidence to drive/finish at all (gate6 G6 finding #8) --
        # TruthRover has no independent cube model, so feed vision that agrees with the (delayed, noisy)
        # rover pose's own contact point, same as a real rigidly-pushed cube would. This still exercises
        # the asymmetry-disturbance-rejection this test is about, just no longer via a cube-less blind
        # dead-reckoning push (a design the gate audit flagged as unsafe).
        cube_xy = (est.pose.x + C.contact_distance * math.cos(est.pose.theta),
                   est.pose.y + C.contact_distance * math.sin(est.pose.theta))
        v, w, done, status = pc.step(est, cube_xy, 0.05, rover.t)
        max_omega = max(max_omega, abs(w))
        if 40 < status.along_mm < p1[0] - 40:
            max_cross_mid = max(max_cross_mid, abs(status.cross_mm))
        omega_cap = min(C.limits.w_fine, NavParams().push_curv_max * max(v, 1.0)) + 1e-6
        assert abs(w) <= omega_cap
        if done:
            break
        rover.command(v, w, DT)
    else:
        pytest.fail("push never finished")

    # Cube (contact_distance ahead of the rover) should land within +-8mm of the goal along-track coord.
    cube_x = rover.x + C.contact_distance * math.cos(rover.theta)
    assert abs(cube_x - p1[0]) < 8.0, f"cube stopped at x={cube_x}, target {p1[0]}"
    # Same P(D)-only steady-state-under-asymmetry story as the straight-line case, but here the omega
    # SATURATION (push_curv_max, the curvature/push-stability limit) is the actual bottleneck: raising
    # push_wn past ~1.5 buys almost nothing because correction is saturation- not gain-limited. 14 mm is
    # a realistic bound for a gently-capped omega loop and still clears the worst-case ~16-17mm
    # capture/channel slack (G5) with margin.
    assert max_cross_mid < 15.0, f"mid-push cross-track {max_cross_mid} mm too large under L/R asymmetry"


def test_push_lost_cube_flag():
    pc = PushController()
    pc.reset((0.0, 0.0), (400.0, 0.0))
    est = RoverEstimate(id=0, pose=Pose(100.0, 0.0, 0.0), v=50.0, omega=0.0)
    # Fresh vision shows the cube far off to the side (rover-left) -> definitely lost.
    # Loss is a debounced safety transition: one bad frame is not enough to
    # discard a cube during vision noise, so provide two consecutive readings.
    pc.step(est, (100.0, 100.0), 0.05, 0.0)
    v, w, done, status = pc.step(est, (100.0, 100.0), 0.05, 0.02)
    assert status.lost_cube is True

    v, w, done, status = pc.step(est, (100.0 + C.contact_distance, 1.0), 0.05, 0.02)
    assert status.lost_cube is False


# --------------------------------------------------------------------------------------- RetreatController
def test_retreat_reverses_distance_and_holds_heading():
    rover = TruthRover(x=0.0, y=0.0, theta=math.radians(20), rng=np.random.default_rng(8))
    rc = RetreatController()
    distance = 150.0
    rc.reset(distance)
    for i in range(4000):
        est = rover.estimate()
        v, w, done = rc.step(est, rover.t)
        if done:
            break
        assert v <= 1e-6
        rover.command(v, w, DT)
    else:
        pytest.fail("retreat never finished")
    travelled = math.hypot(rover.x, rover.y)
    assert abs(travelled - distance) < 8.0
    assert abs(angle_diff(rover.theta, math.radians(20))) < math.radians(3.0)
