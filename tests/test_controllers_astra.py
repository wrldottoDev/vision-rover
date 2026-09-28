"""Gate 6 adversarial contracts. Failures are intentional findings, not xfails.

Independent truth: 20 Hz commands, 5 ms wheel integration, exact first-order motor
lag (100 ms), left gain .85, 15 mm/s wheel deadband, delayed observations, and
seeded 2 mm / 1.5 degree noise. No production kinematics, transforms, simulator,
or controller error helpers are used to calculate truth or acceptance criteria.

The optional cube has its own world position and fixed world orientation. A
frictionless plate advances it only by unilateral normal contact; tangential
motion is not imposed by a fictitious rigid attachment. Paddle penetration is
recorded as a model-boundary violation, not silently converted into cube yaw.
This is a limiting contact model, not a calibrated friction/dynamics simulator.
Delayed-pose tests stress residual feedback delay; production EKF prediction
can reduce this. Separate exact-pose tests isolate bugs independent of the EKF.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.control.controllers import (
    AlignController, CaptureController, PushController,
    RetreatController, SegmentFollower,
)
from rover_strategy.control.pid import PID
from rover_strategy.coordination.supervisor import Supervisor
from rover_strategy.rover.fsm import RoverAgent, S
from rover_strategy.world import Pose, RoverEstimate, Segment, SegKind

DT = 0.05
TRACK = 89.0
PLATE = 47.0
TIP = 102.0
HALF_CHANNEL = 46.75
HALF_CUBE = 30.0


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def estimate(x, y, theta, v=0.0, omega=0.0):
    return RoverEstimate(0, Pose(x, y, theta), v=v, omega=omega)


@dataclass
class CubeTruth:
    x: float
    y: float
    alpha: float = 0.0
    max_paddle_penetration: float = 0.0

    def contact_step(self, rover):
        c, s = math.cos(rover.theta), math.sin(rover.theta)
        dx, dy = self.x - rover.x, self.y - rover.y
        along, lateral = c * dx + s * dy, -s * dx + c * dy
        rel = self.alpha - rover.theta
        half = HALF_CUBE * (abs(math.cos(rel)) + abs(math.sin(rel)))
        # A finite plate only pushes a cube whose lateral projection overlaps it.
        if abs(lateral) < HALF_CHANNEL + half and along > 0:
            penetration = PLATE + half - along
            if penetration > 0:
                self.x += penetration * c
                self.y += penetration * s
                along += penetration
        # At plate contact, the lateral support vertices lie within the rails'
        # x extent for the small yaw angles exercised by these tests.
        if along - half < TIP and along + half > PLATE:
            self.max_paddle_penetration = max(
                self.max_paddle_penetration, abs(lateral) + half - HALF_CHANNEL)


class WheelTruth:
    def __init__(self, x=0.0, y=0.0, theta=0.0, *, latency=.15,
                 noise=True, seed=6, cube=None):
        self.x, self.y, self.theta = x, y, theta
        self.left = self.right = self.v = self.omega = self.t = 0.0
        self.latency, self.noise, self.cube = latency, noise, cube
        self.rng = np.random.default_rng(seed)
        self.history = [self.snapshot()]

    def snapshot(self):
        return (self.t, self.x, self.y, self.theta, self.v, self.omega,
                None if self.cube is None else (self.cube.x, self.cube.y))

    def drive(self, v, omega, duration=DT):
        # Independent differential wheel conversion and common speed scaling.
        l, r = v - TRACK * omega / 2, v + TRACK * omega / 2
        scale = max(1.0, abs(l) / 180, abs(r) / 180)
        l, r = l / scale, r / scale
        l = .85 * l if abs(l) >= 15 else 0.0
        r = r if abs(r) >= 15 else 0.0
        steps = math.ceil(duration / .005)
        h = duration / steps
        decay = math.exp(-h / .1)
        for _ in range(steps):
            # Exact mean wheel speeds over each integration interval.
            lm = l + (self.left - l) * .1 * (1 - decay) / h
            rm = r + (self.right - r) * .1 * (1 - decay) / h
            self.left = l + (self.left - l) * decay
            self.right = r + (self.right - r) * decay
            vm, wm = (lm + rm) / 2, (rm - lm) / TRACK
            mid = self.theta + wm * h / 2
            self.x += vm * math.cos(mid) * h
            self.y += vm * math.sin(mid) * h
            self.theta = angle(self.theta + wm * h)
            self.v = (self.left + self.right) / 2
            self.omega = (self.right - self.left) / TRACK
            self.t += h
            if self.cube is not None:
                self.cube.contact_step(self)
        self.history.append(self.snapshot())

    def observe(self):
        cutoff = self.t - self.latency
        row = next((r for r in reversed(self.history) if r[0] <= cutoff + 1e-9),
                   self.history[0])
        _, x, y, th, v, om, cube = row
        if self.noise:
            x, y = np.array([x, y]) + self.rng.normal(0, 2, 2)
            th += self.rng.normal(0, math.radians(1.5))
            if cube is not None:
                cube = tuple(np.array(cube) + self.rng.normal(0, 2, 2))
        est = estimate(x, y, angle(th), v, om)
        est.age_s = max(0.0, self.t - row[0])
        return est, cube, est.age_s

    def settle(self):
        self.drive(0, 0, 1.0)  # Include the coast AFTER the controller declares done.


@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("heading", [0.0, math.pi - .01, -math.pi + .01])
def test_line_feedback_turn_reduces_world_cross_error(reverse, heading):
    c, s = math.cos(heading), math.sin(heading)
    nose = angle(heading + (math.pi if reverse else 0))
    follower = SegmentFollower()
    follower.reset(Segment(SegKind.STRAIGHT, Pose(0, 0, nose),
                           Pose(400*c, 400*s, nose), reverse=reverse))
    v, w, done = follower.step(estimate(100*c - 10*s, 100*s + 10*c, nose), 0)
    assert not done and (v < 0) == reverse
    # Independent prediction of cross velocity after a short commanded yaw.
    cross_velocity = v * math.sin(nose + w * DT - heading)
    assert cross_velocity < 0


@pytest.mark.parametrize("theta", [0.0, math.pi - .01, -math.pi + .01])
def test_retreat_initial_heading_and_signed_backward_progress(theta):
    rc = RetreatController()
    rc.reset(80)
    v, w, done = rc.step(estimate(200, 200, theta), 0)
    assert v < 0 and abs(w) < 1e-12 and not done
    # Pure backwards displacement must count positively even across the seam.
    v, w, done = rc.step(estimate(200 - 81*math.cos(theta),
                                 200 - 81*math.sin(theta), theta), 1)
    assert done and v == w == 0


@pytest.mark.parametrize("theta", [0.0, math.pi - .01])
def test_retreat_wheel_truth_stays_straight_and_stops(theta):
    rover = WheelTruth(theta=theta, seed=8)
    rc = RetreatController()
    rc.reset(80)
    peak_yaw = 0.0
    for _ in range(160):
        est, _, _ = rover.observe()
        v, w, done = rc.step(est, rover.t)
        assert v <= 0
        rover.drive(v, w)
        peak_yaw = max(peak_yaw, abs(angle(rover.theta - theta)))
        if done:
            break
    else:
        pytest.fail("retreat failed to complete within FSM's 8 s timeout")
    rover.settle()
    back = -(rover.x * math.cos(theta) + rover.y * math.sin(theta))
    cross = -rover.x * math.sin(theta) + rover.y * math.cos(theta)
    assert abs(back - 80) <= 6, f"settled backward progress {back:.2f} mm"
    assert abs(cross) <= 4.3, f"retreat lateral drift {cross:.2f} mm"
    assert peak_yaw <= math.radians(3), f"peak retreat yaw {math.degrees(peak_yaw):.2f} deg"


def test_engaged_fsm_retreat_keeps_heading_under_wheel_asymmetry():
    rover = WheelTruth(latency=0, noise=False)
    agent = RoverAgent(0, C)
    agent.state = S.RETREAT
    agent.engaged = True
    agent.retreat.reset(80)
    ctx = SimpleNamespace(retreat_clear=lambda *args: True,
                          log=lambda *args, **kwargs: None)
    peak_yaw = 0.0
    for _ in range(160):
        est, _, _ = rover.observe()
        v, w = agent.step(rover.t, est, True, ctx)
        rover.drive(v, w)
        peak_yaw = max(peak_yaw, abs(rover.theta))
        if agent.state != S.RETREAT:
            break
    else:
        pytest.fail("FSM retreat did not finish")
    rover.settle()
    assert peak_yaw <= math.radians(3), (
        f"engaged FSM discards retreat steering: yaw {math.degrees(peak_yaw):.2f} deg, "
        f"lateral displacement {rover.y:.2f} mm, despite exact current feedback")


def test_capture_origin_is_fsm_projection_not_cube_centre():
    phi = .7
    fw = np.array([math.cos(phi), math.sin(phi)])
    left = np.array([-fw[1], fw[0]])
    cube = np.array([350., 280.])
    start = cube - 160*fw + 2*left
    dist = float((cube - start) @ fw)
    p0 = cube - dist*fw  # Actual FSM contract, not the cube centre.
    cc = CaptureController()
    cc.reset(tuple(p0), phi, dist - 77 + 6)
    assert not cc.step(estimate(*start, phi), 0)[2]
    assert cc.tracking_error()[0] == pytest.approx(0, abs=1e-10)
    end = start + (dist - 77 + 6 + .01)*fw
    assert cc.step(estimate(*end, phi), 1)[2]


@pytest.mark.parametrize("latency", [.05, .15])
def test_capture_settled_stop_accuracy(latency):
    rover = WheelTruth(latency=latency, seed=6)
    travel = C.prepush_distance - 77 + 6
    cc = CaptureController()
    cc.reset((0, 0), 0, travel)
    for _ in range(200):
        est, _, _ = rover.observe()
        v, w, done = cc.step(est, rover.t)
        rover.drive(v, w)
        if done:
            break
    else:
        pytest.fail("capture exceeded FSM's 10 s timeout")
    rover.settle()
    # The 6 mm overdrive is ALREADY in travel; this bounds additional error.
    assert abs(rover.x - travel) <= 3, (
        f"extra travel beyond requested capture: {rover.x - travel:.2f} mm "
        f"at {latency*1000:.0f} ms latency")


def test_alignment_small_rotation_clears_wheel_deadband_before_timeout():
    rover = WheelTruth(theta=math.radians(2), noise=False)
    ac = AlignController()
    ac.reset((0, 0), 0)
    for _ in range(300):
        est, _, _ = rover.observe()
        v, w, done, _ = ac.step(est, rover.t)
        rover.drive(v, w)
        if done:
            return
    pytest.fail(f"2 degree alignment stuck after 15 s: {math.degrees(rover.theta):.2f} deg; "
                "0.18 rad/s kick is only 8.01 mm/s per wheel")


def ideal_alignment_trace(cross=24.0):
    """Perfect feedback isolates nudge logic from the separate deadband failure."""
    ac = AlignController()
    ac.reset((C.prepush_distance, 0), 0)
    x, y, th = 0., cross, 0.
    trace = []
    for k in range(100):
        v, w, done, phase = ac.step(estimate(x, y, th), k*DT)
        trace.append((x, y, th, phase, done))
        # "failed"/"reposition" are also terminal (gate6 G6 findings #3/#4): the state machine has
        # stopped trying, just without claiming success (done stays False). Only "rotating" with
        # nudges still available, or a nudge in progress, should keep looping.
        if done or phase in ("failed", "reposition"):
            return trace
        if phase == "nudge_turn":
            th = max(-math.radians(20), min(math.radians(20), math.asin(-y/25)))
        elif phase == "nudge_drive":
            # Advance in real per-tick increments (matching the controller's own v_capture*DT), not a
            # single fixed 25 mm jump: the controller may now choose a SHORTER safe nudge distance than
            # align_nudge_distance (gate6 G6 finding #3, drift-budget-limited nudges), and a one-shot
            # jump would silently ignore that and always travel the old fixed distance.
            x += C.limits.v_capture*DT*math.cos(th)
            y += C.limits.v_capture*DT*math.sin(th)
        elif phase in ("rotating", "nudge_back"):
            th = 0.
    pytest.fail("ideal alignment state machine did not finish")


def test_alignment_does_not_claim_success_after_exhausted_nudges():
    x, y, th, phase, done = ideal_alignment_trace()[-1]
    assert not done or abs(y) <= 3, f"alignment claims success at lateral {y:.2f} mm"


def point_inside_convex(poly, point):
    # Independent containment oracle, not production shapes.point_in.
    signs = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        edge, rel = b - a, np.asarray(point) - a
        signs.append(edge[0]*rel[1] - edge[1]*rel[0])
    return min(signs) >= -1e-8 or max(signs) <= 1e-8


def test_alignment_two_nudges_remain_in_reserved_region():
    sup = Supervisor([0])
    polys = sup.manipulation_region(0, Pose(0, 0, 0), 0, 400)
    for x, y, th, phase, _ in ideal_alignment_trace():
        for px, py in [(-47, -49.75), (-47, 49.75), (102, -49.75), (102, 49.75)]:
            pt = (x + px*math.cos(th) - py*math.sin(th),
                  y + px*math.sin(th) + py*math.cos(th))
            assert any(point_inside_convex(p, pt) for p in polys), (
                f"{phase}: footprint corner {pt} leaves reserved union at pose {(x, y, th)}")


def test_alignment_does_not_rotate_back_with_cube_among_paddles():
    for x, y, th, phase, _ in ideal_alignment_trace():
        if phase != "nudge_back":
            continue
        # Axis-aligned cube's near face is inside the final straight paddle reach.
        near_face = C.prepush_distance - 30 - x
        overlap_y = abs(y) < HALF_CHANNEL + 30
        assert not (near_face < TIP and overlap_y), (
            f"rotate-back requested with cube near face {near_face:.2f} mm ahead; tips at {TIP}")


def test_push_fusion_preserves_measured_12mm_channel_offset():
    pc = PushController()
    pc.reset((0, 0), (400, 0))
    est = estimate(-77, -12, 0)
    _, w, _, status = pc.step(est, (0, 0), 0, 0)
    assert abs(status.cross_mm) <= 2, (
        f"perfect cube vision on line becomes fictional cross error {status.cross_mm:.2f} mm; omega={w:.4f}")


@pytest.mark.parametrize("latency", [.05, .15])
def test_push_independent_cube_offset_12mm(latency):
    cube = CubeTruth(0, 0)
    rover = WheelTruth(-77, -12, cube=cube, latency=latency, seed=7)
    pc = PushController()
    pc.reset((0, 0), (400, 0))
    max_cross = 0.0
    for _ in range(420):
        est, obs, age = rover.observe()
        v, w, done, status = pc.step(est, obs, age, rover.t)
        assert not status.lost_cube, "controller lost a cube initially inside the channel"
        assert abs(w) <= abs(v)/400 + 1e-12  # Command curvature alone is insufficient.
        rover.drive(v, w)
        max_cross = max(max_cross, abs(cube.y))
        if cube.max_paddle_penetration > .5:
            pytest.fail(f"independent cube reaches paddle: {cube.max_paddle_penetration:.2f} mm "
                        f"penetration, cube x={cube.x:.1f}; model requires contact/rotation response")
        if done:
            break
    else:
        pytest.fail("push exceeded 21 s (FSM budget for this leg)")
    rover.settle()
    max_cross = max(max_cross, abs(cube.y))
    assert abs(cube.x - 400) <= 6 and abs(cube.y) <= 6, (
        f"settled cube error ({cube.x - 400:.2f}, {cube.y:.2f}) mm; "
        f"peak cross {max_cross:.2f} mm; allowed delivery margin 6 mm")


def test_push_current_pose_plus_delayed_cube_stop_prediction():
    pc = PushController()
    pc.reset((0, 0), (400, 0))
    # Best case current rover pose; last cube frame 150 ms old at 35 mm/s.
    # Current cube 398, and known motor coast 3.5 => it must stop already.
    est = estimate(398 - 77, 0, 0, v=35)
    v, w, done, _ = pc.step(est, (398 - 35*.15, 0), .15, 1)
    assert done and v == w == 0, "fusion delay defeats even an exact current pose's stop prediction"


def test_push_stale_cube_cannot_report_success_from_rover_pose_alone():
    pc = PushController()
    pc.reset((0, 0), (400, 0))
    # The actual cube was lost at x=250. Stale vision does not prove contact.
    v, w, done, status = pc.step(estimate(323, 0, 0), (250, 0), .3, 1)
    assert not done, "reports push complete with stale cube 150 mm short of target"


@pytest.mark.parametrize("half,lat", [(30.0, 19.0), (30*math.sqrt(2), 8.0)])
def test_push_lost_threshold_respects_orientation_dependent_channel(half, lat):
    pc = PushController()
    pc.reset((0, 0), (400, 0))
    assert half + lat > HALF_CHANNEL
    _, _, _, status = pc.step(estimate(0, 0, 0), (PLATE + half, lat), 0, 0)
    assert status.lost_cube, f"impossible channel fit (half-width {half:.2f}, offset {lat}) accepted"


def test_push_low_speed_actual_curvature_respects_limit_under_asymmetry():
    pc = PushController()
    pc.reset((0, 0), (400, 0))
    rover = WheelTruth(360 - 77, -10, noise=False, latency=0)
    v, w, _, _ = pc.step(estimate(rover.x, rover.y, 0), (360, -10), 0, 0)
    # `w == v/400` (the OLD, unfixed assertion) is provably incompatible with the curvature check below:
    # v/400 is exactly the COMMANDED omega/v cap, but under this truth model's fixed 15% per-wheel gain
    # asymmetry, commanding it verbatim deterministically settles to curvature ~0.00428 (see gate6 G6
    # finding #9) -- no controller can satisfy both "w equals the naive cap" and "physical curvature obeys
    # the limit" for this exact scenario. The fix derates omega below the naive cap so the REALISED
    # curvature is what respects push_curv_max; assert that contract instead of the old exact value.
    assert v == 35 and abs(w) <= v/400 + 1e-9
    rover.drive(v, w, .5)
    curvature = abs(rover.omega / rover.v)
    assert curvature <= 1/400 + 1e-6, f"actual curvature {curvature:.5f}, radius {1/curvature:.1f} mm"


def test_fsm_push_passes_real_cube_age_and_does_not_refresh_same_capture():
    agent = RoverAgent(0, C)
    agent.task = SimpleNamespace(color="red")
    agent.state = S.PUSH
    agent.push_budget = 10
    agent.last_seen_push = .75
    seen_ages = []

    class PusherSpy:
        def step(self, est, xy, age, t):
            seen_ages.append(age)
            return 35, 0, False, SimpleNamespace(cross_mm=0, lost_cube=False)

    agent.pusher = PusherSpy()
    ctx = SimpleNamespace(fresh_cube=lambda color, since: (np.array([77., 0]), 1, .80))
    agent._s_push(1.0, estimate(0, 0, 0), None, ctx)
    assert seen_ages == pytest.approx([.20]), f"FSM passed ages {seen_ages} for capture t=.80 at t=1"
    assert agent.last_seen_push <= .80, "same buffered capture resets blindness to wall-clock now"


def test_fsm_verify_capture_rejects_cube_not_touching_front_plate():
    agent = RoverAgent(0, C)
    # Centre 40 mm ahead of plate means 10 mm air gap for a face-aligned cube.
    # At lateral 11, no rotated square at depth 40 fits: 11 + 40 > 46.75.
    # Lead amendment: the 5 mm contact tolerance absorbs measured depth bias (flush cubes read 26-37 mm in closed
    # loop, unknown marker offset); the geometric rule is still enforced beyond it.
    assert not agent._fits_channel(45, 11), "contact tolerance accepts impossible plate/channel geometry"


def verified_contact_fixture(grant):
    agent = RoverAgent(0, C)
    agent.state = S.VERIFY_CAPTURE
    agent.engaged = True
    # Lead amendment: an engaged rover may change the push line by <= 15 deg (FsmParams.push_line_max_change_deg);
    # the original 24 deg fixture now (correctly) aborts to RETREAT.  ~10 deg still tests the recommitted heading.
    leg = SimpleNamespace(cube_end=(300, 39))
    agent.task = SimpleNamespace(color="red", leg=0, budget_used=0., t_start=0.,
                                 plan=SimpleNamespace(legs=[leg]))
    requested = []

    def region(rid, pose, heading, distance):
        requested.append(heading)
        return []

    ctx = SimpleNamespace(
        fresh_cube=lambda color, since: (np.array([77., 0]), 4, 1.),
        cube=lambda color: SimpleNamespace(xy=lambda: np.array([77., 0])),
        log=lambda *args, **kwargs: None,
        set_cube_orientation=lambda *args: None,
        set_cube_pushed=lambda *args: None,
        manipulation_region=region,
        commit_region=lambda rid, polys: grant,
    )
    return agent, ctx, requested


def test_verify_capture_reserves_actual_new_push_heading():
    agent, ctx, headings = verified_contact_fixture(True)
    agent._s_verify_capture(1, estimate(0, 0, 0), None, ctx)
    assert agent.state == S.PUSH
    actual_heading = math.atan2(39, 300-77)
    assert headings == pytest.approx([actual_heading]), (
        f"recommitted heading {headings}, but new push line is {math.degrees(actual_heading):.2f} deg")


def test_verify_capture_denied_region_obeys_timeout():
    agent, ctx, _ = verified_contact_fixture(False)
    # Fresh valid geometry continues arriving, but corridor permission is denied.
    # The state timeout must not depend on observations being missing.
    ctx.fresh_cube = lambda color, since: (np.array([77., 0]), 4, 10.)
    agent._s_verify_capture(10, estimate(0, 0, 0), None, ctx)
    assert agent.state != S.VERIFY_CAPTURE, "denied recommit bypasses 3 s verify timeout indefinitely"


def test_measure_does_not_infer_flush_orientation_for_noncontact_cube():
    agent, ctx, _ = verified_contact_fixture(True)
    agent.state = S.MEASURE
    orientations = []
    ctx.fresh_cube = lambda color, since: (np.array([0., 100]), 4, 1.)
    ctx.set_cube_orientation = lambda color, alpha, std: orientations.append(alpha)
    agent._s_measure(1, estimate(0, 0, 0), None, ctx)
    assert not orientations or all(a is None for a in orientations), (
        f"cube beside rover labelled flush with orientation {orientations}")


def test_pid_integral_does_not_jump_far_past_saturation_on_first_tick():
    pid = PID(kp=0, ki=1, kd=0, out_min=-1, out_max=1)
    pid.step(1000, 0, DT)
    pid.step(0, 0, DT)
    output_after_reversal = pid.step(-1, 0, DT)
    assert abs(pid._integral) <= 1, f"integral jumped to {pid._integral} before saturation check"
    assert output_after_reversal < 1, "saturated output persists after error reverses"


def test_pid_derivative_filter_and_setpoint_kick_contract():
    pid = PID(kp=0, ki=0, kd=1, d_tau=.05)
    assert pid.step(0, 0, DT) == 0
    assert pid.step(100, 0, DT) == 0  # derivative on measurement, no setpoint kick
    assert pid.step(100, 2, DT) == pytest.approx(-20)
    assert pid.step(100, 2, DT) == pytest.approx(-10)


def test_heading_wrap_does_not_create_derivative_kick():
    sf = SegmentFollower()
    sf.reset(Segment(SegKind.ROTATE, Pose(0, 0, math.pi-.01),
                     Pose(0, 0, -math.pi+.05)))
    _, w1, _ = sf.step(estimate(0, 0, math.pi-.01), 0)
    _, w2, _ = sf.step(estimate(0, 0, -math.pi+.01), DT)
    # Bound raised from .3 to comfortably above the wheel-level rotation deadband (2*15/89 ~= .337 rad/s,
    # gate6 G6 finding #5): any nonzero in-place rotation command must clear it, so .3 is no longer a
    # valid "small" ceiling. The point of this test -- no derivative-kick-sized jump from the heading
    # wrap -- is unaffected.
    assert 0 < w1 < .4 and abs(w2) < .4
