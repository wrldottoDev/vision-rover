import math

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.control.controllers import SegmentFollower
from rover_strategy.frames import angle_diff, wrap
from rover_strategy.geometry.footprint import Footprint
from rover_strategy.planning.navigation import (
    DiscObstacle,
    NavPlanner,
    _arc_pose,
    path_collides,
    segment_collides,
    swept_polygons,
)
from rover_strategy.world import Path, Pose, RoverEstimate, SegKind, Segment


BW = BH = C.board.width


def _inside_convex(poly: np.ndarray, point: np.ndarray) -> bool:
    edges = np.roll(poly, -1, axis=0) - poly
    rel = point - poly
    return bool(np.all(edges[:, 0] * rel[:, 1] - edges[:, 1] * rel[:, 0] >= -1e-7))


@pytest.mark.parametrize("heading", [0.0, math.pi / 2, math.pi, -math.pi / 2])
def test_edge_goal_uses_exact_arcs_and_collinear_terminal_straights(heading):
    start = Pose(640.0, 443.0, heading)
    goal = Pose(797.0, 619.0, -math.pi / 2)
    path = NavPlanner().plan(start, goal, [], BW, BH, inflate=0.0, deadline_s=1.0)
    assert path is not None
    assert path.goal == goal
    assert any(seg.kind is SegKind.ARC for seg in path.segments)
    for seg in path.segments:
        if seg.kind is SegKind.STRAIGHT:
            along, cross = seg.start.to_local(seg.end.x, seg.end.y)
            assert abs(cross) < 1e-5
            assert seg.reverse == (along < 0.0)
    assert not path_collides(Footprint(C.rover), path, [], BW, BH, 0.0)
    expected = 0.0
    for seg in path.segments:
        if seg.kind is SegKind.ROTATE:
            dtheta = abs(angle_diff(seg.end.theta, seg.start.theta))
            expected += dtheta / C.limits.w_nav
            if dtheta > 1e-9:
                expected += NavPlanner().params.rotate_settle_s
        else:
            expected += seg.length / C.limits.v_nav
    assert path.cost_s == pytest.approx(expected)


def test_arc_reservation_contains_dense_independent_corners():
    fp = Footprint(C.rover)
    start = Pose(300.0, 300.0, math.radians(17.0))
    probe = Segment(SegKind.ARC, start, start, reverse=False, curvature=1.0 / 120.0)
    end = _arc_pose(probe, 120.0 * math.radians(83.0))
    arc = Segment(SegKind.ARC, start, end, reverse=False, curvature=1.0 / 120.0)
    poly = swept_polygons(fp, Path([arc], 0.0))[0]
    for distance in np.linspace(0.0, arc.length, 4001):
        pose = _arc_pose(arc, float(distance))
        for corner in fp.envelope(pose.x, pose.y, pose.theta):
            assert _inside_convex(poly, corner)


def test_arc_collision_checks_mid_sweep_contact_and_fast_parity():
    fp = Footprint(C.rover)
    start = Pose(300.0, 300.0, 0.0)
    probe = Segment(SegKind.ARC, start, start, curvature=1.0 / 120.0)
    mid = _arc_pose(probe, 120.0 * math.radians(35.0))
    corner = fp.envelope(mid.x, mid.y, mid.theta)[1]
    obstacle = [DiscObstacle(float(corner[0]), float(corner[1]), 0.1)]
    arc = Segment(SegKind.ARC, start, _arc_pose(probe, 120.0 * math.radians(70.0)), curvature=1.0 / 120.0)
    assert segment_collides(fp, arc, obstacle, BW, BH, 0.0)
    import rover_strategy.planning.navigation as nav
    assert nav._fast_segment_collides(nav._FastCache(fp), arc, nav._fast_obstacles(obstacle), BW, BH, 0.0)


def test_arc_follower_reaches_endpoint_with_lag_and_wheel_asymmetry():
    start = Pose(300.0, 300.0, 0.2)
    curvature = 1.0 / 250.0
    probe = Segment(SegKind.ARC, start, start, curvature=curvature)
    seg = Segment(SegKind.ARC, start, _arc_pose(probe, 250.0 * 0.8), curvature=curvature)
    x, y, theta, left, right, t = start.x, start.y, start.theta, 0.0, 0.0, 0.0
    follower = SegmentFollower()
    follower.reset(seg)
    for _ in range(1600):
        est = RoverEstimate(0, Pose(x, y, theta), v=(left + right) / 2.0,
                            omega=(right - left) / C.rover.track_width)
        v, omega, done = follower.step(est, t)
        if done:
            break
        vl = v - C.rover.track_width * omega / 2.0
        vr = v + C.rover.track_width * omega / 2.0
        left += (0.90 * vl - left) * 0.02 / 0.10
        right += (vr - right) * 0.02 / 0.10
        vm = (left + right) / 2.0
        wm = (right - left) / C.rover.track_width
        x += vm * math.cos(theta) * 0.02
        y += vm * math.sin(theta) * 0.02
        theta = wrap(theta + wm * 0.02)
        t += 0.02
    else:
        pytest.fail("ARC follower did not settle")
    # The controller has issued STOP; integrate the motor lag while stopped, as the
    # simulator/FSM does between ticks and the next segment.
    for _ in range(50):
        left += (-left) * 0.02 / 0.10
        right += (-right) * 0.02 / 0.10
        vm = (left + right) / 2.0
        wm = (right - left) / C.rover.track_width
        x += vm * math.cos(theta) * 0.02
        y += vm * math.sin(theta) * 0.02
        theta = wrap(theta + wm * 0.02)
    assert math.hypot(x - seg.end.x, y - seg.end.y) <= 5.0
    assert abs(angle_diff(theta, seg.end.theta)) <= math.radians(3.0)
