import math
import time

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.geometry import shapes as S
from rover_strategy.geometry.footprint import Footprint
from rover_strategy.world import Path, Pose, SegKind, Segment
from rover_strategy.planning.navigation import (
    DiscObstacle,
    NavParams,
    NavPlanner,
    PolyObstacle,
    path_collides,
    pose_collides,
    segment_collides,
    swept_polygons,
)

FP = Footprint(C.rover, inflate=C.margins.pose_uncertainty)   # the safety margin NavPlanner uses by default
FP0 = Footprint(C.rover, inflate=0.0)                          # "never actually touch anything" ground truth
BW = BH = C.board.width


def dense_collision_free(path: Path, fp: Footprint, obstacles, board_w=BW, board_h=BH,
                          margin=C.margins.board) -> bool:
    """Independent, finer-grained re-check of the whole path: samples every segment far more densely
    than segment_collides' own internal sampling and re-checks each sample with plain pose_collides,
    so a bug in segment_collides' sweep logic wouldn't be masked by re-using it as the oracle."""
    for seg in path.segments:
        if seg.kind is SegKind.ROTATE:
            dtheta = seg.end.theta - seg.start.theta
            # unwrap the short way, matching Segment semantics (small deltas in practice)
            while dtheta > math.pi:
                dtheta -= 2 * math.pi
            while dtheta < -math.pi:
                dtheta += 2 * math.pi
            for k in range(41):
                th = seg.start.theta + dtheta * k / 40.0
                if pose_collides(fp, Pose(seg.start.x, seg.start.y, th), obstacles, board_w, board_h, margin):
                    return False
        else:
            for k in range(41):
                t = k / 40.0
                x = seg.start.x + (seg.end.x - seg.start.x) * t
                y = seg.start.y + (seg.end.y - seg.start.y) * t
                if pose_collides(fp, Pose(x, y, seg.start.theta), obstacles, board_w, board_h, margin):
                    return False
    return True


def ends_near(path: Path, goal: Pose, pos_tol=25.0, theta_tol=math.radians(8.0)) -> bool:
    p = path.goal
    d = math.hypot(p.x - goal.x, p.y - goal.y)
    dth = abs(math.atan2(math.sin(p.theta - goal.theta), math.cos(p.theta - goal.theta)))
    return d <= pos_tol and dth <= theta_tol


# ------------------------------------------------------------------------------------------- basics

def test_straight_line():
    p = NavPlanner()
    start, goal = Pose(100, 100, 0.0), Pose(400, 100, 0.0)
    path = p.plan(start, goal, [], BW, BH)
    assert path is not None
    assert dense_collision_free(path, FP, [])
    assert ends_near(path, goal)
    # a pure straight shot shouldn't need to search or rotate
    assert all(s.kind is SegKind.STRAIGHT for s in path.segments)
    assert path.cost_s == pytest.approx(300.0 / C.limits.v_nav, rel=1e-6)


def test_already_at_goal_returns_well_formed_path():
    """Start already within tolerance of goal: must not crash (Path.goal reads segments[-1]) and
    must report ~zero cost."""
    p = NavPlanner()
    pose = Pose(100.0, 100.0, 0.0)
    path = p.plan(pose, pose, [], BW, BH)
    assert path is not None
    assert len(path.segments) >= 1
    assert path.cost_s == pytest.approx(0.0, abs=1e-6)
    assert ends_near(path, pose)


def test_heading_change_in_place():
    p = NavPlanner()
    start, goal = Pose(200, 200, 0.0), Pose(200, 200, math.pi / 2)
    path = p.plan(start, goal, [], BW, BH)
    assert path is not None
    assert dense_collision_free(path, FP, [])
    assert ends_near(path, goal)
    assert any(s.kind is SegKind.ROTATE for s in path.segments)


# ------------------------------------------------------------------------------------- obstacle avoidance

def test_around_cube_obstacle_dense_clearance():
    p = NavPlanner()
    start, goal = Pose(100, 430, 0.0), Pose(700, 430, 0.0)
    cube_r = C.cube.half_diag + C.margins.cube_nav          # unknown-orientation cube, inflated per spec
    obs = [DiscObstacle(400, 430, cube_r)]
    path = p.plan(start, goal, obs, BW, BH)
    assert path is not None
    assert not path_collides(FP, path, obs, BW, BH, C.margins.board)
    assert dense_collision_free(path, FP, obs)                # not just the waypoints
    assert ends_near(path, goal)
    # actually had to detour: the direct line would have gone straight through the cube
    assert len(path.segments) > 1


def test_around_cube_never_touches_non_target_cube():
    """A cube-radius disc placed to force a real detour; check every dense sample keeps the whole
    swept envelope strictly outside the (inflated) cube disc, i.e. it never even touches it."""
    p = NavPlanner()
    start, goal = Pose(150, 200, 0.0), Pose(150, 700, 0.0)
    cube_r = C.cube.half_diag + C.margins.cube_nav
    obs = [DiscObstacle(150, 450, cube_r)]
    path = p.plan(start, goal, obs, BW, BH)
    assert path is not None
    assert dense_collision_free(path, FP, obs)
    assert ends_near(path, goal)


# ---------------------------------------------------------------------------------------- narrow gap

def test_narrow_gap_impassable_fails_or_detours():
    """A wall spanning the full board width except for a 70 mm gap -- narrower than the ~99.5 mm-wide
    rover envelope, with no room to go around the wall's ends -- must not be threaded. Either the
    planner reports failure, or (if it found some other legal route) that route is genuinely
    collision-free and reaches the goal; it must never report success through the gap."""
    gap = 70.0
    cx = 430.0
    wall_l = PolyObstacle(S.rect(0.0, cx - gap / 2, 400.0, 460.0))
    wall_r = PolyObstacle(S.rect(cx + gap / 2, BW, 400.0, 460.0))
    obs = [wall_l, wall_r]
    p = NavPlanner()
    start, goal = Pose(430, 200, math.pi / 2), Pose(430, 650, math.pi / 2)
    path = p.plan(start, goal, obs, BW, BH, deadline_s=0.4)
    if path is None:
        assert p.last_failure
    else:
        assert dense_collision_free(path, FP, obs)
        assert ends_near(path, goal)


# --------------------------------------------------------------------------------------- board edges

def test_goal_near_edge_requires_moving_inward_before_rotating():
    """Goal pose sits close enough to the left board edge that ANY in-place rotation performed
    there collides with the margin, even though the goal pose itself is fine standing still. The
    only legal way in is to reach the goal heading somewhere with room to spare, then drive straight
    in already aligned. Confirm the premise, then confirm the planner still solves it."""
    goal = Pose(66.0, 430.0, 0.0)
    turning_here = Segment(SegKind.ROTATE, Pose(66.0, 430.0, math.pi), goal)
    assert not pose_collides(FP, goal, [], BW, BH, C.margins.board)               # goal itself is fine
    assert segment_collides(FP, turning_here, [], BW, BH, C.margins.board)        # but can't turn there

    p = NavPlanner()
    start = Pose(300.0, 430.0, math.pi)                                          # needs a 180 deg turn
    path = p.plan(start, goal, [], BW, BH)
    assert path is not None
    assert dense_collision_free(path, FP, [])
    assert ends_near(path, goal)
    # the turn must have happened away from the tight spot, not at/right next to the goal
    turns = [s for s in path.segments if s.kind is SegKind.ROTATE]
    assert turns
    assert all(abs(s.start.x - goal.x) > 30.0 for s in turns)


# -------------------------------------------------------------------------------- start-in-collision

def test_start_escape_from_inflated_only_collision():
    """Start pose collides under the normal safety inflate but not at zero inflation (e.g. rover
    just backed off a cube and the safety margin still overlaps it slightly). The planner must
    escape by increasing clearance, never actually touching anything, and then reach the goal."""
    inflate = C.margins.pose_uncertainty
    front_tip = C.rover.x_paddle_tip
    obs = [DiscObstacle(300.0 + front_tip + inflate - 2.0, 300.0, 1.0)]
    start, goal = Pose(300.0, 300.0, 0.0), Pose(300.0, 500.0, 0.0)

    fp_inflate = Footprint(C.rover, inflate=inflate)
    assert pose_collides(fp_inflate, start, obs, BW, BH, C.margins.board)          # margin-only collision
    assert not pose_collides(FP0, start, obs, BW, BH, C.margins.board)             # never a real one

    p = NavPlanner()
    path = p.plan(start, goal, obs, BW, BH)
    assert path is not None
    assert dense_collision_free(path, FP0, obs)                                    # real safety: never touches
    assert ends_near(path, goal)


def test_start_truly_in_collision_returns_none():
    """Start pose overlapping an obstacle even at zero inflation is a real collision; escape cannot
    make that safe, so plan() must fail with a reason rather than silently drive through it."""
    obs = [DiscObstacle(300.0, 300.0, 30.0)]      # centred right on the rotation centre
    p = NavPlanner()
    path = p.plan(Pose(300.0, 300.0, 0.0), Pose(600.0, 300.0, 0.0), obs, BW, BH)
    assert path is None
    assert p.last_failure


# ------------------------------------------------------------------------------------ goal collision

def test_goal_in_collision_returns_none_with_reason():
    obs = [DiscObstacle(400.0, 400.0, 50.0)]
    p = NavPlanner()
    path = p.plan(Pose(100.0, 100.0, 0.0), Pose(400.0, 400.0, 0.0), obs, BW, BH)
    assert path is None
    assert "goal" in p.last_failure.lower()


# ---------------------------------------------------------------------------------------- deadline

def test_deadline_is_respected():
    gap = 70.0
    cx = 430.0
    obs = [PolyObstacle(S.rect(0.0, cx - gap / 2, 400.0, 460.0)),
           PolyObstacle(S.rect(cx + gap / 2, BW, 400.0, 460.0))]
    p = NavPlanner()
    deadline = 0.15
    t0 = time.monotonic()
    path = p.plan(Pose(430, 200, math.pi / 2), Pose(430, 650, math.pi / 2), obs, BW, BH, deadline_s=deadline)
    elapsed = time.monotonic() - t0
    assert elapsed <= deadline + 0.25          # generous slack for python/test-runner overhead
    if path is None:
        assert p.last_failure


# ------------------------------------------------------------------------------------ determinism

def test_deterministic():
    obs = [DiscObstacle(400, 430, C.cube.half_diag + C.margins.cube_nav)]
    start, goal = Pose(100, 430, 0.0), Pose(700, 430, 0.0)
    a = NavPlanner().plan(start, goal, obs, BW, BH)
    b = NavPlanner().plan(start, goal, obs, BW, BH)
    assert a is not None and b is not None
    assert len(a.segments) == len(b.segments)
    assert a.cost_s == pytest.approx(b.cost_s, rel=1e-9)
    for sa, sb in zip(a.segments, b.segments):
        assert sa.kind == sb.kind and sa.reverse == sb.reverse
        assert sa.start.x == pytest.approx(sb.start.x) and sa.end.x == pytest.approx(sb.end.x)
        assert sa.start.y == pytest.approx(sb.start.y) and sa.end.y == pytest.approx(sb.end.y)
        assert sa.start.theta == pytest.approx(sb.start.theta) and sa.end.theta == pytest.approx(sb.end.theta)


# ---------------------------------------------------------------------------------------- perf

def test_typical_query_is_fast():
    """PERFORMANCE requirement: an open field with a few cube obstacles should plan well under the
    0.5 s budget (deadline_s is a hard backstop, not the expected runtime)."""
    rng = np.random.default_rng(0)
    times = []
    for _ in range(10):
        sx, sy = rng.uniform(100, 760, size=2)
        gx, gy = rng.uniform(100, 760, size=2)
        obs = [DiscObstacle(float(x), float(y), C.cube.half_diag + C.margins.cube_nav)
               for x, y in rng.uniform(150, 710, size=(3, 2))]
        p = NavPlanner()
        t0 = time.monotonic()
        p.plan(Pose(sx, sy, 0.0), Pose(gx, gy, rng.uniform(-math.pi, math.pi)), obs, BW, BH, deadline_s=0.5)
        times.append(time.monotonic() - t0)
    assert max(times) < 0.5, f"slowest query took {max(times):.3f}s: {times}"


# ------------------------------------------------------------------------------------------- fuzz

def test_random_fuzz_seeded():
    rng = np.random.default_rng(1234)
    n_cases = 50
    successes = 0
    for i in range(n_cases):
        margin = C.rover.x_paddle_tip + 20.0     # keep random poses comfortably inside the field
        sx, sy = rng.uniform(margin, BW - margin, size=2)
        gx, gy = rng.uniform(margin, BH - margin, size=2)
        sth = float(rng.uniform(-math.pi, math.pi))
        gth = float(rng.uniform(-math.pi, math.pi))
        n_obs = int(rng.integers(0, 4))
        obs = []
        for _ in range(n_obs):
            ox, oy = rng.uniform(80, BW - 80, size=2)
            obs.append(DiscObstacle(float(ox), float(oy), C.cube.half_diag + C.margins.cube_nav))
        start, goal = Pose(float(sx), float(sy), sth), Pose(float(gx), float(gy), gth)

        p = NavPlanner()
        path = p.plan(start, goal, obs, BW, BH, deadline_s=0.5)
        if pose_collides(FP, start, obs, BW, BH, C.margins.board) or pose_collides(FP, goal, obs, BW, BH,
                                                                                     C.margins.board):
            continue   # invalid random case (start/goal itself unreachable); not what this test checks
        if path is None:
            continue   # legitimate planning failure (e.g. random obstacles boxed the goal in) is allowed
        successes += 1
        assert dense_collision_free(path, FP, obs), f"case {i} produced a colliding path"
        assert ends_near(path, goal), f"case {i} did not reach the goal"

    assert successes >= n_cases * 0.5, f"only {successes}/{n_cases} valid cases were solved"


# --------------------------------------------------------------------------------------- swept_polygons

def test_swept_polygons_cover_each_segment():
    p = NavPlanner()
    path = p.plan(Pose(200, 200, 0.0), Pose(200, 200, math.pi / 2), [], BW, BH)
    assert path is not None
    polys = swept_polygons(FP, path)
    assert len(polys) == len(path.segments)
    for poly, seg in zip(polys, path.segments):
        assert S.point_in(poly, FP.envelope(seg.start.x, seg.start.y, seg.start.theta).mean(axis=0))
        assert S.point_in(poly, FP.envelope(seg.end.x, seg.end.y, seg.end.theta).mean(axis=0))


# ------------------------------------------------------------------------------- fast/slow parity guard

def test_fast_path_matches_public_api_on_dense_samples():
    """Regression guard for the internal pure-python fast collision path: re-checks a solved path's
    every segment with the public (numpy) pose_collides/segment_collides, which must agree that
    nothing collides -- this is exactly the bug class (`_escape` once validated only the destination
    pose, not the swept segment) that a plain success/failure test would miss."""
    obs = [DiscObstacle(400, 430, C.cube.half_diag + C.margins.cube_nav)]
    p = NavPlanner()
    path = p.plan(Pose(100, 430, 0.0), Pose(700, 430, 0.0), obs, BW, BH)
    assert path is not None
    for seg in path.segments:
        assert not segment_collides(FP, seg, obs, BW, BH, C.margins.board)
