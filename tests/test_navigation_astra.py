"""ASTRA Gate 5: adversarial contracts, deliberately NOT xfailed.

Geometry oracles below do not call production footprint transforms, hull, SAT,
point-in-polygon, distance, or collision routines. Dense checks use <=0.01 degree
rotation and <=0.5 mm translation steps; constructed witnesses also put an exact
intermediate corner at the missed contact. Sampling is a counterexample oracle,
not a proof that arbitrary continuous motion is safe.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.control.controllers import AlignController, SegmentFollower
from rover_strategy.coordination.supervisor import Supervisor
from rover_strategy.geometry.footprint import Footprint
from rover_strategy.planning import navigation as nav
from rover_strategy.rover.fsm import RoverAgent
from rover_strategy.world import Path, Pose, RoverEstimate, SegKind, Segment


BW = BH = 860.0
FP0 = Footprint(C.rover)


def rect(x0, x1, y0, y1):
    return np.array([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], float)


def delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def corners(pose, inflate=0.0):
    local = rect(-47 - inflate, 102 + inflate, -49.75 - inflate, 49.75 + inflate)
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    return local @ np.array([[c, s], [-s, c]]) + [pose.x, pose.y]


def contains(poly, points, tolerance=1e-8):
    """Independent normalized half-plane test, distances in mm."""
    edge = np.roll(poly, -1, axis=0) - poly
    offset = np.asarray(points)[:, None, :] - poly[None, :, :]
    signed = (edge[None, :, 0] * offset[:, :, 1]
              - edge[None, :, 1] * offset[:, :, 0]) / np.linalg.norm(edge, axis=1)
    return bool(np.all(signed >= -tolerance))


def overlap(a, b):
    for poly in (a, b):
        for i, p in enumerate(poly):
            edge = poly[(i + 1) % len(poly)] - p
            normal = np.array([-edge[1], edge[0]])
            ap, bp = a @ normal, b @ normal
            if ap.max() < bp.min() - 1e-9 or bp.max() < ap.min() - 1e-9:
                return False
    return True


def independently_collides(pose, obstacles, inflate=0.0, margin=0.0):
    box = corners(pose, inflate)
    if (box.min() < margin - 1e-9 or box[:, 0].max() > BW - margin + 1e-9
            or box[:, 1].max() > BH - margin + 1e-9):
        return True
    for obstacle in obstacles:
        if isinstance(obstacle, nav.DiscObstacle):
            # Exact oriented-rectangle/disc distance via inverse rigid transform.
            dx, dy = obstacle.x - pose.x, obstacle.y - pose.y
            c, s = math.cos(pose.theta), math.sin(pose.theta)
            x, y = c * dx + s * dy, -s * dx + c * dy
            ex = max(-47 - inflate - x, 0.0, x - 102 - inflate)
            ey = max(abs(y) - 49.75 - inflate, 0.0)
            if math.hypot(ex, ey) <= obstacle.r + 1e-9:
                return True
        elif overlap(box, obstacle.poly):
            return True
    return False


def dense_poses(seg):
    if seg.kind is SegKind.ROTATE:
        angle = delta(seg.end.theta, seg.start.theta)
        count = max(1, math.ceil(abs(angle) / math.radians(0.01)))
        for t in np.linspace(0, 1, count + 1):
            yield Pose(seg.start.x, seg.start.y, seg.start.theta + t * angle)
    else:
        count = max(1, math.ceil(seg.length / 0.5))
        for t in np.linspace(0, 1, count + 1):
            yield Pose(seg.start.x + t * (seg.end.x - seg.start.x),
                       seg.start.y + t * (seg.end.y - seg.start.y), seg.start.theta)


def first_collision(path, obstacles, inflate=0.0, margin=0.0):
    for index, seg in enumerate(path.segments):
        for pose in dense_poses(seg):
            if independently_collides(pose, obstacles, inflate, margin):
                return index, pose
    return None


def corner_graze(mid_degrees=2.5, penetration=0.05):
    angle = math.atan2(49.75, 102) + math.radians(mid_degrees)
    radius = math.hypot(102, 49.75) + C.cube.half_diag - penetration
    return nav.DiscObstacle(430 + radius * math.cos(angle),
                            430 + radius * math.sin(angle), C.cube.half_diag)


@pytest.mark.parametrize("kind", ["disc", "polygon"])
def test_rotation_must_detect_contact_between_samples(kind):
    start = Pose(430, 430, 0)
    seg = Segment(SegKind.ROTATE, start, start.rotated_to(math.radians(10)))
    if kind == "disc":
        obstacle = corner_graze()
    else:
        # A long convex reservation with its near face tangent to the corner arc.
        a = math.atan2(49.75, 102) + math.radians(2.5)
        r = math.hypot(102, 49.75)
        local = rect(r - 0.02, r + 80, -150, 150)
        obstacle = nav.PolyObstacle(local @ np.array([[math.cos(a), math.sin(a)],
                                                      [-math.sin(a), math.cos(a)]]) + 430)
    assert not independently_collides(seg.start, [obstacle])
    assert not independently_collides(seg.end, [obstacle])
    assert independently_collides(start.rotated_to(math.radians(2.5)), [obstacle])
    public = nav.segment_collides(FP0, seg, [obstacle], BW, BH, 0)
    fast = nav._fast_segment_collides(nav._FastCache(FP0), seg,
                                      nav._fast_obstacles([obstacle]), BW, BH, 0)
    assert public and fast, f"intermediate contact missed: public={public}, fast={fast}"


def test_plan_returned_rotation_is_independently_collision_free():
    start = Pose(430, 430, 0)
    path = nav.NavPlanner().plan(start, start.rotated_to(math.radians(10)),
                                 [corner_graze()], BW, BH, inflate=0)
    assert path is not None
    hit = first_collision(path, [corner_graze()])
    assert hit is None, f"planner returned a colliding rotation: {hit}"


@pytest.mark.parametrize("margin", [0.0, C.margins.board])
def test_rotation_must_not_cross_board_between_samples(margin):
    radius = math.hypot(102, 49.75)
    peak = -math.atan2(49.75, 102)
    start = Pose(BW - margin - radius + 0.02, 430, peak - math.radians(2.5))
    seg = Segment(SegKind.ROTATE, start, start.rotated_to(start.theta + math.radians(10)))
    assert not independently_collides(seg.start, [], margin=margin)
    assert not independently_collides(seg.end, [], margin=margin)
    assert independently_collides(start.rotated_to(peak), [], margin=margin)
    assert nav.segment_collides(FP0, seg, [], BW, BH, margin), "corner leaves allowed board between samples"


@pytest.mark.parametrize("inflate", [0.0, C.margins.pose_uncertainty])
@pytest.mark.parametrize("degrees", [3.0, 90.0, -170.0])
def test_rotation_reservation_contains_every_dense_corner(inflate, degrees):
    start = Pose(430, 430, math.radians(179))
    seg = Segment(SegKind.ROTATE, start, start.rotated_to(start.theta + math.radians(degrees)))
    poly = nav.swept_polygons(Footprint(C.rover, inflate), Path([seg], 0))[0]
    for pose in dense_poses(seg):
        assert contains(poly, corners(pose, inflate)), f"unreserved corner at {pose}, inflation={inflate}"


@pytest.mark.parametrize("reverse", [False, True])
def test_straight_sweep_catches_thin_obstacle_between_endpoints(reverse):
    start = Pose(650 if reverse else 150, 430, 0)
    end = Pose(150 if reverse else 650, 430, 0)
    seg = Segment(SegKind.STRAIGHT, start, end, reverse=reverse)
    obstacle = nav.PolyObstacle(rect(420, 420.001, 400, 460))
    assert not independently_collides(start, [obstacle])
    assert not independently_collides(end, [obstacle])
    assert first_collision(Path([seg], 0), [obstacle]) is not None
    assert nav.segment_collides(FP0, seg, [obstacle], BW, BH, 0)
    assert nav._fast_segment_collides(nav._FastCache(FP0), seg,
                                     nav._fast_obstacles([obstacle]), BW, BH, 0)


@pytest.mark.parametrize("reverse", [False, True])
def test_straight_reservation_contains_dense_translations(reverse):
    start = Pose(430, 430, 0.37)
    seg = Segment(SegKind.STRAIGHT, start, start.moved(-180 if reverse else 180), reverse)
    poly = nav.swept_polygons(FP0, Path([seg], 0))[0]
    for pose in dense_poses(seg):
        assert contains(poly, corners(pose))


@pytest.mark.parametrize("dx", [20.0, -20.0])
def test_analytic_straight_is_executable_without_strafing(dx):
    start, goal = Pose(430, 430, 0), Pose(430 + dx, 448, 0)
    path = nav.NavPlanner().plan(start, goal, [], BW, BH)
    assert path is not None
    for seg in path.segments:
        if seg.kind is SegKind.STRAIGHT:
            along, lateral = seg.start.to_local(seg.end.x, seg.end.y)
            assert abs(lateral) < 1e-6, f"differential drive cannot strafe {lateral} mm: {seg}"
            assert seg.reverse == (along < 0)
            assert abs(delta(seg.end.theta, seg.start.theta)) < 1e-6


def test_actual_follower_motion_stays_in_even_inflated_reservation():
    path = nav.NavPlanner().plan(Pose(300, 300, 0), Pose(340, 318, 0), [], BW, BH)
    assert path is not None
    polys = nav.swept_polygons(Footprint(C.rover, C.margins.pose_uncertainty), path)
    pose = path.segments[0].start
    dt, t = 0.005, 0.0
    follower = SegmentFollower()
    for seg in path.segments:
        follower.reset(seg)
        for _ in range(10000):
            v, w, done = follower.step(RoverEstimate(1, pose), t)
            if done:
                break
            if seg.kind is SegKind.STRAIGHT:
                # This witness is below the FSM's deviation thresholds, so the
                # FSM would actually allow these commands to reach the guard.
                _, cross, head = follower.tracking_error()
                assert abs(cross) <= 35 and abs(head) <= math.radians(30)
            if abs(w) < 1e-12:
                x, y = pose.x + v * dt * math.cos(pose.theta), pose.y + v * dt * math.sin(pose.theta)
            else:
                x = pose.x + v / w * (math.sin(pose.theta + w * dt) - math.sin(pose.theta))
                y = pose.y - v / w * (math.cos(pose.theta + w * dt) - math.cos(pose.theta))
            pose = Pose(x, y, pose.theta + w * dt)
            t += dt
            for point in corners(pose):
                assert any(contains(poly, [point]) for poly in polys), f"follower leaves reservation at t={t}: {pose}"
        else:
            pytest.fail("ideal follower did not finish")


def test_escape_does_not_drive_into_a_real_cube():
    start = Pose(430, 430, 0)
    obstacles = [corner_graze(2.0, 0.03),
                 nav.DiscObstacle(430 - 47 - C.cube.half_diag - 1, 430 - 49.75, C.cube.half_diag)]
    assert not independently_collides(start, obstacles)
    assert independently_collides(start, obstacles, inflate=C.margins.pose_uncertainty)
    path = nav.NavPlanner().plan(start, start.rotated_to(math.radians(24)), obstacles, BW, BH)
    assert path is not None
    hit = first_collision(path, obstacles)
    assert hit is None, f"escape crosses real obstacle despite increasing endpoint clearance: {hit}"


def test_heading_cache_cannot_hide_board_collision():
    pose = Pose(430, BH - 49.75 - 0.0002, 0.000004)
    cache = nav._FastCache(FP0)
    cache.poly(pose.x, pose.y, 0.0)  # Same rounded key as pose.theta.
    assert independently_collides(pose, [])
    assert nav.pose_collides(FP0, pose, [], BW, BH, 0)
    assert nav._fast_pose_collides(cache, pose, [], BW, BH, 0), "warm cache uses wrong heading"


def test_heading_cache_result_is_independent_of_insertion_order():
    pose = Pose(430, BH - 49.75 - 0.0002, 0.000004)
    warm, cold = nav._FastCache(FP0), nav._FastCache(FP0)
    warm.poly(pose.x, pose.y, 0)
    assert nav._fast_pose_collides(warm, pose, [], BW, BH, 0) == nav._fast_pose_collides(cold, pose, [], BW, BH, 0)


def test_deadline_covers_analytic_connector_and_return(monkeypatch):
    # A deterministic clock models an expensive connector without a flaky sleep.
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(nav, "time", SimpleNamespace(monotonic=lambda: clock.now))
    planner = nav.NavPlanner()
    original = planner._analytic_connect

    def expensive_connector(*args, **kwargs):
        result = original(*args, **kwargs)
        clock.now += 1.0
        return result

    monkeypatch.setattr(planner, "_analytic_connect", expensive_connector)
    path = planner.plan(Pose(200, 300, 0), Pose(500, 300, 0), [], BW, BH, deadline_s=0.1)
    assert path is None and "deadline" in planner.last_failure, "successful result returned after deadline without rechecking time"


def test_expired_budget_does_not_enter_escape(monkeypatch):
    clock = SimpleNamespace(now=0.0)

    def monotonic():
        clock.now += 1.0
        return clock.now

    monkeypatch.setattr(nav, "time", SimpleNamespace(monotonic=monotonic))
    planner = nav.NavPlanner()
    entered = []
    original = planner._escape

    def record_escape(*args, **kwargs):
        entered.append(True)
        return original(*args, **kwargs)

    monkeypatch.setattr(planner, "_escape", record_escape)
    planner.plan(Pose(300, 300, 0), Pose(300, 500, 0),
                 [nav.DiscObstacle(406, 300, 1)], BW, BH, deadline_s=0.0)
    assert not entered, "escape search runs before checking an already expired deadline"


def test_heuristic_is_lower_bound_for_actual_accepted_goal(monkeypatch):
    priorities = []
    original = nav.heapq.heappush

    def record(heap, item):
        priorities.append(item[0])
        return original(heap, item)

    monkeypatch.setattr(nav.heapq, "heappush", record)
    start, goal = Pose(300, 300, 0), Pose(500, 300, math.radians(2))
    path = nav.NavPlanner().plan(start, goal, [], BW, BH)
    assert path is not None
    assert priorities[0] <= path.cost_s + 1e-9, f"h={priorities[0]} exceeds feasible accepted cost {path.cost_s}"


def test_first_analytic_connection_does_not_beat_a_cheaper_legal_lattice_path():
    start, goal = Pose(430, 430, 0), Pose(470, 470, math.pi / 2)
    bend = Pose(470, 430, 0)
    turned = bend.rotated_to(math.pi / 2)
    # All three moves are actual planner primitives: 40 mm, goal-heading turn,
    # 40 mm. One settle rather than the analytic connector's two settles.
    alternative = Path([Segment(SegKind.STRAIGHT, start, bend),
                        Segment(SegKind.ROTATE, bend, turned),
                        Segment(SegKind.STRAIGHT, turned, goal)],
                       80 / C.limits.v_nav + (math.pi / 2) / C.limits.w_nav
                       + nav.NavParams().rotate_settle_s)
    assert first_collision(alternative, [], 6, 12) is None
    path = nav.NavPlanner().plan(start, goal, [], BW, BH)
    assert path is not None
    assert path.cost_s <= alternative.cost_s + 1e-9, f"first connector costs {path.cost_s:.6f}s; legal lattice path costs {alternative.cost_s:.6f}s"


def test_fsm_nav_goal_tolerance_does_not_require_alignment_off_board():
    # ALIGN mitigates the 18 mm tolerance in open space, but its corrective nudge
    # must also be feasible at the board edge. Test actual controller commands.
    start, goal = Pose(300, 86, 0), Pose(300, 68, 0)
    path = nav.NavPlanner().plan(start, goal, [], BW, BH)
    assert path is not None
    ctx = SimpleNamespace(plan_nav=lambda *_: path, commit_path=lambda *_: True)
    agent = RoverAgent(1, C)
    _, _, status = agent._navigate_to(0.0, RoverEstimate(1, start), ctx, goal, [])
    assert status in ("arrived", "moving")
    # Start ALIGN at the endpoint the planner promises, with perfect tracking.
    pose = path.goal
    align = AlignController()
    align.reset((goal.x + C.prepush_distance, goal.y), goal.theta)
    v = w = 0.0
    dt = 0.01
    for k in range(4000):
        v, w, done, phase = align.step(RoverEstimate(1, pose, v, w), k * dt)
        if done:
            assert abs(goal.to_local(pose.x, pose.y)[1]) <= C.rover.inner_half_width - C.cube.half_diag
            break
        # AlignController emits pure rotations or translations (never an arc).
        pose = Pose(pose.x + v * dt * math.cos(pose.theta),
                    pose.y + v * dt * math.sin(pose.theta), pose.theta + w * dt)
        assert not independently_collides(pose, []), f"accepted nav goal needs off-board ALIGN at t={k * dt}, phase={phase}, pose={pose}"
    else:
        pytest.fail("alignment did not finish within 40 simulated seconds")


@pytest.mark.parametrize("heading", [math.pi / 2, math.pi, -math.pi / 2])
def test_fsm_already_at_goal_arrives_at_every_heading(heading):
    pose = Pose(430, 430, heading)
    path = nav.NavPlanner().plan(pose, pose, [], BW, BH)
    assert path is not None
    ctx = SimpleNamespace(plan_nav=lambda *_: path, commit_path=lambda *_: True,
                          log=lambda *args, **kwargs: None)
    agent = RoverAgent(1, C)
    for tick in range(40):
        _, _, status = agent._navigate_to(tick * 0.01, RoverEstimate(1, pose), ctx, pose, [])
        if status == "arrived":
            break
    assert status == "arrived", f"already at exact goal heading={heading}, got {status}"


def test_align_has_a_tighter_lateral_target_than_capture_slack():
    # Existing mitigation: do not falsely claim FSM immediately captures at 18 mm.
    assert AlignController().params.align_lateral_tol < C.rover.inner_half_width - C.cube.half_diag


@pytest.mark.parametrize("case", ["uncertainty", "board_margin", "other_reservation"])
def test_supervisor_revalidation_preserves_planning_contract(case):
    supervisor = Supervisor([1, 2])
    start, end = Pose(300, 300, 0), Pose(500, 300, 0)
    obstacles = []
    if case == "uncertainty":
        obstacles = [nav.DiscObstacle(450, 352.75 + C.cube.half_diag, C.cube.half_diag)]
    elif case == "board_margin":
        start, end = Pose(65, 65, 0), Pose(200, 65, 0)
    else:
        supervisor.res.reserve(2, [rect(420, 450, 250, 350)], 1, 0)
    path = Path([Segment(SegKind.STRAIGHT, start, end)], 1)
    supervisor._cube_obstacles = lambda: obstacles
    all_obstacles = obstacles + supervisor._other_region(1)
    assert first_collision(path, all_obstacles, C.margins.pose_uncertainty, C.margins.board) is not None
    assert not supervisor.path_still_clear(1, path, 0), f"revalidation dropped {case}"


def test_inflation_and_board_margin_are_additive():
    # Default rear constraint is x >= 47 + 6 + 12 = 65, not just centre-in-board.
    fp = Footprint(C.rover, C.margins.pose_uncertainty)
    for x, collision in [(64.9, True), (65.1, False)]:
        pose = Pose(x, 300, 0)
        assert independently_collides(pose, [], 6, 12) == collision
        assert nav.pose_collides(fp, pose, [], BW, BH, 12) == collision


def test_corner_reverse_escape_is_swept_checked():
    start, goal = Pose(740, 68, 0), Pose(500, 68, 0)
    planner = nav.NavPlanner()
    path = planner.plan(start, goal, [], BW, BH)
    assert path is not None
    assert any(seg.reverse for seg in path.segments)
    assert first_collision(path, [], 6, 12) is None
    # Obstacle immediately behind the axle footprint makes the start genuinely invalid.
    blocked = planner.plan(start, goal, [nav.DiscObstacle(690, 68, 10)], BW, BH)
    assert blocked is None


def test_closed_barrier_never_returns_path_through_narrow_gap():
    obstacles = [nav.PolyObstacle(rect(0, 395, 400, 460)),
                 nav.PolyObstacle(rect(465, 860, 400, 460))]
    planner = nav.NavPlanner(nav.DEFAULT, nav.NavParams(max_expansions=80))
    path = planner.plan(Pose(430, 200, math.pi / 2), Pose(430, 650, math.pi / 2),
                        obstacles, BW, BH, deadline_s=5)
    assert path is None, "70 mm gap cannot pass a 99.5 mm rover"
    assert planner.last_failure


def test_seeded_open_paths_are_continuously_safe_and_deterministic():
    rng = np.random.default_rng(507)
    for _ in range(5):
        start = Pose(*rng.uniform(250, 600, 2), rng.uniform(-math.pi, math.pi))
        goal = Pose(*rng.uniform(250, 600, 2), rng.uniform(-math.pi, math.pi))
        first = nav.NavPlanner().plan(start, goal, [], BW, BH, deadline_s=5)
        second = nav.NavPlanner().plan(start, goal, [], BW, BH, deadline_s=5)
        assert first is not None and second is not None
        assert first == second
        assert first_collision(first, [], 6, 12) is None
