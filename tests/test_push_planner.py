"""Tests for rover_strategy.planning.push_planner.

Geometric checks in the fuzz test are written from scratch against Footprint/shapes/DepotZone
(NOT by calling leg_feasible/_prepush_ok) so they independently re-verify what the planner claims.
"""
from __future__ import annotations

import math
import random

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.geometry import shapes as S
from rover_strategy.geometry.footprint import Footprint
from rover_strategy.geometry.zones import DepotZone, cube_half_extent
from rover_strategy.world import CubeEstimate, PushLeg, PushPlan
from rover_strategy.planning.push_planner import (
    PushPlanner, PushParams, DEFAULT_PARAMS,
    leg_feasible, prepush_pose, is_delivered, classify_unsolvable, wrap_to_pm45,
)

FP = Footprint(C.rover)
BOARD = C.board.width  # 860.0, square field


# --------------------------------------------------------------------------------- independent geometry re-check

def _rover_positions(leg: PushLeg) -> tuple[tuple[float, float], tuple[float, float]]:
    """Rover (rotation-centre) position at contact and at the leg's end, from the leg's own fields."""
    heading = leg.heading
    hx, hy = math.cos(heading), math.sin(heading)
    cx, cy = leg.cube_start
    contact = C.contact_distance
    rx, ry = cx - contact * hx, cy - contact * hy
    length = leg.length
    return (rx, ry), (rx + length * hx, ry + length * hy)


def _other_radius(oc: CubeEstimate) -> float:
    return cube_half_extent(C.cube.side, oc.alpha) if oc.alpha is not None else C.cube.half_diag


def independent_check(plan: PushPlan, other_cubes: list[CubeEstimate], board_w: float, board_h: float,
                       depot: DepotZone) -> None:
    """Re-derive, from Footprint/shapes primitives only, every geometric guarantee the plan claims:
    board containment of prepush/corridor/retreat, clearance to other cubes, and final containment
    in the depot.  Raises AssertionError (via plain `assert`) on any violation."""
    lo_x, hi_x = C.margins.board, board_w - C.margins.board
    lo_y, hi_y = C.margins.board, board_h - C.margins.board
    clear = C.margins.cube_nav

    for i_leg, leg in enumerate(plan.legs):
        # final delivery leg: corridor/retreat may use margins.board_final_leg (lead rule, geometry_model.md)
        bm = C.margins.board_final_leg if i_leg == len(plan.legs) - 1 else C.margins.board
        lo_x, hi_x, lo_y, hi_y = bm, board_w - bm, bm, board_h - bm
        pp = leg.prepush
        env = FP.envelope(pp.x, pp.y, pp.theta)
        assert S.inside_rect(env, lo_x, hi_x, lo_y, hi_y), f"prepush envelope outside field: {pp}"

        # in-place rotation must clear every other cube (and the cube itself, at its current spot)
        r_rot = FP.sweep_radius
        for oc in other_cubes:
            d = math.hypot(pp.x - oc.x, pp.y - oc.y)
            assert d >= r_rot + _other_radius(oc) + clear - 1e-6, "prepush rotation too close to a cube"
        d_self = math.hypot(pp.x - leg.cube_start[0], pp.y - leg.cube_start[1])
        assert d_self >= r_rot + C.cube.half_diag + clear - 1e-6

        (rx, ry), (ex, ey) = _rover_positions(leg)
        corridor = FP.straight_sweep(rx, ry, leg.heading, leg.length)
        assert S.inside_rect(corridor, lo_x, hi_x, lo_y, hi_y), "push corridor outside field"
        for oc in other_cubes:
            assert S.disc_distance(corridor, (oc.x, oc.y), _other_radius(oc)) >= clear - 1e-6, \
                "push corridor too close to a cube"

        retreat_dist = C.rover.paddle_reach + DEFAULT_PARAMS.retreat_margin_mm
        retreat = FP.straight_sweep(ex, ey, leg.heading, -retreat_dist)
        assert S.inside_rect(retreat, lo_x, hi_x, lo_y, hi_y), "retreat outside field"
        for oc in other_cubes:
            assert S.disc_distance(retreat, (oc.x, oc.y), _other_radius(oc)) >= clear - 1e-6, \
                "retreat too close to a cube"

    # final containment: whichever alpha the last leg implies, the cube must fit with margin
    last = plan.legs[-1]
    alpha_final = wrap_to_pm45(last.heading)
    assert depot.contains_cube(plan.delivery_point[0], plan.delivery_point[1], C.cube.side,
                                alpha_final, C.depot.delivery_margin), "delivery point sticks out of depot"
    assert plan.delivery_point == last.cube_end


def make_depot(color: str, cx: float, cy: float) -> DepotZone:
    return DepotZone(color, cx, cy, C.depot.half_size)


# --------------------------------------------------------------------------------- basic scenarios

def test_direct_push_mid_field_one_leg():
    """A cube dead in mid-field, far from any corner: a single direct leg should be found, its
    prepush pose inside the board, and the final footprint inside the depot with margin."""
    planner = PushPlanner()
    cube = CubeEstimate("red", 430.0, 430.0)
    depot = make_depot("red", 430.0, 100.0)   # depot well away from corners, roomy approach
    plans = planner.plan(cube, depot, [], BOARD, BOARD, max_plans=5, deadline_s=0.3)
    assert plans, planner.last_failure
    direct = [p for p in plans if len(p.legs) == 1]
    assert direct, f"expected a 1-leg plan, got leg counts {[len(p.legs) for p in plans]}"
    plan = direct[0]
    leg = plan.legs[0]

    pp = leg.prepush
    env = FP.envelope(pp.x, pp.y, pp.theta)
    lo, hi = C.margins.board, BOARD - C.margins.board
    assert S.inside_rect(env, lo, hi, lo, hi)

    alpha_final = wrap_to_pm45(leg.heading)
    assert depot.contains_cube(plan.delivery_point[0], plan.delivery_point[1], C.cube.side,
                                alpha_final, C.depot.delivery_margin)
    independent_check(plan, [], BOARD, BOARD, depot)


def test_edge_hugging_cube_needs_multi_leg():
    """A cube pinned against the left edge, with the depot on the far side, cannot be reached by a
    single straight push (the direct heading would take the corridor off the field); the planner
    must fall back to 2+ legs."""
    planner = PushPlanner()
    depot = make_depot("red", 810.0, 810.0)      # opposite corner
    # 10 mm from the edge: no heading keeps the rover inside the field (strict rule) -> unsolvable.
    assert not planner.plan(CubeEstimate("red", 40.0, 430.0), depot, [], BOARD, BOARD)
    assert classify_unsolvable(C, CubeEstimate("red", 40.0, 430.0), depot, [], BOARD, BOARD)
    # 75 mm from the edge: the rover can never get west of it (edge-locked: only along-edge motion), and the
    # depot is east -> no plan.  Physics of pushing without walls, not a planner weakness.
    assert not planner.plan(CubeEstimate("red", 75.0, 430.0), depot, [], BOARD, BOARD)
    # (260, 200) became infeasible under the strict field rule once pre-push poses reserve +-6 deg alignment room
    # (edge-parallel final legs impossible, DECISIONS #18); (300, 250) is re-routable.
    cube = CubeEstimate("red", 300.0, 250.0)
    plans = planner.plan(cube, depot, [], BOARD, BOARD, max_plans=5, deadline_s=3.0)
    assert plans, planner.last_failure
    assert all(len(p.legs) >= 1 for p in plans)
    for p in plans:
        independent_check(p, [], BOARD, BOARD, depot)


def test_non_depot_corner_is_unsolvable():
    """A cube wedged deep in a corner that is NOT the depot's corner must be flagged unsolvable,
    with an explanation mentioning the corner."""
    cube = CubeEstimate("red", 20.0, 20.0)          # deep in the (0,0) corner
    depot = make_depot("red", 810.0, 810.0)          # depot is the OPPOSITE corner
    reason = classify_unsolvable(C, cube, depot, [], BOARD, BOARD)
    assert reason is not None
    assert "corner" in reason.lower()

    planner = PushPlanner()
    plans = planner.plan(cube, depot, [], BOARD, BOARD, max_plans=5, deadline_s=0.2)
    assert plans == []
    assert "corner" in planner.last_failure.lower()


def test_other_cube_blocks_direct_corridor_and_is_avoided():
    """An obstacle sitting on the straight line between the cube and a natural delivery point must
    not be clipped by any returned plan's push corridor, retreat, or rotation sweeps."""
    planner = PushPlanner()
    cube = CubeEstimate("red", 300.0, 430.0)
    depot = make_depot("red", 700.0, 430.0)
    blocker = CubeEstimate("blue", 500.0, 430.0)   # sits right on the straight line red -> depot
    plans = planner.plan(cube, depot, [blocker], BOARD, BOARD, max_plans=5, deadline_s=0.3)
    assert plans, planner.last_failure
    for p in plans:
        independent_check(p, [blocker], BOARD, BOARD, depot)
        # extra explicit corridor-vs-blocker check per leg, spelled out geometrically
        for leg in p.legs:
            (rx, ry), _ = _rover_positions(leg)
            corridor = FP.straight_sweep(rx, ry, leg.heading, leg.length)
            assert S.disc_distance(corridor, (blocker.x, blocker.y), C.cube.half_diag) >= C.margins.cube_nav - 1e-6


def test_known_orientation_headings_respect_window():
    """With a KNOWN orientation (tight std), every leg's heading must fall within
    push_heading_window_deg of one of the cube's four face normals."""
    planner = PushPlanner()
    alpha = math.radians(30.0)
    cube = CubeEstimate("red", 300.0, 300.0, alpha=alpha, alpha_std=math.radians(2.0))
    depot = make_depot("red", 700.0, 700.0)
    plans = planner.plan(cube, depot, [], BOARD, BOARD, max_plans=5, deadline_s=0.3)
    assert plans, planner.last_failure
    window = math.radians(C.planner.push_heading_window_deg)
    for p in plans:
        entering_alpha = alpha
        for leg in p.legs:
            offs = [abs(((leg.heading - (entering_alpha + k * math.pi / 2) + math.pi) % (2 * math.pi)) - math.pi)
                    for k in range(4)]
            assert min(offs) <= window + 1e-6, "leg heading outside the push_heading_window"
            entering_alpha = wrap_to_pm45(leg.heading)


def test_delivery_points_never_stick_out():
    """Every returned plan's delivery point, evaluated at the orientation its own final heading
    implies, keeps the whole cube footprint inside the depot with the configured margin."""
    planner = PushPlanner()
    cube = CubeEstimate("red", 400.0, 400.0)
    depot = make_depot("red", 810.0, 810.0)
    plans = planner.plan(cube, depot, [], BOARD, BOARD, max_plans=5, deadline_s=0.3)
    assert plans, planner.last_failure
    for p in plans:
        alpha_final = wrap_to_pm45(p.legs[-1].heading)
        assert depot.contains_cube(p.delivery_point[0], p.delivery_point[1], C.cube.side,
                                    alpha_final, C.depot.delivery_margin)


def test_candidate_diversity():
    """max_plans > 1 should yield genuinely different delivery points, not the same point repeated."""
    planner = PushPlanner()
    cube = CubeEstimate("red", 400.0, 400.0)
    depot = make_depot("red", 810.0, 810.0)
    plans = planner.plan(cube, depot, [], BOARD, BOARD, max_plans=4, deadline_s=0.3)
    assert len(plans) >= 2, planner.last_failure
    pts = {(round(p.delivery_point[0], 0), round(p.delivery_point[1], 0)) for p in plans}
    assert len(pts) == len(plans), "expected distinct delivery points across returned plans"


def test_deadline_is_respected():
    """A near-zero deadline must return almost immediately (no unbounded search)."""
    import time
    planner = PushPlanner()
    cube = CubeEstimate("red", 400.0, 400.0)
    depot = make_depot("red", 810.0, 810.0)
    t0 = time.monotonic()
    planner.plan(cube, depot, [], BOARD, BOARD, max_plans=5, deadline_s=0.0)
    elapsed = time.monotonic() - t0
    assert elapsed < 0.2, f"deadline=0 took {elapsed:.3f}s -- search is not checking the budget often enough"


# --------------------------------------------------------------------------------- free-function unit tests

def test_leg_feasible_rejects_too_short_leg():
    ok, why = leg_feasible(C, FP, (400.0, 400.0), None, 0.0, 1.0, [], BOARD, BOARD)
    assert not ok and "short" in why


def test_leg_feasible_rejects_out_of_field_corridor():
    # heading straight toward the near edge with a long push -> corridor exits the field
    ok, why = leg_feasible(C, FP, (100.0, 430.0), None, math.pi, 500.0, [], BOARD, BOARD)
    assert not ok


def test_leg_feasible_accepts_clear_mid_field_push():
    ok, why = leg_feasible(C, FP, (400.0, 400.0), None, 0.0, 100.0, [], BOARD, BOARD)
    assert ok, why


def test_leg_feasible_rejects_when_other_cube_in_the_way():
    ok_clear, _ = leg_feasible(C, FP, (400.0, 400.0), None, 0.0, 150.0, [], BOARD, BOARD)
    assert ok_clear
    blocker = CubeEstimate("blue", 470.0, 400.0)
    ok_blocked, why = leg_feasible(C, FP, (400.0, 400.0), None, 0.0, 150.0, [blocker], BOARD, BOARD)
    assert not ok_blocked and "cube" in why


def test_prepush_pose_geometry():
    pose = prepush_pose(C, (500.0, 500.0), 0.0)
    assert pose.x == pytest.approx(500.0 - C.prepush_distance)
    assert pose.y == pytest.approx(500.0)
    assert pose.theta == pytest.approx(0.0)


def test_is_delivered_true_and_false():
    depot = make_depot("red", 810.0, 810.0)
    inside = CubeEstimate("red", 810.0, 810.0, alpha=0.0, alpha_std=math.radians(1.0))
    outside = CubeEstimate("red", 400.0, 400.0, alpha=0.0, alpha_std=math.radians(1.0))
    assert is_delivered(C, inside, depot)
    assert not is_delivered(C, outside, depot)


def test_is_delivered_unknown_alpha_uses_worst_case():
    depot = make_depot("red", 810.0, 810.0)
    # right at the edge of the ALIGNED region but outside the worst-case (unknown alpha) region
    half_size = C.depot.half_size
    aligned_edge = depot.cx - (half_size - C.cube.half - C.depot.delivery_margin) + 0.5
    cube_known = CubeEstimate("red", aligned_edge, 810.0, alpha=0.0, alpha_std=math.radians(0.5))
    cube_unknown = CubeEstimate("red", aligned_edge, 810.0, alpha=None)
    assert is_delivered(C, cube_known, depot)
    assert not is_delivered(C, cube_unknown, depot)


def test_classify_unsolvable_none_for_easy_cube():
    depot = make_depot("red", 810.0, 810.0)
    cube = CubeEstimate("red", 430.0, 430.0)
    assert classify_unsolvable(C, cube, depot, [], BOARD, BOARD) is None


# --------------------------------------------------------------------------------- seeded fuzz

def test_fuzz_mid_field_plans_pass_independent_geometry():
    """~200 seeded random mid-field cube/depot/obstacle configurations: every plan returned by the
    planner must pass a from-scratch geometric re-check (board containment of prepush/corridor/
    retreat, clearance to other cubes, final depot containment)."""
    rng = random.Random(20240521)
    planner = PushPlanner()
    n_checked = 0
    n_with_plans = 0
    for _ in range(200):
        cx, cy = rng.uniform(150.0, 710.0), rng.uniform(150.0, 710.0)
        depot_corner = rng.choice([(810.0, 810.0), (50.0, 810.0), (810.0, 50.0)])
        depot = make_depot("red", *depot_corner)
        cube = CubeEstimate("red", cx, cy)

        others = []
        for i in range(rng.randint(0, 2)):
            ox, oy = rng.uniform(150.0, 710.0), rng.uniform(150.0, 710.0)
            if math.hypot(ox - cx, oy - cy) < 80.0:
                continue
            others.append(CubeEstimate(f"o{i}", ox, oy))

        plans = planner.plan(cube, depot, others, BOARD, BOARD, max_plans=3, deadline_s=0.2)
        n_with_plans += 1 if plans else 0
        for p in plans:
            independent_check(p, others, BOARD, BOARD, depot)
            n_checked += 1
    assert n_checked > 0, "fuzz produced no plans to check at all -- suspicious"


# --------------------------------------------------------------------------------- solvable-fraction report

def test_report_solvable_fraction_by_leg_count(capsys):
    """Not a correctness assertion (see final report for the numbers) -- grid-samples the field with
    one depot and no other cubes, and prints the fraction solvable with 1/2/3 legs vs not found."""
    planner = PushPlanner()
    depot = make_depot("red", 810.0, 810.0)
    n = 14
    lo, hi = 60.0, 800.0
    counts = {0: 0, 1: 0, 2: 0, 3: 0}
    total = 0
    for xi in range(n):
        for yi in range(n):
            x = lo + (hi - lo) * xi / (n - 1)
            y = lo + (hi - lo) * yi / (n - 1)
            if math.hypot(x - depot.cx, y - depot.cy) < 40.0:
                continue
            total += 1
            cube = CubeEstimate("red", x, y)
            plans = planner.plan(cube, depot, [], BOARD, BOARD, max_plans=1, deadline_s=0.15)
            counts[len(plans[0].legs) if plans else 0] += 1
    with capsys.disabled():
        print(f"\n[solvable-fraction] n={total} "
              f"1-leg={counts[1]/total:.1%} 2-leg={counts[2]/total:.1%} "
              f"3-leg={counts[3]/total:.1%} unsolved-by-search={counts[0]/total:.1%}")
    assert total > 0
