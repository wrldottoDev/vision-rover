import math

import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.world import Pose, PushLeg, PushPlan, RoverEstimate
from rover_strategy.planning.task_allocator import (
    AllocatorParams, TaskAllocator, distribute_cubes, should_reallocate,
)


def _plan(color, cube_start, cube_end, heading, prepush, cost_s, risk=0.0):
    leg = PushLeg(cube_start=cube_start, cube_end=cube_end, heading=heading, prepush=prepush)
    return PushPlan(color=color, legs=[leg], delivery_point=cube_end, cost_s=cost_s, risk=risk)


def _rover(rid, x, y, theta=0.0):
    return RoverEstimate(id=rid, pose=Pose(x, y, theta))


# --------------------------------------------------------------------------- distribute_cubes
def test_distribute_cubes_count_matches_formula():
    assert len(list(distribute_cubes(["a", "b", "c"], [1, 2]))) == 24     # 3! * 4
    assert len(list(distribute_cubes(["a", "b"], [1, 2]))) == 6           # 2! * 3
    assert len(list(distribute_cubes(["a"], [1, 2]))) == 2                # 1! * 2
    assert len(list(distribute_cubes(["a", "b", "c"], [1]))) == 6         # 3! single rover


def test_distribute_cubes_every_cube_appears_exactly_once():
    for dist in distribute_cubes(["a", "b", "c"], [1, 2]):
        seen = dist.get(1, []) + dist.get(2, [])
        assert sorted(seen) == ["a", "b", "c"]


# --------------------------------------------------------------------------- makespan beats greedy
def _greedy_nearest(rovers, plans, nav_time):
    """Sequential 'assign to whichever rover is currently cheaper' baseline: processes cubes in a
    fixed input order and never reconsiders -- the classic list-scheduling trap."""
    poses = {r: est.pose for r, est in rovers.items()}
    loads = {r: 0.0 for r in rovers}
    seq = {r: [] for r in rovers}
    for color, cands in plans.items():
        plan = cands[0]
        best_r = min(rovers, key=lambda r: loads[r] + nav_time(poses[r], plan.legs[0].prepush))
        seq[best_r].append((color, plan))
        loads[best_r] += nav_time(poses[best_r], plan.legs[0].prepush) + plan.cost_s + 1.5
        poses[best_r] = Pose(*plan.delivery_point, plan.legs[-1].heading)
    return max(loads.values())


def test_allocator_beats_greedy_on_crafted_scheduling_case():
    # Classic 2-machine adversarial case: costs [1, 1, 2] processed in that order. Greedy
    # (assign each job, in order, to whichever machine is currently cheaper) piles jobs 1 and 3
    # onto machine 1 (tie broken low id) for a load of 3, leaving machine 2 at 1: makespan 3.
    # The true optimum groups jobs 1+2 (load 2) against job 3 alone (load 2): makespan 2.
    zero_nav = lambda a, b: 0.0
    rovers = {1: _rover(1, 0, 0), 2: _rover(2, 0, 0)}
    plans = {
        "red": [_plan("red", (0, 0), (10, 0), 0.0, Pose(0, 0, 0), cost_s=1.0)],
        "green": [_plan("green", (1000, 0), (1010, 0), 0.0, Pose(1000, 0, 0), cost_s=1.0)],
        "blue": [_plan("blue", (2000, 0), (2020, 0), 0.0, Pose(2000, 0, 0), cost_s=2.0)],
    }

    greedy_makespan = _greedy_nearest(rovers, plans, zero_nav)

    allocator = TaskAllocator(cfg=C)
    alloc = allocator.allocate(rovers, plans, locked={}, nav_time=zero_nav)

    assert alloc.makespan < greedy_makespan
    groups = {frozenset(t.color for t in tasks) for tasks in alloc.assignment.values()}
    assert frozenset({"red", "green"}) in groups
    assert frozenset({"blue"}) in groups


# --------------------------------------------------------------------------- locked tasks
def test_locked_cube_stays_with_its_rover_and_goes_first():
    rovers = {1: _rover(1, 0, 0), 2: _rover(2, 500, 0)}
    plans = {
        "red": [_plan("red", (0, 0), (50, 0), 0.0, Pose(0, 0, 0), cost_s=2.0)],
        "green": [_plan("green", (100, 100), (150, 100), 0.0, Pose(100, 100, 0), cost_s=2.0)],
        "blue": [_plan("blue", (500, 100), (550, 100), 0.0, Pose(500, 100, 0), cost_s=2.0)],
    }
    allocator = TaskAllocator(cfg=C)
    alloc = allocator.allocate(rovers, plans, locked={1: "red", 2: None})

    assert alloc.assignment[1][0].color == "red"
    assert all(t.color != "red" for t in alloc.assignment[2])


# --------------------------------------------------------------------------- skip cubes with no plans
def test_cube_with_no_plans_is_skipped_and_reported():
    rovers = {1: _rover(1, 0, 0), 2: _rover(2, 500, 0)}
    plans = {
        "red": [_plan("red", (0, 0), (50, 0), 0.0, Pose(0, 0, 0), cost_s=2.0)],
        "green": [],
    }
    allocator = TaskAllocator(cfg=C)
    alloc = allocator.allocate(rovers, plans, locked={})

    assert alloc.skipped == ("green",)
    assert all(t.color != "green" for tasks in alloc.assignment.values() for t in tasks)


# --------------------------------------------------------------------------- single rover degradation
def test_single_rover_available_gets_all_cubes():
    rovers = {1: _rover(1, 0, 0)}
    plans = {
        "red": [_plan("red", (0, 0), (50, 0), 0.0, Pose(0, 0, 0), cost_s=1.0)],
        "green": [_plan("green", (300, 300), (350, 300), 0.0, Pose(300, 300, 0), cost_s=1.0)],
    }
    allocator = TaskAllocator(cfg=C)
    alloc = allocator.allocate(rovers, plans, locked={2: "blue"})   # rover 2 is down (LOST), lock ignored

    assert set(alloc.assignment.keys()) == {1}
    assert {t.color for t in alloc.assignment[1]} == {"red", "green"}
    assert alloc.makespan > 0


# --------------------------------------------------------------------------- precedence
def test_precedence_forces_blocking_cube_first():
    # red's only push corridor runs straight through blue's current resting position -> blue must
    # be pushed out of the way before red can be pushed, even on a single rover with no choice of
    # plan otherwise (so the only free variable is *order*).
    rovers = {1: _rover(1, -100, 0)}
    plans = {
        "red": [_plan("red", (0, 0), (500, 0), 0.0, Pose(-166, 0, 0), cost_s=3.0)],
        "blue": [_plan("blue", (250, 0), (250, 300), math.pi / 2, Pose(250, -166, math.pi / 2), cost_s=3.0)],
    }
    allocator = TaskAllocator(cfg=C)
    alloc = allocator.allocate(rovers, plans, locked={})

    colors_in_order = [t.color for t in alloc.assignment[1]]
    assert colors_in_order.index("blue") < colors_in_order.index("red")


# --------------------------------------------------------------------------- determinism
def test_allocate_is_deterministic():
    rovers = {1: _rover(1, 0, 0), 2: _rover(2, 400, 0)}
    plans = {
        "red": [_plan("red", (0, 0), (50, 0), 0.0, Pose(0, 0, 0), cost_s=1.5)],
        "green": [_plan("green", (400, 0), (450, 0), 0.0, Pose(400, 0, 0), cost_s=1.5)],
        "blue": [_plan("blue", (200, 400), (250, 400), 0.0, Pose(200, 400, 0), cost_s=1.5)],
    }
    allocator = TaskAllocator(cfg=C)
    a1 = allocator.allocate(rovers, plans, locked={})
    a2 = allocator.allocate(rovers, plans, locked={})

    shape1 = {r: [t.color for t in tasks] for r, tasks in a1.assignment.items()}
    shape2 = {r: [t.color for t in tasks] for r, tasks in a2.assignment.items()}
    assert shape1 == shape2
    assert a1.makespan == a2.makespan


# --------------------------------------------------------------------------- hard "both rovers" rule
def test_require_both_rovers_splits_work_even_if_makespan_alone_prefers_one():
    # Both cubes sit right next to rover 1; rover 2 is far away. Pure makespan minimisation (no
    # other-rover-must-work constraint) would dump both on rover 1 and leave rover 2 idle. The
    # rules (reglamento.md s12 / el_reto.md) require BOTH rovers to deliver at least one cube each
    # when both are healthy and there's more than one cube to go around.
    rovers = {1: _rover(1, 0, 0), 2: _rover(2, 5000, 5000)}
    plans = {
        "red": [_plan("red", (0, 0), (50, 0), 0.0, Pose(0, 0, 0), cost_s=1.0)],
        "green": [_plan("green", (60, 0), (110, 0), 0.0, Pose(60, 0, 0), cost_s=1.0)],
    }

    strict = TaskAllocator(cfg=C, params=AllocatorParams(require_both_rovers=True))
    alloc = strict.allocate(rovers, plans, locked={})
    assert all(len(tasks) >= 1 for tasks in alloc.assignment.values())

    lax = TaskAllocator(cfg=C, params=AllocatorParams(require_both_rovers=False))
    alloc_lax = lax.allocate(rovers, plans, locked={})
    assert any(len(tasks) == 0 for tasks in alloc_lax.assignment.values())
    # relaxing the rule should never be worse for pure makespan than obeying it
    assert alloc_lax.makespan <= alloc.makespan


def test_require_both_rovers_does_not_apply_with_only_one_cube_or_one_rover():
    rovers = {1: _rover(1, 0, 0), 2: _rover(2, 5000, 5000)}
    plans = {"red": [_plan("red", (0, 0), (50, 0), 0.0, Pose(0, 0, 0), cost_s=1.0)]}
    allocator = TaskAllocator(cfg=C)
    alloc = allocator.allocate(rovers, plans, locked={})
    assert sum(len(t) for t in alloc.assignment.values()) == 1   # no crash, no forced empty-cube split


# --------------------------------------------------------------------------- should_reallocate
def test_should_reallocate_hysteresis():
    rovers = {1: _rover(1, 0, 0), 2: _rover(2, 400, 0)}
    plans = {
        "red": [_plan("red", (0, 0), (50, 0), 0.0, Pose(0, 0, 0), cost_s=1.5)],
        "green": [_plan("green", (400, 0), (450, 0), 0.0, Pose(400, 0, 0), cost_s=1.5)],
    }
    allocator = TaskAllocator(cfg=C)
    old = allocator.allocate(rovers, plans, locked={})

    # a "new" allocation identical in shape but objective nudged slightly better should NOT
    # trigger a switch under a loose hysteresis
    import dataclasses
    tiny_gain = dataclasses.replace(old, objective=dataclasses.replace(
        old.objective, total=old.objective.total - 0.01))
    assert should_reallocate(old, tiny_gain, hysteresis=1.0) is False

    big_gain = dataclasses.replace(old, objective=dataclasses.replace(
        old.objective, total=old.objective.total - 10.0),
        assignment={1: [], 2: list(old.assignment[1]) + list(old.assignment[2])})
    assert should_reallocate(old, big_gain, hysteresis=1.0) is True

    assert should_reallocate(old, old, hysteresis=0.0) is False
