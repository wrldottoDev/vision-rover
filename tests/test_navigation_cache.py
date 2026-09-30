"""C06: the collision-check caching/AABB optimisation must not change what the planner finds.

Lead review of C06 (2026-09-29): the original version of this test only compared the NEW code's
cache-enabled vs cache-disabled runs against EACH OTHER. A completeness regression shared by both
paths (e.g. an over-aggressive AABB reject) would pass that check silently -- the reviewer's run
found 14/30 queries returning no path. This version instead replays the same 30 queries through a
frozen copy of `navigation.py` from immediately before C06
(`tests/fixtures/navigation_pre_c06_reference.py`, `git show b5d19d4:rover_strategy/planning/navigation.py`)
and requires all three (pre-C06 reference, current cached, current uncached) to agree bit-for-bit.
"""
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

import rover_strategy.planning.navigation as nav
from rover_strategy.config import DEFAULT as C
from rover_strategy.geometry import shapes as S
from rover_strategy.world import Pose


def _load_reference_navigation():
    """Load the pre-C06 navigation.py as `rover_strategy.planning._reference_pre_c06`, so its
    relative imports (`from ..config import ...`) resolve against the REAL, current dependency
    modules -- only the collision-check algorithm itself is the frozen "before" snapshot."""
    ref_path = Path(__file__).parent / "fixtures" / "navigation_pre_c06_reference.py"
    name = "rover_strategy.planning._reference_pre_c06"
    spec = importlib.util.spec_from_file_location(name, ref_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _query_specs():
    """Raw, module-agnostic query parameters -- obstacle instances are built per-module in
    `_build_obstacles` because `isinstance(o, DiscObstacle)` in the (reference or current)
    `_fast_obstacles()` only recognises that SAME module's own class, not a structurally-identical
    class from a different module object."""
    rng = np.random.default_rng(0xCACED)
    specs = []
    for _ in range(30):
        sx, sy, gx, gy = rng.uniform(150.0, 710.0, size=4)
        obstacles = []
        for _ in range(int(rng.integers(0, 4))):
            ox, oy = rng.uniform(100.0, 760.0, size=2)
            obstacles.append(("disc", float(ox), float(oy), C.cube.half_diag + C.margins.cube_nav))
        for _ in range(int(rng.integers(0, 5))):
            ox, oy = rng.uniform(120.0, 740.0, size=2)
            width, height = rng.uniform(20.0, 75.0, size=2)
            angle = float(rng.uniform(-math.pi, math.pi))
            local = S.rect(float(-width / 2), float(width / 2), float(-height / 2), float(height / 2))
            c, s = math.cos(angle), math.sin(angle)
            poly = local @ np.array([[c, s], [-s, c]]) + [ox, oy]
            obstacles.append(("poly", poly))
        specs.append((float(sx), float(sy), float(rng.uniform(-math.pi, math.pi)),
                      float(gx), float(gy), float(rng.uniform(-math.pi, math.pi)), obstacles))
    return specs


def _build_obstacles(module, obstacle_specs):
    built = []
    for spec in obstacle_specs:
        if spec[0] == "disc":
            _, ox, oy, r = spec
            built.append(module.DiscObstacle(ox, oy, r))
        else:
            built.append(module.PolyObstacle(spec[1]))
    return built


def _queries(module, specs):
    for sx, sy, sth, gx, gy, gth, obstacle_specs in specs:
        yield Pose(sx, sy, sth), Pose(gx, gy, gth), _build_obstacles(module, obstacle_specs)


def _snapshot(path):
    if path is None:
        return None
    return path.cost_s, tuple(path.segments)


def _plan_all(module, specs, *, cache_enabled: bool):
    queries = _queries(module, specs)
    if cache_enabled:
        return [_snapshot(module.NavPlanner().plan(start, goal, obstacles, C.board.width, C.board.height,
                                                    deadline_s=2.0))
                for start, goal, obstacles in queries]
    fast_cache = module._FastCache

    class UncachedFastCache(fast_cache):
        def __init__(self, fp):
            super().__init__(fp, enabled=False)

    original = module._FastCache
    module._FastCache = UncachedFastCache
    try:
        return [_snapshot(module.NavPlanner().plan(start, goal, obstacles, C.board.width, C.board.height,
                                                    deadline_s=2.0))
                for start, goal, obstacles in queries]
    finally:
        module._FastCache = original


def test_cached_and_uncached_queries_match_the_pre_c06_reference():
    specs = _query_specs()
    reference = _load_reference_navigation()

    baseline = _plan_all(reference, specs, cache_enabled=True)
    found = sum(1 for snap in baseline if snap is not None)
    # Many of these random cluttered layouts are genuinely infeasible or exceed the 2 s deadline even
    # on the reference implementation (verified: 16/30) -- that is a property of the query generator,
    # not a regression. This only guards against a vacuous test (every query trivially None).
    assert found >= 1, "no seeded query is solvable even on the pre-C06 reference; queries are too hard"

    cached = _plan_all(nav, specs, cache_enabled=True)
    uncached = _plan_all(nav, specs, cache_enabled=False)

    assert cached == baseline, "C06 caching changed a result versus the pre-C06 reference (cache enabled)"
    assert uncached == baseline, "C06 changed a result versus the pre-C06 reference (cache disabled)"
