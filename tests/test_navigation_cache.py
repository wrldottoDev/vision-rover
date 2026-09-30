import math

import numpy as np

import rover_strategy.planning.navigation as nav
from rover_strategy.config import DEFAULT as C
from rover_strategy.geometry import shapes as S
from rover_strategy.world import Pose


def _queries():
    rng = np.random.default_rng(0xCACED)
    for _ in range(30):
        sx, sy, gx, gy = rng.uniform(150.0, 710.0, size=4)
        obstacles = []
        for _ in range(int(rng.integers(0, 4))):
            ox, oy = rng.uniform(100.0, 760.0, size=2)
            obstacles.append(nav.DiscObstacle(float(ox), float(oy), C.cube.half_diag + C.margins.cube_nav))
        for _ in range(int(rng.integers(0, 5))):
            ox, oy = rng.uniform(120.0, 740.0, size=2)
            width, height = rng.uniform(20.0, 75.0, size=2)
            angle = float(rng.uniform(-math.pi, math.pi))
            local = S.rect(float(-width / 2), float(width / 2), float(-height / 2), float(height / 2))
            c, s = math.cos(angle), math.sin(angle)
            obstacles.append(nav.PolyObstacle(local @ np.array([[c, s], [-s, c]]) + [ox, oy]))
        yield (Pose(float(sx), float(sy), float(rng.uniform(-math.pi, math.pi))),
               Pose(float(gx), float(gy), float(rng.uniform(-math.pi, math.pi))), obstacles)


def _snapshot(path):
    if path is None:
        return None
    return path.cost_s, tuple(path.segments)


def test_cached_and_uncached_queries_are_bit_identical(monkeypatch):
    queries = list(_queries())
    cached = [_snapshot(nav.NavPlanner().plan(start, goal, obstacles, C.board.width, C.board.height,
                                               deadline_s=2.0))
              for start, goal, obstacles in queries]

    fast_cache = nav._FastCache

    class UncachedFastCache(fast_cache):
        def __init__(self, fp):
            super().__init__(fp, enabled=False)

    monkeypatch.setattr(nav, "_FastCache", UncachedFastCache)
    uncached = [_snapshot(nav.NavPlanner().plan(start, goal, obstacles, C.board.width, C.board.height,
                                                 deadline_s=2.0))
                for start, goal, obstacles in queries]
    assert uncached == cached
