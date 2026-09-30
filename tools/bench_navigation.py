"""Reproducible navigation collision-check benchmark.

Run with ``.venv/bin/python tools/bench_navigation.py`` from the repository root.
The query set is deliberately fixed: changes in the reported timings therefore come
from navigation implementation changes, not from a new random workload.
"""
from __future__ import annotations

import math
import pathlib
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from rover_strategy.config import DEFAULT as C
from rover_strategy.geometry import shapes as S
from rover_strategy.planning.navigation import DiscObstacle, NavPlanner, PolyObstacle
from rover_strategy.world import Pose


SEED = 0xC06
QUERY_COUNT = 40
BOARD_W = C.board.width
BOARD_H = C.board.height


def _queries() -> list[tuple[Pose, Pose, list[DiscObstacle | PolyObstacle]]]:
    rng = np.random.default_rng(SEED)
    queries = []
    # Keep endpoints in the same comfortably navigable interior band for every run;
    # obstacles still include dense cases that exercise the collision hot path.
    for _ in range(QUERY_COUNT):
        sx, sy, gx, gy = rng.uniform(150.0, 710.0, size=4)
        start = Pose(float(sx), float(sy), float(rng.uniform(-math.pi, math.pi)))
        goal = Pose(float(gx), float(gy), float(rng.uniform(-math.pi, math.pi)))
        obstacles: list[DiscObstacle | PolyObstacle] = []
        for _ in range(int(rng.integers(0, 4))):
            ox, oy = rng.uniform(110.0, 750.0, size=2)
            obstacles.append(DiscObstacle(float(ox), float(oy), C.cube.half_diag + C.margins.cube_nav))
        for _ in range(int(rng.integers(0, 11))):
            ox, oy = rng.uniform(120.0, 740.0, size=2)
            width, height = rng.uniform(15.0, 90.0, size=2)
            angle = float(rng.uniform(-math.pi, math.pi))
            local = S.rect(float(-width / 2), float(width / 2),
                           float(-height / 2), float(height / 2))
            c, s = math.cos(angle), math.sin(angle)
            rotated = local @ np.array([[c, s], [-s, c]]) + np.array([ox, oy])
            obstacles.append(PolyObstacle(rotated,))
        queries.append((start, goal, obstacles))
    return queries


def main() -> None:
    timings: list[float] = []
    successes = 0
    for start, goal, obstacles in _queries():
        planner = NavPlanner()
        t0 = time.perf_counter()
        path = planner.plan(start, goal, obstacles, BOARD_W, BOARD_H, deadline_s=1.0)
        timings.append(time.perf_counter() - t0)
        successes += path is not None
    ordered = sorted(timings)
    p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
    print(f"seed={SEED} queries={QUERY_COUNT} successes={successes}")
    print(f"mean={statistics.fmean(timings):.6f}s p95={p95:.6f}s max={max(timings):.6f}s")


if __name__ == "__main__":
    main()
