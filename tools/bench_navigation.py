"""Reproducible navigation collision-check benchmark: reports BEFORE (pre-C06 reference
implementation) vs AFTER (current) timings and the resulting speedup.

Run with ``.venv/bin/python tools/bench_navigation.py`` from the repository root.
The query set is deliberately fixed: changes in the reported timings therefore come
from navigation implementation changes, not from a new random workload.

Lead review of C06 (2026-09-29): the original version of this tool only benchmarked whichever
navigation.py happened to be importable (the "after" state) and never actually measured or printed
a before/after comparison, despite the task requiring one. This version loads the frozen pre-C06
snapshot (`tests/fixtures/navigation_pre_c06_reference.py`,
`git show b5d19d4:rover_strategy/planning/navigation.py`) the same way
`tests/test_navigation_cache.py` does, and benchmarks both against the identical query set.
"""
from __future__ import annotations

import importlib.util
import math
import pathlib
import statistics
import sys
import time

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from rover_strategy.config import DEFAULT as C
from rover_strategy.geometry import shapes as S
from rover_strategy.world import Pose
import rover_strategy.planning.navigation as current_nav


SEED = 0xC06
QUERY_COUNT = 40
BOARD_W = C.board.width
BOARD_H = C.board.height


def _load_reference_navigation():
    ref_path = REPO_ROOT / "tests" / "fixtures" / "navigation_pre_c06_reference.py"
    name = "rover_strategy.planning._reference_pre_c06"
    spec = importlib.util.spec_from_file_location(name, ref_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _query_specs() -> list[tuple[Pose, Pose, list[tuple]]]:
    """Module-agnostic query parameters; obstacle instances are built per-module in `_build_obstacles`
    (a class from one module's `isinstance` checks does not recognise another module's equivalent
    class, even when both were loaded from the same source)."""
    rng = np.random.default_rng(SEED)
    specs = []
    # Keep endpoints in the same comfortably navigable interior band for every run;
    # obstacles still include dense cases that exercise the collision hot path.
    for _ in range(QUERY_COUNT):
        sx, sy, gx, gy = rng.uniform(150.0, 710.0, size=4)
        obstacles = []
        for _ in range(int(rng.integers(0, 4))):
            ox, oy = rng.uniform(110.0, 750.0, size=2)
            obstacles.append(("disc", float(ox), float(oy), C.cube.half_diag + C.margins.cube_nav))
        for _ in range(int(rng.integers(0, 11))):
            ox, oy = rng.uniform(120.0, 740.0, size=2)
            width, height = rng.uniform(15.0, 90.0, size=2)
            angle = float(rng.uniform(-math.pi, math.pi))
            local = S.rect(float(-width / 2), float(width / 2),
                           float(-height / 2), float(height / 2))
            c, s = math.cos(angle), math.sin(angle)
            rotated = local @ np.array([[c, s], [-s, c]]) + np.array([ox, oy])
            obstacles.append(("poly", rotated))
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


def _run(module, specs) -> tuple[list[float], int]:
    timings: list[float] = []
    successes = 0
    for sx, sy, sth, gx, gy, gth, obstacle_specs in specs:
        start, goal = Pose(sx, sy, sth), Pose(gx, gy, gth)
        obstacles = _build_obstacles(module, obstacle_specs)
        planner = module.NavPlanner()
        t0 = time.perf_counter()
        path = planner.plan(start, goal, obstacles, BOARD_W, BOARD_H, deadline_s=1.0)
        timings.append(time.perf_counter() - t0)
        successes += path is not None
    return timings, successes


def _summary(label: str, timings: list[float], successes: int) -> float:
    ordered = sorted(timings)
    p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
    total = sum(timings)
    print(f"[{label}] seed={SEED:#x} queries={QUERY_COUNT} successes={successes} total={total:.4f}s")
    print(f"[{label}] mean={statistics.fmean(timings):.6f}s p95={p95:.6f}s max={max(timings):.6f}s")
    return total


def main() -> None:
    specs = _query_specs()
    reference = _load_reference_navigation()

    before_timings, before_ok = _run(reference, specs)
    after_timings, after_ok = _run(current_nav, specs)

    before_total = _summary("before (pre-C06 reference)", before_timings, before_ok)
    after_total = _summary("after  (current)", after_timings, after_ok)

    speedup = before_total / after_total if after_total > 0 else float("inf")
    print(f"speedup: {speedup:.2f}x total wall time (before/after); successes before={before_ok} after={after_ok}")
    if before_ok != after_ok:
        print("WARNING: success count changed between before and after -- see test_navigation_cache.py "
              "for the bit-identical-results check; this benchmark alone does not prove correctness.")


if __name__ == "__main__":
    main()
