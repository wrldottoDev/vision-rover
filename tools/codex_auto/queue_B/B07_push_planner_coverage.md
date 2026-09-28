Push-planner search completeness and symmetry. File: rover_strategy/planning/push_planner.py (+ tests). Evidence:
`DL=2.0 PYTHONPATH=. .venv/bin/python tools/solvability_map.py 40` gives single-cube coverage top-right 49 %,
bottom-right 54 %, bottom-left 43 % with far fewer 2-leg solutions for bottom-left (51 vs ~160) although the problem
is symmetric under reflections of the field (depots at the three non-start corners). Find the root cause (candidate
generation, elbow/hop candidate construction, grid axes offsets, ordering + deadline, marker-tier filtering...),
fix it without weakening any safety check, and add a test that coverage is symmetric under x/y reflections within a
few percent on a coarse grid (use generous deadlines in the test). Also make the default search find 2-leg plans
before spending budget on 3-leg ones, and report typical plan() time. Do not change cfg/margins.

ADDITIONAL (same task): cluttered layouts. Seeds of family official_like (e.g. seed 102: blue (521,535), green
(495,426), red (268,495); seed 101: blue (244,195), red (486,263), green (660,673); depots from
`rover_strategy.simulation.scenarios.generate(seed, "official_like").depots` in official cells) produce NO plan for
some interior cubes because other cubes block every corridor. Implement in the push planner a way to report the
blocking cube(s) (e.g. `last_blockers: list[str]`) so the allocator can sequence "blocker first", and make sure a
cube whose ONLY obstacle is another undelivered cube is not reported as unsolvable. Measure: for seeds 100-129 of
official_like, count cubes with a plan when others are present vs removed (report table). Test with
overhang_allowance 0 and 20 mm (config.board.overhang_allowance_mm; use cfg.planning()).
