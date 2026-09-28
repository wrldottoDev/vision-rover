Push-planner search completeness and symmetry. File: rover_strategy/planning/push_planner.py (+ tests). Evidence:
`DL=2.0 PYTHONPATH=. .venv/bin/python tools/solvability_map.py 40` gives single-cube coverage top-right 49 %,
bottom-right 54 %, bottom-left 43 % with far fewer 2-leg solutions for bottom-left (51 vs ~160) although the problem
is symmetric under reflections of the field (depots at the three non-start corners). Find the root cause (candidate
generation, elbow/hop candidate construction, grid axes offsets, ordering + deadline, marker-tier filtering...),
fix it without weakening any safety check, and add a test that coverage is symmetric under x/y reflections within a
few percent on a coarse grid (use generous deadlines in the test). Also make the default search find 2-leg plans
before spending budget on 3-leg ones, and report typical plan() time. Do not change cfg/margins.
