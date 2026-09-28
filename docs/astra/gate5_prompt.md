You are ASTRA, independent adversarial reviewer. GATE 5: navigation planner. Repo root = current dir.
Read rover_strategy/planning/navigation.py, tests/test_navigation.py, rover_strategy/geometry/{shapes.py,footprint.py},
rover_strategy/config.py, rover_strategy/world.py, docs/geometry_model.md, and how it is used in
rover_strategy/coordination/supervisor.py (plan_nav, path_still_clear, swept_polygons for reservations) and rover_strategy/rover/fsm.py.
Rover: differential drive, rotates in place, asymmetric envelope x in [-47,102] mm, |y|<=49.75, sweep radius 113.5 mm;
field 860x860 with NO walls (whole envelope must stay inside); cubes are disc obstacles; the other rover's reservation is
polygon obstacles.
Assume it is WRONG. Find: collision checks that can miss contact (rotation sweep sampling, straight sweep between samples,
fast-path vs public-path divergence, inflation semantics, board margin), swept_polygons that do NOT cover the actual motion
(so reservations under-approximate -> two rovers can collide), heuristic inadmissibility causing bad paths, escape logic that
drives INTO obstacles, goal tolerance mismatches with the FSM (goal_pos_tol 18 mm vs capture lateral tolerance ~4-16 mm!),
determinism, deadline behaviour, failure modes near edges/corners, paths that reverse blindly. WRITE adversarial tests in
tests/test_navigation_astra.py (dense independent swept-area verification incl. rotations; swept_polygons ⊇ true swept area;
pathological layouts) and run them with `.venv/bin/python -m pytest -q tests/test_navigation_astra.py`.
Report CRITICAL/HIGH/MEDIUM/LOW with file:line, failure scenario, concrete fix. Modify only the new test file.
