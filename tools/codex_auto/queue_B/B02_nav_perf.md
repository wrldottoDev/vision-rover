Make rover_strategy/planning/navigation.py at least 3x faster without changing results. Profile first:
`PYTHONPATH=. .venv/bin/python -c "import cProfile,pstats; from rover_strategy.simulation.runner import run_seed; cProfile.run('run_seed(0, \"official_like\", max_time=20)','/tmp/p.out'); pstats.Stats('/tmp/p.out').sort_stats('cumulative').print_stats(25)"`
(hot spots so far: _fast_hull, _fast_segment_collides, _analytic_connect). Ideas: cache per-heading envelope and
rotation-interval hulls in the rover frame and translate them; bounding-circle early outs; avoid Python-level hull
recomputation for rotations (precompute the swept hull of each rotation interval once per heading bin in the rover frame).
Keep all safety semantics (continuous rotation sweeps, arc padding, superset reservations). Acceptance: tests/
test_navigation.py and tests/test_navigation_astra.py results unchanged or better; measured speedup reported
(before/after timings on the profile command and on 30 random 3-obstacle queries).
