Make rover_strategy/planning/navigation.py >= 3x faster on a REPRODUCIBLE benchmark that you add as
tools/bench_navigation.py (fixed seeds: 40 random queries on an 860 mm field with 0-3 cube discs and 0-10 convex
polygon obstacles; print mean/p95/max time and success count). Report before/after with the same script. Keep all
safety semantics (continuous rotation/arc sweeps with padding, superset reservations, final validation). All
navigation tests must still pass.
