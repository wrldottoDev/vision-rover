Navigation performance, collision-checking ONLY (two previous attempts rejected for changing the search). File:
rover_strategy/planning/navigation.py. You may change ONLY the implementation of collision checks / sweep geometry
(_fast_* helpers, caching, bounding tests). The search (successor generation incl. every ARC successor, strides, ordering,
connectors, deadlines) must remain IDENTICAL: add a test that for 30 seeded random queries the returned paths are
bit-identical before and after (store expected results from the current main in the test via a fixture generated at test
time from a reference implementation copy, or compare segment lists produced with caching disabled vs enabled).
Benchmark: add tools/bench_navigation.py (40 fixed-seed queries) and report before/after; target >= 2x.
