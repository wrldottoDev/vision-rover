Navigation performance WITHOUT completeness loss (two earlier attempts were rejected — see
tools/codex_auto/done/C03_nav_perf.md, C04_fix_C03_review.md and their reviews in git tag archive-c-* under
docs/codex_log/). File: rover_strategy/planning/navigation.py (+ tools/bench_navigation.py from tag archive-c-* if
useful, tests). Allowed techniques: caching of per-heading/per-arc swept hulls in the rover frame, SAT axis caching,
bounding-box/circle rejection, cheaper data structures. NOT allowed: coarser far-field strides, skipping ARC successors,
two-phase searches that only try arcs after a no-arc search fails. Acceptance: all tests/test_navigation*.py pass;
reviewer's dense-rectangle case and a curved-translation-only case as regression tests; >= 2x mean speedup on the
reproducible benchmark (report before/after).
