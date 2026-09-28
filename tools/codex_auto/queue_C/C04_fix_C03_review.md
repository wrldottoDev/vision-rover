Fix the reviewer findings on YOUR previous commit (task C03, navigation performance). Read
docs/codex_log/C03_nav_perf_review.md in this worktree. Required: (1) revert the far-field stride increase (160 mm)
or make it adaptive with a guaranteed fallback to the fine stride when the coarse search fails, so no route that the
parent found is lost — add the reviewer's dense-rectangle case as a regression test; (2) do not omit ARC successors
merely because an in-place rotation is free (at minimum when obstacles are within sweep_radius + 150 mm); add a
test where only a curved translation avoids an obstacle; (3) keep the caches/rejection tests; rerun
tools/bench_navigation.py and report before/after (keep >= 2x mean speedup if possible, correctness first).
