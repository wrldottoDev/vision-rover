Fix the reviewer findings on YOUR previous commit (task B07, push planner coverage). Read
docs/codex_log/B07_push_planner_coverage_review.md in this worktree. Required:
(1) blocker diagnosis must run even when the obstacle-free classify_unsolvable() gives a reason: that classifier only
proves first-leg impossibility; a cube whose failure disappears when other cubes are removed must report
last_blockers and NOT be reported as geometrically unsolvable;
(2) every diagnostic probe must be charged against the caller's deadline (one absolute deadline for the whole call);
(3) 3-leg search must still run when fewer than max_plans plans were found or when all found plans are in the
corner-marker overlap tier; remove the test assertion that codified the completeness loss and replace it with a test
that a marker-free 3-leg plan is found when only marker-overlapping 2-leg plans exist (construct such a case).
Keep the symmetry fix. Report the clutter table again (seeds 100-129, overhang 0/20).
