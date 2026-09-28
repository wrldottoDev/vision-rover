Fix the reviewer findings recorded for the simulator work already merged into main (tasks A01-A04). Read
docs/reviews/codex_reviews.md and the archived reviews in tools/codex_auto/done/ (task texts) — the reviews themselves
are in git tag archive-a-* under docs/codex_log/A0*_review.md (`git show <tag>:docs/codex_log/A01_sim_judge_review.md`).
Files: rover_strategy/simulation/{runner.py,monte_carlo.py,sensors.py,physics.py,scenarios.py} + tests.
Must fix: (1) participation credit only for a rover that pushed the cube (>= 30 mm of engaged displacement while it
was that rover's task) AND whose push ended with the cube delivered; merely touching a cube never counts;
(2) use planner_reason for the planner_rejected class; (3) a cube that left the field is out of play permanently;
(4) sensors: no telemetry published before a first valid 4-marker homography; default capture clock 30 Hz,
publication 20 Hz as in the official engine; (5) test_simulation.py regressions (if any remain); (6) motor model
per-step cost: precompute the piecewise curves so the simulator is >= 30x real time again (measure). Do not edit
rover_strategy/coordination or rover/fsm.py.
