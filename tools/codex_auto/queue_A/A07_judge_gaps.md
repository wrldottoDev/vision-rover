Close the simulator-judge gaps found by the reviewer of A06 (see tools/codex_auto/done/A06_fix_lane_a_review.md and
the review in git tag archive-a-* or the worktree history): (1) record a safety event when a rover ROTATES
(|omega| > 0.2 rad/s) while a cube is inside its channel (between the paddles, touching or within 5 mm of the plate) —
report it as a metric `rotations_with_cube` and include it in monte_carlo aggregation (not a hard failure);
(2) participation/transport credit only from CAUSAL push displacement: cube displacement integrated only over steps where
the rover's front plate is in contact AND the rover's actual forward velocity > 5 mm/s; (3) contact rejection must use the
actual rover velocity, not the commanded direction. Files: rover_strategy/simulation/{physics.py,runner.py,monte_carlo.py}
+ tests. Keep >= 30x real time.
