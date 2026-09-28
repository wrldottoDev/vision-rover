Fix the simulator JUDGE defects found by the Gate 8 audit. Read docs/astra/gate8_report.md fully and
tests/test_simulation_astra.py. Files you may edit: rover_strategy/simulation/runner.py,
rover_strategy/simulation/monte_carlo.py (and tests/test_simulation_astra.py ONLY where a test is provably wrong).
Must fix: (1) never label a run "physically_infeasible" from the planner's own conservative checks — keep a separate
"planner_rejected" class and preserve the original failure reason; only independently justified conditions may be
"physically_infeasible"; (2) success requires both rovers to have participated (each transported and deposited >= 1
cube, attributed from ground truth contacts) when >= 2 cubes exist; (3) completion only after all rovers have stopped
(truth wheel speeds ~0, no pending commands) and truth delivery stayed valid for a settling interval (>= 1 s); report
delivery time and settled completion separately; (4) any other runner/metrics findings in the report (episode counting
hiding long violations, stall detection, time-to-complete definition). Acceptance: all judge-related tests in
tests/test_simulation_astra.py pass; tests/test_simulation.py still passes; full suite not worse.
