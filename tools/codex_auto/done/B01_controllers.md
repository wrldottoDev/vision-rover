Resolve the remaining controller failures. Files: rover_strategy/control/controllers.py, rover_strategy/control/pid.py,
tests/test_controllers.py (re-baseline only with justification). Read docs/astra/gate6_report.md and the failing tests
in tests/test_controllers_astra.py and tests/test_controllers.py (run them). Known root causes: RetreatController
straightness/stop accuracy under 150 ms latency + 15 % asymmetry (consider a proper PD with derivative filtering and a
smoothed position for the stop trigger); PushController cross-track saturation under 15 % asymmetry (consider feed-
forward estimation of the wheel asymmetry from the observed omega/v mismatch — an adaptive bias term — instead of
loosening the curvature cap); lost-cube boundary case. Tests exercising rover_strategy/rover/fsm.py behaviour are the
lead's: leave them and list them. Acceptance: all controller tests pass except listed FSM-owned ones; no weakened
assertions without written justification.
