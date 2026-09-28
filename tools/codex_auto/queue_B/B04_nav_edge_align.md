Navigation/alignment near field edges. Failing test: tests/test_navigation_astra.py::test_fsm_nav_goal_tolerance_does_not_require_alignment_off_board
and Gate 5 finding 3 (docs/astra/gate5_report.md). A nav goal reached within tolerance near an edge can require an ALIGN
correction that would leave the field. Fix in rover_strategy/planning/navigation.py and/or
rover_strategy/control/controllers.py (AlignController): e.g. make the planner's terminal tolerance direction-aware
(tight lateral tolerance w.r.t. the goal heading line) and make AlignController check every nudge's swept footprint
against the field (+ margin) and return "reposition" instead of executing an off-field nudge (it may need board
dimensions: add optional args with safe defaults from config). Do not edit fsm.py. Acceptance: that test and all
navigation/controller tests pass or are no worse.
