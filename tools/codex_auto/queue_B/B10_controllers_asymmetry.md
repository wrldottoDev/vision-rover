Controllers under wheel asymmetry. File: rover_strategy/control/controllers.py (PushController, RetreatController only;
do not change SegmentFollower) + tests. Failing: tests/test_controllers_astra.py retreat straightness/stop (x2), push with
12 mm cube offset (x2), lost-cube threshold (x2), and tests/test_controllers.py::test_push_lost_cube_flag.
Constraints (from rejected attempts B01/B05, see tools/codex_auto/done and docs/reviews/codex_reviews.md): KEEP the
2-consecutive-reading lost-cube debounce — you may amend the lost-cube tests to feed two consecutive fresh readings (the
debounce is a lead design decision) but not their geometry; KEEP the physical curvature cap at all times; no special cases
keyed to test dt/speeds; no online wheel-bias estimation (rejected twice). Allowed: better loop design (e.g. integral action
on heading with anti-windup for steady asymmetry; derivative filtering; smoothed stop trigger). Report per test.
