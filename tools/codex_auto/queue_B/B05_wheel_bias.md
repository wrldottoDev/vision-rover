Redo online wheel-asymmetry compensation for PushController and RetreatController in
rover_strategy/control/controllers.py (do NOT touch SegmentFollower: another worker owns it). A previous attempt
(B01, REJECTED — see tools/codex_auto/done/B01_controllers.md and docs/reviews/codex_reviews.md) removed the lost-cube
debounce and added a test-shaped special case. Requirements: estimate q = (g_r - g_l)/(g_r + g_l) online from
(commanded twist, EKF omega/v) with bounded, low-pass estimation that only runs with valid excitation; use it as
feed-forward in the curvature mapping; keep the physical curvature cap active until the estimate is valid; KEEP the
2-reading lost-cube debounce; no special cases keyed on dt/speed values from tests. Evaluate on
tests/test_controllers_astra.py and tests/test_controllers.py; report which failures remain and why.
Real data: rover 11's right wheel is ~7-9 % faster (docs/MEASUREMENTS.md).
