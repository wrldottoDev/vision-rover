# Known issues (keep current; newest first)

_Verified against `main` (post A07/B10/C06 merges), 2026-09-29 (Claude Sonnet 5): full suite **290 passed / 2
failed**. See `docs/reviews/codex_reviews.md` for the review of each merged lane and what got fixed vs left open._

## Blocking
- **Monte Carlo baseline is stale, not currently blocking-wrong, just unmeasured.** The 2026-09-28 seeds 100-111
  numbers (1/36 cubes) predate the A01-A07 simulator merges, the B07+B08 push-planner fix, the C02 arc primitives and
  the B09 board-guard fix — i.e. predate most of what actually matters. Do not quote those numbers anymore. A07
  (judge-gaps fix) is now merged, so a fresh re-baseline is unblocked — see HANDOFF next steps. Until it runs,
  "no closed-loop delivery yet" / "planner_no_solution under clutter" / "WAIT/yield churn under 45 mm starts" should
  be treated as *last confirmed* 2026-09-28, not necessarily current.

## Fixed (kept here for traceability; do not re-litigate)
- **Simulator judging defects (Astra Gate 8)** — `docs/astra/gate8_report.md` listed 24 failing tests in
  `tests/test_simulation_astra.py`. Re-run 2026-09-29: **38/38 passing.** Fixed by the A01-A06 lane-A merges
  (corner-marker freeze modelling, planner-reason-based classification instead of "physically_infeasible", both-rover
  participation, settled completion, contact/friction physics, occlusion union, etc.). Some Gate 8 findings (e.g.
  scenario coverage stratification, item 13) may still be partially open — re-check against the report if you touch
  `simulation/scenarios.py`.
- **Edge-parallel pre-push poses unreachable** (no arc primitives): fixed by C02 (`f6d9526`, arc motion primitives).
  `tests/test_navigation_astra.py`: 54/54 passing 2026-09-29 (was: 1 failure, FSM goal-tolerance-vs-align near edges).
- **Board exit (seed 107, 20 mm overhang)**: fixed at the root by `e653bf8` (supervisor board guard). See HANDOFF.
- **Navigation planning perf**: C06 (lead-reviewed, merged) delivers a measured **1.94x** total wall-time speedup on
  the collision-check hot path (AABB precomputation + per-heading caching), search provably kept bit-identical
  (`tests/test_navigation_cache.py` replays 30 seeded queries against a frozen pre-C06 reference implementation).
  Just under the 2x target asked for; not chased further given two prior attempts at this task were rejected for
  changing the search itself. Still the dominant cost in a simulated run — worth another pass later, not blocking.
- **A07 judge gaps** (rotations-with-cube metric, causal transport-credit, velocity-based contact rejection):
  merged, lead-fixed to read the actually-realized (post-noise) wheel speeds rather than the pre-noise lagged
  target (see `docs/reviews/codex_reviews.md`).
- **5 of 7 controller adversarial failures** (B10: push 12mm cube offset x2, lost-cube threshold x2,
  `test_push_lost_cube_flag`): fixed via better loop design (heading integral with anti-windup), 0 regressions.

## Important
- **Controllers: 2 adversarial failures remain** (`tests/test_controllers_astra.py::test_retreat_wheel_truth_stays_straight_and_stops`,
  both latency parametrizations): peak retreat yaw ~4.1 deg vs the 3 deg bound, under a full UNCOMPENSATED 15% wheel
  asymmetry with the bare `RetreatController` tested in isolation (no supervisor-level `ROVER_MOTORS` feed-forward in
  front of it). A fixed-direction feedforward was tried and rejected (wrong for balanced R10, duplicates the
  supervisor's existing per-rover compensation); reactive PID + a rover-agnostic integral cannot reliably close this
  gap (hand-tuning kp/kd/tau does not converge — see commit 93b97d5). Needs a deliberate architecture decision (thread
  real per-rover calibration into the controller, lead-owned files) or a re-scoped test, not another patch attempt.
  See "Open items" in `docs/reviews/codex_reviews.md`.
- **Push curvature cap is reactive-only once feedback is available** (one control tick of lag before an excess
  realized curvature is caught): known, documented inline in `controllers.py`; fixing it needs a coupled cap+gain
  retune (a naive fix regresses 5 other tests — verified), scoped as a future task, not fixed blind.
- DEGRADED policy now implemented with ASSUMED thresholds; needs real latency logs to tune.
- Protocol v2 (USER-REPORTED) not supported; no schema in repo.
- Marker offset ~24 mm (USER-REPORTED) not yet applied; must confirm whether the deployed vision compensates it.
- Motor model in the simulator is random asymmetry, not the measured R10/R11 characterisation.
- Supervisor plans synchronously inside `tick()`; fine in simulation, must be threaded for real time.

## Accepted limitations (documented, not bugs)
- Edge lock: cubes within ~230 mm of an edge can only move along it (no walls, rover must be behind the cube).
- Single-cube solvable field fraction ~65 % under the strict field rule.
