# Known issues (keep current; newest first)

_Verified against `main` @ b5d19d4 / 37df87a, 2026-09-29 (Claude Sonnet 5): full suite 280 passed / 7 failed, all 7
below (B10 scope). Two items below marked FIXED were previously listed here as blocking but were confirmed resolved
by direct re-run — this file had not been updated after the A01-A06 merges landed._

## Blocking
- **Monte Carlo baseline is stale, not currently blocking-wrong, just unmeasured.** The 2026-09-28 seeds 100-111
  numbers (1/36 cubes) predate the A01-A06 simulator merges, the B07+B08 push-planner fix, the C02 arc primitives and
  the B09 board-guard fix — i.e. predate most of what actually matters. Do not quote those numbers anymore. A fresh
  re-baseline is queued behind A07 (see HANDOFF next steps); until then, "no closed-loop delivery yet" /
  "planner_no_solution under clutter" / "WAIT/yield churn under 45 mm starts" should be treated as *last confirmed*
  2026-09-28, not necessarily current.
- **Navigation planning is slow** (~0.3-1.5 s per query, dominates simulation time) — Python hull/SAT in hot loop.
  Lane C task C06 (queued/running) targets >= 2x via collision-check-only changes, search kept bit-identical.

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

## Important
- DEGRADED policy now implemented with ASSUMED thresholds; needs real latency logs to tune.
- Protocol v2 (USER-REPORTED) not supported; no schema in repo.
- Marker offset ~24 mm (USER-REPORTED) not yet applied; must confirm whether the deployed vision compensates it.
- Motor model in the simulator is random asymmetry, not the measured R10/R11 characterisation.
- **Controllers: 7 adversarial failures**, confirmed still failing 2026-09-29 (`tests/test_controllers_astra.py`:
  retreat straightness under 15% asymmetry + latency x2, push cross-track under 15% asymmetry with 12mm cube offset
  x2, lost-cube orientation-dependent channel threshold x2; `tests/test_controllers.py::test_push_lost_cube_flag`
  x1) — root causes in `docs/astra/gate6_report.md`. This is the exact scope of task B10 (queued/running).
- Supervisor plans synchronously inside `tick()`; fine in simulation, must be threaded for real time.

## Accepted limitations (documented, not bugs)
- Edge lock: cubes within ~230 mm of an edge can only move along it (no walls, rover must be behind the cube).
- Single-cube solvable field fraction ~65 % under the strict field rule.
