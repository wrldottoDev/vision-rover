# Known issues (keep current; newest first)

## Blocking
- **No closed-loop delivery yet.** Single-rover: navigation, alignment, capture and first push leg work; failures seen
  in align timeouts, capture-verification after interrupted capture, and occasional replanning dead ends. Two
  rovers starting < 45 mm apart still spend long periods in WAIT/yield churn (`tools/debug_run.py 0 official_like`).
- **Simulator judging defects (Astra Gate 8, FAILED)** — `docs/astra/gate8_report.md`, 24 failing tests in
  `tests/test_simulation_astra.py`: corner-marker freeze not modelled; "physically_infeasible" class assigned from the
  planner's own conservative checks (hides planner failures); success ignores both-rover participation; completion
  declared before rovers stop/settle; plus physics/sensor gaps. Monte Carlo numbers are NOT trustworthy until fixed.
- **Navigation planning is slow** (~0.3-1.5 s per query, dominates simulation time) — Python hull/SAT in hot loop.

## Important
- Telemetry policy assumes ~50-150 ms latency; the team reports p95 470 ms / max 1420 ms. With `lost_age_s = 0.4`
  rovers would stop constantly. DEGRADED-mode driving (slow, no capture start) is NOT IMPLEMENTED.
- Protocol v2 (USER-REPORTED) not supported; no schema in repo.
- Marker offset ~24 mm (USER-REPORTED) not yet applied; must confirm whether the deployed vision compensates it.
- Motor model in the simulator is random asymmetry, not the measured R10/R11 characterisation.
- Controllers: 8 adversarial failures in `tests/test_controllers_astra.py` (retreat straightness under 15 %
  asymmetry + latency; push cross-track saturation under 15 % asymmetry; lost-cube boundary) — root causes in
  `docs/astra/gate6_report.md` and the I2 worker report summarised in HANDOFF.
- `tests/test_controllers.py` 1 failure after the lost-cube latency change (needs re-baselining).
- `tests/test_navigation_astra.py`: FSM goal-tolerance-vs-align near edges (1 test).
- Supervisor plans synchronously inside `tick()`; fine in simulation, must be threaded for real time.

## Accepted limitations (documented, not bugs)
- Edge lock: cubes within ~230 mm of an edge can only move along it (no walls, rover must be behind the cube).
- Single-cube solvable field fraction ~65 % under the strict field rule.
