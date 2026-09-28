# Opus review gates (final-auditor verdicts)

Numbering follows the 2026-09-28 brief. Evidence: independent GPT-Astra audits in `docs/astra/`, closed-loop runs,
unit/adversarial tests. Re-evaluate after each significant merge. _Last review: 2026-09-28._

| Gate | Verdict | Why |
|---|---|---|
| 1 Repository evidence + rules | **PASS WITH ISSUES** | Rules/contract read and cross-checked (Astra Gate 1). Open: depot size/criterion, "leaving the surface" definition, v2 protocol, marker-freeze risk — all require organisers/team input (HANDOFF). |
| 2 Measurements + provenance | **PASS WITH ISSUES** | Every constant tagged in config.py / MEASUREMENTS.md. Axle position, marker offset, motor test duration, latency logs are missing; user-reported values not yet verifiable. |
| 3 Geometry + collision | **PASS** | SAT/distance primitives tested; footprint envelope vs U-shape separation sound; depot erosion exact for squares; continuous rotation sweeps with arc padding (Gate 5 fixes). Board contract made consistent across planners (12 mm; 5 mm final leg). |
| 4 Architecture + interfaces | **PASS WITH ISSUES** | Clear module boundaries; lead-owned FSM/supervisor. Issues: supervisor mixes global-planner and executor roles (target split deferred); synchronous planning in tick (must be threaded for real time). |
| 5 Localization / Kalman | **PASS WITH ISSUES** | Unicycle EKF with replay, gating, consistent re-init, fault latch; 40/40 tests incl. adversarial. Closed-loop retuning removed false LOST. Issue: ~2 deg / ~4 mm bias from unknown marker offset is unobservable without calibration; thresholds ASSUMED vs real latency. |
| 6 Navigation | **PASS WITH ISSUES** | Arc primitives merged (C02). Edge-parallel poses within ~1 mm of the strict limit remain geometrically unreachable (rules question). Slow; perf task C03 running. |
| 7 Capture / push control | **PASS WITH ISSUES** | Closed loop: align, capture (depth 27-29 mm, lateral < 2 mm), verified, push leg completed. Adversarial failures remain under 15 % asymmetry + 150 ms latency (lane B). Orientation inference is model-dependent. |
| 8 Task allocation | **PASS WITH ISSUES** | Exhaustive makespan allocation with both-rovers rule; nav time estimates are Euclidean, not planner-based. |
| 9 Dual-rover coordination | **FAIL** | Safety invariants hold in tests (0 collisions observed) but liveness is poor: close starts cause long WAIT/yield churn; no full two-rover mission completed. |
| 10 Simulator fidelity | **FAIL (in progress)** | Astra Gate 8: judge defects (false success/infeasible labels), no marker freeze, kind dynamics. Lane A assigned. Measured R10/R11 motor model pending (lane A04). |
| 11 Monte Carlo failures | **FAIL (baseline taken)** | 12-seed baseline: 1/36 cubes. Root causes: strict-rule geometry, cluttered-layout corridor blocking, stalls. Tasks queued (B07). |
| 12 Integrated system | **FAIL** | First end-to-end deliveries achieved (seed 0: 2/3 with both rovers); random layouts mostly fail. SIMULATION ONLY, NOT PHYSICALLY VALIDATED. |
