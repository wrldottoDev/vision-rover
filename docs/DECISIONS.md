# Decision log (append-only; newest last)

Format: decision — why — evidence — status.

1. **Internal units mm/rad/s, y-up frame; convert only at the telemetry boundary.** Avoids cell/row-down/degree
   mixing. Verified against the official detector sign convention (Astra Gate 1). ACTIVE.
2. **Centralised PC planner for the Python reference.** Rules allow the external PC to plan/allocate/avoid collisions
   (reglamento s6). Target split plan.py / code.py / C drivers deferred to Phase 9. ACTIVE.
3. **Two footprints**: convex envelope (incl. channel) for everything we must not touch; true U-shape only for
   physics/capture. Conservative against non-target contact. ACTIVE.
4. **Depot = 100 mm square, whole cube inside** — no official definition exists; parametric
   (`config.depot`). Placement confirmation is an ENGINEERING criterion, not an official verdict. ASSUMED.
5. **Strict field rule**: whole rover envelope inside the 860 mm field (board margin 12 mm; 5 mm on final delivery
   legs). Costs ~20 % of solvable layouts vs using the physical margin — organisers must be asked. ACTIVE / OPEN.
6. **Unicycle EKF [x,y,theta,v,omega]** instead of [x,y,vx,vy,theta,omega]: lateral velocity of a diff-drive is
   unobservable/zero; v,omega states make gyro/odometry linear future updates. Delayed measurements replayed at
   capture time. ACTIVE.
7. **Freshness from capture time only**, duplicate/frozen captures ignored (official vision can freeze). ACTIVE.
8. **Velocity-gap actuator-fault detector removed from LOST decision** (false LOST under normal motor lag in closed
   loop); stuck wheels detected by a latched signed heading-innovation bias while commanding motion. ACTIVE.
9. **Plan per leg, replan from observed state after every leg.** No stale plans. ACTIVE.
10. **Spatial (not time-indexed) reservations** with progressive trimming, plus execution guard and imminent-collision
    estop (0.45 s horizon = tick + latency + braking). Timing on real motors is unreliable. ACTIVE; deadlock handling
    still immature.
11. **Allocation by exhaustive enumeration, makespan objective, both rovers must participate** (reglamento s12). ACTIVE.
12. **Orientation belief** from contact depth only (flush => heading mod 90, std 10 deg); poorly-known orientation
    degrades to a soft risk prior instead of forbidding all headings. ACTIVE.
13. **Workflow (2026-09-28)**: Opus = architect/final auditor; GPT Luna (codex) = implementation workers on
    `codex-auto*` branches; nothing merges to main without Opus review. ACTIVE.
14. **Vision-loss policy adapted to reported latency (p95 470 ms)**: GOOD <= 0.35 s capture age; DEGRADED up to 1.5 s
    = keep executing at 0.5x speed, never start a capture; LOST beyond 1.5 s, or blind path > 60 mm, or sigma > 15 mm,
    or actuator-fault latch. Cube observations usable for control up to 0.6 s capture age (latency-compensated).
    ASSUMED values pending real latency logs. ACTIVE.
15. **Per-rover wheel feed-forward** (`config.ROVER_MOTORS`): R11 right command x0.92 (NOT FINAL), R10 x0.99. The
    estimator models the intended twist; feed-forward only compensates the hardware. ACTIVE.
16. **Arc motion primitives required**: without them edge-parallel pre-push poses are unreachable (coverage 65 % ->
    5-29 % when reachability is enforced). Delegated to Luna lane C; reachability check kept as an option. IN PROGRESS.
17. **Delivery confirmation uses the nominal orientation** (belief mean, else last flush-push heading) with a 1 mm
    margin; the worst case over the belief is logged as "marginal". Reason: with the ASSUMED 100 mm depot and 100 mm
    rovers along edges, correct placements sit within a few mm of the boundary (seed 0: truth-delivered cubes were
    rejected by the worst-case rule). Confirmation is an engineering decision (whether to re-push), not an official
    verdict. ACTIVE.
18. **Pre-push poses must allow +-6 deg alignment rotations inside the field** and the board guard keeps a 0.3 s
    rotational lookahead (a 0.15 s lookahead produced a real board exit). ACTIVE.
