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
