# Global planner ("plan.py" role)

Today implemented inside `rover_strategy/` and driven by `coordination/supervisor.py`. A standalone `plan.py`
entry point is NOT IMPLEMENTED yet (Phase 9).

## Pipeline per allocation event (a rover becomes idle)
1. **World model** — `Supervisor.ingest`: parse frame, map capture clock to local clock (min offset over 20 s window),
   drop duplicate/future/frozen frames, update rover EKFs and cube trackers.
2. **Push plans per cube** — `planning/push_planner.py::PushPlanner.plan(cube, depot, other_cubes, W, H)`:
   - delivery candidates D in the depot eroded by the rotated cube (+ `depot.delivery_margin`); worst case over the
     orientation belief; corner-marker overlap penalised.
   - legs C -> [I1 -> [I2]] -> D, straight pushes; elbow points (D.x, C.y)/(C.x, D.y) + grid; each leg checked for
     pre-push pose (field + rotation clearance), rover corridor from pre-push through contact to leg end (field,
     other cubes), cube sweep, retreat. Final leg may use `margins.board_final_leg`; legs starting at intermediate
     points use `intermediate_slack_mm`.
   - orientation window (H-11): known orientation -> heading within 20 deg of a face normal; uncertain -> soft risk.
   - ranked by (risk, cost). `classify_unsolvable` explains impossible cases (e.g. edge-locked cubes).
3. **Allocation** — `planning/task_allocator.py`: all cube->rover assignments x orders x top-k plans; rover time =
   nav estimate + push cost + overheads; objective = makespan + lambda*sum + risk + spatial conflict penalty;
   precedence (corridor over another cube), locked in-progress tasks, both-rovers rule. Supervisor additionally keeps
   a mission-level participation rule.
4. **Navigation to pre-push** — `planning/navigation.py::NavPlanner.plan(start, goal, obstacles, W, H)`: Hybrid A*
   (x, y, theta-bin) with straight +-40 mm and in-place rotation primitives, exact RTR analytic connector kept as
   incumbent, continuous rotation-sweep checks (arc-error padding), start escape when only inside margins, goal
   tolerance 5 mm / 3 deg, swept polygons are guaranteed supersets (used for reservations).
   Obstacles: cubes as discs (target cube with a tighter margin), the other rover's reservation polygons (padded by
   min(45 mm, current distance - 3 mm) per polygon).
5. **Reservations** — commit region = path sweep + manipulation region (rotation disc at pre-push, capture/push
   corridor, retreat sweeps). Trimmed after each completed segment. Denied commits -> WAIT; stuck -> yield/parking.

## Known gaps
- Joint optimisation of delivery point / route / conflict schedule is partial (allocator uses plan costs + Euclidean
  nav estimates, not actual A* times).
- Obstacles (yellow blocks) supported as discs but untested (edition 1 has none).
- Throughput: navigation dominates CPU; see KNOWN_ISSUES.
