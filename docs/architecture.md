# Architecture (Python reference implementation)

Centralised PC-side brain (legal per reglamento s6) consuming official telemetry v1 and emitting per-wheel speed
commands (mm/s) to each rover. Embedded code later only needs: command receiver + wheel speed loop + watchdog.

```
TCP NDJSON (vision / sim emulator)
   -> vision/parser.py      official cells/row-down/deg -> Frame (mm, y-up, rad, capture times)
   -> coordination/supervisor.py
        estimation/pose_estimator.py   EKF per rover [x,y,theta,v,omega], delayed-measurement replay, gating, GOOD/DEGRADED/LOST
        estimation/cube_tracker.py     per cube position filter, stale-growth, unexpected-move detection, orientation belief
        planning/push_planner.py       delivery candidates (eroded depot), multi-leg push plans, feasibility, ranking
        planning/task_allocator.py     exhaustive assignment x order x plan choice, makespan objective, both-rovers rule
        planning/navigation.py         lattice/hybrid A* over (x,y,theta) with oriented footprint, rotate/straight primitives
        planning/reservations.py       spatial corridor reservations (not time-indexed), priorities, imminent-collision check
        rover/fsm.py                   RoverAgent FSM per rover (states, timeouts, failure paths)
        control/controllers.py         segment follower, align, capture, push (heading + cross-track PD), retreat
   -> WheelCommand per rover (and later ESP-NOW status: coordination/protocol.py)
simulation/  physics (quasi-static pushing, U-shaped rover), sensors (official-format messages with latency, drops,
             occlusion, bias), scenarios (seeded families), runner + monte_carlo (metrics, failure classes, seeds)
```

## Key decisions (and why)
1. **Units/frames** mm/rad/s, y-up; conversion only at parser/publisher (geometry_model.md).
2. **Two footprints**: convex envelope for everything we must not touch; true U-shape only for capture/physics.
3. **Plan-per-leg**: a task replans from the observed cube after every leg; stale plans are never executed.
4. **Spatial reservations** instead of time-indexed ones: execution timing is unreliable (motor lag, 50-300 ms vision
   latency); a rover moves only inside its committed region, and committed regions are disjoint. WAIT is an action.
   Deadlock -> the lower-priority / task-less rover is sent to a parking pose outside the other's intended region.
5. **Two safety layers**: reservations (planning) + imminent-collision / board guard on commanded twists (reactive).
6. **Estimation**: unicycle EKF (a diff-drive cannot slide sideways, so a [x,y,vx,vy] model has an unobservable
   lateral mode); vision updates replayed at capture time; prediction never licenses blind driving (LOST => stop).
7. **Freshness by capture time only** (Astra CRITICAL: the official vision can freeze with rising seq).
8. **Cube orientation**: unknown until inferred from contact depth at capture (flush => heading mod 90) — a belief with
   std, used as worst case over +-2 sigma for depot erosion.
9. **Allocation**: exhaustive (<= 24 orderings x plan choices), makespan + robustness penalties, both rovers must work.
