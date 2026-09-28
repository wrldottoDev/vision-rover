# Project context — Vision Rover Challenge (read this first)

## 1. The challenge
Universidad Cenfotec's *Vision Rover Challenge*: two official CenfoBot rovers (ESP32 IdeaBoard, differential drive,
two front "paddles" forming a U channel, IMU, IR, ultrasonic, colour sensor, ESP-NOW/Wi-Fi) must autonomously push
three 60 mm cubes (red, green, blue) into the depot of the matching colour. An overhead camera + official vision
software publishes the world state. After the start no human input is allowed. Scoring: mainly completion time and
number of correctly delivered cubes; both robots must participate. Round ~600 s (USER-REPORTED).
Hard rules and our interpretations: `docs/RULES_AND_CONSTRAINTS.md`. Evidence: `_source/` (official repo + prior
research), inventory in `docs/repository_findings.md`.

## 2. Geometry (details and provenance: `docs/MEASUREMENTS.md`, `docs/geometry_model.md`)
- Field 860 x 860 mm (43 x 43 cells of 20 mm) inside a 1000 mm physical board, **no walls**. Depots at the three
  non-start corners, 2.5 cells in from the edges; the depot point is the inner corner of a 100 mm corner ArUco.
- Cube 60 mm. Rover 99.5 x 149 mm incl. 55 mm paddles; channel 93.5 mm; circumscribed radius 113.5 mm.
- Capture tolerance: lateral slack (93.5 - 60(|cos d|+|sin d|))/2 = 16.75 mm aligned, **4.32 mm at 45 deg** — of the
  order of vision noise. Cube orientation is NOT in the telemetry.

## 3. Vision / telemetry
TCP 2026, NDJSON one JSON per line, ~20 Hz, last value wins. Fields (v1): grid, rovers {id, col, row, theta deg CCW,
age_ms}, cubes {color, col, row, age_ms}, obstacles (empty), start, depots {color, col, row}, phase
(IDLE/READY/RUNNING/FINISHED). Row grows DOWN. `ts_ms` = capture time. Occluded objects keep their last position with
growing `age_ms`. If two corner markers are occluded the official engine freezes (seq advances, ts_ms frozen).
The team reports a newer **v2** protocol and real latency p95 470 ms / max 1420 ms — not in the repo (UNVERIFIED);
the parser currently accepts v1 only.

## 4. Architecture (target vs today) — `docs/ARCHITECTURE.md`
Target: `plan.py` (global planner/coach) -> per-rover `roverNN/code.py` executors (FSM, control, local estimation)
-> C drivers (motors, IMU, IR, sonar, ESP-NOW); vision provides global correction.
Today: everything is the Python package `rover_strategy/`, run centrally on the PC inside a closed-loop simulator.
`coordination/supervisor.py` plays both "plan.py" and "both code.py" roles; `rover/fsm.py` + `control/` are the
executor logic that will later move to the rovers. Phase 9 (embedded split) must wait for validated algorithms.

## 5. Algorithms (status)
| Area | Module | Approach | Status |
|---|---|---|---|
| Frames/units | `frames.py`, `vision/parser.py` | mm/rad/s, y-up; conversion only at the parser | implemented, tested |
| Localization | `estimation/pose_estimator.py` | EKF [x,y,theta,v,omega] (unicycle, no lateral slip), delayed-measurement replay, Mahalanobis gate, consistent-cluster re-init, heading-innovation actuator-fault latch, GOOD/DEGRADED/LOST | implemented, tested; policy for long real latency (470 ms p95) NOT yet adapted |
| Cubes | `estimation/cube_tracker.py` | per-cube static-with-jumps KF, stale-uncertainty growth, orientation belief | implemented, tested |
| Delivery geometry | `geometry/zones.py`, `planning/push_planner.py` | depot eroded by rotated cube; candidate D points; multi-leg straight pushes (<=3) with pre-push/corridor/retreat checks; orientation window | implemented, tested |
| Navigation | `planning/navigation.py` | Hybrid A* over (x,y,theta) with rotate/straight primitives, oriented footprint, continuous rotation sweeps, analytic RTR connector | implemented, tested; slow (profile) |
| Control | `control/controllers.py` | segment follower, align (with nudges), capture, push (cube cross-track PD, curvature cap), retreat | implemented; some adversarial tests fail |
| Rover FSM | `rover/fsm.py` | IDLE-NAVIGATE-ALIGN-CAPTURE-VERIFY_CAPTURE-PUSH-MEASURE-VERIFY_DELIVERY-RETREAT-CONFIRM + WAIT/RELOCALIZE/ESTOP/YIELD; engaged-cube invariant | implemented |
| Allocation | `planning/task_allocator.py` | exhaustive assignment x order x plan, makespan + penalties, both-rovers rule | implemented, tested |
| Coordination | `planning/reservations.py`, supervisor | spatial (not time-indexed) committed regions, trimming, execution guard, imminent-collision estop, yield/parking | implemented; still deadlock-prone in close starts |
| Simulator | `simulation/` | quasi-static pushing (ellipsoidal limit surface), U-shaped rover, motor lag/deadband/asymmetry/latency, official-format vision emulator with drops/occlusion/outliers | implemented; Astra Gate 8 FAILED (see KNOWN_ISSUES) |
| Monte Carlo | `simulation/monte_carlo.py`, `runner.py` | seeded families, truth-judged metrics, failure classes | implemented; results NOT yet meaningful |

## 6. Key findings so far (evidence-backed)
1. **Edge lock** (physics of pushing without walls): the rover must be behind the cube; a cube closer than ~230 mm to
   an edge can only be pushed along that edge. Single-cube solvable fraction of the field (strict "stay inside the
   860 mm field"): ~65 %; if 70 mm of the physical board margin may be used: ~85 % (`tools/solvability_map.py`).
2. **Corner-depot squeeze**: pushing along an edge into a corner depot, rover half-width + board margin vs the 100 mm
   depot leaves a ~2 mm feasible band with 12 mm margins; we use 5 mm on the final leg (~11 mm band).
3. **Marker occlusion**: a delivered cube at the depot point overlaps the corner ArUco; two occluded corners freeze
   the official vision (all subsequent motion stops). Organisers' depot geometry must be confirmed.
4. **Capture**: 4.3 mm worst-case slack; orientation inferred only from contact depth (flush 30 mm vs 42 mm).
5. Diagonal final pushes put the cube at ~45 deg -> no longer fits a 100 mm depot with margin; final legs are
   axis-aligned (L-shaped two-leg plans dominate).

## 7. Status (2026-09-28)
- ~260 unit/adversarial tests; ~35 adversarial tests still failing (KNOWN_ISSUES).
- Closed loop: single rover reaches, aligns, captures and completes push legs in simulation; **a full delivery and
  a full 3-cube/2-rover mission have NOT yet succeeded** in closed loop. Monte Carlo statistics are therefore not
  yet meaningful. SIMULATION ONLY; NOT PHYSICALLY VALIDATED.
- Real hardware data: motor distance characterisation (MEASUREMENTS.md). No closed-loop hardware test yet.

## 8. Next steps — see `docs/HANDOFF.md`.
