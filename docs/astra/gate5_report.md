**GATE 5: FAIL.** Added only [tests/test_navigation_astra.py](/Users/ottogonzalez/Documents/rover/V1/tests/test_navigation_astra.py), with independent geometry checks and dense motion verification.

Ran the requested command:

```text
.venv/bin/python -m pytest -q tests/test_navigation_astra.py
28 failed, 9 passed in 0.74s
```

1. **HIGH — “Straight” segments require strafing.**  
   [navigation.py:464](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:464) accepts up to 18 mm lateral displacement without changing heading. For `(300,300,0) → (340,318,0)`, the real follower curves outside even the **6 mm-inflated reservation after approximately 0.105 s**, while remaining below FSM deviation thresholds. Forward and reverse connectors are affected.  
   **Fix:** require collinear straight endpoints; otherwise generate and validate explicit rotations and translations.

2. **HIGH — Rotation sampling misses obstacle contact and board crossings, including during escape.**  
   [navigation.py:118](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:118), [navigation.py:424](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:424). A 10° rotation misses a cube-sized disc contacted around 2.5°. Polygon reservations and board boundaries have equivalent counterexamples. A two-disc layout makes the **default-inflation escape** return rotations penetrating a real obstacle despite improved endpoint clearance.  
   **Fix:** check conservative continuous rotation sweeps, including escape moves. Per-interval endpoint hulls require conservative arc-error padding; merely reducing sampling spacing remains unsound.

3. **HIGH — Navigation tolerance demands unsafe ALIGN corrections near edges.**  
   [navigation.py:522](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:522), [controllers.py:313](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:313). Start `(300,86,0)`, goal `(300,68,0)` returns “arrived.” ALIGN then commands a corrective nudge crossing the physical field boundary at approximately **1.82 s**. ALIGN’s 3 mm target mitigates the tolerance mismatch in open space, but does not establish maneuver feasibility.  
   **Fix:** use capture-compatible terminal tolerances and collision-check/reserve correction maneuvers before execution.

4. **HIGH — Already-at-goal paths fail the FSM at ordinary headings.**  
   [navigation.py:388](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:388), [fsm.py:376](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:376). The zero-length straight segment gets an eastward tracking reference. At headings ±90° or 180°, the FSM reports `deviated` despite being exactly at the goal, potentially exhausting the task budget through replanning.  
   **Fix:** explicitly handle zero-motion arrival without constructing a directional straight segment.

5. **HIGH — Revalidation drops planning constraints.**  
   [supervisor.py:255](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:255). `path_still_clear` ignores other-rover reservations, removes footprint uncertainty inflation, and uses zero board margin. All three independently return clear for paths violating planning constraints.  
   **Fix:** preserve the planning collision context during revalidation, with explicit handling for authorized escape prefixes.

6. **MEDIUM — Rotation reservation polygons under-approximate swept area.**  
   [navigation.py:144](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:144). Sampled convex hulls omit corner arcs between samples; 3° intervals can omit approximately **0.039 mm** radially. Tests fail with both raw and inflated footprints, including angle wrapping. The normal 45 mm separation margin mitigates this particular error; an end-to-end two-rover collision was not demonstrated.  
   **Fix:** conservatively pad sampled hulls by the angular interpolation error or use enclosing rotation polygons.

7. **MEDIUM — Deadline excludes substantial work.**  
   [navigation.py:373](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:373), [navigation.py:552](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:552), [navigation.py:587](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:587). Escape runs with an expired budget; an expensive analytic connection can return success after expiry; smoothing has no deadline.  
   **Fix:** propagate one absolute deadline through every phase and check before returning.

8. **MEDIUM — Search cost guarantees fail.**  
   [navigation.py:513](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:513) overestimates cost to accepted tolerance goals: **1.843115 s heuristic versus 1.818182 s feasible cost**. Separately, [navigation.py:555](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:555) immediately accepts an analytic path costing **2.336257 s**, although a legal lattice path costs **2.199270 s**.  
   **Fix:** make the heuristic tolerance-aware; retain analytic connections as incumbents until the search lower bound justifies termination.

9. **LOW — Heading cache changes collision results with insertion order.**  
   [navigation.py:221](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/navigation.py:221). Headings `0` and `0.000004` share a key. Warming the cache at zero hides a board crossing detected by the public checker. The advertised public final-path validation is absent.  
   **Fix:** use exact heading keys or conservatively bounded quantization, plus final validation.

Valid straight sweeps passed independent forward/reverse checks, including thin obstacles between endpoints. Additive inflation/board margins, blocked-gap rejection, and repeated open-field determinism also passed.