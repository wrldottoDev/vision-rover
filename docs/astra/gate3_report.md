**GATE 3: FAIL — changes required.** The implementation does not preserve its stated reservation or cube-engagement invariants, and several recovery paths cannot make progress.

I read all requested files and checked the supporting modules as they became available. I reproduced the main transitions and contract failures with in-memory probes; missing services were stubbed where necessary. These were isolated checks, not an end-to-end simulation. No files were modified.

1. **CRITICAL — Interrupts lose the fact that a cube occupies the channel.**  
   [fsm.py:248](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:248), [fsm.py:205](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:205).

   **Failure:** `VERIFY_CAPTURE → EMERGENCY_STOP → clear_estop()` becomes `NAVIGATE`, because `_resume()` omits `VERIFY_CAPTURE`. Similarly, `order_yield()` accepts `EMERGENCY_STOP`, `RELOCALIZE`, and `RETREAT` even when the interrupted maneuver has a cube engaged. It overwrites the saved state and permits navigation/rotation without completing retreat. The first transition was reproduced directly.

   **Fix:** Track `channel_may_be_occupied` independently of the FSM state. Preserve it across every interruption. Until fresh geometry proves clearance, permit only a validated retreat or stop; prohibit navigation, alignment turns, and parking maneuvers.

2. **CRITICAL — Manipulation resumes after its reservation has been released.**  
   [supervisor.py:249](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:249), [fsm.py:248](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:248).

   **Failure:** A rover pushing east loses localization. The next refresh replaces its entire corridor with its current envelope and clears `committed[rid]`. The other rover can reserve the vacated downstream corridor. Recovery returns the first rover to `VERIFY_CAPTURE`, then `PUSH`, without calling `commit_region()`. Resuming `RETREAT` has the same defect.

   **Fix:** Retain the occupied/braking region while stopped. Require successful reacquisition of the complete remaining maneuver before every motion-producing resume.

3. **CRITICAL — Reservation refresh can create overlapping reservations, and execution never checks containment.**  
   [supervisor.py:254](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:254), [supervisor.py:229](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:229).

   **Failure:** After a pose correction or tracking deviation, `_refresh_reservations()` appends the actual envelope and calls `reserve()` without conflict validation. The available reservation implementation replaces entries unconditionally. I reproduced a refresh leaving mutually conflicting reservations. No final command check verifies that the commanded swept footprint remains inside the committed region.

   Sequential execution does not make `conflicts()` followed by `reserve()` inherently racy; the unchecked refresh and subsequent motion are the problem.

   **Fix:** Validate all refreshed occupancies together, stop affected rovers on violation, and require the swept motion plus stopping envelope to remain inside an approved reservation.

4. **CRITICAL — Capture’s distance origin disagrees with the controller contract.**  
   [fsm.py:401](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:401), [controllers.py:342](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:342).

   **Failure:** With cube centre `x=500`, rover prepush `x≈326.09`, heading east, the FSM computes capture travel `≈102.91 mm` and passes the **cube centre** as the controller’s line origin. The controller compares rover position relative to that origin against `102.91`. It therefore targets rover `x≈602.91`, rather than `429`: approximately **174 mm of unintended additional advance**. A direct probe confirmed that capture still commands forward motion at the intended stopping pose.

   **Fix:** Define capture explicitly as either start pose plus travel, or an absolute contact endpoint. Both caller and controller must use that same reference.

5. **CRITICAL — The retreat controller turns immediately and measures progress in the wrong direction.**  
   [fsm.py:509](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:509), [controllers.py:492](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:492).

   **Failure:** Retreat constructs a line with `path_dir=current_heading` and `reverse=True`. `_Line.reference_heading` then adds π, requesting a 180° heading change. Backward travel makes `along` negative, so the requested distance is never reached. Starting at `(423,430,0)`, the actual controller returned `(-25 mm/s, +0.5 rad/s)` immediately and still did so after an 80 mm backward displacement.

   **Fix:** Use a backward path direction with the original body heading, or a dedicated straight-retreat controller. Verify positive backward progress and prohibit intentional turning until channel clearance is established.

6. **CRITICAL — Failed retreat clearance expires after 50 ms; timeout counts as successful clearance.**  
   [fsm.py:503](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:503), [supervisor.py:162](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:162).

   **Failure:** At dwell `0.01 s`, `retreat_clear=False` stops motion. At `0.05 s`, the check is skipped and the controller executes. If a board guard or obstruction prevents retreat, six seconds later `_after_retreat()` permits replanning/navigation or reports delivery anyway. The clearance function checks only other reservations: a delivered cube directly behind the rover is invisible to it.

   **Fix:** Maintain clearance throughout retreat, checking board, cubes, obstacles, and reservations. Timeout must preserve engagement and enter blocked recovery; only observed displacement and geometric clearance may authorize turning.

7. **HIGH — `_leg_region()` does not reserve the maneuvers actually executed.**  
   [fsm.py:320](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:320), [fsm.py:387](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:387).

   **Failure:** For prepush `(326,430,0)` and cube endpoint `(650,430)`, the reserved leg begins at envelope `x=279`. An early capture failure at rover `x=330` followed by the fixed 80 mm retreat reaches envelope `x=203`. An approach from the south need not reserve that western area.

   Alignment can also rotate and make lateral nudges outside the fixed straight corridor. Later, `_s_verify_capture()` recomputes a push line to the old endpoint without checking heading change or reserving it. A short leg whose endpoint is passed during capture can even produce a reversed reference direction.

   **Fix:** Reserve alignment, capture, payload, and abort sweeps from actual poses. Any changed push line must be geometrically revalidated and recommitted before execution.

8. **CRITICAL — Clock alignment can classify substantially stale telemetry as fresh.**  
   [supervisor.py:73](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:73).

   **Failure:** If the capture clock is one second ahead of the local clock, the measured offset is approximately `−1 s`, but the `abs(offset)>2` condition suppresses correction. Captures are then future-dated. With reception at local `100` and capture timestamp `101`, the estimator still reported `GOOD`, age zero, at local `100.8` in a direct probe.

   For larger offsets, interpreting minimum arrival-minus-capture as pure clock offset also hides baseline transport latency.

   **Fix:** Use a consistent clock mapping with an uncertainty bound, reject future-dated observations, and include clock/latency uncertainty in freshness decisions. Never condition clock correction on an arbitrary two-second threshold.

9. **CRITICAL — Synchronous planning can emit commands based on expired telemetry.**  
   [supervisor.py:205](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:205), [supervisor.py:422](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:422).

   **Failure:** `tick(t)` checks freshness once, then runs allocation, navigation, and potentially seven navigation searches during parking selection. Navigation has a one-second default search deadline. Commands are finally evaluated and emitted using the original `t` and estimates. A frame fresh at entry can be seconds stale at output. Meanwhile, the previous hardware command may still be active.

   **Fix:** Separate bounded planning work from the command/watchdog loop. Recheck actual monotonic time, telemetry age, world revision, and safety immediately before issuing commands.

10. **HIGH — One unseen rover is treated as absent from collision avoidance.**  
    [supervisor.py:215](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:215), [supervisor.py:328](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:328).

    **Failure:** The first running frame contains rover 10 but rover 11’s marker is occluded. `len(ests)==1` permits allocation and motion. Rover 11 receives STOP, but its physical body has no reservation; pairwise estop is skipped. Rover 10 can plan through its location.

    **Fix:** Require localization of both physical rovers before initial motion, or represent the missing rover by a conservative known occupancy region that blocks conflicting motion.

11. **HIGH — The reactive safety check assumes commands instantly become velocities.**  
    [supervisor.py:349](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:349), [supervisor.py:317](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:317).

    **Failure:** `_with_cmd()` replaces measured velocity with the command. After STOP, collision checking assumes immediate rest. Two rovers approaching at `110 mm/s` with a 30 mm remaining gap need about 40 mm combined stopping distance at the configured `300 mm/s²`, even without command latency.

    The board guard also checks only one future envelope. At `x≈749.5`, rotating from roughly 10° to 42° leaves endpoint envelopes inside its accepted bounds while the intermediate paddle sweep exceeds them.

    **Fix:** Check swept reachable/braking envelopes using estimated current velocity, acceleration and latency bounds. Sample or conservatively bound the whole board sweep.

12. **HIGH — Cube occlusion permits continued blind pushing.**  
    [fsm.py:457](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:457), [controllers.py:418](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:418).

    **Failure:** Rover telemetry remains fresh while the cube disappears or escapes the channel. The FSM passes `cube_xy=None`; the controller substitutes rover pose plus nominal contact offset and explicitly reports `lost_cube=False` when vision is unavailable. A direct call with a five-second-old unseen cube still commanded forward motion.

    **Fix:** Give cube engagement its own observation-age, uncertainty, and blind-distance budget. Expiry must stop manipulation and require reacquisition; fresh rover pose cannot establish continued cube contact.

13. **HIGH — Capture verification accepts geometrically impossible channel occupancy.**  
    [fsm.py:432](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:432).

    **Failure:** Verification accepts lateral displacement up to `46.75−30+4 = 20.75 mm` regardless of orientation. A 45° cube permits only **4.32 mm**. Thus a measured cube at local `(89.4,20)` passes even though its projected shape intersects the paddle region. The depth interval also allows substantial gap or apparent penetration without proving contact.

    **Fix:** Test the cube’s worst-case projected footprint against the channel over its orientation and pose uncertainty. Contact verification needs consistent fresh relative-motion/contact evidence, not a centre-only rectangle.

14. **HIGH — Orientation inference asserts information official telemetry does not supply.**  
    [fsm.py:441](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:441), [fsm.py:475](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:475).

    **Failure:** A 45° cube has true contact depth `42.43 mm`. A plausible `−12 mm` combined relative-position bias makes it appear flush and assigns heading with a 7° standard deviation. At depot centre offset 10 mm, that narrowed belief can accept a cube whose actual footprint extends outside the depot. Independently, every successful nonfinal push assigns orientation from rover heading without checking depth at all.

    **Fix:** Preserve unknown orientation unless a calibrated observation model supports narrowing it. Propagate offset/contact uncertainty, retain ambiguous orientations, and invalidate or widen the belief during transport. Do not infer orientation merely because a push completed.

15. **HIGH — Delivery confirmation does not require fresh, distinct, stationary observations.**  
    [supervisor.py:99](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:99), [supervisor.py:166](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:166).

    **Failure:** Six samples ending **650 ms ago** still returned delivered in a direct probe. Samples need not follow entry into `VERIFY_DELIVERY`, and no stationarity test exists. Object observations repeated with age up to 60 ms are appended again, so the frame count can contain duplicate observation timestamps. Confirmation precedes retreat and is not repeated before `on_delivered()`.

    **Fix:** Count distinct object capture times after motion has settled, require a recent final observation and bounded displacement over the confirmation interval, then verify again after disengagement. Compute orientation-interval extrema exactly, including any interior 45° maximum.

16. **HIGH — Cached motion ignores cubes that move after planning.**  
    [fsm.py:285](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:285), [supervisor.py:312](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:312).

    **Failure:** Rover 10 commits a path near a delivered cube. The cube is subsequently displaced into that path by rover 11’s retreat or imperfect push. Rover 10 continues the cached path: navigation checks displacement of its own target, not other cubes, and the reactive layer checks only board and rover–rover collision. It can knock the delivered cube farther out.

    **Fix:** Associate paths with obstacle-world revisions and revalidate remaining sweeps when cubes move. Include all non-target cubes, especially delivered ones, in the final motion guard.

17. **HIGH — Normal WAIT retries prevent deadlock detection.**  
    [fsm.py:263](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:263), [supervisor.py:369](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:369).

    **Failure:** Every 0.5 seconds, WAIT transitions to NAVIGATE or YIELD. `_resolve_deadlock()` clears `wait_since` whenever the state is no longer WAIT/ESTOP. Consequently the two-second threshold never arrives during ordinary repeated reservation denial. An isolated 60-second WAIT/NAVIGATE trace produced zero yield orders.

    **Fix:** Measure time since meaningful physical/task progress across retry states. Clear that timer only on progress or resolution of the blocking dependency.

18. **HIGH — Retry limits do not bound several actual failure loops.**  
    [fsm.py:287](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:287), [fsm.py:302](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:302), [fsm.py:337](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:337).

    **Failure:** Reservation denial, navigation timeout, and tracking deviation return WAIT without increasing task failures. Each new path resets `nav_t0`. A rover whose wheels never move can therefore repeat valid-path/time-out cycles forever. Cube-moved and changed-heading replans also bypass failure accounting. Conversely, `nav_fail` is not reset after successful navigation, so unrelated old failures accumulate. Estop clear/retrigger cycles can repeatedly reset push timing.

    **Fix:** Maintain per-maneuver and per-cube cumulative no-progress budgets, counting these failure classes. Preserve budgets through interrupts; reset appropriate counters only after verified progress.

19. **HIGH — Parking is a destination heuristic, not a committed deadlock-resolution maneuver.**  
    [supervisor.py:403](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:403), [fsm.py:356](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:356).

    **Failure:** Only the six cheapest endpoint candidates are searched. Six nearby unreachable poses can hide a reachable farther parking pose. The selection checks endpoint clearance from the keeper’s intended region, but does not coordinate the evacuation path with that intention. Even navigation failure is handled like arrival, and after successful parking a tasked rover retries its original task after 0.5 seconds without waiting for the keeper to finish.

    **Fix:** Create a persistent keeper/yielder episode: validate and commit evacuation, require verified parking arrival, hold there until an explicit release condition, and search additional candidates when the nearest six fail.

20. **HIGH — An incomplete first observation can permanently finish the mission.**  
    [supervisor.py:263](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:263), [supervisor.py:275](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:275).

    **Failure:** With healthy rover estimates but no cubes yet observed, `all(...)` over the empty set is true and both agents become `DONE`; reproduced directly. Later cube detections cannot restart them because allocation admits only `IDLE`. Once both are DONE, displaced-delivery monitoring also stops because `_allocate()` returns before checking delivered cubes.

    **Fix:** Establish mission inventory before declaring completion. Monitor delivery validity independently of allocation and permit automatic reopening when a cube appears or delivery is undone.

21. **HIGH — Allocation loses the busy rover’s lock and can starve the idle rover.**  
    [supervisor.py:267](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:267), [supervisor.py:291](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:291).

    **Failure:** The supervisor passes plans only for unclaimed cubes, but includes busy rovers in `rovers` and their claimed colours in `locked`. The current allocator retains a lock only if that colour is also in `plans`. In a direct example, busy rover 10 locked to red received the free blue task in the allocation while idle rover 11 received nothing. The supervisor then ignores rover 10’s allocation because it is busy.

    **Fix:** Either allocate only idle rovers against free tasks, or include explicit immutable active-task commitments, remaining costs, and future availability in the allocator input.

22. **HIGH — “Both rovers participate” is enforced per allocation batch, not per mission.**  
    [supervisor.py:184](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:184), [task_allocator.py:238](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/planning/task_allocator.py:238).

    **Failure:** No per-rover delivery history is retained. If only one cube is currently visible/plannable at each allocation, the nearer rover can receive all three sequentially. The “both” constraint never activates, even with both rovers healthy throughout.

    **Fix:** Track each rover’s verified participation and enforce the remaining mission obligation across observations, cooldowns, and replanning—not merely when two plans happen to be available together.

23. **HIGH — Current shared interfaces raise exceptions before normal operation.**  
    [supervisor.py:97](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:97), [supervisor.py:295](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:295), [fsm.py:507](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:507).

    **Failure:** Three concrete mismatches exist in the available implementations:

    - `CubeTracker(color, self.cfg)` passes `Config` where `CubeTrackerConfig` is expected; construction raises missing `init_pos_std`.
    - Allocation returns `.assignment` containing `RoverTask`, while the supervisor expects `.sequences` containing unpackable pairs.
    - Retreat exposes `_distance`, while the FSM reads `.distance`.

    These are current integration findings, not complaints about absent modules.

    **Fix:** Freeze and reconcile these public contracts, and verify ingestion, allocation consumption, and first retreat execution together. Exceptions must also cause command output to fail closed.

24. **MEDIUM — Published obstacles are discarded by the supervisor.**  
    [world.py:76](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/world.py:76), [supervisor.py:71](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:71).

    **Failure:** `Frame.obstacles` exists and the parser populates it, but ingestion never stores it. Navigation, push planning, parking, and retreat therefore ignore a nonempty official obstacle list, contradicting H9.

    **Fix:** Retain obstacles with a defined conservative geometry and pass them through every planning and safety contract.

25. **HIGH — Assumed depot geometry becomes mission truth, while the known visibility hazard remains unhandled.**  
    [config.py:88](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/config.py:88), [supervisor.py:166](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:166), [RULES_AND_CONSTRAINTS.md:28](/Users/ottogonzalez/Documents/rover/V1/docs/RULES_AND_CONSTRAINTS.md:28).

    **Failure:** The documentation labels the 100 mm depot square as assumed, but `is_delivered()` uses it to retire tasks and finish the mission. Official v1 provides neither that boundary nor a delivery verdict. Separately, two successful corner deliveries can obscure two anchors and freeze vision permanently; freshness stopping prevents further motion but leaves the third delivery impossible. Neither allocation nor the supervisor’s planning contract enforces preservation of anchor visibility.

    **Fix:** Separate engineering placement confirmation from official acceptance. Parameterize the acceptance model explicitly, and make preservation of adequate corner visibility a checked planning constraint or report the configured three-delivery mission infeasible.

**Spatial reservations versus time-expanded planning:** spatial reservations can provide safety for two rovers if they conservatively contain every executed maneuver, remain disjoint, survive interruptions, and include braking uncertainty. This implementation meets none of those guarantees consistently. Reactive estop cannot repair the missing ownership and progress rules.

A concrete liveness counterexample needs no narrow passage: put rover 10 at `(250,250,0)` and rover 11 at `(600,600,π/2)`. Give rover 10 a cube near `(600,774)` whose staging pose is occupied by rover 11; give rover 11 a cube near `(424,250)` whose staging pose is occupied by rover 10. A third cube can sit at `(700,250)`. Each single-rover navigation request rejects its occupied goal. A feasible joint solution is to park rover 11 around `(700,500)`, let rover 10 leave and acquire its staging region, then release rover 11. The current retry timer prevents that resolution from starting.

For this two-rover problem, **persistent prioritized planning with an explicit parking/holding protocol** is a reasonable first solution. Time-expanded planning or CBS could search joint alternatives, but would still require execution monitoring, uncertain-duration occupancy, and reservation repair. Adding timestamps alone would not fix these failures.