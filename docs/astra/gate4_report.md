**GATE 4: FAIL.** Added only [tests/test_estimation_astra.py](/Users/ottogonzalez/Documents/rover/V1/tests/test_estimation_astra.py).

Ran the requested command:

```text
.venv/bin/python -m pytest -q tests/test_estimation_astra.py
12 failed, 13 passed
```

The existing suite passes: **15 passed**.

Findings, ordered by severity:

1. **CRITICAL — Reinitialization trusts the fifth outlier unconditionally.** [pose_estimator.py:269](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:269)  
   Five mutually inconsistent observations move a stationary estimate **500 mm** and immediately report **GOOD**. With 1 Hz, five-frame outlier bursts, the independent simulator produces **233 mm peak error and 18 reinitializations**. Reinitialization also resets freshness, allowing repeated failures to defeat the age safeguard.  
   **Fix:** Require spatial and temporal consistency among reacquisition candidates; keep the tracker LOST until recovery is confirmed. Do not treat the rejection count as evidence that the latest measurement is correct.

2. **HIGH — Future capture timestamps corrupt freshness and replay ordering.** [pose_estimator.py:235](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:235)  
   A capture at `1.0` arriving at `0.1` is accepted. At local time `0.6`, a frozen feed still reports **GOOD with age −0.4 s**. The future timestamp also poisons deduplication and creates unsorted checkpoints.  
   **Fix:** Validate clock alignment and reject/quarantine future captures before updating any timestamp, counter, or checkpoint. Clock faults must not refresh health.

3. **HIGH — Stuck-wheel motion causes rejection/reinitialization cycles and unsafe confidence.** [pose_estimator.py:192](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:192), [pose_estimator.py:263](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:263)  
   The stuck-wheel simulation accepts only **137/178 valid frames**, reinitializes **9 times**, and has **555 driveable ticks** exceeding 15 mm position error or 6° heading error. Commands repeatedly pull velocity estimates toward motion the rover cannot perform.  
   **Fix:** Detect sustained command/measurement disagreement, model wheel gain or motion bias, inflate uncertainty appropriately, and force LOST during unresolved actuator faults. Merely widening the gate is insufficient.

4. **HIGH — Process covariance depends on polling frequency.** [pose_estimator.py:219](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:219)  
   Identical elapsed time at 20 ms versus 2 ms prediction cadence gives **5.81× different velocity variances**. `σ²h²` has valid units for a constant acceleration sample, but independently resampling it at every numerical substep changes the physical noise model. Position/velocity noise cross terms are also omitted.  
   **Fix:** Define continuous noise spectral densities and discretize the complete lagged system, or give acceleration noise an explicit physical correlation time independent of integration cadence.

5. **HIGH — Command trimming can invalidate a retained replay checkpoint.** [pose_estimator.py:156](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:156)  
   Checkpoints and commands retain separate cutoff anchors. With sparse checkpoints, replay starts before the retained command history; `_cmd_active()` silently substitutes zero. A valid measurement inside the nominal history window is rejected with the short buffer and accepted with the complete buffer.  
   **Fix:** Retain commands back through the earliest retained checkpoint, including its active command, or advance that checkpoint before trimming.

6. **HIGH — Stopping a push erases accumulated cube uncertainty.** [cube_tracker.py:122](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/cube_tracker.py:122)  
   After 0.5 s of unseen pushing, `set_pushed(False)` changes reported uncertainty from **84.87 mm to 1.51 mm**, without an observation. The current mode is applied retrospectively to the entire stale interval.  
   **Fix:** Timestamp mode transitions and accumulate covariance piecewise; changing modes must preserve uncertainty already accumulated.

7. **HIGH — Cube jump detection leaves a confidently incorrect position.** [cube_tracker.py:108](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/cube_tracker.py:108)  
   A genuine 40 mm jump leaves **32 mm error with 1.5 mm reported standard deviation**. An unannounced neighbour push at 70 mm/s leaves **13.81 mm lag** and generates **12 jump flags in 20 frames**. The detector compares measurements against its lagging static estimate.  
   **Fix:** Enter a motion/reacquisition mode when displacement is detected, inflate position covariance, and distinguish sustained motion from isolated observation glitches.

8. **MEDIUM — Blind travel measures displacement, not distance travelled.** [pose_estimator.py:321](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:321)  
   Reversing can accumulate over 40 mm of travel while ending within 4 mm of the anchor. The distance guard does not trigger. The test isolates this guard; default age/covariance limits can independently stop the rover.  
   **Fix:** Accumulate absolute travelled distance through propagation and replay since the last accepted capture.

9. **MEDIUM — An update can move estimator time backwards.** [pose_estimator.py:293](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/estimation/pose_estimator.py:293)  
   After prediction to `0.3`, processing an update with `t_now=0.2` resets `_t` to `0.2`.  
   **Fix:** Preserve a monotonic current horizon and replay corrections through `max(current_time, t_now)`.

10. **MEDIUM — Existing tests overstate robustness.** [test_estimation.py:35](/Users/ottogonzalez/Documents/rover/V1/tests/test_estimation.py:35), [test_estimation.py:265](/Users/ottogonzalez/Documents/rover/V1/tests/test_estimation.py:265)  
    Truth shares the filter’s lag and integration model. The frozen-feed test even supplies future captures without asserting their rejection. All 15 tests pass despite the failures above.  
    **Fix:** Retain the independent wheel-level tests and timestamp/history contracts as required checks.

11. **LOW — `degraded_age_s` has no effect.** [config.py:125](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/config.py:125)  
    Changing this policy field changes no estimator behaviour.  
    **Fix:** Remove it or define and implement its distinct purpose.

The checks that passed matter: **F matches the implemented mean’s numerical derivative**; Joseph form, innovation/state heading wrapping, and covariance symmetry/positive definiteness passed. Normal delayed checkpoint invalidation, old-history rejection, duplicate handling, cube orientation wrapping, and age-adjusted stale initialization also passed.

At the tested 10 ms cadence, **10% and 15% wheel-gain mismatch did not falsely reject legitimate turns**: both accepted all 160 valid frames. With 150 ms latency, exactly 10% frame drops, deadband, unequal motor lag, heading crossings, and two-frame outlier bursts, position RMS was **1.85/2.20 mm**. The three-degree-of-freedom gate is structurally appropriate; the failures concern fault handling, uncertainty modelling, and reinitialization.