**GATE 6: FAIL.** Added only [tests/test_controllers_astra.py](/Users/ottogonzalez/Documents/rover/V1/tests/test_controllers_astra.py).

Ran the requested command:

```text
.venv/bin/python -m pytest -q tests/test_controllers_astra.py
23 failed, 12 passed in 0.22s
```

The independent model uses 20 Hz control, 100 ms wheel lag, 0.85 left-wheel gain, 15 mm/s deadband, delayed/noisy observations, and continued integration after stopping. Cube motion is modeled separately through unilateral plate contact.

1. **HIGH — FSM disables retreat heading correction.** [fsm.py:297](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:297) zeros `omega` while engaged outside CAPTURE/PUSH, including RETREAT. With exact current feedback, retreat produces **8.39° yaw and 6.35 mm lateral displacement**.  
   **Fix:** allow bounded retreat heading correction; prohibit free rotation without disabling compensation for unequal wheels.

2. **HIGH — Recommitted corridor uses the wrong heading.** [fsm.py:548](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:548) reserves `est.pose.theta`, then starts pushing along `phi_new`. The reproduction reserves **0°** and starts a **24.15°** push.  
   **Fix:** reserve the actual feasible transition and push sweep; retreat/re-align when the new direction cannot safely be acquired while engaged.

3. **HIGH — Alignment nudges violate clearance assumptions.** [controllers.py:328](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:328), [supervisor.py:209](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/coordination/supervisor.py:209). An allowed 24 mm initial lateral error produces a footprint outside the manipulation reservation. After two nudges, rotate-back is requested with the cube’s near face **96.93 mm ahead**, inside the **102 mm** paddle reach. The execution guard may stop this maneuver, leaving ALIGN unable to finish.  
   **Fix:** plan and reserve each nudge’s complete swept footprint, including braking and cube clearance, before executing it.

4. **HIGH — Alignment reports success outside tolerance.** [controllers.py:310](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:310) treats exhausted nudges as success. Perfect-feedback reproduction finishes with **6.90 mm lateral error**, exceeding both its 3 mm target and worst-orientation capture slack.  
   **Fix:** return an explicit reposition/failure result when nudges are exhausted.

5. **HIGH — Rotation deadband compensation cannot move the wheels.** [controllers.py:171](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:171). The 0.18 rad/s minimum commands only **8.01 mm/s per wheel**. A noiseless 2° alignment remains stationary throughout the 15 s timeout.  
   **Fix:** calibrate wheel-level deadband compensation; the stated deadband requires at least approximately **0.337 rad/s** for an in-place command, with controlled settling.

6. **HIGH — Cube fusion introduces systematic position error.** [controllers.py:439](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:439). A cube exactly on its desired line but offset 12 mm inside the channel becomes a fictional **−4.8 mm cross-track error**, commanding maximum low-speed curvature. Independent cube simulations finish with errors **(+2.59, +13.14) mm** at 50 ms latency and **(+7.46, +12.98) mm** at 150 ms.  
   **Fix:** estimate actual contact depth and lateral offset with uncertainty; do not blend every observation toward a fixed centered 77 mm attachment.

7. **HIGH — Stopping does not consistently account for measurement age and coasting.** [controllers.py:372](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:372), [controllers.py:469](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:469), [controllers.py:521](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:521). Capture overshoots requested travel by **6.67 mm** at 150 ms, beyond the already included overdrive. An 80 mm retreat settles at **94.84 mm**. Separately, delayed cube fusion defeats push stopping even with an exact current rover pose.  
   **Fix:** time-align observations, predict physical stopping distance, and distinguish “stop requested” from “settled.”

8. **HIGH — Stale vision permits fictitious push completion.** [controllers.py:450](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:450), [fsm.py:566](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:566). Stale observations produce `lost_cube=False`; rover pose alone can produce `done=True`. The FSM passes observation age as zero and refreshes `last_seen_push` from buffered captures.  
   **Fix:** preserve capture timestamps, advance freshness only on new captures, represent contact as unknown when stale, and require fresh cube evidence for completion. VERIFY_DELIVERY provides a later check, but does not repair this control behavior.

9. **HIGH — Command curvature is not physical curvature.** [controllers.py:489](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:489). At 35 mm/s, a nominal 400 mm turn radius becomes **233.5 mm** under the specified asymmetry. Low-speed gain scheduling also doubles cross-track gain while halving available yaw authority, encouraging saturation.  
   **Fix:** compensate wheel gains and constrain estimated physical curvature; tune the complete delayed, saturated loop.

10. **HIGH — VERIFY_CAPTURE can wait indefinitely.** [fsm.py:549](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:549). Fresh valid observations plus repeated reservation denial bypass the verification timeout.  
    **Fix:** enforce the timeout independently of observation availability and enter bounded retreat/recovery.

11. **MEDIUM — Contact checks accept incompatible geometry.** [controllers.py:456](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/controllers.py:456), [fsm.py:514](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:514). The lost threshold accepts a face-aligned cube offset 19 mm and a diagonal cube offset 8 mm. Verification accepts depth 40/lateral 11, although a touching square with that depth cannot fit.  
    **Fix:** use orientation-dependent geometry and explicit uncertainty; classify ambiguous contact as unverified.

12. **MEDIUM — MEASURE invents orientation without validating contact.** [fsm.py:594](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/rover/fsm.py:594). A cube beside the rover yields negative depth and is labeled flush with the rover heading.  
    **Fix:** validate contact before orientation inference; otherwise retain unknown orientation.

13. **MEDIUM — Anti-windup checks the old integral only.** [pid.py:53](/Users/ottogonzalez/Documents/rover/V1/rover_strategy/control/pid.py:53). A single 50 ms integral update can jump to **50** against an output limit of 1 and remain saturated after error reversal.  
    **Fix:** constrain the candidate integral update or back-calculate from saturation.

14. **MEDIUM — Existing tests provide false confidence.** [test_controllers.py:26](/Users/ottogonzalez/Documents/rover/V1/tests/test_controllers.py:26), [test_controllers.py:291](/Users/ottogonzalez/Documents/rover/V1/tests/test_controllers.py:291). They use milder conditions, omit post-stop coasting, and test pushing without an independent cube. Their “worst-case 16–17 mm slack” claim actually describes the most permissive orientation.  
    **Fix:** retain independent geometry, wheel dynamics, FSM integration, real timeouts, and settled-position assertions.

The passing checks confirm forward/reverse signs, heading wrapping, positive backward progress, and the FSM’s projected capture origin. PID derivative filtering and setpoint-kick avoidance also pass.

The delayed-pose simulations are stress tests; EKF prediction can reduce their errors. The cube model is a documented contact limit, not calibrated hardware physics. These runs demonstrate accuracy failures, not proof of divergent instability. No separate CRITICAL or LOW finding is asserted.