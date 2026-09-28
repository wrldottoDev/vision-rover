# Opus reviews of GPT-Luna worker commits

| Task | Branch/commit | Luna reviewer | Opus verdict | Notes |
|---|---|---|---|---|
| B01_controllers | codex-auto-b 20f33a0 | REJECT (2 HIGH) | **REJECT** | Removes the closed-loop lost-cube debounce (reintroduces false "cube lost" seen in simulation); adds a test-shaped special case in RetreatController (dt > 0.25 s & v < 1 => done); physical-curvature fallback bypassed while bias unlearned. Keep idea: online wheel-asymmetry estimation (q = (g_r-g_l)/(g_r+g_l)) -> redo as a separate task with closed-loop validation. |
| A01_sim_judge | codex-auto d2c4657 | REJECT (2 HIGH) | **MERGE WITH FIXES (deferred)** | Correct direction (settled completion, planner_rejected class, participation). Must fix before merge: participation credit only for a rover that pushed the cube >= N mm while engaged AND ended with it delivered; use planner_reason in classification; irreversible cube exit. Integrate after lane A finishes (later tasks build on it). |
| B05_wheel_bias | codex-auto-b (after B03) | REJECT (2 HIGH) | **REJECT** | Physical curvature cap removed once q "valid"; command/motion time matching uses vision-fix age against propagated EKF state. Static per-rover feed-forward (config.ROVER_MOTORS) covers the need; online adaptation shelved until hardware data. Claimed "double-counted cube extrapolation" on main verified NOT present. |
| A02_sim_sensors | codex-auto | REJECT (3 HIGH) | **pending consolidated review** | Freeze model + separate clocks direction good; issues: initial frozen state before first valid homography, capture default 20 Hz (official 30 Hz), 2 regressions in test_simulation.py. |
| A03_sim_physics | codex-auto | REJECT | **pending consolidated review** | Suite 23 -> 14 failures. Review with A04/A05. |
