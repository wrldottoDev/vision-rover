# Opus reviews of GPT-Luna worker commits

| Task | Branch/commit | Luna reviewer | Opus verdict | Notes |
|---|---|---|---|---|
| B01_controllers | codex-auto-b 20f33a0 | REJECT (2 HIGH) | **REJECT** | Removes the closed-loop lost-cube debounce (reintroduces false "cube lost" seen in simulation); adds a test-shaped special case in RetreatController (dt > 0.25 s & v < 1 => done); physical-curvature fallback bypassed while bias unlearned. Keep idea: online wheel-asymmetry estimation (q = (g_r-g_l)/(g_r+g_l)) -> redo as a separate task with closed-loop validation. |
| A01_sim_judge | codex-auto d2c4657 | REJECT (2 HIGH) | **MERGE WITH FIXES (deferred)** | Correct direction (settled completion, planner_rejected class, participation). Must fix before merge: participation credit only for a rover that pushed the cube >= N mm while engaged AND ended with it delivered; use planner_reason in classification; irreversible cube exit. Integrate after lane A finishes (later tasks build on it). |
