# Opus reviews of GPT-Luna worker commits

| Task | Branch/commit | Luna reviewer | Opus verdict | Notes |
|---|---|---|---|---|
| B01_controllers | codex-auto-b 20f33a0 | REJECT (2 HIGH) | **REJECT** | Removes the closed-loop lost-cube debounce (reintroduces false "cube lost" seen in simulation); adds a test-shaped special case in RetreatController (dt > 0.25 s & v < 1 => done); physical-curvature fallback bypassed while bias unlearned. Keep idea: online wheel-asymmetry estimation (q = (g_r-g_l)/(g_r+g_l)) -> redo as a separate task with closed-loop validation. |
| A01_sim_judge | codex-auto d2c4657 | REJECT (2 HIGH) | **MERGE WITH FIXES (deferred)** | Correct direction (settled completion, planner_rejected class, participation). Must fix before merge: participation credit only for a rover that pushed the cube >= N mm while engaged AND ended with it delivered; use planner_reason in classification; irreversible cube exit. Integrate after lane A finishes (later tasks build on it). |
| B05_wheel_bias | codex-auto-b (after B03) | REJECT (2 HIGH) | **REJECT** | Physical curvature cap removed once q "valid"; command/motion time matching uses vision-fix age against propagated EKF state. Static per-rover feed-forward (config.ROVER_MOTORS) covers the need; online adaptation shelved until hardware data. Claimed "double-counted cube extrapolation" on main verified NOT present. |
| A02_sim_sensors | codex-auto | REJECT (3 HIGH) | **pending consolidated review** | Freeze model + separate clocks direction good; issues: initial frozen state before first valid homography, capture default 20 Hz (official 30 Hz), 2 regressions in test_simulation.py. |
| A03_sim_physics | codex-auto | REJECT | **pending consolidated review** | Suite 23 -> 14 failures. Review with A04/A05. |
| C02_arc_segments | codex-auto-c | REJECT (1 HIGH: arcs skipped where rotation free) | **MERGED** (f6d9526) | Capability needed; completeness gap tracked. |
| B03_telemetry_client | codex-auto-b | MERGE | **MERGED** (55a018d) | |
| B06_alloc_timing | tag review-b06 | — | **NOT MERGED (unreviewed)** | Allocator time model; review later. |
| B07+B08 push planner coverage | codex-auto-b | REJECT -> MERGE-WITH-FIXES | **MERGED** (5c6339b) | Symmetry, blockers, 2-leg-first with 3-leg completeness. |
| C03+C04 nav perf | tags archive-c-* | REJECT, REJECT | **REJECTED** | Coarse stride / arc-skipping lose routes; redo as C05 with constraints. |
| A01-A05 simulator | codex-auto | REJECT x4, MERGE-WITH-FIXES | **MERGED as a whole** (sim adversarial failures 35 -> 7) | HIGH findings queued as A06. |
| A07_judge_gaps | codex-auto 63fc7af + 301ec6e | REJECT (2 HIGH) | **MERGED WITH FIXES** (lead commit 301ec6e, merge into main) | rotations_with_cube / causal transport-credit / contact-rejection read `rover.wl/wr` (pre-noise lagged target) instead of the actually-realized post-noise speed. Fix: persist `wl_realized`/`wr_realized` in `_advance_rover`, read those instead; added a regression test reproducing the reviewer's exact repro. 283->284 passed, 0 regressions. |
| B10_controllers_asymmetry | codex-auto-b 3baab72 + 93b97d5 | REJECT (1 HIGH, 1 MEDIUM) | **MERGED WITH FIXES** (lead commit 93b97d5, merge into main) | HIGH (push curvature cap reactive-only once feedback available): attempted forcing the conservative physical cap unconditionally, but this regressed 5 other passing tests (coupled cap/gain tuning) -- reverted, documented inline as an open follow-up, not fixed blind. MEDIUM (fixed-direction `retreat_wheel_bias_feedforward` applied to every rover, wrong for balanced R10, duplicates the supervisor's per-rover ROVER_MOTORS feed-forward): removed; the rover-agnostic heading-integral term already added by this task is the correct mechanism. Net: 5/7 originally-failing adversarial tests fixed clean, 0 regressions (280->285 passed). 2 retreat-straightness tests remain open -- see "Open items" below. |
| C06_nav_perf_collision_only | codex-auto-c 4976379 + 637dbc2 | REJECT (1 CRITICAL, 1 HIGH, 2 MEDIUM, 1 LOW) | **MERGED WITH FIXES** (lead commit 637dbc2, merge into main) | CRITICAL (no commit had actually landed -- a `run_lane.sh` git-commit race across the 3 concurrently-running lane worktrees silently swallowed all three implementer commits): committed properly. HIGH (`_obstacle_far` squared-distance shortcut not provably conservative at tangency): reverted to `math.hypot` (it's a coarse pre-filter, not the hot loop; no measurable perf cost). MEDIUM (equivalence test only compared the new code's cache-on vs cache-off against each other, so a shared completeness regression would pass silently -- matches the reviewer's own 14/30-queries-failed observation): rewrote to replay against a frozen pre-C06 reference implementation (`tests/fixtures/navigation_pre_c06_reference.py`); all three now agree bit-for-bit on the 16/30 queries solvable at all. MEDIUM (benchmark never measured "before", just "after"): rewrote `tools/bench_navigation.py` to benchmark both; measured **1.94x** (just under the 2x target, reported honestly). LOW (trailing whitespace): fixed. 0 regressions (281 passed both before and after this lane, +40 net new lines of test/fixture coverage). |

## Open items handed off from this review round (2026-09-29)
- **B10 retreat-straightness** (2 tests, `test_retreat_wheel_truth_stays_straight_and_stops`): the bare
  `RetreatController`, tested in isolation under a full uncompensated 15% wheel asymmetry (no supervisor-level
  `ROVER_MOTORS` compensation in front of it), cannot suppress the initial transient yaw below the test's 3 deg bound
  using only reactive PID + a rover-agnostic integral (verified: hand-tuning kp/kd/tau does not reliably converge --
  see commit 93b97d5's body for the exact numbers tried). Needs a deliberate decision, not another blind gain search:
  either (a) thread real per-rover `ROVER_MOTORS` calibration into `RoverAgent`'s controller construction (touches
  lead-owned `rover/fsm.py`; must preserve DECISIONS.md #15's "EKF models the intended twist" invariant), or (b) a
  ruling that this adversarial scenario double-counts a defense the supervisor already provides, and the test should
  be re-scoped (not weakened) to feed the controller a *residual* miscalibration instead of the raw worst case.
- **B10 push curvature cap** (no failing test today, but a real gap): the reactive-only cap can lag one control tick
  behind a realised curvature excess once feedback is available. Fixing it needs a coupled cap+gain retune (verified:
  a naive "always apply the physical cap" change regresses 5 other tests) -- scope as its own task, not a quick patch.
- **C06 benchmark**: 1.94x measured vs the 2x target asked for. Given two prior attempts at this exact task were
  already rejected for changing the search (forbidden) and this one is a real, honestly-measured, zero-regression
  1.94x with a provable bit-identical-results test, further squeezing is a nice-to-have, not blocking.
