# HANDOFF — what the next session needs to know (update after every meaningful change)

_Last updated: 2026-09-29 by Claude Sonnet 5 (continuing as lead/auditor; Otoniel handed off the repo, out of
Claude/Codex quota for the session)._

## Where things stand (2026-09-29, end of session)
- B09 (board exit, seed 107) is FIXED at the root: `e653bf8` tightened the supervisor board guard
  (`Supervisor._execution_guard`) to never let a rover cross the buffered allowed edge, and to only permit inward
  motion once already outside it. Reviewed and merged by Opus same day.
- **A07, B10, C06 all reviewed and merged to `main`.** GPT-Luna's reviewer REJECTed all three (legitimate findings
  in each, see `docs/reviews/codex_reviews.md`); the lead (Claude Sonnet 5) fixed the findings directly rather than
  re-running another Codex round, verified with the full suite each time, then merged. Net: 280 passed/7 failed ->
  **290 passed/2 failed**, zero regressions. Gate 8 simulator-judge defects and edge-parallel-nav unreachability
  (previously mis-documented as still-blocking) were independently confirmed FIXED by existing merges. Full detail
  and the two items deliberately left open (B10 retreat-straightness needs an architecture decision, not another
  patch; B10 push-curvature-cap reactive lag is documented in-code) are in KNOWN_ISSUES.md and codex_reviews.md.
- Codex quota was not exhausted this session (all 3 lanes hit `QUEUE_EMPTY`, not `QUOTA_EXHAUSTED`) — but note the
  `run_lane.sh` implementer commit silently failed to land for all 3 lanes when run concurrently (a likely git
  lock/race across worktrees sharing one `.git` object store); the lead had to commit the work manually before
  review could proceed. If re-running lanes concurrently, verify `git log` on each worktree actually advanced past
  the queue commit before trusting `docs/codex_log/*_report.md`.

## MC re-baseline (2026-09-29, official_like seeds 100-111, 600s, post A07/B10/C06 merge)
**Strict field rule (overhang 0mm)**: `runs/mc_strict_2026-09-29/summary.json` (not committed, gitignored; re-run
`PYTHONPATH=. .venv/bin/python -m rover_strategy.simulation.monte_carlo --start 100 --n 12 --family official_like
--max-time 600 --workers 8 --overhang 0 --out runs/mc_strict_2026-09-29` to reproduce). Result: **7/36 cubes
delivered** (was 1/36 stale-baseline), 0/12 missions 3/3, 0 rover-rover collisions, 0 cube exits, 0 non-target
contacts (safety clean). `failure_classes`: coordination_deadlock 8, stall 2, planner_rejected 2. Wall time ~61 min
for 12 seeds @ 8 workers.
**20mm overhang**: `runs/mc_overhang20_2026-09-29/summary.json` (finished after this doc's first draft). Result:
**9/36 cubes delivered** (vs 7/36 strict) -- a modest improvement, NOT the dramatic jump the coverage-map analysis
(PROJECT_CONTEXT 1b) would suggest, because `failure_classes` is IDENTICAL in shape to strict: coordination_deadlock
8/12 (same count), stall 3, planner_rejected 1. Safety clean again (0 collisions/exits/contacts). Wall time ~77 min.
**Conclusion: the overhang/field-margin rules question, while still open and worth asking organisers, is no longer
the dominant lever for mission completion post-A07/B10/C06 -- coordination_deadlock is, at both settings.** Fixing
the allocator/coordination issue below is now higher-value than chasing the overhang answer alone.

**Root-cause dig on seed 100 (coordination_deadlock)** via `tools/debug_run.py 100 official_like 600`: the
`coordination_deadlock` label is a THRESHOLD on `yield_failed > 3 OR commit_denied > 200` (runner.py:357) and, in
this seed, actually fires from EARLY rover-rover yield contention (`yield_failed=9`) — but the run also spends its
entire second half (t=64.65 to end, 54+s of a 120s partial run) on a SEPARATE, undocumented problem: after rover 10
delivers green, the allocator repeatedly (~every 6s) fails to plan a push for red with `no_plan: "cube 'red' blocked
by undelivered cube(s): green"` — except green IS already delivered (confirmed in the same trace at t=64.6). The
blocker-diagnosis (`push_planner._diagnose_blockers`) doesn't check delivery status, it just asks "would removing
this cube's obstacle hull unblock a plan?" — true here because the now-DELIVERED green cube's fixed resting position
in its depot physically blocks red's only corridor. Since a delivered cube must never be re-disturbed
(delivery_undone stays 0, correctly), this may be a genuine geometry dead end for this seed/layout, OR the search
just isn't trying enough alternate D-points/corridors around a known-fixed obstacle — undetermined. Symptom either
way: rover 10 sits IDLE for the rest of the run (no reassignment, no backoff) instead of the allocator recognizing
red is stuck and either giving up on it (freeing rover 10 to help elsewhere) or trying harder with the *known* fixed
obstacle in mind. **Not fixed this session** — needs its own investigation (is it geometrically solvable at all?
if not, the message and failure class should say so plainly instead of retrying every 6s for 600s; if it is, the
search needs to route around a cube it already knows is permanently fixed).

## Immediate next steps (priority order)
1. Investigate the "blocked by an already-delivered cube" dead end above — check a few more failing seeds
   (101,102,104-106,109,111 all coordination_deadlock too) to see how often this specific pattern (vs. pure early
   rover-rover yield contention) is the actual driver, then decide: smarter search around fixed obstacles, or an
   explicit "give up and reassign" path in the allocator/supervisor when a cube has failed N consecutive plans.
   Consider splitting the crude `yield_failed>3` threshold into two distinct failure classes so MC stats don't
   conflate "rovers got in each other's way early" with "a cube become permanently unplannable".
2. B10 retreat-straightness (2 tests): decide whether to thread real per-rover `ROVER_MOTORS` calibration into the
   controller (touches lead-owned `rover/fsm.py`) or re-scope the adversarial test. See codex_reviews.md "Open items".
4. Corner near-misses (few mm; rules question), Organiser questions (below) — they dominate achievable success.

## Open questions for the team / organisers (cannot be resolved from the repo)
- Official depot size and delivery criterion; depot positions vs corner markers (marker freeze risk).
- Does leaving the 860 mm effective field (but staying on the 1000 mm board) count as "salida"? **Decisive**:
  20 mm of allowed overhang raises single-cube coverage from ~30-54 % to ~63-66 % (PROJECT_CONTEXT 1b).
- Protocol v2 schema; real latency logs; whether marker offset is compensated by the deployed vision.
- Duration of the motor-distance experiments (to convert to mm/s).

## Gotchas
- `tools/*` need `PYTHONPATH=.`. Debug runs print events only at the end.
- Nav planner is slow; a 240 s simulated run can take several minutes.
- Supervisor + FSM are the lead's files; worker preamble forbids workers from editing them.
