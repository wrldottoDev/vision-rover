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

## Immediate next steps (priority order)
1. **Re-baseline MC on the merged simulator** (strict and 20 mm) — now unblocked (A07 judge fixes are in). This
   session ran a first pass; see results below. Scale to 50 seeds if a bigger sample is wanted.
2. B10 retreat-straightness (2 tests): decide whether to thread real per-rover `ROVER_MOTORS` calibration into the
   controller (touches lead-owned `rover/fsm.py`) or re-scope the adversarial test. See codex_reviews.md "Open items".
3. Remaining failure classes: planner_no_solution (strict geometry + clutter: use last_blockers for sequencing),
   corner near-misses (few mm; rules question), coordination liveness (deadlock in close starts, < 45 mm).
4. Organiser questions (below) — they dominate achievable success.

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
