# HANDOFF — what the next session needs to know (update after every meaningful change)

_Last updated: 2026-09-29 by Claude Sonnet 5 (continuing as lead/auditor; Otoniel handed off the repo, out of
Claude/Codex quota for the session)._

## Where things stand (2026-09-29, picking up from 2026-09-28 ~14:35)
- B09 (board exit, seed 107) is FIXED at the root: `e653bf8` tightened the supervisor board guard
  (`Supervisor._execution_guard`) to never let a rover cross the buffered allowed edge, and to only permit inward
  motion once already outside it. Reviewed and merged by Opus same day. HANDOFF previously listed this as
  "under analysis" — that was stale; treat it as closed pending a fresh MC confirmation.
- Test suite on `main` (b5d19d4, fresh clone, 2026-09-29): 280 passed, 7 failed — all 7 match documented
  KNOWN_ISSUES controller adversarial failures (retreat straightness x2, push 12 mm offset x2, lost-cube threshold
  x2, `test_push_lost_cube_flag`). This is exactly the B10 task scope.
- Three tasks were queued and not yet run: A07 (judge gaps: rotations-with-cube metric, causal-contact
  participation credit, velocity-based contact rejection), B10 (controllers under wheel asymmetry — the 7 failing
  tests above), C06 (nav collision-check perf, search must stay bit-identical). Launched all three lanes
  (worktrees `../V1-codex-a/b/c`, branches `codex-auto`/`codex-auto-b`/`codex-auto-c`) 2026-09-29; see
  `docs/codex_log/STATUS_<lane>` in each worktree and `tools/codex_auto/done/` once reviewed/merged.

## Immediate next steps (priority order)
1. Review A07/B10/C06 commits (`git log main..codex-auto[-b|-c]`), cherry-pick what passes review.
2. Re-baseline MC on the merged simulator (strict and 20 mm), 12 -> 50 seeds — only meaningful once A07 (judge
   fixes) lands; numbers before that are not trustworthy (KNOWN_ISSUES).
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
