# HANDOFF — what the next session needs to know (update after every meaningful change)

_Last updated: 2026-09-28 by Claude Opus 5.5 (lead/final auditor)._

## Where things stand (2026-09-28 ~07:35)
- Closed loop works end to end in simulation (SIMULATION ONLY). Monte Carlo, official_like seeds 100-111, 600 s:
  strict field rule 9/36 cubes, 0 missions 3/3, 0 collisions/contacts/exits; with 20 mm allowed overhang 18/36 cubes,
  1 full 3/3 mission in 104.8 s, 0 collisions/contacts, 1 board exit (seed 107, under analysis B09).
- Merged today from GPT-Luna lanes: arcs (C02), telemetry adapters (B03), push-planner coverage/blockers (B07+B08),
  simulator fidelity (A01-A05). Simulator changed materially -> re-baseline MC after A06.
- Codex lanes running: A06 (sim review fixes), B09 (board-exit analysis, report only), C05 (nav perf w/o
  completeness loss). Quota not exhausted as of 07:31.

## Immediate next steps (priority order)
1. Re-baseline MC on the merged simulator (strict and 20 mm), 12 -> 50 seeds.
2. Review B09 report; fix the board exit at its root (safety first).
3. Remaining failure classes: planner_no_solution (strict geometry + clutter: use last_blockers for sequencing),
   corner near-misses (few mm; rules question), coordination liveness.
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
