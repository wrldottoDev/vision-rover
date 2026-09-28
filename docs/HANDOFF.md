# HANDOFF — what the next session needs to know (update after every meaningful change)

_Last updated: 2026-09-28 by Claude Opus 5.5 (lead/final auditor)._

## Where things stand (2026-09-28 ~03:40)
- Python reference system complete as modules; closed loop now reaches real milestones in simulation
  (SIMULATION ONLY): two rovers navigate from a tight start, capture (verified depth 26-37 mm, lateral < 4 mm),
  push multi-leg plans; seed 0 (`official_like`) delivered a cube into its depot (truth) at ~95 s. Full 3-cube
  missions not yet completed.
- Current blockers: (1) no arc motions -> edge-parallel pre-push poses unreachable (last leg into corner depots,
  cubes pushed next to an edge); (2) delivery confirmation depends on the orientation belief (fixed today: confirm
  after retreat + depth/push-length orientation inference, being re-tested); (3) align timeouts seen in two-rover runs.
- Codex/GPT-Luna lanes (`tools/codex_auto/`, worktrees ../V1-codex-{a,b,c}):
  A: simulator judge/sensors/physics/motor model/MC triage (A01, A02 done, UNREVIEWED except A01 notes);
  B: B03 merged; B01 rejected; now B05 wheel-bias redo, B06 allocator time model;
  C: C01 rejected; now C02 ARC segments end to end (world.SegKind.ARC added by lead), C03 nav benchmark/perf.
  Reviews: `docs/reviews/codex_reviews.md`. Gate status: `docs/reviews/GATES.md`.

## Immediate next steps (priority order)
1. Review/merge lane C (arcs) -> rerun `tools/single_rover.py` and seed 0; expect corner deliveries to become
   reachable; then re-enable `PushParams.require_straight_approach` semantics adapted to arcs if needed.
2. Review lane A (A01 needs fixes: participation credit, planner_reason classification, irreversible cube exit),
   merge A02-A04, then first meaningful Monte Carlo (official_like, 50 -> 200 seeds).
3. Align timeouts in two-rover runs (investigate with tools/debug_run.py 0 official_like + STALL=400).
4. Marker offset (~24 mm, USER-REPORTED) and v2 protocol: need team confirmation.

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
