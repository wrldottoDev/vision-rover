# HANDOFF — what the next session needs to know (update after every meaningful change)

_Last updated: 2026-09-28 by Claude Opus 5.5 (lead/final auditor)._

## Where things stand
- Python reference system complete as modules and unit-tested (see PROJECT_CONTEXT §5). **Closed-loop mission has
  not yet delivered a cube end-to-end in simulation.** The last session was debugging the closed loop; the chain
  navigate -> align -> capture -> push leg now works for a single rover (`tools/single_rover.py 430 450 0`).
- Monte Carlo harness exists but results are not meaningful yet (judge defects, Gate 8).
- Repo is on GitHub (`origin/main`). Work style: Opus reviews; GPT-Luna workers (codex CLI) implement on
  `codex-auto*` branches via `tools/codex_auto/` (see its README). Their commits are UNREVIEWED until merged to main.

## Immediate next steps (priority order)
1. Single rover end-to-end delivery: run `tools/single_rover.py` on a few cube positions, fix each failure at its
   root (align timeout, interrupted capture, replanning after the last leg).
2. Adopt the real telemetry reality: DEGRADED mode (drive slowly, no capture start) with ages up to ~1.5 s, LOST
   beyond; per the team's p95 470 ms. Update `TelemetryPolicy` + FSM + simulator latency distribution.
3. Apply measured motor characterisation per rover (R11 right faster, ratio ~0.92; saturating throttle curve) in the
   simulator motor model and as feed-forward in the command mapping; keep 0.92 marked NOT FINAL.
4. Marker offset (~24 mm forward, USER-REPORTED): confirm whether the deployed vision compensates it; then set
   `config.rover.marker_offset_fwd`.
5. Two-rover start: rovers start < 45 mm apart; reduce WAIT/yield churn (explicit start sequencing: rover with a free
   exit goes first).
6. Fix simulator judge (Gate 8 findings) before quoting Monte Carlo numbers.
7. Navigation speed (profile: `_fast_hull`/`_fast_segment_collides`).
8. Then Monte Carlo 200 -> 1000 -> 10000 on `official_like`, failure-driven fixes, then optimisation.

## Open questions for the team / organisers (cannot be resolved from the repo)
- Official depot size and delivery criterion; depot positions vs corner markers (marker freeze risk).
- Does leaving the 860 mm effective field (but staying on the 1000 mm board) count as "salida"?
- Protocol v2 schema; real latency logs; whether marker offset is compensated by the deployed vision.
- Duration of the motor-distance experiments (to convert to mm/s).

## Gotchas
- `tools/*` need `PYTHONPATH=.`. Debug runs print events only at the end.
- Nav planner is slow; a 240 s simulated run can take several minutes.
- Supervisor + FSM are the lead's files; worker preamble forbids workers from editing them.
