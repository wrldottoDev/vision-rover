# CLAUDE.md — start here

Vision Rover Challenge: two differential-drive rovers (ArUco ids 10, 11) push three cubes (red/green/blue) into
colour-matched corner depots, guided by an overhead camera (official telemetry, TCP 2026, NDJSON). Python reference
implementation + closed-loop simulator. Hardware integration has NOT started.

## Session startup (mandatory)
1. Read `docs/PROJECT_CONTEXT.md`, then `docs/HANDOFF.md`, then `docs/DECISIONS.md`.
2. `git log --oneline -n 20`, `git status`.
3. Run tests before touching architecture: `.venv/bin/python -m pytest -q -p no:cacheprovider tests/`
   (create the venv with `python3 -m venv .venv && .venv/bin/pip install numpy pytest matplotlib`).
4. Known failing tests are listed in `docs/KNOWN_ISSUES.md` — do not "fix" them by weakening assertions.

## Rules for every session
- Never invent measurements. Every constant lives in `rover_strategy/config.py` with a status tag
  (MEASURED / DERIVED / ESTIMATED / ASSUMED / USER-REPORTED / UNVERIFIED). See `docs/MEASUREMENTS.md`.
- Never hardcode a competition layout, seed or coordinate. Fix root causes.
- Separation of responsibilities (target architecture): `plan.py` = global planner; `roverNN/code.py` = rover
  executor (FSM, control, local estimation); C drivers = motors/IMU/IR/sonar/ESP-NOW. Today all of it runs as the
  `rover_strategy` package on the PC (see `docs/ARCHITECTURE.md`).
- R10 and R11 are physically different robots (R11 right wheel faster). Keep per-rover parameters.
- Simulation is evidence, never proof of physical success. Label results SIMULATION ONLY.
- Units: mm, rad, s; internal frame y-UP (official telemetry is cells, row-down, degrees).
- Update `docs/HANDOFF.md` after meaningful work; commit with meaningful messages; push stable checkpoints to
  `origin` (git@github.com:wrldottoDev/vision-rover.git). Never force-push. Never commit secrets.

## Working conventions
- Branch `main` = integrated, reviewed work. `codex-auto*` branches = unreviewed GPT-Luna worker output
  (see `tools/codex_auto/README.md`); cherry-pick only after review.
- Worker instructions: `docs/worker_common.md`. Review reports: `docs/reviews/`, `docs/astra/`, `docs/codex_log/`.
- Debug tools: `tools/debug_run.py SEED FAMILY [T]`, `tools/snapshot.py`, `tools/single_rover.py`, `tools/trace_single.py`.
