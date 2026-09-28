# vision-rover

Strategy software for the **Vision Rover Challenge**: two differential-drive rovers push three coloured cubes into
colour-matched corner depots using an overhead camera's telemetry. This repository contains the Python reference
implementation (planner, estimator, controllers, rover FSM, two-rover coordination) and a closed-loop simulator with
Monte Carlo tooling.

**Maturity: research prototype. SIMULATION ONLY, NOT PHYSICALLY VALIDATED. Full closed-loop missions do not yet
succeed in simulation** — see `docs/KNOWN_ISSUES.md` and `docs/HANDOFF.md`.

## Layout
```
rover_strategy/
  config.py            all physical constants and tuning, with provenance tags
  frames.py world.py   units/frames; shared data model
  vision/              telemetry parser + TCP client (official protocol v1)
  estimation/          EKF pose estimator, cube tracker
  geometry/            polygons/SAT, rover footprint, depot zones
  planning/            push/delivery planner, Hybrid-A* navigation, task allocator, reservations
  control/             PID, segment follower, align/capture/push/retreat controllers
  rover/fsm.py         per-rover mission FSM (future code.py logic)
  coordination/        supervisor (future plan.py role), ESP-NOW status protocol
  simulation/          physics, vision emulator, scenarios, closed-loop runner, Monte Carlo
tests/                 unit + adversarial tests
tools/                 debug/visualisation tools, codex worker automation
docs/                  project memory (start with PROJECT_CONTEXT.md, HANDOFF.md)
_source/               official challenge repo + prior research (reference evidence)
```
Not yet present: `plan.py`, `rover10/code.py`, `rover11/code.py`, C drivers (NOT IMPLEMENTED; Phase 9).

## Setup and use
```
python3 -m venv .venv && .venv/bin/pip install numpy pytest matplotlib
.venv/bin/python -m pytest -q -p no:cacheprovider tests/
PYTHONPATH=. .venv/bin/python tools/single_rover.py 430 450 0
PYTHONPATH=. .venv/bin/python -m rover_strategy.simulation.monte_carlo --n 50 --family official_like --workers 8
```
Live telemetry client: `rover_strategy/vision/client.py` (connects to the official vision or its mock publisher on
port 2026). No rover deployment path exists yet.

## Safety / competition constraints
Software-only changes; no human input after start; move only in phase RUNNING; stay on the field; both robots must
participate. Details: `docs/RULES_AND_CONSTRAINTS.md`.
