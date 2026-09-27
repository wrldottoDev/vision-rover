# Common rules for all implementation workers

Project root: /Users/ottogonzalez/Documents/rover/V1   (Python package `rover_strategy`, tests in `tests/`)
Python: ALWAYS use `/Users/ottogonzalez/Documents/rover/V1/.venv/bin/python` (numpy + pytest installed; NO other deps allowed).
Run tests: `cd /Users/ottogonzalez/Documents/rover/V1 && .venv/bin/python -m pytest -q tests/<yourfile>`.

Read FIRST (the frozen shared foundation — do NOT edit these files):
- rover_strategy/config.py   (all constants, units mm/rad/s, `DEFAULT` config; see prepush_distance, contact_distance)
- rover_strategy/world.py    (Pose, RoverEstimate, CubeEstimate, WorldState, Frame, WheelCommand, Segment, Path, PushLeg, PushPlan)
- rover_strategy/frames.py   (wrap, angle_diff, Grid conversions)
- rover_strategy/geometry/{shapes.py,footprint.py,zones.py}  (SAT overlap, distance, Footprint.envelope/parts/straight_sweep, DepotZone)
- docs/interpretation_v0.md  (physical model + rules interpretation)

Conventions: internal frame x right, y UP, theta CCW from +x, radians. Rover pose = rotation centre (axle midpoint).
Rover frame: +x forward, +y left. Footprint envelope x in [-47, 102], |y| <= 49.75 (paddles = the front 55 mm).
Cube side 60 mm; orientation alpha is modulo 90 deg; unknown => None (worst case half-diagonal 42.43).

Rules:
- Only create/edit the files assigned to you (plus your own test file(s)). Never touch other modules; never run git.
- If you need a new tunable constant, define it in a small frozen dataclass in YOUR module (e.g. `NavParams`) with a
  provenance comment (MEASURED/ASSUMED/TUNED). List every such constant in your final report so the lead can
  centralise it. Never hard-code board coordinates or special-case layouts.
- Typed Python, dataclasses, no global mutable state, deterministic (seeded numpy Generator passed in).
- Write focused pytest tests that would FAIL if the logic were wrong (not smoke tests). Aim for edge cases.
- Keep code lean: no speculative abstractions. Clear docstrings on math (units!).
- Final report (<= 60 lines): files created, public API signatures, design decisions + why, tests and results,
  known limitations, constants to centralise, anything in the foundation you believe is WRONG.
