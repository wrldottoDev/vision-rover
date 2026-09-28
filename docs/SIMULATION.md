# Simulation and Monte Carlo

## Components (`rover_strategy/simulation/`)
- `physics.py` — ground truth at dt = 10 ms. Rovers: wheel speed commands with latency, first-order lag, deadband,
  per-wheel gain, speed noise. Contact: rover parts (body + two thin paddles; open channel) vs cubes resolved by a
  quasi-static ellipsoidal limit-surface law (translation + rotation), 4 Gauss-Seidel iterations; cube-cube chains;
  rover-rover overlap = collision (step rejected); counters `collision, rover_exit, rover_fell, cube_exit,
  non_target_contact` (edge-triggered episodes).
- `sensors.py` — VisionEmulator producing official v1 messages: latency + jitter, dropped frames, per-rover dropout
  bursts, marker offset, Gaussian + heavy-tailed noise, cube occlusion from the rover BODY (channel open from above),
  partial-occlusion bias. Corner-marker freeze: NOT MODELLED (Gate 8).
- `scenarios.py` — seeded families: `start_zone`, `arbitrary`, `hard`, `official_like` (rovers in the start corner,
  3 cubes in the interior band [160, 700] mm like the official example layout).
- `runner.py` — closed loop: physics -> emulator -> parser -> Supervisor -> commands; truth-judged outcome and failure
  classes; `monte_carlo.py` — parallel seeds, aggregate metrics, failed seeds saved.

## How to run
```
PYTHONPATH=. .venv/bin/python -m rover_strategy.simulation.monte_carlo --n 200 --family official_like --workers 8 --out runs/mc
PYTHONPATH=. .venv/bin/python tools/debug_run.py SEED FAMILY [MAX_T]          # event log of one seed
PYTHONPATH=. .venv/bin/python tools/snapshot.py SEED FAMILY 5,10,20             # PNG snapshots in runs/
PYTHONPATH=. .venv/bin/python tools/single_rover.py X Y [ALPHA_DEG]              # one rover, one cube
PYTHONPATH=. .venv/bin/python tools/solvability_map.py 40                         # push-plan coverage map
```

## Fidelity status
Astra Gate 8 = FAIL (`docs/astra/gate8_report.md`). Do not quote Monte Carlo rates until the judge fixes land.
Motor model should adopt the measured R10/R11 asymmetry (MEASUREMENTS.md) — NOT DONE.
Everything here is SIMULATION ONLY.
