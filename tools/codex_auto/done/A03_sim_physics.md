Fix the PHYSICS/dynamics findings of the Gate 8 audit (docs/astra/gate8_report.md) in
rover_strategy/simulation/physics.py: contact model issues (contact point/normal, rotation sign, tunnelling at speed,
clamps hiding penetration, cube-cube chains), rover dynamics too kind (acceleration limits, slip/speed loss under
pushing load). Pushing load: the team measured ~2 cm less travel over a ~50 cm run when pushing a cube (~4 % speed
loss, USER-REPORTED, add as a parameter with that default). Acceptance: related tests in tests/test_simulation_astra.py
pass; tests/test_simulation.py passes (update a test only if it encoded the old wrong physics — justify);
full suite not worse; simulation speed not worse than 30x real time for 2 rovers + 3 cubes (measure and report).
