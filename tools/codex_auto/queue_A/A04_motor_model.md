Replace the random motor asymmetry of the simulator with the MEASURED per-rover characterisation in docs/MEASUREMENTS.md
("Motors" section). Files: rover_strategy/simulation/physics.py (MotorParams / wheel model), rover_strategy/simulation/
scenarios.py. Implement: a per-wheel saturating throttle->speed curve (piecewise-linear through the measured normalised
points: distance at 30/50/70/100 % relative to the 100 % value; the absolute speed scale stays the existing sim max
speed because the experiment duration is UNKNOWN — document that), R10 nearly balanced (L/R ratios from the table),
R11 right wheel faster (L/R 0.907/0.926/0.931 at 50/70/100 %), run-to-run variation from the R11 repeated
measurements (e.g. 50 %: 48/44/44 vs 52/48/50 cm), cube-pushing speed loss (~4 %). Rover id 10 and 11 must get their own
model in every family; keep randomisation around the measured values (hard family: wider). Commands remain mm/s wheel
speeds; the simulator maps them to throttle through the inverse of the NOMINAL (not the true) curve so the strategy
sees realistic unmodelled asymmetry. Add tests. Acceptance: new tests pass; full suite not worse.
