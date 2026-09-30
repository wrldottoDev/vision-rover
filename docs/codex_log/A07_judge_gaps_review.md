Reviewed the uncommitted diff against `b5d19d4`.

- HIGH — `rover_strategy/simulation/physics.py:829-830` (motion noise at `575-579`): rotation metrics use raw `wl/wr`, while pose integration uses noisy effective speeds. With default noise, seed 1 and commands `(-8.85, 8.85)` produce actual `omega=0.2024 rad/s` but no event; other seeds produce false events below `0.2`.

- HIGH — `rover_strategy/simulation/physics.py:691,785-793`: causal-credit and contact gating use raw wheel averages rather than realized forward velocity. Reproduced with seed 5: effective push speed `4.936 mm/s` (<5), raw speed `5.1 mm/s`, yet transport credit is awarded. This can also misclassify contact direction near zero.

Tests: `282 passed, 7 failed, 1 skipped`; the 7 failures are pre-existing. Simulation benchmark: `90.8x` real time. No seed-specific production logic or deleted tests found, but the new tests all use zero noise and miss these defects.

VERDICT: REJECT