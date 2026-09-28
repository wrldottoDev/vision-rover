# Testing

Run all: `.venv/bin/python -m pytest -q -p no:cacheprovider tests/` (~1 min).

| File | Scope |
|---|---|
| test_foundation.py, test_parser.py | frames, geometry primitives, zones, telemetry parsing |
| test_estimation.py, test_estimation_astra.py | EKF (noise, dropouts, latency replay, wrap, outliers, kidnap, frozen feed, stuck wheel), cube tracker |
| test_navigation.py, test_navigation_astra.py | planner correctness, dense swept-area checks, superset reservations, deadlines, supervisor revalidation |
| test_push_planner.py | delivery geometry, multi-leg plans, unsolvable classification, independent geometric re-check fuzz |
| test_controllers.py, test_controllers_astra.py | PID, rotate/straight/align/capture/push/retreat under an independent wheel-level truth model |
| test_allocation.py, test_reservations.py, test_protocol.py | allocator enumeration/makespan/constraints, reservations, ESP-NOW frame |
| test_simulation.py, test_simulation_astra.py | physics contact, sensor semantics/schema, scenarios, runner judging |

`*_astra.py` files were written by an adversarial reviewer (GPT Astra). A failing astra test is a finding, not noise;
change an astra test only when it is provably wrong, and record why in the test and in DECISIONS/KNOWN_ISSUES.
Integration evidence comes from the closed-loop tools in `docs/SIMULATION.md`.
