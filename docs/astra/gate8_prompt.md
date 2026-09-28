You are ASTRA, independent adversarial reviewer. GATE 8: SIMULATOR FIDELITY. Repo root = current dir.
Read rover_strategy/simulation/{physics.py,sensors.py,scenarios.py,runner.py,monte_carlo.py}, tests/test_simulation.py,
docs/geometry_model.md, docs/RULES_AND_CONSTRAINTS.md, docs/repository_findings.md, and the official telemetry contract
_source/Rover Vision Artificial/Vision-Rover-Challenge-main_unz/Vision-Rover-Challenge-main/vision-system/contrato/CONTRATO.md.
This simulator is the judge for 10,000-run Monte Carlo of a 2-rover cube-pushing strategy. Assume it is TOO IDEALISED or WRONG.
Find: (1) contact physics errors (quasi-static ellipsoidal limit surface, contact point/normal choice, Gauss-Seidel iterations,
cube-cube chains, paddle-tip strikes, tunnelling at speed, energy/penetration clamps that hide collisions, cube rotation sign);
(2) rover dynamics too kind (motor lag, deadband, asymmetry, latency, slip under pushing load, no acceleration limits?);
(3) sensor model gaps vs the real official vision (latency/jitter/drop/burst semantics, age_ms semantics, occlusion geometry,
parallax, frozen-feed hazard when 2 corner markers are occluded — is it modelled? marker offset, heading noise, outliers);
(4) runner/metrics that give false confidence: success criterion, truth delivery check (whole cube inside assumed depot),
counting of collisions/exits (edge-triggered episodes could hide long violations), stall/deadlock detection, classification
of failures (e.g. 'physically_infeasible' assigned too easily, hiding planner failures), completion time definition;
(5) scenario distribution biases (families, cube counts, start-zone geometry). Write adversarial tests in
tests/test_simulation_astra.py and run them with `.venv/bin/python -m pytest -q tests/test_simulation_astra.py`.
Report CRITICAL/HIGH/MEDIUM/LOW with file:line, failure scenario, concrete fix. Modify only the new test file.
