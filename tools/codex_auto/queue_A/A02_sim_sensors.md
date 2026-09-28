Fix the SENSOR-model findings of the Gate 8 audit (docs/astra/gate8_report.md) in rover_strategy/simulation/sensors.py
(and scenarios.py if needed for parameters). Must implement: (1) corner-marker occlusion + official freeze behaviour:
when >= 2 of the 4 corner-marker regions (100 mm black + 20 mm white, centred on field corners; in-field quarter) are
significantly covered by a cube or rover (from above, with parallax), the published messages keep advancing `seq` but
repeat the last good state with frozen ts_ms/age; with exactly 1 covered, continue normally (vision keeps a saved
homography); make the coverage threshold a documented parameter; (2) separate capture and publication clocks as the
official engine does; (3) any other sensor findings in the report. Also add an optional USER-REPORTED latency profile
("realistic_latency": age p95 ~470 ms, max ~1420 ms, heavy tail) selectable per scenario family (new family
"official_like_slow" in scenarios.py) — do NOT make it the default. Acceptance: related tests in
tests/test_simulation_astra.py pass; tests/test_simulation.py passes; full suite not worse.
