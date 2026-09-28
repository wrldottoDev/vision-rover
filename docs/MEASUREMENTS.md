# Measurements and parameter provenance

Status tags: **MEASURED** (physical measurement or CAD with declared units), **DERIVED** (computed from measured
values), **ESTIMATED** (from experiments with known gaps), **ASSUMED** (defensible default, must be confirmed),
**USER-REPORTED** (given by the team in conversation, no file evidence in the repo yet), **UNVERIFIED**.
Authoritative values live in `rover_strategy/config.py`. Full source inventory: `docs/repository_findings.md`.

## Field
| Quantity | Value | Status | Source |
|---|---|---|---|
| Effective field | 43 x 43 cells of 20 mm = 860 x 860 mm | MEASURED | `vision/config_vision.json`, `contrato/config_simulador.json` (read from telemetry `grid` at runtime) |
| Physical board | 1000 x 1000 mm, 70 mm unused margin per side, no walls | MEASURED | fabrication files, MONTAJE.md |
| Corner ArUco | 100 mm black + 20 mm white, centred on field corners | MEASURED | config_vision.json |
| Depot points | 3 non-start corners, 2.5 cells in from both edges | MEASURED (positions marked PROVISIONAL by organisers) | config_simulador.json:59 |
| Depot size / delivery rule | 100 mm square, whole cube inside | **ASSUMED** — no official definition exists | RULES_AND_CONSTRAINTS.md |
| Round duration | ~600 s | USER-REPORTED | — |

## Cube
| Quantity | Value | Status |
|---|---|---|
| Side | 60 mm (3 cells) | MEASURED (cubos.dxf, config_vision.json) |
| Projected width | 60(|cos a|+|sin a|) in [60, 84.85] mm | DERIVED |

## Rover (both units nominal; R10 != R11)
| Quantity | Value | cells | Status | Source |
|---|---|---|---|---|
| Chassis length | 94.0 mm | 4.70 | MEASURED (DXF) | `_source/.../05_Mediciones_CAD/dimensiones_rover.md` |
| Outer width | 99.5 mm | 4.98 | MEASURED (DXF + ruler 100) | same |
| Channel (paddle inner faces) | 93.5 mm (ruler: 94.05 +- 0.54) | 4.68 | MEASURED | same |
| Paddle reach ahead of front plate | 55 mm | 2.75 | MEASURED (ruler, +-1 mm) | same, M6 |
| Total length incl. paddles | 149-150 mm | 7.45 | MEASURED | same, M8 |
| Wheel diameter / track | 33 / 89 mm | | MEASURED (ruler) | same, M13 |
| Axle (rotation centre) vs chassis centre | coincident | | **ASSUMED** | not measured |
| Circumscribed radius about rotation centre | 113.49 mm | 5.67 | DERIVED (depends on the axle assumption) | config.RoverGeometry.sweep_radius |
| Centre-to-cube-centre at flush contact | 77.0 mm | 3.85 | DERIVED (47 + 30) | config.contact_distance |
| Centre-to-cube-centre, 45 deg cube touching plate | 89.4 mm | 4.47 | DERIVED (47 + 42.43); the "4.47 cells" figure from prior work |
| Prior research pre-push distance | 129.43 mm | 6.47 | DERIVED in prior work (s = 40 mm) — NOT used |
| Current pre-push distance | 173.9 mm | 8.70 | DERIVED: sweep radius + half diagonal + 12 + 6 (full rotation clearance) | config.prepush_distance |
| ArUco marker -> reference point | marker ~24 mm FORWARD of the reference point | 1.2 | **USER-REPORTED**; the official vision config in the repo has offset 0 and both measurement attempts were rejected (INSUFICIENTE) | config.RoverGeometry.marker_offset_* |
| Marker size / height | 40 mm / 90 mm | | PROVISIONAL / MEASURED | config_vision.json |

Open question: does the deployed vision already compensate the marker offset? If yes, set `marker_offset_fwd=0`
to avoid double compensation (`frames.marker_to_body`).

## Vision / telemetry
| Quantity | Value | Status |
|---|---|---|
| Transport | TCP 2026, NDJSON, ~20 Hz, last-value-wins | MEASURED (CONTRATO.md) |
| Protocol version | repo contract = **v1**; team reports **v2** | v2 = USER-REPORTED, schema not in repo -> parser accepts v1 only (NOT IMPLEMENTED for v2) |
| Mock publisher noise | 1.2 mm / 1.5 deg sigma | MEASURED (config_simulador.json) |
| Prior research cube sigma | 5 mm = 0.25 cells (worst case with ~22 % occlusion) | DERIVED in prior work |
| Real age p95 / max | 470 ms / 1420 ms | USER-REPORTED (no log in repo) |
| Fit error at ~70 % occlusion | up to 34 mm | MEASURED on synthetic generator (vision README) |

## Motors (real hardware, 2026-09 characterisation) — USER-REPORTED
Distance travelled per wheel over a fixed test interval. **The interval duration is UNKNOWN**, so values are not
converted to mm/s. Only ratios/shapes are used.

| Throttle | R10 L | R10 R | R11 L (mean) | R11 R (mean) | R11 L/R |
|---|---|---|---|---|---|
| 30 % | 39.6 cm | 40.0 cm | — | — | — |
| 50 % | 49.5 | 50.0 | 45.33 (48/44/44) | 50.0 (52/48/50) | 0.907 |
| 70 % | 54.0 | 54.2 | 50.0 | 54.0 | 0.926 |
| 100 % | 57.5 | 58.0 | 54.0 (x5) | 58.0 (x5) | 0.931 |

- R10 L/R ~ 0.99-1.00 (balanced). R11 right wheel ~7-9 % faster; feed-forward `RIGHT_CORRECTION ~ 0.92` under test,
  NOT final. An early R11 right 100 % = 30 cm reading is treated as anomalous (not repeated).
- Distance vs throttle saturates strongly (R10: 0.69 of full at 30 %, 0.86 at 50 %, 0.94 at 70 %): throttle is not
  proportional to speed. ESTIMATED shape only.
- Pushing a cube costs ~2 cm over the same interval (~4 %). USER-REPORTED, needs repetition.
- Needed next: fixed-duration runs (e.g. 2.00 s) per wheel, forward AND reverse, several throttles incl. near the
  deadband, with and without a cube; record duration explicitly.
