# Rules and constraints (authoritative interpretation after Astra Gate 1)

Paths: `O/` = official repo (`_source/Rover Vision Artificial/Vision-Rover-Challenge-main_unz/Vision-Rover-Challenge-main/`).

## Hard rules (official)
| # | Rule | Source | How the software honours it |
|---|---|---|---|
| H1 | Software-only changes; no hardware/electronics changes; official vision may not be modified | `O/reglamento.md:13-24`, `O/robot.md:95` | We only consume telemetry v1; nothing depends on vision internals |
| H2 | Fully autonomous after start: no manual commands, no touching robots/cubes, no code changes, no selective restarts | `O/reglamento.md:99-115` | Supervisor runs closed loop; all recovery is automatic |
| H3 | Move only in phase `RUNNING`; stop immediately on `FINISHED` (and in IDLE/READY) | `O/vision-system/contrato/CONTRATO.md` s5 | Supervisor gates every command on phase; STOP otherwise |
| H4 | Penalty for leaving the competition surface | `O/reglamento.md:152-158` | Whole rover envelope (incl. paddles) kept inside the 860x860 effective field minus `margins.board` (strict reading; `board.overhang_allowance_mm = 0`) |
| H5 | Both robots must participate; success = both robots transport and deposit assigned objects | `O/reglamento.md:170-176`, `O/el_reto.md` success section | Allocator hard constraint `require_both_rovers` when both are healthy and >= 2 cubes remain |
| H6 | Robots start from the established start zone | `O/reglamento.md:125` | Scenario family `start_zone`; planner itself assumes nothing about start poses |
| H7 | Scoring: mainly time and number of correctly delivered objects; also placement precision, no collisions, recovery | `O/reglamento.md:176-180`, `O/el_reto.md` | Objective = makespan with robustness penalties; safety constraints are hard |
| H8 | 2 or 3 cubes, unique colours; each goes to the depot of its colour; depot colours/positions read from telemetry | `CONTRATO.md` s3 | Everything keyed by colour/id; never by list index |
| H9 | `obstacles` empty this edition but must be iterated | `CONTRATO.md` s3 | Parser converts them; planner treats them as obstacles if ever present |

## Telemetry semantics that constrain the design
- TCP NDJSON port 2026, ~20 Hz, last-value-wins (seq gaps normal). `ts_ms` = CAPTURE time. `age_ms` = time since last real observation of that object.
- **Freeze hazard (Astra CRITICAL):** if two corner markers are occluded the official vision keeps publishing with rising `seq` but frozen `ts_ms` / ages. Freshness is therefore judged ONLY from capture time (`ts_ms - age_ms`), duplicate capture times are ignored, and rovers stop when data is stale (`TelemetryPolicy`).
- Cube orientation is NOT published (v1). Cube position = centre of the fitted base square; unreliable detections do not refresh (last position kept, age grows). With ~70 % occlusion fitting errors reach ~34 mm, so stale/occluded cube positions get growing uncertainty.
- Published rover pose = marker pose corrected by the vision's configured marker->rotation-centre offsets (currently 0, position offset NOT measured) and marker-height parallax. `config.rover.marker_offset_*` is only a residual correction (default 0) to avoid double compensation.

## Unknowns and the defaults chosen (each is a config knob)
| Unknown | Default | Risk | Knob |
|---|---|---|---|
| Depot size/shape and delivery criterion (no official definition; positions marked PROVISIONAL) | 100 mm square centred on the published point; delivered = whole cube footprint inside | Could be larger (easier) or a point tolerance | `depot.half_size`, `depot.delivery_margin` |
| Corner marker vs depot overlap | Depot point is the inner corner of the 100 mm corner marker; a delivered cube overlaps the marker (>= 10x10 mm black even at the best spot). Two occluded markers freeze vision. | Could make the 3rd delivery impossible on the current provisional layout | delivery-point cost; flagged in plan notes |
| Rover rotation centre vs chassis centre | coincident | footprint error of a few mm | `rover.chassis_center_x` + `margins.pose_uncertainty` |
| Throttle -> mm/s calibration | 180 mm/s at throttle 1 (sim) | speeds/timeouts off | `limits.*`; must be measured per wheel, fwd/rev, under load |
| Start zone size / arrangement | official sim poses (4,4) and (4,8) cells, 45 deg | none for the planner | scenario family |
| Round time limit | none known | — | sim run limit 300 s |

## Firmware conventions (for later embedded work)
- `motor_1` = left, `motor_2` = right; throttle in [-1, 1], positive = forward (`O/codigos/test_motores.py`).
- `O/codigos/turn_angle.py` treats POSITIVE angles as CLOCKWISE — opposite to telemetry/our CCW convention. Our command interface is wheel speeds with omega = (v_right - v_left) / track, never that helper's angle sign.
- A motor-command watchdog on the rover (stop if no fresh command within ~300 ms) is required for real hardware.
