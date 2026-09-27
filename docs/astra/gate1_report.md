codex
**Gate 1 + Gate 2: changes required.** F1’s coordinate transform is correct. The main blockers are depot assumptions, stale telemetry handling, and unverified pose/contact geometry. No files modified.

References below use:
- `O/` = `_source/Rover Vision Artificial/Vision-Rover-Challenge-main_unz/Vision-Rover-Challenge-main/`
- `V/` = `O/vision-system/vision/`
- `S/` = `_source/Rover Vision Artificial/`

## CRITICAL

**1. R4/R6: corner occlusion can freeze observations while messages still say `RUNNING`.**

For axis-aligned corner markers, R4’s arithmetic is correct. At `(43,43)`:

- Black marker: **100 mm**, spanning `[40.5,45.5]²` cells.
- Including the **20 mm white border**: `[39.5,46.5]²`.
- Depot example `(40.5,40.5)` is the black marker’s inward corner.
- An aligned 60 mm cube centred there spans `[39,42]²`, overlapping **30 × 30 mm** of black marker, plus its white surround.

Thus successive deliveries can obscure multiple anchors. Exact corner coincidence assumes marker alignment and the provisional depot coordinates.

With **one marker missing**, vision requires a previously established four-marker homography and checks that the three visible centres deviate by at most **2 mm**. It can retain that geometry indefinitely. With **two missing**, geometry raises an error. Publication continues from the old state: `seq` increases, but `ts_ms`, object ages and possibly `phase` remain unchanged.

Evidence: `V/config_vision.json:14`, `V/geometry/coordenadas.py:218`, `V/sistema.py:284`, `V/publish/telemetria.py:136`.

**Fix:** require fresh capture timestamps and acceptable object ages in addition to `RUNNING`; use a motor-command watchdog. Effective observation age is approximately `now − ts_ms + age_ms`, with clock alignment. Validate depot placement against marker visibility before treating multi-cube delivery as feasible.

## HIGH

**2. R3: the 100 mm depot square is unsupported, and even the example positions are provisional.**

No geometric depot boundary, size, shape or numerical delivery tolerance was found in the official sources. There **is** a general success criterion—correctly depositing assigned objects—but no test that implements “correctly.”

Vision declares depot points in configuration and copies them into telemetry. It neither detects depot regions nor evaluates delivery. `config_simulador.json:59` explicitly marks positions and colours **unconfirmed against the physical setup**.

The research simulator does define delivery: **cube-centre distance ≤ 1 cell = 20 mm**, independent of orientation. That is a research convention, not an official rule.

Evidence: `O/reglamento.md:170`, `O/el_reto.md:135`, `O/vision-system/contrato/config_simulador.json:59`, `V/mundo.py:135`, `S/04_Banco_Pruebas/simulador.py:387`.

**Fix:** default to the published point as the placement target, with a configurable engineering stopping tolerance. Keep official delivery status **unverified** until the actual acceptance criterion is supplied. Retain the 100 mm/full-containment model only as an explicitly experimental scenario.

**3. G4 is semantically wrong; G2/G6/G7 lack a verified common reference point.**

Published rover pose means **rotation centre and paddle-forward heading**, after marker-offset correction. It equals marker pose only because the current offsets are zero. Position is also corrected for marker-height parallax.

The two rejected position-offset measurements do not establish zero offset. Angular alignment near zero was measured for **rover 10 with that particular marker mounting**, not both robots. Separately, the chassis-centre-to-axle offset remains unknown.

Evidence: `V/detectors/rovers.py:128`, `:190`, `:272`; `V/config_vision.json:130`, `:139`, `:142`. Both `V/mediciones/desfases_rover10_20260809_*.json:2` report `INSUFICIENTE`.

**Fix:** distinguish marker centre, axle midpoint and chassis centre. Measure their transforms for each rover; express collision geometry about the actual controlled point. Avoid applying an offset twice if vision already compensates it. Until measured, the **113.49 mm** sweep radius is conditional.

**4. C1/C4: a push does not establish cube orientation.**

The channel is wider than the cube’s maximum projection: **93.5 > 84.85 mm**. It permits every cube orientation. The research explicitly warns that a cube can rotate during transport. Actual verified face-to-face contact would constrain orientation, but telemetry does not establish that contact.

The detector internally estimates cube angle modulo 90°, then tracking discards it. There is **no hidden orientation field in official v1 telemetry**. Modifying official vision to expose it conflicts with `robot.md`.

Evidence: `S/03_Informe_Resultados/fuente/informe.tex:1335`; `V/detectors/cubos.py:230`, `:327`; `V/tracking/seguimiento.py:118`; `O/vision-system/contrato/schema.py:98`; `O/robot.md:95`.

**Fix:** retain an orientation belief unless a validated contact manoeuvre constrains it. Erode the assumed depot by the worst cube projection over that belief. Do not set cube angle equal to commanded push heading automatically.

**5. R8/F1: the supplied turn helper uses the opposite angle sign.**

Wheel mapping is correct: motor 1 left, motor 2 right. However, `turn_angle.py` converts a **positive** requested angle into left-forward/right-reverse: **clockwise**. Telemetry and F1 use positive **counterclockwise** angles.

Evidence: `O/codigos/wifi_command_receiver.py:56`; `O/codigos/turn_angle.py:128`, `:181`; `O/codigos/test_motores.py:27`.

**Fix:** negate CCW angle requests when using that helper, or implement the canonical differential-drive convention:
`ω = (v_right − v_left) / track`.

## MEDIUM

**6. G8: 5 mm is neither a measured Gaussian σ nor a universal occlusion bound.**

The mock’s **0.06-cell positional σ = 1.2 mm** and **1.5° angular σ** are correct. The research’s 5 mm value is an adopted working allowance covering a reported **4.88 mm** synthetic error with roughly 22% occlusion.

Published cube position estimates the **base-square centre**, using a fitted base/top silhouette—not the coloured blob centroid. This removes the intended centroid/parallax bias; it does not prove statistically unbiased measurements. At approximately 70% occlusion, documented fitting errors reach **34 mm**.

Unreliable detections normally do not refresh position or age. Tracking retains the last accepted position, then removes it after **more than 60 seconds** without acceptance.

Evidence: `O/vision-system/contrato/config_simulador.json:61`; `S/01_Modelo_Matematico/fuente/modelo.tex:656`; `V/detectors/cubos.py:85`, `:327`; `V/detectors/README.md:113`, `:137`; `V/tracking/seguimiento.py:114`, `:149`; `V/config_vision.json:84`.

**Fix:** separate mock noise, observed fitting error, systematic bias and stale-position uncertainty. Grow uncertainty during occlusion; do not keep a fixed 5 mm envelope around a moving, unseen cube.

**7. C2/C3: the reachability percentage is stale, and impossibility claims exceed the evidence.**

The report revises **36.3% → 25.7%**; the stored N3 run gives **25.8946%**. More importantly, that calculation allows the **70 mm physical margin**, checks a chassis rectangle, and uses a **129.43 mm** staging distance. It does not implement strict R5 plus the asymmetric paddle footprint and G7’s 165.91 mm clearance.

An exactly edge-trapped cube may be unrecoverable under specified push primitives. “In a non-depot corner” is too imprecise to prove impossibility.

Evidence: `S/03_Informe_Resultados/fuente/informe.tex:1411`; `S/04_Banco_Pruebas/salidas/run_n3.txt:9`; `S/04_Banco_Pruebas/n3.py:12`, `:43`, `:56`, `:77`.

**Fix:** remove the inherited percentage; recompute under the chosen footprint, boundary and staging policy. State impossibility only for defined contact configurations and permitted manipulation primitives.

**8. R8: no measured throttle-to-speed calibration is provided.**

`motor_calibration.py` estimates relative motor compensation using gyro drift. It does not measure millimetres travelled. The mock’s **6 cells/s = 120 mm/s** is a simulator parameter, not a throttle calibration.

Evidence: `O/codigos/motor_calibration.py:3`, `:111`; `O/vision-system/contrato/config_simulador.json:34`; `O/codigos/ideaboard.py:39`.

**Fix:** measure each wheel’s forward/reverse speed versus throttle, including deadband and pushing load. Do not infer distance from wheel diameter without measured rotation.

**9. Missing strategy constraints and start-zone ambiguity.**

- Both official robots must participate; physical/electronic modifications are prohibited.
- Official vision cannot be modified.
- No manual commands, object repositioning, code changes or selective robot restarts during a run.
- Start from the assigned zone. `(0,0)` identifies its corner marker; configured `start=(2.5,2.5)` identifies the zone centre. No zone dimensions or two-rover starting arrangement are specified.
- Time and correctly delivered object count are principal criteria. No numerical round limit, penalty schedule or collision-point tariff was found.

Evidence: `O/reglamento.md:15`, `:99`, `:125`, `:152`, `:170`; `O/robot.md:95`; `V/config_vision.json:102`.

**Fix:** add these constraints explicitly. Read `start` and depot mappings from telemetry; do not infer a 100 mm starting square or invent a round duration.

## LOW

**10. G1/G2: nominal dimensions are supported, but precision and provenance need qualification.**

The assembled measurements used an uncalibrated ruler, approximately **±1 mm**. They support 55 mm paddle reach, roughly 100 × 150 mm envelope, 33 mm wheel diameter and 89 mm track. Paddle height is **0–14 mm**, with tips reaching the floor.

The exact 93.5/99.5/94 mm dimensions come from the documented *separate-pieces DXF*. That DXF is absent from the supplied extracted files; the older supplied fabrication drawing has different reported dimensions.

Evidence: `S/05_Mediciones_CAD/dimensiones_rover.md:29`, `:74`, `:144`, `:158`, `:201`; `S/05_Mediciones_CAD/medir_dxf.py:6`.

**Fix:** label the rectangular chassis/rail model nominal and verify it on both assembled robots. Preserve measurement uncertainty in collision margins.

**11. G7/C1: correct arithmetic, overstated wording.**

G7’s circumscribed-disc clearance is a **sufficient full-rotation clearance**, not a necessary distance for every approach. C1’s quoted ±20/±7.6 mm values assume **zero margin**.

Evidence: `S/04_Banco_Pruebas/modelo.py:161`; `S/05_Mediciones_CAD/recalcular.py:147`; `docs/interpretation_v0.md:47`, `:54`.

**Fix:** state the manoeuvre assumptions and distinguish rotation, approach and delivery margins.

## CONFIRMED claims

- **R1:** TCP/NDJSON, port 2026, protocol v1 and nominal 20 Hz publication. Publication rate does **not** guarantee fresh observations. Evidence: `O/vision-system/contrato/CONTRATO.md:25`; `V/config_vision.json:89`.
- **R2/R5:** current effective field is **860 × 860 mm**, versus **1000 × 1000 mm** physical board and 70 mm margins. Official mounting documentation calls those margins unused. Strict R5 is defensible; a nonzero overhang allowance is not established permission. No walls is supported by the research correction. Evidence: `O/vision-system/MONTAJE.md:127`; `S/03_Informe_Resultados/fuente/informe.tex:1258`.
- **R3/R7/R9:** three colour-matched depot points; two robots, configured IDs 10/11; 2–3 unique-colour cubes; obstacles empty this edition. Identify by ID/colour and handle absent entries. Evidence: `O/vision-system/contrato/CONTRATO.md:192`, `:215`, `:230`, `:262`; `O/vision-system/contrato/config_simulador.json:28`.
- **R6/R8:** movement only during `RUNNING`; central PC planning and automatic commands are permitted. Positive throttle means forward in the supplied examples; left turn is `motor_1<0`, `motor_2>0`. Evidence: `O/vision-system/contrato/CONTRATO.md:337`; `O/reglamento.md:73`; `O/codigos/test_motores.py:13`.
- **F1:** **do not negate telemetry θ.** The detector computes `atan2(-forward_row, forward_col)`. For `(col,row,θ)=(10,20,90°)`, internal pose is **`(200 mm,460 mm,π/2)`**. Advancing 20 mm changes telemetry row to 19 and internal y to 480 mm. Evidence: `V/detectors/rovers.py:176`; `O/vision-system/contrato/CONTRATO.md:299`.
- **G3/G4:** axle/chassis coincidence remains an assumption; position-offset attempts were rejected; approximately zero angular offset is supported for the measured rover only.
- **G5–G7/C1 arithmetic:** independently recalculated, with the reference-point and orientation qualifications above:

| Quantity | Verified value |
|---|---:|
| Cube projected width | 60–84.8528 mm |
| Capture lateral tolerance | 4.3236–16.7500 mm |
| Sweep radius about chassis centre | 113.4860 mm |
| Straight paddle clearance, margin 10 mm | 154.4264 mm |
| Full-rotation clearance, margin 10 mm | 165.9124 mm |
| Assumed depot centre tolerance, aligned, margin 0 | ±20 mm |
| Same, cube at 45°, margin 0 | ±7.5736 mm |
| Same, cube at 45°, margin 10 mm | **Empty feasible region** |

- **C4’s telemetry claim:** cube orientation is not transmitted. Any orientation inferred through interaction remains model-dependent.
tokens used
135,908
**Gate 1 + Gate 2: changes required.** F1’s coordinate transform is correct. The main blockers are depot assumptions, stale telemetry handling, and unverified pose/contact geometry. No files modified.

References below use:
- `O/` = `_source/Rover Vision Artificial/Vision-Rover-Challenge-main_unz/Vision-Rover-Challenge-main/`
- `V/` = `O/vision-system/vision/`
- `S/` = `_source/Rover Vision Artificial/`

## CRITICAL

**1. R4/R6: corner occlusion can freeze observations while messages still say `RUNNING`.**

For axis-aligned corner markers, R4’s arithmetic is correct. At `(43,43)`:

- Black marker: **100 mm**, spanning `[40.5,45.5]²` cells.
- Including the **20 mm white border**: `[39.5,46.5]²`.
- Depot example `(40.5,40.5)` is the black marker’s inward corner.
- An aligned 60 mm cube centred there spans `[39,42]²`, overlapping **30 × 30 mm** of black marker, plus its white surround.

Thus successive deliveries can obscure multiple anchors. Exact corner coincidence assumes marker alignment and the provisional depot coordinates.

With **one marker missing**, vision requires a previously established four-marker homography and checks that the three visible centres deviate by at most **2 mm**. It can retain that geometry indefinitely. With **two missing**, geometry raises an error. Publication continues from the old state: `seq` increases, but `ts_ms`, object ages and possibly `phase` remain unchanged.

Evidence: `V/config_vision.json:14`, `V/geometry/coordenadas.py:218`, `V/sistema.py:284`, `V/publish/telemetria.py:136`.

**Fix:** require fresh capture timestamps and acceptable object ages in addition to `RUNNING`; use a motor-command watchdog. Effective observation age is approximately `now − ts_ms + age_ms`, with clock alignment. Validate depot placement against marker visibility before treating multi-cube delivery as feasible.

## HIGH

**2. R3: the 100 mm depot square is unsupported, and even the example positions are provisional.**

No geometric depot boundary, size, shape or numerical delivery tolerance was found in the official sources. There **is** a general success criterion—correctly depositing assigned objects—but no test that implements “correctly.”

Vision declares depot points in configuration and copies them into telemetry. It neither detects depot regions nor evaluates delivery. `config_simulador.json:59` explicitly marks positions and colours **unconfirmed against the physical setup**.

The research simulator does define delivery: **cube-centre distance ≤ 1 cell = 20 mm**, independent of orientation. That is a research convention, not an official rule.

Evidence: `O/reglamento.md:170`, `O/el_reto.md:135`, `O/vision-system/contrato/config_simulador.json:59`, `V/mundo.py:135`, `S/04_Banco_Pruebas/simulador.py:387`.

**Fix:** default to the published point as the placement target, with a configurable engineering stopping tolerance. Keep official delivery status **unverified** until the actual acceptance criterion is supplied. Retain the 100 mm/full-containment model only as an explicitly experimental scenario.

**3. G4 is semantically wrong; G2/G6/G7 lack a verified common reference point.**

Published rover pose means **rotation centre and paddle-forward heading**, after marker-offset correction. It equals marker pose only because the current offsets are zero. Position is also corrected for marker-height parallax.

The two rejected position-offset measurements do not establish zero offset. Angular alignment near zero was measured for **rover 10 with that particular marker mounting**, not both robots. Separately, the chassis-centre-to-axle offset remains unknown.

Evidence: `V/detectors/rovers.py:128`, `:190`, `:272`; `V/config_vision.json:130`, `:139`, `:142`. Both `V/mediciones/desfases_rover10_20260809_*.json:2` report `INSUFICIENTE`.

**Fix:** distinguish marker centre, axle midpoint and chassis centre. Measure their transforms for each rover; express collision geometry about the actual controlled point. Avoid applying an offset twice if vision already compensates it. Until measured, the **113.49 mm** sweep radius is conditional.

**4. C1/C4: a push does not establish cube orientation.**

The channel is wider than the cube’s maximum projection: **93.5 > 84.85 mm**. It permits every cube orientation. The research explicitly warns that a cube can rotate during transport. Actual verified face-to-face contact would constrain orientation, but telemetry does not establish that contact.

The detector internally estimates cube angle modulo 90°, then tracking discards it. There is **no hidden orientation field in official v1 telemetry**. Modifying official vision to expose it conflicts with `robot.md`.

Evidence: `S/03_Informe_Resultados/fuente/informe.tex:1335`; `V/detectors/cubos.py:230`, `:327`; `V/tracking/seguimiento.py:118`; `O/vision-system/contrato/schema.py:98`; `O/robot.md:95`.

**Fix:** retain an orientation belief unless a validated contact manoeuvre constrains it. Erode the assumed depot by the worst cube projection over that belief. Do not set cube angle equal to commanded push heading automatically.

**5. R8/F1: the supplied turn helper uses the opposite angle sign.**

Wheel mapping is correct: motor 1 left, motor 2 right. However, `turn_angle.py` converts a **positive** requested angle into left-forward/right-reverse: **clockwise**. Telemetry and F1 use positive **counterclockwise** angles.

Evidence: `O/codigos/wifi_command_receiver.py:56`; `O/codigos/turn_angle.py:128`, `:181`; `O/codigos/test_motores.py:27`.

**Fix:** negate CCW angle requests when using that helper, or implement the canonical differential-drive convention:
`ω = (v_right − v_left) / track`.

## MEDIUM

**6. G8: 5 mm is neither a measured Gaussian σ nor a universal occlusion bound.**

The mock’s **0.06-cell positional σ = 1.2 mm** and **1.5° angular σ** are correct. The research’s 5 mm value is an adopted working allowance covering a reported **4.88 mm** synthetic error with roughly 22% occlusion.

Published cube position estimates the **base-square centre**, using a fitted base/top silhouette—not the coloured blob centroid. This removes the intended centroid/parallax bias; it does not prove statistically unbiased measurements. At approximately 70% occlusion, documented fitting errors reach **34 mm**.

Unreliable detections normally do not refresh position or age. Tracking retains the last accepted position, then removes it after **more than 60 seconds** without acceptance.

Evidence: `O/vision-system/contrato/config_simulador.json:61`; `S/01_Modelo_Matematico/fuente/modelo.tex:656`; `V/detectors/cubos.py:85`, `:327`; `V/detectors/README.md:113`, `:137`; `V/tracking/seguimiento.py:114`, `:149`; `V/config_vision.json:84`.

**Fix:** separate mock noise, observed fitting error, systematic bias and stale-position uncertainty. Grow uncertainty during occlusion; do not keep a fixed 5 mm envelope around a moving, unseen cube.

**7. C2/C3: the reachability percentage is stale, and impossibility claims exceed the evidence.**

The report revises **36.3% → 25.7%**; the stored N3 run gives **25.8946%**. More importantly, that calculation allows the **70 mm physical margin**, checks a chassis rectangle, and uses a **129.43 mm** staging distance. It does not implement strict R5 plus the asymmetric paddle footprint and G7’s 165.91 mm clearance.

An exactly edge-trapped cube may be unrecoverable under specified push primitives. “In a non-depot corner” is too imprecise to prove impossibility.

Evidence: `S/03_Informe_Resultados/fuente/informe.tex:1411`; `S/04_Banco_Pruebas/salidas/run_n3.txt:9`; `S/04_Banco_Pruebas/n3.py:12`, `:43`, `:56`, `:77`.

**Fix:** remove the inherited percentage; recompute under the chosen footprint, boundary and staging policy. State impossibility only for defined contact configurations and permitted manipulation primitives.

**8. R8: no measured throttle-to-speed calibration is provided.**

`motor_calibration.py` estimates relative motor compensation using gyro drift. It does not measure millimetres travelled. The mock’s **6 cells/s = 120 mm/s** is a simulator parameter, not a throttle calibration.

Evidence: `O/codigos/motor_calibration.py:3`, `:111`; `O/vision-system/contrato/config_simulador.json:34`; `O/codigos/ideaboard.py:39`.

**Fix:** measure each wheel’s forward/reverse speed versus throttle, including deadband and pushing load. Do not infer distance from wheel diameter without measured rotation.

**9. Missing strategy constraints and start-zone ambiguity.**

- Both official robots must participate; physical/electronic modifications are prohibited.
- Official vision cannot be modified.
- No manual commands, object repositioning, code changes or selective robot restarts during a run.
- Start from the assigned zone. `(0,0)` identifies its corner marker; configured `start=(2.5,2.5)` identifies the zone centre. No zone dimensions or two-rover starting arrangement are specified.
- Time and correctly delivered object count are principal criteria. No numerical round limit, penalty schedule or collision-point tariff was found.

Evidence: `O/reglamento.md:15`, `:99`, `:125`, `:152`, `:170`; `O/robot.md:95`; `V/config_vision.json:102`.

**Fix:** add these constraints explicitly. Read `start` and depot mappings from telemetry; do not infer a 100 mm starting square or invent a round duration.

## LOW

**10. G1/G2: nominal dimensions are supported, but precision and provenance need qualification.**

The assembled measurements used an uncalibrated ruler, approximately **±1 mm**. They support 55 mm paddle reach, roughly 100 × 150 mm envelope, 33 mm wheel diameter and 89 mm track. Paddle height is **0–14 mm**, with tips reaching the floor.

The exact 93.5/99.5/94 mm dimensions come from the documented *separate-pieces DXF*. That DXF is absent from the supplied extracted files; the older supplied fabrication drawing has different reported dimensions.

Evidence: `S/05_Mediciones_CAD/dimensiones_rover.md:29`, `:74`, `:144`, `:158`, `:201`; `S/05_Mediciones_CAD/medir_dxf.py:6`.

**Fix:** label the rectangular chassis/rail model nominal and verify it on both assembled robots. Preserve measurement uncertainty in collision margins.

**11. G7/C1: correct arithmetic, overstated wording.**

G7’s circumscribed-disc clearance is a **sufficient full-rotation clearance**, not a necessary distance for every approach. C1’s quoted ±20/±7.6 mm values assume **zero margin**.

Evidence: `S/04_Banco_Pruebas/modelo.py:161`; `S/05_Mediciones_CAD/recalcular.py:147`; `docs/interpretation_v0.md:47`, `:54`.

**Fix:** state the manoeuvre assumptions and distinguish rotation, approach and delivery margins.

## CONFIRMED claims

- **R1:** TCP/NDJSON, port 2026, protocol v1 and nominal 20 Hz publication. Publication rate does **not** guarantee fresh observations. Evidence: `O/vision-system/contrato/CONTRATO.md:25`; `V/config_vision.json:89`.
- **R2/R5:** current effective field is **860 × 860 mm**, versus **1000 × 1000 mm** physical board and 70 mm margins. Official mounting documentation calls those margins unused. Strict R5 is defensible; a nonzero overhang allowance is not established permission. No walls is supported by the research correction. Evidence: `O/vision-system/MONTAJE.md:127`; `S/03_Informe_Resultados/fuente/informe.tex:1258`.
- **R3/R7/R9:** three colour-matched depot points; two robots, configured IDs 10/11; 2–3 unique-colour cubes; obstacles empty this edition. Identify by ID/colour and handle absent entries. Evidence: `O/vision-system/contrato/CONTRATO.md:192`, `:215`, `:230`, `:262`; `O/vision-system/contrato/config_simulador.json:28`.
- **R6/R8:** movement only during `RUNNING`; central PC planning and automatic commands are permitted. Positive throttle means forward in the supplied examples; left turn is `motor_1<0`, `motor_2>0`. Evidence: `O/vision-system/contrato/CONTRATO.md:337`; `O/reglamento.md:73`; `O/codigos/test_motores.py:13`.
- **F1:** **do not negate telemetry θ.** The detector computes `atan2(-forward_row, forward_col)`. For `(col,row,θ)=(10,20,90°)`, internal pose is **`(200 mm,460 mm,π/2)`**. Advancing 20 mm changes telemetry row to 19 and internal y to 480 mm. Evidence: `V/detectors/rovers.py:176`; `O/vision-system/contrato/CONTRATO.md:299`.
- **G3/G4:** axle/chassis coincidence remains an assumption; position-offset attempts were rejected; approximately zero angular offset is supported for the measured rover only.
- **G5–G7/C1 arithmetic:** independently recalculated, with the reference-point and orientation qualifications above:

| Quantity | Verified value |
|---|---:|
| Cube projected width | 60–84.8528 mm |
| Capture lateral tolerance | 4.3236–16.7500 mm |
| Sweep radius about chassis centre | 113.4860 mm |
| Straight paddle clearance, margin 10 mm | 154.4264 mm |
| Full-rotation clearance, margin 10 mm | 165.9124 mm |
| Assumed depot centre tolerance, aligned, margin 0 | ±20 mm |
| Same, cube at 45°, margin 0 | ±7.5736 mm |
| Same, cube at 45°, margin 10 mm | **Empty feasible region** |

- **C4’s telemetry claim:** cube orientation is not transmitted. Any orientation inferred through interaction remains model-dependent.
EXIT 0
