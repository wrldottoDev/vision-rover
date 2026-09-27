# Lead-engineer interpretation v0 (input to Astra Gate 1 + 2)

Sources live under `_source/Rover Vision Artificial/` (official repo unzipped in
`Vision-Rover-Challenge-main_unz/Vision-Rover-Challenge-main/`, prior research in
`01_..05_` folders). Everything below is my reading; each item is a claim to attack.

## Rules / contract
R1. Telemetry: TCP NDJSON port 2026, 20 Hz, v=1, fields per `vision-system/contrato/CONTRATO.md`.
    Cube messages carry NO orientation. Rovers carry theta in degrees CCW, 0 = +col, row grows DOWN.
R2. Units in telemetry: cells (float); cell_mm read from `grid` (20.0). Effective field 43x43 cells = 860x860 mm.
    Physical board 1000x1000 mm (50x50 squares), 70 mm dead margin per side, no walls.
R3. Depots: 3 points (one per color) at the 3 non-start corners, e.g. (40.5,2.5),(2.5,40.5),(40.5,40.5) cells.
    **No official depot size / shape / "delivered" criterion exists anywhere in the repo.**
    My default: depot zone = axis-aligned square of side 100 mm (5 cells) centred on the depot point,
    i.e. the 100x100 mm corner square of the effective field. Delivered = all 4 cube corners inside that
    square (with the published cube centre and an orientation belief). Parameterised in config.
R4. The depot point (40.5,40.5) coincides with the INNER corner of the 100 mm black corner ArUco marker
    centred on the field corner (43,43). A delivered cube therefore overlaps the corner marker region
    -> risk of occluding a corner marker. Vision keeps working with 3 of 4 markers.
R5. "Salida de la superficie" is penalised. I interpret the competition surface strictly as the effective
    860x860 field: the planner keeps the whole rover footprint (incl. paddles) inside it minus a margin.
    Config knob `board_overhang_allowance_mm` (default 0).
R6. Phases: move only in RUNNING; stop immediately on FINISHED or anything else.
R7. Two robots, ArUco ids 10 and 11 (from sim config); identify by id, never by list index.
R8. External PC may plan, allocate, avoid collisions and send commands automatically -> centralised
    PC-side planner is legal. Rover motor command = throttle in [-1,1] per wheel (motor_1 = left, motor_2 = right,
    per `codigos/test_motores.py` left(): motor_1=-s, motor_2=+s).
R9. Cubes may be 2 or 3; obstacles list empty this edition but must be iterated.

## Frames / units (canonical internal)
F1. Internal: millimetres, radians, seconds. x = col*cell_mm, y = (rows - row)*cell_mm (y UP, right-handed),
    theta_int = radians(theta_deg) CCW from +x. Official forward vector (dcol, drow) = (cos, -sin) maps to
    internal (dx, dy) = (cos, sin). Conversion only at the telemetry parser and the sim publisher.

## Geometry (from `05_Mediciones_CAD/dimensiones_rover.md`, measured + DXF)
G1. Chassis 94.0 long x 99.5 wide outer (93.5 inner channel + 2x3 mm panels). Paddles extend 55 mm ahead
    of the front plate, inner faces 93.5 apart, 3 mm thick. Total 149-150 x 100 mm.
G2. Footprint in rover frame relative to chassis centre: x in [-47, +102], y in [-49.75, +49.75].
    Front plate (push face) at x = +47. Paddle rails x in [47,102], |y| in [46.75, 49.75].
G3. Rotation centre (axle midpoint) position relative to chassis centre is NOT measured. Default: coincident.
    Wheel diameter 33 mm, track 89 mm (measured).
G4. Published rover pose = ArUco marker pose. Marker->rotation-centre offset NOT measured (two attempts
    rejected as INSUFFICIENT); angular offset measured ~0 deg (+-1). Default offset (0,0) + margin.
G5. Cube 60 mm. Projected width w(D) = 60(|cos D|+|sin D|) in [60, 84.85]. Capture lateral tolerance
    e_max(D) = (93.5 - w(D))/2 in [4.32, 16.75] mm.
G6. In-place rotation sweep radius about chassis centre = hypot(102, 49.75) = 113.5 mm.
G7. Pre-push distance (rotation centre to cube centre) must satisfy both
    paddle clearance 102 + 42.43 + m and rotation clearance 113.5 + 42.43 + m -> ~166 mm with m = 10.
G8. Vision noise: sim publisher sigma_pos = 0.06 cell = 1.2 mm, sigma_theta = 1.5 deg; prior research
    adopts 5 mm cube sigma worst case (occlusion).

## Consequences I intend to design around
C1. Final push leg into a depot should be (nearly) aligned with the depot axes: a flush-pushed cube takes
    the push heading as orientation; eroded valid region half-size = 50 - 30(|cos a|+|sin a|) - margin,
    i.e. +-20 mm aligned, +-7.6 mm at 45 deg.
C2. Many cube positions near edges need multi-leg pushes (prior research: 36% of field lacks a direct push).
C3. Cubes against a board edge whose required push direction points away from the edge are unsolvable under
    strict R5 unless an along-edge leg exists. Cubes in a non-depot corner are unsolvable under strict R5.
C4. Cube orientation is unobservable from contract v1; after a flush push it is inferred = push heading mod 90.
