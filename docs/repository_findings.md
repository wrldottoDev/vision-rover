# Repository findings — Vision Rover Challenge

Research worker A. Read-only pass over `_source/Rover Vision Artificial/`. All
paths below are relative to that directory unless marked `V1/docs/...`.
Shorthand used in citations:

- `REPO` = `Vision-Rover-Challenge-main_unz/Vision-Rover-Challenge-main/`
- `VS`   = `REPO/vision-system/`
- `RES01..RES05` = `01_Modelo_Matematico/` … `05_Mediciones_CAD/`
- `BANCO` = `04_Banco_Pruebas/`

Status tags follow the source's own vocabulary: **MEASURED** (medido/confirmado),
**DERIVED** (calculado desde otras medidas), **ASSUMED** (declarado sin
verificar), **PROVISIONAL** (explicitly marked pending in the source).

---

## 1. Authoritative measurements

### 1.1 Board / grid

| Item | Value | Status | Source |
|---|---|---|---|
| Physical board | 50×50 squares (1000×1000 mm), no walls | MEASURED | `reglamento.md:5-9`, `robot.md:16` |
| Effective playfield | 43×43 cells × 20.0 mm = 860×860 mm | **MEASURED** 4-Aug-2026, by counting squares between corner-marker *centers* | `VS/vision/config_vision.json:7-10`, `VS/contrato/config_simulador.json:15-19` |
| Dead margin | 3.5 cells/side (7 cells total per axis) where corner markers sit; unused | MEASURED (derived from above) | `VS/contrato/CONTRATO.md:179-190` |
| Cell size | 20.0 mm | MEASURED | same as above |
| Corner marker dict | `DICT_4X4_50`, ids 0–3 | ASSUMED/fixed by config | `VS/vision/config_vision.json:13` |
| Corner marker size | 100×100 mm black + 20 mm white border (20% margin rule) | **CONFIRMED** 9-Aug-2026 | `VS/vision/config_vision.json:14-16` |
| Corner layout rule | ID0=origin(start corner)=(0,0); 1,2,3 clockwise: (cols,0),(cols,rows),(0,rows) | ASSUMED — physical mounting instruction, not measured | `VS/vision/config_vision.json:23-48`, `VS/CLAUDE.md` (marker layout block) |
| Marker deviation threshold (3-marker fallback) | 2.0 mm | DERIVED, from measured 0.25° camera-shift test (3-marker error 1.58 mm vs real 0.92 mm) | `VS/vision/config_vision.json:18-21` |

### 1.2 Cube

| Item | Value | Status | Source |
|---|---|---|---|
| Side / height | 60.0 mm (height = side, it's a cube) | **CONFIRMED** 9-Aug-2026 | `VS/vision/config_vision.json:53-65` |
| Colors | red, green, blue (yellow reserved for obstacles) | fixed by contract | `VS/vision/config_vision.json:55-64`, `VS/contrato/schema.py:78-88` |
| Localization point | bottom edge (floor contact), NOT color-blob centroid | design decision, MEASURED consequence (blob centroid error 30-42mm) | `VS/vision/config_vision.json:62-63`, `VS/vision/detectors/cubos.py:31-63` |
| Cube fabrication size (independent check) | 180.30 units side / 3 = 60.10 mm/cube | MEASURED (DXF cross-check) | `RES05/dimensiones_rover.md:21` |

### 1.3 Rover chassis / channel / paddles / wheels / track

Two independent DXF sources disagree slightly; the "separated parts" file is
authoritative because it declares mm natively (`$INSUNITS=4`) while the
single-body DXF needed 3-way scale inference.

| Item | Combined DXF value | Status | Source |
|---|---|---|---|
| Chassis body (single DXF) | 98.66 × 93.20 mm | MEASURED (scale inferred via 3 independent checks) | `RES05/dimensiones_rover.md:20-29` |
| Chassis body (separated-parts DXF) | 93.50 × 94.00 mm | MEASURED, `$INSUNITS=4` (native mm) | `RES05/dimensiones_rover.md:84-93` |
| Chassis w/ tabs (outer) | 99.50 × 94.00 mm | MEASURED/DERIVED (tabs = 3.00 mm/side = acrylic thickness, "through-joint") | `RES05/dimensiones_rover.md:96-116` |
| **W_u** — inner channel width (paddle-to-paddle, governs `e_max`) | **93.50 mm** | MEASURED/DERIVED from tabs; cross-checked by ruler on assembled prototype: 94.05 ± 0.54 mm (DXF's 93.50 within 1.0σ) | `RES05/dimensiones_rover.md:96-116, 173-186` |
| **W_ext** — outer footprint width (governs collisions, R_rov) | **99.50 mm** | MEASURED (DXF) + ruler check (100 mm) | `RES05/dimensiones_rover.md:107-108, 164` |
| Total length | 149 mm (94 chassis + 55 paddle reach), ruler-confirmed 150 mm | MEASURED | `RES05/dimensiones_rover.md:192-199` |
| **λ — paddle reach ahead of front plate** | 55 mm | **MEASURED on assembled rover** (ruler ±1 mm) — not derivable from any DXF | `RES05/dimensiones_rover.md:163, 189-199` |
| Paddle geometry | parallel channel ("U", straight rails), not a V-funnel | Author-supplied fact; a V-funnel reading of a front photo was tried twice and refuted (perspective artifact) | `RES05/dimensiones_rover.md:53-68`; informe.tex:1311-1333 |
| Paddle height off floor | 0–14 mm | MEASURED (ruler) | `RES05/dimensiones_rover.md:166, 206-207` |
| Wheel diameter | 33 mm | MEASURED | `RES05/dimensiones_rover.md:168, 208-210` |
| Track (wheel-to-wheel) | 89 mm | MEASURED | `RES05/dimensiones_rover.md:168, 208` |
| Rover footprint asymmetric about chassis center | x ∈ [−47, +102] mm, y ∈ [−49.75, +49.75] mm (front plate at x=+47) | DERIVED from above | `informe.tex:824-833` (R_rov formula), interpretation_v0.md G2 |
| R_rov (envelope radius, corrected) | 113.49 mm (was wrongly 68.44 mm before λ was known) | DERIVED, +66% | `informe.tex:821-833` |
| **`sintetico.cuerpo_rover` in config_vision.json** | 120×140×90 mm | **PROVISIONAL / WRONG for real geometry** — explicitly for synthetic-image rendering only, not consumed by detection | `VS/vision/config_vision.json:238-244`; flagged as discrepancy in `RES05/dimensiones_rover.md:35-46` |
| Rover marker→rotation-center offset (position) | (0, 0) mm (both axes) | **NOT MEASURED / left at 0 as best-available value.** Two capture attempts REJECTED as INSUFFICIENT (robot translated 6–12 cm while turning); only bound obtained: offset < ~10 mm | `VS/vision/config_vision.json:140-141` |
| Rover marker angular offset | 0° | **MEASURED and CONFIRMED** ≈0° (±1°) via two independent 4-orientation sessions (+0.18°, −0.50°) | `VS/vision/config_vision.json:139` |
| Rover ArUco marker (on-robot) size | 40 mm black + 5 mm white border | **PROVISIONAL — NOT YET VERIFIED** for stable detection at working camera height | `VS/vision/config_vision.json:67-79` |
| Marker mount height above board | 90 mm | **CONFIRMED** 9-Aug-2026 | `VS/vision/config_vision.json:199-201` |

### 1.4 Camera

| Item | Value | Status | Source |
|---|---|---|---|
| Production camera | ArgomTech CAM40, wide-angle, manual focus, 1920×1080 @30fps MJPG | ASSUMED default profile | `VS/vision/config_vision.json:263-273, 336-338` |
| Alt. profile present | Logitech C270 | calibration profile exists | `VS/vision/vision/calibraciones/logitech_c270.json` |
| Camera mount height (synthetic sim) | 2100 mm | matches real CAM40 mount ("minimum that fits the whole board") | `VS/vision/config_vision.json:253-254` |
| Camera height for precision-test tool | 1300 mm | different rig/purpose (bench precision test) — **do not conflate with production mount height** | `VS/vision/config_vision.json:364` |
| Distortion correction | mandatory, precomputed undistort maps, `alpha=0` (crop) by default | design decision | `VS/vision/vision/geometry/distorsion.py:1-32, 443-462` |
| Camera pose (extrinsics) | solved per-frame via `solvePnP` from the 4 corner markers (never hand-declared) | derived at runtime | `VS/vision/vision/geometry/coordenadas.py:332-388` |
| Exposure/focus/WB | always manual, fixed; verified "by effect" (two very different test values must show different images) | design decision | `VS/vision/vision/config_vision.json:280-313` |

### 1.5 Vision noise / publish figures

| Item | Value | Status | Source |
|---|---|---|---|
| Simulator position noise σ | 0.06 cell = 1.2 mm (Gaussian) | ASSUMED (simulator design parameter, not measured on real hardware) | `VS/contrato/config_simulador.json:61-64` |
| Simulator theta noise σ | 1.5° | ASSUMED (same) | ibid |
| Real detector position error, cube clear | 1.05–1.40 mm (matches published baseline within 0.47%) | **MEASURED** (synthetic-image pipeline, real ArUco+homography+silhouette code) | `informe.tex:421-433` |
| Real detector position error, rover pushing cube (~22% occluded) | 4.88 mm | MEASURED | `informe.tex:429, 745-767` |
| Real detector position error, rover covering ~70% | 34.31 (cenital) / 12.63 (tilted) mm, flagged `confiable=false` | MEASURED | `informe.tex:430` |
| Real detector angular error, cube clear | mean 0.42°, p95 1.00°, max 1.50° (quantized by optimizer step, not image noise) | MEASURED | `informe.tex:438-463` |
| Real detector angular error, ~22% occluded | mean 10.92°, max 19.00°, **100% still flagged `confiable=true`** | MEASURED — root of H-03 | `informe.tex:464-484` |
| `residuo_maximo_celdas` (cube reliability threshold) | 0.20 cells | DERIVED from measured residues: 0.014-0.017 (clear), 0.073-0.133 (22% occluded), 0.24-0.28 (70% occluded) | `VS/vision/config_vision.json:191-192` |
| Publish rate | 20.0 Hz, own timer, decoupled from camera capture (~30 fps camera, ~51 Hz detection loop) | fixed by config | `VS/vision/config_vision.json:93-95`; `informe.tex:1088-1090` |
| Track age-out (ghost sweep) | 60000 ms (not for occlusion — for spurious detections / object truly removed) | design constant | `VS/vision/config_vision.json:84-85` |

---

## 2. Telemetry contract (for planner design)

Source of truth: `VS/contrato/CONTRATO.md` (spec) + `VS/contrato/schema.py`
(validator/wire format) + `VS/vision/vision/sistema.py`,
`vision/geometry/coordenadas.py`, `vision/detectors/cubos.py`,
`vision/tracking/seguimiento.py` (implementation).

### 2.1 Fields

- Root: `v`, `seq`, `ts_ms`, `phase`, `grid{cols,rows,cell_mm}`, `rovers[]`,
  `cubes[]`, `obstacles[]`, `start{col,row}`, `depots[]`.
  Exact field set is enforced by `_CAMPOS_MENSAJE` etc. — unknown/missing
  fields reject the message (`VS/contrato/schema.py:93-101, 346-357`).
- `rovers[]`: `id` (ArUco id = identity, never index), `col`, `row`,
  `theta` (deg, 0=+col, CCW, range **[0,360]**), `age_ms`.
  `CONTRATO.md:192-204`; `schema.py:151-179`.
- `cubes[]`: `color` (identity, no id), `col`, `row`, `age_ms`. **No
  orientation field** — confirmed absent from `_CAMPOS_CUBE` and
  `Cube` dataclass. `CONTRATO.md:206-216`; `schema.py:97-98,182-197`.
- `obstacles[]`: `col,row,age_ms`, no color/id (yellow reserved,
  interchangeable). Always empty this edition but iterate, don't assume.
  `CONTRATO.md:218-243`.
- `start`/`depots`: static, declared not detected, **no `age_ms`**.
  `CONTRATO.md:244-280`.

### 2.2 Timing semantics

- `ts_ms` = **frame CAPTURE instant** (Unix ms), sealed at the camera source
  and carried unchanged through rectify→detect→track→publish. It is *not*
  send time. `CONTRATO.md:162`; `VS/vision/vision/mundo.py:23-29`;
  `VS/vision/vision/sources camara.py` (ts sealed at capture, not verified
  in this pass but referenced throughout `sistema.py`).
- `age_ms` = ms since the object was last *actually seen* (rover: ArUco
  found; cube: found **and** fit judged `confiable`). Grows monotonically
  while occluded; does **not** reset on an unreliable detection.
  `CONTRATO.md:200,213`; `VS/vision/vision/tracking/seguimiento.py:26-44,109-122`.
- `seq` increments **once per published message**, not per processed frame;
  gaps mean a client didn't drain fast enough (last-value-wins), not lost
  camera frames. `CONTRATO.md:160,413-429`.
- Last-value-wins: one-message-per-client slot; a slow client sees fewer
  messages, always the newest. Never queued. `CONTRATO.md:413-429`;
  `VS/vision/vision/publish/telemetria.py:24-44,136-146`.
- Publisher and processing loop run on **independent clocks**: publish
  fires on its own 20 Hz timer even if a camera frame stalls, re-emitting
  the last good state with an aging `ts_ms`. `telemetria.py:119-146`.

### 2.3 Phases

`IDLE → READY → RUNNING → FINISHED → READY (next round)`. Vision is sole
arbiter; robot obeys `phase`, moving only in `RUNNING`, stopping immediately
otherwise. Vision keeps publishing in every phase. `CONTRATO.md:332-352`;
implemented as an explicit transition table in `sistema.py:98-134` (Árbitro)
and `mock_publisher.py:433-467` (Fase). Same 3 command names (`ready`,
`start`, `stop`) drive both the simulator and the real system.

### 2.4 Occlusion behaviour (age_ms, never disappears)

Both cubes and rovers **never leave the list** when hidden: last known
position is kept, `age_ms` grows. This is a hard contract guarantee
(`CONTRATO.md:386-411, 935-955`) implemented identically in the tracker
(`seguimiento.py:80-138`) and the simulator (`mock_publisher.py:350-414`).
Key nuance not obvious from the contract text alone: **for cubes**, an
"unreliable" fit (`confiable=False`, i.e. residual > 0.20 cells) is
*treated the same as no detection* — it does **not** refresh position or
reset age (`seguimiento.py:114-119`, config flag
`refrescar_con_cubos_no_confiables=false` — `config_vision.json:86-88`).
So a cube being pushed can show `age_ms=0` (the shape is still detected)
while its *position/angle accuracy has silently degraded* (see §1.5) —
`age_ms` alone is not a reliability signal for cube pose precision.

### 2.5 Corner-marker occlusion (4 / 3 / 2 visible)

This governs whether *any* telemetry is trustworthy, not per-object aging.
Implemented in `AnclajeCancha` (`coordenadas.py:167-278`):

| Visible corners | Behaviour |
|---|---|
| **4** | Full homography recomputed every frame from the 4 marker centers (intersection of quadrilateral diagonals, not corner-average — bias-free under perspective, `coordenadas.py:123-165`). |
| **3** | The last good homography (from when 4 were visible) is **kept indefinitely** — no time limit — because the camera is bolted down and doesn't move mid-round. The 3 visible markers are reprojected through the saved homography each frame purely to *verify* it (max deviation vs. declared cell, in mm). If deviation ≤ 2.0 mm (config `desvio_maximo_mm`), coordinates keep publishing at full precision. If it exceeds 2.0 mm, geometry is declared stale and the frame is dropped (fail-open: last-good *state* — not homography — keeps publishing). Rationale: an affine fit from 3 points errs 36–67 mm vs. 0.5 mm for the true homography at realistic tilt angles — 3-point re-fitting is explicitly rejected as worse than reusing the saved 4-point one. `coordenadas.py:167-278`; `config_vision.json:18-21`. |
| **≤2** | `ErrorGeometria` raised — no coordinate system can be built or verified. The whole-frame try/except in `sistema.py:288-298` catches this: the tracker is **not** told a frame happened (ages of all objects keep growing as if nothing was observed), and the publisher keeps re-emitting the last good `EstadoMundo`. `coordenadas.py:239-244`; `sistema.py:165-194,276-298`. |

Net effect for a planner: robust to **one** missing/occluded corner marker
indefinitely; two or more missing corners means the *entire* telemetry
stream freezes (ages grow, but published `col/row/theta` numbers stop
updating) until 3+ reappear and re-verify.

### 2.6 How cube position is computed; reliability flag; residual threshold

`vision/detectors/cubos.py`: the visible color blob is **not** the cube
(it includes the lid, offset outward by parallax). The detector fits the
**silhouette model** — convex hull of the base square (60 mm known side)
plus its parallax-shifted "lid" — to the observed contour via a 3-unknown
(`col,row,theta`) coordinate-descent search, coarse-then-fine
(`cuadrado`/`silueta_modelo`/`ajustar_cubo`, `cubos.py:175-263`). The cost
function uses a **robust trimmed mean** (`recorte_robusto=0.75`, i.e. worst
25% of contour points discarded) so that the chassis edge occluding part of
the cube doesn't pull the fit (`cubos.py:198-215,306-333`). Output
`residuo_celdas` is the trimmed-mean fit residual;
`confiable = residuo_celdas <= 0.20` (`cubos.py:329-333`;
`config_vision.json:190-192`). **Only the published (col,row) is the base
center**; the detector *does* compute an internal `theta_grados` for the
cube during the fit (needed to build the silhouette model) but **this
angle is never put on the wire** — `mundo.py:117-150` builds `schema.Cube`
with only `color, col, row, age_ms`, confirmed by `schema.py:97-98,182-197`.

### 2.7 Can ANY cube orientation info be obtained?

**Not from the v1 contract as published.** The official vision engine
computes cube `theta_grados` internally (`cubos.py:105,239-263,327`) but the
contract schema and validator have no field for it — this is stated
explicitly by the team's own H-11 finding: *"el detector oficial lo calcula
internamente pero el contrato v1 no publica [el ángulo]"* (`informe.tex:809-811,
1175-1176`). A consuming team's only options are: (a) request a contract
version bump from the organizers (out of participants' control), or
(b) re-derive an approximate orientation client-side after a **flush push**
by assuming the cube took the push heading mod 90° (matches
`interpretation_v0.md` C4) — this is an inference, not a measurement, and
the vision system's own `confiable` flag would not protect it even if it
existed (H-03, §4 below).

---

## 3. Rover firmware facts (`REPO/codigos/`)

### 3.1 Motor API, sign conventions

- Hardware: `IdeaBoard.motor_1`/`motor_2` are `adafruit_motor.motor.DCMotor`
  objects on PWM pins IO12/IO14 (motor_1) and IO13/IO15 (motor_2), 50 Hz.
  `codigos/ideaboard.py:39-44`.
- `.throttle` range is **[-1.0, 1.0]** per motor (float). Confirmed by every
  example (`test_motores.py`, `command_protocol.py: valid_motor_speed`,
  `wifi_command_receiver.py: motor()`).
- **Left/right assignment and sign, from `test_motores.py:13-39`:**
  - `forward`: motor_1=+speed, motor_2=+speed
  - `backward`: motor_1=−speed, motor_2=−speed
  - `left` (turn left in place): motor_1=−speed, motor_2=+speed
  - `right` (turn left... i.e. turn right): motor_1=+speed, motor_2=−speed

  This is consistent across every other firmware sample that drives both
  motors (`code_PID.py:48-49,121-122`, `move_heading.py:247-248`,
  `turn_angle.py:181-187`): **motor_1 = left wheel, motor_2 = right wheel**,
  with `left()` = motor_1 negative / motor_2 positive matching a
  counter-clockwise-from-above in-place spin when combined with the
  contract's CCW-positive `theta` convention (matches `interpretation_v0.md`
  R8). No firmware file documents this mapping explicitly in prose — it is
  inferred consistently from every example's throttle signs.

### 3.2 PID / heading examples and their gains

| File | Controller | Gains (as shipped) | Notes |
|---|---|---|---|
| `code_PID.py::straight_move` | P-I-D on gyro-Z rate error, correction split ± across motors | Kp=0.15, Ki=0.8, Kd=0.05, `max_correccion`=0.3 | `dt` is hard-coded to `1` (bug: `dt = 1 #t_actual...`), so Ki/Kd are effectively mis-scaled — **gains not trustworthy as tuned values**, only as a worked example. `code_PID.py:71-132` |
| `code_PID.py::girar_grados` | Open-loop-ish gyro-integrated turn with 2-stage speed (0.25→0.15 at half-remaining) | speed=0.25 default, "−2°" correction for overshoot | `code_PID.py:39-69` |
| `move_heading.py::move_heading` | PID on heading error (integrated gyro-Z), correction split across motors | Kp=0.015, Ki=0.0005, Kd=0.002, max_correction=0.30 | Real `dt` used (`time.monotonic()` diff); heading integrated from gyro, not absolute. `move_heading.py:118-249` |
| `turn_angle.py::turn_angle` | Two-speed (fast/slow near target) open-loop gyro-integration turn | speed=0.30, slow_speed=0.15 under 30° remaining, tolerance=2° | `turn_angle.py:89-210` |

All three IMU-based files independently implement **gyro drift calibration**
(`calibrate_drift`/`calibrar_drift`): average gyro-Z over 2-3 s at rest,
rejecting samples >0.05-0.008 rad/s as outliers, subtract from every
subsequent reading. `code_PID.py:23-37`, `move_heading.py:76-111`,
`turn_angle.py:48-82`.

**None of these gain sets should be trusted as pre-tuned for this specific
rover** — they're teaching examples ("Ver documento explicativo" link in
`code_PID.py:5-6`) with an acknowledged bug (`dt=1`) in one of them, and no
file states they were tuned against the mass/friction of *this* mechanically
final rover (paddles, W_ext=99.5mm etc. — measured after these files were
written).

### 3.3 IMU availability

- Rover ships with LSM6DS3TRC (accel+gyro) over I2C at 0x6B (0x6b in
  `code_acc.py`), read via `adafruit_lsm6ds`. `code_acc.py:1-16`,
  `code_PID.py:13-18`, `move_heading.py:27-38`, `turn_angle.py:22-33`.
- Configured ranges in `code_acc.py`: accel ±8G, gyro ±2000°/s, both at
  1.66 kHz data rate (`code_acc.py:13-16`) — this is the only file that sets
  explicit ranges/rates; the PID/heading/turn examples use sensor defaults.
- `code_acc.py` also demonstrates writing samples to `datos.csv` via the
  `storage` module (on-board flash logging). `code_acc.py:44-51`.
- Reglamento/robot.md also list ultrasonic (HC-SR04, footprint confirmed in
  DXF cross-check, `RES05/dimensiones_rover.md:20`), 4x IR line/grid
  sensors, and a color sensor as onboard — **none of these are exercised in
  any closed-loop test in the team's own N4 campaign** (`informe.tex:1150-1152`,
  see §4).

### 3.4 Command protocol format

`codigos/command_protocol.py` defines a pipe-delimited text protocol,
explicitly transport-agnostic ("TCP/Wi-Fi, ESP-NOW, serial, test files",
`command_protocol.py:24-30`):

| Command | Wire format | Params validated |
|---|---|---|
| `PING` | `PING` | none |
| `STOP` | `STOP` | none |
| `MOTOR` | `MOTOR\|left\|right` | both in [-1.0, 1.0] |
| `TURN` | `TURN\|angle\|speed` | speed in [0, 1] |
| `HEADING` | `HEADING\|heading\|speed\|duration` | speed in [-1.0,1.0], duration > 0 |

`parse_command()` returns a dict with `valid:bool` and either the parsed
fields or an `error` string; never raises. `command_protocol.py:173-432`.
**This is a protocol *definition* only** — no file in `codigos/` wires it to
an actual transport loop; `wifi_command_receiver.py` implements its *own*,
simpler, space-delimited `MOTOR <l> <r>` / `STOP` / `PING` grammar over raw
TCP and does **not** import `command_protocol.py` (`wifi_command_receiver.py:78-133`
vs `command_protocol.py`) — the two command grammars are **not the same**
and would need to be reconciled by whoever builds the real rover client.

### 3.5 ESP-NOW example: capabilities/limits

`codigos/espnow_bidirectional.py` demonstrates bidirectional ESP-NOW between
two IdeaBoards using CircuitPython's `espnow` module: PING/PONG heartbeat
every 2.0 s, `STATE|<x>` and `TASK|<x>` message types, sender-id-prefixed
payloads (`"<ROBOT_ID>|<CMD>|<DATA>"`), non-blocking receive loop with
10 ms poll (`espnow_bidirectional.py:41-421`). A fixed radio channel (6) is
set via a temporary, immediately-stopped AP (`start_ap`/`stop_ap`) so both
peers share a channel without an actual Wi-Fi network
(`espnow_bidirectional.py:79-91`). **No payload-size limit or latency
figure is stated anywhere in this file or in `codigos/ESPNOW/README.md`** —
the README is a bare MicroPython-flavored (`network`+`espnow`, not
CircuitPython) "hello world" between two boards with no protocol beyond a
literal `"Hola"` string (`codigos/ESPNOW/README.md:19-78`). **Gap**: ESP-NOW
payload ceiling (ESP32 hardware limit is commonly ~250 bytes, but this is
*not stated in the repository* and must not be assumed without checking the
IdeaBoard's actual `espnow` module docs).

### 3.6 Wi-Fi TCP command receiver

`codigos/wifi_command_receiver.py`: CircuitPython `wifi`+`socketpool`
server, binds `0.0.0.0:5000`, one client at a time (`server.listen(1)`),
non-blocking socket (`setblocking(False)`), newline-delimited commands,
replies `OK\n`/`ERROR\n` per line. Implements its own **watchdog**:
`WATCHDOG_SECONDS = 0.5` — if no valid command arrives for 0.5 s, motors are
force-stopped (`wifi_command_receiver.py:36-39,253-261`). This watchdog is
a firmware-side safety net independent of (and stricter than) the vision
contract's suggested 500 ms client-side latency cutoff (§4, H-08) — worth
reusing/tightening in the real client, since the team's own analysis found
500 ms far too loose (H-08).

---

## 4. Prior research summary (`01`–`05`, `04_Banco_Pruebas`)

The team ran a 5-level, 50-test verification campaign (`03_Informe_Resultados/
fuente/informe.tex`) against three independent truth sources: analytic
ground truth (N0/N1/N3), the *official* synthetic-image vision pipeline
unmodified (N2), and a from-scratch rigid-body dynamics simulator built for
this campaign (N4) — deliberately **not** the official
`contrato/mock_publisher.py`, which they found geometrically broken (H-01).

### 4.1 What the prior planner/simulator did

- `BANCO/modelo.py`: reference implementation of the math model — frame
  conventions, square/cube parametrization, four closed-form orientation
  estimators B1–B4 (vertex+center, edge, diagonal, Procrustes-on-4-points),
  support-function width bound, parallax, planning primitives.
- `BANCO/simulador.py`: from-scratch dynamics sim — unicycle kinematics,
  rigid-body contact via the separating-axis theorem, directional push
  restricted to the front face, quasi-static rotation from an instantaneous
  center, **silhouette-based** (not footprint-based) occlusion, v1
  telemetry with configurable noise/loss/latency. Corrected mid-campaign
  from footprint-overlap occlusion (which cannot trigger during a real push,
  the same flaw as H-01) to silhouette-overlap (`informe.tex:586-623`).
- `BANCO/planificador.py`: geometric planner consuming v1 telemetry —
  two-stage approach (maneuver ring, then straight push leg), retreat-
  before-turn, multi-leg push decomposition, trap-zone detection/
  abandonment, pairwise corridor-yield collision avoidance.
- `BANCO/n0_n1.py`, `n2.py`, `n3.py`, `n4.py`: the five test levels;
  `arnes.py` is the harness (pass/fail registry).

### 4.2 Monte Carlo / headline numeric results

| Metric | Result | Source |
|---|---|---|
| N4.2, 1 rover, 200 seeds | 98.5% full delivery, t50=43.6s, t90=48.4s, 0 collisions, 100% clean runs | `informe.tex:626-642` |
| N4.2, 2 rovers, 200 seeds | 96.5% full delivery, t50=28.4s (35% faster), t90=32.6s, **0.2 collisions/run mean → 5.5% of runs collide**, 94.5% clean | `informe.tex:626-649` |
| N4.3 (lateral-noise sweep, 1 rover, 40 seeds/point) | delivery rate <0.5 first at σ=14mm — 1.1× the model's predicted 12.57mm control budget (agreement within 10%) | `informe.tex:651-677` |
| N4.4 (latency sweep) | delivery collapses between 100–200 ms with a tight (60s) mission budget; 500ms cutoff rule (H-08) never fires below the point where it would matter | `informe.tex:679-705` |
| N4.6 (occlusion during push, 121 samples) | mean 6.2% / max 26.6% of cube silhouette occluded; `age_ms` stays **0** throughout; lateral error mean 5.07mm, max 13.37mm (exceeds the model's 12.57mm budget once, delivery still succeeded — budget is conservative) | `informe.tex:736-771` |
| N4.8 (single cube vs. 3 simultaneous) | 3.00/3 single-cube; **1.00/3** with all three present, isolating a navigation gap (not a push-geometry gap) | `informe.tex:120-125` |
| Un-deliveries (des-entregas) | **4.00 per run** — cubes that reached the delivery tolerance and were later swept back out by the rover's own in-place turn (H-15) | `informe.tex:866-880` |
| Trap zone (H-09) | 11.22% of the board is unrecoverable-toward-depot (down to 9.53% at zero safety margin — a rover-size problem, not a margin problem); grows to **6.46%** once λ=55mm is included (H-12 adenda) | `informe.tex:1038-1067`, adenda §1250-1310 |
| Direct-push infeasible zone (H-05) | 36.3% of the board (disk-envelope model: 39.96%) | `informe.tex:988-999` |
| Corridor interference (N3.8 → H-07) | 46% of random push-corridor pairs interfere (geometric condition agrees with dense sampling 99.99%) | `informe.tex:1019-1035` |

### 4.3 Known failure modes

- **H-15 — un-deliveries (des-entregas), 4.00/run**: rover retreats-then-
  turns without first backing off `λ=55mm`, so its own paddles sweep the
  cube it just delivered back out of the depot. Fix (implemented in the
  N4 planner): straight retreat of exactly λ before turning.
  `informe.tex:866-880`.
- **Collisions, 5.5% of 2-rover runs**: pairwise corridor-yield is not
  real coordination; N3.8's 46%-interference figure is the underlying
  geometric cause. Down from "100% collide" under the old (wrong, 120×140mm)
  body assumption — the real, smaller rover collides far less, but the
  coordination gap is unchanged. `informe.tex:1019-1035`.
- **Freezes (H-13)**: several planner decision branches return "don't
  move" (v=0, ω=0); once paddles were modeled, an unreachable-before branch
  became reachable and the rover froze with cubes undelivered (200 edge
  events/run). Mitigated with a 2.5s-stuck watchdog that releases the
  target (symptom-level fix, not root-cause). `informe.tex:882-894`.
- **H-14 — other cubes not treated as navigation obstacles**: with the old
  rectangular-footprint model this was nearly harmless; with the real
  paddles the leading tip *hooks* a non-target cube and drags it up to
  36 cells off its depot. `informe.tex:850-864`.

### 4.4 Prior constants — trustworthy vs. suspect

| Constant | Trust it? | Why |
|---|---|---|
| `W_u = 93.50 mm`, `W_ext = 99.50 mm` | **Trust** — closes exactly via the tab/joint geometry in the separated-parts DXF, cross-checked by ruler within 1σ | `RES05/dimensiones_rover.md:96-186` |
| `λ = 55 mm` | **Trust, but it's a single-prototype ruler measurement (±1mm), not a calibrated-instrument reading**; explicitly flagged "verify with calipers when the definitive rover is available" | `RES05/dimensiones_rover.md:163,187`, `informe.tex:815-817` |
| Wheel diameter 33mm, track 89mm | **Trust as ballpark** — ruler-measured on a "preliminary version...uncalibrated school ruler", ±1mm, not cross-checked by a second method | `RES05/dimensiones_rover.md:144,168` |
| `sintetico.cuerpo_rover` (120×140×90mm) | **Do not use for planning geometry** — provisional, for synthetic-image rendering only, 21.6-50.2% oversized vs. DXF | `config_vision.json:238-244`; `dimensiones_rover.md:35-46` |
| Simulator `radio_empuje_celdas=1.5`, `radio_oclusion_celdas=2.0` (official `mock_publisher`) | **Do not calibrate against** — geometrically impossible: real contact distance is 3.3–3.7× the declared push radius; official sim can never occlude a cube during a legitimate push | `informe.tex:896-916` (H-01) |
| Official sim initial rover poses (4 cells apart) | **Do not reuse** — the two rovers' bodies overlap 40mm at those poses (H-06); team's own N4 bench uses 7 cells | `informe.tex:1001-1017` |
| Contract's 500ms latency-cutoff guidance (RI4) | **Inert as specified** — delivery already collapses at 200ms; the rule never fires in the regime where it would help. Design the loop for <100ms, not 500 | `informe.tex:1070-1092` (H-08) |
| Cube `confiable` flag, reused to gate an angle | **Do not reuse for angle filtering** — calibrated against position error only; 57% false-negative rate if applied to a 5° angle-error threshold | `informe.tex:935-967` (H-03) |
| Rover PID gains in `code_PID.py`/`move_heading.py`/`turn_angle.py` | **Treat as starting points only** — teaching examples, one has a `dt=1` bug, none tuned against final rover mass/paddles | §3.2 above |

---

## 5. Contradictions and gaps

| # | Topic | Contradiction / gap | Most defensible default | Risk of that default |
|---|---|---|---|---|
| 1 | Depot zone size/shape | **No source anywhere** (reglamento, el_reto, CONTRATO.md, config_vision.json, config_simulador.json, modelo.tex) defines a depot's spatial extent. Only a center point per color is given. | Axis-aligned square, side 100mm (5 cells), centered on the depot point (matches `interpretation_v0.md` R3) | Could be smaller/larger than the real physical marking on the board; if the real zone is smaller, "delivered" fires too early; if larger, too late |
| 2 | Delivery criterion | **Not defined in any source.** `reglamento.md:170-178` only says "deposited correctly"; no geometric test (all corners inside? centroid inside? overlap %?) is specified anywhere, including the team's own math model (`grep` of `modelo.tex` found no formal definition). | All 4 cube-base corners inside the depot square, using published cube center + inferred/assumed orientation | Orientation itself is not directly observable (§2.7) — any corner-based test inherits that uncertainty |
| 3 | Corner-marker ↔ depot-corner overlap | Depot point (40.5,40.5) coincides with the *inner* corner of the 100mm corner ArUco marker centered at board corner (43,43) — a delivered cube sits right against/over that marker's zone. No source discusses this interaction explicitly; it is a geometric inference from combining `config_vision.json` depot list + marker layout. | Assume corner-marker occlusion risk near that depot; rely on the confirmed 3-marker fallback (§2.5) rather than requiring 4 markers during delivery | If 2 corners get occluded simultaneously (marker + something else), telemetry freezes exactly when precision matters most |
| 4 | Start-zone physical size | Only a center point `(2.5, 2.5)` cells is given (`CONTRATO.md:244-251`); H-06 shows the *simulator's* rover start poses overlap bodies by 40mm, and the team explicitly flags "measure the real start zone" as unresolved (`informe.tex:1012-1017`). | Assume ≥7 cells separation between the two rovers' start poses (matches team's own N4 bench fix) until physically measured | If the real zone is tighter, two real (99.5mm-wide) rovers may not fit side-by-side as planned |
| 5 | Board has walls or not | `reglamento.md`/`el_reto.md`/`robot.md` never say explicitly; the team's own informe.tex **revises this mid-document**: first modeled as walls (H-09 v1), then corrected in the Adenda to "no vertical walls — it's a cliff, overshoot falls off and is lost" (`informe.tex:1258-1272`). | Treat board edge as a cliff, not a wall: keep the *wheel line* (not full body) inside the 43×43 area, forbid any planned trajectory that could carry the center of mass past the edge | Falling off ends the round for that cube/rover — a much worse failure mode than a wall-bump; must be treated as a hard, one-way constraint |
| 6 | Rover marker→center offset (position) | **Explicitly unmeasured** — two calibration attempts both REJECTED as insufficient (robot translated 6-12cm while "spinning"); current value (0,0) is a placeholder, not a measurement, per the config file's own wording. | Use (0,0) but budget an explicit ±10mm position-offset uncertainty margin in any tight maneuver (matches `interpretation_v0.md` G4) | If the true offset is close to 10mm, any margin tighter than that is silently wrong |
| 7 | Cube orientation availability | Team's H-11 explicitly states the ambiguity: detector computes it, contract doesn't publish it. Not really a contradiction — a confirmed, documented gap — but critical because H-11 makes blind pushing marginal/negative in 22.8% of orientations under `W_u=93.50`. | Either request a contract change from organizers, or restrict pushes to within ~21° of a cube-face normal and accept some cubes need a re-approach (matches `interpretation_v0.md` C4) | Ignoring this reproduces exactly the negative-control-budget failure mode H-11 documents |
| 8 | Official simulator vs. real geometry | Official `contrato/mock_publisher.py` push/occlusion radii are 3.3-3.7× too small for the real rover body (H-01) — a team calibrating exclusively against the official simulator will learn wrong approach distances and ram cubes on the real board. | Do not use official `mock_publisher.py` distances for approach-distance tuning; use geometry derived from §1.3 (W_ext, λ, R_rov) directly, only using the official simulator for contract-format/timing testing | None if followed; the risk is entirely in *not* following it (this is the team's strongest documented warning) |
| 9 | Two widths, one name ("ancho") | Casual descriptions (and `sintetico.cuerpo_rover`) can conflate `W_ext` (99.50mm, outer footprint) with `W_u` (93.50mm, inner channel) — team explicitly flags this as a 69% tolerance-inflation bug that "makes H-11 disappear" if made. | Always carry both constants separately, named distinctly, in any planner code/spec derived from this document | Silent tolerance inflation, i.e. planner thinks it has margin it doesn't have |
| 10 | ESP-NOW payload/latency figures | Not stated anywhere in the repo (`codigos/espnow_bidirectional.py`, `codigos/ESPNOW/README.md`). | Assume the common ESP32 ESP-NOW payload ceiling (~250 bytes) and treat it as unverified; keep inter-rover messages short and self-delimited (the pipe-format already is) | If actual driver limit differs, message truncation could silently corrupt a coordination message |
| 11 | Wi-Fi command grammar vs. shared protocol module | `codigos/command_protocol.py` (pipe-delimited, PING/STOP/MOTOR/TURN/HEADING) is never actually used by `codigos/wifi_command_receiver.py`, which implements its own space-delimited MOTOR/STOP/PING grammar. Neither is authoritative for "the" rover client, which per `CONTRATO.md:100-108` doesn't exist yet in this repo (only promised in CircuitPython for a future date). | Build the real client protocol from `command_protocol.py`'s richer grammar (it already covers TURN/HEADING) rather than `wifi_command_receiver.py`'s narrower one, but validate against IdeaBoard's actual CircuitPython socket behavior first | Building against the wrong one means throwing away work when the "real" reference client ships |
| 12 | Camera mount height: 2100mm vs 1300mm | Two different height values appear in `config_vision.json`: `sintetico.altura_camara_mm=2100` (labeled as matching the real CAM40 mount) vs `precision.altura_camara_mm=1300` (a bench precision-test rig parameter). Not actually contradictory once read carefully (different purposes) but easy to conflate. | Use 2100mm as the production mount-height assumption; treat 1300mm as belonging only to the bench precision-test tool | Using 1300mm for parallax reasoning about the real production camera would be wrong by ~40% |
| 13 | Rover marker size 40mm | Marked **PROVISIONAL — not yet verified** for stable detection at working camera height; a documented fallback (60mm on the long side of the available 50×70mm space) exists if 40mm fails. | Do not assume 40mm marker detection reliability figures for planning; if own testing shows instability, budget for the 60mm fallback and its slightly different `borde_blanco_mm` | Silent detection drop-outs at range if 40mm turns out too small |
| 14 | Depot color↔corner assignment | Both `config_vision.json` and `config_simulador.json` explicitly warn this is decided at physical setup time and can change; current file values (green NE, blue SW, red SE corner, using contract axes) are placeholders. | Never hard-code a color→corner mapping; always read `depots[]` from the live message and match cubes by `color`, per contract rule 6.1 | Hard-coding breaks the moment the field crew re-tapes the board |
| 15 | Board-edge fall risk vs. planner's "stay inside effective 43×43" rule | The team's own Adenda correction (item 5 above) means a planner must treat overshoot past the 43×43 boundary as catastrophic (falls off), not merely a rules violation — this is a *late* correction to their own model, so any planner logic copied from the pre-Adenda parts of `modelo.tex`/earlier informe sections may still assume a wall. | Explicitly re-derive any "stay inside boundary" logic from the Adenda section (`informe.tex:1258-1272`), not from earlier passages | Reusing pre-Adenda logic silently reintroduces a wall assumption that no longer holds |

---

## Key file index (for follow-up reading)

- Contract spec: `VS/contrato/CONTRATO.md`, `VS/contrato/schema.py`,
  `VS/contrato/mock_publisher.py`, `VS/contrato/config_simulador.json`
- Vision engine: `VS/vision/vision/sistema.py`,
  `VS/vision/vision/geometry/coordenadas.py`,
  `VS/vision/vision/geometry/distorsion.py`,
  `VS/vision/vision/detectors/cubos.py`,
  `VS/vision/vision/detectors/rovers.py`,
  `VS/vision/vision/tracking/seguimiento.py`,
  `VS/vision/vision/publish/telemetria.py`,
  `VS/vision/vision/mundo.py`,
  `VS/vision/vision/configuracion.py`,
  `VS/vision/vision/config_vision.json`
- Rover firmware examples: `REPO/codigos/*.py` (see §3 table),
  `REPO/codigos/ESPNOW/README.md`
- Physical measurements: `RES05/dimensiones_rover.md` (chassis, channel, λ,
  wheels, track — with two ruler-based "adendas" refining W_u and adding λ)
- Prior research verdict: `RES03/fuente/informe.tex` (H-01…H-15, N0–N4
  results, adenda on real dimensions); `RES04_Banco_Pruebas/*.py` (reference
  implementation, simulator, planner, test levels), `RES04_Banco_Pruebas/
  resultados/*.json` (raw metrics), `RES04_Banco_Pruebas/LEEME.md` (index)
- Lead's interpretation under review: `V1/docs/interpretation_v0.md`
