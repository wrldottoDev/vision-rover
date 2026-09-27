# Geometry model (canonical; all values live in `rover_strategy/config.py`)

## Frames and units
- Internal: mm, rad, s. `x = col * cell_mm`, `y = (rows - row) * cell_mm` (y UP), `theta = radians(theta_deg)` CCW from +x.
  Official forward `(dcol, drow) = (cos, -sin)` maps to internal `(cos, sin)`; no sign flip of theta (verified by Astra
  against `vision/detectors/rovers.py:176`). Example: official `(10, 20, 90 deg)` -> `(200, 460, pi/2)`.
- Only `frames.py` / `vision/parser.py` / the simulator publisher convert units.
- Angles wrapped to [-pi, pi). Cube orientation `alpha` is modulo 90 deg, stored in [-pi/4, pi/4).

## Field
Effective field 860 x 860 mm (43 x 43 cells of 20 mm, read from telemetry `grid`). Physical board 1000 x 1000,
70 mm unused margin per side, no walls. Corner ArUco: 100 mm black + 20 mm white, centred on each field corner;
its in-field quarter is a 70 x 70 mm square at each corner.

## Rover (rover frame: origin = rotation centre, +x forward, +y left)
| Quantity | Value | Status |
|---|---|---|
| Chassis length / outer width / channel width | 94.0 / 99.5 / 93.5 mm | DXF (separate pieces) + ruler (W_u = 94.05 +- 0.54) |
| Paddle reach beyond front plate | 55 mm (rails 3 mm thick, 0-14 mm tall, touch the floor) | ruler |
| Envelope | x in [-47, 102], y in [-49.75, 49.75] | derived; assumes axle at chassis centre |
| Push face (front plate) | x = +47 | derived |
| Rotation sweep radius | hypot(102, 49.75) = 113.49 mm | derived, conditional on axle position |
| Wheel diameter / track | 33 / 89 mm | ruler |
| Marker | 40 mm, ~90 mm high; residual offset to rotation centre unknown (vision config says 0) | provisional |

Collision checks against anything the rover must not touch use the convex **envelope** (the channel counts as occupied),
inflated by margins. The simulator and capture logic use the real U shape (`Footprint.parts`).

## Cube
60 mm square. Axis-aligned half extent for orientation `a`: `30(|cos a| + |sin a|)` in [30, 42.43].
Unknown orientation => half diagonal 42.43 (disc obstacle).

## Capture geometry (finding H-11, re-verified)
Lateral capture tolerance `e_max(d) = (93.5 - 60(|cos d|+|sin d|)) / 2` in [4.32, 16.75] mm, `d` = push heading vs nearest face normal.
With vision noise of the same order at 45 deg, capture of an unknown-orientation cube is not guaranteed; mitigations:
stationary averaging before capture, orientation inference from contact geometry (below), verification + retry.

**Orientation observability through contact:** when the cube rests against the front plate, the cube-centre distance
from the plate equals its projected half-depth `30(|cos a|+|sin a|)`: 30 mm flush vs 42.4 mm at 45 deg. Measuring the
cube centre in the rover frame during/after capture discriminates flush contact (orientation ~ push heading mod 90)
from corner contact, up to sign. This is model-dependent (needs a calibrated marker/axle offset) and is treated as a
belief with std, never as ground truth.

## Distances
- Pre-push distance (rotation centre to cube centre): `max(102, 113.49) + 42.43 + prepush_clearance + pose_uncertainty`
  = 113.49 + 42.43 + 12 + 6 = **173.9 mm**. Sufficient (not necessary) for BOTH a straight approach without paddle
  contact and a full in-place rotation at the pre-push pose for any cube orientation.
- Contact distance (flush): 47 + 30 = 77 mm. Capture travel = 173.9 - 77 = ~97 mm.
- Retreat before turning after a push: >= paddle reach + clearance so the paddles clear the cube (finding H-15).

## Depot (ASSUMED, see rules_constraints.md)
100 mm square centred on the published point. Valid cube-centre region for orientation `a` and margin `m`:
the square eroded by the rotated cube: half-size `50 - 30(|cos a|+|sin a|) - m` -> +-14 mm aligned with m = 6;
empty at 45 deg with m >= 7.6. Hence final legs into a depot should be axis-aligned.
