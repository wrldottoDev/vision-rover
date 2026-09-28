# Rover executor ("code.py" role)

Today: `rover/fsm.py::RoverAgent` + `control/controllers.py`, stepped by the supervisor at 20 Hz on the PC, emitting
per-wheel speeds (mm/s). Target: the same logic on each rover (`rover10/code.py`, `rover11/code.py`, NOT IMPLEMENTED),
receiving plans from plan.py, with C drivers for motors/IMU/IR/sonar and ESP-NOW for peer status
(`coordination/protocol.py`: 25-byte versioned status frame with CRC8, implemented and tested, not deployed).

## FSM
IDLE -> NAVIGATE -> ALIGN -> CAPTURE -> VERIFY_CAPTURE -> PUSH -> MEASURE -> (next leg: RETREAT -> replan) |
(final: VERIFY_DELIVERY -> RETREAT -> CONFIRM -> IDLE). Interrupts: WAIT, RELOCALIZE (estimate unsafe),
EMERGENCY_STOP (safety layer), YIELD (parking for the other rover). Every state has a timeout and a failure path;
failures charge a per-task budget; exhausted budget releases the cube with a cooldown.

Invariants:
- `engaged` = the channel may hold a cube. While engaged only capture/push along the committed line or a STRAIGHT
  retreat (bounded heading-hold) is allowed; cleared only after measured backward displacement >= paddle reach + 25 mm.
- Resuming after an interrupt never resumes blind manipulation: engaged -> VERIFY_CAPTURE first.
- Capture verification: >= 4 distinct fresh cube captures; geometric channel-fit test (depth + |lateral| <= 46.75).
- Orientation inference: depth <= 33 mm => flush => orientation = heading mod 90 (std 10 deg); else unknown.
- Delivery confirmation: n distinct fresh stationary captures, whole footprint (worst case over belief) inside the
  assumed depot; re-confirmed after the retreat (CONFIRM).

## Control
- SegmentFollower: rotate (PID on unwrapped heading, deadband kick duty-cycled), straight (line tracking
  omega = -k_h e_h - k_y e_y with gain scheduling, forward/reverse), predictive stopping (v * stop_lag).
- AlignController: rotate to the push heading, bounded lateral nudges (<= 20 mm drift), refuses rotation with the
  cube near the paddles, returns reposition/failed.
- CaptureController: slow straight advance along the push line to contact + 6 mm.
- PushController: controls the cube's measured cross-track (filtered rover-frame cube offset), curvature limit 1/400
  mm^-1, near-goal slowdown, requires fresh cube evidence to finish, debounced lost-cube detection (latency
  compensated).
- RetreatController: straight reverse with heading hold.

## Vision loss (target behaviour vs status)
GOOD -> normal; DEGRADED -> continue slowly, no new capture (NOT IMPLEMENTED: today only GOOD/LOST gating with
lost_age 0.4 s); LOST (age, covariance, blind travel, actuator-fault latch) -> stop -> RELOCALIZE -> resume with
re-verification/replan. IMU/odometry fusion: interface stubs only (NOT IMPLEMENTED).
