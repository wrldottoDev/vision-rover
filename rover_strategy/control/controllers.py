"""Motion + push controllers.  Run on the central PC at ~20-50 Hz on the EKF pose estimate
(`world.RoverEstimate`) and output unicycle (v mm/s, omega rad/s) -> `world.WheelCommand.from_unicycle`.

Shared design notes (see NavParams below for the tuned numbers):

Line tracking (`_Line`).  For a straight path from p0 to p1 with geometric direction `path_dir`, define
    along, cross = to_local(pose)        # along-track / cross-track (left positive), via Pose.to_local
    e_h = angle_diff(theta, reference_heading)     # reference_heading = path_dir, or path_dir+pi if reverse
Exact unicycle kinematics give, regardless of forward/reverse:
    d(along)/dt  = v_body * cos(phi),  d(cross)/dt = v_body * sin(phi)     (phi = theta - path_dir)
and since reference_heading is path_dir shifted by a constant (0 or pi), e_h differs from phi by that
same constant, so d(e_h)/dt = d(phi)/dt = omega always, and d(cross)/dt = v_body*sin(phi) = |v_body|*sin(e_h)
in BOTH modes (forward: v_body=+|v|, phi=e_h; reverse: v_body=-|v|, phi=e_h+pi, sin flips, signs cancel).
So a single law  omega = -k_h*e_h - k_y*e_y  (k_h,k_y > 0), with the *speed magnitude* |v| in the gain
schedule, steers correctly for forward AND reverse without any special-casing -- this is also why a rover
left of the line (e_y>0) must turn right (negative omega) whether driving forward or in reverse.

Gain scheduling (`line_gains`).  Linearising sin(e_h)~=e_h gives the 2-state system
    e_y' = |v| e_h,  e_h' = omega = -k_h e_h - k_y e_y
whose characteristic polynomial is s^2 + k_h s + |v| k_y = 0, i.e. a mass-spring-damper with
natural frequency wn = sqrt(|v| k_y) and damping 2 zeta wn = k_h.  Choosing a target (wn, zeta) at the
operating speed gives  k_y = wn^2 / |v|,  k_h = 2 zeta wn.  zeta=1 (critical) is used everywhere here to
get no-overshoot convergence in the ideal model; the derivative terms in PushController's law add extra
damping against the un-modelled ~0.1 s motor lag and ~100-150 ms vision latency that the linear model
ignores.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..config import Config, DEFAULT
from ..frames import angle_diff, wrap
from ..world import Pose, RoverEstimate, Segment, SegKind
from .pid import PID


# --------------------------------------------------------------------------------------- tuning
@dataclass(frozen=True)
class NavParams:
    """Every tunable constant for this module.  TUNED = chosen analytically then adjusted against the
    truth-model tests; ASSUMED = defensible default pending real hardware tuning."""

    # -- rotation (SegmentFollower ROTATE, AlignController) -------------------------------------
    rotate_kp: float = 2.2          # TUNED: 1/s. omega = kp*err - kd*omega_meas (+ small ki).
    rotate_ki: float = 0.15         # TUNED: trims steady bias from L/R gain asymmetry.
    rotate_kd: float = 0.35         # TUNED: phase-lead margin against ~0.1s motor lag + 100ms vision latency.
    rotate_omega_deadband: float = 0.18   # ASSUMED: rad/s. Min |omega| commanded while turning, so the
                                            # per-wheel differential clears the ~10-20 mm/s motor deadband.
    rotate_tol: float = math.radians(1.0)     # TUNED: heading settle tolerance.
    rotate_settle_ticks: int = 4              # consecutive in-tolerance ticks required to call it settled.
    rotate_omega_settle: float = 0.05         # rad/s, |est.omega| below this counts as "stopped turning".

    # -- straight-line tracking (SegmentFollower STRAIGHT / Capture / Retreat) -------------------
    line_wn: float = 3.0            # TUNED: rad/s natural frequency of the cross-track/heading loop.
    line_zeta: float = 1.0          # critical damping (no overshoot in the linear model).
    line_v_gain_floor: float = 15.0 # mm/s: floor used in k_y = wn^2/|v| so the gain doesn't blow up near a stop.
    line_along_tol: float = 3.0     # mm: "done" tolerance on along-track progress.
    v_min_moving: float = 25.0      # ASSUMED: mm/s floor while still moving, comfortably above deadband.

    # -- alignment (AlignController) --------------------------------------------------------------
    align_tol: float = math.radians(0.5)      # tighter than nav rotate: pre-push precision matters.
    align_lateral_tol: float = 3.0            # mm. TUNED: under the worst-case capture slack (~4.3 mm, G5).
    align_nudge_distance: float = 25.0        # mm. ASSUMED: short deliberate hop used to correct lateral offset.
    align_nudge_max_angle: float = math.radians(20.0)
    align_max_nudges: int = 2

    # -- push (PushController) --------------------------------------------------------------------
    push_wn: float = 1.5             # TUNED: gentler than nav (2.0) -- the omega SATURATION (push_curv_max)
                                      # is what actually enforces "no sharp turn", not a weak wn; too small a
                                      # wn only pushes more of the L/R-asymmetry disturbance rejection onto a
                                      # larger steady-state cross-track offset (k_y = wn^2/v), for no benefit.
    push_zeta: float = 1.3          # slightly overdamped: prioritise "no sharp turn" over settling speed.
    push_hdot_gain: float = 0.5     # TUNED: extra damping on the (EKF-supplied) heading rate est.omega.
    push_ydot_gain: float = 0.006   # TUNED: extra damping on the analytic cross-track rate (s/mm... see step()).
    push_cross_deadband: float = 3.0   # mm. Sub-slack deadband so paddle slack doesn't cause chatter.
    push_curv_max: float = 1.0 / 400.0  # 1/mm. Curvature limit: see PushController docstring.
    push_slow_zone: float = 60.0    # mm remaining -> switch from v_push to v_push_slow.
    push_stop_lag_s: float = 0.10   # s. ASSUMED: motor+command lag used to predict coast-to-stop distance
                                     # (coast distance under first-order speed decay with time constant
                                     # tau is exactly v*tau, so this should track the real motor tau).
    push_stop_tol: float = 1.0      # mm: trigger the stop this early relative to a perfect prediction.
    push_lost_lateral: float = 20.0  # mm. ASSUMED: fresh-vision cube lateral offset (rover frame) => lost.
    push_lost_gap: float = 40.0      # mm. ASSUMED: fresh-vision along-offset from expected contact => lost.
    push_vision_weight: float = 0.6  # weight given to fresh vision vs. the contact-offset estimate.

    accel: float = DEFAULT.limits.accel   # mm/s^2, shared with config's planning assumption.


def line_gains(v_mag: float, wn: float, zeta: float, v_floor: float) -> tuple[float, float]:
    """k_h, k_y for the critically-damped (by default) line-tracking law at speed magnitude `v_mag`."""
    v = max(abs(v_mag), v_floor)
    return 2.0 * zeta * wn, (wn * wn) / v


def _decel_speed(remaining: float, accel: float, v_cap: float) -> float:
    """Speed that can still be brought to zero within `remaining` distance at `accel` -- the standard
    accel-limited stopping profile v = sqrt(2 a d), clamped to the cruise cap."""
    return min(v_cap, math.sqrt(2.0 * accel * max(remaining, 0.0)))


# --------------------------------------------------------------------------------------- helpers
class _AngleUnwrapper:
    """Turns a wrapped (+-pi) angle stream into a continuous scalar, so plain PID math and its
    derivative-on-measurement filter work correctly across the +-pi seam instead of seeing a spurious
    jump of ~2*pi."""

    def __init__(self, initial: float):
        self.value = initial
        self._last_wrapped = wrap(initial)

    def push(self, angle: float) -> float:
        self.value += angle_diff(angle, self._last_wrapped)
        self._last_wrapped = wrap(angle)
        return self.value


@dataclass
class _Line:
    """A straight reference line from p0 in direction `path_dir`, optionally driven in reverse."""

    p0: tuple[float, float]
    path_dir: float
    reverse: bool = False

    @property
    def reference_heading(self) -> float:
        return wrap(self.path_dir + math.pi) if self.reverse else self.path_dir

    def errors(self, x: float, y: float, theta: float) -> tuple[float, float, float]:
        """-> (along, cross[left+], heading_error) at world point (x, y, theta)."""
        ref = Pose(self.p0[0], self.p0[1], self.path_dir)
        along, cross = ref.to_local(x, y)
        e_h = angle_diff(theta, self.reference_heading)
        return along, cross, e_h

    @staticmethod
    def through(p0: tuple[float, float], p1: tuple[float, float], reverse: bool = False) -> "_Line":
        return _Line(p0, math.atan2(p1[1] - p0[1], p1[0] - p0[0]), reverse)


class _RotateHold:
    """In-place rotation to a target heading: PID(P,I,D) on the unwrapped heading, plus a minimum
    |omega| kick to clear the motor deadband, settled after N consecutive in-tolerance ticks with the
    estimator reporting near-zero angular rate (so we don't declare done mid-overshoot)."""

    def __init__(self, params: NavParams, tol: float, omega_cap: float):
        self.params = params
        self.tol = tol
        self.omega_cap = omega_cap
        self.pid = PID(params.rotate_kp, params.rotate_ki, params.rotate_kd,
                        out_min=-omega_cap, out_max=omega_cap)
        self._target_unw: float | None = None
        self._unw: _AngleUnwrapper | None = None
        self._settle_count = 0

    def reset(self, target_theta: float) -> None:
        self._target_final = target_theta
        self._target_unw = None
        self._unw = None
        self._settle_count = 0
        self.pid.reset()

    def step(self, theta: float, omega_est: float, dt: float) -> tuple[float, bool]:
        if self._unw is None:
            self._unw = _AngleUnwrapper(theta)
            self._target_unw = self._unw.value + angle_diff(self._target_final, wrap(theta))
        theta_unw = self._unw.push(theta)
        err = angle_diff(self._target_final, wrap(theta))
        omega = self.pid.step(setpoint=self._target_unw, measurement=theta_unw, dt=dt)
        if 0.0 < abs(omega) < self.params.rotate_omega_deadband and abs(err) > self.tol:
            omega = math.copysign(self.params.rotate_omega_deadband, omega)
        if abs(err) < self.tol and abs(omega_est) < self.params.rotate_omega_settle:
            self._settle_count += 1
        else:
            self._settle_count = 0
        return omega, self._settle_count >= self.params.rotate_settle_ticks


class _LineHold:
    """Straight-line tracking PD (no integral -- see module docstring): omega = -k_h*e_h - k_y*e_y,
    gains scheduled from the current speed magnitude."""

    def __init__(self, params: NavParams, wn: float, zeta: float, omega_cap: float):
        self.params = params
        self.wn = wn
        self.zeta = zeta
        self.omega_cap = omega_cap
        self.line: _Line | None = None

    def reset(self, line: _Line) -> None:
        self.line = line

    def errors(self, pose: Pose) -> tuple[float, float, float]:
        assert self.line is not None
        return self.line.errors(pose.x, pose.y, pose.theta)

    def omega(self, e_h: float, e_y: float, v_mag: float) -> float:
        k_h, k_y = line_gains(v_mag, self.wn, self.zeta, self.params.line_v_gain_floor)
        return max(-self.omega_cap, min(self.omega_cap, -k_h * e_h - k_y * e_y))


# --------------------------------------------------------------------------------------- SegmentFollower
class SegmentFollower:
    """Follows one `world.Segment` (ROTATE in place, or STRAIGHT forward/reverse with closed-loop line
    tracking and an accel-limited, overshoot-free stopping profile)."""

    def __init__(self, cfg: Config = DEFAULT, params: NavParams = NavParams()):
        self.cfg = cfg
        self.params = params
        self._rotate = _RotateHold(params, params.rotate_tol, cfg.limits.w_nav)
        self._line = _LineHold(params, params.line_wn, params.line_zeta, cfg.limits.w_nav)
        self.seg: Segment | None = None
        self._last_along = 0.0
        self._last_cross = 0.0
        self._last_heading = 0.0
        self._last_t: float | None = None

    def reset(self, seg: Segment) -> None:
        self.seg = seg
        self._last_along = self._last_cross = self._last_heading = 0.0
        self._last_t = None
        if seg.kind is SegKind.ROTATE:
            self._rotate.reset(seg.end.theta)
        else:
            line = _Line.through((seg.start.x, seg.start.y), (seg.end.x, seg.end.y), reverse=seg.reverse)
            self._line.reset(line)

    def _dt(self, t: float) -> float:
        dt = 1.0 / 30.0 if self._last_t is None else max(t - self._last_t, 1e-6)
        self._last_t = t
        return dt

    def step(self, est: RoverEstimate, t: float) -> tuple[float, float, bool]:
        assert self.seg is not None, "call reset(seg) first"
        dt = self._dt(t)
        pose = est.pose
        if self.seg.kind is SegKind.ROTATE:
            omega, done = self._rotate.step(pose.theta, est.omega, dt)
            self._last_heading = angle_diff(self.seg.end.theta, pose.theta)
            return 0.0, omega, done

        along, cross, e_h = self._line.errors(pose)
        self._last_along, self._last_cross, self._last_heading = along, cross, e_h
        length = self.seg.length
        remaining = length - along
        if remaining <= self.params.line_along_tol:
            return 0.0, 0.0, True

        v_cap = self.cfg.limits.v_nav
        v_mag = min(_decel_speed(remaining, self.params.accel, v_cap),
                    _decel_speed(max(along, 0.0), self.params.accel, v_cap))
        v_mag = max(v_mag, self.params.v_min_moving)
        omega = self._line.omega(e_h, cross, v_mag)
        v = -v_mag if self.seg.reverse else v_mag
        return v, omega, False

    def tracking_error(self) -> tuple[float, float, float]:
        """-> (along, cross, heading) from the most recent step()."""
        return self._last_along, self._last_cross, self._last_heading


# --------------------------------------------------------------------------------------- AlignController
class AlignController:
    """At the pre-push pose: rotate precisely to heading `phi`, then (only if needed) correct a small
    lateral offset relative to the push line with a rotate/drive/rotate-back nudge -- the capture lateral
    tolerance can be as small as ~4 mm (G5), so this matters even though CaptureController also tracks
    the line while it drives in."""

    _ROTATE, _NUDGE_TURN, _NUDGE_DRIVE, _NUDGE_BACK, _DONE = (
        "rotating", "nudge_turn", "nudge_drive", "nudge_back", "done")

    def __init__(self, cfg: Config = DEFAULT, params: NavParams = NavParams()):
        self.cfg = cfg
        self.params = params
        self._rotate = _RotateHold(params, params.align_tol, cfg.limits.w_fine)
        self.line: _Line | None = None
        self._phi = 0.0
        self._phase = self._ROTATE
        self._nudge_count = 0
        self._nudge_target: float | None = None
        self._nudge_start: tuple[float, float] | None = None
        self._last_t: float | None = None

    def reset(self, line_point: tuple[float, float], phi: float) -> None:
        self.line = _Line(line_point, phi, reverse=False)
        self._phi = phi
        self._phase = self._ROTATE
        self._nudge_count = 0
        self._nudge_target = None
        self._nudge_start = None
        self._last_t = None
        self._rotate.reset(phi)

    def _dt(self, t: float) -> float:
        dt = 1.0 / 30.0 if self._last_t is None else max(t - self._last_t, 1e-6)
        self._last_t = t
        return dt

    def step(self, est: RoverEstimate, t: float) -> tuple[float, float, bool, str]:
        assert self.line is not None
        dt = self._dt(t)
        pose = est.pose

        if self._phase == self._ROTATE:
            omega, done = self._rotate.step(pose.theta, est.omega, dt)
            if not done:
                return 0.0, omega, False, self._phase
            _, cross, _ = self.line.errors(pose.x, pose.y, pose.theta)
            if abs(cross) <= self.params.align_lateral_tol or self._nudge_count >= self.params.align_max_nudges:
                self._phase = self._DONE
                return 0.0, 0.0, True, self._phase
            delta = math.asin(max(-1.0, min(1.0, -cross / self.params.align_nudge_distance)))
            delta = max(-self.params.align_nudge_max_angle, min(self.params.align_nudge_max_angle, delta))
            self._nudge_target = wrap(self._phi + delta)
            self._rotate.reset(self._nudge_target)
            self._phase = self._NUDGE_TURN
            return 0.0, 0.0, False, self._phase

        if self._phase == self._NUDGE_TURN:
            omega, done = self._rotate.step(pose.theta, est.omega, dt)
            if not done:
                return 0.0, omega, False, self._phase
            self._nudge_start = (pose.x, pose.y)
            self._phase = self._NUDGE_DRIVE
            return 0.0, 0.0, False, self._phase

        if self._phase == self._NUDGE_DRIVE:
            assert self._nudge_start is not None
            dx, dy = pose.x - self._nudge_start[0], pose.y - self._nudge_start[1]
            travelled = math.hypot(dx, dy)
            if travelled >= self.params.align_nudge_distance:
                self._nudge_count += 1
                self._rotate.reset(self._phi)
                self._phase = self._NUDGE_BACK
                return 0.0, 0.0, False, self._phase
            return self.cfg.limits.v_capture, 0.0, False, self._phase

        if self._phase == self._NUDGE_BACK:
            omega, done = self._rotate.step(pose.theta, est.omega, dt)
            if not done:
                return 0.0, omega, False, self._phase
            self._phase = self._ROTATE
            self._rotate.reset(self._phi)
            return 0.0, 0.0, False, self._phase

        return 0.0, 0.0, True, self._DONE


# --------------------------------------------------------------------------------------- CaptureController
class CaptureController:
    """Slow straight advance at `config.limits.v_capture` along the push line from the pre-push pose,
    with the same closed-loop line tracking as SegmentFollower, until the given travel distance
    (prepush_distance - contact_distance, plus a small caller-supplied overdrive) has been covered."""

    def __init__(self, cfg: Config = DEFAULT, params: NavParams = NavParams()):
        self.cfg = cfg
        self.params = params
        self._line = _LineHold(params, params.line_wn, params.line_zeta, cfg.limits.w_fine)
        self._travel = 0.0
        self._last: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def reset(self, line_p0: tuple[float, float], phi: float, travel_mm: float) -> None:
        self._line.reset(_Line(line_p0, phi, reverse=False))
        self._travel = travel_mm
        self._last = (0.0, 0.0, 0.0)

    def step(self, est: RoverEstimate, t: float) -> tuple[float, float, bool]:
        pose = est.pose
        along, cross, e_h = self._line.errors(pose)
        self._last = (along, cross, e_h)
        if along >= self._travel:
            return 0.0, 0.0, True
        v = self.cfg.limits.v_capture
        omega = self._line.omega(e_h, cross, v)
        return v, omega, False

    def tracking_error(self) -> tuple[float, float, float]:
        return self._last


# --------------------------------------------------------------------------------------- PushController
@dataclass(frozen=True)
class PushStatus:
    along_mm: float
    cross_mm: float
    heading_err_rad: float
    lost_cube: bool


class PushController:
    """Pushes the cube along line L = (p0 -> p1).  Controls the CUBE's position (rover pose plus
    `config.contact_distance` along heading, optionally fused with fresh vision), not just the rover:
    cross-track error `e_y` is measured on that cube estimate, heading error `e_h` on the rover (cube
    orientation is not reliably observable -- interpretation_v0 C4).

    Control law: omega = -(K_h*e_h + K_hd*e_h') - (K_y*e_y + K_yd*e_y'), all terms saturated and rate
    terms taken analytically rather than by differentiating noisy positions:
      * e_h' = est.omega  -- the EKF already fuses this from vision + the motion model, so
        finite-differencing our own heading error would just reinject noise the estimator already
        filtered out.
      * e_y' = est.v*sin(e_h) + contact_distance*est.omega*cos(e_h)  -- exact kinematic derivative of a
        point rigidly offset `contact_distance` ahead of the rover (first term: translation of the
        rover; second: the offset arm sweeping sideways as the rover yaws). Cheap, analytic, and immune
        to the ~2-5 mm/frame vision jitter that a raw d(e_y)/dt would amplify by 1/dt.

    Push-stability / curvature limit.  A paddle keeps the cube captured through geometric contact plus a
    few mm of slack (G5), not a rigid joint: if the rover yaws quickly, the contact point (the arm above)
    sweeps sideways at `omega*contact_distance`, and once that sideways sweep outruns what the slack can
    absorb the cube slips or is left behind. Bounding the path curvature kappa = omega/v <= push_curv_max
    keeps the turn radius >= 1/push_curv_max (400 mm with the default), independent of speed, which is
    the "does not turn faster than the cube can follow" constraint. omega is clamped to
    push_curv_max * v on every tick in addition to the analytic-PD saturation.

    Cross-track deadband: the cube has lateral slack inside the channel (G5: capture tolerance up to
    ~4-17 mm depending on orientation), so a small dead zone on e_y before it drives omega prevents
    chattering the rover heading to chase sub-slack cube wobble.

    Speed schedule: capture speed floor while still moving (never below `v_capture`, which clears the
    motor deadband) ramped up (accel-limited) to `v_push` cruise, dropped to `v_push_slow` inside
    `push_slow_zone` mm of the goal, then an abrupt stop. The stop is triggered predictively: once the
    cube's estimated along-track position PLUS the extra distance it will coast during the ~0.1 s motor
    +command lag (`v * push_stop_lag_s`) reaches the goal, so the realised stopping position -- not the
    command -- lands at p1.
    """

    def __init__(self, cfg: Config = DEFAULT, params: NavParams = NavParams()):
        self.cfg = cfg
        self.params = params
        self.line: _Line | None = None
        self._length = 0.0
        self._status = PushStatus(0.0, 0.0, 0.0, False)

    def reset(self, p0: tuple[float, float], p1: tuple[float, float]) -> None:
        self.line = _Line.through(p0, p1, reverse=False)
        self._length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        self._status = PushStatus(0.0, 0.0, 0.0, False)

    def _cube_estimate(self, est: RoverEstimate, cube_xy: tuple[float, float] | None,
                        cube_age_s: float) -> tuple[float, float]:
        pose = est.pose
        cd = self.cfg.contact_distance
        dead_reckon = (pose.x + cd * math.cos(pose.theta), pose.y + cd * math.sin(pose.theta))
        if cube_xy is not None and cube_age_s <= self.cfg.telemetry.cube_fresh_s:
            w = self.params.push_vision_weight
            return (w * cube_xy[0] + (1 - w) * dead_reckon[0],
                    w * cube_xy[1] + (1 - w) * dead_reckon[1])
        return dead_reckon

    def _lost_cube(self, est: RoverEstimate, cube_xy: tuple[float, float] | None, cube_age_s: float) -> bool:
        if cube_xy is None or cube_age_s > self.cfg.telemetry.cube_fresh_s:
            return False
        pose = est.pose
        along_local, lat_local = pose.to_local(cube_xy[0], cube_xy[1])
        gap = abs(along_local - self.cfg.contact_distance)
        return abs(lat_local) > self.params.push_lost_lateral or gap > self.params.push_lost_gap

    def step(self, est: RoverEstimate, cube_xy: tuple[float, float] | None, cube_age_s: float,
              t: float) -> tuple[float, float, bool, PushStatus]:
        assert self.line is not None
        pose = est.pose
        cube_x, cube_y = self._cube_estimate(est, cube_xy, cube_age_s)
        cube_along, cube_cross, _ = self.line.errors(cube_x, cube_y, pose.theta)
        _, _, e_h = self.line.errors(pose.x, pose.y, pose.theta)
        lost = self._lost_cube(est, cube_xy, cube_age_s)
        self._status = PushStatus(cube_along, cube_cross, e_h, lost)

        remaining = self._length - cube_along
        predicted_stop = cube_along + est.v * self.params.push_stop_lag_s
        if predicted_stop >= self._length - self.params.push_stop_tol:
            return 0.0, 0.0, True, self._status

        v_cap = self.cfg.limits.v_push if remaining > self.params.push_slow_zone else self.cfg.limits.v_push_slow
        v_ramp = _decel_speed(max(cube_along, 0.0), self.params.accel, v_cap)
        v = max(self.cfg.limits.v_capture, v_ramp)

        e_y = cube_cross
        if abs(e_y) <= self.params.push_cross_deadband:
            e_y_eff = 0.0
        else:
            e_y_eff = e_y - math.copysign(self.params.push_cross_deadband, e_y)

        e_h_dot = est.omega
        e_y_dot = est.v * math.sin(e_h) + self.cfg.contact_distance * est.omega * math.cos(e_h)
        k_h, k_y = line_gains(v, self.params.push_wn, self.params.push_zeta, self.params.line_v_gain_floor)
        omega = -(k_h * e_h + self.params.push_hdot_gain * e_h_dot) \
                - (k_y * e_y_eff + self.params.push_ydot_gain * e_y_dot)

        omega_cap = min(self.cfg.limits.w_fine, self.params.push_curv_max * max(v, 1.0))
        omega = max(-omega_cap, min(omega_cap, omega))
        return v, omega, False, self._status

    def status(self) -> PushStatus:
        return self._status


# --------------------------------------------------------------------------------------- RetreatController
class RetreatController:
    """Reverses straight `distance_mm` along the heading held at reset/first tick, with heading-hold
    (reusing the same line-tracking law so small drift is corrected, not just heading error)."""

    def __init__(self, cfg: Config = DEFAULT, params: NavParams = NavParams()):
        self.cfg = cfg
        self.params = params
        self._line = _LineHold(params, params.line_wn, params.line_zeta, cfg.limits.w_fine)
        self._distance = 0.0
        self._armed = False

    def reset(self, distance_mm: float) -> None:
        self._distance = distance_mm
        self._armed = False

    def step(self, est: RoverEstimate, t: float) -> tuple[float, float, bool]:
        pose = est.pose
        if not self._armed:
            # `_Line.path_dir` is the geometric direction of TRAVEL, not the nose heading -- since we
            # drive backward, travel direction is the reverse of the heading being held.
            self._line.reset(_Line((pose.x, pose.y), wrap(pose.theta + math.pi), reverse=True))
            self._armed = True

        along, cross, e_h = self._line.errors(pose)
        remaining = self._distance - along
        if remaining <= self.params.line_along_tol:
            return 0.0, 0.0, True
        v_cap = self.cfg.limits.v_retreat
        v_mag = max(self.params.v_min_moving,
                    min(_decel_speed(remaining, self.params.accel, v_cap),
                        _decel_speed(max(along, 0.0), self.params.accel, v_cap)))
        omega = self._line.omega(e_h, cross, v_mag)
        return -v_mag, omega, False
