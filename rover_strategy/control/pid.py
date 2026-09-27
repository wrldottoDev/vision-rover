"""Small, generic PID controller.

Used for every closed loop in controllers.py (heading hold, cross-track, etc.). Features:

- Output saturation (`out_min`/`out_max`).
- Conditional-integration anti-windup: the integral is only advanced when doing so would NOT push
  an already-saturated output further into saturation.  This is simpler and cheaper than back-
  calculation and is exact for our use (no external actuator model available at this layer).
- Derivative-on-measurement with a first-order low-pass filter: differentiating the measurement
  instead of the error avoids a derivative kick when the setpoint jumps, and the low-pass filter
  (time constant `d_tau`) keeps vision/EKF pose noise from being amplified by the 1/dt of a raw
  finite difference.

Units are whatever the caller uses; the controller itself is dimensionless plumbing.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    out_min: float = float("-inf")
    out_max: float = float("inf")
    d_tau: float = 0.05   # s, derivative low-pass filter time constant. TUNED: below the ~0.1 s motor
                          # lag so it filters sensor noise without adding much extra phase lag.

    _integral: float = field(default=0.0, init=False, repr=False)
    _prev_meas: float | None = field(default=None, init=False, repr=False)
    _d_filt: float = field(default=0.0, init=False, repr=False)

    def reset(self) -> None:
        """Clear all internal state (call whenever the setpoint context changes discontinuously)."""
        self._integral = 0.0
        self._prev_meas = None
        self._d_filt = 0.0

    def step(self, setpoint: float, measurement: float, dt: float) -> float:
        dt = max(dt, 1e-9)
        error = setpoint - measurement

        # Derivative on measurement, low-pass filtered.
        d_meas = 0.0 if self._prev_meas is None else (measurement - self._prev_meas) / dt
        alpha = dt / (self.d_tau + dt)
        self._d_filt += alpha * (d_meas - self._d_filt)
        self._prev_meas = measurement

        # Conditional-integration anti-windup: evaluate saturation using the OLD integral, then
        # only fold this step's contribution in if that doesn't drive further into saturation.
        u_before = self.kp * error + self.ki * self._integral - self.kd * self._d_filt
        pushing_high = u_before >= self.out_max and error > 0.0
        pushing_low = u_before <= self.out_min and error < 0.0
        if not (pushing_high or pushing_low):
            self._integral += error * dt

        u = self.kp * error + self.ki * self._integral - self.kd * self._d_filt
        return max(self.out_min, min(self.out_max, u))


def _demo() -> None:
    """Runnable self-check: a first-order plant (dx/dt = u) should settle at the setpoint, output
    should never exceed saturation, and integral should stop growing once permanently saturated
    (anti-windup)."""
    pid = PID(kp=2.0, ki=1.0, kd=0.1, out_min=-1.0, out_max=1.0)
    x, dt = 0.0, 0.01
    for _ in range(2000):
        u = pid.step(setpoint=5.0, measurement=x, dt=dt)
        assert -1.0 - 1e-9 <= u <= 1.0 + 1e-9
        x += u * dt
    assert abs(x - 5.0) < 0.05, x

    # Anti-windup: setpoint unreachable at this saturation -> integral must stop growing, not diverge.
    pid.reset()
    x = 0.0
    for _ in range(500):
        pid.step(setpoint=1e6, measurement=x, dt=dt)
        x += 1.0 * dt  # plant capped far below setpoint regardless of u
    integral_at_500 = pid._integral
    for _ in range(500):
        pid.step(setpoint=1e6, measurement=x, dt=dt)
        x += 1.0 * dt
    assert abs(pid._integral - integral_at_500) < 1e-6, "integral kept growing while saturated"

    print("pid._demo OK")


if __name__ == "__main__":
    _demo()
