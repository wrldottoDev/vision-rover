from __future__ import annotations

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.simulation import scenarios as SC
from rover_strategy.simulation.physics import (
    ContactParams,
    NOMINAL_CURVE,
    R10_LEFT_CURVE,
    R10_RIGHT_CURVE,
    R11_LEFT_CURVE,
    R11_RIGHT_CURVE,
    THROTTLE_KNOTS,
    MotorParams,
    PhysicsWorld,
    SimRover,
)


def test_measured_curves_reproduce_wheel_ratios_at_the_measured_knots():
    r10 = MotorParams(
        curve_left=R10_LEFT_CURVE,
        curve_right=R10_RIGHT_CURVE,
        full_speed_left_factor=57.5 / 58.0,
    )
    r11 = MotorParams(
        curve_left=R11_LEFT_CURVE,
        curve_right=R11_RIGHT_CURVE,
        full_speed_left_factor=54.0 / 58.0,
    )

    r10_ratios = [r10.speed_at_throttle(t, "left") / r10.speed_at_throttle(t, "right")
                  for t in THROTTLE_KNOTS[1:]]
    r11_ratios = [r11.speed_at_throttle(t, "left") / r11.speed_at_throttle(t, "right")
                  for t in THROTTLE_KNOTS[1:]]
    assert r10_ratios == pytest.approx([39.6 / 40.0, 49.5 / 50.0, 54.0 / 54.2, 57.5 / 58.0])
    assert r11_ratios[1:] == pytest.approx([0.907, 0.926, 0.931], abs=0.001)


def test_nominal_inverse_is_saturating_and_true_rover_curve_is_not_used():
    nominal = MotorParams()
    assert nominal.throttle_for_command(180.0 * NOMINAL_CURVE[2]) == pytest.approx(0.5)
    assert nominal.throttle_for_command(240.0) == pytest.approx(1.0)
    assert nominal.throttle_for_command(-180.0) == pytest.approx(-1.0)

    r11 = MotorParams(curve_left=R11_LEFT_CURVE, curve_right=R11_RIGHT_CURVE,
                      full_speed_left_factor=54.0 / 58.0)
    # A 180 mm/s command means nominal full throttle; the R11 left wheel is
    # intentionally slower because inversion is not made from the true curve.
    assert r11.speed_at_throttle(r11.throttle_for_command(180.0), "left") == pytest.approx(
        180.0 * 54.0 / 58.0
    )
    assert r11.speed_at_throttle(r11.throttle_for_command(180.0), "right") == pytest.approx(180.0)


def test_every_generated_family_keeps_distinct_measured_models_for_r10_and_r11():
    for family in SC.FAMILIES:
        scenario = SC.generate(123, family)
        motors = {rover.id: rover.motor for rover in scenario.rovers}
        assert set(motors) == {10, 11}
        assert motors[10].full_speed_left_factor == pytest.approx(57.5 / 58.0)
        assert motors[11].full_speed_left_factor == pytest.approx(54.0 / 58.0)
        r10_ratio = motors[10].speed_at_throttle(0.7, "left") / motors[10].speed_at_throttle(0.7, "right")
        r11_ratio = motors[11].speed_at_throttle(0.7, "left") / motors[11].speed_at_throttle(0.7, "right")
        assert r10_ratio > 0.97
        assert r11_ratio == pytest.approx(0.926, abs=0.04)


def test_r11_repeated_run_variation_is_seeded_and_hard_family_is_wider():
    ordinary = [SC._motor_params(np.random.default_rng(seed), 11, False)
                .speed_at_throttle(0.5, "left") for seed in range(20)]
    hard = [SC._motor_params(np.random.default_rng(seed), 11, True)
            .speed_at_throttle(0.5, "left") for seed in range(20)]
    assert len(set(ordinary)) > 1
    assert np.std(hard) > np.std(ordinary)


def test_cube_load_reduces_realized_wheel_speed_by_the_measured_four_percent():
    motor = MotorParams(tau_s=1e-6, latency_s=0.0, speed_noise_std=0.0,
                        max_accel_mm_s2=1e9)
    unloaded = PhysicsWorld(C, [SimRover(10, 400, 400, 0, motor)], [], ContactParams(),
                            np.random.default_rng(1))
    loaded = PhysicsWorld(C, [SimRover(10, 400, 400, 0, motor)], [], ContactParams(),
                          np.random.default_rng(1))
    loaded._rover_load[10] = 1
    for world in (unloaded, loaded):
        world.rovers[10].cmd_left = world.rovers[10].cmd_right = 100.0
        world._advance_rover(world.rovers[10], 0.1)
    assert loaded.rovers[10].wl / unloaded.rovers[10].wl == pytest.approx(0.96)
