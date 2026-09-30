"""Regression tests for A07 simulator judging gaps."""
from __future__ import annotations

from dataclasses import asdict, replace

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.simulation.monte_carlo import aggregate
from rover_strategy.simulation.physics import ContactParams, MotorParams, PhysicsWorld, SimCube, SimRover
from rover_strategy.simulation.runner import RunResult
from rover_strategy.world import WheelCommand


def motor(**changes) -> MotorParams:
    return replace(MotorParams(), tau_s=0.001, latency_s=0.0, speed_noise_std=0.0,
                   max_accel_mm_s2=10_000.0, **changes)


def test_rotation_with_cube_is_edge_triggered_and_not_a_hard_failure():
    rover = SimRover(10, 200.0, 200.0, 0.0, motor())
    cube = SimCube("red", rover.x + C.rover.x_front_plate + C.cube.half, rover.y, 0.0)
    world = PhysicsWorld(C, [rover], [cube], ContactParams(), np.random.default_rng(0))
    world.set_command(10, WheelCommand(-40.0, 40.0), 0.0)

    world.step(0.02)
    world.step(0.02)

    assert world.counts["rotations_with_cube"] == 1
    events = [event for event in world.events if event.kind == "rotation_with_cube"]
    assert len(events) == 1
    assert events[0].data["rover"] == 10 and events[0].data["cube"] == "red"

    result = RunResult(1, "test", outcome="success", rotations_with_cube=1)
    summary = aggregate([asdict(result)])
    assert summary["rotations_with_cube_total"] == 1
    assert summary["rotations_with_cube_runs"] == 1
    assert result.outcome == "success"


def test_contact_rejection_uses_actual_forward_velocity():
    rover = SimRover(10, 200.0, 200.0, 0.0, motor())
    cube = SimCube("red", rover.x + C.rover.x_front_plate + C.cube.half - 1.0, rover.y, 0.0)
    world = PhysicsWorld(C, [rover], [cube], ContactParams(), np.random.default_rng(0))
    world.set_target(10, "red")

    # The command is reversing, but the lagged wheels are still driving forward:
    # a commanded-direction gate would incorrectly reject this causal contact.
    rover.cmd_left = rover.cmd_right = -40.0
    rover.wl = rover.wr = 40.0
    before = cube.x
    world._resolve_contacts()
    assert cube.x > before

    # Conversely, a forward command must not push while the actual wheels are
    # still reversing away from the plate.
    cube.x = before
    rover.cmd_left = rover.cmd_right = 40.0
    rover.wl = rover.wr = -40.0
    world._resolve_contacts()
    assert cube.x == pytest.approx(before)


def test_transport_credit_requires_plate_contact_and_actual_forward_speed():
    rover = SimRover(10, 200.0, 200.0, 0.0, motor())
    plate_cube = SimCube("red", rover.x + C.rover.x_front_plate + C.cube.half - 1.0, rover.y, 0.0)
    paddle_cube = SimCube(
        "blue", rover.x + C.rover.x_front_plate + C.cube.half + 2.0,
        rover.y + C.rover.inner_half_width + C.cube.half - 3.0, 0.0,
    )
    world = PhysicsWorld(C, [rover], [plate_cube, paddle_cube], ContactParams(), np.random.default_rng(0))
    world.set_target(10, "red")
    rover.cmd_left = rover.cmd_right = 0.0
    rover.wl = rover.wr = 40.0
    before = {color: world.cube_pose(color)[:2] for color in world.cubes}
    touched = world._resolve_contacts()
    world._update_contact_records(touched, before)
    assert world.cubes["red"].engaged_displacement[10] > 0.0
    assert world.cubes["blue"].engaged_displacement == {}

    slow_rover = SimRover(10, 200.0, 200.0, 0.0, motor())
    slow_cube = SimCube("red", slow_rover.x + C.rover.x_front_plate + C.cube.half - 1.0, slow_rover.y, 0.0)
    slow_world = PhysicsWorld(C, [slow_rover], [slow_cube], ContactParams(), np.random.default_rng(0))
    slow_world.set_target(10, "red")
    slow_rover.wl = slow_rover.wr = 5.0
    before = {"red": slow_world.cube_pose("red")[:2]}
    touched = slow_world._resolve_contacts()
    slow_world._update_contact_records(touched, before)
    assert slow_cube.engaged_displacement == {}
