"""Tests for rover_strategy.simulation (physics, sensors, scenarios).

Focused on properties that would FAIL if the contact model, event counting, sensor
noise model or scenario generator were wrong -- not smoke tests.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.world import WheelCommand
from rover_strategy.simulation.physics import ContactParams, MotorParams, PhysicsWorld, SimCube, SimRover
from rover_strategy.simulation.sensors import SensorParams, VisionEmulator
from rover_strategy.simulation import scenarios as SC

_SCHEMA_DIR = (
    Path(__file__).resolve().parents[1]
    / "_source" / "Rover Vision Artificial"
    / "Vision-Rover-Challenge-main_unz" / "Vision-Rover-Challenge-main"
    / "vision-system" / "contrato"
)
if str(_SCHEMA_DIR) not in sys.path:
    sys.path.insert(0, str(_SCHEMA_DIR))
import schema  # noqa: E402  (official contract validator, pure stdlib)


def _still_motor(**kw) -> MotorParams:
    """A motor with no lag/latency/noise, for deterministic contact-model tests."""
    return MotorParams(tau_s=0.02, speed_noise_std=0.0, latency_s=0.0, **kw)


# --------------------------------------------------------------------------- #
# contact model
# --------------------------------------------------------------------------- #


def test_flat_plate_push_aligns_rotated_cube():
    rng = np.random.default_rng(0)
    rover = SimRover(id=10, x=0.0, y=100.0, theta=0.0, motor=_still_motor())
    cube = SimCube(color="red", x=200.0, y=100.0, alpha=math.radians(30.0))
    world = PhysicsWorld(C, [rover], [cube], ContactParams(), rng)
    world.set_target(10, "red")
    cmd = WheelCommand(70.0, 70.0)
    dt = 0.01
    x0 = cube.x
    first_contact_x = None
    for _ in range(20000):
        world.set_command(10, cmd, world.t)
        world.step(dt)
        if first_contact_x is None and abs(cube.x - x0) > 1e-6:
            first_contact_x = cube.x
        if first_contact_x is not None and (cube.x - first_contact_x) > 100.0:
            break
    assert first_contact_x is not None, "cube was never touched"
    # aligned to the plate (0 mod 90deg) within a few degrees after ~100mm of push
    err = math.degrees(cube.alpha) % 90.0
    err = min(err, 90.0 - err)
    assert err < 3.0, f"cube did not align to the pushing plate: {err} deg off"


def test_paddle_tip_corner_strike_is_bounded_and_finite():
    """A glancing paddle-tip hit on a cube corner must translate/rotate the cube
    plausibly, without any single-step teleport (no tunnelling / no explosion), and
    with no NaN/inf state -- momentum-free (pure geometric quasi-statics)."""
    rng = np.random.default_rng(0)
    rover = SimRover(id=10, x=0.0, y=0.0, theta=0.0, motor=_still_motor())
    cube = SimCube(color="red", x=250.0, y=75.0, alpha=0.0)  # offset so only a paddle clips a corner
    world = PhysicsWorld(C, [rover], [cube], ContactParams(), rng)
    world.set_target(10, "red")
    cmd = WheelCommand(70.0, 70.0)
    dt = 0.01
    max_step_disp = 0.0
    prev = (cube.x, cube.y)
    for _ in range(6000):
        world.set_command(10, cmd, world.t)
        world.step(dt)
        assert math.isfinite(cube.x) and math.isfinite(cube.y) and math.isfinite(cube.alpha)
        disp = math.hypot(cube.x - prev[0], cube.y - prev[1])
        max_step_disp = max(max_step_disp, disp)
        prev = (cube.x, cube.y)
        if rover.x > 400.0:
            break
    # one step moves the rover ~0.7mm at this speed/dt; a sane per-step cube nudge
    # must stay of that same order of magnitude, not jump centimetres (tunnelling).
    assert max_step_disp < 5.0, f"single-step cube displacement too large: {max_step_disp} mm"
    # got moved out of the rover's way, not left overlapping it forever
    env = world.footprint.envelope(rover.x, rover.y, rover.theta)
    from rover_strategy.geometry import shapes as S
    cube_poly = S.square(cube.x, cube.y, C.cube.side, cube.alpha)
    assert not S.overlap(env, cube_poly)


def test_deep_initial_overlap_does_not_explode():
    """Adversarial hand-built state (paddle rail embedded deep inside a cube along one
    axis -- a legitimate large SAT-MTV depth, see ContactParams.max_correction_mm):
    the solver must converge to a stable, finite, non-overlapping state, not diverge."""
    rng = np.random.default_rng(0)
    rover = SimRover(id=10, x=0.0, y=0.0, theta=0.0, motor=_still_motor())
    cube = SimCube(color="red", x=90.0, y=55.0, alpha=0.0)
    world = PhysicsWorld(C, [rover], [cube], ContactParams(), rng)
    world.set_target(10, "red")
    stop = WheelCommand(0.0, 0.0)
    positions = []
    for _ in range(150):
        world.set_command(10, stop, world.t)
        world.step(0.01)
        assert math.isfinite(cube.x) and math.isfinite(cube.y)
        positions.append((cube.x, cube.y))
    # settles down (last 50 steps barely move) instead of diverging
    tail = positions[-50:]
    spread = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in tail for b in tail)
    assert spread < 1.0


def test_cube_in_channel_moves_with_rover_and_reversing_leaves_it():
    rng = np.random.default_rng(0)
    rover = SimRover(id=10, x=0.0, y=100.0, theta=0.0, motor=_still_motor())
    cube = SimCube(color="red", x=C.contact_distance, y=100.0, alpha=0.0)
    world = PhysicsWorld(C, [rover], [cube], ContactParams(), rng)
    world.set_target(10, "red")
    dt = 0.01
    fwd = WheelCommand(50.0, 50.0)
    for _ in range(500):
        world.set_command(10, fwd, world.t)
        world.step(dt)
    gap = cube.x - rover.x
    assert abs(gap - C.contact_distance) < 1.0     # cube stays flush against the plate
    assert rover.x > 200.0                          # actually travelled (moved WITH the rover)

    cube_x_before_reverse = cube.x
    back = WheelCommand(-50.0, -50.0)
    for _ in range(500):
        world.set_command(10, back, world.t)
        world.step(dt)
    assert rover.x < cube_x_before_reverse - 200.0   # rover backed away substantially
    assert cube.x == pytest.approx(cube_x_before_reverse, abs=1e-9)  # cube untouched


# --------------------------------------------------------------------------- #
# rover-rover / board events
# --------------------------------------------------------------------------- #


def test_rover_rover_collision_counted_and_stops_both():
    rng = np.random.default_rng(0)
    r1 = SimRover(id=10, x=0.0, y=0.0, theta=0.0, motor=_still_motor())
    r2 = SimRover(id=11, x=200.0, y=0.0, theta=math.pi, motor=_still_motor())
    world = PhysicsWorld(C, [r1, r2], [], ContactParams(), rng)
    cmd = WheelCommand(60.0, 60.0)
    dt = 0.01
    for _ in range(1000):
        world.set_command(10, cmd, world.t)
        world.set_command(11, cmd, world.t)
        world.step(dt)
    assert world.counts["collision"] == 1     # edge-triggered: one sustained contact, not one per step
    assert r1.x == 0.0 and r2.x == 200.0       # rejected step: never actually advanced into each other


def test_rover_exit_and_fell_events():
    rng = np.random.default_rng(0)
    r = SimRover(id=10, x=C.board.width - 10.0, y=C.board.height / 2.0, theta=0.0, motor=_still_motor())
    world = PhysicsWorld(C, [r], [], ContactParams(), rng)
    cmd = WheelCommand(60.0, 60.0)
    dt = 0.01
    for _ in range(2000):
        world.set_command(10, cmd, world.t)
        world.step(dt)
    assert world.counts["rover_exit"] >= 1
    assert world.counts["rover_fell"] >= 1


def test_cube_exit_event():
    rng = np.random.default_rng(0)
    r = SimRover(id=10, x=C.board.width - 100.0, y=C.board.height / 2.0, theta=0.0, motor=_still_motor())
    cube = SimCube("red", C.board.width - 100.0 + C.contact_distance, C.board.height / 2.0, 0.0)
    world = PhysicsWorld(C, [r], [cube], ContactParams(), rng)
    world.set_target(10, "red")
    cmd = WheelCommand(60.0, 60.0)
    dt = 0.01
    for _ in range(2000):
        world.set_command(10, cmd, world.t)
        world.step(dt)
    assert world.counts["cube_exit"] >= 1


def test_non_target_contact_counted():
    rng = np.random.default_rng(0)
    r = SimRover(id=10, x=0.0, y=100.0, theta=0.0, motor=_still_motor())
    cube = SimCube("blue", 200.0, 100.0, 0.0)   # rover targets 'red', touches 'blue'
    world = PhysicsWorld(C, [r], [cube], ContactParams(), rng)
    world.set_target(10, "red")
    cmd = WheelCommand(60.0, 60.0)
    dt = 0.01
    for _ in range(1500):
        world.set_command(10, cmd, world.t)
        world.step(dt)
    assert world.counts["non_target_contact"] == 1
    assert 10 in cube.touched_by


# --------------------------------------------------------------------------- #
# sensors
# --------------------------------------------------------------------------- #


def _run_sensor_sim(rng_phys, rng_sensor, sensor_params, steps=4000, dt=0.01, drive=True):
    motor = MotorParams()
    r1 = SimRover(id=10, x=100.0, y=100.0, theta=0.2, motor=motor)
    r2 = SimRover(id=11, x=700.0, y=700.0, theta=1.0, motor=motor)
    cubes = [SimCube("red", 400.0, 400.0, 0.0), SimCube("green", 300.0, 500.0, 0.3), SimCube("blue", 500.0, 300.0, 0.6)]
    world = PhysicsWorld(C, [r1, r2], cubes, ContactParams(), rng_phys)
    depots = {"red": (40.5, 40.5), "green": (40.5, 2.5), "blue": (2.5, 40.5)}
    emu = VisionEmulator(sensor_params, rng_sensor, depots, (2.5, 2.5))
    emu.set_phase("RUNNING")
    cmd = WheelCommand(45.0, 52.0) if drive else WheelCommand(0.0, 0.0)
    messages = []
    for _ in range(steps):
        world.set_command(10, cmd, world.t)
        world.set_command(11, cmd, world.t)
        world.step(dt)
        emu.capture(world, world.t)
        messages.extend(emu.poll(world.t))
    return world, emu, messages


def test_sensor_messages_validate_against_official_schema():
    rng_phys = np.random.default_rng(0)
    rng_sensor = np.random.default_rng(1)
    _, _, messages = _run_sensor_sim(rng_phys, rng_sensor, SensorParams())
    assert len(messages) > 100
    for m in messages:
        err = schema.validate_message(m)
        assert err is None, err


def test_sensor_seq_gaps_from_drops_and_monotonic_delivery():
    rng_phys = np.random.default_rng(0)
    rng_sensor = np.random.default_rng(1)
    _, _, messages = _run_sensor_sim(rng_phys, rng_sensor, SensorParams(frame_drop_prob=0.2))
    seqs = [m["seq"] for m in messages]
    assert seqs == sorted(seqs)                      # never reordered (single ordered channel)
    assert any(b - a > 1 for a, b in zip(seqs, seqs[1:])), "expected some seq gaps from dropped frames"


def test_sensor_occlusion_freezes_position_and_grows_age():
    """A cube directly under a stationary rover's chassis must be reported as fully
    covered: last known position kept, age_ms growing (contract s6.2 semantics)."""
    rng_phys = np.random.default_rng(0)
    motor = MotorParams()
    r = SimRover(id=10, x=400.0, y=400.0, theta=0.0, motor=motor)
    cube = SimCube("red", 400.0, 400.0, 0.0)   # fully inside the chassis body rect
    world = PhysicsWorld(C, [r], [cube], ContactParams(), rng_phys)   # never step(): keep this exact state
    depots = {"red": (40.5, 40.5), "green": (40.5, 2.5), "blue": (2.5, 40.5)}
    sp = SensorParams(rover_loss_prob=0.0, rover_loss_burst_prob=0.0, frame_drop_prob=0.0,
                      outlier_prob=0.0, latency_mean_s=0.0, latency_jitter_s=0.0,
                      cube_pos_noise_std_mm=0.0, cube_pos_bias_mm=0.0)
    emu = VisionEmulator(sp, np.random.default_rng(1), depots, (2.5, 2.5))
    emu.set_phase("RUNNING")
    frac, _ = emu._coverage(cube, world)
    assert frac > 0.99   # fully covered by the body rect

    # A stale track is only available after one real detection.  Establish it
    # from an unobscured capture, then move the rover over the cube.
    r.x = 300.0
    emu.capture(world, 0.0)
    emu.poll(0.0)
    r.x = 400.0
    last = None
    for t in (0.05, 0.1, 0.2, 0.4, 0.8, 1.2):
        emu.capture(world, t)
        for m in emu.poll(t):
            last = m
    red = next(c for c in last["cubes"] if c["color"] == "red")
    assert red["age_ms"] > 500
    # frozen at the true internal position (400,400)mm -> cells (20, 23) via Grid
    assert abs(red["col"] - 20.0) < 0.05 and abs(red["row"] - 23.0) < 0.05


def test_sensor_rover_loss_burst_grows_age_and_recovers():
    """Force one burst (prob=1.0), then stop starting new ones and check the age,
    which was frozen mid-burst, drops back to ~0 once the burst window elapses."""
    rng_sensor = np.random.default_rng(7)
    motor = MotorParams()
    r = SimRover(id=10, x=400.0, y=400.0, theta=0.0, motor=motor)
    world = PhysicsWorld(C, [r], [], ContactParams(), np.random.default_rng(0))
    depots = {"red": (40.5, 40.5), "green": (40.5, 2.5), "blue": (2.5, 40.5)}
    emu = VisionEmulator(
        SensorParams(rover_loss_prob=0.0, rover_loss_burst_prob=0.0, rover_loss_burst_s=0.5,
                     frame_drop_prob=0.0, outlier_prob=0.0,
                     latency_mean_s=0.0, latency_jitter_s=0.0),
        rng_sensor, depots, (2.5, 2.5),
    )
    emu.set_phase("RUNNING")
    emu.capture(world, 0.0)    # establish the initial real track
    emu.poll(0.0)
    emu.params = SensorParams(rover_loss_prob=0.0, rover_loss_burst_prob=1.0,
                              rover_loss_burst_s=0.5, frame_drop_prob=0.0,
                              outlier_prob=0.0, latency_mean_s=0.0,
                              latency_jitter_s=0.0)
    emu.capture(world, 0.05)   # guaranteed to start a burst (prob=1.0)
    (m0,) = emu.poll(0.05)
    age0 = next(x for x in m0["rovers"] if x["id"] == 10)["age_ms"]
    assert age0 > 0             # the 30 Hz capture clock has advanced since t=0

    emu.params = SensorParams(rover_loss_prob=0.0, rover_loss_burst_prob=0.0,
                               frame_drop_prob=0.0, outlier_prob=0.0)
    ages = []
    for t in (0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 0.9, 1.1):
        emu.capture(world, t)
        for m in emu.poll(t):
            ages.append(next(x for x in m["rovers"] if x["id"] == 10)["age_ms"])
    assert max(ages) > 100         # burst kept it stale for a while
    assert ages[-1] == 0           # ... and it recovered once the burst window passed


# --------------------------------------------------------------------------- #
# scenarios
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("family", SC.FAMILIES)
def test_scenario_validity_over_many_seeds(family):
    for seed in range(500):
        sc = SC.generate(seed, family)
        problems = SC.validate(sc, C)
        assert not problems, f"seed={seed} family={family}: {problems}"


@pytest.mark.parametrize("family", SC.FAMILIES)
def test_scenario_determinism(family):
    a = SC.generate(123, family)
    b = SC.generate(123, family)
    assert a.to_json() == b.to_json()


def test_scenario_different_seeds_differ():
    a = SC.generate(1, "arbitrary")
    b = SC.generate(2, "arbitrary")
    assert a.to_json() != b.to_json()


def test_scenario_json_roundtrip():
    sc = SC.generate(42, "hard")
    back = SC.Scenario.from_json(sc.to_json())
    assert back.to_json() == sc.to_json()


def test_scenario_depot_colors_cover_cube_colors():
    for seed in range(20):
        sc = SC.generate(seed, "arbitrary")
        cube_colors = {c.color for c in sc.cubes}
        assert cube_colors <= set(sc.depots.keys())
        assert set(sc.depots.keys()) == set(SC.COLORS)


def test_scenario_instantiate_builds_runnable_world():
    sc = SC.generate(5, "start_zone")
    world, emu = SC.instantiate(sc, C)
    assert isinstance(world, PhysicsWorld) and isinstance(emu, VisionEmulator)
    assert set(world.rovers.keys()) == {r.id for r in sc.rovers}
    assert set(world.cubes.keys()) == {c.color for c in sc.cubes}
    for rid in world.rovers:
        world.set_command(rid, WheelCommand(20.0, 20.0), world.t)
    world.step(0.01)
    emu.set_phase("RUNNING")
    emu.capture(world, world.t)
    # nothing asserted on poll() timing here (latency), just that it doesn't blow up
    emu.poll(world.t + 1.0)


def test_scenario_cube_count_is_2_or_3():
    counts = {len(SC.generate(seed, "hard").cubes) for seed in range(60)}
    assert counts <= {2, 3}
    assert counts == {2, 3}   # both should appear across 60 seeds


# --------------------------------------------------------------------------- #
# performance
# --------------------------------------------------------------------------- #


def test_speed_benchmark():
    rng = np.random.default_rng(0)
    motor = MotorParams()
    rovers = [SimRover(id=10, x=100.0, y=100.0, theta=0.3, motor=motor),
              SimRover(id=11, x=700.0, y=700.0, theta=2.0, motor=motor)]
    cubes = [SimCube("red", 400.0, 400.0, 0.1), SimCube("green", 300.0, 500.0, 0.4), SimCube("blue", 500.0, 300.0, 0.9)]
    world = PhysicsWorld(C, rovers, cubes, ContactParams(), rng)
    dt = 0.01
    n = 20000
    cmd_a, cmd_b = WheelCommand(60.0, 65.0), WheelCommand(-40.0, 50.0)
    t0 = time.perf_counter()
    for _ in range(n):
        world.set_command(10, cmd_a, world.t)
        world.set_command(11, cmd_b, world.t)
        world.step(dt)
    wall = time.perf_counter() - t0
    factor = (n * dt) / wall
    print(f"\n[bench] 2 rovers + 3 cubes, dt=0.01s: {factor:.1f}x real time ({n} steps in {wall:.2f}s)")
    assert factor >= 15.0, f"too slow for 10k-run Monte Carlo: {factor:.1f}x real time"
