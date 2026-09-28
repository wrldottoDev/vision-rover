"""Gate 8 adversarial fidelity tests, independent of planner performance.

Failures are intentionally unmarked: they are unmet simulator/judge requirements,
not expected failures to hide from CI. Runner tests replace only the supervisor
with a small scripted actor so that scoring defects cannot be blamed on planning.
Sensor geometry fixtures are snapshots and do not advance contact physics.
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import asdict, replace
from types import SimpleNamespace

import numpy as np
import pytest

from rover_strategy.config import DEFAULT as C
from rover_strategy.geometry.zones import DepotZone
from rover_strategy.simulation import runner, scenarios as SC
from rover_strategy.simulation.monte_carlo import aggregate
from rover_strategy.simulation.physics import (
    ContactParams, MotorParams, PhysicsWorld, SimCube, SimRover,
    _clip_convex, _poly_area, _pts, _sat_contact, _square_pts,
)
from rover_strategy.simulation.sensors import SensorParams, VisionEmulator
from rover_strategy.world import STOP, WheelCommand


def motor(**changes):
    return replace(MotorParams(), tau_s=0.01, latency_s=0.0,
                   speed_noise_std=0.0, **changes)


def world(rovers=(), cubes=(), contact=None):
    return PhysicsWorld(C, list(rovers), list(cubes), contact or ContactParams(),
                        np.random.default_rng(123))


def quiet_sensor(**changes):
    params = SensorParams(
        jitter_std_s=0.0, latency_mean_s=0.0, latency_jitter_s=0.0,
        frame_drop_prob=0.0, rover_loss_prob=0.0, rover_loss_burst_prob=0.0,
        marker_offset_mm=0.0, marker_angle_offset_rad=0.0,
        pos_noise_std_mm=0.0, heading_noise_std_rad=0.0,
        outlier_prob=0.0, cube_pos_noise_std_mm=0.0, cube_pos_bias_mm=0.0,
        partial_extra_noise_std_mm=0.0, partial_bias_mm=0.0,
        parallax_enabled=False,
    )
    return replace(params, **changes)


def emulator(**changes):
    return VisionEmulator(quiet_sensor(**changes), np.random.default_rng(456),
                          {"red": (40.5, 2.5), "blue": (2.5, 2.5)}, (2.5, 2.5))


def depth(a, b):
    return _sat_contact(a, b)[0] or 0.0


def cube_poly(c):
    return _square_pts(c.x, c.y, C.cube.side, c.alpha)


def chain(order=("red", "blue", "green"), blocker=False):
    positions = {"red": 277.0, "blue": 337.0, "green": 397.0}
    rovers = [SimRover(10, 200, 430, 0, motor())]
    if blocker:
        rovers.append(SimRover(11, 474, 430, math.pi, motor()))
    w = world(rovers, [SimCube(k, positions[k], 430, 0) for k in order])
    w.set_target(10, "red")
    w.set_command(10, WheelCommand(180, 180), 0)
    w.step(runner.PHYS_DT)
    return w


def test_chain_solver_leaves_no_penetration_at_production_timestep():
    w = chain()
    residual = depth(cube_poly(w.cubes["red"]), cube_poly(w.cubes["blue"]))
    assert residual < 1e-3, f"three-cube chain retains {residual:.6f} mm penetration"


def test_chain_solution_is_independent_of_dictionary_insertion_order():
    a, b = chain(), chain(("green", "blue", "red"))
    assert a.cube_pose("green") == pytest.approx(b.cube_pose("green"), abs=1e-3)


def test_cube_chain_cannot_push_through_a_stationary_second_rover():
    w = chain(blocker=True)
    r = w.rovers[11]
    penetration = max(depth(_pts(p), cube_poly(w.cubes["green"]))
                      for p in w.footprint.parts(r.x, r.y, r.theta))
    assert penetration < 1e-3, f"chain pushed cube {penetration:.4f} mm into rover 11"


def test_chain_contact_propagates_non_target_attribution():
    w = chain()
    assert w.cubes["green"].x > 397  # actual unintended motion, not proximity
    assert w.counts["non_target_contact"] > 0, (
        "rover moved two non-target cubes through its target, without an event"
    )


def test_loaded_open_loop_rover_is_not_identical_to_unloaded_rover():
    unloaded = world([SimRover(10, 200, 430, 0, motor())])
    loaded = world([SimRover(10, 200, 430, 0, motor())],
                   [SimCube("red", 277, 430, 0), SimCube("blue", 337, 430, 0)])
    for w in (unloaded, loaded):
        w.set_command(10, WheelCommand(100, 100), 0)
        for _ in range(100):
            w.step(0.01)
    assert loaded.cubes["blue"].x > 400
    assert loaded.rovers[10].x < unloaded.rovers[10].x - 1e-6, (
        "contact has no motor-load/traction feedback at all"
    )


def test_motor_respects_the_configured_acceleration_assumption():
    m = MotorParams(latency_s=0, speed_noise_std=0)
    w = world([SimRover(10, 430, 430, 0, m)])
    w.set_command(10, WheelCommand(180, 180), 0)
    w.step(0.01)
    acceleration = w.rovers[10].wl / 0.01
    assert acceleration <= C.limits.accel, (
        f"truth acceleration {acceleration} exceeds planner assumption {C.limits.accel}"
    )


def test_command_does_not_act_before_its_latency_has_elapsed():
    m = MotorParams(latency_s=0.05, speed_noise_std=0)
    w = world([SimRover(10, 430, 430, 0, m)])
    w.set_command(10, WheelCommand(180, 180), 0)
    for _ in range(5):
        w.step(0.01)
    assert w.rovers[10].x == pytest.approx(430), (
        "command due at 50 ms was integrated over the preceding 10 ms"
    )


def test_large_step_cannot_tunnel_without_contact_or_rejection():
    """API stress test; runner currently uses 10 ms, not this 2 s step."""
    w = world([SimRover(10, 200, 430, 0, motor())], [SimCube("red", 400, 430, 0)])
    w.set_command(10, WheelCommand(180, 180), 0)
    w.step(2.0)
    assert w.cubes["red"].touched_by, "rover crossed the entire cube without a contact"


@pytest.mark.parametrize("sign", [-1, 1])
def test_limit_surface_rotation_sign_and_linearized_contact_displacement(sign):
    """Control: the cross-product sign and first-order LS formula are correct."""
    c = SimCube("red", 430, 430, 0)
    w = world(cubes=[c])
    p, n, penetration = (400, 430 + sign * 20), (1, 0), 0.01
    w._apply_twist(c, p, n, penetration)
    assert sign * c.alpha < 0
    point_normal_motion = (c.x - 430) - sign * 20 * c.alpha
    assert point_normal_motion == pytest.approx(penetration)


def test_mirrored_paddle_tip_hits_have_opposite_rotation():
    angles = []
    for sign in (-1, 1):
        w = world([SimRover(10, 200, 430, 0, motor())],
                  [SimCube("red", 350, 430 + sign * 75, 0)])
        w.set_command(10, WheelCommand(180, 180), 0)
        # With the production 300 mm/s^2 acceleration cap, contact occurs
        # after roughly 0.35 s; 300 ms encoded the old instantaneous motor.
        for _ in range(40):
            w.step(0.01)
        angles.append(w.cubes["red"].alpha)
    assert abs(angles[0]) > 0.1
    assert angles[0] == pytest.approx(-angles[1], abs=1e-6)


def test_cube_beyond_physical_board_is_irretrievably_out_of_play():
    c = SimCube("red", C.board.width + C.board.physical_margin_mm + 60, 430, 0)
    w = world(cubes=[c])
    w.step(0.01)
    assert not c.in_play, "cube remains a movable, observable floor object after falling"


def test_never_detected_rover_does_not_leak_truth_on_first_frame():
    w = world([SimRover(10, 123, 456, 0.7, motor())])
    e = emulator(rover_loss_prob=1.0)
    e.capture(w, 0)
    msg, = e.poll(0)
    assert msg["rovers"] == [], "unseen marker was published at exact truth with age_ms=0"


def test_never_detected_occluded_cube_does_not_leak_truth():
    w = world([SimRover(10, 430, 430, 0, motor())], [SimCube("red", 430, 430, 0)])
    e = emulator()
    assert e._coverage(w.cubes["red"], w)[0] == pytest.approx(1)
    e.capture(w, 0)
    msg, = e.poll(0)
    assert msg["cubes"] == [], "unseen cube was initialized from ground truth"


def test_sensor_withholds_telemetry_until_first_four_marker_homography():
    w = world(cubes=[SimCube("red", 30, 30, 0), SimCube("blue", 830, 830, 0)])
    e = emulator()
    e.set_phase("RUNNING")
    e.capture(w, 0.0)
    assert e.poll(0.0) == []
    assert e._latest_state is None
    w.cubes["red"].x, w.cubes["red"].y = 300, 300
    w.cubes["blue"].x, w.cubes["blue"].y = 500, 500
    e.capture(w, 0.1)
    assert e._latest_state is not None


def test_sensor_defaults_match_official_capture_and_publication_clocks():
    assert SensorParams().capture_hz == pytest.approx(30.0)
    assert SensorParams().publish_hz == pytest.approx(20.0)


def test_latest_value_wins_for_a_slow_consumer():
    e, w = emulator(), world()
    for t in (0.0, 0.05, 0.10, 0.15, 0.20):
        e.capture(w, t)
    messages = e.poll(1.0)
    assert len(messages) == 1, "CONTRATO 6.3 forbids replaying a backlog to a slow client"
    assert messages[0]["ts_ms"] == 200


def test_two_occluded_corner_markers_freeze_capture_time_but_not_sequence():
    """Documented Gate-1 hazard: cubes cover two in-field black marker quarters.

    Seed a good homography first. The official engine then re-emits its last
    EstadoMundo when fewer than three corners remain usable (sistema.py:288).
    """
    w = world(cubes=[SimCube("red", 300, 430, 0), SimCube("blue", 500, 430, 0)])
    e = emulator()
    e.capture(w, 0)
    before, = e.poll(0)
    w.cubes["red"].x, w.cubes["red"].y = 30, 830
    w.cubes["blue"].x, w.cubes["blue"].y = 830, 830
    e.capture(w, 0.05)
    after, = e.poll(0.05)
    assert after["seq"] > before["seq"]
    assert after["ts_ms"] == before["ts_ms"], "corner occlusion never freezes this emulator"
    assert after["cubes"] == before["cubes"]


def test_occlusion_uses_union_of_both_rover_shadows():
    w = world([SimRover(10, 377, 430, 0, motor()), SimRover(11, 483, 430, 0, motor())],
              [SimCube("red", 430, 430, 0)])
    e = emulator()
    # Disjoint 24 mm strips of a 60 mm square: 40% + 40% = 80%, not 40%.
    coverage, _ = e._coverage(w.cubes["red"], w)
    assert coverage == pytest.approx(0.8)


def test_parallax_projects_body_to_cube_top_not_to_floor():
    w = world([SimRover(10, 700, 430, 0, motor())], [SimCube("red", 780, 430, 0)])
    e = emulator(parallax_enabled=True)
    H, h_rover, h_cube = 2100, 90, 60
    k = (H - h_cube) / (H - h_rover)
    projected = [(430 + k * (x - 430), 430 + k * (y - 430))
                 for x, y in w.footprint.parts(700, 430, 0)[0]]
    expected = _poly_area(_clip_convex(cube_poly(w.cubes["red"]), projected)) / 3600
    assert 0 < expected < 0.05
    assert e._coverage(w.cubes["red"], w)[0] == pytest.approx(expected)


def test_cached_observation_age_is_independent_of_transport_latency():
    """Control: age_ms counts missed detections; ts_ms carries network age."""
    w = world([SimRover(10, 300, 430, 0, motor())])
    e = emulator(latency_mean_s=0.2)
    e.capture(w, 0)
    e.params = replace(e.params, rover_loss_prob=1.0)
    w.rovers[10].x = 350
    e.capture(w, 0.05)
    first, = e.poll(0.2)
    second, = e.poll(0.25)
    assert first["rovers"][0]["age_ms"] == 0
    # The official capture clock is 30 Hz, independent of 20 Hz publication.
    assert second["rovers"][0]["age_ms"] == 33
    assert second["rovers"][0]["col"] == first["rovers"][0]["col"]
    assert second["ts_ms"] == 33


def install_actor(monkeypatch, w, *, busy=False, speed=0.0, counters=None):
    """Only control decisions are stubbed; runner, timing, sensors and truth remain real."""
    class Actor:
        def __init__(self, ids, cfg):
            self.agents = {
                rid: SimpleNamespace(
                    task=SimpleNamespace(color="red") if busy else None,
                    engaged=busy,
                    stats=SimpleNamespace(replans=0, recoveries=0, estops=0,
                                          capture_failures=0, push_cross_track=[]),
                ) for rid in ids
            }
            self.est, self.events, self.counters = {}, [], Counter(counters or {})

        def ingest(self, frame, t):
            pass

        def tick(self, t):
            return {rid: WheelCommand(speed, speed) if rid == 10 else STOP
                    for rid in self.agents}

    sc = SC.Scenario(
        seed=123, family="adversarial", rovers=[], cubes=[],
        depots={"red": (40.5, 2.5), "blue": (2.5, 2.5)}, start=(2.5, 2.5),
        sensor=quiet_sensor(), contact=ContactParams(), board_cols=43, board_rows=43, cell_mm=20,
    )
    monkeypatch.setattr(runner, "Supervisor", Actor)
    monkeypatch.setattr(SC, "instantiate", lambda sc, cfg: (w, emulator()))
    return sc


def delivered_scene():
    w = world([SimRover(10, 430, 430, 0, motor()), SimRover(11, 430, 650, 0, motor())],
              [SimCube("red", 810, 810, 0), SimCube("blue", 50, 810, 0)])
    # A scoring snapshot after prior deliveries: no truth claim that both helped.
    for c in w.cubes.values():
        c.touched_by[10] = 0.0
    return w


def test_success_requires_both_rovers_to_have_participated(monkeypatch):
    w = delivered_scene()
    sc = install_actor(monkeypatch, w)
    res = runner.run_scenario(sc, max_time=0.1)
    assert res.outcome != "success", "H5 violated: rover 11 never transported any cube"


def test_participation_requires_engaged_displacement_not_touch_history():
    w = delivered_scene()
    w.cubes["red"].touched_by = {10: 0.0, 11: 1.0}
    transported, deposited = runner._participation(
        w, {"red": True, "blue": True}
    )
    assert transported == {10: 0, 11: 0}
    assert deposited == {10: 0, 11: 0}

    w.cubes["red"].engaged_displacement = {10: 29.9, 11: 30.0}
    transported, deposited = runner._participation(
        w, {"red": True, "blue": True}
    )
    assert transported[10] == 0 and deposited[10] == 0
    assert transported[11] == 1 and deposited[11] == 1


def test_cube_exit_history_prevents_clean_success(monkeypatch):
    w = delivered_scene()
    w.cubes["blue"].touched_by = {11: 0.0}
    w.counts["cube_exit"] = 1
    sc = install_actor(monkeypatch, w)
    res = runner.run_scenario(sc, max_time=0.1)
    assert res.outcome != "success", "an exited-and-returned cube is scored as clean success"


def test_cube_exit_is_irreversible_even_if_truth_position_returns_inside():
    c = SimCube("red", C.board.width + 1.0, 430, 0)
    w = world(cubes=[c])
    w.step(0.01)
    c.x = C.board.width / 2.0
    w.step(0.01)
    assert not c.in_play


def test_planner_reason_drives_planner_rejected_class():
    res = runner.RunResult(1, "test", n_cubes=2, delivered=0)
    sup = SimpleNamespace(events=[], counters={})
    assert runner.classify(res, sup, 1.0, 10.0, 0.0, 10.0,
                           planner_reason="no feasible push") == "planner_rejected"


def test_completion_waits_for_motor_stop_and_delivery_stability(monkeypatch):
    w = delivered_scene()
    w.cubes["blue"].touched_by = {11: 0.0}
    r = w.rovers[10]
    r.x, r.y = 733, 810  # flush push; cube initially centred in its depot
    r.motor = MotorParams(tau_s=0.2, latency_s=0.05, speed_noise_std=0)
    r.wl = r.wr = r.cmd_left = r.cmd_right = 180
    w.set_target(10, "red")
    sc = install_actor(monkeypatch, w)
    res = runner.run_scenario(sc, max_time=0.1)
    # Replay physical coast-down after the judge's claimed completion.
    for _ in range(100):
        w.step(0.01)
    depots = {"red": DepotZone("red", 810, 810, 50)}
    assert not runner.truth_delivered(w, depots, C)["red"]
    assert res.outcome != "success", (
        f"success at {res.completion_time}s while moving; queued STOP later loses delivery"
    )


def test_truth_delivery_excludes_out_of_play_cubes():
    c = SimCube("red", 810, 810, 0, in_play=False)
    assert not runner.truth_delivered(world(cubes=[c]), {"red": DepotZone("red", 810, 810, 50)}, C)["red"]


@pytest.mark.parametrize("alpha,x,expected", [(0, 830, True), (0, 830.01, False),
                                             (math.pi / 4, 820, False)])
def test_truth_delivery_checks_entire_rotated_cube(alpha, x, expected):
    c = SimCube("red", x, 810, alpha)
    assert runner.truth_delivered(world(cubes=[c]), {"red": DepotZone("red", 810, 810, 50)}, C)["red"] is expected


def test_physical_infeasibility_does_not_mean_planner_margin_failure():
    # Start 180 mm behind the cube, clear even of the paddle tips. The 10.25 mm
    # side clearance is physically legal but below the planner's 12 mm margin.
    w = world([SimRover(10, 60, 250, math.pi / 2, motor())], [SimCube("red", 60, 430, 0)])
    depots = {"red": DepotZone("red", 50, 810, 50)}
    reason = runner.infeasibility(w, depots, C)
    w.set_target(10, "red")
    w.set_command(10, WheelCommand(50, 50), 0)
    for _ in range(966):
        w.step(0.01)
    assert runner.truth_delivered(w, depots, C)["red"]
    assert not w.counts  # all of the push stays inside the field without violations
    assert not reason, f"physically infeasible despite a demonstrated legal push: {reason}"


def test_stall_watchdog_recognizes_continuous_cube_progress(monkeypatch):
    w = world([SimRover(10, 200, 430, 0, motor()), SimRover(11, 430, 650, 0, motor())],
              [SimCube("red", 277, 430, 0), SimCube("blue", 550, 650, 0)])
    sc = install_actor(monkeypatch, w, busy=True, speed=50)
    res = runner.run_scenario(sc, max_time=0.6, stall_s=0.2)
    assert w.cubes["red"].x > 285
    assert res.sim_time >= 0.6, "moving cube called stalled solely because no depot count changed"


def test_classified_deadlock_increments_reported_deadlocks(monkeypatch):
    w = world([SimRover(10, 200, 200, 0, motor()), SimRover(11, 650, 200, 0, motor())],
              [SimCube("red", 400, 430, 0), SimCube("blue", 550, 650, 0)])
    sc = install_actor(monkeypatch, w, busy=True, counters={"yield_failed": 4})
    res = runner.run_scenario(sc, max_time=0.1)
    assert res.failure_class == "coordination_deadlock"
    assert res.deadlocks >= 1, "deadlock metrics are always their dataclass default zero"


def test_three_of_three_metric_does_not_count_two_cube_missions():
    result = runner.RunResult(1, "start_zone", outcome="success", n_cubes=2, delivered=2,
                              completion_time=20)
    assert aggregate([asdict(result)])["delivered_3of3_rate"] == 0


def test_scenario_grid_is_preserved_when_instantiating_without_matching_config():
    larger = replace(C, board=replace(C.board, cols=50, rows=50))
    sc = SC.generate(7, "arbitrary", larger)
    w, e = SC.instantiate(SC.Scenario.from_json(sc.to_json()))
    e.params = quiet_sensor()
    e.capture(w, 0)
    msg, = e.poll(0)
    assert msg["grid"]["cols"] == sc.board_cols, "serialized grid silently replaced with DEFAULT"


def test_distribution_audit_controls():
    """Document actual mixture; do not pretend two cubes violate the contract."""
    samples = [SC.generate(seed, "start_zone") for seed in range(20)]
    assert {len(sc.cubes) for sc in samples} == {2, 3}
    assert all(len(SC.generate(seed, "official_like").cubes) == 3 for seed in range(5))
    assert all(sc.sensor.latency_mean_s == 0.05 for sc in samples)
    assert all(sc.sensor.marker_offset_mm == 4 for sc in samples)


def test_sustained_collision_counter_is_only_an_episode_counter():
    """Witness: one second of driving into another rover is reported as one event.

    This is not a false-success bug (the runner fails any collision); it shows why
    the count alone cannot measure collision duration or recovery severity.
    """
    w = world([SimRover(10, 200, 430, 0, motor()),
               SimRover(11, 405, 430, math.pi, motor())])
    for rid in w.rovers:
        w.set_command(rid, WheelCommand(180, 180), 0)
    for _ in range(100):
        w.step(0.01)
    assert w.counts["collision"] == 1
    # Acceleration-limited rovers reach the collision after a small approach;
    # they are then held stopped for the single sustained episode.
    assert w.rovers[10].x < 201 and w.rovers[11].x > 404
    assert w.rovers[10].wl == w.rovers[10].wr == 0
    assert w.rovers[11].wl == w.rovers[11].wr == 0
