import numpy as np

from rover_strategy.config import DEFAULT as C
from rover_strategy.world import Pose, RoverEstimate
from rover_strategy.geometry import shapes as S
from rover_strategy.planning.reservations import (
    ReservationTable, priority, imminent_collision, resolve_deadlock,
)


def _est(rid, x, y, theta, v=0.0, omega=0.0):
    return RoverEstimate(id=rid, pose=Pose(x, y, theta), v=v, omega=omega)


# --------------------------------------------------------------------------- conflicts (inflated)
def test_conflicts_detects_direct_overlap():
    t = ReservationTable(C)
    rectA = S.rect(0, 100, 0, 100)
    t.reserve(1, [rectA], priority=5, t=0.0)
    assert t.conflicts(2, [rectA]) == [1]
    assert t.conflicts(1, [rectA]) == []          # never conflicts with itself


def test_conflicts_respects_rover_rover_margin_inflation():
    t = ReservationTable(C)
    rectA = S.rect(0, 100, 0, 100)
    t.reserve(1, [rectA], priority=5, t=0.0)
    margin = C.margins.rover_rover

    close = S.rect(100 + margin * 0.2, 100 + margin * 0.2 + 40, 0, 100)   # gap < margin
    far = S.rect(100 + margin * 3, 100 + margin * 3 + 40, 0, 100)         # gap > margin
    assert t.conflicts(2, [close]) == [1]
    assert t.conflicts(2, [far]) == []


def test_release_clears_reservation():
    t = ReservationTable(C)
    rectA = S.rect(0, 100, 0, 100)
    t.reserve(1, [rectA], priority=5, t=0.0)
    t.release(1)
    assert t.conflicts(2, [rectA]) == []
    assert t.polygons_of(1) == []


def test_as_obstacles_excludes_own_and_inflates_others():
    t = ReservationTable(C)
    rectA = S.rect(0, 100, 0, 100)
    t.reserve(2, [rectA], priority=5, t=0.0)

    assert t.as_obstacles(for_rover=2) == []       # never your own reservation
    obstacles = t.as_obstacles(for_rover=1)
    assert len(obstacles) == 1
    inflated = obstacles[0]
    margin = C.margins.rover_rover
    # a real outward offset: bounding box grows by roughly the margin on every side
    assert inflated[:, 0].min() < rectA[:, 0].min() - margin * 0.9
    assert inflated[:, 0].max() > rectA[:, 0].max() + margin * 0.9


def test_trim_drops_traversed_polygons_only():
    t = ReservationTable(C)
    rectA = S.rect(0, 100, 0, 100)         # about to be traversed
    rectB = S.rect(1000, 1100, 0, 100)     # untouched, far away
    t.reserve(1, [rectA, rectB], priority=5, t=0.0)

    progress = [S.rect(-10, 50, 0, 100)]   # overlaps rectA only
    t.trim(1, progress)

    remaining = t.polygons_of(1)
    assert len(remaining) == 1
    assert np.allclose(remaining[0], rectB)


# --------------------------------------------------------------------------- priority
def test_priority_state_dominates_and_id_breaks_ties():
    assert priority("PUSH", 11) > priority("NAVIGATE", 10)
    assert priority("CAPTURE", 99) > priority("IDLE", 0)
    assert priority("RETREAT_WITH_CUBE", 11) > priority("WAIT", 10)
    # same state -> lower rover id wins
    assert priority("NAVIGATE", 10) > priority("NAVIGATE", 11)
    assert priority("PUSH", 10) > priority("PUSH", 11)


# --------------------------------------------------------------------------- imminent_collision
def test_imminent_collision_true_head_on():
    a = _est(1, 0, 0, 0.0, v=100.0)
    b = _est(2, 500, 0, 3.14159265, v=100.0)
    assert imminent_collision(a, b, horizon_s=3.0, cfg=C) is True


def test_imminent_collision_false_parallel_separated():
    a = _est(1, 0, 0, 0.0, v=100.0)
    b = _est(2, 0, 300, 0.0, v=100.0)      # same heading, same speed, 300 mm apart laterally
    assert imminent_collision(a, b, horizon_s=5.0, cfg=C) is False


def test_imminent_collision_false_when_stationary_and_far():
    a = _est(1, 0, 0, 0.0)
    b = _est(2, 2000, 2000, 0.0)
    assert imminent_collision(a, b, horizon_s=3.0, cfg=C) is False


# --------------------------------------------------------------------------- deadlock resolution
def test_resolve_deadlock_none_before_timeout():
    assert resolve_deadlock("NAVIGATE", 10, 1.0, "NAVIGATE", 11, 1.0, timeout_s=3.0) is None
    assert resolve_deadlock("NAVIGATE", 10, 5.0, "NAVIGATE", 11, 1.0, timeout_s=3.0) is None


def test_resolve_deadlock_lower_priority_yields_deterministically():
    # same state -> higher rover id (lower priority) yields
    assert resolve_deadlock("NAVIGATE", 10, 5.0, "NAVIGATE", 11, 5.0, timeout_s=3.0) == 11
    # state dominates id: the NAVIGATE one yields even though it has the lower id
    assert resolve_deadlock("PUSH", 11, 5.0, "NAVIGATE", 10, 5.0, timeout_s=3.0) == 10
    # symmetric call order gives the same answer
    assert resolve_deadlock("NAVIGATE", 10, 5.0, "PUSH", 11, 5.0, timeout_s=3.0) == 10
