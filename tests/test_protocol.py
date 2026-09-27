import pytest

from rover_strategy.world import Pose
from rover_strategy.coordination.protocol import (
    StatusMessage, pack, unpack, ProtocolError, SIZE, to_wire_priority,
)


def test_round_trip_basic():
    msg = StatusMessage(
        robot_id=11, timestamp_ms=123456, state="PUSH", mission="red",
        pose=Pose(123.4, -456.7, 1.234), priority=200,
        reservation_bbox=(10.0, -20.0, 300.0, 400.0),
    )
    data = pack(msg)
    assert len(data) == SIZE <= 64
    back = unpack(data)

    assert back.robot_id == 11
    assert back.timestamp_ms == 123456
    assert back.state == "PUSH"
    assert back.mission == "red"
    assert back.priority == 200
    assert back.reservation_bbox == pytest.approx((10.0, -20.0, 300.0, 400.0), abs=1.0)
    assert back.pose.x == pytest.approx(123.4, abs=1.0)
    assert back.pose.y == pytest.approx(-456.7, abs=1.0)
    assert back.pose.theta == pytest.approx(1.234, abs=1e-3)


def test_round_trip_no_reservation_and_no_mission():
    msg = StatusMessage(robot_id=10, timestamp_ms=0, state="IDLE", mission=None,
                         pose=Pose(0.0, 0.0, 0.0), priority=0, reservation_bbox=None)
    back = unpack(pack(msg))
    assert back.mission is None
    assert back.reservation_bbox is None
    assert back.state == "IDLE"


def test_unknown_state_round_trips_as_unknown_marker():
    msg = StatusMessage(robot_id=10, timestamp_ms=1, state="SOME_FUTURE_STATE", mission=None,
                         pose=Pose(0, 0, 0), priority=0)
    back = unpack(pack(msg))
    assert back.state == "UNKNOWN"


def test_corruption_detected_via_crc():
    msg = StatusMessage(robot_id=10, timestamp_ms=42, state="NAVIGATE", mission="green",
                         pose=Pose(10, 20, 0.5), priority=50)
    data = bytearray(pack(msg))
    data[5] ^= 0xFF        # flip a byte in the body
    with pytest.raises(ProtocolError):
        unpack(bytes(data))


def test_wrong_length_rejected():
    with pytest.raises(ProtocolError):
        unpack(b"\x00" * (SIZE - 1))
    with pytest.raises(ProtocolError):
        unpack(b"\x00" * (SIZE + 5))


def test_version_mismatch_rejected():
    msg = StatusMessage(robot_id=10, timestamp_ms=0, state="IDLE", mission=None,
                         pose=Pose(0, 0, 0), priority=0)
    data = bytearray(pack(msg))
    data[0] = 99                                   # bump the version byte
    # recompute CRC so this fails on the version check, not by accident on CRC
    from rover_strategy.coordination.protocol import _crc8, _BODY_FMT
    data[-1] = _crc8(bytes(data[:-1]))
    with pytest.raises(ProtocolError):
        unpack(bytes(data))


def test_to_wire_priority_monotonic_and_clamped():
    assert to_wire_priority(0) == 0
    assert to_wire_priority(10 * 1000 - 0) <= 255
    assert to_wire_priority(100 * 1000 - 0) == 250          # realistic max (base=100, id=0), no clamp needed yet
    assert to_wire_priority(999_999) == 255                 # out-of-range input still clamps
    assert to_wire_priority(9990) < to_wire_priority(99989)
