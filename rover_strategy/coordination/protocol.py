"""Minimal future ESP-NOW status broadcast: robot_id, timestamp, state, mission, pose, priority and
a small reservation bounding box.  Packed with `struct` into a fixed 25-byte frame (budget: <=64
bytes), versioned, CRC8-protected.

Units on the wire: position mm (int16), heading milliradians (int16) -- see config.py's canonical
mm/rad/s.  `StatusMessage` itself stays in plain mm/rad (a `world.Pose`) so callers never touch the
wire encoding directly.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from ..world import COLORS, Pose

PROTOCOL_VERSION = 1

# Small fixed vocabulary for the 1-byte state code. Unknown states still round-trip (as "UNKNOWN")
# rather than raising, since this is a wire format shared with firmware that may lag the state names
# used here.
_STATES = ("IDLE", "NAVIGATE", "CAPTURE", "PUSH", "RETREAT", "RETREAT_WITH_CUBE", "WAIT", "LOST", "STUCK")
_STATE_TO_CODE = {s: i for i, s in enumerate(_STATES)}
_CODE_TO_STATE = dict(enumerate(_STATES))
_UNKNOWN_STATE_CODE = 255

_MISSION_TO_CODE = {None: 0, **{c: i + 1 for i, c in enumerate(COLORS)}}
_CODE_TO_MISSION = {v: k for k, v in _MISSION_TO_CODE.items()}

# version, robot_id, timestamp_ms, state, mission, priority, flags, x, y, theta, bx0, by0, bx1, by1
_BODY_FMT = "<BBIBBBBhhhhhhh"
_FMT = _BODY_FMT + "B"          # + CRC8
SIZE = struct.calcsize(_FMT)     # 25 bytes


class ProtocolError(ValueError):
    """Malformed or corrupted status message."""


def to_wire_priority(raw: int) -> int:
    """Compress reservations.priority()'s large ordinal (state*1000 - rover_id) into the wire's
    0..255 range.  Lossy on purpose: peers only need the relative ordering for situational awareness,
    the authoritative decision is made locally from the full-precision value."""
    return max(0, min(255, raw // 400))


def _crc8(data: bytes) -> int:
    """CRC-8, poly 0x07, init 0x00, no reflection. Plain stdlib loop -- 9 bytes of payload doesn't
    justify a lookup table."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _clamp_i16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


@dataclass(frozen=True)
class StatusMessage:
    robot_id: int
    timestamp_ms: int
    state: str
    mission: str | None                                    # cube colour, or None
    pose: Pose                                              # x, y mm; theta rad
    priority: int                                           # 0..255 wire ordinal, see to_wire_priority
    reservation_bbox: tuple[float, float, float, float] | None = None   # x0, y0, x1, y1 mm, or None


def pack(msg: StatusMessage) -> bytes:
    state_code = _STATE_TO_CODE.get(msg.state, _UNKNOWN_STATE_CODE)
    mission_code = _MISSION_TO_CODE.get(msg.mission, 0)
    has_bbox = msg.reservation_bbox is not None
    bx0, by0, bx1, by1 = msg.reservation_bbox if has_bbox else (0.0, 0.0, 0.0, 0.0)
    body = struct.pack(
        _BODY_FMT,
        PROTOCOL_VERSION,
        msg.robot_id & 0xFF,
        msg.timestamp_ms & 0xFFFFFFFF,
        state_code,
        mission_code,
        max(0, min(255, msg.priority)),
        1 if has_bbox else 0,
        _clamp_i16(msg.pose.x), _clamp_i16(msg.pose.y), _clamp_i16(msg.pose.theta * 1000.0),
        _clamp_i16(bx0), _clamp_i16(by0), _clamp_i16(bx1), _clamp_i16(by1),
    )
    return body + bytes([_crc8(body)])


def unpack(data: bytes) -> StatusMessage:
    if len(data) != SIZE:
        raise ProtocolError(f"expected {SIZE} bytes, got {len(data)}")
    body, crc = data[:-1], data[-1]
    if _crc8(body) != crc:
        raise ProtocolError("CRC8 mismatch: corrupted message")
    (version, robot_id, timestamp_ms, state_code, mission_code, wire_priority, flags,
     x, y, theta_mrad, bx0, by0, bx1, by1) = struct.unpack(_BODY_FMT, body)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    state = _CODE_TO_STATE.get(state_code, "UNKNOWN")
    mission = _CODE_TO_MISSION.get(mission_code)
    pose = Pose(float(x), float(y), theta_mrad / 1000.0)
    bbox = (float(bx0), float(by0), float(bx1), float(by1)) if flags & 1 else None
    return StatusMessage(robot_id, timestamp_ms, state, mission, pose, wire_priority, bbox)
