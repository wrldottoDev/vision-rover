"""Canonical shared data model.  All modules exchange these types.  Units: mm, rad, s (see config.py)."""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field

import numpy as np

from .frames import wrap
from .geometry.zones import DepotZone

COLORS = ("red", "green", "blue")


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float

    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def forward(self) -> np.ndarray:
        return np.array([math.cos(self.theta), math.sin(self.theta)])

    def moved(self, dist: float) -> "Pose":
        return Pose(self.x + dist * math.cos(self.theta), self.y + dist * math.sin(self.theta), self.theta)

    def rotated_to(self, theta: float) -> "Pose":
        return Pose(self.x, self.y, wrap(theta))

    def to_local(self, px: float, py: float) -> tuple[float, float]:
        """World point -> this pose's frame (x forward, y left)."""
        dx, dy = px - self.x, py - self.y
        c, s = math.cos(self.theta), math.sin(self.theta)
        return c * dx + s * dy, -s * dx + c * dy


class TrackQuality(enum.Enum):
    GOOD = "GOOD"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


# ---------------------------------------------------------------- observations (parser output)
@dataclass(frozen=True)
class RoverObs:
    id: int
    pose: Pose            # rotation-centre pose (marker offset already removed)
    t_capture: float      # s, capture time of the frame in which it was last really seen
    age_s: float


@dataclass(frozen=True)
class CubeObs:
    color: str
    x: float
    y: float
    t_capture: float
    age_s: float


@dataclass(frozen=True)
class Frame:
    seq: int
    t_capture: float      # s (ts_ms / 1000)
    t_received: float     # s, local clock
    phase: str
    board_w: float
    board_h: float
    rovers: dict[int, RoverObs]
    cubes: dict[str, CubeObs]
    depots: dict[str, DepotZone]
    obstacles: tuple[tuple[float, float], ...] = ()


# ---------------------------------------------------------------- estimates (world model)
@dataclass
class RoverEstimate:
    id: int
    pose: Pose
    v: float = 0.0                       # mm/s forward
    omega: float = 0.0                   # rad/s
    cov: np.ndarray = field(default_factory=lambda: np.diag([25.0, 25.0, 0.001]))   # x,y,theta
    age_s: float = 0.0                   # since last accepted vision update
    quality: TrackQuality = TrackQuality.GOOD

    @property
    def pos_std(self) -> float:
        return math.sqrt(max(self.cov[0, 0], self.cov[1, 1]))

    @property
    def heading_std(self) -> float:
        return math.sqrt(self.cov[2, 2])


@dataclass
class CubeEstimate:
    color: str
    x: float
    y: float
    pos_std: float = 2.0
    age_s: float = 0.0
    # Orientation modulo 90 deg in [-pi/4, pi/4); None = unknown (not observable from contract v1).
    alpha: float | None = None
    alpha_std: float = math.pi / 4
    delivered: bool = False

    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])


@dataclass
class WorldState:
    t: float
    phase: str
    board_w: float
    board_h: float
    rovers: dict[int, RoverEstimate]
    cubes: dict[str, CubeEstimate]
    depots: dict[str, DepotZone]


# ---------------------------------------------------------------- plans and commands
@dataclass(frozen=True)
class WheelCommand:
    v_left: float         # mm/s
    v_right: float

    @staticmethod
    def from_unicycle(v: float, w: float, track: float, vmax: float) -> "WheelCommand":
        l, r = v - w * track / 2.0, v + w * track / 2.0
        k = max(1.0, abs(l) / vmax, abs(r) / vmax)      # scale both to keep curvature
        return WheelCommand(l / k, r / k)

    def unicycle(self, track: float) -> tuple[float, float]:
        return (self.v_left + self.v_right) / 2.0, (self.v_right - self.v_left) / track


STOP = WheelCommand(0.0, 0.0)


class SegKind(enum.Enum):
    ROTATE = "ROTATE"
    STRAIGHT = "STRAIGHT"     # forward if end is ahead, reverse otherwise (see `reverse`)
    ARC = "ARC"               # constant-curvature motion from start to end (see `curvature`, `reverse`)


@dataclass(frozen=True)
class Segment:
    """One motion primitive.  STRAIGHT: start/end collinear with the heading.  ROTATE: in place.
    ARC: constant signed curvature `curvature` (1/mm, + = turning left/CCW w.r.t. the direction of travel);
    the heading changes by curvature * arc_length; `reverse` = driven backwards."""
    kind: SegKind
    start: Pose
    end: Pose
    reverse: bool = False
    curvature: float = 0.0

    @property
    def length(self) -> float:
        """Path length of the rotation centre (chord for STRAIGHT, arc length for ARC, 0 for ROTATE)."""
        if self.kind is SegKind.ARC and abs(self.curvature) > 1e-12:
            from .frames import angle_diff
            return abs(angle_diff(self.end.theta, self.start.theta) / self.curvature)
        return math.hypot(self.end.x - self.start.x, self.end.y - self.start.y)


@dataclass
class Path:
    segments: list[Segment]
    cost_s: float                       # estimated execution time

    @property
    def goal(self) -> Pose:
        return self.segments[-1].end


@dataclass(frozen=True)
class PushLeg:
    cube_start: tuple[float, float]
    cube_end: tuple[float, float]
    heading: float                      # push direction (rad)
    prepush: Pose                       # rover pose to reach before capture (heading == push heading)

    @property
    def length(self) -> float:
        return math.hypot(self.cube_end[0] - self.cube_start[0], self.cube_end[1] - self.cube_start[1])


@dataclass
class PushPlan:
    color: str
    legs: list[PushLeg]
    delivery_point: tuple[float, float]
    cost_s: float                       # estimated push-phase time (excludes first navigation)
    risk: float                         # 0..1-ish penalty, higher is worse
    notes: str = ""
