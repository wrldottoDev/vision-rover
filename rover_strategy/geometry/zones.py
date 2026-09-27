"""Depot zones and the delivery criterion (whole cube footprint inside the zone)."""
from __future__ import annotations

import math
from dataclasses import dataclass

from .shapes import Poly, rect, square


def cube_half_extent(side: float, alpha: float | None) -> float:
    """Axis-aligned half extent of a square of `side` rotated by alpha (None = unknown => worst case)."""
    if alpha is None:
        return side / math.sqrt(2.0)
    return side / 2.0 * (abs(math.cos(alpha)) + abs(math.sin(alpha)))


@dataclass(frozen=True)
class DepotZone:
    color: str
    cx: float
    cy: float
    half: float

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return self.cx - self.half, self.cx + self.half, self.cy - self.half, self.cy + self.half

    def polygon(self) -> Poly:
        return rect(*self.bounds)

    def valid_center_region(self, side: float, alpha: float | None, margin: float) -> tuple[float, float, float, float] | None:
        """Axis-aligned box of cube-centre positions for which the whole cube (orientation alpha) lies inside
        the zone with `margin` to spare.  zone eroded by the rotated cube (exact for a square zone)."""
        h = self.half - cube_half_extent(side, alpha) - margin
        if h < 0:
            return None
        return self.cx - h, self.cx + h, self.cy - h, self.cy + h

    def contains_cube(self, x: float, y: float, side: float, alpha: float | None, margin: float = 0.0) -> bool:
        r = self.valid_center_region(side, alpha, margin)
        return r is not None and r[0] <= x <= r[1] and r[2] <= y <= r[3]


def cube_polygon(x: float, y: float, side: float, alpha: float) -> Poly:
    return square(x, y, side, alpha)
