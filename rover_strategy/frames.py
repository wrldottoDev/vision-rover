"""Unit and frame conversions.  The ONLY place official (cells, row-down, degrees) meets internal
(mm, y-up, radians).

Official (CONTRATO.md s4): col right, row DOWN, theta degrees CCW, 0 = +col, forward = (cos, -sin) in (col,row).
Internal: x = col*cell, y = (rows - row)*cell, theta = rad(theta_deg); forward = (cos, sin).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi


def wrap(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % TWO_PI - math.pi


def angle_diff(a: float, b: float) -> float:
    """Signed smallest a - b in [-pi, pi)."""
    return wrap(a - b)


@dataclass(frozen=True)
class Grid:
    cols: int = 43
    rows: int = 43
    cell_mm: float = 20.0

    def to_internal_xy(self, col: float, row: float) -> tuple[float, float]:
        return col * self.cell_mm, (self.rows - row) * self.cell_mm

    def to_official_xy(self, x: float, y: float) -> tuple[float, float]:
        return x / self.cell_mm, self.rows - y / self.cell_mm

    @staticmethod
    def to_internal_heading(theta_deg: float) -> float:
        return wrap(math.radians(theta_deg))

    @staticmethod
    def to_official_heading(theta: float) -> float:
        return math.degrees(theta) % 360.0

    def to_internal_pose(self, col: float, row: float, theta_deg: float) -> tuple[float, float, float]:
        x, y = self.to_internal_xy(col, row)
        return x, y, self.to_internal_heading(theta_deg)


def marker_to_body(x: float, y: float, th: float, fwd: float, left: float, dth: float) -> tuple[float, float, float]:
    """Published marker pose -> rotation-centre pose. (fwd, left) is the vector marker->rotation centre
    expressed in the rover frame (same convention as config_vision.json desfase_marcador_a_centro_mm)."""
    t = wrap(th + dth)
    c, s = math.cos(t), math.sin(t)
    return x + c * fwd - s * left, y + s * fwd + c * left, t


def body_to_marker(x: float, y: float, th: float, fwd: float, left: float, dth: float) -> tuple[float, float, float]:
    c, s = math.cos(th), math.sin(th)
    return x - (c * fwd - s * left), y - (s * fwd + c * left), wrap(th - dth)
