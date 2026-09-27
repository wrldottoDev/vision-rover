"""Oriented rover footprint.  Rover frame origin = rotation centre, +x forward, +y left.

envelope(): conservative convex hull (chassis + paddles + the channel between them).  Used for every
            collision check against things the rover must NOT touch (non-target cubes, other rover, board).
parts():    the real U shape (body + two paddle rails).  Used by the simulator and capture reasoning.
"""
from __future__ import annotations

import math

import numpy as np

from ..config import RoverGeometry
from .shapes import Poly, hull, rect, transform


class Footprint:
    def __init__(self, g: RoverGeometry, inflate: float = 0.0):
        self.g = g
        i = inflate
        self._env = rect(g.x_rear - i, g.x_paddle_tip + i, -g.outer_half_width - i, g.outer_half_width + i)
        self._body = rect(g.x_rear, g.x_front_plate, -g.outer_half_width, g.outer_half_width)
        self._pl = rect(g.x_front_plate, g.x_paddle_tip, g.inner_half_width, g.outer_half_width)
        self._pr = rect(g.x_front_plate, g.x_paddle_tip, -g.outer_half_width, -g.inner_half_width)
        self.sweep_radius = max(float(np.linalg.norm(p)) for p in self._env)

    def envelope(self, x: float, y: float, th: float) -> Poly:
        return transform(self._env, x, y, th)

    def parts(self, x: float, y: float, th: float) -> list[Poly]:
        return [transform(p, x, y, th) for p in (self._body, self._pl, self._pr)]

    def straight_sweep(self, x: float, y: float, th: float, dist: float) -> Poly:
        """Convex region swept by the envelope translating `dist` along heading (negative = reverse)."""
        a = self.envelope(x, y, th)
        b = self.envelope(x + dist * math.cos(th), y + dist * math.sin(th), th)
        return hull(np.vstack([a, b]))

    def local_envelope(self) -> Poly:
        return self._env.copy()
