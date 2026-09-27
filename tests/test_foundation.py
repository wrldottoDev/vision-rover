import math
import numpy as np
from rover_strategy.config import DEFAULT as C
from rover_strategy.frames import Grid, wrap, angle_diff, marker_to_body, body_to_marker
from rover_strategy.geometry import shapes as S
from rover_strategy.geometry.footprint import Footprint
from rover_strategy.geometry.zones import DepotZone, cube_half_extent


def test_official_heading_convention():
    g = Grid()
    # theta 90 official points to decreasing row = +y internally
    x0, y0, th = g.to_internal_pose(10, 10, 90.0)
    x1, y1 = g.to_internal_xy(10, 9)          # one cell "up" on screen
    assert abs(math.atan2(y1 - y0, x1 - x0) - th) < 1e-9
    assert g.to_internal_xy(0, 43) == (0.0, 0.0)
    assert g.to_official_xy(*g.to_internal_xy(12.35, 7.5)) == (12.35, 7.5)


def test_wrap():
    assert abs(wrap(3 * math.pi) + math.pi) < 1e-12
    assert abs(angle_diff(math.radians(359), math.radians(1)) - math.radians(-2)) < 1e-12


def test_marker_roundtrip():
    p = marker_to_body(100, 50, 0.7, 10, -3, 0.01)
    q = body_to_marker(*p, 10, -3, 0.01)
    assert np.allclose(q, (100, 50, 0.7))


def test_footprint_numbers():
    fp = Footprint(C.rover)
    assert abs(fp.sweep_radius - math.hypot(102, 49.75)) < 1e-9
    e = fp.envelope(0, 0, 0)
    assert e[:, 0].min() == -47 and e[:, 0].max() == 102
    assert C.prepush_distance > 102 + C.cube.half_diag


def test_sat_and_distance():
    a = S.rect(0, 10, 0, 10)
    b = S.rect(10, 20, 0, 10)
    c = S.rect(13, 20, 0, 10)
    assert S.overlap(a, b) and not S.overlap(a, c)
    assert abs(S.distance(a, c) - 3) < 1e-9
    d = S.square(20, 5, 10, math.pi / 4)      # diamond, left vertex at x = 20 - 7.07
    assert abs(S.distance(a, d) - (10 - 10 / math.sqrt(2))) < 1e-9


def test_depot_region():
    z = DepotZone("red", 810, 810, 50)
    r0 = z.valid_center_region(60, 0.0, 0.0)
    assert np.allclose(r0, (790, 830, 790, 830))
    r45 = z.valid_center_region(60, math.pi / 4, 0.0)
    assert abs((r45[1] - r45[0]) / 2 - (50 - 30 * math.sqrt(2))) < 1e-9
    assert z.valid_center_region(60, None, 10) is None
    assert abs(cube_half_extent(60, None) - 42.426) < 1e-3
