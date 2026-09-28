"""Convex-polygon primitives (numpy arrays of shape (n,2), CCW).  SAT overlap and exact distances."""
from __future__ import annotations

import math

import numpy as np

Poly = np.ndarray


def rect(x0: float, x1: float, y0: float, y1: float) -> Poly:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def transform(points: Poly, x: float, y: float, th: float) -> Poly:
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    return points @ R.T + np.array([x, y])


def square(cx: float, cy: float, side: float, alpha: float) -> Poly:
    h = side / 2.0
    return transform(rect(-h, h, -h, h), cx, cy, alpha)


def _axes(P: Poly) -> np.ndarray:
    e = np.roll(P, -1, axis=0) - P
    n = np.stack([-e[:, 1], e[:, 0]], axis=1)
    L = np.linalg.norm(n, axis=1, keepdims=True)
    return n[L[:, 0] > 1e-12] / L[L[:, 0] > 1e-12]


def overlap(A: Poly, B: Poly) -> bool:
    """SAT: True if the closed convex polygons intersect (touching counts)."""
    for ax in np.vstack([_axes(A), _axes(B)]):
        pa, pb = A @ ax, B @ ax
        if pa.max() < pb.min() or pb.max() < pa.min():
            return False
    return True


def _seg_point_dist(a: np.ndarray, b: np.ndarray, p: np.ndarray) -> float:
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.linalg.norm(a + t * ab - p))


def bbox_gap(A: Poly, B: Poly) -> float:
    """Cheap lower bound of distance(A, B) from axis-aligned bounding boxes (0 if the boxes overlap)."""
    dx = max(0.0, float(B[:, 0].min() - A[:, 0].max()), float(A[:, 0].min() - B[:, 0].max()))
    dy = max(0.0, float(B[:, 1].min() - A[:, 1].max()), float(A[:, 1].min() - B[:, 1].max()))
    return math.hypot(dx, dy)


def _pts_segs_min(P: Poly, Q: Poly) -> float:
    """min distance from points P to the closed polygon boundary Q (vectorised)."""
    a = Q
    ab = np.roll(Q, -1, axis=0) - Q                                  # (m,2)
    ap = P[:, None, :] - a[None, :, :]                               # (n,m,2)
    den = np.einsum("mk,mk->m", ab, ab)
    t = np.clip(np.einsum("nmk,mk->nm", ap, ab) / np.where(den > 0, den, 1.0), 0.0, 1.0)
    d = ap - t[:, :, None] * ab[None, :, :]
    return float(np.sqrt(np.einsum("nmk,nmk->nm", d, d).min()))


def distance(A: Poly, B: Poly) -> float:
    """Euclidean distance between convex polygons; 0 if they intersect."""
    if overlap(A, B):
        return 0.0
    return min(_pts_segs_min(A, B), _pts_segs_min(B, A))


def point_in(P: Poly, p) -> bool:
    e = np.roll(P, -1, axis=0) - P
    w = np.asarray(p, float) - P
    return bool(np.all(e[:, 0] * w[:, 1] - e[:, 1] * w[:, 0] >= -1e-9))


def disc_distance(P: Poly, c, r: float) -> float:
    """Distance from convex polygon to a disc (0 if they intersect)."""
    c = np.asarray(c, float)
    if point_in(P, c):
        return 0.0
    n = len(P)
    d = min(_seg_point_dist(P[i], P[(i + 1) % n], c) for i in range(n))
    return max(0.0, d - r)


def inside_rect(P: Poly, x0: float, x1: float, y0: float, y1: float) -> bool:
    return bool(P[:, 0].min() >= x0 and P[:, 0].max() <= x1 and P[:, 1].min() >= y0 and P[:, 1].max() <= y1)


def hull(points: np.ndarray) -> Poly:
    """Monotone-chain convex hull, CCW."""
    pts = sorted(map(tuple, np.asarray(points, float)))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return np.array(lo[:-1] + hi[:-1])
