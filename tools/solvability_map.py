"""Fraction of cube positions (single cube, no other objects) for which PushPlanner finds a plan, per depot."""
import sys, numpy as np
from rover_strategy.planning.push_planner import PushPlanner
from rover_strategy.world import CubeEstimate
from rover_strategy.geometry.zones import DepotZone
step = float(sys.argv[1]) if len(sys.argv) > 1 else 40
pp = PushPlanner()
for name, (dx, dy) in {"top-right": (810, 810), "bottom-left": (50, 50), "bottom-right": (810, 50)}.items():
    dep = DepotZone("red", dx, dy, 50)
    tot = ok = 0; legs = {}
    for x in np.arange(45, 820, step):
        for y in np.arange(45, 820, step):
            if dep.contains_cube(x, y, 60, None, -42.5):   # inside/overlapping depot: skip
                continue
            tot += 1
            ps = pp.plan(CubeEstimate("red", float(x), float(y)), dep, [], 860, 860, max_plans=1, deadline_s=float(__import__("os").environ.get("DL", 0.3)))
            if ps:
                ok += 1; n = len(ps[0].legs); legs[n] = legs.get(n, 0) + 1
    print(name, f"solvable {ok}/{tot} = {ok/tot:.1%}", "legs:", dict(sorted(legs.items())))
