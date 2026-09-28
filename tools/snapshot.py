"""Render sim snapshots: python tools/snapshot.py SEED FAMILY T1,T2,... -> runs/snap_<seed>_<t>.png
Shows true rover envelopes (colored by id), cubes, depots, committed reservations and agent states."""
import sys, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from rover_strategy.simulation import scenarios as SC
from rover_strategy.simulation import runner as R
from rover_strategy.coordination import supervisor as SUP
from rover_strategy.geometry.footprint import Footprint
from rover_strategy.geometry import shapes as S
from rover_strategy.config import DEFAULT as C

seed, fam = int(sys.argv[1]), sys.argv[2]
times = sorted(float(x) for x in sys.argv[3].split(","))
sc = SC.generate(seed, fam)
cap = {}
orig = SUP.Supervisor.tick
def tick(self, t):
    out = orig(self, t)
    if times and t >= times[0]:
        cap[times.pop(0)] = (self, t)
        draw(self, t)
    return out
SUP.Supervisor.tick = tick
fp = Footprint(C.rover)
world_ref = {}
orig_inst = SC.instantiate
def inst(*a, **k):
    w, e = orig_inst(*a, **k); world_ref["w"] = w; return w, e
R.SC.instantiate = inst
cols = {10: "tab:blue", 11: "tab:orange"}
def draw(sup, t):
    w = world_ref["w"]
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.add_patch(Rectangle((0, 0), 860, 860, fill=False, lw=2))
    for c, d in sup.depots.items():
        ax.add_patch(Rectangle((d.cx - d.half, d.cy - d.half), 2 * d.half, 2 * d.half, fill=False, ec=c, ls="--"))
    for rid, polys in sup.committed.items():
        for p in polys:
            ax.add_patch(Polygon(p, closed=True, fc=cols[rid], alpha=0.08, ec=cols[rid], lw=0.5))
    for rid, r in w.rovers.items():
        for part in fp.parts(r.x, r.y, r.theta):
            ax.add_patch(Polygon(part, closed=True, fc=cols[rid], alpha=0.8))
        a = sup.agents[rid]
        ax.text(r.x, r.y - 70, f"{rid}:{a.state.value}\n{a.task.color if a.task else ''}", fontsize=7, ha="center")
    for c, cb in w.cubes.items():
        ax.add_patch(Polygon(S.square(cb.x, cb.y, 60, cb.alpha), closed=True, fc=c, alpha=0.9))
    ax.set_xlim(-20, 880); ax.set_ylim(-20, 880); ax.set_aspect("equal"); ax.set_title(f"seed {seed} {fam} t={t:.2f}")
    fig.savefig(f"runs/snap_{seed}_{t:.1f}.png", dpi=80); plt.close(fig)
R.run_scenario(sc, max_time=max(cap.keys(), default=0) + max([0] + times) + 1 if times else 1)
