"""Trace rover 10 in the single-rover scenario: true speed, commanded (v,w), estimator heading std, state."""
import sys, math
from dataclasses import replace
from rover_strategy.simulation import scenarios as SC
from rover_strategy.simulation import runner as R
from rover_strategy.coordination import supervisor as SUP
cx, cy = float(sys.argv[1]), float(sys.argv[2]); T = float(sys.argv[3])
sc = SC.generate(1, "official_like")
r0 = sc.rovers[0]; far = replace(sc.rovers[1], x=780.0, y=780.0, theta=math.pi)
sc = replace(sc, rovers=[replace(r0, x=430.0, y=700.0, theta=-math.pi / 2), far],
             cubes=[SC.CubeInit(color="red", x=cx, y=cy, alpha=0.0)])
print("motor10", sc.rovers[0].motor)
orig = SUP.Supervisor.tick
w_ref = {}
oi = SC.instantiate
def inst(*a, **k):
    w, e = oi(*a, **k); w_ref["w"] = w; return w, e
R.SC.instantiate = inst
last = {}
def tick(self, t):
    out = orig(self, t)
    w = w_ref["w"]; r = w.rovers[10]
    if 10 in self.est and int(t * 20) % 5 == 0:
        e = self.est[10].estimate(t); v, om = self.cmd[10]
        p = last.get("p"); sp = math.hypot(r.x - p[0], r.y - p[1]) / (t - p[2]) if p else 0
        last["p"] = (r.x, r.y, t)
        print(f"t={t:5.2f} {self.agents[10].state.value:10s} cmd v={v:6.1f} w={om:5.2f} true=({r.x:5.0f},{r.y:5.0f},{math.degrees(r.theta):6.1f}) sp={sp:5.1f} est_h={math.degrees(e.pose.theta):6.1f} hstd={math.degrees(e.heading_std):4.1f} ev={e.v:6.1f} {getattr(self.est[10],'lost_reasons',[])}")
    return out
SUP.Supervisor.tick = tick
R.run_scenario(sc, max_time=T)
