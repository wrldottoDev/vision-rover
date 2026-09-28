"""Single-rover, single-cube closed loop: isolates capture/push/delivery from coordination.
python tools/single_rover.py CUBE_X CUBE_Y [ALPHA_DEG] [DEPOT=bottom-right|...]"""
import sys, math, json
from dataclasses import asdict, replace
from rover_strategy.simulation import scenarios as SC
from rover_strategy.simulation.runner import run_scenario
from rover_strategy.coordination import supervisor as SUP

cx, cy = float(sys.argv[1]), float(sys.argv[2])
alpha = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0
sc = SC.generate(1, "official_like")
r0 = sc.rovers[0]
far = replace(sc.rovers[1], x=780.0, y=780.0, theta=math.pi)       # park rover 11 in a corner, out of the way
sc = replace(sc, rovers=[replace(r0, x=430.0, y=700.0, theta=-math.pi / 2), far],
             cubes=[SC.CubeInit(color="red", x=cx, y=cy, alpha=alpha)])
cap = {}
orig = SUP.Supervisor.__init__
def init(self, *a, **k):
    orig(self, *a, **k); cap["s"] = self
SUP.Supervisor.__init__ = init
r = run_scenario(sc, max_time=120)
s = cap["s"]
for e in s.events:
    if e[2] in ("budget", "commit_denied"):
        continue
    print(e)
d = asdict(r); d.pop("last_events"); print(json.dumps(d))
