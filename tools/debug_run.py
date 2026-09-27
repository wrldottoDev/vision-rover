"""Debug one seed: python tools/debug_run.py SEED FAMILY [MAX_T] — prints scenario, event log, result."""
import sys, json
from dataclasses import asdict
from rover_strategy.simulation import scenarios as SC
from rover_strategy.simulation.runner import run_scenario
from rover_strategy.coordination import supervisor as SUP

seed, fam = int(sys.argv[1]), sys.argv[2]
mt = float(sys.argv[3]) if len(sys.argv) > 3 else 200
sc = SC.generate(seed, fam)
print("rovers", [(r.id, round(r.x), round(r.y), round(r.theta, 2)) for r in sc.rovers])
print("cubes", [(c.color, round(c.x), round(c.y), round(c.alpha, 2)) for c in sc.cubes])
print("depots", sc.depots)
captured = {}
orig = SUP.Supervisor.__init__
def init(self, *a, **k):
    orig(self, *a, **k); captured['s'] = self
SUP.Supervisor.__init__ = init
r = run_scenario(sc, max_time=mt)
s = captured['s']
skip = set(sys.argv[4].split(',')) if len(sys.argv) > 4 else {'estop'}
last = None
for e in s.events:
    if e[2] in skip: continue
    key = (e[1], e[2], str(e[3]))
    if key == last: continue
    last = key
    print(e)
d = asdict(r); d.pop('last_events'); print(json.dumps(d))
