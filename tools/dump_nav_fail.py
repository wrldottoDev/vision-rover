"""Run a seed; pickle the first few failing NavPlanner.plan queries to runs/navfail_*.pkl."""
import sys, pickle, os
from rover_strategy.planning import navigation as N
from rover_strategy.simulation.runner import run_seed
os.makedirs("runs", exist_ok=True)
orig = N.NavPlanner.plan
k = [0]
def plan(self, start, goal, obstacles, bw, bh, inflate=None, deadline_s=1.0):
    r = orig(self, start, goal, obstacles, bw, bh, inflate, deadline_s)
    if r is None and k[0] < 6:
        pickle.dump((start, goal, obstacles, bw, bh, inflate, deadline_s, self.last_failure),
                    open(f"runs/navfail_{k[0]}.pkl", "wb"))
        print(k[0], self.last_failure, start, goal, len(obstacles))
        k[0] += 1
    return r
N.NavPlanner.plan = plan
run_seed(int(sys.argv[1]), sys.argv[2], max_time=float(sys.argv[3]) if len(sys.argv) > 3 else 5)
