"""Monte Carlo driver: run many seeded scenarios in parallel processes, classify outcomes, aggregate metrics,
save every failing seed for deterministic reproduction.

Usage:  python -m rover_strategy.simulation.monte_carlo --n 1000 --family start_zone --out runs/mc_dev
        python -m rover_strategy.simulation.monte_carlo --seeds 17,523 --family hard --verbose
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict

from .runner import RunResult, run_seed

SAFETY_KEYS = ("rover_rover_collisions", "non_target_contacts", "rover_exits", "falls")


def _work(args):
    seed, family, max_time, overhang = args
    try:
        from dataclasses import replace
        from ..config import DEFAULT
        cfg = replace(DEFAULT, board=replace(DEFAULT.board, overhang_allowance_mm=overhang))
        return asdict(run_seed(seed, family, max_time=max_time, cfg=cfg))
    except Exception as exc:          # a crash is a finding, never hidden
        import traceback
        return {"seed": seed, "family": family, "outcome": "crash", "failure_class": "software_crash",
                "error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()[-2000:]}


def pct(xs, q):
    if not xs:
        return None
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))]


def aggregate(results: list[dict]) -> dict:
    n = len(results)
    ok = [r for r in results if r.get("outcome") == "success"]
    times = [r["completion_time"] for r in ok if r.get("completion_time") is not None]
    settled_times = [r["settled_completion_time"] for r in results
                     if r.get("settled_completion_time") is not None]
    delivery_times = [r["delivery_time"] for r in results if r.get("delivery_time") is not None]
    three_cube = [r for r in results if r.get("n_cubes") == 3]
    feasible = [r for r in results if r.get("failure_class") != "physically_infeasible"]
    agg = {
        "runs": n,
        "success_rate": len(ok) / n if n else 0.0,
        "success_rate_feasible": (len([r for r in feasible if r.get("outcome") == "success"]) / len(feasible)) if feasible else 0.0,
        # A 2/2 mission is not a 3/3 result.  Stratify the metric by mission size
        # so mixed scenario families cannot hide the denominator.
        "delivered_3of3_rate": (sum(1 for r in three_cube if r.get("delivered", 0) == 3) /
                                 len(three_cube)) if three_cube else 0.0,
        "three_cube_runs": len(three_cube),
        "mean_delivered": statistics.mean([r.get("delivered", 0) for r in results]) if n else 0,
        "failure_classes": dict(Counter(r.get("failure_class", "none") for r in results if r.get("outcome") != "success")),
        "planner_rejected_runs": sum(1 for r in results if r.get("failure_class") == "planner_rejected"),
        "physical_infeasible_runs": sum(1 for r in results if r.get("failure_class") == "physically_infeasible"),
        "mean_delivery_time": statistics.mean(delivery_times) if delivery_times else None,
        "median_delivery_time": statistics.median(delivery_times) if delivery_times else None,
        "mean_settled_completion_time": statistics.mean(settled_times) if settled_times else None,
        "median_settled_completion_time": statistics.median(settled_times) if settled_times else None,
        "mean_completion_time": statistics.mean(times) if times else None,
        "median_completion_time": statistics.median(times) if times else None,
        "p95_completion_time": pct(times, 0.95),
        "max_completion_time": max(times) if times else None,
    }
    for k in SAFETY_KEYS + ("deadlocks", "delivery_undone", "cube_exits", "replans", "recoveries", "estops",
                            "capture_failures", "collision_duration_s", "non_target_contact_duration_s",
                            "max_collision_duration_s", "max_non_target_contact_duration_s"):
        vals = [r.get(k, 0) or 0 for r in results]
        agg[k + "_total"] = sum(vals)
        agg[k + "_runs"] = sum(1 for v in vals if v)
    for k in ("est_pos_rmse", "est_heading_rmse_deg", "push_cross_track_p95", "max_occlusion_s"):
        vals = [r[k] for r in results if r.get(k) is not None]
        agg[k + "_mean"] = statistics.mean(vals) if vals else None
        agg[k + "_max"] = max(vals) if vals else None
    rec = [r for r in results if (r.get("recoveries") or 0) > 0]
    agg["recovery_success_rate"] = (sum(1 for r in rec if r.get("outcome") == "success") / len(rec)) if rec else None
    agg["replans_per_run"] = agg["replans_total"] / n if n else 0
    return agg


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--seeds", type=str, default="")
    ap.add_argument("--family", default="start_zone")
    ap.add_argument("--max-time", type=float, default=300.0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default="")
    ap.add_argument("--overhang", type=float, default=0.0,
                    help="allowed rover overhang beyond the effective field, mm (rules interpretation; default strict 0)")
    a = ap.parse_args(argv)
    seeds = [int(s) for s in a.seeds.split(",")] if a.seeds else list(range(a.start, a.start + a.n))
    t0 = time.time()
    jobs = [(s, a.family, a.max_time, a.overhang) for s in seeds]
    if a.workers > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(a.workers) as ex:
            results = list(ex.map(_work, jobs, chunksize=4))
    else:
        results = [_work(j) for j in jobs]
    agg = aggregate(results)
    agg["wall_s"] = round(time.time() - t0, 1)
    agg["family"] = a.family
    agg["overhang_mm"] = a.overhang
    print(json.dumps(agg, indent=1))
    fails = [r for r in results if r.get("outcome") != "success"]
    for r in fails[:40]:
        print(f"FAIL seed={r['seed']} class={r.get('failure_class')} delivered={r.get('delivered')}/{r.get('n_cubes')} "
              f"{r.get('error', '')} {r.get('detail', '')}")
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, "summary.json"), "w") as f:
            json.dump(agg, f, indent=1)
        with open(os.path.join(a.out, "results.jsonl"), "w") as f:
            for r in results:
                f.write(json.dumps(r) + "\n")
        with open(os.path.join(a.out, "failed_seeds.txt"), "w") as f:
            for r in fails:
                f.write(f"{r['seed']} {a.family} {r.get('failure_class')}\n")


if __name__ == "__main__":
    main()
