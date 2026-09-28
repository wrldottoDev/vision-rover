Monte Carlo triage (analysis task; code edits only in tools/ and a new report). Run:
`PYTHONPATH=. .venv/bin/python -m rover_strategy.simulation.monte_carlo --n 24 --family official_like --workers 6 --max-time 300 --out runs/mc_codex`
(it may take long; if it exceeds your time, use --n 12). For the failing seeds, reproduce with
`PYTHONPATH=. .venv/bin/python tools/debug_run.py SEED official_like 300` and `tools/snapshot.py`, and classify each
failure by ROOT CAUSE (subsystem + mechanism, with the event evidence). Write docs/codex_log/A05_mc_triage.md with:
per-seed table (seed, class, root cause, evidence lines, proposed fix and which file), aggregated statistics, and a
ranked list of the top 5 fixes by expected impact. Do NOT edit rover_strategy/ in this task.
