Root-cause analysis (report only; you may add tools/ scripts and tests but do NOT edit lead-owned files
rover_strategy/coordination/supervisor.py, rover_strategy/rover/fsm.py, config.py, world.py). With the allowed-overhang
interpretation of 20 mm, seed 107 of family official_like produced a board exit:
`PYTHONPATH=. .venv/bin/python -m rover_strategy.simulation.monte_carlo --seeds 107 --family official_like --workers 1 --max-time 600 --overhang 20`
and `STALL=300 PYTHONPATH=. .venv/bin/python tools/debug_run.py 107 official_like 600` (note: debug_run uses overhang 0;
add an OVERHANG env option to tools/debug_run.py mirroring monte_carlo --overhang). Find exactly which rover/state/
command produced the exit and why the supervisor execution guard (Supervisor._execution_guard) and the push planner's
final-leg margin did not prevent it. Deliver docs/codex_log/B09_board_exit_analysis.md with evidence (times, poses,
states, commands) and a precise proposed patch (diff in the report) for the lead to review.
