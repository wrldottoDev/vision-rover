# Codex worker automation (GPT-Luna implementers + GPT-Luna reviewer; Opus is final auditor)

- Two lanes, each in its own git worktree/branch so they never conflict:
  lane A `../V1-codex-a` branch `codex-auto` (simulator/judge/motor model/MC triage),
  lane B `../V1-codex-b` branch `codex-auto-b` (controllers/navigation/telemetry).
- `run_lane.sh LANE WORKTREE QUEUE_DIR` runs each task file: implementer (`gpt-5.6-luna`, workspace-write, high
  effort) -> full test suite -> commit "[UNREVIEWED]" -> reviewer (`gpt-5.6-luna`, read-only) -> commit review.
  Stops on the first quota/rate-limit error; status in `docs/codex_log/STATUS_<lane>` of the worktree.
- Reports: `docs/codex_log/<task>_report.md`, `<task>_review.md`, `<task>_tests.txt`, `runner_<lane>.log`.
- Workers may not edit `rover/fsm.py`, `coordination/supervisor.py`, `config.py`, `world.py` (lead-owned).
- Nothing reaches `main` without Opus review: inspect with `git log main..codex-auto`, cherry-pick good commits.

Launch (detached):
```
nohup tools/codex_auto/run_lane.sh A ../V1-codex-a tools/codex_auto/queue_A > /tmp/codex_lane_A.out 2>&1 &
```
