#!/bin/bash
# Usage: run_lane.sh LANE WORKTREE_DIR QUEUE_DIR
# Runs every task file in QUEUE_DIR (sorted) with a GPT-Luna implementer inside WORKTREE_DIR, commits the result,
# then runs a GPT-Luna reviewer on that commit and commits the review. Stops at the first quota/rate-limit error.
set -u
LANE="$1"; WT="$2"; QUEUE="$3"
HERE="$(cd "$(dirname "$0")" && pwd)"
LOG="$WT/docs/codex_log"; mkdir -p "$LOG"
STATUS="$LOG/STATUS_$LANE"
IMPL_MODEL="${IMPL_MODEL:-gpt-5.6-luna}"
REVIEW_MODEL="${REVIEW_MODEL:-gpt-5.6-luna}"
quota_hit() { grep -qiE "usage limit|rate limit|rate_limit|quota|429|hit your|insufficient|exceeded" "$1"; }
echo "RUNNING $(date)" > "$STATUS"
cd "$WT" || exit 1
while true; do
  task=""
  for f in $(ls "$QUEUE"/*.md 2>/dev/null | sort); do   # re-scan each time: tasks can be added while running
    [ -f "$LOG/$(basename "$f" .md).done" ] || { task="$f"; break; }
  done
  [ -z "$task" ] && break
  name="$(basename "$task" .md)"
  echo "[$(date)] $LANE start $name" >> "$LOG/runner_$LANE.log"
  before="$(.venv/bin/python -m pytest -q -p no:cacheprovider tests/ 2>&1 | tail -1)"
  codex exec -m "$IMPL_MODEL" -c model_reasoning_effort=high -s workspace-write --skip-git-repo-check -C "$WT" \
      -o "$LOG/${name}_report.md" "$(cat "$HERE/preamble.md" "$task")" < /dev/null > "$LOG/${name}_raw.log" 2>&1
  rc=$?
  if [ $rc -ne 0 ] && quota_hit "$LOG/${name}_raw.log"; then
    echo "QUOTA_EXHAUSTED $(date) during $name (impl)" > "$STATUS"; git reset -q --hard; exit 0
  fi
  after="$(.venv/bin/python -m pytest -q -p no:cacheprovider tests/ 2>&1 | tail -1)"
  printf "tests before: %s\ntests after: %s\ncodex exit: %s\n" "$before" "$after" "$rc" > "$LOG/${name}_tests.txt"
  git add -A && git commit -q -m "codex-auto($LANE): $name [UNREVIEWED]

tests before: $before
tests after:  $after
implementer: $IMPL_MODEL (exit $rc)" || true
  sha="$(git rev-parse --short HEAD)"
  codex exec -m "$REVIEW_MODEL" -c model_reasoning_effort=high -s read-only --skip-git-repo-check -C "$WT" \
      -o "$LOG/${name}_review.md" "$(cat "$HERE/review_preamble.md") Review commit $sha (run: git show $sha). Task was:
$(cat "$task")" < /dev/null > "$LOG/${name}_review_raw.log" 2>&1
  rc2=$?
  touch "$LOG/${name}.done"
  git add -A && git commit -q -m "codex-auto($LANE): review of $name ($sha)" || true
  echo "[$(date)] $LANE done $name impl_rc=$rc review_rc=$rc2 $after" >> "$LOG/runner_$LANE.log"
  if [ $rc2 -ne 0 ] && quota_hit "$LOG/${name}_review_raw.log"; then
    echo "QUOTA_EXHAUSTED $(date) during $name (review)" > "$STATUS"; exit 0
  fi
done
echo "QUEUE_EMPTY $(date)" > "$STATUS"
