#!/usr/bin/env bash
# Batch-regenerate NotebookLM reports for every active notebook in the
# mapping doc: submit all generations up front (they run server-side in
# parallel), then wait for each artifact and download it.
#
# Usage:
#   tools/notebooks-refresh.sh                 # briefing-doc for all active notebooks
#   FORMAT=study-guide tools/notebooks-refresh.sh
#   MAPPING=docs/reference/notebook-mapping.md OUTDIR=tmp/notebooklm tools/notebooks-refresh.sh
#
# Prereq: notebooklm login (one-time, OAuth stored at ~/.notebooklm).
# Cannot run in CI — auth lives on the local machine.

set -euo pipefail

MAPPING="${MAPPING:-docs/reference/notebook-mapping.md}"
FORMAT="${FORMAT:-briefing-doc}"
OUTDIR="${OUTDIR:-tmp/notebooklm}/${FORMAT}s"

if ! command -v notebooklm >/dev/null 2>&1; then
  echo "notebooklm CLI not found. Install with: pip install notebooklm-py" >&2
  exit 1
fi

if ! notebooklm auth check >/dev/null 2>&1; then
  echo "notebooklm is not authenticated. Run: notebooklm login" >&2
  exit 1
fi

mkdir -p "$OUTDIR"
TASKS_TSV="$OUTDIR/tasks.tsv"
: > "$TASKS_TSV"

# Extract backtick-wrapped UUIDs from the mapping doc's tables
# shellcheck disable=SC2016  # backticks are literal markdown, not expansion
ids=$(grep -oE '`[0-9a-f]{8}-[0-9a-f-]+`' "$MAPPING" | tr -d '`' | sort -u)

echo "== Submitting $FORMAT generation for $(wc -w <<<"$ids" | tr -d ' ') notebooks =="
for nid in $ids; do
  out=$(notebooklm generate report --format "$FORMAT" --notebook "$nid" --retry 3 --json 2>>"$OUTDIR/errors.log") || {
    echo "  SUBMIT-FAILED ${nid:0:8} (see $OUTDIR/errors.log)"; continue; }
  tid=$(python3 -c "import json,sys; print(json.load(sys.stdin).get('task_id',''))" <<<"$out")
  if [ -n "$tid" ]; then
    printf '%s\t%s\n' "$nid" "$tid" >> "$TASKS_TSV"
    echo "  submitted ${nid:0:8} -> $tid"
  else
    echo "  SUBMIT-FAILED ${nid:0:8}: $out"
  fi
  sleep 5
done

echo "== Waiting for artifacts and downloading to $OUTDIR =="
fail=0
while IFS=$'\t' read -r nid tid; do
  if notebooklm artifact wait "$tid" -n "$nid" --timeout 1800 >/dev/null; then
    if notebooklm download report "$OUTDIR/${nid:0:8}-$FORMAT.md" -a "$tid" -n "$nid" >/dev/null; then
      echo "  downloaded ${nid:0:8}-$FORMAT.md"
    else
      echo "  DOWNLOAD-FAILED ${nid:0:8} $tid"; fail=$((fail+1))
    fi
  else
    echo "  WAIT-FAILED ${nid:0:8} $tid"; fail=$((fail+1))
  fi
done < "$TASKS_TSV"

echo "== Done ($fail failure(s)). Skim the reports, then update sources per $MAPPING =="
exit $((fail > 0 ? 1 : 0))
