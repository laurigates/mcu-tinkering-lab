#!/usr/bin/env bash
# Local NotebookLM refresh helper. Reads the active notebook IDs from
# docs/reference/notebook-mapping.md and reports source counts + recently
# merged ADR/PRD files that may need to be seeded.
#
# Prereq: notebooklm login (one-time, OAuth stored at ~/.notebooklm)

set -euo pipefail

MAPPING="${MAPPING:-docs/reference/notebook-mapping.md}"
SINCE="${SINCE:-1 month ago}"

if ! command -v notebooklm >/dev/null 2>&1; then
  echo "notebooklm CLI not found. Install with: pip install notebooklm-py" >&2
  exit 1
fi

if ! notebooklm status >/dev/null 2>&1; then
  echo "notebooklm is not authenticated. Run: notebooklm login" >&2
  exit 1
fi

printf '%s\n' "== Active notebooks (source counts) =="
# Extract backtick-wrapped UUIDs from the mapping doc's table
grep -oE '`[0-9a-f]{8}-[0-9a-f-]+`' "$MAPPING" \
  | tr -d '`' \
  | sort -u \
  | while read -r nid; do
      title=$(notebooklm list --json | python3 -c "
import json, sys
nid = sys.argv[1]
for nb in json.load(sys.stdin)['notebooks']:
    if nb['id'] == nid:
        print(nb['title']); break
" "$nid")
      count=$(notebooklm source list --notebook "$nid" --json \
        | python3 -c "import json,sys; print(len(json.load(sys.stdin).get('sources', [])))")
      printf '  %-8s  %3s sources  — %s\n' "${nid:0:8}" "$count" "${title:-<unknown>}"
    done

printf '\n%s\n' "== ADR/PRD files changed since $SINCE =="
git log --since="$SINCE" --name-only --pretty=format: \
  -- docs/decisions docs/requirements \
  | sort -u | grep -v '^$' | sed 's/^/  /' || true

printf '\n%s\n' "== Next steps =="
cat <<'EOF'
  For each notebook that looks stale or drifted:
    notebooklm use <id>
    notebooklm generate report --format briefing-doc
    notebooklm download report tmp/<id>-briefing.md

  To add sources from a new ADR/PRD:
    notebooklm source add --notebook <id> docs/decisions/ADR-NNN-....md

  Full mapping: docs/reference/notebook-mapping.md
EOF
