#!/usr/bin/env bash
# PostToolUse:Edit hook — run `make lint` and `make format` only when the edited
# file is a C/C++ or Python source. Prevents unrelated lint runs on every
# justfile, Markdown, or config edit.
#
# Reads the Claude Code hook payload on stdin and extracts
# `.tool_input.file_path` via jq. Files outside the relevant extensions
# produce exit 0 immediately so no lint chore runs.
set -euo pipefail

# Extract the edited file path from the hook payload
# If jq fails or the payload has no file_path, skip silently
file_path="$(jq -r '.tool_input.file_path // empty' 2>/dev/null || true)"

if [[ -z "$file_path" ]]; then
    exit 0
fi

case "$file_path" in
    *.c|*.h|*.cpp|*.hpp|*.py)
        cd "$CLAUDE_PROJECT_DIR"
        exec make lint format
        ;;
    *)
        exit 0
        ;;
esac
