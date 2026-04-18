#!/usr/bin/env bash
# PostToolUse:Edit hook — lint and format only the edited file.
#
# Previous versions ran `just lint format` which walks the full tree and
# reformats unrelated C files (flattening HID descriptor tables etc.). This
# scopes the work to the single edited path so the author's intent elsewhere
# in the tree is preserved.
#
# Reads the Claude Code hook payload on stdin and extracts
# `.tool_input.file_path` via jq. Non-C/Python paths exit 0 immediately.
set -euo pipefail

file_path="$(jq -r '.tool_input.file_path // empty' 2>/dev/null || true)"

if [[ -z "$file_path" || ! -f "$file_path" ]]; then
    exit 0
fi

# Skip files in vendored/generated trees even if they are C/Python.
case "$file_path" in
    */managed_components/*|*/components/esp-idf-lib/*|*/external/*|*/build/*|*/.esphome/*|*/.venv/*)
        exit 0
        ;;
esac

case "$file_path" in
    *.c|*.h|*.cpp|*.hpp)
        if command -v clang-format >/dev/null 2>&1; then
            clang-format -i --style=file "$file_path"
        fi
        if command -v cppcheck >/dev/null 2>&1; then
            cppcheck \
                --enable=warning,style,performance,portability \
                --suppress=missingIncludeSystem \
                --suppress=unmatchedSuppression \
                --suppress=unusedStructMember \
                --inline-suppr \
                --quiet \
                --template=gcc \
                "$file_path" || true
        fi
        ;;
    *.py)
        if command -v ruff >/dev/null 2>&1; then
            ruff format "$file_path" >/dev/null || true
            ruff check --fix "$file_path" || true
        fi
        ;;
    *)
        exit 0
        ;;
esac
