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
#
# Only files of this repository are touched: the project checkout or one of
# its worktrees (.claude/worktrees/*), tracked or new. A file in any other
# repository, including a clone under the project's tmp/, or in no repository
# is left alone; ruff would apply its defaults to code with another style
# (issue #1111). tools/test_post_edit_lint_hook.py pins this.
#
# Which ruff (issue #1130): the one on PATH, with its fix step gated on its
# version. When `ruff --version` matches the ruff-pre-commit rev pinned in the
# repository's .pre-commit-config.yaml, the hook formats and fixes; otherwise
# it only formats. Another version's fixes can delete code the pinned version
# keeps (.claude/rules/python-lint.md); a formatting difference is redone by
# the pinned ruff-format hook before the commit. Calling
# `pre-commit run ruff --files` instead was rejected: it took 520 ms per edit
# against 33 ms (mean of 5 runs, 2026-11-13), and its `--fix` hook args remove
# the not-yet-used imports this hook keeps.
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
    *.c|*.h|*.cpp|*.hpp|*.py) ;;
    *) exit 0 ;;
esac

# Find each path's repository from the path alone. An inherited GIT_DIR or
# GIT_WORK_TREE would resolve every path to that one repository and let the
# scope check below pass for a file in another one.
unset GIT_DIR GIT_WORK_TREE GIT_COMMON_DIR GIT_INDEX_FILE

# The repository's shared git directory: the same for the main checkout and
# every worktree of it, and different for any other clone.
git_common_dir() {
    local dir
    dir="$(git -C "$1" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)" || return 1
    realpath "$dir"
}

project_dir="${CLAUDE_PROJECT_DIR:-$(dirname "${BASH_SOURCE[0]}")/../..}"
project_git="$(git_common_dir "$project_dir")" || exit 0
file_git="$(git_common_dir "$(dirname "$file_path")")" || exit 0
if [[ "$file_git" != "$project_git" ]]; then
    exit 0
fi

case "$file_path" in
    *.c|*.h|*.cpp|*.hpp)
        # Single-source the clang-format + cppcheck invocation via tools/cquality.sh
        # (same flags as the justfile lint-c / CI). Fully non-blocking.
        repo_root="$(git -C "$(dirname "$file_path")" rev-parse --show-toplevel 2>/dev/null || true)"
        cq="${repo_root:+$repo_root/}tools/cquality.sh"
        if [[ -x "$cq" ]]; then
            "$cq" format-file "$file_path" || true
            "$cq" lint-file "$file_path" || true
        fi
        ;;
    *.py)
        # The hook runs between the Edits of one change, so an import or a
        # `# noqa` added before the code that needs it must survive: F401
        # (unused import) and RUF100 (unused noqa) are reported, not fixed.
        # The pre-commit ruff hook applies every fix before a commit.
        if command -v ruff >/dev/null 2>&1; then
            ruff format "$file_path" >/dev/null || true
            repo_root="$(git -C "$(dirname "$file_path")" rev-parse --show-toplevel 2>/dev/null || true)"
            pinned="$(sed -n '/astral-sh\/ruff-pre-commit/,/rev:/s/^[[:space:]]*rev:[[:space:]]*"\{0,1\}v\{0,1\}\([0-9][0-9.]*\).*/\1/p' \
                "$repo_root/.pre-commit-config.yaml" 2>/dev/null || true)"
            found="$(ruff --version 2>/dev/null || true)"
            found="${found#ruff }"
            if [[ -n "$pinned" && "$found" == "$pinned" ]]; then
                ruff check --fix --unfixable F401,RUF100 "$file_path" || true
            else
                echo "post-edit-lint: ruff ${found:-?} on PATH, pre-commit pins ${pinned:-?}; formatted, no fixes" >&2
            fi
        fi
        ;;
esac
