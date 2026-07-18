#!/usr/bin/env bash
# Single source of truth for C/C++ lint (cppcheck) and format (clang-format).
#
# Consumed by:
#   - justfile        : lint-c / format-c / format-check-c recipes
#   - CI test.yml     : C/C++ Linting + Format Check jobs
#   - post-edit hook  : .claude/hooks/post-edit-lint.sh (single-file)
#
# Before this script, the find-expression, the exclude list, and the cppcheck
# flag set were copy-pasted across all three call sites (5-7 copies) and had
# already drifted (CI omitted the `.venv` exclude). Owning them here once is the
# offload-to-deterministic-substrate + local-ci-parity fix: change the exclude
# list in ONE place and every consumer picks it up.
#
# Usage:
#   tools/cquality.sh lint [--advisory]   # cppcheck the tree (blocking; --advisory never fails)
#   tools/cquality.sh format              # clang-format -i the tree
#   tools/cquality.sh format-check        # clang-format --dry-run --Werror (blocking)
#   tools/cquality.sh lint-file   FILE    # cppcheck a single file (advisory)
#   tools/cquality.sh format-file FILE    # clang-format -i a single file
set -euo pipefail

# --- Canonical, single-definition config -------------------------------------

# Directories excluded from all C/C++ checks: vendored/generated/build trees.
EXCLUDES=(
    '*/managed_components/*'
    '*/components/esp-idf-lib/*'
    '*/external/*'
    '*/build/*'
    '*/.esphome/*'
    '*/.venv/*'
)

# cppcheck flags shared by the whole-tree lint and the single-file hook.
# shellcheck disable=SC2054  # --enable=... commas are cppcheck flag syntax, not element separators
CPPCHECK_FLAGS=(
    --enable=warning,style,performance,portability
    --suppress=missingIncludeSystem
    --suppress=unmatchedSuppression
    --suppress=unusedStructMember
    --inline-suppr
    --template=gcc
)

# Print NUL-delimited list of in-scope C/C++ files under packages/.
_c_files() {
    local find_args=(packages -type f
        '(' -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' ')')
    local ex
    for ex in "${EXCLUDES[@]}"; do
        find_args+=(! -path "$ex")
    done
    find "${find_args[@]}" -print0
}

_require() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "Error: $1 not found — $2" >&2
        exit 1
    }
}

# --- Subcommands -------------------------------------------------------------

cmd="${1:-}"
shift || true

case "$cmd" in
    lint)
        # Default blocks (exit 1 on findings); --advisory reports without failing
        # (CI uploads tmp/cppcheck-report.txt as an artifact instead of failing).
        exitcode=1
        [[ "${1:-}" == "--advisory" ]] && exitcode=0
        _require cppcheck "brew install cppcheck (or apt-get install cppcheck)"
        mkdir -p tmp
        _c_files | xargs -0 cppcheck "${CPPCHECK_FLAGS[@]}" \
            --error-exitcode="$exitcode" 2>&1 | tee tmp/cppcheck-report.txt
        echo "C/C++ lint complete"
        ;;
    format)
        _require clang-format "brew install clang-format"
        _c_files | xargs -0 clang-format -i --style=file
        echo "C/C++ code formatted"
        ;;
    format-check)
        _require clang-format "brew install clang-format"
        _c_files | xargs -0 clang-format --dry-run --Werror --style=file
        echo "C/C++ formatting check passed"
        ;;
    lint-file)
        f="${1:?usage: cquality.sh lint-file FILE}"
        command -v cppcheck >/dev/null 2>&1 &&
            cppcheck "${CPPCHECK_FLAGS[@]}" --quiet "$f" || true
        ;;
    format-file)
        f="${1:?usage: cquality.sh format-file FILE}"
        command -v clang-format >/dev/null 2>&1 &&
            clang-format -i --style=file "$f"
        ;;
    *)
        echo "usage: cquality.sh {lint [--advisory]|format|format-check|lint-file FILE|format-file FILE}" >&2
        exit 2
        ;;
esac
