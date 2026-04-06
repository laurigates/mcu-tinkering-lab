---
name: lint
description: Run linting and formatting checks on codebase
argument-hint: "[type]"
allowed-tools: Bash(just:*), Bash(cppcheck:*), Bash(ruff:*), Bash(clang-format:*)
---

# Lint and Format Codebase

Run linting on the codebase: $ARGUMENTS

## Available Lint Types
- (no argument) - Run all linters: `just lint`
- `c` or `cpp` - Run C/C++ linting only: `just lint-c`
- `python` or `py` - Run Python linting only: `just lint-python`
- `format` - Format all code: `just format`
- `format-check` - Check formatting without modifying: `just format-check`

## Tools Used
- **C/C++**: cppcheck for static analysis, clang-format for formatting
- **Python**: ruff for linting and formatting

Report any issues found with file locations and descriptions.
For formatting issues, suggest running `just format` to auto-fix.
