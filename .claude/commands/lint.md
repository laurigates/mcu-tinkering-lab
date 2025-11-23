---
allowed-tools: Bash(make:*), Bash(cppcheck:*), Bash(ruff:*), Bash(clang-format:*)
argument-hint: [type]
description: Run linting and formatting checks on codebase
---

## Task
Run linting on the codebase: $ARGUMENTS

## Available Lint Types
- (no argument) - Run all linters: `make lint`
- `c` or `cpp` - Run C/C++ linting only: `make lint-c`
- `python` or `py` - Run Python linting only: `make lint-python`
- `format` - Format all code: `make format`
- `format-check` - Check formatting without modifying: `make format-check`

## Tools Used
- **C/C++**: cppcheck for static analysis, clang-format for formatting
- **Python**: ruff for linting and formatting

Report any issues found with file locations and descriptions.
For formatting issues, suggest running `make format` to auto-fix.
