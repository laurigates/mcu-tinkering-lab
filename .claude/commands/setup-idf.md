---
description: Set up ESP-IDF development environment (Docker-based)
allowed-tools: Bash(docker:*), Bash(just:*)
---

# Setup ESP-IDF Development Environment

All ESP-IDF projects build inside Docker containers. No local ESP-IDF installation needed.

## Steps

1. Check the current environment:
   ```bash
   just check-environment
   ```

2. Build the Docker development images:
   ```bash
   just docker-build
   ```

3. Install host-side development tools (linters, formatters, pre-commit):
   ```bash
   just install-dev-tools
   ```

4. Or do both at once:
   ```bash
   just setup-all
   ```

## Verify

Test that a project builds in the container:
```bash
just webserver::build
```

## Host-Side Tools

These run natively (not in container):
- `esptool` — for flashing firmware (`pip install esptool`)
- `clang-format` — C/C++ formatting (`brew install clang-format`)
- `cppcheck` — C/C++ linting (`brew install cppcheck`)
- `ruff` — Python linting (`pip install ruff`)
- `pre-commit` — git hooks (`pip install pre-commit`)

## Interactive Container Shell

For debugging build issues:
```bash
just docker-dev          # Root-level interactive shell
just webserver::shell    # Project-specific shell
```
