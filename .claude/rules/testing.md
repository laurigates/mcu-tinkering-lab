# Testing Requirements

## Python Simulation Tests

```bash
cd packages/robocar/simulation
uv run pytest tests/ --cov
```

- Maintain test coverage for simulation code
- Run tests before committing Python changes

## C/C++ Firmware

- Static analysis via `cppcheck` (run in CI)
- Format checking via `clang-format --dry-run`
- **Host unit tests exist for robocar-unified** — 11 suites under
  `packages/robocar/unified/test/`, run natively with no ESP-IDF container:

  ```bash
  just robocar-unified::test
  ```

  `test/CMakeLists.txt` compiles the ESP-IDF-free modules straight from `main/`
  against the shims in `test/include`. They cover exactly what a bench cannot
  stage — the uint32 millisecond wrap at day 49, an exact brightness shift with
  no motion — which is why a regression there would otherwise reach hardware
  unnoticed.

  **No CI job runs them yet.** `.github/workflows/` only `ctest`s the Pico
  project, so a regression in these suites lands green. Local/CI parity gap, per
  `~/.claude/rules/local-ci-parity.md`; the gap is also flagged at the recipe
  itself in `packages/robocar/unified/justfile`.

## CI Verification

All PRs must pass:
- ESP32 firmware builds (matrix across projects)
- Python test suite with coverage
- Pre-commit hooks (format, lint, secret scanning)
