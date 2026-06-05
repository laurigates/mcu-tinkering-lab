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
- **Host unit tests exist for robocar-unified** — 17 suites under
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

### Host-based unit tests (shared-core pattern)

Extract the hardware-agnostic logic (parameter mapping, state machines, math)
into a `*_core.{c,h}` with **no ESP-IDF dependencies**, then compile that one
source of truth three ways — so the tested logic and the shipped logic are
literally the same compiled code, with no reimplementation and no drift:

1. **Firmware** — add `*_core.c` to the component's
   `idf_component_register(SRCS ...)`; `main.c` wraps it with the ADC/GPIO/LEDC
   I/O. The refactor is behavior-preserving.
2. **Host unit test** — `cc -std=c11 -Wall -Wextra -I main test/test_*.c
   main/*_core.c -lm`, a plain-assert `main()` returning non-zero on failure.
   No framework needed. Expose as a `just test` recipe. Pin down bound, clamp,
   and edge-case behaviour so a future ADC-driver migration can't silently
   change it.
3. **Host simulator** (optional, for fast iteration without flashing) — compile
   `*_core.c` to a shared lib (`cc -dynamiclib -O2 -o sim/lib*.dylib
   main/*_core.c`) and drive the real logic from Python via `ctypes`. Use an
   out-pointer for struct returns to stay ABI-safe. Fake only the genuinely
   hardware-bound transducers (e.g. webcam → ADC inputs, speakers → PWM output).

Keep `*_core` free of platform headers so the identical logic runs in firmware,
tests, and sim. Worked example: `packages/audio/kids-audio-toy` —
`main/audio_core.{c,h}` (mapping/modulation logic), `test/test_audio_core.c`
(`just test`), `sim/toy_sim.py` (webcam-in / speaker-out via ctypes, `just sim`).

## CI Verification

All PRs must pass:
- ESP32 firmware builds (matrix across projects)
- Python test suite with coverage
- Pre-commit hooks (format, lint, secret scanning)
