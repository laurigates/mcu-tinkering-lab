# Testing Requirements

## Python Simulation Tests

```bash
cd packages/robocar/simulation
uv run pytest tests/ --cov
```

- Maintain test coverage for simulation code
- Run tests before committing Python changes

## Schematics

```bash
just schematics::test      # router, metrics and symbol tests
just schematics::metrics   # per-circuit routing metrics; compare against the pins in test_routing.py
```

Run both after touching `docs/schematics/` (`components.py`, `routing.py`,
`circuits/*.py`). See [`schematic-layout.md`](schematic-layout.md).

## KiCad Carriers

```bash
just kicad::test                  # tools/kicad unit tests (stdlib unittest; no KiCad)
just kicad::check                 # what carrier-check.yml runs in CI, KiCad-free
just <project>::carrier-check     # local, KiCad 10.0.6: ERC, DRC parity, netlist parity,
                                  # board check, then the check stamp CI verifies
just <project>::perfboard-check   # the perfboard drawing and wire list are current
```

Run `kicad::test` after touching `tools/kicad/`, the parts registry or a
`hardware.toml` with carrier entries. Run `carrier-check` after anything that
changes a carrier's schematic or board, and commit the stamp it writes. The
unit tests use synthetic boards and stamps and never read the committed board
or stamp, so the `kicad-carrier` pre-commit hook (which `auto-fix.yml` can act
on) never asks for a board edit; `carrier-check.yml` checks those, and only a
local KiCad run can fix them. Tests checked against #749's netlist export
build from a frozen copy of the balancebot inputs (`tools/kicad/testfixture.py`),
not the live project. The suite does hold the committed schematic and perfboard
files equal to what their generators write, so after a legitimate pin move run
both `just <project>::carrier-schematic` and `just <project>::perfboard` before
the unit suite and the hook pass. See the `carrier-pcb` skill and ADR-027.

## C/C++ Firmware

- Static analysis via `cppcheck` (run in CI)
- Format checking via `clang-format --dry-run`
- **Host unit tests exist for robocar-unified** — the suites under
  `packages/robocar/unified/test/` (one `add_test` each in its
  `CMakeLists.txt`; the recipe's ctest summary gives the current count), run
  natively with no ESP-IDF container:

  ```bash
  just robocar-unified::test
  ```

  `test/CMakeLists.txt` compiles the ESP-IDF-free modules straight from `main/`
  against the shims in `test/include`. They cover exactly what a bench cannot
  stage — the uint32 millisecond wrap at day 49, an exact brightness shift with
  no motion — which is why a regression there would otherwise reach hardware
  unnoticed.

  `.github/workflows/_ci-build-esp32.yml`'s Host unit tests step runs this on
  every PR touching the project, probing both `test/` and `tests/host/` and
  installing `libcjson-dev` so `test_gemini_parse` builds rather than silently
  skipping (issue #482). The same step also runs `packages/audio/kids-audio-toy`'s
  plain-`cc` host suite (`just kids-audio::test`).

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
