---
reviewed: 2026-11-03
---

# Testing Requirements

## Python Simulation Tests

```bash
cd packages/robotics/robocar/simulation
uv sync --extra dev          # the core (numpy, pymunk, pyyaml) and the test tools
uv run pytest tests/ --cov
```

- Maintain test coverage for simulation code. CI enforces
  `--cov-fail-under=25` on the core and `dev` dependencies; the plant's tests
  meet it, and no module is omitted from the measurement to meet it (ADR-036)
- Run tests before committing Python changes
- The plant (`robocar_sim.plant`) must stay deterministic: `tests/test_plant.py`
  and `tests/test_plant_stepper.py` compare two runs in-process and across
  processes
- `ty` ignores unresolved imports there, so `tests/test_imports.py` is what
  fails when a module imports one that was removed. A module that needs a
  `viz`-extra package is listed with it in that test and skipped in CI, which
  does not install `viz`

## Suites That Create a Git Repository

A test module that runs `git init` in a temporary directory sets
`setUpModule = gitenv.isolate` (`tools/gitenv.py`), and a pytest project
deletes every `GIT_*` variable in an autouse conftest fixture
(`docs/schematics/conftest.py`): a hook run during a commit from a linked
worktree inherits `GIT_DIR` and `GIT_INDEX_FILE`, which otherwise send those
git calls to the repository being committed (#1123). `tools/test_git_env.py`
fails for a suite that does neither.

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

## Printed Mechanisms

```bash
just mechanism::test                  # tools/mechanism: clash sweep, servo envelopes, one render
just mechanism::check <design dir>    # a design's clash sweep and walls; must exit 0
just mechanism::animate <design dir>  # the same clash test between gate poses, as AV1 video
```

Run `mechanism::test` after touching `tools/mechanism/` or
`tools/enclosure/enclosure/thickness.py` (both use that gate; its own tests are
`tools/enclosure/tests/test_thickness.py`). The render test skips without
off-screen VTK (`just mechanism::setup`). See the `printed-mechanism` skill.

## Printed-Part Standard

```bash
just cadstd::test        # tools/cadstd: tables, grid, cutters, coupon (build123d)
just cadstd::test-core   # the stdlib tables only; the `cadstd` pre-commit hook
```

Run `cadstd::test` after touching `tools/cadstd/`, then `enclosure::test` and
`mechanism::test`, which import it. A value change in `cadstd` moves every
part built from it: `test_cadstd.py` pins the values existing parts use, so
change the pin and say which parts move (ADR-035).

## C/C++ Firmware

- Static analysis via `cppcheck` (run in CI)
- Format checking via `clang-format --dry-run`
- **Host unit tests exist for robocar** — the suites under
  `packages/robotics/robocar/test/` (one `add_test` each in its
  `CMakeLists.txt`; the recipe's ctest summary gives the current count), run
  natively with no ESP-IDF container:

  ```bash
  just robocar::test
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
- **Component host suites** — every `packages/<...>/components/<name>/test/`
  (the thinkpack components and xbox-switch-bridge's `status_led`) is a plain
  `make` suite, run together by:

  ```bash
  just components::test
  ```

  CI runs it as the `components` entry of `.github/python-suites.json`, on any
  change under `packages/components/` or a listed component. `unity_compat.h`,
  `esp_log.h` and `esp_err.h` live once in `packages/components/host-test/`;
  a suite keeps only the stubs no other suite needs (`status_led`'s driver
  mocks). A new component suite is a `test/Makefile` whose default target
  builds and runs it; `tools/test_python_suites.py` fails if a component's
  `test/` has no Makefile, if the `components` suite's paths do not select
  the component, or if a header is copied between suites (#1077).

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

### Reading a mutation check against a host suite

The suites' `test_assert()` prints `FAIL:` and then calls `assert()`, which
aborts. With stdout redirected to a file or a pipe it is block-buffered, and
`abort()` does not flush it, so the `FAIL:` line is usually lost. A mutation
check that greps for `FAIL` therefore reads a killed mutant as a survivor. Read
the exit status (134 for the abort) and the last `[n] Running:` line, which
names the test that failed. New suites call `fflush(stdout)` before the
`assert()` (`test_ota_park.c`, `test_ota_announce.c`). Observed 2026-11-08: a
`grep FAIL` showed nothing for a mutant that had in fact failed
`text_skips_thought_parts`.

## CI Verification

All PRs must pass:
- ESP32 firmware builds (matrix across projects)
- Python test suite with coverage
- Pre-commit hooks (format, lint, secret scanning)
- The Python Suites job in `test.yml`: each uv project suite in
  `.github/python-suites.json` (`enclosure::test`, `mechanism::test`,
  `cadstd::test`, `glitchtip::test`, `robocar-twin::test`,
  `synth::tts-test`, the live probe's offline tests, and the make-only
  `components::test`, which has no uv `dir`) when a path it lists
  changes, with `UV_LOCKED=1` so a stale `uv.lock` fails rather than
  re-resolving. A new uv project with tests gets an entry there; the
  `python-suites` pre-commit hook (`tools/test_python_suites.py`) fails
  until it has one.
