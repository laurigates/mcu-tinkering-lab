# Contributing to MCU Tinkering Lab

How to set up, build, test and submit changes in this monorepo.

## Contents

- [Setup](#setup)
- [Workflow](#workflow)
- [Build commands](#build-commands)
- [Testing](#testing)
- [Code style](#code-style)
- [Commit messages](#commit-messages)
- [Pull requests](#pull-requests)
- [Adding a project](#adding-a-project)
- [Documentation](#documentation)
- [CI/CD](#cicd)
- [Troubleshooting](#troubleshooting)

## Setup

Requirements:

- Docker, or Podman with `CONTAINER_CMD=podman`. ESP-IDF builds run in the
  `espressif/idf:v5.4` container, so no local ESP-IDF install is needed.
- [just](https://github.com/casey/just)
- [uv](https://github.com/astral-sh/uv) and Python 3.11+ for the simulation and
  Python tooling
- Git

```bash
git clone https://github.com/laurigates/mcu-tinkering-lab.git
cd mcu-tinkering-lab
just setup-all            # Docker images, dev tools, pre-commit hooks
just check-environment    # verify Docker and serial port setup
```

From a fork, clone the fork instead and add this repository as `upstream`.

Builds and `menuconfig` run in the container. Flashing and the serial monitor
run on the host, because USB passthrough into containers is unreliable on
macOS. To reach serial devices from inside the container anyway, uncomment the
`devices` and `privileged` entries in `docker-compose.yml`.

## Workflow

1. Branch from `main`: `feat/<topic>` or `fix/<topic>`.
2. Make the change, with tests for new hardware-independent logic.
3. Check it:

   ```bash
   just format
   just lint
   just <module>::build
   just <module>::test     # where the project has host tests
   ```

4. Commit with a [conventional commit message](#commit-messages). The
   pre-commit hooks run on commit.
5. Push and open a pull request against `main`.

## Build commands

Each project is a `just` module. `just list-projects` lists them, and
`just --list <module>` shows one project's recipes.

```bash
just <module>::build          # containerized build
just <module>::flash          # flash from the host (PORT=/dev/... overrides detection)
just <module>::monitor        # serial monitor
just <module>::menuconfig     # containerized menuconfig

just build-all                # robocar main + camera only
just clean-all                # clean every project build

just lint                     # cppcheck + ruff
just format                   # clang-format + ruff format
just format-check             # check only, no changes

just docker-dev               # interactive ESP-IDF shell
just docker-clean             # remove containers and volumes
```

## Testing

| Suite | Command |
|---|---|
| robocar-unified host tests | `just robocar-unified::test` |
| kids-audio-toy host tests | `just kids-audio::test` |
| balancebot host tests | `just balancebot::test` |
| Robocar simulation | `cd packages/robocar/simulation && uv sync && uv run pytest tests/ --cov` |
| All pre-commit hooks | `pre-commit run --all-files` |

Host tests compile hardware-independent firmware modules with the native
compiler and run them without a board. They cover cases a bench cannot stage,
such as the 32-bit millisecond counter wrapping at day 49.

To make a module host-testable, move its logic into a `*_core.{c,h}` with no
ESP-IDF headers. Compile that file into the firmware and into a plain-assert
test `main()`, and expose the test as a `just test` recipe.
`packages/audio/kids-audio-toy` is the worked example, and
`.claude/rules/testing.md` has the full pattern.

## Code style

| Language | Formatter | Linter / checker |
|---|---|---|
| C/C++ | clang-format: Google base, 4-space indent, 100 columns, Linux braces, `char *ptr` | cppcheck |
| Python | ruff format (the simulation uses 100 columns) | ruff, ty |

`.clang-format` and `ruff.toml` are the source of truth. Run `just format`
rather than formatting by hand.

The pre-commit hooks check:

- formatting and lint (clang-format, ruff, ty)
- secrets (gitleaks) and credential files
- flash recipes against partition tables (`tools/check-flash-recipes.py`)
- generated wiring tables and build guides against `hardware.toml` and
  `pin_config.h`
- trailing whitespace, end of file, YAML validity, build artifacts

Credentials (`credentials.h`, `wifi_config.h`, `*.key`, `*.secret`, `*.token`)
are gitignored and never committed. Use `sdkconfig.defaults` for non-sensitive
configuration.

## Commit messages

Commits follow [Conventional Commits](https://www.conventionalcommits.org/).
release-please builds release PRs and changelogs from them.

```
<type>(<scope>): <subject>

<body>
```

| Type | Use for |
|---|---|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `refactor` | Code change with no behaviour change |
| `perf` | Performance |
| `test` | Tests |
| `build` | Build system, dependencies |
| `ci` | CI workflows |
| `chore` | Other maintenance |

The scope is the project's `just` module name (`robocar-unified`,
`telegram`, `thinkpack-brainbox`) or an area such as `ci` or `docs`.

```
feat(robocar-main): add WiFi reconnection logic
fix(robocar-camera): fix memory leak in image capture
feat(i2c)!: switch the I2C protocol checksum to CRC8

BREAKING CHANGE: frames now carry CRC8 instead of CRC16.
```

## Pull requests

Before opening one:

- `just format` and `just lint` are clean
- the affected projects build and their tests pass
- documentation reflects the change
- the branch is up to date with `main`

The title uses the commit format. The description says what changed and why,
how it was tested (host tests, a bench boot, or neither), and any breaking
change.

During review, push changes as new commits rather than force-pushing, so
reviewers can see what changed since their last pass.

## Adding a project

```bash
./tools/scaffold/new-esp32-project.sh
```

The script asks for a name and domain folder, then generates `CMakeLists.txt`,
`main/`, `sdkconfig.defaults` and a justfile using the shared containerized
recipes. Then:

1. Register the module in the root `justfile`:
   `mod <name> 'packages/<domain>/<name>'`.
2. Add an entry to `.github/project-matrix.json` with `system`, `project`,
   `path` and `target`, plus `fetch_bluepad32: true` if it vendors bluepad32.
   `build.yml` builds it on any push or PR that touches its files. If its
   `EXTRA_COMPONENT_DIRS` reaches outside the project directory (for example
   `../../components`), list each of those component directories, relative to
   `packages/`, in the entry's `extra_paths`. The `check-component-extra-paths`
   pre-commit hook names any that are missing.
3. Run `python3 tools/check-flash-recipes.py` to check the flash recipe against
   the partition table.
4. Optional: add a `flasher.json` to list it in the
   [web flasher](https://laurigates.github.io/mcu-tinkering-lab/).
   `packages/audio/kids-audio-toy/flasher.json` shows the fields.

The scaffolding script's own "next steps" output still mentions a Makefile
and per-project workflows. Follow the list above instead.

`.claude/rules/containerized-builds.md` covers the justfile conventions and
the flash recipe checks in detail.

## Documentation

Each project's `README.md` covers what it does, the hardware it needs, how to
build, flash and configure it, and the license. Wiring goes in `WIRING.md`.

Architecture decisions go in `docs/decisions/` as `ADR-NNN-<title>.md`, and
requirements in `docs/requirements/` as `PRD-NNN-<title>.md`. See
[docs/README.md](docs/README.md).

## CI/CD

| Workflow | Purpose |
|---|---|
| `build.yml` | Builds every changed firmware project (ESP-IDF, ESPHome, Pico SDK), discovered from `.github/project-matrix.json` |
| `test.yml` | Pre-commit, pytest, cppcheck, format check |
| `build-firmware.yml` | On release: builds firmware, attaches binaries, deploys the web flasher, and fetches every published file to verify it |
| `release-please.yml` | Release PRs from conventional commits |
| `hardware-check.yml`, `build-guide-check.yml`, `schematics-check.yml` | Generated docs and schematics match their sources |
| `refresh-idf-locks.yml` | Monthly refresh of tracked `dependencies.lock` files |

## Troubleshooting

**Serial port permission denied (Linux).** Add the user to `dialout` and log in
again: `sudo usermod -a -G dialout $USER`.

**Port not detected.** `just list-devices` lists connected boards and USB-serial
adapters. Set `PORT=/dev/...` explicitly when more than one is connected.

**Out of disk space.** `just clean-all`, then `just docker-clean`.

## Questions and issues

Bugs, feature requests and questions go to
[Issues](https://github.com/laurigates/mcu-tinkering-lab/issues).
