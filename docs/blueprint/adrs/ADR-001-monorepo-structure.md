---
id: ADR-001
title: Monorepo Structure for Multi-Platform MCU Projects
status: accepted
created: 2026-03-05
---

# ADR-001: Monorepo Structure for Multi-Platform MCU Projects

## Context

The lab produces firmware for multiple microcontroller families (ESP32, Arduino, STM32)
and a Python simulation that mirrors one of the embedded projects. These artifacts share
no build toolchain but do share documentation, CI configuration, and development
conventions. The team needed to decide whether to maintain separate repositories per
project or consolidate under a single repository.

Separate repositories would mean duplicated CI pipelines, fragmented documentation, and
difficulty keeping the simulation in sync with the firmware it models.

## Decision

All projects are housed in a single monorepo under `packages/`, organized by platform:

```
packages/
  esp32-projects/   # all ESP-IDF projects
  arduino-projects/ # (planned)
  stm32-projects/   # (planned)
  shared-libs/      # cross-platform shared code (planned)
```

A root `Makefile` (60+ targets) and `justfile` provide a unified entry point for
build, flash, test, lint, format, Docker, and CI operations across all packages.

## Consequences

**Positive**
- Single clone, single CI configuration, single pre-commit setup.
- Cross-project changes (e.g., updating clang-format rules) land in one PR.
- The simulation and the firmware it validates evolve together in the same PR.
- Dependency on shared libraries (when implemented) is a path reference, not a
  versioned package.

**Negative**
- ESP-IDF CI must filter on `packages/esp32-projects/**` path triggers to avoid
  rebuilding firmware on Python-only changes.
- New contributors must understand the multi-platform layout before making changes.
- Large monorepos with binary build artifacts (`.bin`, `.elf`) require disciplined
  `.gitignore` enforcement (handled via pre-commit hook).

## Alternatives Considered

- **Per-project repositories**: rejected due to CI duplication and sync burden between
  simulation and firmware.
- **Git submodules per platform**: rejected due to added complexity for contributors
  and no tangible benefit at current scale.
