---
name: build-guide
description: Generate a printable Typst build guide (source + PDF) for an ESP32/MCU project by analyzing its docs, source, and schematic
argument-hint: "<project-path>"
user-invocable: true
allowed-tools: Read, Write, Edit, Grep, Glob, Bash, Agent
---

## Task

Generate or update a printable **build guide** for the project at `$1` (relative
path under `packages/<domain>/`). The guide is a Typst document plus a rendered
PDF, written to `<project-path>/docs/build-guide.{typ,pdf}`, aimed at someone
assembling and flashing the hardware from scratch.

It differs from the other doc skills by audience and format:
- `wiring-doc` → `WIRING.md`: terse pin reference for developers.
- `project-readme` → `README.md`: quick-start for the repo.
- `build-guide` → `docs/build-guide.pdf`: an at-the-bench, print-ready guide
  that pulls BOM + wiring + power + assembly + flash + checkout into one styled
  document.

## Process

1. **Read the project sources** to gather everything the guide needs. Prefer
   authoritative sources over prose:
   - `main/pin_config.h` (or `main/*.h`) — **authoritative** GPIO / bus /
     channel assignments. Pin numbers come from here, not the README.
   - `WIRING.md` — existing wiring tables, I²C topology, power diagram.
   - `README.md`, `CLAUDE.md` — purpose, architecture, provisioning, OTA notes.
   - `sdkconfig.defaults` — target chip, PSRAM/flash size, brown-out, mDNS,
     stack sizes, console. Surface any load-bearing settings in troubleshooting.
   - `justfile` — real build / flash / monitor recipe names and flash offsets.
   - `partitions.csv` — flash layout / OTA partitions and offsets.
   - `main/CMakeLists.txt` — `REQUIRES` reveals components (camera, mdns, ota…).
   - Any schematic image under `docs/schematics/images/<name>.png` — embed it.

2. **Identify the board** and its constraints (PSRAM mode, 3.3 V-only GPIOs,
   native USB-Serial-JTAG vs external adapter, GPIO budget, strapping pins).

3. **Write `<project-path>/docs/build-guide.typ`** using the shared template
   (see below). Fill each section from real project data; omit sections the
   project doesn't use rather than padding with placeholders.

4. **Compile the PDF** and **link both files from the README** (see Compiling
   and Wiring up sections).

## Using the shared template

Styling and helpers live in **`tools/typst/build-guide.typ`** so all guides look
consistent and improve together. Import it and drive the document with
`#show: guide.with(...)`:

```typst
#import "../../../../tools/typst/build-guide.typ": guide, callout, htable, theme

#show: guide.with(
  title: "<project-name>",
  subtitle: "<one-line what it is>",
  version: "<version.txt contents, if any>",
  intro: [ short paragraph for the title page ],
  meta: (
    ("Target MCU", [ESP32-S3 (8 MB PSRAM / flash)]),
    ("Toolchain", [ESP-IDF v5.4 (containerized)]),
  ),
  difficulty: [Intermediate · \~2–3 h],
  header-right: "<board name>",
  footer-note: [ Pin data mirrors `main/pin_config.h`, which is authoritative. ],
)

= 1 · Overview
...
```

The relative import path assumes the guide lives at
`packages/<domain>/<project>/docs/`; from there `../../../../tools/typst/...`
reaches the template. Adjust `../` depth for other nesting.

**Helpers exported by the template:**
- `guide(...)` — page/text/heading styling + title page + table of contents.
  Applied via `#show: guide.with(...)`.
- `callout(title, body, kind: "info")` — soft left-barred box. `kind` is one of
  `info`, `ok`, `warn`, `danger`, `purple`. Override with `bar:`/`fill:` colors.
- `htable(cols, header, ..rows, aligns: none)` — accent-header, zebra-body
  table. `header` and each row are arrays of content cells.
- `theme` — the color dictionary (`theme.accent`, `theme.muted`, `theme.rule`…).

Do **not** re-declare page setup, fonts, heading styles, or table colors in the
project file — inherit them from the template so a future style change is a
one-file edit.

## Section structure

Use numbered level-1 headings so the table of contents reads as a build order.
Include a section only when the project uses it:

1. **Overview** — what it is, architecture in 1–2 paragraphs, `callout`s for the
   headline facts (e.g. core split, "what you get").
2. **Bill of Materials** — `htable` of Qty / Component / Notes, then a short
   "Tools required" paragraph. Flag voltage gotchas with a `warn` callout.
3. **System Architecture** — embed the schematic via `#figure(image(...))`;
   explain the bus/topology in prose.
4. **Wiring Reference** — subsections of `htable`s derived from `pin_config.h`:
   GPIO map, I²C topology, PWM/expander channel map, sensor pinouts.
5. **Power** — rails, source, and a `danger` callout for the common-ground rule.
6. **Assembly Steps** — an ordered `+` list, power-first, ending with a
   pre-power-up continuity check.
7. **Build & Flash** — real `just <module>::*` recipes in a code block, flash
   offsets table (from justfile / partitions.csv), download-mode `warn` callout.
8. **First Boot & Provisioning** — WiFi provisioning (Improv/creds), mDNS
   hostname, AI backend, OTA.
9. **Functional Checkout** — an `htable` of tick-box (`☐`) checks vs expected
   results, ordered to match assembly.
10. **Troubleshooting** — `htable` of Symptom / cause & fix, seeded from the
    board's known failure modes and load-bearing sdkconfig settings.

End with a small muted footer line pointing at the authoritative sources
(`pin_config.h`, any ADR) and the regenerate command.

## Compiling

Typst is not part of the container toolchain. If `typst` is missing, install the
CLI from crates.io (GitHub release binaries are blocked by the egress proxy):

```bash
CARGO_HTTP_CAINFO=/root/.ccr/ca-bundle.crt cargo install typst-cli --locked
```

Compile from the guide's directory with the **repo root** as the sandbox root so
the template import and shared schematic image resolve (both live outside the
project dir). Use the **canonical flags** — the `build-guide-check.yml` CI guard
recompiles every guide and fails if the committed PDF differs byte-for-byte, so
the PDF must be produced deterministically:

```bash
cd packages/<domain>/<project>/docs
typst compile --creation-timestamp 0 --ignore-system-fonts --root ../../../.. build-guide.typ
```

- `--creation-timestamp 0` pins the embedded PDF timestamp; without it the file
  carries wall-clock time and never reproduces.
- `--ignore-system-fonts` embeds only Typst's bundled fonts, so glyph fallbacks
  are identical on every machine (otherwise Linux FreeSans vs macOS SF diverge).
- Match the Typst version the guard pins (see `TYPST_VERSION` in
  `.github/workflows/build-guide-check.yml`); a different compiler release
  produces different bytes. Install it with
  `cargo install typst-cli --version <that-version> --locked`.

Verify by rendering a page or two to PNG (`--pages 1,3`) and eyeballing the
layout before committing.

Commit **both** `build-guide.typ` and `build-guide.pdf` — the PDF is the
deliverable and lets people use the guide without installing Typst.

## Wiring up the README

Add a short "Printable build guide" section to the project `README.md` linking
`docs/build-guide.typ` and `docs/build-guide.pdf`, plus the regenerate command.

## Style rules

- **Authoritative pins** — every pin, address, channel, and offset comes from
  source (`pin_config.h`, `partitions.csv`, `justfile`). If source and prose
  disagree, trust source and flag it.
- **Fill, don't pad** — omit sections the project doesn't use. A tight 6-page
  guide beats a padded 12-page one.
- **One style, one file** — never copy the template's styling into the project
  guide; import it. Improvements go in `tools/typst/build-guide.typ`.
- **Callouts for hazards** — voltage mismatches, common-ground, PSRAM mode,
  download-mode go in `warn`/`danger` callouts, not buried in prose.
- **Keep meta honest** — read `version.txt`, `sdkconfig.defaults`, and the
  justfile for the meta box; don't invent toolchain versions.
