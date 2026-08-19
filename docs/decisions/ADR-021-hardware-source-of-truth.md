# ADR-021: One Hardware Source of Truth — a Board × Header × Parts Join

**Status**: accepted
**Date**: 2026-08-19
**Source**: conversation 2026-08-18/19
**Confidence**: 7/10

---

## Context

A pin assignment in this repo is written down many times. For `robocar-unified`,
the same facts appear in **seven hand-maintained places** on top of the C header
that defines them:

| Consumer | How it gets pins today |
|---|---|
| `main/*.c` firmware | `main/pin_config.h` — canonical |
| `docs/auto/pin_defs.typ` | generated, drift-guarded (#377) |
| `docs/build-guide.typ` → `.pdf` | interpolates `#I2S_BCLK_PIN` etc. |
| `docs/schematics/circuits/robocar_unified.py` | hand-transcribed |
| `WIRING.md` master GPIO table (11 rows) | hand |
| `WIRING.md` per-peripheral tables (amp, ultrasonic) | hand |
| `WIRING.md` Mermaid diagrams (×3) | hand |
| `README.md` component table | hand |

Three of those live inside `WIRING.md` alone. Worse, the **netlist is drawn
twice by two different tools** — Mermaid in `WIRING.md` and schemdraw in
`docs/schematics/circuits/` — so the two drawings can disagree with each other
and nothing compares them.

Two existing drift guards cover the generated half and work well:
`schematics-check.yml` fails on a stale `images/*.svg`, and
`build-guide-check.yml` fails on a stale `build-guide.pdf`. Neither can help
with the hand-maintained half, and one has a hole: `schematics-check.yml` is
path-filtered to `docs/schematics/**`, so a firmware-only pin change **does not
run it at all** — and if it did, the check would pass, because the SVG still
matches its own now-wrong source.

The reason nobody has closed the gap by generating more is that the header
cannot express what the documents need. `pin_config.h` names a *role*
(`I2S_BCLK_PIN`), never the *part* the role lands on. Knowing that the I2S trio
goes to a MAX98357A, or that `D4` is the silkscreen name for `GPIO5`, requires
information that lives elsewhere.

## Decision

Treat the pin table as what it actually is — a **three-way join** — and generate
every restatement from it.

| Layer | Lives in | Scope |
|---|---|---|
| **Board facts** — `D4 ↔ GPIO5`, alternate functions, strapping pins | `docs/reference/boards/<board>.md` | per board, shared across projects |
| **Project roles** — which GPIO plays which role here | `<project>/main/pin_config.h` | per project |
| **Parts & nets** — which physical component a role lands on | `<project>/hardware.toml` (**new**) | per project |

Every `WIRING.md` pin table in this repo is that join, computed by hand.

`pin_config.h` **stays canonical and hand-written**. `hardware.toml` is a
sidecar that describes the hardware around it; it deliberately contains **no pin
numbers**, only role names that must resolve against the header:

```toml
[source]
header = "main/pin_config.h"                              # role → GPIO
board   = "docs/reference/boards/xiao-esp32s3-sense.md"   # D-number ↔ GPIO

[parts.amp]
name = "MAX98357A"
kind = "i2s-amp"

[[nets]]
role = "I2S_BCLK_PIN"
to   = "amp.BCLK"
note = "bit clock"

# A pin with no drawn net must be excused explicitly. This is what makes the
# completeness gate meaningful rather than decorative.
undrawn = [
  { role = "MIC_PDM_CLK_PIN", why = "internal to the Sense module, not brought out to a pin header" },
]
```

A new `tools/hardware/` library performs the join and feeds every consumer:

```
docs/reference/boards/<board>.md ─┐
<project>/main/pin_config.h      ─┼─> tools/hardware/  (parse + join)
<project>/hardware.toml          ─┘        │
                                           ├─> docs/auto/pin_defs.typ   [committed, guarded]
                                           ├─> WIRING.md  pin table     ] marker-block
                                           ├─> WIRING.md  Mermaid ×3    ] injection,
                                           ├─> README.md  components    ] --check in CI
                                           └─> circuits/*.py            [imported at render time]
                                                    └─> images/*.svg    [committed, guarded]
```

Two delivery mechanisms, chosen per consumer:

- **Marker-block injection** for Markdown — `<!-- BEGIN GENERATED: pin-table -->`
  … `<!-- END GENERATED -->`, with a `--check` mode for CI. Only tables and
  diagrams are injected; `WIRING.md`'s prose is load-bearing and stays
  hand-written — the power section, the "this replaces the microSD slot"
  trade-off callout, the flashing instructions.
- **Live import** for the schematic — `render.py` already loads circuit modules
  dynamically, so a circuit calls the join at import time. Nothing extra is
  committed on that path; the SVG is the committed artifact and is already
  guarded.

## What stays hand-authored, on purpose

- **Schematic layout.** `circuits/*.py` placement is human judgment about a
  drawing someone wires from — *"shifted down 8 units so PCA's right side is
  unobstructed"*, *"offset rather than directly beneath it so the ch2 bus gets
  its own vertical corridor"*. `Router` routes wires; it does not place
  components. An autoplacer would be a larger project than the schematics
  directory and would produce a worse drawing. Only the **pin labels and net
  endpoints** are generated.
- **Mermaid is different, and that is why it is generated.** Mermaid performs
  its own layout, so emitting it from the netlist loses nothing. It exists in
  `WIRING.md` because GitHub renders it inline, which the schemdraw SVG does
  not replace.
- **ADRs.** ADR-013/019/020 cite GPIO numbers as historical record. An ADR
  states what was decided at a date; auto-updating one would be falsification.
- **`CLAUDE.md` prose.** Judgment about what the pins *mean*, not a table.

## Why the header stays canonical rather than generated

The rejected alternative was making `hardware.toml` canonical and generating
`pin_config.h` from it, which would make firmware and docs incapable of
disagreeing. It was not chosen because it puts a generated file in the
compile path, inverts the "the code is the truth" instinct for anyone reading
firmware, and requires its own drift guard on the generated header. The sidecar
gets most of the guarantee — a role that does not resolve to a real macro fails
a test — at none of that cost.

## Consequences

**Gained**

- A pin change in `pin_config.h` propagates to the WIRING table, both Mermaid
  diagrams, the README table, the build guide, and the schematic in one
  `just hardware::gen`.
- The netlist has one definition. The two drawings cannot disagree.
- A **completeness gate** becomes possible for the first time: a pin macro that
  is neither wired in a net nor listed in `undrawn` fails. Adding
  `#define LIDAR_PWM_PIN GPIO_NUM_12` today changes no document and no
  drawing, silently and permanently; nothing in the repo catches that class.

**Costs**

- One new file per project, and `hardware.toml` can itself go stale against
  reality — it is guarded by role resolution against the header, not by physics.
- Marker blocks have to be placed in `WIRING.md`/`README.md` by hand once.
- The three projects that carry schematics use three different pin conventions
  (`robocar` `I2S_BCLK_PIN GPIO_NUM_7` in `main/pin_config.h`; `balancebot`
  `PIN_IMU_SDA 6` in `src/pin_config.h`; `gamepad-synth` the robocar shape but
  **in `main.c`**, with no `pin_config.h` at all). The parser needs a per-project
  descriptor regardless.
- `build-guide.typ` currently hardcodes `GPIO43`/`GPIO44` because the USB-serial
  pins exist only in a doc comment. They must be promoted to real macros or the
  generated table keeps a hand-written hole.

**Explicitly out of scope**

- The ESPHome projects (`audiobook-player`, `presence-detector`, `wireguard-ha`),
  where the YAML *is* the firmware and the join does not apply the same way.
- The other 12 `WIRING.md` files, most of which carry light or no pin tables.
  `robocar-unified` proves the chain end-to-end because it has every consumer.

## Alternatives considered

- **Header stays the only source; derive what it can express.** Cheapest, and
  it does fix pin numbers. Rejected because it cannot drive the parts columns or
  the netlist, so the Mermaid/schemdraw duplication survives.
- **Autogenerate the schematic outright.** Rejected: see *What stays
  hand-authored*.
- **A test that asserts circuit pin labels match the header, with no
  generation.** Considered as the cheap standalone option, and correct on its
  own — but it becomes a **tautology** once labels are generated from the header,
  since it would assert something true by construction. The assertion worth
  keeping is completeness, not agreement.
- **A rule in `.claude/rules/` telling the agent to update the assets.** Rejected
  as the primary mechanism: mechanical, repeatable checks belong in a
  deterministic gate, not in an instruction the agent has to remember. A much
  smaller rule survives, covering only what no test can hold (see below).

## Implementation notes

Phased, tracked in the epic. Ordering matters — the parser is the shared
foundation, and the highest-value gate lands before the invasive refactor:

1. `tools/hardware/` parser + join; re-point `generate-pin-defs.py` onto it.
   `pin_defs.typ` must come out **byte-identical**, which the existing
   build-guide drift guard verifies for free — this phase self-tests.
2. Completeness + role-resolution tests, and widen `schematics-check.yml` paths
   to `packages/**/{main,src}/pin_config.h`, `packages/**/hardware.toml`, and
   `**/WIRING.md`. Without this the suite does not run when only firmware changes.
3. Marker-block injection into `WIRING.md` and `README.md`; promote the
   USB-serial pins to real macros.
4. Schematic consumes the join — factories take the pin map, circuits index by
   role. Optional: its payoff is free pin renames and `D4/GPIO5` dual labels,
   not correctness, which phase 2 already secures.

A small `.claude/rules/` entry still earns its place for the residue no gate can
hold: schematic layout is hand-authored on purpose, and `test_routing.py`
hardcodes coordinates (`.at((5.75, …))`, `esp.center.x + 4.5`) against symbol
sizes declared in `components.py` (`size=(3, 5)`) — so a symbol resize can leave
`test_wire_avoids_obstacle_placed_between_pins` passing while asserting nothing.

## Related

- [ADR-013](ADR-013-single-board-xiao-esp32s3.md) — the XIAO consolidation whose
  pin budget this documents
- #377 — the Typst build-guide drift guard; the working precedent this generalizes
- #439 — a generated artifact going stale without tripping its own guard
- `docs/schematics/README.md` — router behaviour, SVG-not-PNG diff rationale
- `.claude/rules/esp-idf-sdkconfig.md` — the sibling "generated file goes stale
  silently" hazard in this repo
