---
name: part-substitution
description: Replace a carrier part with another one (a sister part, another maker's equivalent, a newer revision) by comparing both datasheets on the numbers the circuit depends on, checking the footprint and pin order, and recording the change. Use when a part cannot be bought, is out of stock at LCSC, is NRND or obsolete, or a cheaper or better part is proposed.
argument-hint: <original part> [candidate part]
allowed-tools: Read, Write, Edit, Grep, Glob, Bash, WebFetch, WebSearch
---

# Substituting a Part

A substitute is a different part. It is accepted only after its datasheet has
been read against the original's on every number the circuit depends on, and
its package and pin order have been checked against the registry entry the
board was built from. A part number that looks like a sibling proves neither.

Where this skill sits:

- `.claude/rules/part-sourcing.md` decides that a part needs a source; the
  `aliexpress-parts` skill's sourcing check finds that one is missing.
- This skill decides whether another part can stand in, and records it.
- `.claude/rules/power-stage-checklist.md` applies on top for anything on a
  supply path.

## 1. List what the circuit depends on

Start from the design, not the datasheet. The `hardware.toml` comment above
the part and the project README's design notes record why the part was chosen;
the balancebot's Q1 comment names VDSS, VGSS, RDS(on) at two gate voltages,
the threshold and the dissipation at the fuse's hold current. Add:

- **Absolute maximums against the worst case the design names**: a reversed
  pack, a hot-plug overshoot, the clamp voltage that protects a gate.
- **The operating point**: on-resistance at the gate drive the circuit really
  has (at the lowest supply), not the datasheet's headline −10 V figure;
  a regulator's current limit at its minimum, not typical.
- **Derived figures**: dissipation (I²R at the design current), temperature
  rise from the thermal resistance, timing that depends on capacitance.

## 2. Read both datasheets

Use the manufacturer's PDF and record its document number and revision. Never
quote a value from memory (`analog-fault-isolation.md`).

- WebFetch returns a PDF as raw bytes and saves them to a file; the harness
  note names it. Turn it into text and page images:

  ```sh
  .claude/skills/part-substitution/datasheet.sh <saved.pdf> tmp/datasheets 1
  ```

  It writes `tmp/datasheets/<stem>.txt` and page PNGs and prints the document
  number and the ratings lines. Read the tables in the text before quoting,
  and view the PNG for the pin-out: it is a drawing, and `pdftotext` drops its
  labels (only the equivalent circuit's G/D/S letters come through).
- A manufacturer's part pages may refuse automated fetches (Diodes returned
  403) while its PDFs download fine.

## 3. Compare in a table

One row per number from step 1, with the original, the candidate, and the
document and page for each. Mark each row better, equal, worse but inside the
design's margin, or fails. One failing row rejects the candidate, whatever the
other rows say.

## 4. Check the package and pin order

- The same package name does not mean the same pin order. Compare the
  candidate's pin-out drawing pin by pin with the registry entry's `[pins]`
  and footprint (`docs/reference/parts/<slug>.toml`), tab included.
- Check package variants: Diodes calls its SOT-223 "Type DN" on some sheets;
  read the package outline where the name differs.
- A polarised part (diode, electrolytic, FET) is checked by the drawing, not
  by the name.

## 5. Decide the scope

- **Hand-build only**: the design keeps the original, and the substitute is
  fitted to boards built by hand. Write it into the carrier README's assembly
  notes with the comparison figures, and into the quote CSV. The original's
  registry entry gets no `buy` for the substitute: they are different parts.
- **Design change**: the substitute replaces the original for every build.
  Add a registry entry (or change the instance's `part`, `value`, `lcsc` and
  `buy`), then regenerate and check the carrier (the `carrier-pcb` skill).
  `carrier-check` needs KiCad 10.0.6 to rewrite the stamp, and
  `just kicad::sourcing` should list one fewer JLCPCB-only part.

## 6. Check it on arrival

A seller with few sales can ship a remarked part. Read the marking under a
loupe and measure what can be measured before the part goes on a board: a
FET's body diode and gate leakage with the meter's diode range, a PTC's cold
resistance, a capacitor's value on the LCR tester.

## Worked example: balancebot Q1 (2026-10-30)

The ZXMP4A16G reverse-polarity P-FET is not sold on AliExpress.

| Number the design depends on | ZXMP4A16G (DS33584 Rev. 6) | ZXMP3A16G (DS33575 Rev. 6) | ZXMP6A17G (DS33375 Rev. 9) |
|---|---|---|---|
| VDSS | −40 V | −30 V: above a reversed 12.6 V pack and the board's estimated overshoot | −60 V |
| VGSS | ±20 V | ±20 V | ±20 V |
| RDS(on) max at −10 V | 60 mΩ | 45 mΩ | 125 mΩ |
| RDS(on) max at −4.5 V | 100 mΩ | 70 mΩ | 190 mΩ |
| VGS(th) min | −1.0 V | −1.0 V | −1.0 V |
| Ciss (R4 timing) | 1007 pF | 1022 pF | 637 pF |
| Dissipation at F1's 3 A hold, −10 V | 0.54 W | 0.41 W | 1.13 W: fails |
| SOT-223 pin-out | G, D, S, tab D | G, D, S, tab D (page 1 drawing) | read the drawing first |

The ZXMP3A16G was accepted for hand-built boards. The ZXMP6A17G, the cheaper
listing, was rejected on dissipation, over twice the original's at the fuse's
hold current. The F1 PTC had no substitute with a datasheet: every AliExpress
1812 "3 A 24 V" PTC states no hold or trip current, so it stays with JLCPCB
or LCSC, or one is measured before fitting.

## Related

- `.claude/rules/part-sourcing.md`, `aliexpress-parts` skill (sourcing check)
- `.claude/rules/power-stage-checklist.md`, `.claude/rules/analog-fault-isolation.md`
- `carrier-pcb` skill: registry entries, regenerating the carrier
