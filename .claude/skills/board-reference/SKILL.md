---
name: board-reference
description: Research and write the reference pages for a board, module or breakout that has none yet — docs/reference/boards/<slug>.md plus a datasheets/ summary for its silicon. Use when a board or part is named (bought, being considered, being wired) and no page exists for it.
argument-hint: <board name, listing URL, or both>
allowed-tools: Read, Write, Edit, Grep, Glob, Bash, WebFetch, WebSearch
---

# Board Reference Pages

Every board or module the repo uses or is considering gets two documents:

| Page | Holds | Read by |
|---|---|---|
| `docs/reference/boards/<slug>.md` | This specific board: identification, parts on it, pin mapping, power, flashing, quirks, what is still unverified | People, and `tools/hardware` (ADR-021) via the pin-mapping table |
| `docs/reference/datasheets/<category>--<part>.md` | The silicon or module the board carries, every value cited to a datasheet section or table | People; linked from the board page |

Run this when a board is mentioned and step 1 finds no page. Don't wait to be
asked: a pin taken from memory today becomes wrong wiring later.

## 1. Check what exists

```sh
ls docs/reference/boards/ docs/reference/datasheets/
rg -il '<model>|<chip>|<alias>' docs/reference/
```

Search for every name the part goes by: listing titles often use a different
name than the board's silkscreen (#662's "HC-SR04P" was an RCWL-1601). If a
board page exists, update it instead. If only the silicon page exists, write
the board page and link to it.

## 2. Identify the exact product

Pin out the specific board, not the product family. The user's chip name can
be wrong. Before going further, confirm it against the silkscreen, a listing
photo or `esptool.py chip_id`.

- **A listing URL:** fetch the gallery photos from the listing itself, using
  the commands in `.claude/rules/board-layout-from-vendor-files.md` § "When it
  does not apply". Read every photo. One listing title often covers several
  different boards; note which photo shows the board in question and list the
  others so nobody confuses them.
- **No listing:** ask the user for the listing link or photos of both faces.
  Meanwhile, search community device databases (Tasmota templates at
  templates.blakadder.com, devices.esphome.io).

## 3. Gather facts, strongest source first

1. **Vendor board file** (Eagle `.brd`/`.sch` in the vendor's GitHub repo) for
   header order: `tools/breakout-pinout.py`, per the board-layout rule.
2. **Vendor schematic or wiki.**
3. **The silicon datasheet PDF.** Download it to `/tmp`, run
   `pdftotext -layout`, and cite each value by section or table
   (`Table 3-4`, `§3.1.3`). Link the PDF in the page; don't commit it.
4. **Listing photos:** parts markings, header count, dimension arrows.
5. **Community templates and forum threads:** GPIO assignments that others
   confirmed working, and known problems.

Never fill a gap from memory. A value with no source goes in the checklist,
not in a table. When sources disagree (a 5 V listing against a 3.3 V one),
record both and say which the hardware supports, for example an AMS1117-3.3 on
the board implies roughly 5 V in.

## 4. Write the board page

Model it on the closest existing page: `esp32-c3-super-mini.md` for an MCU dev
board, `rcwl-1601.md` or `cjmcu-2317.md` for a module identified from
listing photos, `sparkfun-tb6612fng.md` for a breakout with a vendor board
file. Include only the sections that apply, in this order:

1. `# <Name>`, then one paragraph: what it is, its role here, and a link to the
   datasheet page.
2. **The fitted board:** silkscreen, listing title and item number, which
   photo, and any similar boards it must not be confused with.
3. **Specifications** table, with a `Source` column when the facts come from
   mixed sources.
4. **Pin mapping:** exactly one table whose first header contains `Pin` and
   which has a column headed exactly `GPIO` (`tools/hardware/board.py`). GPIO
   cells hold `GPIOn` or `—`. Put the word `strapping` in the function cells of
   strapping pins. Add `Side`/`Pos` (`tools/hardware/layout.py` conventions)
   only when the order comes from a vendor file or the fitted board. A breakout
   with no GPIOs uses `| Pin | Side | Pos | … |` with no GPIO column. A second
   table with a pin-first header and a `GPIO` column makes the parser fail;
   tables keyed by GPIO (strapping lists, etc.) must have `GPIO` as their
   first column.
5. **Power**, **Flashing**, and a config snippet (ESPHome YAML or ESP-IDF
   `sdkconfig.defaults`) when they differ from the obvious.
6. **Known issues and quirks.**
7. **In this repository:** which package uses it, or "not yet used".
8. **Source:** where each class of fact came from, and in bold what has
   **not** been checked on the fitted board.
9. **Check on the physical board:** a `- [ ]` list of every open claim, each
   one something a person with the board can settle.
10. **References**, as links.

Slug: lowercase, hyphenated, without the vendor prefix unless it's needed to
tell boards apart (`esp-ir-tr.md`, `xiao-esp32c6.md`).

## 5. Write the silicon page, if missing

`docs/reference/datasheets/<category>--<part>.md`, using the prefixes in that
directory's `README.md`. Model it on `mcu--esp8285.md`: **Used in:**, a Key
Specs table with a `Source` column, pin notes (strapping, reserved pins), and
links to the datasheets. Add it to the index in
`docs/reference/datasheets/README.md`.

## 6. Verify

```sh
python3 -c "from pathlib import Path; from tools.hardware.board import parse_board_table as p; b=p(Path('docs/reference/boards/<slug>.md')); print([(x.name, x.gpio, x.strapping) for x in b.pins])"
just hardware::test
```

The parse must list every row with the GPIO you meant. A breakout page without
a GPIO column should fail with "no pin-mapping table". That failure confirms
the parser is looking at the page. Check that each relative link resolves.

Don't add a `board =` key to any `hardware.toml` until the page's checklist
items affecting pins are ticked (#779 convention).
