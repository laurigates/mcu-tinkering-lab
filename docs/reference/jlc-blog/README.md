# JLC Blogs: What Applies to This Repo

The JLCPCB blog (jlcpcb.com/blog) and the JLC3DP blog (jlc3dp.com/blog) were
read on 2026-10-27 to find content that could improve this repo's skills, tools
and reviews. Each domain file ranks the useful posts and lists proposed changes.
Every proposal names its source URL and the repo file it would change, and is
marked new, already covered, or conflicting.

| File | Domain | Posts read in full | Proposals |
|------|--------|-------------------:|-----------|
| [fab-dfm-layout.md](fab-dfm-layout.md) | Bare-board fab, DFM, silkscreen, mask, vias, panels (`carrier-pcb`, `tools/kicad`, `pcb-reviewer`) | 52 | A–I |
| [pcba-sourcing.md](pcba-sourcing.md) | PCBA, BOM/CPL, fees, stencils, paste, hand assembly (`jlcpcb-order`, `jlc.py`, `fab.py`, `aliexpress-parts`) | 64 | A–E |
| [circuits-power-rf.md](circuits-power-rf.md) | Current vs. copper, strapping pins, antenna/RF, decoupling, ESP32 pages (`tools/hardware`, `pcb-reviewer`, `board-reference`) | 58 | P1–P12 |
| [3dp-enclosure-mechanism.md](3dp-enclosure-mechanism.md) | JLC3DP process rules, clearances, heat, inserts (`enclosure-cad`, `printed-mechanism`, `tools/mechanism`) | 54 | A–D |

## Fetching

[`tools/jlc-blog.py`](../../../tools/jlc-blog.py) reads the CMS JSON that
serves both blogs. The JSON needs no login. The endpoints are listed in the
script's docstring.

```sh
tools/jlc-blog.py index --site pcb -o jlc-blog-index.jsonl     # 1101 posts (sitemap lists 995)
tools/jlc-blog.py index --site 3dp -o jlc3dp-index.jsonl        # 419 posts
tools/jlc-blog.py search 'silkscreen|solder mask' --index jlc-blog-index.jsonl
tools/jlc-blog.py fetch dfm-analysis-in-pcb-design --out jlc-blog/   # Markdown, cached
```

The script uses an undocumented API, and `robots.txt` disallows `/api`. The
script therefore sleeps 0.5 s between requests and caches each post it fetches.
Do not run it as a bulk crawler.

## How far to trust it

Treat the blog like the **uncited** label in
[servo-selection.md](../servo-selection.md): about 55–65 % of the posts read
were generic or marketing, and some contradict each other. Each domain file
lists these conflicts in its noise section and does not resolve them. Two
kinds of post are reliable, because JLC states facts about its own process in
them:

- the "Technical Guidance" posts, such as `character-design-specifications`;
- the dated "Product Update" posts, such as `jlcpcb-pcb-osp-finish` and
  `reduce-pcb-assembly-cost-prototype`.

Where a post disagrees with the
[capabilities page](https://jlcpcb.com/capabilities/pcb-capabilities), the
capabilities page wins.

The flagged post `dfm-analysis-in-pcb-design` is an overview with no numbers.
The usable DFM rules are in the posts ranked above it in `fab-dfm-layout.md`.

## Priority backlog

These are ordered by risk to an order and by how cheaply each can be checked.
The first group would change a board or an order that is already committed.

1. **Silkscreen text below JLC's floor.** The capabilities page treats
   characters shorter than 1.0 mm or with strokes thinner than 0.15 mm as
   unidentifiable. `character-design-specifications` allows 0.8 mm only with
   the paid high-precision option. `boardcheck.SILK_MIN_MM = 0.8`, ADR-027,
   the carrier-pcb trap and `pcb-reviewer` all use 0.8. The integrated carrier
   uses 0.8 mm text. *(fab A)*
   **Done:** `boardcheck` now requires 1.0 mm; 0.8 mm needs
   `[silk] high_precision = true`, which both robocar carriers set and
   `carrier-fab` turns into an `ORDER:` line.
2. **Silkscreen near pads is clipped without notice.** JLC removes silkscreen
   within 0.15 mm of a pad. The carriers set `min_silk_clearance 0`. The fix
   is a `.kicad_dru` rule and a measurement in `review.py`. *(fab A)*
   **Done:** each carrier's `.kicad_dru` rule "silkscreen 0.15 mm clear of
   pads" moves KiCad's `silk_over_copper` limit 0.15 mm out (measured).
   It found 43 cases on the robocar boards. `build_board.clip_silk` now
   cuts footprint lines back from pads. The crowded labels were moved or
   set to 0.8 mm in `board.toml`, and `carrier-silk` applied both to the
   routed boards with the copper unchanged (`carrier-pcb` traps 47–49).
3. **Surface finish.**
   - Since 2026-07-30, OSP is free on 2-layer boards and ENIG is no longer
     free.
   - HASL bridges at pitch ≤ 0.5 mm, and the MAX98357A TQFN is 0.5 mm pitch.
   - OSP lasts about 1 week in open air and must not get hot air.
   - `jlcpcb-order` should pick the finish from the finest pitch and from who
     solders the board. *(fab B, pcba A)*
4. **Economic PCBA eligibility.** Any of these forces Standard PCBA: parts on
   both sides, anything below 0402, pitch under 0.4 mm, more than 50 boards,
   V-cut, castellated holes or edge plating. A footprint check in
   `tools/kicad/jlc.py` would catch most of these. *(pcba A/B)*
5. **Confirm before production.** Add the $0.45 "Confirm Parts Placement" and
   "Confirm Production Files" options, and a JLCDFM run, to the
   `jlcpcb-order` steps. JLC's own CAM edits silkscreen, mask dams and
   mouse-bite webs. *(fab E, pcba A)*
6. **Rings and tented vias.**
   - The XT30PW-M footprint's 0.20 mm annular ring is below the recommended
     0.25 mm.
   - Tenting is reliable only up to a 0.4–0.5 mm drill. The power class's
     0.5 mm via drill is exactly at that limit, and nothing checks it.
     *(fab C)*
7. **Solder mask.** The capabilities page asks for ≥ 0.09 mm from a mask
   opening to the next trace. The carriers set
   `solder_mask_to_copper_clearance 0`. *(fab D)*
8. **Fiducial distance from the edge.** The integrated carrier's fiducials
   sit 3.5 mm in. The posts give 3.35 mm and 3.85 mm. Settle this from JLC's
   help page before a Standard PCBA order. *(fab F, pcba C)*

The second group is tool and gate work for future designs:

9. **Running clearance in the clash sweep.** `CLASH_TOL_MM3` fails only on
   overlapping volume. JLC gives 0.5 mm (SLA), 0.6 mm (MJF/SLS) and 1.0 mm
   (SLM) for moving parts, so the sweep should also report near-misses below
   these figures. *(3dp A)*
10. **Heat.** Resin softens at about 50–65 °C. A part next to a regulator,
    motor or stepper heatsink should be MJF nylon. Add this as a trap in
    `enclosure-cad` and `printed-mechanism`. *(3dp B/C)*
11. **MJF accuracy.**
    - Tolerance is ±0.2 mm or ±0.2 %; the skill says ±0.3.
    - Holes under 5 mm print 0.2–0.3 mm undersize.
    - The minimum powder-escape hole is 2.5 mm, and no check looks for
      sealed voids. *(3dp A/B)*
12. **Current against copper.** Add `current_a` to the net classes. Then
    `review.py` can check track width against IPC-2221 and count vias at one
    per amp. *(circuits P1)*
13. **Strapping pins in the join.** `tools/hardware/board.py` already reads a
    `strapping` flag, but nothing uses it. `join()` should reject a pull-up or
    a bus on a strap pin unless the entry explains why. *(circuits P2)*
14. **Antenna keep-out.** Add a carrier-pcb trap and a `pcb-reviewer` item.
    No current carrier is affected: they use U.FL antennas or the RP2350. A
    XIAO ESP32-C3 or C6 with a PCB antenna would be. *(fab H, circuits P3)*
15. **Decoupling vias and the ESP32 reference pages.** These are review
    items and documentation. *(circuits P4–P6)*

Topics the blogs had nothing usable on:
- I2C pull-ups and bus capacitance;
- level shifting;
- motor-driver flyback layout;
- 1S LiPo charging;
- NFC antenna matching.
